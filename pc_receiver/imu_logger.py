"""BLE로 IMU와 PDM 마이크를 동시에 기록해 두 개의 CSV로 저장한다.

사용법: python imu_logger.py [기록 초]  (기본 3초, 마이크 최대 5초)
"""

import asyncio
import csv
import datetime
import struct
import sys

from bleak import BleakClient, BleakScanner


NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> 보드
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 -> PC
MIC_PACKET_MAGIC = 0xA5


class SensorStreamParser:
    def __init__(self, done_event: asyncio.Event):
        self.done_event = done_event
        self.buffer = bytearray()
        self.binary_mode = False
        self.imu_rows = []
        self.mic_samples = []
        self.mic_sample_rate = None
        self.mic_expected = None
        self.mic_overflow = False

    def feed(self, data: bytes):
        self.buffer.extend(data)

        while True:
            if self.binary_mode:
                if len(self.buffer) < 2:
                    return

                if self.buffer[0] != MIC_PACKET_MAGIC:
                    try:
                        next_magic = self.buffer.index(MIC_PACKET_MAGIC, 1)
                    except ValueError:
                        self.buffer.clear()
                        return
                    del self.buffer[:next_magic]

                count = self.buffer[1]
                packet_size = 2 + count * 2
                if len(self.buffer) < packet_size:
                    return

                del self.buffer[:2]
                if count == 0:
                    self.binary_mode = False
                    continue

                pcm_bytes = bytes(self.buffer[: count * 2])
                del self.buffer[: count * 2]
                self.mic_samples.extend(struct.unpack(f"<{count}h", pcm_bytes))
                continue

            newline = self.buffer.find(b"\n")
            if newline < 0:
                return

            raw_line = bytes(self.buffer[:newline])
            del self.buffer[: newline + 1]
            self._handle_text_line(raw_line.decode("ascii", errors="replace").strip())

    def _handle_text_line(self, line: str):
        if not line:
            return
        if line == "END":
            self.done_event.set()
            return
        if line.startswith("MIC_BINARY,"):
            fields = line.split(",")
            if len(fields) >= 3:
                self.mic_sample_rate = int(fields[1])
                self.mic_expected = int(fields[2])
                self.mic_overflow = len(fields) >= 4 and fields[3] == "1"
                self.binary_mode = True
            return

        fields = line.split(",")
        if len(fields) == 7:
            try:
                self.imu_rows.append([int(value) for value in fields])
            except ValueError:
                pass


async def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    if seconds <= 0:
        raise SystemExit("기록 시간은 0보다 커야 합니다.")

    done = asyncio.Event()
    parser = SensorStreamParser(done)

    def on_notify(_, data: bytearray):
        parser.feed(bytes(data))

    print("보드(Nordic_UART) 검색 중...")
    device = await BleakScanner.find_device_by_name("Nordic_UART", timeout=10.0)
    if device is None:
        print("보드를 찾지 못했습니다. 전원과 BLE 광고 상태를 확인하세요.")
        return

    async with BleakClient(device) as client:
        print("연결:", device.address)
        await client.start_notify(NUS_TX, on_notify)

        print(f"[s] IMU/MIC 동시 기록 시작 ({seconds:g}초)")
        await client.write_gatt_char(NUS_RX, b"s", response=False)
        await asyncio.sleep(seconds)

        print("[e] 기록 정지, 데이터 수신 중...")
        await client.write_gatt_char(NUS_RX, b"e", response=False)
        try:
            await asyncio.wait_for(done.wait(), timeout=120.0)
        except asyncio.TimeoutError:
            print("수신 제한 시간 초과: 현재까지 받은 데이터만 저장합니다.")

        await client.stop_notify(NUS_TX)

    timestamp = datetime.datetime.now().strftime("%H%M%S")
    imu_filename = f"imu_{timestamp}.csv"
    mic_filename = f"mic_{timestamp}.csv"

    with open(imu_filename, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "ax", "ay", "az", "gx", "gy", "gz"])
        writer.writerows(parser.imu_rows)

    with open(mic_filename, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "pcm"])
        writer.writerows(enumerate(parser.mic_samples))

    print(f"IMU {len(parser.imu_rows)}개 -> {imu_filename}")
    print(
        f"MIC {len(parser.mic_samples)}개"
        f" ({parser.mic_sample_rate or 'unknown'} Hz) -> {mic_filename}"
    )
    if parser.mic_expected is not None and len(parser.mic_samples) != parser.mic_expected:
        print(f"경고: MIC 예상 {parser.mic_expected}개 중 {len(parser.mic_samples)}개 수신")
    if parser.mic_overflow:
        print("경고: 보드의 MIC 5초 버퍼가 가득 차 일부 샘플이 잘렸습니다.")


if __name__ == "__main__":
    asyncio.run(main())
