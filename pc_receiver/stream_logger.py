"""BLE 실시간 스트리밍 수신기 — 마이크는 WAV, IMU는 CSV로 저장한다.

사용법: python stream_logger.py [기록 초] [마이크 kHz] [IMU Hz] [프로파일]
        기본 5초 / 16kHz / 보드 기본값(416Hz) / 프로파일 0.
        마이크 kHz는 16, 8, 4, 2, 1 중 하나.
        IMU Hz는 104, 208, 416, 833, 1666, 3333, 6664 중 하나.
        프로파일은 0~4 (아래 표).

주의: PC(Windows)는 BLE 처리량이 105~155kbps라 마이크 16kHz만으로도 이미 모자란다.
      IMU를 올려 성능을 판정하려면 반드시 폰에서 할 것. PC에서는 마이크를 4kHz로
      낮춰서(`python stream_logger.py 10 4 3333`) IMU 쪽만 검증하는 용도로 쓴다.
      압축 프로파일도 마찬가지로, PC에서는 "디코딩이 맞는지"만 확인한다.

보드는 녹음 중 계속 패킷을 흘려보낸다. 패킷마다 '첫 샘플의 절대 번호'가
들어 있어서, 번호가 건너뛰면 그만큼 유실된 것으로 판단할 수 있다.

와이어 포맷 (펌웨어 main.c와 공유):

  기존 포맷 (프로파일 0에서 쓰는 것)
    [0]     매직: 0xA5=마이크 PCM16, 0x5A=IMU raw 12B, 0x01=텍스트/상태
    [1..4]  첫 샘플의 절대 번호 (uint32 little-endian)
    [5]     샘플 개수 (uint8)
    [6..]   마이크: int16 LE PCM / IMU: int16 LE x 6축

  압축 포맷 (프로파일 1 이상)
    [0]     매직: 0xA6=마이크 IMA ADPCM 4비트, 0x5B=IMU 12비트 팩
    [1..4]  첫 샘플의 절대 번호 (uint32 LE)
    [5..6]  샘플 개수 (uint16 LE)
    [7]     IMU: 포맷(0=6축, 1=가속도만) / 마이크: ADPCM step index
    [8..]   IMU: 12비트 값 연속 팩
            마이크: [8..9] predictor(int16 LE), [10..] 4비트 코드 (하위 니블 먼저)

프로파일 (0xA2 명령으로 선택). 마이크 16k + IMU 6664Hz 기준 필요 대역폭:
    0 RAW          929 kbps  기존 포맷 (무손실)
    5 BOTH_MIC8    약 660 kbps  마이크 8비트 u-law + IMU 12비트 (마이크 오차 10%->1.4%)
    1 MIC_ADPCM    732 kbps  마이크만 압축
    2 IMU_PACK     769 kbps  IMU만 12비트
    3 BOTH         572 kbps  <- 목표
    4 BOTH_ACCEL   372 kbps  자이로를 버린 안전판
"""

import asyncio
import csv
import datetime
import struct
import sys
import wave

from bleak import BleakClient, BleakScanner


NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> 보드
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 -> PC
NAIL_ADDRESS = "EA:25:47:A6:4C:2A"

CMD_SET_RATE = 0xA0
CMD_SET_IMU_RATE = 0xA1
CMD_SET_PROFILE = 0xA2

IMU_RATES = (104, 208, 416, 833, 1666, 3333, 6664)

PROFILE_NAMES = {
    0: "RAW (마이크 PCM16 + IMU 12B)",
    1: "MIC_ADPCM (마이크 4비트 + IMU 12B)",
    2: "IMU_PACK (마이크 PCM16 + IMU 12비트)",
    3: "BOTH (마이크 4비트 + IMU 12비트)",
    4: "BOTH_ACCEL (마이크 4비트 + 가속도만 12비트)",
    5: "BOTH_MIC8 (마이크 8비트 u-law + IMU 12비트)",
}

MAGIC_MIC = 0xA5
MAGIC_IMU = 0x5A
MAGIC_TEXT = 0x01
MAGIC_MIC_ADPCM = 0xA6
MAGIC_MIC_ULAW = 0xA7
MAGIC_IMU_PACK = 0x5B
HEADER_BYTES = 6
HEADER2_BYTES = 8
ADPCM_HDR_BYTES = 10

IMU_PACK_FMT_6AXIS = 0
IMU_PACK_FMT_ACCEL = 1

MIC_SAMPLE_RATE = 16000

# --- IMA ADPCM 디코더 (펌웨어 인코더와 짝) ---
IMA_STEP_TABLE = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
)
IMA_INDEX_TABLE = (-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8)


def adpcm_decode(payload: bytes, count: int, pred: int, index: int):
    """4비트 코드열 -> int16 샘플 리스트.

    predictor/step index를 패킷마다 받으므로 앞 패킷이 유실돼도
    이 패킷만으로 정확히 복원된다.
    """
    out = []
    for i in range(count):
        byte = payload[i >> 1]
        code = (byte & 0x0F) if (i & 1) == 0 else (byte >> 4)

        step = IMA_STEP_TABLE[index]
        delta = step >> 3
        if code & 4:
            delta += step
        if code & 2:
            delta += step >> 1
        if code & 1:
            delta += step >> 2

        pred = pred - delta if (code & 8) else pred + delta
        pred = max(-32768, min(32767, pred))

        index = max(0, min(88, index + IMA_INDEX_TABLE[code & 0x0F]))
        out.append(pred)
    return out, pred, index


def ulaw_decode(payload: bytes, count: int):
    """G.711 u-law 8비트 -> int16. 상태가 없어서 패킷 단위로 완결된다."""
    out = []
    for i in range(count):
        u = (~payload[i]) & 0xFF
        t = ((u & 0x0F) << 3) + 0x84
        t <<= (u & 0x70) >> 4
        out.append((0x84 - t) if (u & 0x80) else (t - 0x84))
    return out


def unpack12(payload: bytes, n: int):
    """12비트 연속 팩 -> 원래 스케일(<<4)의 int16 n개.

    펌웨어가 하위 4비트를 버리고 보냈으므로 되돌릴 때 다시 4비트 올린다.
    원시 LSB와 같은 축척이라 g/dps 변환 계수를 그대로 쓸 수 있다.
    """
    out = []
    pos = 0
    i = 0
    while i + 1 < n:
        b0, b1, b2 = payload[pos], payload[pos + 1], payload[pos + 2]
        pos += 3
        a = b0 | ((b1 & 0x0F) << 8)
        b = (b1 >> 4) | (b2 << 4)
        for v in (a, b):
            if v & 0x800:
                v -= 0x1000
            out.append(v << 4)
        i += 2
    if i < n:
        b0, b1 = payload[pos], payload[pos + 1]
        v = b0 | ((b1 & 0x0F) << 8)
        if v & 0x800:
            v -= 0x1000
        out.append(v << 4)
    return out


def pack12_bytes(n: int) -> int:
    return (n * 12 + 7) // 8


class StreamParser:
    """BLE 알림 하나 = 패킷 하나. (ATT가 경계를 보존하므로 재조립이 필요 없다)"""

    def __init__(self, done_event: asyncio.Event):
        self.done_event = done_event

        # 절대 번호 -> 값. 유실 구간은 비어 있게 두고 저장할 때 메운다.
        self.mic_chunks = []   # (start_index, [샘플...])
        self.imu_chunks = []   # (start_index, [(ax..gz), ...])

        self.mic_received = 0
        self.imu_received = 0
        self.mic_max_index = 0
        self.imu_max_index = 0

        self.status_lines = []
        self.bad_packets = 0
        self.mic_rate = MIC_SAMPLE_RATE   # RATE 줄이 오면 덮어쓴다
        self.imu_rate = None              # 보드가 RATE 줄로 알려준 실제 IMU ODR
        self.profile = None               # 보드가 RATE 줄로 알려준 프로파일

        # 무선 구간에서 실제로 흐른 바이트/패킷. 압축 효과를 보려면 디코딩된
        # 샘플 수가 아니라 이걸로 처리량을 계산해야 한다.
        self.wire_bytes = 0
        self.wire_packets = 0
        self.has_gyro = True              # 프로파일 4는 자이로가 없다

    def feed(self, data: bytes):
        self.wire_bytes += len(data)
        self.wire_packets += 1

        if len(data) < HEADER_BYTES:
            # 텍스트 패킷은 헤더 없이 짧을 수 있다
            if data and data[0] == MAGIC_TEXT:
                self._handle_text(data[1:])
            else:
                self.bad_packets += 1
            return

        magic = data[0]

        if magic == MAGIC_TEXT:
            self._handle_text(data[1:])
            return

        if magic in (MAGIC_MIC_ADPCM, MAGIC_MIC_ULAW, MAGIC_IMU_PACK):
            self._feed_packed(magic, data)
            return

        index, count = struct.unpack_from("<IB", data, 1)
        payload = data[HEADER_BYTES:]

        if magic == MAGIC_MIC:
            if len(payload) < count * 2:
                self.bad_packets += 1
                return
            samples = list(struct.unpack_from(f"<{count}h", payload, 0))
            self._add_mic(index, samples)

        elif magic == MAGIC_IMU:
            if len(payload) < count * 12:
                self.bad_packets += 1
                return
            rows = []
            for i in range(count):
                rows.append(struct.unpack_from("<6h", payload, i * 12))
            self._add_imu(index, rows)

        else:
            self.bad_packets += 1

    def _feed_packed(self, magic: int, data: bytes):
        """압축 포맷(0xA6 / 0x5B) 디코딩. 헤더가 8바이트이고 count가 uint16이다."""
        if len(data) < HEADER2_BYTES:
            self.bad_packets += 1
            return
        index, count, aux = struct.unpack_from("<IHB", data, 1)

        if magic == MAGIC_MIC_ULAW:
            if len(data) < HEADER2_BYTES + count:
                self.bad_packets += 1
                return
            self._add_mic(index, ulaw_decode(data[HEADER2_BYTES:], count))
            return

        if magic == MAGIC_MIC_ADPCM:
            if len(data) < ADPCM_HDR_BYTES + (count + 1) // 2:
                self.bad_packets += 1
                return
            pred = struct.unpack_from("<h", data, 8)[0]
            samples, _, _ = adpcm_decode(data[ADPCM_HDR_BYTES:], count, pred, aux)
            self._add_mic(index, samples)
            return

        # MAGIC_IMU_PACK
        per_sample = 3 if aux == IMU_PACK_FMT_ACCEL else 6
        n_values = count * per_sample
        if len(data) < HEADER2_BYTES + pack12_bytes(n_values):
            self.bad_packets += 1
            return
        values = unpack12(data[HEADER2_BYTES:], n_values)

        rows = []
        if per_sample == 3:
            self.has_gyro = False
            for i in range(count):
                ax, ay, az = values[i * 3: i * 3 + 3]
                rows.append((ax, ay, az, 0, 0, 0))
        else:
            for i in range(count):
                rows.append(tuple(values[i * 6: i * 6 + 6]))
        self._add_imu(index, rows)

    def _add_mic(self, index, samples):
        self.mic_chunks.append((index, samples))
        self.mic_received += len(samples)
        self.mic_max_index = max(self.mic_max_index, index + len(samples))

    def _add_imu(self, index, rows):
        self.imu_chunks.append((index, rows))
        self.imu_received += len(rows)
        self.imu_max_index = max(self.imu_max_index, index + len(rows))

    def _handle_text(self, raw: bytes):
        text = raw.decode("ascii", errors="replace").strip()
        if not text:
            return
        if text == "END":
            self.done_event.set()
            return
        if text.startswith("RATE,"):
            for field in text.split(",")[1:]:
                if field.startswith("mic_hz="):
                    self.mic_rate = int(field.split("=")[1])
                elif field.startswith("imu_hz="):
                    self.imu_rate = int(field.split("=")[1])
                elif field.startswith("profile="):
                    self.profile = int(field.split("=")[1])
        self.status_lines.append(text)


def build_mic_track(chunks, total_len):
    """유실 구간을 0(무음)으로 메워 시간축이 어긋나지 않게 한다."""
    track = [0] * total_len
    for start, samples in chunks:
        for offset, value in enumerate(samples):
            pos = start + offset
            if 0 <= pos < total_len:
                track[pos] = value
    return track


async def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    if seconds <= 0:
        raise SystemExit("기록 시간은 0보다 커야 합니다.")

    khz = int(sys.argv[2]) if len(sys.argv) > 2 else 16
    if khz not in (16, 8, 4, 2, 1):
        raise SystemExit("마이크 kHz는 16, 8, 4, 2, 1 중 하나여야 합니다.")

    imu_hz = int(sys.argv[3]) if len(sys.argv) > 3 else None
    if imu_hz is not None and imu_hz not in IMU_RATES:
        raise SystemExit(f"IMU Hz는 {', '.join(map(str, IMU_RATES))} 중 하나여야 합니다.")

    profile = int(sys.argv[4]) if len(sys.argv) > 4 else None
    if profile is not None and profile not in PROFILE_NAMES:
        raise SystemExit(f"프로파일은 {', '.join(map(str, PROFILE_NAMES))} 중 하나여야 합니다.")

    done = asyncio.Event()
    parser = StreamParser(done)

    def on_notify(_, data: bytearray):
        parser.feed(bytes(data))

    print(f"Nail({NAIL_ADDRESS}) 검색 중...")
    device = await BleakScanner.find_device_by_address(NAIL_ADDRESS, timeout=10.0)
    if device is None:
        print("보드를 찾지 못했습니다. 전원과 BLE 광고 상태를 확인하세요.")
        return

    async with BleakClient(device) as client:
        print("연결:", device.address)
        await client.start_notify(NUS_TX, on_notify)

        if khz != 16:
            print(f"[0xA0] 마이크 레이트 {khz}kHz로 변경")
            await client.write_gatt_char(NUS_RX, bytes([CMD_SET_RATE, khz]), response=False)
            await asyncio.sleep(0.3)

        if imu_hz is not None:
            print(f"[0xA1] IMU 레이트 {imu_hz}Hz로 변경")
            await client.write_gatt_char(
                NUS_RX,
                bytes([CMD_SET_IMU_RATE, imu_hz & 0xFF, (imu_hz >> 8) & 0xFF]),
                response=False,
            )
            await asyncio.sleep(0.3)

        if profile is not None:
            print(f"[0xA2] 프로파일 {profile}: {PROFILE_NAMES[profile]}")
            await client.write_gatt_char(
                NUS_RX, bytes([CMD_SET_PROFILE, profile]), response=False
            )
            await asyncio.sleep(0.3)

        print(f"[s] 실시간 스트리밍 시작 ({seconds:g}초)")
        await client.write_gatt_char(NUS_RX, b"s", response=False)
        await asyncio.sleep(seconds)

        print("[e] 정지, 잔량 수신 중...")
        await client.write_gatt_char(NUS_RX, b"e", response=False)
        try:
            await asyncio.wait_for(done.wait(), timeout=30.0)
        except asyncio.TimeoutError:
            print("수신 제한 시간 초과: 현재까지 받은 데이터만 저장합니다.")

        await client.stop_notify(NUS_TX)

    timestamp = datetime.datetime.now().strftime("%H%M%S")
    wav_name = f"mic_{timestamp}.wav"
    csv_name = f"imu_{timestamp}.csv"

    # ---- 마이크 -> WAV ----
    track = build_mic_track(parser.mic_chunks, parser.mic_max_index)
    with wave.open(wav_name, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(parser.mic_rate)
        wav.writeframes(struct.pack(f"<{len(track)}h", *track))

    # ---- IMU -> CSV ----
    imu_rows = []
    for start, rows in parser.imu_chunks:
        for offset, row in enumerate(rows):
            imu_rows.append((start + offset,) + row)
    imu_rows.sort()

    with open(csv_name, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        if parser.has_gyro:
            writer.writerow(["index", "ax", "ay", "az", "gx", "gy", "gz"])
            writer.writerows(imu_rows)
        else:
            # 프로파일 4는 자이로를 아예 안 보낸다. 0으로 채우면 "정지한 자이로"와
            # 구분이 안 되므로 열 자체를 빼서 없다는 사실이 드러나게 한다.
            writer.writerow(["index", "ax", "ay", "az"])
            writer.writerows(row[:4] for row in imu_rows)

    # ---- 결과 요약 ----
    mic_lost = parser.mic_max_index - parser.mic_received
    imu_lost = parser.imu_max_index - parser.imu_received

    def pct(lost, total):
        return f"{(100.0 * lost / total):.2f}%" if total else "-"

    print()
    print(f"MIC  {parser.mic_received}개 수신 / {parser.mic_max_index}개 기대"
          f"  유실 {mic_lost} ({pct(mic_lost, parser.mic_max_index)})  -> {wav_name}")
    print(f"IMU  {parser.imu_received}개 수신 / {parser.imu_max_index}개 기대"
          f"  유실 {imu_lost} ({pct(imu_lost, parser.imu_max_index)})  -> {csv_name}")

    if parser.mic_received:
        actual_rate = parser.mic_max_index / seconds
        print(f"실측 마이크 레이트 약 {actual_rate:.0f} Hz (보드 보고 {parser.mic_rate} Hz)")

    if parser.imu_received:
        # 보드가 보고한 ODR과 실측이 어긋나면 FIFO 배수가 못 따라간 것이다.
        # (보드의 STAT 줄 imu_fifo_ovr도 같이 볼 것)
        actual_imu = parser.imu_max_index / seconds
        reported = f"보드 보고 {parser.imu_rate} Hz" if parser.imu_rate else "보드 미보고"
        print(f"실측 IMU 레이트 약 {actual_imu:.0f} Hz ({reported})")

    if parser.wire_packets:
        # 무선 구간에 실제로 흐른 바이트로 계산한다. 압축을 켜면 "디코딩된
        # 샘플 수 x 원본 크기"와 크게 달라지므로 둘을 같이 본다.
        wire_kbps = parser.wire_bytes * 8 / seconds / 1000
        raw_bits = (parser.mic_received * 2 + parser.imu_received * 12) * 8
        print(f"실측 처리량 약 {wire_kbps:.0f} kbps"
              f"  ({parser.wire_packets / seconds:.0f} 패킷/초,"
              f" 압축 전 환산 {raw_bits / seconds / 1000:.0f} kbps)")

    if parser.profile is not None:
        print(f"프로파일 {parser.profile}: {PROFILE_NAMES.get(parser.profile, '?')}")

    for line in parser.status_lines:
        print("보드 보고:", line)
    if parser.bad_packets:
        print(f"경고: 해석 실패 패킷 {parser.bad_packets}개")


if __name__ == "__main__":
    asyncio.run(main())
