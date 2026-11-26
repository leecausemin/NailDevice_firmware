"""
test_logger.py - 자동 파이프라인 검증
연결 -> 's'(시작) -> 3초 기록 -> 'e'(정지) -> 데이터 수신 -> CSV 저장
사용법: python test_logger.py
"""
import asyncio
import csv
import datetime
from bleak import BleakScanner, BleakClient

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> 보드 (명령 쓰기)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 -> PC (알림 수신)

lines = []
partial = ""
done = asyncio.Event()


def on_notify(_, data: bytearray):
    """보드가 보낸 데이터 조각을 줄 단위로 모은다."""
    global partial
    text = partial + bytes(data).decode(errors="replace")
    parts = text.split("\n")
    partial = parts[-1]                 # 마지막 미완성 조각은 남겨둠
    for ln in parts[:-1]:
        ln = ln.strip()
        if ln == "END":
            done.set()                  # 전송 끝 신호
        elif ln:
            lines.append(ln)


async def main():
    print("보드(Nordic_UART) 검색...")
    dev = await BleakScanner.find_device_by_name("Nordic_UART", timeout=10.0)
    if dev is None:
        print("보드를 못 찾음. LED/광고 상태 확인.")
        return

    async with BleakClient(dev) as client:
        print("연결됨:", dev.address)
        await client.start_notify(NUS_TX, on_notify)

        print("[s] 녹화 시작 -> 3초간 기록...")
        await client.write_gatt_char(NUS_RX, b"s", response=False)
        await asyncio.sleep(3.0)

        print("[e] 정지 -> 데이터 수신 대기...")
        done.clear()
        lines.clear()
        await client.write_gatt_char(NUS_RX, b"e", response=False)
        try:
            await asyncio.wait_for(done.wait(), timeout=30.0)
        except asyncio.TimeoutError:
            print("(타임아웃 - 받은 것까지만 저장)")

        await client.stop_notify(NUS_TX)

    fn = "test_" + datetime.datetime.now().strftime("%H%M%S") + ".csv"
    with open(fn, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "value"])
        for ln in lines:
            p = ln.split(",")
            if len(p) == 2:
                w.writerow(p)

    print(f"\n수신 {len(lines)}개 -> 저장: {fn}")
    if lines:
        print("처음 3개:", lines[:3], "  마지막:", lines[-1])


if __name__ == "__main__":
    asyncio.run(main())
