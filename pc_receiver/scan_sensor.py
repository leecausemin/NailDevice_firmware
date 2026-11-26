"""
scan_sensor.py - 센서 WHO_AM_I 스캔 결과를 BLE로 받아 출력
연결 -> 'w'(스캔 요청) -> 결과 줄 수신 -> END 까지 그대로 출력
사용법: python scan_sensor.py
기대: LSM6DSL이 잡히면 어떤 줄에 '= 0x6A  <<< FOUND!' 가 나온다.
"""
import asyncio
from bleak import BleakScanner, BleakClient

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> 보드 (명령 쓰기)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 -> PC (알림 수신)

partial = ""
done = asyncio.Event()
found = False


def on_notify(_, data: bytearray):
    global partial, found
    text = partial + bytes(data).decode(errors="replace")
    parts = text.split("\n")
    partial = parts[-1]
    for ln in parts[:-1]:
        ln = ln.rstrip()
        if ln == "END":
            done.set()
        elif ln:
            print("  ", ln)
            if "= 0x6A" in ln:   # 결과 줄에서만 판정 (안내문 오탐 방지)
                found = True


async def main():
    print("보드(Nordic_UART) 검색...")
    dev = await BleakScanner.find_device_by_name("Nordic_UART", timeout=10.0)
    if dev is None:
        print("보드를 못 찾음. 전원/광고 상태 확인.")
        return

    async with BleakClient(dev) as client:
        print("연결됨:", dev.address)
        await client.start_notify(NUS_TX, on_notify)

        print("[w] 센서 스캔 요청 -> 결과 수신:\n")
        await client.write_gatt_char(NUS_RX, b"w", response=False)
        try:
            await asyncio.wait_for(done.wait(), timeout=30.0)
        except asyncio.TimeoutError:
            print("(타임아웃 - 받은 것까지만 표시)")

        await client.stop_notify(NUS_TX)

    print()
    if found:
        print(">>> 센서 발견! (위 줄 중 0x6A 있는 조합이 정답 핀맵)")
    else:
        print(">>> 이번에도 0x6A 없음 (모두 0x00/0xFF). 배선/핀맵 재검토 필요.")


if __name__ == "__main__":
    asyncio.run(main())
