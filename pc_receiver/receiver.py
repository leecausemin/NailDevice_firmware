"""
receiver.py - 대화형 데이터 로거 수신기
  s  입력 -> 보드에 녹화 시작 신호
  e  입력 -> 정지 + 데이터 받아서 CSV로 저장
  q  입력 -> 종료
사용법: python receiver.py
"""
import asyncio
import csv
import datetime
from bleak import BleakScanner, BleakClient

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> 보드 (명령)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 -> PC (데이터 알림)

lines = []
partial = ""
done = asyncio.Event()


def on_notify(_, data: bytearray):
    global partial
    text = partial + bytes(data).decode(errors="replace")
    parts = text.split("\n")
    partial = parts[-1]
    for ln in parts[:-1]:
        ln = ln.strip()
        if ln == "END":
            done.set()
        elif ln:
            lines.append(ln)


def save_csv():
    fn = "log_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    with open(fn, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "value"])
        for ln in lines:
            p = ln.split(",")
            if len(p) == 2:
                w.writerow(p)
    return fn


async def main():
    print("보드(Nordic_UART) 검색...")
    dev = await BleakScanner.find_device_by_name("Nordic_UART", timeout=10.0)
    if dev is None:
        print("보드를 못 찾음. LED/광고 상태 확인.")
        return

    async with BleakClient(dev) as client:
        print("연결됨:", dev.address)
        await client.start_notify(NUS_TX, on_notify)
        loop = asyncio.get_event_loop()

        print("\n명령:  s=녹화시작   e=정지+저장   q=종료")
        while True:
            cmd = (await loop.run_in_executor(None, input, "> ")).strip().lower()

            if cmd == "s":
                await client.write_gatt_char(NUS_RX, b"s", response=False)
                print("  >> 녹화 시작. 보드가 데이터를 쌓는 중...")

            elif cmd == "e":
                done.clear()
                lines.clear()
                await client.write_gatt_char(NUS_RX, b"e", response=False)
                print("  >> 정지. 데이터 수신 중...")
                try:
                    await asyncio.wait_for(done.wait(), timeout=60.0)
                except asyncio.TimeoutError:
                    print("  (타임아웃 - 받은 것까지만 저장)")
                fn = save_csv()
                print(f"  >> {len(lines)}개 저장 완료 -> {fn}")

            elif cmd == "q":
                break
            else:
                print("  s / e / q 중 하나를 입력하세요.")

        await client.stop_notify(NUS_TX)
    print("종료.")


if __name__ == "__main__":
    asyncio.run(main())
