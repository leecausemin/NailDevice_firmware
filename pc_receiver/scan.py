"""
scan.py - 주변 BLE 기기를 스캔해서 우리 보드(Nordic_UART)를 찾는다.
사용법:  python scan.py
"""
import asyncio
from bleak import BleakScanner

# Nordic UART Service(NUS)의 서비스 UUID
NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"


async def main():
    print("BLE 스캔 중... (5초 대기)\n")
    devices = await BleakScanner.discover(timeout=5.0, return_adv=True)

    if not devices:
        print("아무 기기도 못 찾음. (보드 전원/광고 상태 확인)")
        return

    ours = None
    for addr, (dev, adv) in devices.items():
        name = adv.local_name or (dev.name if dev.name else "(이름 없음)")
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        is_ours = ("Nordic_UART" in name) or (NUS_SERVICE in uuids)
        mark = "   <<<=== 우리 보드!" if is_ours else ""
        print(f"  {name:22s} {addr}   RSSI={adv.rssi}{mark}")
        if is_ours:
            ours = (name, addr)

    print()
    if ours:
        print(f"찾았다! '{ours[0]}'  주소={ours[1]}")
        print("=> 다음 단계: 이 보드에 연결해서 데이터 받기")
    else:
        print("Nordic_UART를 못 찾음. 보드 LED1이 깜빡이는지, 전원 스위치 확인.")


if __name__ == "__main__":
    asyncio.run(main())
