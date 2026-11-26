# NailDevice firmware

nRF52840 (S140 SoftDevice) 기반 IMU + 마이크 BLE 스트리밍 펌웨어.

## 폴더

| 폴더 | 대상 보드 | BLE 이름 |
|---|---|---|
| `nail_firmware/` | Nail 보드 | `NailDevice` |
| `erm_frimware/` | Dual ERM 보드 (BMI160) | `ERM` |
| `ble_app_uart/` | 개발용 (루트 `flash.sh`로 플래시) | `Nordic_UART` |
| `pc_receiver/` | PC 쪽 BLE 수신/로깅 스크립트 | - |

보드마다 펌웨어가 다르니 서로 바꿔서 굽지 말 것.

## 준비물

- nRF52840 DK (J-Link), 보드는 DK의 `DEBUG OUT`에 연결
- [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools) (`nrfjprog`)
- GNU Arm Embedded Toolchain **10.3.1** (`10 2021.10`)
  - 기본 경로: `C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/`
- GnuWin32 Make (`C:/Program Files (x86)/GnuWin32/bin/make.exe`)
- Git Bash (또는 WSL)
- nRF5 SDK → `C:/nrf_sdk` (다른 곳에 두면 각 Makefile의 `SDK_ROOT` 수정)

## 펌웨어 업로드

아래는 전부 Git Bash에서 실행. `<board>` 자리에 `nail_firmware` 또는 `erm_frimware`.

**1. 칩 연결 확인**

```bash
nrfjprog -f nrf52 --memrd 0x10000060 --n 8
```

**2. 처음 굽는 보드면 recover + SoftDevice 설치**

```bash
nrfjprog -f nrf52 --recover

cd <board>/ble_app_uart/pca10056/s140/armgcc
make flash_softdevice GNU_INSTALL_ROOT="C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/" GNU_VERSION=10.3.1
```

recover 하면 SoftDevice까지 지워지므로 SoftDevice 설치를 꼭 이어서 할 것.

**3. 앱 빌드 + 플래시**

```bash
cd <board>
bash flash.sh
```

`flash.sh`는 빌드 후 앱을 굽고, 실패하면 자동으로 recover → SoftDevice → 앱 순서로 다시 굽는다.

빌드 결과 HEX: `<board>/ble_app_uart/pca10056/s140/armgcc/_build/nrf52840_xxaa.hex`

## 문제 생겼을 때

- **구웠는데 광고가 안 뜸 / RTT 출력 없음**: SoftDevice 버전 확인.

  ```bash
  nrfjprog -f nrf52 --memrd 0x3008 --n 4
  ```

  `00027000`이 나와야 정상 (링커 스크립트 앱 시작 주소 0x27000). `00026000`이면 다른 SoftDevice(6.x)가 올라가 있는 상태라 2번부터 다시 할 것. 이 경우 `flash.sh`가 알아서 고치지 못한다.

- **`Unable to connect to a debugger`**: J-Link RTT Viewer가 켜져 있으면 끄고, DK USB를 뽑았다 다시 꽂기.

보드별 상세 내용은 각 폴더의 `FLASH_MANUAL.txt` 참고.
