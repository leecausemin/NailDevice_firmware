# Nail Device IMU/BLE 디버깅 진행 상황

작성일: 2026-07-16

## 1. 최종 목표

Nail Device에 장착된 센서 중 하나 이상의 데이터를 수집하여 BLE로 PC에 전송한다.

- IMU: 가속도/자이로 데이터
- 마이크: 음성 또는 진동 데이터
- 우선순위: 먼저 통신에 성공하는 센서를 사용

## 2. 사용 중인 장비와 연결

### 장비

- 디버거 보드: Nordic nRF52840 DK, PCA10056
- 대상 보드: ISP1807 기반 Nail flexible PCB
- 대상 MCU: ISP1807 내부 nRF52840 Rev3
- 연결 어댑터: Cortex SWD Adapter

### SWD 연결

PCA10056의 `DEBUG OUT`을 Cortex SWD Adapter를 거쳐 Nail Device에 연결했다.

| 선 색상 | 어댑터 표기 | 실제 역할 |
|---|---|---|
| 검은색 | GND | 공통 접지 |
| 빨간색 | TMS | SWDIO |
| 황금색 | TCK | SWDCLK |

이 세 선은 센서 데이터 통신선이 아니라, Nail Device의 MCU를 프로그래밍하고 디버깅하기 위한 SWD 연결이다.

## 3. 정상 동작이 확인된 부분

다음 명령이 정상 동작했다.

```powershell
nrfjprog --deviceversion -f NRF52
```

출력:

```text
1050232190 - NRF52840_xxAA_REV3
```

따라서 다음 경로는 정상이다.

```text
PC
  → PCA10056 디버거
  → DEBUG OUT
  → Cortex SWD Adapter
  → Nail Device의 ISP1807/nRF52840 MCU
```

추가로 확인된 사항:

- Nail Device 배터리/전원 연결
- 펌웨어 빌드 성공
- 펌웨어 플래시 성공
- 플래시 후 verify 성공
- 대상 MCU 리셋 성공
- RTT 로그 출력 성공
- BLE NUS 기반 광고 및 PC 통신용 코드 실행 가능

`nrfjprog --deviceversion`은 MCU 연결만 확인한다. MCU에 연결된 IMU나 마이크의 동작까지 확인하는 명령은 아니다.

## 4. PPT에서 확인한 실제 부품

검사한 파일:

```text
C:\Users\user\Desktop\CM_Lab\하계인턴\펌웨어\Nail Device Soldering.pptx
```

PPT의 BOM과 PCB 이미지를 통해 확인한 주요 부품:

| 역할 | 부품 |
|---|---|
| MCU/BLE 모듈 | ISP1807 |
| IMU | LSM6DSLTR |
| 마이크/진동 센서 | V2S200D |
| 마이크 형식 | Single-bit PDM |
| 전압 레귤레이터 | TPS7A0230PDQNR, 3 V LDO |
| 비교기 | MIC842HYMT-TR |
| 외부 메모리 | MT29F4G01 계열 |

PPT에 포함된 PCB 이미지는 앞·뒷면 레이아웃 캡처다. 부품 위치와 동박 경로는 보이지만 일반적인 회로도처럼 다음 정보가 문자로 명시되어 있지는 않다.

```text
LSM6DSL SCK → nRF52840 P0.xx
LSM6DSL MOSI → nRF52840 P0.xx
```

PCB의 일부 신호는 `NS2`, `NS3`, `NS4` 같은 자동 네트명으로 표시되어 있다.

## 5. LSM6DSL 핀 역할

확인된 LSM6DSL 쪽 SPI 연결은 다음과 같다.

| SPI 신호 | LSM6DSL 핀 |
|---|---|
| IMU_SCK | 13번 SCL |
| IMU_MOSI | 14번 SDA |
| IMU_MISO | 1번 SDO/SA0 |
| IMU_CS | 12번 CS |

아직 필요한 정보는 이 신호들이 ISP1807/nRF52840의 어느 GPIO에 연결되었는지다.

예:

```text
LSM6DSL 13 SCL  → ISP1807/nRF52840 P?.??
LSM6DSL 14 SDA  → ISP1807/nRF52840 P?.??
LSM6DSL 1 SDO   → ISP1807/nRF52840 P?.??
LSM6DSL 12 CS   → ISP1807/nRF52840 P?.??
```

## 6. 사수 코드에서 발견한 핀 설정

검사한 폴더:

```text
C:\Users\user\Desktop\CM_Lab\하계인턴\펌웨어\JY_Firmware
```

이 폴더에는 여러 하드웨어용 코드와 설정이 혼합되어 있다.

### LSM6DSL/HW8 후보

`app_config.h`에서 발견한 설정:

```c
#define TPS_CTRL_PIN      14
#define SENSOR_SCK_PIN    27
#define SENSOR_MOSI_PIN   15
#define SENSOR_MISO_PIN   6
#define IMU_SS_PIN        25
#define IMU_INT1_PIN      8
```

이를 nRF52840 GPIO로 표현하면:

| 역할 | GPIO |
|---|---|
| 전원 제어 후보 | P0.14 |
| SCK | P0.27 |
| MOSI | P0.15 |
| MISO | P0.06 |
| CS | P0.25 |
| INT1 | P0.08 |

### 다른 리비전으로 추정되는 주석 처리 설정

```text
SCK=17, MOSI=13, MISO=15, CS=19
SCK=27, MOSI=26, MISO=40, CS=39
```

### BMI160 테스트 보드 후보

`main.c`에 다음 설정도 존재했다.

```text
SCK=40, MOSI=39, MISO=37, CS=38
TPS 제어 후보=23, 19
```

이 설정은 Nail PPT의 LSM6DSL이 아니라 다른 BMI160 시험 보드용 코드로 판단된다.

## 7. 수행한 IMU 진단

LSM6DSL의 `WHO_AM_I` 레지스터 주소는 `0x0F`이며 정상값은 `0x6A`다.

정상 통신:

```text
MCU가 0x0F를 읽음
→ LSM6DSL이 0x6A 반환
```

### 7.1 기본 SPI 검사

다음 LSM6DSL 후보 핀맵을 검사했다.

```text
SCK=27, MOSI=15, MISO=6, CS=25
```

결과:

```text
WHO_AM_I = 0xFF
```

### 7.2 사수 코드에 남은 다른 핀맵 검사

다음 핀맵들도 검사했다.

```text
SCK=17, MOSI=13, MISO=15, CS=19
SCK=27, MOSI=26, MISO=40, CS=39
```

결과는 `0x00` 또는 `0xFF`였으며 `0x6A`는 나오지 않았다.

### 7.3 SPI mode 검사

LSM6DSL 후보에 대해 다음 모드를 검사했다.

- SPI mode 0
- SPI mode 3

정상 ID `0x6A`는 검출되지 않았다.

### 7.4 저속 제한 SPI 자동 스캔

안전성을 고려해 사수 코드에서 등장한 신호 묶음만 사용했다.

- SPI 속도: 125 kHz
- SCK/MOSI/MISO 후보 묶음
- MOSI/MISO 반전 조합
- CS 후보: 8, 19, 23, 25, 36, 38, 39, 43
- SPI mode 0/3
- ID를 두 번 읽어 초기 SPI 전환 문제 확인

결과:

- `0x00`, `0xFF` 이외의 유효 응답 없음
- `0x6A` 미검출

### 7.5 I2C 가능성 검사

LSM6DSL은 SPI와 I2C를 모두 지원하므로 I2C로 연결됐을 가능성도 검사했다.

- 주소 후보: `0x6A`, `0x6B`
- 사수 코드에 등장한 GPIO 쌍 10개
- GPIO open-drain 방식의 저속 소프트웨어 I2C 사용

결과:

```text
I2C ACK 없음
```

현재는 I2C보다 SPI로 연결됐을 가능성이 높다고 판단한다.

### 7.6 전원 제어 후보 P0.14

사수 설정에는 `TPS_CTRL_PIN=14`가 있다.

- P0.14 LOW 상태에서는 펌웨어 실행과 BLE/RTT가 유지됨
- P0.14 HIGH 상태에서 검사 중 실행이 중단되는 현상이 발생

따라서 현재 자동 탐색에서는 P0.14 HIGH를 사용하지 않고 LOW 상태를 유지한다.

P0.14가 실제 센서 전원 EN인지, 다른 중요 신호인지 아직 확정되지 않았다.

## 8. `0xFF`와 `0x00`의 의미

### `0x6A`

```text
LSM6DSL이 정상적으로 발견됨
```

### `0xFF`

이 값은 일반적으로 센서가 실제로 `0xFF`를 응답했다기보다 MISO가 계속 HIGH로 읽혔다는 뜻이다.

가능한 원인:

- MISO GPIO 번호가 틀림
- CS GPIO 번호가 틀림
- IMU가 선택되지 않음
- IMU 전원이 꺼짐
- MISO 입력이 연결되지 않아 HIGH로 떠 있음

### `0x00`

MISO가 계속 LOW로 읽힌 상태일 가능성이 높다.

가능한 원인:

- MISO GPIO 번호가 틀림
- 센서 전원이 꺼짐
- 해당 신호가 GND 또는 LOW 상태의 다른 회로에 연결됨

`0xFF` 또는 `0x00`만으로 IMU 고장을 확정할 수 없다.

## 9. 마이크 상태

PPT에서 마이크가 V2S200D single-bit PDM 방식이라는 것은 확인했다.

PDM 마이크에는 LSM6DSL의 `WHO_AM_I` 같은 고정 ID 레지스터가 없다. 따라서 다음 핀을 알아야 한다.

```text
PDM CLK
PDM DATA
필요한 경우 마이크 전원 제어 GPIO
```

JY_Firmware에는 일반적인 PDM 마이크 드라이버가 포함되어 있지만 Nail PCB의 정확한 `MIC_CLK`, `MIC_DATA` GPIO 정의는 발견하지 못했다.

## 10. BLE 및 PC 수신 코드

현재 작업 중인 펌웨어:

```text
C:\Users\user\Desktop\CM_LAB\하계인턴\firmware\ble_app_uart\main.c
```

PC 수신 코드:

```text
C:\Users\user\Desktop\CM_LAB\하계인턴\firmware\pc_receiver\receiver.py
```

현재 펌웨어는 Nordic UART Service 기반이다.

- PC에서 `s` 전송: 기록 시작
- PC에서 `e` 전송: 기록 중지 및 BLE 덤프

아직 실제 센서 값 대신 시험용 증가값을 저장하는 부분이 남아 있다. IMU 또는 마이크 핀맵이 확인되면 이 부분을 실제 센서 읽기로 교체할 수 있다.

## 11. 기존 수집 데이터 발견

다음 폴더에 동일 계열 장치에서 이미 수집한 것으로 보이는 데이터가 있다.

```text
C:\Users\user\Desktop\CM_LAB\하계인턴\Nail
```

발견한 파일 종류:

- `IMU_M_HJ_V1_....csv`
- `GYRO_M_HJ_V1_....csv`
- `Mic_M_HJ_V1_....csv`
- `Temp_M_HJ_V1_....csv`

예시 CSV 열:

```text
time,AccX,AccY,AccZ
time,Mic1,Mic2
```

따라서 Nail Device에서 IMU와 마이크 데이터를 수집한 이력이 있으며, 하드웨어 설계 자체는 동작 가능한 것으로 보인다.

해당 데이터를 생성한 PC 수집 프로그램이나 완성 펌웨어는 현재 폴더에서 발견하지 못했다.

## 12. 현재 문제의 핵심

현재 MCU 프로그래밍과 BLE 실행은 정상이다.

막힌 지점은 다음 하나다.

```text
ISP1807/nRF52840 MCU와 LSM6DSL 또는 V2S200D 사이의 정확한 GPIO 핀맵이 아직 확정되지 않음
```

현재 상태를 그림으로 표현하면:

```text
PC → PCA10056 → SWD → ISP1807/nRF52840   정상
                              ├─ BLE     정상
                              ├─ IMU     GPIO/전원 미확정
                              └─ 마이크  GPIO/전원 미확정
```

## 13. 해결 방법

### 방법 A: 원본 설계 파일 또는 핀맵 확보

가장 빠르고 안전하다.

필요한 자료:

- 실제 Nail Device 펌웨어 소스
- 실제 사용했던 HEX 파일
- 회로도 PDF
- Altium `.SchDoc`, `.PcbDoc`
- KiCad `.kicad_sch`, `.kicad_pcb`
- Eagle `.sch`, `.brd`
- LSM6DSL 및 V2S200D GPIO 핀맵

사수에게 요청할 문장:

> Nail Device의 실제 핀맵을 알 수 있을까요? LSM6DSL의 SCK, MOSI, MISO, CS와 V2S200D의 PDM CLK/DATA가 ISP1807의 어느 GPIO에 연결됐는지 필요합니다. 실제 사용했던 펌웨어 소스나 HEX, 회로도 또는 PCB 설계 원본이 있으면 부탁드립니다.

### 방법 B: 멀티미터 도통 검사

원본 자료가 없다면 전원을 끈 상태에서 멀티미터 도통 모드로 확인한다.

찾아야 하는 LSM6DSL 핀:

- 13번 SCL/SCK
- 14번 SDA/MOSI
- 1번 SDO/MISO
- 12번 CS

각 핀이 ISP1807의 어느 패드로 연결되는지 확인한 뒤, ISP1807 공식 핀표를 통해 nRF52840 GPIO 번호로 변환한다.

### 방법 C: GPIO 자동 탐색 확대

소프트웨어로 더 넓은 GPIO 조합을 검사할 수는 있다. 하지만 모든 GPIO를 무작정 출력으로 구동하면 다음 회로와 충돌할 수 있다.

- 전원 레귤레이터
- 배터리/충전 회로
- 외부 플래시
- PDM 마이크
- SWD
- 기타 제어 신호

따라서 PCB 배선 또는 도통 검사로 후보를 줄인 뒤 실행하는 것이 안전하다.

## 14. 다음 작업

정확한 IMU GPIO 네 개가 확인되면:

1. LSM6DSL `WHO_AM_I=0x6A` 확인
2. LSM6DSL 초기화
3. 가속도 및 자이로 데이터 읽기
4. 보드를 움직일 때 값 변화 확인
5. 시험용 증가값을 실제 IMU 값으로 교체
6. BLE NUS로 PC 전송
7. `receiver.py`에서 CSV 저장

마이크 GPIO가 먼저 확인되면:

1. nRF52840 PDM 초기화
2. PDM 샘플 버퍼 수집
3. 보드를 두드리거나 소리를 낼 때 변화 확인
4. BLE 전송량에 맞게 다운샘플링 또는 특징값 계산
5. PC에서 CSV 또는 오디오 데이터로 저장

