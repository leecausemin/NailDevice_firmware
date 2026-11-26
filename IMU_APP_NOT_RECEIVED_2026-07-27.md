# IMU 신호가 폰 앱에서 안 잡히는 문제 — 코드 리딩 진단

작성일: 2026-07-27
대상: `firmware/ble_app_uart/main.c` (working tree, 2026-07-24 15:31), `NailDeviceApp` (APK 2026-07-24 14:25)

> ✅ **아래 §1·§2·§3·§4의 패치는 2026-07-27에 코드에 적용했다.** 변경 요약과 검증 순서는
> `PROGRESS.md` §8.6 / §8.6-A. **단, 보드·폰 미연결 상태여서 빌드도 실측도 아직 못 했다.**
> 이 문서는 "왜 그렇게 고쳤나"의 근거로 남긴다.

---

## 0. 결론 먼저

**와이어 포맷과 앱 디코더는 문제가 없다.** 아래 항목을 코드에서 그대로 옮겨 수치 검증했다.

| 검증 항목 | 결과 |
|---|---|
| `pack12()` ↔ 앱 `decodePackedPacket()` 12비트 IMU 라운드트립 (경계값·홀수 개수 포함 2000케이스) | **완전 일치 (하위 4비트만 0으로 복원)** |
| 6축 패킷 크기 | 26샘플 → 242바이트, 앱 길이검사와 일치 |
| `ulaw_encode_sample()` ↔ 앱 u-law 확장 | 진폭비 0.96~1.02, 스케일 정상 |
| 축 순서 | 펌웨어 `imu_sample_t {ax..gz}` ↔ 앱 accel-first 디코드 일치 |

그래서 원인은 **디코딩이 아니라 ① 센서 초기화, ② 프로파일 합의, ③ BLE 스케줄러** 중 하나다.
아래 §1이 가장 의심스럽고, §2는 "총체적 무수신"을 만들 수 있는 유일한 경로이며,
§3은 부분 유실을 전부 IMU에 몰아주는 구조적 문제다.

**중요한 시간축:** PROGRESS.md §6.6.6의 폰 실측(프로파일 5에서 마이크·IMU 양쪽 0.00%)은
**2026-07-22**다. APK는 07-24 14:25(소스는 07-23 17:43까지), `main.c`는 **07-24 15:31에 수정되고
15:51에 재빌드**됐다. 즉 **마지막 실측 이후 유일하게 바뀐 게 07-24 펌웨어 수정**이고,
그 수정 내용은 PROGRESS.md에 기록되지 않았다. `main.c`에서 07-24 날짜가 붙은 곳은 한 군데,
`lsm6_pins_init()`의 전원 사이클 추가다. → 1순위 용의자.

---

## 1. 1순위: `lsm6_pins_init()`가 CS를 띄운 채로 센서 전원을 다시 올린다

`main.c:1973` 부근 (07-24 추가분):

```c
static void lsm6_pins_init(void)
{
    nrf_gpio_cfg_output(IMU_PWR);
    nrf_gpio_pin_clear(IMU_PWR);   // 센서 전원 OFF
    nrf_delay_ms(100);
    nrf_gpio_pin_set(IMU_PWR);     // 센서 전원 ON
    nrf_delay_ms(20);
    nrf_gpio_cfg_output(IMU_SCK);  // ← 여기서야 SPI 핀을 잡는다
    nrf_gpio_cfg_output(IMU_MOSI);
    nrf_gpio_cfg_output(IMU_CS);
    nrf_gpio_cfg_input(IMU_MISO, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(IMU_SCK);
    nrf_gpio_pin_set(IMU_CS);      // ← CS를 HIGH로 만드는 건 전원이 다 올라온 뒤
}
```

문제는 순서다.

1. **전원이 내려간 100 ms 동안, 그리고 다시 올라오는 순간까지 `IMU_CS`/`IMU_SCK`/`IMU_MOSI`는
   설정 전이라 high-Z(입력)다.** LSM6DSL은 전원 상승 시점의 CS 상태로 인터페이스 모드가
   결정되는 부품이다. CS가 풀업 없이 떠 있으면 SPI/I2C 판정이 매번 달라질 수 있고,
   그 결과가 정확히 `WHO_AM_I = 0xFF` 또는 `0x00`이다 —
   `IMU_BLE_DEBUG_STATUS.md` §8에 적어둔 그 증상이다.
2. `nrf_delay_ms(20)`은 LSM6DSL의 전원 상승 + 부팅 시간을 다 덮기에 짧다.
   `lsm6_init()`이 50 ms 간격으로 10번 재시도하니 보통은 가려지지만,
   ①에서 모드가 잘못 latch되면 재시도로는 절대 안 풀린다(전원을 다시 사이클해야 한다).
3. 07-20 백업(`main.c.bak`)의 같은 함수는 **전원을 끄지 않았다** — `pin_set(IMU_PWR)` 한 줄뿐이다.
   즉 07-22 실측이 성공했을 때는 이 위험 구간이 없었다.

### 제안 패치

전원을 올리기 **전에** CS/SCK를 원하는 idle 상태로 구동하고, 상승 후 대기를 늘린다.

```c
static void lsm6_pins_init(void)
{
    /* ① 먼저 SPI 핀을 idle 상태로 확정한다. CS를 띄운 채 레일을 올리면
     *    LSM6DSL의 인터페이스 모드가 잘못 latch돼 WHO_AM_I가 0xFF/0x00만 나온다. */
    nrf_gpio_cfg_output(IMU_SCK);
    nrf_gpio_cfg_output(IMU_MOSI);
    nrf_gpio_cfg_output(IMU_CS);
    nrf_gpio_cfg_input(IMU_MISO, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(IMU_SCK);     // mode3 idle HIGH
    nrf_gpio_pin_clear(IMU_MOSI);
    nrf_gpio_pin_set(IMU_CS);      // CS 비활성(HIGH)을 먼저 확보

    /* ② 그 다음 전원 사이클. */
    nrf_gpio_cfg_output(IMU_PWR);
    nrf_gpio_pin_clear(IMU_PWR);
    nrf_delay_ms(100);             // 레일 방전
    nrf_gpio_pin_set(IMU_PWR);
    nrf_delay_ms(50);              // 20ms는 부팅에 부족하다 (t_boot + 안정화)
}
```

> ⚠️ 레일이 내려간 상태에서 출력으로 HIGH를 물리면 센서 ESD 다이오드를 통한 back-power가
> 생길 수 있다. 그게 걱정되면 ①에서 CS만 `NRF_GPIO_PIN_PULLUP` 입력으로 두고,
> 레일이 올라온 뒤 출력으로 승격시키는 방식이 더 안전하다. 둘 중 어느 쪽이든
> **CS가 전원 상승 순간에 확실히 HIGH여야 한다**는 점이 핵심이다.

**확인 방법:** RTT 로그에 `LSM6DSL init OK (WHO_AM_I=0x6A)`가 뜨는지, 또는 앱 연결 후
`RATE,...,imu_ok=1`인지 본다. `imu_ok=0`이면 이 절이 원인이고, IMU 값은 전부 0으로 나온다.

---

## 2. 2순위: 프로파일이 실제로 적용됐는지 앱이 확인하지 않는다 (총체적 무수신 경로)

- 보드는 부팅 시 **항상 `STREAM_PROFILE_RAW`(0)**로 시작한다 (`main.c:381`).
- 앱은 연결할 때 `0xA2 <profile>`을 보낸다. 기본값은 `StreamProfile.BothMic8` = **5**.
- 보드는 `requested > STREAM_PROFILE_MAX`면 **조용히 무시**하고 RTT에만 남긴다
  (`Profile %u rejected`). 그리고 `RATE,...,profile=N`으로 현재 값을 다시 알려준다.
- **그런데 앱 `NailBleClient.onText()`는 RATE 줄에서 `mic_hz`와 `imu_hz`만 파싱하고
  `profile=`은 읽지 않는다.** 즉 프로파일 협상이 실패해도 앱은 자기가 5라고 믿는다.

이게 왜 IMU 무수신으로 이어지는가: 보드가 프로파일 0(raw)에 머무르면
마이크 16 kHz + IMU 6664 Hz = **929 kbps**가 되고, 링크 상한은 약 600 kbps다.
그리고 §3의 스케줄러는 마이크를 먼저 다 비운다 → **부족분 전부가 IMU 몫에서 빠진다.**
PROGRESS.md §6.6.5에 이미 같은 현상이 기록돼 있다: *"3333 Hz — 마이크 0.00%, IMU 시작 버스트 후 0"*.
**차트가 처음 한순간 움직였다가 죽는다면 거의 확실히 이 경로다.**

발생 조건은 두 가지 중 하나다.
- 보드에 구워진 hex가 프로파일 5를 모르는 빌드(= 07-24 이전 빌드)다 → 5는 거부되고 0으로 남는다.
- 또는 `0xA2` 쓰기가 유실됐다(`writeCommand` 큐가 250 ms ack 타임아웃으로 넘어가는 경우).

### 제안 패치 (앱)

`onText()`의 RATE 파싱에 profile을 추가해, 보드가 보고한 값과 앱의 값이 다르면
경고하고 앱 상태를 보드 쪽으로 맞춘다(또는 재전송한다).

```kotlin
if (field.startsWith("profile=")) {
    field.removePrefix("profile=").toIntOrNull()?.let { code ->
        val onBoard = StreamProfile.entries.firstOrNull { it.code == code }
        if (onBoard != null && onBoard != _status.value.streamProfile) {
            Log.e(TAG, "profile mismatch: app=${_status.value.streamProfile.code} board=$code" +
                " — 0xA2 가 거부되거나 유실됐다. 보드 값으로 맞춘다.")
            _status.update { it.copy(streamProfile = onBoard) }
            expectedImuIndex = -1
            expectedMicIndex = -1
        }
    }
}
```

`imu_ok=0`도 지금은 logcat에만 남는다. `DeviceStatus`에 `imuOk`/`boardProfile`을 넣고
StatusPanel에 띄우면, "IMU가 왜 0인가"를 앱만 보고 판단할 수 있다.

---

## 3. 구조적 문제: `stream_service()`가 부족분을 100% IMU에 전가한다

`main.c:1688` 이후:

```c
/* ① 마이크를 먼저 다 비운다. */
while (m_mic_tail != m_mic_head) { if (!stream_send_mic_packet(...)) return; }
/* ② 남은 대역은 전부 IMU. */
while (m_imu_tail != m_imu_head) { if (!stream_send_imu_packet(...)) return; }
```

- 이 정책 자체는 의도된 것이다(PROGRESS.md §6.6.5: 라운드로빈이 사실상 IMU 우선이어서
  마이크가 91% 죽었던 것을 고친 결과). 하지만 결과적으로 **링크가 1 kbps라도 모자라면
  손실이 전부 IMU에 쏠린다.** 그래서 "마이크는 완벽한데 IMU만 안 잡힌다"는 증상이
  이 시스템의 **기본 실패 모드**다.
- 그리고 **함수 위 doxygen 주석은 아직 "IMU와 마이크를 한 패킷씩 번갈아 보낸다"고 적혀 있다.**
  코드는 마이크 우선인데 주석은 라운드로빈이다. 다음 사람이 반드시 오해한다 — 주석을 고쳐야 한다.
- 앱 기본값(`DeviceStatus`)이 마이크 16 kHz + IMU 6664 Hz + **프로파일 5(651 kbps 실측)**인데,
  PROGRESS.md의 검증된 운용점은 **프로파일 3(594 kbps, 양쪽 0.00%)**이다.
  프로파일 5는 상한(약 600~650 kbps)에 마진이 거의 없다. **기본값을 `Both`(3)로 내리는 것을 권한다.**
  마이크 정밀도가 필요할 때만 설정에서 5로 올리면 된다.

여유를 만드는 확실한 카드는 IMU 3333 Hz(§6.6.5에서 570 kbps) 또는 프로파일 4다.

---

## 4. 그 외 확인된 작은 결함

| 위치 | 내용 |
|---|---|
| `main()` 프로파일 요청 처리 | `m_profile` 변경 시 링(`m_imu_head/tail`)은 0으로 되돌리지만 **FIFO를 재시작하지 않고** `m_imu_t0_valid`도 안 지운다. 세션 중간에 프로파일을 바꾸면 샘플 경계 위상이 남아 축이 밀릴 수 있고(§8 트러블슈팅의 "축끼리 뒤섞임"), SYNC 원점은 낡은 값이 된다. `0xA1` 경로처럼 `lsm6_fifo_start()`를 같이 불러줄 것. |
| `lsm6_set_rate()` 실패 경로 | `m_imu_ok`가 false면 ODR 변경이 조용히 무시된다. RATE 줄에 `imu_ok=0`이 실려 나가지만 앱이 UI로 보여주지 않아 사용자는 알 수 없다. |
| 앱 `ImuSamplingRate` | 104/208 Hz가 목록에서 빠졌다(펌웨어는 여전히 지원). 링크 문제를 격리할 때 저레이트 카드가 없어진 셈이라, 디버깅용으로 하나쯤 남기는 게 낫다. |
| 앱 차트 | IMU 패킷이 0이면 `TickResampler`가 마지막 값을 hold한다 → **차트는 "0에서 평평한 선"으로 보이고 에러는 없다.** 무수신과 "센서가 정지" 상태가 화면상 구분되지 않는다. `link:` 줄의 `imu ... rx`를 같이 봐야 한다. |

---

## 5. 3분 트리아지 (이 순서대로 보면 원인이 한 개로 좁혀진다)

```
! "$ADB" logcat -s NailBleClient
```

1. **`RATE,...,imu_ok=` 값**
   - `0` → **§1**이다. 센서 초기화 실패. 펌웨어 패치 후 재플래시, 그리고 Nail 전원 물리적으로 껐다 켜기.
   - `1` → 다음으로.
2. **`RATE,...,profile=` 값** (logcat의 `board:` 줄에 그대로 찍힌다)
   - 앱에서 고른 값과 다르면 → **§2**다. 프로파일이 적용되지 않았다.
   - 같으면 다음으로.
3. **`link:` 줄의 `imu N rx / M lost`**
   - `0 rx`가 계속 → 보드가 아예 안 보내는 것. `e` 정지 후 `STAT,...,imu_produced=`를 본다.
     `imu_produced=0`이면 드레인/FIFO 경로(§4 첫 줄), `>0`인데 앱이 0 rx면 BLE 스케줄러(§3).
   - `rx > 0`인데 유실률이 크다 → **§3**. 프로파일 3으로 내리거나 IMU 3333 Hz로.
4. **rx도 정상인데 값이 전부 0/이상함**
   - 정지 상태에서 `|accel| ≈ 1.0 g`인지 확인. 아니면 축 정렬/스케일 문제인데,
     디코더 자체는 이번에 수치 검증했으므로 FIFO 위상 정렬(§4)을 먼저 의심할 것.

가장 빠른 격리 실험: **PC 수신기로 같은 설정을 돌려본다.**

```
! cd /c/Users/user/Desktop/CM_LAB/2026_summer/firmware/pc_receiver && python stream_logger.py 10 4 1666 3
```

여기서 IMU가 나오면 보드는 멀쩡하고 앱/링크 문제(§2·§3), 안 나오면 보드 문제(§1)다.
(07-22에도 이 방법으로 GATT 쓰기 유실을 찾아냈다 — PROGRESS.md §8.5.)
