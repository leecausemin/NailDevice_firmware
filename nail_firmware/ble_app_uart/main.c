/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_spi.h"   /* BMI160 IMU 통신용 SPI */
#include "nrfx_pdm.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "NailDevice"                                /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

/* 광고 타임아웃(10ms 단위). 0 = 무제한(계속 광고).
 * 과거에 0이 안 먹었던 건 flags가 LIMITED_DISC_MODE였기 때문 — Limited 탐색 모드는
 * 스펙상 최대 180초라 무제한을 SoftDevice가 거부했다. GENERAL_DISC_MODE로 바꾼
 * 지금은 0(무제한)이 정상 동작하고, 180초 만에 광고가 꺼지던 문제도 사라진다.
 * on_adv_evt의 IDLE 재광고는 만일을 위한 예비로 남겨둔다(0이면 사실상 호출 안 됨). */
#define APP_ADV_DURATION                0                                           /**< 0 = unlimited advertising. */

/* 처리량 우선 설정. 연결 간격이 짧을수록 초당 전송 기회가 많아진다.
 * 7.5ms는 BLE 규격상 최소값이며 Android가 실제로 잘 받아준다.
 * (iOS는 최소 15ms라 MAX 쪽으로 수렴한다) */
/* MIN=MAX=7.5ms로 못박는다. 범위를 주면 안드로이드가 가운데(11.25ms)를 골랐다(실측).
 *
 * 간격이 왜 중요한가: 안드로이드는 연결 이벤트당 처리하는 패킷 수에 상한이 있다.
 * 11.25ms에서 실측 817kbps = 이벤트당 약 4.7패킷인데, 이론상 8패킷이 들어갈 시간이
 * 있는데도 그렇다. 즉 시간이 아니라 패킷 수가 병목이다. 그러면 간격을 줄이는 만큼
 * 초당 이벤트가 늘어 처리량이 그대로 비례해 올라간다(7.5ms면 1.5배 기대). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< 15 ms로 고정 */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< 15 ms로 고정 */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
/* 5초는 스트리밍에 너무 길다. 그동안은 상대가 정한 느린 간격으로 돌아서
 * 초반 데이터가 통째로 유실된다. 1초로 당겨 빨리 빠른 간격으로 넘어간다. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                       /**< Time from initiating event to first sd_ble_gap_conn_param_update call (1 second). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

/* 알림 송신 큐 깊이. 이 값을 올리면 SoftDevice가 요구하는 RAM도 늘어나므로
 * 링커의 RAM ORIGIN을 같이 올려야 한다. 부족하면 sd_ble_enable이 NO_MEM으로
 * 죽어서 광고조차 안 뜬다(플래시 직후 기기가 안 보이면 이걸 먼저 의심할 것).
 * 대략 큐 1칸당 (MTU 247 + 헤더) ≈ 251바이트로 잡으면 된다. */
/* 24로 올려봤지만 처리량 변화가 없었다(2M 기준 610 -> 607 kbps). 큐는 병목이
 * 아니다. MTU를 517로 키우면서 큐 1칸이 251 -> 521바이트가 됐으므로, 쓸데없이
 * 깊게 잡아 SoftDevice RAM만 먹지 않도록 10으로 되돌린다. */
#define HVN_TX_QUEUE_SIZE               10

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/* =====================================================================
 *  실시간 스트리밍 로거 (real-time streaming logger)
 *  - PC/폰이 's' 보내면 센서 가동 -> 샘플이 쌓이는 즉시 BLE로 흘려보냄
 *  - 'e' 보내면 센서 정지 -> 남은 잔량을 밀어낸 뒤 상태 요약 전송
 *
 *  생산자(PDM ISR, 샘플 타이머) -> 링버퍼 -> 소비자(메인 루프, 논블로킹 전송).
 *  전송이 막혀도 센서 샘플링 박자는 유지된다. 링버퍼가 넘치면 샘플을 버리되,
 *  패킷에 실린 절대 샘플 번호로 수신측이 유실 위치와 개수를 정확히 알 수 있다.
 * ===================================================================== */
/* --- IMU 취득 방식: 센서 내장 FIFO를 주기적으로 몰아 읽는다 ---
 *
 * 예전 방식은 app_timer로 ODR과 비슷한 주기를 만들어 매번 한 샘플씩 읽었다.
 * 그 방식은 두 가지 이유로 1.6kHz 이상에서 무너진다:
 *   ① RTC 틱이 16384Hz라 주기를 틱 정수로만 표현할 수 있다. 416Hz를 420Hz로
 *      폴링하던 것처럼 항상 어긋나고(같은 샘플 중복/누락), 3.3kHz면 5틱=3276Hz,
 *      6.6kHz는 app_timer 최소 주기(5틱) 자체를 넘어 아예 만들 수 없다.
 *   ② 샘플마다 CS를 내리고 올려 오버헤드가 샘플 수에 비례해 붙는다.
 *
 * FIFO를 쓰면 샘플 타이밍은 센서 자체 클럭이 찍고(지터 없음), 펌웨어는 그냥
 * 쌓인 만큼 한 번의 CS 구간에서 연속으로 빨아들이면 된다. 폴링 주기가 ODR과
 * 무관해지므로 6664Hz까지 같은 코드로 올라간다.
 *
 * RTC 틱 = 32768 / (APP_TIMER_CONFIG_RTC_FREQUENCY + 1) = 16384 Hz. */
#define IMU_INIT_ATTEMPTS        10
#define IMU_DEFAULT_ODR_HZ       416U
#define APP_TIMER_CLOCK_HZ       16384U

/* 배수 시간(=한 번에 묶이는 샘플 수)과 메인 루프 블로킹 시간의 절충.
 * 16틱 = 0.98ms. 최고 레이트(6664Hz)에서도 한 번에 6~7샘플뿐이라 비트뱅잉이
 * 메인 루프를 잡아두는 시간이 0.1ms 수준이고, BLE 연결 이벤트(7.5ms)마다
 * 송신 큐를 채울 기회가 충분히 남는다. */
#define IMU_DRAIN_PERIOD_TICKS   16U
/* 한 번의 배수에서 읽을 최대 샘플 수. BLE가 밀려 메인 루프가 늦어졌을 때
 * 따라잡되, 한 번에 너무 오래 붙잡지는 않도록 상한을 둔다.
 * 24샘플 = 288바이트 ≈ 0.85ms(비트뱅 ~2.7MHz). */
#define IMU_DRAIN_MAX_SAMPLES    24U

/* V2S200D PDM microphone. 1.280MHz / ratio 80 gives exactly 16,000 PCM samples/s. */
#define MIC_CLK_PIN          31
#define MIC_DIN_PIN          30
#define MIC_SAMPLE_RATE_HZ   16000UL
#define PDM_BUFFER_SAMPLES   256
#define PDM_BUFFER_COUNT     2

/* 링버퍼 크기는 반드시 2의 거듭제곱 (인덱스를 & 마스크로 구하므로).
 *
 * 2026-07-27: 둘 다 4배로 키웠다. 예전 크기(마이크 0.5초 / IMU 0.31초)는
 * "BLE가 연결 이벤트 몇 개를 놓치는 정도"만 견디는 값이다. 그런데 안드로이드는
 * 화면이 꺼지거나 앱이 백그라운드로 가면 연결 간격을 수백 ms로 늘려버리고,
 * 그 구간이 1초를 넘으면 예전 크기로는 무조건 링이 넘쳤다. 장시간 연속 녹음이
 * 목표라면 "잠깐 밀렸다가 따라잡는" 구간을 무손실로 넘겨야 한다.
 *
 * ⚠️ 링커가 RAM 부족으로 실패하면 IMU_RING_SAMPLES를 4096으로 먼저 내릴 것.
 *    (IMU 링이 96KB로 제일 크다. 마이크 링은 33KB.) */
#define MIC_CHUNK_COUNT      64U   /**< 청크 64개 x 256샘플 = 약 1.0초분, 33KB */
/* 96KB = 416Hz에서 19.7초, 6664Hz에서도 1.23초분. */
#define IMU_RING_SAMPLES     8192U

/* --- BLE 패킷 규격 (나중에 만들 Android 앱과 공유하는 와이어 포맷) ---
 *   [0]     매직: 0xA5=마이크, 0x5A=IMU, 0x01=텍스트/상태
 *   [1..4]  이 패킷 첫 샘플의 절대 번호 (uint32 little-endian)  <- 유실 감지의 핵심
 *   [5]     이 패킷에 담긴 샘플 개수 (uint8)
 *   [6..]   페이로드 (마이크: int16 LE PCM, IMU: int16 LE 6축 = 12바이트/샘플)
 *
 *   수신측은 절대 번호가 건너뛰면 그만큼 유실된 것으로 판단한다.
 */
#define PKT_MAGIC_MIC     0xA5
#define PKT_MAGIC_IMU     0x5A
#define PKT_MAGIC_TEXT    0x01
#define PKT_HEADER_BYTES  6
/* ATT MTU 247 - 3 = 244바이트. 이 값이 최적이고 더 키우면 손해다.
 *
 * 2026-07-22에 517로 키워봤다(패킷 수가 병목이니 패킷을 키우자는 발상).
 * 결과: ① SoftDevice RAM이 모자라 광고가 아예 안 떴고, ② 근본적으로 BLE
 * 링크계층 PDU가 251바이트 상한이라 514바이트 알림은 LL 패킷 3개(251+251+19)로
 * 쪼개진다. 마지막 19바이트가 슬롯 하나를 통째로 낭비해 오히려 느려진다.
 * 244(=251-7, ATT+L2CAP 헤더 제외)가 LL 패킷 하나를 정확히 채우는 값이다. */
#define PKT_MAX_BYTES     244

/* --- 압축 와이어 포맷 (프로파일 1 이상에서만 나간다) ---------------------
 *
 * 왜 필요한가: 마이크 16kHz(269kbps) + IMU 6664Hz(660kbps) = 929kbps인데
 * 폰 실측 링크 상한이 약 600kbps다. 간격·큐·PHY 튜닝은 §6.6.5에서 이미 다
 * 소진했으므로 남은 수단은 와이어에 나가는 바이트를 줄이는 것뿐이다.
 *
 * 두 가지 코덱을 넣고 프로파일로 조합한다. 어느 쪽도 기본값이 아니므로
 * 프로파일 0이면 예전 포맷 그대로 나간다(앱 호환).
 *
 *   0xA6 = 마이크 IMA ADPCM 4비트   256kbps -> 64kbps
 *   0x5B = IMU 12비트 팩            12B/샘플 -> 9B(6축) 또는 4.5B(가속도만)
 *
 * 새 포맷은 count가 uint16이다(ADPCM은 한 패킷에 256샘플이 들어가 uint8을 넘음).
 *
 *   [0]     매직
 *   [1..4]  첫 샘플의 절대 번호 (uint32 LE)
 *   [5..6]  샘플 개수 (uint16 LE)
 *   [7]     보조 바이트 (IMU=포맷, 마이크=ADPCM step index)
 *   [8..]   페이로드
 */
#define PKT_MAGIC_MIC_ADPCM 0xA6
#define PKT_MAGIC_MIC_ULAW  0xA7
#define PKT_MAGIC_IMU_PACK  0x5B
#define PKT_HEADER2_BYTES   8
/* ADPCM은 헤더2 뒤에 디코더 시작 상태(predictor int16)를 더 싣는다.
 * 매 패킷이 자기 상태를 들고 있어야 앞 패킷이 유실돼도 이어서 디코딩된다. */
#define PKT_ADPCM_HDR_BYTES 10

#define IMU_PACK_FMT_6AXIS  0   /**< ax,ay,az,gx,gy,gz 각 12비트 = 9B/샘플 */
#define IMU_PACK_FMT_ACCEL  1   /**< ax,ay,az 각 12비트 = 4.5B/샘플 */
/* 한 패킷에 담기는 12비트 값의 최대 개수. (244-8)*8/12 = 157 -> 6과 3 모두로
 * 나누어떨어지는 156으로 맞춘다. 6축이면 26샘플, 가속도만이면 52샘플. */
#define IMU_PACK_MAX_VALS   156

/* 스트리밍 프로파일. 0xA2 명령으로 고른다.
 *
 * 마이크 16kHz + IMU 6664Hz 기준 필요 대역폭(헤더 포함 근사):
 * 폰 실측(갤럭시 A15) 결과. 링크 상한은 약 700~770 kbps다:
 *   0 RAW          771 kbps  포화. IMU 25.7% 유실
 *   1 MIC_ADPCM    (미측정)  마이크만 압축
 *   2 IMU_PACK     695 kbps  포화. IMU 16.4% 유실
 *   3 BOTH         594 kbps  <- 양쪽 유실 0.00%
 *   4 BOTH_ACCEL   367 kbps  자이로를 버린 안전판. 유실 0.00%
 *   5 BOTH_MIC8    약 660 kbps  마이크를 8비트 u-law로. 오차 10% -> 약 1%
 */
#define STREAM_PROFILE_RAW        0
#define STREAM_PROFILE_MIC_ADPCM  1
#define STREAM_PROFILE_IMU_PACK   2
#define STREAM_PROFILE_BOTH       3
#define STREAM_PROFILE_BOTH_ACCEL 4
/* 5 = 마이크를 8비트 u-law로. 4비트 ADPCM보다 오차가 훨씬 작고(약 10% -> 약 1%)
 * 대역폭은 약 66 kbps만 더 든다. 마이크는 전체의 11%뿐이라 여기에 비트를 더
 * 주는 게 싸다. 대신 총합이 약 660 kbps로 실측 상한(약 700~770)에 가까워지므로
 * 여유가 줄어든다 — 폰이나 거리가 바뀌면 프로파일 3으로 내려올 것. */
#define STREAM_PROFILE_BOTH_MIC8  5
#define STREAM_PROFILE_MAX        5

/**@brief 프로파일이 마이크에 쓰는 코덱. */
#define MIC_CODEC_PCM16   0
#define MIC_CODEC_ADPCM4  1
#define MIC_CODEC_ULAW8   2

static uint8_t mic_codec_for(uint8_t profile)
{
    switch (profile)
    {
        case STREAM_PROFILE_MIC_ADPCM:
        case STREAM_PROFILE_BOTH:
        case STREAM_PROFILE_BOTH_ACCEL: return MIC_CODEC_ADPCM4;
        case STREAM_PROFILE_BOTH_MIC8:  return MIC_CODEC_ULAW8;
        default:                        return MIC_CODEC_PCM16;
    }
}

#define CMD_SET_PROFILE   0xA2

/** IMU 한 샘플: 가속도 3축 + 자이로 3축 (각 16비트 원시값) */
typedef struct { int16_t ax, ay, az, gx, gy, gz; } imu_sample_t;

/** 마이크 청크: PDM 버퍼 한 덩어리 + 그 첫 샘플의 절대 번호.
 *  청크마다 번호를 들고 있어서, 버려진 청크가 있어도 남은 데이터의
 *  위치가 흐트러지지 않는다. 다운샘플링하면 count가 256보다 작아진다. */
typedef struct {
    uint32_t index;
    uint16_t count;
    int16_t  samples[PDM_BUFFER_SAMPLES];
} mic_chunk_t;

/* 마이크 샘플레이트: PDM 하드웨어는 16000Hz 고정이고, 정수배로 나눠서 낮춘다.
 * 앱의 SamplingRate(16k/8k/4k/2k/1k)와 대응. 0xA0 명령으로 바꾼다. */
#define MIC_BASE_RATE_HZ  MIC_SAMPLE_RATE_HZ
#define CMD_SET_RATE      0xA0
#define CMD_SET_IMU_RATE  0xA1
#define CMD_SET_LATENCY   0xA3

/** 앱이 보내는 kHz 값 → 데시메이션 계수. 0이면 지원 안 하는 값. */
static uint16_t mic_decim_from_khz(uint8_t khz)
{
    switch (khz)
    {
        case 16: return 1;
        case 8:  return 2;
        case 4:  return 4;
        case 2:  return 8;
        case 1:  return 16;
        default: return 0;
    }
}

APP_TIMER_DEF(m_sample_timer);                    /**< 샘플링용 반복 타이머 */
static volatile bool m_recording    = false;      /**< 녹화 중 여부 */
static volatile bool m_scan_request = false;      /**< 메인 루프에 센서 스캔 요청 */
static volatile bool m_sample_due   = false;      /**< 타이머가 세팅→메인루프가 FIFO 배수 */
static volatile bool m_stop_request = false;      /**< 'e' 수신 → 메인 루프가 마무리 */
static volatile bool m_stream_start_request = false;/**< 's' 수신 → 메인 루프가 두 스트림을 함께 시작 */
static volatile uint16_t m_imu_rate_request = 0;  /**< 0xA1 수신 → 메인 루프가 ODR 변경 */
static volatile uint8_t  m_mic_rate_request = 0;  /**< 0xA0 수신 → 메인 루프가 데시메이션 변경 (kHz, 0=요청 없음) */

/* --- 명령 처리 규약: 인터럽트에서는 플래그만, 실제 일은 메인 루프에서 ---
 *
 * nus_data_handler()는 인터럽트 컨텍스트에서 돈다(sdk_config의
 * NRF_SDH_DISPATCH_MODEL = 0 = INTERRUPT). 즉 메인 루프가 무엇을 하고 있든
 * 그 사이를 끊고 들어올 수 있다. 그래서 이 핸들러에서 해서는 안 되는 일이 둘 있다:
 *
 *   ① 레지스터 비트뱅잉 — SoftDevice 타이밍이 깨진다.
 *   ② 링버퍼 인덱스 조작 — CRITICAL_REGION으로도 못 막는다. 임계구역은 인터럽트를
 *      막을 뿐, 이미 중간에 끊긴 메인 루프를 되돌려주지 않는다. 메인 루프가
 *      stream_send_mic_packet() 안에서 전송까지 끝내고 m_mic_tail++ 하기 직전에
 *      끊기면, 인덱스를 0으로 민 뒤 돌아온 메인이 tail을 1로 올려 tail > head가 된다.
 *      m_mic_head - m_mic_tail은 부호 없는 뺄셈이라 0 - 1 = 4294967295가 되고,
 *      PDM ISR의 "링에 자리 있나" 검사가 영영 거짓이 되어 그 세션 마이크가 통째로
 *      죽는다. 조용히 죽는다 — 에러도 안 나고 STAT의 mic_dropped만 치솟는다.
 *
 * 0xA1(IMU 레이트)과 0xA2(프로파일)는 처음부터 이 규약을 지켰지만 0xA0(마이크
 * 레이트)만 빠져 있었다(2026-07-27 수정). 새 명령을 추가할 때도 같은 모양을 지킬 것:
 * 핸들러는 값 검증과 m_*_request 세팅까지만 한다.
 */

/* --- 두 스트림의 원점 맞추기 ---
 *
 * 마이크와 IMU는 각자 절대 샘플 번호를 0부터 세고, 앱은 index/레이트로 시간을 만든다.
 * 그래서 두 index 0이 같은 순간을 가리켜야 두 CSV를 겹쳐 볼 수 있다.
 *
 * 예전에는 's'를 받은 자리에서 PDM만 바로 켜고 IMU FIFO 리셋은 메인 루프에 미뤘다.
 * 그 사이 지연은 메인 루프가 무슨 일을 하고 있었느냐에 따라 매번 달라서, 두 원점의
 * 차이가 고정값도 아니었다. 이제는 정말로 둘 다 메인 루프 한 곳에서 연달아 시작한다
 * (stream_start_apply(). 2026-07-27까지는 이 주석만 그렇게 말하고 코드는 여전히
 * 마이크를 핸들러에서 켜고 있었다 — 주석을 믿고 SYNC를 걷어내면 안 됐던 이유).
 *
 * 그래도 PDM 기동 지연과 센서의 첫 샘플 대기는 하드웨어 몫으로 남는다. 남은 만큼은
 * 같은 RTC로 양쪽 첫 샘플 시각을 재서 SYNC 줄로 올려보낸다 — 앱이 그 값만큼 밀면
 * 정렬 오차가 RTC 분해능(1/16384초 = 61us)까지 내려간다. */
#define RTC_COUNTER_MASK   0x00FFFFFFUL
#define PDM_BUFFER_TICKS   (((uint32_t)PDM_BUFFER_SAMPLES * APP_TIMER_CLOCK_HZ) / MIC_BASE_RATE_HZ)

static volatile uint32_t m_mic_t0       = 0;      /**< 마이크 샘플 0의 RTC 시각 */
static volatile uint32_t m_imu_t0       = 0;      /**< IMU 샘플 0의 RTC 시각 */
static volatile bool     m_mic_t0_valid = false;
static volatile bool     m_imu_t0_valid = false;
static volatile bool     m_sync_report  = false;  /**< 양쪽 원점이 다 잡힘 → SYNC 줄 전송 */

/* 마이크 청크 링: head는 PDM ISR만, tail은 메인 루프만 건드린다(단일 생산자/소비자). */
static mic_chunk_t       m_mic_chunks[MIC_CHUNK_COUNT];
static volatile uint32_t m_mic_head     = 0;   /**< ISR이 채운 청크 수 */
static uint32_t          m_mic_tail     = 0;   /**< 메인 루프가 보낸 청크 수 */
static uint16_t          m_mic_offset   = 0;   /**< 현재 청크에서 이미 보낸 샘플 수 */
static volatile uint32_t m_mic_produced = 0;   /**< PDM이 만들어낸 절대 샘플 수(데시메이션 후) */
static volatile uint32_t m_mic_dropped  = 0;   /**< 링이 꽉 차서 버린 샘플 수 */
/* m_mic_produced / m_imu_head는 레이트·프로파일이 바뀔 때마다 0으로 되돌아간다
 * (앱이 새 구간을 index 0부터 세야 하므로). 그래서 그것만으로는 세션 전체 생산량을
 * 알 수 없다 — STAT은 "수신측이 자기가 센 것과 대조해 검증"하라고 있는 숫자인데,
 * 중간에 프로파일을 한 번만 바꿔도 변경 이후 구간만 세어 조용히 과소보고했다.
 * 리셋 직전 값을 여기에 누적해 두고 STAT은 (누적 + 현재)를 보고한다. */
static uint32_t          m_mic_total    = 0;   /**< 리셋으로 버려진 이전 구간들의 마이크 생산량 합 */
static uint32_t          m_imu_total    = 0;   /**< 리셋으로 버려진 이전 구간들의 IMU 생산량 합 */
static volatile uint16_t m_mic_decim    = 1;   /**< 1,2,4,8,16 → 16k,8k,4k,2k,1k */
static volatile bool     m_rate_report  = false; /**< 메인 루프가 RATE 줄을 보내야 함 */
static volatile bool     m_link_report  = false; /**< 메인 루프가 LINK 줄을 보내야 함 */
/* 1초마다 나가는 상태 한 줄. 텍스트 한 줄이라 대역폭은 무시할 수준인데,
 * "언제부터 어느 쪽이 밀리기 시작했나"를 사후에 알 수 있는 유일한 근거다.
 * 지금까지 IMU가 죽어도 STAT('e' 이후)에서만 보였고, 앱은 'e' 직후 연결을
 * 끊어서 그 STAT조차 못 받았다. */
static volatile bool     m_health_report = false;
static volatile uint32_t m_health_ticks  = 0;   /**< 배수 틱 카운터. 1024틱 = 1초 */
static volatile uint32_t m_uptime_sec    = 0;   /**< 스트리밍 시작 후 경과 초 */
static bool              m_imu_ok       = false; /**< lsm6_init 성공 여부. 실패 시 IMU가 0만 낸다 */
static uint16_t          m_imu_odr_hz   = IMU_DEFAULT_ODR_HZ;
static uint8_t           m_imu_odr_bits = 0x60;   /**< CTRL1_XL/CTRL2_G의 ODR 필드 (416Hz) */
static uint32_t          m_imu_fifo_ovr = 0;      /**< 센서 FIFO 오버런 횟수. 0이 아니면 배수가 늦은 것 */

/* IMU 링: 생산·소비 모두 메인 루프라 경합이 없다. head가 곧 절대 샘플 번호. */
static imu_sample_t      m_imu_ring[IMU_RING_SAMPLES];
static uint32_t          m_imu_head    = 0;
static uint32_t          m_imu_tail    = 0;
static uint32_t          m_imu_dropped = 0;

/* --- IMU 스톨 감시 (2026-07-27 추가) ---
 *
 * 지금까지 IMU가 죽는 방식은 전부 "조용한 죽음"이었다. lsm6_fifo_drain()은
 * FIFO가 비어 있으면 그냥 return하고, m_imu_ok는 true로 남고, RATE 줄에는
 * imu_ok=1이 계속 나간다. 앱은 마지막 값을 hold해서 0에서 평평한 선을 그린다.
 * 실제로 2026-07-24 폰 녹음 5건이 전부 이 상태였다 — 40초 동안 마이크는
 * 637,952샘플 무손실, IMU는 0샘플. 로그도 에러도 없었다.
 *
 * 그래서 두 가지를 넣는다.
 *   ① 감지: 배수가 연속으로 빈손이면 스톨로 판정한다. 정상이라면 최저 ODR
 *      104Hz에서도 배수 1024Hz 기준 10번에 한 번은 샘플이 나온다.
 *   ② 복구: 먼저 레지스터만 다시 쓰는 소프트 복구(1ms 미만), 그래도 안 되면
 *      전원 레일을 껐다 켜는 하드 복구.
 *
 * ⚠️ 하드 복구를 nrf_delay_ms로 짜면 안 된다. 전원 OFF 100ms + 부팅 50ms에
 *    WHO_AM_I 재시도까지 하면 메인 루프를 최대 650ms 붙잡는데, 그 동안 마이크
 *    링(1초)이 절반 넘게 차고 BLE 송신도 멈춘다. IMU를 살리려다 마이크를
 *    죽이는 셈이다. 그래서 상태기계로 쪼개 메인 루프를 돌려주면서 진행한다. */
#define IMU_STALL_DRAINS       2048U  /**< 배수 1024Hz 기준 약 2초간 빈손이면 스톨 */
#define IMU_SOFT_RECOVER_MAX   2U     /**< 소프트 복구를 이만큼 시도한 뒤 전원 사이클 */
#define IMU_PWR_OFF_TICKS      ((100UL * APP_TIMER_CLOCK_HZ) / 1000UL)  /**< 레일 방전 100ms */
#define IMU_PWR_BOOT_TICKS     (( 60UL * APP_TIMER_CLOCK_HZ) / 1000UL)  /**< 상승+t_boot 60ms */
#define IMU_RETRY_TICKS        (2UL * APP_TIMER_CLOCK_HZ)               /**< 복구 실패 후 재시도 백오프 2초 */

/* --- 지연 상한 (2026-07-27 추가) ---
 *
 * 링을 키운 대가로 생긴 문제를 여기서 막는다. 링크가 수요를 조금이라도 못 따라가면
 * 링은 '꽉 찬 채로 유지'되고, 그때부터 지연 = 링 깊이가 된다(bufferbloat).
 * 링을 샘플 수로 잡아둔 탓에 IMU는 ODR이 낮을수록 시간 깊이가 폭증했다:
 *
 *     IMU 링 8192샘플 = 6664Hz에서 1.2초, 1666Hz에서 4.9초, 416Hz에서 19.7초
 *     마이크 링 64청크 = 항상 1.0초 (청크 하나가 데시메이션과 무관하게 16ms)
 *
 * 그래서 두 증상이 같이 나온다.
 *   - 센서를 쳤는데 화면이 늦게 반응한다  -> 링에 고인 시간
 *   - IMU와 마이크가 어긋나 보인다        -> 두 링에 고인 시간의 '차이'
 *
 * 해결은 링을 시간으로 제한하는 것이다. 두 스트림에 같은 예산을 걸면 지연이 묶이고
 * 어긋남도 예산 이하로 함께 줄어든다. 큰 링은 그대로 두는 것이 맞다 — 안드로이드가
 * 순간적으로 멈추는 구간(보통 100~300ms)은 여전히 무손실로 흡수하고, 상한은
 * '만성적으로 밀릴 때'만 작동해서 오래된 것을 버린다.
 *
 * ⚠️ **이 숫자 하나가 두 가지를 동시에 뜻한다.** 시뮬레이션 두 표가 정확히 대칭이었다:
 *
 *      무손실로 견딜 수 있는 링크 멈춤  =  최대 화면 지연  =  이 값
 *
 * 상한보다 긴 멈춤은 반드시 데이터를 잃는다(6664Hz, 10초마다 멈춤, 링크 여유 130%):
 *
 *      멈춤 →     200ms   600ms   1000ms  1500ms
 *      상한 400ms  100%     98.0%   94.0%   88.9%
 *      상한 1000ms 100%    100.0%  100.0%   94.9%
 *      상한 없음   100%    100.0%  100.0%   97.2%
 *
 * 처음에 400으로 잡았다가 1000으로 올렸다. 400은 흔한 1초짜리 멈춤에서 6%를 버렸는데,
 * 6664Hz 기준 지연 이득은 1.0초 대 1.23초로 거의 없었다 — 링 크기가 어차피 천장이라
 * 그렇다. 반면 낮은 ODR에서는 상한이 시간 기준이라 효과가 그대로 남는다:
 * 링 8192샘플은 1666Hz에서 4.9초, 416Hz에서 19.7초인데 전부 1초로 잘린다.
 * 그 레이트대는 링크 여유가 넉넉해서 잘라내는 실제 손실이 사실상 0이다.
 *
 * 튜닝하려면 HEALTH의 imu_lag_ms 최고치를 먼저 볼 것. 그게 실제 링크 멈춤 길이다.
 * 그 값보다 상한이 크면 무손실이고, 작으면 그만큼 버린다.
 *
 * --- 2026-07-28: 고정값 -> 런타임 값 (0xA3) ---
 *
 * 위의 대칭이 곧 문제였다. 한 숫자가 "무손실로 견딜 멈춤"과 "최대 화면 지연"을
 * 동시에 정하는데, 이 둘은 언제 무엇이 중요한지가 서로 반대다:
 *
 *   - 실시간 모니터링 중: 화면 지연이 전부다. 1초 밀린 파형은 쓸모가 없고,
 *     어차피 화면에 남지 않을 오래된 샘플을 붙들고 있을 이유가 없다.
 *   - CSV 녹음 중: 화면은 부차적이고 파일에 구멍이 나면 안 된다. 안드로이드가
 *     화면을 끄거나 앱을 백그라운드로 보내면 연결 간격이 수백 ms로 늘어나는데,
 *     그 구간은 링크에 여유만 있으면 원래 무손실로 따라잡히던 것이다.
 *
 * 그래서 하나로 고를 수가 없다. 앱이 자기 상태를 알고 있으므로(녹음 중인지
 * 아닌지) 0xA3으로 그때그때 알려주게 했다. 링 크기는 건드리지 않는다 —
 * 녹음 모드의 1000ms를 담으려면 지금 크기가 그대로 필요하다.
 *
 * 부팅 기본값은 MONITOR다. 앱은 연결 직후 항상 스트림을 시작하고, 그 시점은
 * 정의상 아직 녹음 전이다. 반대로 두면(RECORD 기본) 0xA3이 유실됐을 때 증상이
 * "이유 없이 1초 밀린 화면"으로 나타나는데, 그건 지금 고치려는 바로 그 증상이라
 * 원인을 찾기가 제일 어렵다. MONITOR 기본은 실패해도 "녹음에 구멍"으로 나타나고
 * 그건 HEALTH의 itrim/mtrim에 즉시 숫자로 찍힌다. */
#define STREAM_LATENCY_MONITOR_MS 150UL    /**< 실시간 우선. 화면 지연 상한 */
#define STREAM_LATENCY_RECORD_MS  1000UL   /**< 무손실 우선. 예전 고정값과 동일 */
#define STREAM_LATENCY_MIN_MS     50UL     /**< 이보다 짧으면 패킷 몇 개도 못 담는다 */
#define STREAM_LATENCY_MAX_MS     5000UL   /**< 링 크기가 어차피 천장이라 그 이상은 무의미 */
/* 청크 하나가 담는 시간. PDM은 항상 16kHz로 돌고 데시메이션은 출력 개수만 줄이므로
 * 청크의 '시간 길이'는 레이트와 무관하게 고정이다. 256/16000 = 16ms. */
#define MIC_CHUNK_MS              ((1000UL * PDM_BUFFER_SAMPLES) / MIC_SAMPLE_RATE_HZ)

typedef enum
{
    IMU_RCV_NONE = 0,   /**< 정상 동작 중 */
    IMU_RCV_WAIT_OFF,   /**< EN=LOW. 레일이 방전되기를 기다린다 */
    IMU_RCV_WAIT_ON,    /**< EN=HIGH. 센서 부팅을 기다린다 */
} imu_recover_stage_t;

static imu_recover_stage_t m_imu_rcv_stage    = IMU_RCV_NONE;
static uint32_t            m_imu_rcv_deadline = 0;     /**< app_timer 틱 기준 만료 시각 */
static uint32_t            m_imu_idle_drains  = 0;     /**< 연속으로 빈손이었던 배수 횟수 */
static uint8_t             m_imu_soft_tries   = 0;     /**< 이번 스톨에서 시도한 소프트 복구 횟수 */
static uint32_t            m_imu_stall_cnt    = 0;     /**< 세션 중 감지된 스톨 횟수 (HEALTH로 보고) */
static uint32_t            m_imu_recover_cnt  = 0;     /**< 그 중 전원 사이클까지 간 횟수 */
static bool                m_imu_fifo_realign = false; /**< 오버런 감지 → 이번 배수 끝에 FIFO 재정렬 */

/* 지연 상한에 걸려 버린 양. ISR이 만지는 m_mic_dropped와 섞으면 경합이 나므로
 * 메인 루프 전용 카운터를 따로 둔다. HEALTH/STAT에서 합쳐서 보고한다. */
static uint32_t            m_imu_trimmed      = 0;
static uint32_t            m_mic_trimmed      = 0;
/* 'e' 이후 잔량을 끝까지 밀어내는 동안에는 지연 상한을 끈다. 안 그러면 마무리
 * flush가 곧 데이터 폐기가 된다 — 정확히 반대로 동작하는 셈. */
static bool                m_stream_flushing  = false;

/* --- 가중 공정 큐잉(WFQ) 가상 시각 (2026-07-27 추가) ---
 *
 * 예전 stream_service()는 마이크를 다 비운 뒤에만 IMU를 보냈다. 그래서 링크가
 * 조금만 느려져도 부족분이 전부 IMU에서 빠졌다 — 유실률이 0%에서 100%로 계단처럼
 * 튀는 구조이고, "마이크는 완벽한데 IMU만 안 나온다"가 이 시스템의 기본 실패
 * 모드였던 이유다.
 *
 * 대신 두 스트림에 '가상 시각'을 하나씩 두고, 패킷을 보낼 때마다 그 패킷이 담은
 * 스트림 시간만큼 앞으로 민다. 항상 가상 시각이 뒤처진 쪽을 보내면 두 스트림이
 * 자기 레이트에 비례해 대역을 나눠 갖는다(링크가 남으면 둘 다 100%, 모자라면
 * 둘 다 같은 비율로 유실).
 *
 * 나눗셈을 피하려고 상대 스트림의 레이트를 곱해 둔 값을 비용으로 쓴다:
 *     IMU 1패킷 = imu_samples / imu_hz 초   ->  비용 imu_samples * mic_hz
 *     MIC 1패킷 = mic_samples / mic_hz 초   ->  비용 mic_samples * imu_hz
 * 최대값은 256 * 6664 = 1.7e6이라 int32에 넉넉히 들어간다. 매 회 min을 빼서
 * 정규화하므로 장시간 돌려도 누적 오버플로가 없다.
 *
 * ⚠️ 공평성을 "호출 횟수"로 걸면 안 된다. 2026-07-21에 "호출마다 IMU 1패킷"을
 *    넣었다가 마이크가 89% 죽었다. stream_service()는 초당 1000번 넘게 불리므로
 *    그것만으로 IMU 필요량이 다 채워지고 마이크 차례가 오기 전에 큐가 찬다.
 *    WFQ는 호출 횟수와 무관하게 "보낸 패킷이 담은 시간"으로만 판단한다. */
static int32_t           m_vt_imu = 0;
static int32_t           m_vt_mic = 0;

/* 스트리밍 프로파일. 실제 적용은 메인 루프에서(링 리셋이 따라붙으므로). */
static volatile uint8_t  m_profile         = STREAM_PROFILE_RAW;
static volatile uint8_t  m_profile_request = 0xFF;  /**< 0xFF = 요청 없음 */

/* 지연 예산. 0xA3으로 앱이 바꾼다. 위 STREAM_LATENCY_* 설명 참고.
 * 읽는 쪽(stream_trim_backlog)도 쓰는 쪽(메인 루프)도 전부 thread 모드라
 * 경합이 없지만, 요청은 BLE 이벤트 컨텍스트에서 들어오므로 규약대로 분리한다. */
static uint32_t          m_latency_budget_ms = STREAM_LATENCY_MONITOR_MS;
static volatile uint16_t m_latency_request   = 0;   /**< 0 = 요청 없음 */

/* IMA ADPCM 인코더 상태. 마이크 스트림 전체에 걸쳐 연속으로 굴러가고,
 * 패킷마다 "그 패킷 시작 시점의 상태"를 같이 실어 보낸다. 그래야 앞 패킷이
 * 유실돼도 수신측이 그 패킷부터 바로 정확하게 디코딩할 수 있다. */
static int16_t           m_adpcm_pred = 0;
static uint8_t           m_adpcm_idx  = 0;

static int16_t           m_pdm_buf[PDM_BUFFER_COUNT][PDM_BUFFER_SAMPLES];
static volatile uint8_t  m_pdm_buf_next  = 0;
static volatile bool     m_mic_recording = false;
static volatile bool     m_mic_error     = false;

static bool lsm6_set_rate(uint16_t odr_hz);
static void lsm6_fifo_start(void);
static void lsm6_fifo_stop(void);
static void lsm6_fifo_drain(void);


/**@brief 두 스트림의 링·절대번호·원점을 한꺼번에 처음으로 되돌린다.
 *
 * @details 마이크 레이트(0xA0), IMU 레이트(0xA1), 프로파일(0xA2) — 셋 중 무엇이
 *          바뀌어도 동시에 무효가 되는 것이 세 가지 있다:
 *
 *            ① 링에 남아 있는 이전 포맷/레이트 샘플. 뒤에 섞이면 앱이 한 구간을
 *               두 가지 규칙으로 해석한다.
 *            ② 절대 샘플 번호. 앱은 index/레이트로 시간을 만들므로, 레이트가
 *               바뀌면 같은 index가 다른 시각을 뜻하게 된다.
 *            ③ 두 스트림의 원점(t0)과 이미 보낸 SYNC 값.
 *
 *          ③이 특히 조용하다. 예전 0xA0은 ①②를 마이크에만 적용하고 IMU와 t0는
 *          건드리지 않았다. 그러면 마이크는 0번부터 다시 세는데 IMU는 계속 세고,
 *          앱은 옛날 원점을 가리키는 SYNC로 둘을 겹친다. 에러도 안 나고 그래프도
 *          멀쩡해 보이는데 시간축만 어긋난다. 세 경로가 같은 함수를 부르게 해서
 *          다시 갈라지지 않게 한다.
 *
 * @warning 메인 루프에서만 부를 것. 링 인덱스를 만지므로 인터럽트에서 부르면
 *          위 "명령 처리 규약" 주석의 ②에 그대로 걸린다.
 */
static void stream_reset_all(void)
{
    /* 버려지는 구간의 생산량은 STAT을 위해 누적해 둔다. */
    m_mic_total += m_mic_produced;
    m_imu_total += m_imu_head;

    /* 마이크 쪽은 PDM ISR과 공유하므로 임계구역 안에서. 메인 루프에서 부르는 한
     * ISR이 중간에 끼어드는 경우만 막으면 되고, 그건 CRITICAL_REGION으로 충분하다. */
    CRITICAL_REGION_ENTER();
    m_mic_head     = 0;
    m_mic_tail     = 0;
    m_mic_offset   = 0;
    m_mic_produced = 0;
    m_mic_dropped  = 0;
    CRITICAL_REGION_EXIT();

    /* ADPCM 인코더는 스트림마다 처음부터. 수신측은 패킷 헤더에 실린 상태를 그대로
     * 받아 쓰므로 값 자체가 틀리진 않지만, 앞 구간의 step index가 남아 있으면
     * 새 구간 초반 몇 샘플이 필요 이상으로 거칠어진다. */
    m_adpcm_pred = 0;
    m_adpcm_idx  = 0;

    /* IMU 링은 생산·소비 모두 메인 루프라 경합이 없다. */
    m_imu_head    = 0;
    m_imu_tail    = 0;
    m_imu_dropped = 0;

    /* 크레딧과 스톨 카운터도 같이 되돌린다. 링을 비워놓고 크레딧만 남기면
     * 다음 stream_service()가 있지도 않은 샘플을 보내려다 헛돌고, 스톨 카운터를
     * 남기면 리셋 직후 정상적인 "아직 샘플 없음" 구간을 스톨로 오판한다. */
    m_vt_imu           = 0;
    m_vt_mic           = 0;
    m_imu_idle_drains  = 0;
    m_imu_soft_tries   = 0;
    m_imu_fifo_realign = false;

    /* IMU 링 번호를 0으로 되돌리면 센서 FIFO도 같이 비워야 한다. FIFO에 남아 있던
     * 워드는 이전 구간의 것이고, 패턴 위상이 샘플 경계와 어긋난 채로 배수가 이어지면
     * 축이 통째로 밀린다 — 그럴듯해 보여서 안 들키는 종류의 오류다(§8 트러블슈팅). */
    if (m_recording && m_imu_ok)
    {
        lsm6_fifo_start();
    }
    m_sample_due = false;

    /* 원점도 다시 재야 SYNC 값이 새 구간을 가리킨다. */
    m_mic_t0_valid = false;
    m_imu_t0_valid = false;
    m_sync_report  = false;
}

/* 실제로 협상된 링크 파라미터. 우리가 요청한 값과 상대가 받아준 값은 다를 수
 * 있어서(특히 Windows), 처리량이 안 나올 때 원인을 가르는 근거가 된다. */
static volatile uint16_t m_conn_interval_units = 0;   /**< 1.25ms 단위 */
static volatile uint8_t  m_phy_tx              = 0;   /**< 1=1M, 2=2M */
static volatile uint8_t  m_phy_rx              = 0;
static volatile uint16_t m_tx_octets           = 0;   /**< DLE 결과. 251이어야 정상 */
static volatile uint16_t m_tx_time_us          = 0;

static void ble_send_bytes(uint8_t const *p_data, uint16_t len);
static void ble_send_str(const char *s);
static void advertising_start(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief 배수 타이머 콜백 - 센서 FIFO를 비울 때가 됐다고 알린다.
 *
 * @details ISR 컨텍스트에서는 비트뱅잉(블로킹)을 하지 않는다. 플래그만 세팅하고
 *          실제 FIFO 읽기는 메인 루프에서 수행한다(SoftDevice 타이밍 보호).
 *          이 주기는 IMU 레이트와 무관하다. 샘플 타이밍은 센서가 찍는다.
 */
static void sample_timer_handler(void * p_context)
{
    if (m_recording)
    {
        m_sample_due = true;

        /* 1초마다 HEALTH 한 줄. ISR에서는 플래그만 세운다. */
        if (++m_health_ticks >= (APP_TIMER_CLOCK_HZ / IMU_DRAIN_PERIOD_TICKS))
        {
            m_health_ticks  = 0;
            m_uptime_sec++;
            m_health_report = true;
        }
    }
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // 샘플링용 반복 타이머 생성
    err_code = app_timer_create(&m_sample_timer, APP_TIMER_MODE_REPEATED, sample_timer_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void pdm_event_handler(nrfx_pdm_evt_t const * const p_evt)
{
    if (p_evt->buffer_requested && m_mic_recording)
    {
        int16_t *p_buffer = m_pdm_buf[m_pdm_buf_next];
        m_pdm_buf_next = (uint8_t)((m_pdm_buf_next + 1) % PDM_BUFFER_COUNT);
        if (nrfx_pdm_buffer_set(p_buffer, PDM_BUFFER_SAMPLES) != NRFX_SUCCESS)
        {
            m_mic_error = true;
        }
    }

    if ((p_evt->buffer_released != NULL) && m_mic_recording)
    {
        /* 첫 버퍼가 나온 순간이 마이크 스트림의 원점을 알려준다. 이 이벤트는 버퍼가
         * 다 찼을 때 오므로, 샘플 0은 버퍼 길이만큼 앞이다. */
        if (!m_mic_t0_valid)
        {
            m_mic_t0       = (app_timer_cnt_get() - PDM_BUFFER_TICKS) & RTC_COUNTER_MASK;
            m_mic_t0_valid = true;
            if (m_imu_t0_valid) m_sync_report = true;
        }

        /* 링에 자리가 있으면 청크를 통째로 넣는다. 자리가 없으면(=전송이 밀려서
         * 소비가 못 따라옴) 이번 청크는 버린다. 버려도 m_mic_produced는 계속
         * 증가하므로, 다음 청크의 절대 번호에 구멍이 생겨 수신측이 유실을 안다. */
        /* 256은 1/2/4/8/16 어느 것으로도 나누어떨어져서 버퍼 경계에 잔여가 안 생긴다. */
        uint16_t decim     = m_mic_decim;
        uint16_t out_count = (uint16_t)(PDM_BUFFER_SAMPLES / decim);

        if ((m_mic_head - m_mic_tail) < MIC_CHUNK_COUNT)
        {
            mic_chunk_t   *p_chunk = &m_mic_chunks[m_mic_head & (MIC_CHUNK_COUNT - 1U)];
            int16_t const *p_src   = (int16_t const *)p_evt->buffer_released;

            p_chunk->index = m_mic_produced;
            p_chunk->count = out_count;

            if (decim == 1)
            {
                memcpy(p_chunk->samples, p_src, sizeof(p_chunk->samples));
            }
            else
            {
                /* 박스 평균. 그냥 솎아내면 에일리어싱으로 없던 저주파가 생긴다. */
                for (uint16_t i = 0; i < out_count; i++)
                {
                    int32_t sum = 0;
                    for (uint16_t j = 0; j < decim; j++)
                    {
                        sum += p_src[i * decim + j];
                    }
                    p_chunk->samples[i] = (int16_t)(sum / (int32_t)decim);
                }
            }
            m_mic_head++;
        }
        else
        {
            m_mic_dropped += out_count;
        }

        m_mic_produced += out_count;
    }

    if (p_evt->error != NRFX_PDM_NO_ERROR)
    {
        m_mic_error = true;
    }
}

static void mic_init(void)
{
    nrfx_pdm_config_t config = NRFX_PDM_DEFAULT_CONFIG(MIC_CLK_PIN, MIC_DIN_PIN);
    config.mode       = NRF_PDM_MODE_MONO;
    config.edge       = NRF_PDM_EDGE_LEFTFALLING;
    /* This SDK's nrf_pdm_freq_t predates the nRF52840's 1.280MHz enum, although
     * its device header contains the official register value. The PDM remains
     * disabled throughout init, so both clock and ratio are set before START. */
    config.clock_freq = (nrf_pdm_freq_t)PDM_PDMCLKCTRL_FREQ_1280K;

    APP_ERROR_CHECK(nrfx_pdm_init(&config, pdm_event_handler));
    NRF_PDM->RATIO = PDM_RATIO_RATIO_Ratio80;
    NRF_LOG_INFO("PDM microphone initialized (CLK=P0.31 DIN=P0.30, 1280k/80=16000 Hz)");
}

/* PDM start/stop 실패는 보드를 죽이지 않는다.
 *
 * 예전에는 둘 다 APP_ERROR_CHECK로 감싸 있었다. nrfx_pdm은 stop이 비동기라
 * STOPPING 상태에서 start를 부르면 NRFX_ERROR_BUSY를 돌려주는데, 그게 곧바로
 * 보드 리셋이 됐다. 즉 앱이 'e' 직후 's'를 조금 빨리 보내기만 해도 장치가 죽었다.
 *
 * 이제 호출 경로에 중복 가드가 있어서(m_recording 검사) 정상적으로는 여기 도달하지
 * 않지만, 그래도 만에 하나 실패하면 m_mic_error만 세우고 계속 간다. 그 플래그는
 * STAT의 mic_err로 나가므로 "마이크가 왜 안 오지"를 로그 없이도 가릴 수 있다.
 * 리셋은 원인을 지워버리기 때문에 오히려 디버깅이 어려웠다. */
#define MIC_START_RETRY   10   /**< STOPPING이 풀리기를 기다리는 최대 횟수(1ms 간격) */

static void mic_record_start(void)
{
    m_pdm_buf_next  = 0;
    m_mic_error     = false;
    m_mic_recording = true;

    /* stop은 비동기다: STOP 태스크를 걸어도 STOPPED 이벤트가 올 때까지 드라이버가
     * STOPPING에 머물고, 그 동안 start는 BUSY를 돌려준다. 'e' 직후 's'가 오면 딱
     * 그 창에 걸린다. 세션 시작에서 한 번뿐인 경로라 최대 10ms 기다려주는 편이
     * 마이크가 통째로 빠진 녹음을 하나 얻는 것보다 싸다. */
    nrfx_err_t err = NRFX_SUCCESS;
    for (uint8_t i = 0; i < MIC_START_RETRY; i++)
    {
        err = nrfx_pdm_start();
        if (err != NRFX_ERROR_BUSY)
        {
            break;
        }
        nrf_delay_ms(1);
    }

    if (err != NRFX_SUCCESS)
    {
        m_mic_recording = false;
        m_mic_error     = true;
        NRF_LOG_ERROR("nrfx_pdm_start failed: 0x%08x", (unsigned)err);
    }
}

static void mic_record_stop(void)
{
    if (!m_mic_recording)
    {
        return;
    }
    m_mic_recording = false;

    nrfx_err_t err = nrfx_pdm_stop();
    if (err != NRFX_SUCCESS)
    {
        m_mic_error = true;
        NRF_LOG_ERROR("nrfx_pdm_stop failed: 0x%08x", (unsigned)err);
    }
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        // PC가 보낸 첫 바이트를 명령으로 해석한다.
        uint8_t cmd = p_evt->params.rx_data.p_data[0];

        if (cmd == 's' || cmd == 'S')          // ---- 스트리밍 시작 ----
        {
            /* 이미 돌고 있으면 무시한다. 예전에는 가드가 없어서 's'를 두 번 보내면
             * 이미 START된 PDM에 다시 start를 걸었고, 그 오류가 보드 리셋이 됐다. */
            if (m_recording || m_stream_start_request)
            {
                NRF_LOG_WARNING("=> 's' ignored: already streaming");
            }
            else
            {
                /* 링·원점 리셋도, PDM/FIFO 기동도 전부 메인 루프에서 한다.
                 * 여기서 마이크만 먼저 켜면 두 스트림의 원점 차이가 매번 달라진다. */
                m_stream_start_request = true;
                NRF_LOG_INFO("=> Streaming START requested");
            }
        }
        else if (cmd == 'e' || cmd == 'E')     // ---- 스트리밍 정지 ----
        {
            if (!m_recording)
            {
                NRF_LOG_WARNING("=> 'e' ignored: not streaming");
            }
            else
            {
                /* m_recording만 즉시 내려 샘플 타이머가 더 이상 배수를 예약하지
                 * 않게 하고, 나머지(타이머 정지, PDM 정지, 잔량 flush, 요약)는
                 * 전부 메인 루프에서 한다. */
                m_recording    = false;
                m_stop_request = true;
                NRF_LOG_INFO("=> Streaming STOP requested. Will flush remainder.");
            }
        }
        else if (cmd == 'w' || cmd == 'W')     // ---- 센서 WHO_AM_I 스캔 ----
        {
            m_scan_request = true;             // 실제 스캔/전송은 메인 루프에서 처리
            NRF_LOG_INFO("=> Sensor scan requested. Will report over BLE.");
        }
        else if (cmd == CMD_SET_RATE)          // ---- 마이크 샘플레이트 변경: 0xA0 <kHz> ----
        {
            if (p_evt->params.rx_data.length >= 2)
            {
                /* 링 인덱스를 여기서 건드리면 안 된다 — 위 "명령 처리 규약" ② 참고.
                 * 0xA1/0xA2와 똑같이 요청만 걸어두고 메인 루프가 처리한다.
                 * 지원하지 않는 값이면 메인 루프가 조용히 무시하고 현재 레이트를
                 * RATE 줄로 다시 알린다. */
                m_mic_rate_request = p_evt->params.rx_data.p_data[1];
                NRF_LOG_INFO("=> Mic rate requested: %u kHz", m_mic_rate_request);
            }
        }
        else if (cmd == CMD_SET_IMU_RATE)      // ---- IMU ODR 변경: 0xA1 <Hz uint16 LE> ----
        {
            if (p_evt->params.rx_data.length >= 3)
            {
                /* 실제 레지스터 조작(비트뱅잉)은 BLE 이벤트 컨텍스트에서 하면 안 된다.
                 * 요청만 걸어두고 메인 루프가 처리한다. 지원하지 않는 값이면
                 * 메인 루프가 조용히 무시하고 현재 레이트를 RATE 줄로 다시 알린다. */
                m_imu_rate_request = (uint16_t)p_evt->params.rx_data.p_data[1] |
                                     ((uint16_t)p_evt->params.rx_data.p_data[2] << 8);
                NRF_LOG_INFO("=> IMU rate requested: %u Hz", m_imu_rate_request);
            }
        }
        else if (cmd == CMD_SET_PROFILE)       // ---- 스트리밍 프로파일 변경: 0xA2 <profile> ----
        {
            if (p_evt->params.rx_data.length >= 2)
            {
                /* 링 리셋이 따라붙으므로 실제 적용은 메인 루프에서 한다.
                 * 지원하지 않는 값은 메인 루프가 무시하고 현재 값을 다시 알린다. */
                m_profile_request = p_evt->params.rx_data.p_data[1];
                NRF_LOG_INFO("=> Profile requested: %u", m_profile_request);
            }
        }
        else if (cmd == CMD_SET_LATENCY)       // ---- 지연 예산 변경: 0xA3 <ms uint16 LE> ----
        {
            if (p_evt->params.rx_data.length >= 3)
            {
                /* 요청만 걸어둔다. 예산 자체는 링을 안 건드리지만, 이 값을 읽는
                 * stream_trim_backlog()는 tail을 미는 함수라 같은 규약을 지킨다.
                 * 0은 "요청 없음"이므로 앱이 0을 보내도 무시된다(범위 검사에서도 걸림). */
                m_latency_request = (uint16_t)p_evt->params.rx_data.p_data[1] |
                                    ((uint16_t)p_evt->params.rx_data.p_data[2] << 8);
                NRF_LOG_INFO("=> Latency budget requested: %u ms", m_latency_request);
            }
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        /* 예전에는 여기서 연결을 끊었다. 하지만 우리가 7.5ms를 못박아 요청하게 된
         * 뒤로는 상대가 거절할 수 있고, 그때 끊어버리면 아예 못 쓰게 된다.
         * 거절당하면 상대가 정한 간격으로 그냥 계속 쓴다 — 처리량이 떨어질 뿐
         * 데이터는 정상이다. 실제 간격은 LINK 줄에 찍히므로 나중에 확인할 수 있다. */
        m_link_report = true;
        NRF_LOG_WARNING("Conn param negotiation failed; keeping peer's interval.");
    }
    (void)err_code;
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            /* 예제 기본 동작은 sleep_mode_enter()로 보드를 꺼버린다. 그러면 폰
             * 앱을 켤 때마다 보드를 리셋해야 해서 못 쓴다. 대신 다시 광고한다. */
            advertising_start();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            /* 연결 시점의 초기 간격. 이후 협상되면 CONN_PARAM_UPDATE가 덮어쓴다. */
            m_conn_interval_units =
                p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval;

            /* 2M PHY를 먼저 요청한다(같은 시간에 2배를 실어보냄). 상대가 1M만
             * 지원하면 거절당하고 1M으로 남을 뿐이라 실패해도 문제없다. */
            {
                ble_gap_phys_t const phys =
                {
                    .rx_phys = BLE_GAP_PHY_2MBPS,
                    .tx_phys = BLE_GAP_PHY_2MBPS,
                };
                (void)sd_ble_gap_phy_update(m_conn_handle, &phys);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        /* 아래 두 이벤트는 "상대가 실제로 뭘 받아줬는지"를 기록한다.
         * 처리량 진단의 핵심 근거다. 정지 시 요약에도 실어 보내지만, 앱은 'e'를
         * 보내자마자 연결을 끊어서 그 요약을 못 받는다. 그래서 협상이 끝나는
         * 이 시점에 스스로 LINK 줄을 한 번 보낸다(텍스트 한 줄이라 처리량 영향 없음). */
        case BLE_GAP_EVT_PHY_UPDATE:
            m_phy_tx = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
            m_phy_rx = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
            m_link_report = true;
            NRF_LOG_INFO("PHY updated: tx=%d rx=%d", m_phy_tx, m_phy_rx);
            break;

        /* Data Length Extension 결과. 이게 251이 아니면 244바이트 알림 하나가
         * 링크계층 패킷 여러 개로 쪼개져 처리량이 통째로 깎인다. 예를 들어
         * 27바이트로 남으면 알림 하나가 LL 패킷 10개가 되어 10배 손해다.
         * "패킷/초가 상한인가 바이트/초가 상한인가"를 가르는 근거이므로
         * 반드시 눈으로 확인할 것. */
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
            m_tx_octets = p_ble_evt->evt.gap_evt.params.data_length_update
                              .effective_params.max_tx_octets;
            m_tx_time_us = p_ble_evt->evt.gap_evt.params.data_length_update
                              .effective_params.max_tx_time_us;
            m_link_report = true;
            NRF_LOG_INFO("DLE: tx_octets=%d tx_time=%d", m_tx_octets, m_tx_time_us);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            m_conn_interval_units =
                p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
            m_link_report = true;
            NRF_LOG_INFO("Conn interval: %d units", m_conn_interval_units);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    /* 알림 송신 큐를 깊게 잡는다. SoftDevice 기본값은 1(BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT)
     * 이고 SDK 기본 설정은 이 항목을 건드리지 않는다. 큐가 1이면 한 패킷을 보내고
     * TX_COMPLETE를 기다려야 다음을 넣을 수 있어서, 메인 루프가 조금만 늦어도
     * (예: IMU 비트뱅잉 읽기) 그 연결 이벤트를 통째로 놓친다.
     * 깊게 잡으면 SoftDevice가 한 이벤트에서 여러 패킷을 알아서 내보낸다. */
    ble_cfg_t gatts_cfg;
    memset(&gatts_cfg, 0, sizeof(gatts_cfg));
    gatts_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
    gatts_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = HVN_TX_QUEUE_SIZE;

    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &gatts_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            // 부팅 시 UART RX 라인의 프레이밍/브레이크 에러로 앱이 죽는 것을 방지.
            // 우리는 UART가 아니라 BLE만 사용하므로, 통신 에러는 무시하고 로그만 남긴다.
            NRF_LOG_WARNING("UART comm error 0x%x (ignored)", p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief 패킷 헤더 6바이트를 채운다. (매직 + 절대 샘플번호 LE + 개수) */
static void pkt_write_header(uint8_t *p_pkt, uint8_t magic, uint32_t index, uint8_t count)
{
    p_pkt[0] = magic;
    p_pkt[1] = (uint8_t)(index         & 0xFFU);
    p_pkt[2] = (uint8_t)((index >>  8) & 0xFFU);
    p_pkt[3] = (uint8_t)((index >> 16) & 0xFFU);
    p_pkt[4] = (uint8_t)((index >> 24) & 0xFFU);
    p_pkt[5] = count;
}


/**@brief 새 포맷(0xA6/0x5B)용 8바이트 헤더. count가 uint16인 것만 다르다. */
static void pkt_write_header2(uint8_t *p_pkt, uint8_t magic, uint32_t index,
                              uint16_t count, uint8_t aux)
{
    p_pkt[0] = magic;
    p_pkt[1] = (uint8_t)(index         & 0xFFU);
    p_pkt[2] = (uint8_t)((index >>  8) & 0xFFU);
    p_pkt[3] = (uint8_t)((index >> 16) & 0xFFU);
    p_pkt[4] = (uint8_t)((index >> 24) & 0xFFU);
    p_pkt[5] = (uint8_t)(count        & 0xFFU);
    p_pkt[6] = (uint8_t)((count >> 8) & 0xFFU);
    p_pkt[7] = aux;
}


/* ===================== IMA ADPCM 4비트 (마이크) =========================
 *
 * 표준 IMA/DVI ADPCM. 샘플당 16비트 -> 4비트라 마이크 대역폭이 정확히 1/4이
 * 된다(256 -> 64 kbps). 손실 압축이지만 예측 오차만 양자화하므로 SNR이
 * 대략 20dB 이상 유지되고, 두드림처럼 큰 과도 신호에는 특히 잘 맞는다
 * (step size가 신호를 따라 커진다).
 *
 * ⚠️ 손실이다. 파형 자체를 정밀하게 봐야 하는 분석에는 프로파일 0을 쓸 것.
 */
static const int16_t ima_step_table[89] = {
        7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
       19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
       50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
      130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
      337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
      876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
     2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
     5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int8_t ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

/**@brief 샘플 한 개를 4비트 코드로 인코딩하고 인코더 상태를 갱신한다.
 *
 * @details 인코더는 자기가 만들어낼 디코딩 결과(*p_pred)를 그대로 따라가야
 *          한다. 안 그러면 오차가 누적돼 수신 파형이 서서히 어긋난다.
 */
static uint8_t adpcm_encode_sample(int16_t sample, int16_t *p_pred, uint8_t *p_idx)
{
    int32_t step  = ima_step_table[*p_idx];
    int32_t diff  = (int32_t)sample - (int32_t)(*p_pred);
    uint8_t code  = 0;

    if (diff < 0)
    {
        code = 8;
        diff = -diff;
    }

    /* diff를 step의 1/8 단위로 3비트 양자화 (표준 IMA 순서: 4, 2, 1) */
    int32_t delta = step >> 3;
    if (diff >= step)      { code |= 4; diff -= step;      delta += step;      }
    step >>= 1;
    if (diff >= step)      { code |= 2; diff -= step;      delta += step;      }
    step >>= 1;
    if (diff >= step)      { code |= 1;                    delta += step;      }

    /* 디코더와 똑같이 예측값을 갱신한다 */
    int32_t pred = (int32_t)(*p_pred) + ((code & 8) ? -delta : delta);
    if (pred >  32767) { pred =  32767; }
    if (pred < -32768) { pred = -32768; }
    *p_pred = (int16_t)pred;

    int32_t idx = (int32_t)(*p_idx) + ima_index_table[code & 0x0F];
    if (idx < 0)  { idx = 0;  }
    if (idx > 88) { idx = 88; }
    *p_idx = (uint8_t)idx;

    return code;
}


/* ===================== 8비트 u-law (마이크, G.711) ======================
 *
 * 샘플당 16 -> 8비트. ADPCM 4비트(오차 약 10%)보다 정밀하고(약 1%), 대신
 * 대역폭이 두 배다(67 -> 133 kbps). 마이크는 전체 트래픽의 11%뿐이라
 * 이 정도는 감당된다.
 *
 * 왜 8비트 ADPCM이 아니라 u-law인가:
 *   - 8비트 IMA ADPCM은 표준이 없다. step 적응 규칙을 직접 설계해야 하고,
 *     펌웨어/PC/앱 세 곳이 비트 단위로 같아야 하는데 그걸 대조할 레퍼런스가
 *     없다. 이 프로젝트는 §6.7처럼 "그럴듯해 보이는 조용한 버그"에 이미
 *     한 번 당했다.
 *   - u-law는 G.711 표준이고 파이썬 audioop와 바이트 단위로 대조할 수 있다.
 *   - **상태가 없다.** 패킷마다 predictor를 실을 필요도 없고, 패킷 하나가
 *     유실돼도 그 패킷 몫만 잃는다. ADPCM처럼 상태가 어긋날 여지가 아예 없다.
 *
 * 로그 압축이라 큰 소리든 작은 소리든 **상대 오차가 일정**하다. 조용한 구간이
 * 묻히지 않는다는 뜻이고, 이건 ADPCM과 같은 성질이다.
 */
/* 아래는 G.711 표준(Sun의 g711.c) 그대로다. 14비트 영역에서 계산한다.
 * 임의로 손대지 말 것 — audioop와 바이트 단위로 일치하는지가 이 코덱을
 * 믿을 수 있는 유일한 근거다. */
#define ULAW_CLIP  8159   /* 14비트 영역 상한 */
#define ULAW_BIAS  33     /* 0x84 >> 2 */

static uint8_t ulaw_encode_sample(int16_t sample)
{
    static const int16_t seg_uend[8] =
        { 0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF };

    int32_t pcm  = sample >> 2;      /* 16비트 -> 14비트 */
    uint8_t mask;

    if (pcm < 0)
    {
        pcm  = -pcm;
        mask = 0x7F;
    }
    else
    {
        mask = 0xFF;
    }
    if (pcm > ULAW_CLIP)
    {
        pcm = ULAW_CLIP;
    }
    pcm += ULAW_BIAS;

    uint8_t seg = 0;
    while ((seg < 8) && (pcm > seg_uend[seg]))
    {
        seg++;
    }
    if (seg >= 8)
    {
        return (uint8_t)(0x7F ^ mask);
    }

    uint8_t uval = (uint8_t)((seg << 4) | ((pcm >> (seg + 1)) & 0x0F));
    return (uint8_t)(uval ^ mask);
}


/* ===================== 12비트 팩 (IMU) ==================================
 *
 * 16비트 원시값의 하위 4비트를 버리고 12비트로 담는다. 값의 범위(±16g,
 * ±2000dps)는 그대로고 분해능만 1/16이 된다.
 *
 * 왜 12비트로 잘라도 되나: LSM6DSL의 가속도 잡음 밀도는 80 ug/sqrt(Hz)라
 * 3.3kHz 대역에서 rms 잡음이 약 4.6 mg다. ±16g를 12비트로 나누면 LSB가
 * 7.8 mg이고 양자화 잡음은 rms 2.25 mg라 센서 자체 잡음보다 작다. 즉
 * 버리는 4비트는 대부분 센서 잡음이다. 자이로는 여유가 적어(잡음 rms
 * 0.22 dps vs LSB 0.98 dps) 약간 손해를 본다.
 *
 * ⚠️ 원시 LSB의 하위 4비트가 필요한 분석에는 프로파일 0을 쓸 것.
 */

/**@brief 12비트 값 n개를 연속 비트스트림으로 패킹. 반환 = 쓴 바이트 수. */
static uint16_t pack12(uint8_t *dst, int16_t const *src, uint16_t n)
{
    uint16_t byte_pos = 0;
    uint16_t i        = 0;

    /* 두 개씩 묶으면 정확히 3바이트라 시프트가 단순해진다 */
    for (; (uint16_t)(i + 1U) < n; i += 2U)
    {
        uint16_t a = (uint16_t)(src[i]     >> 4) & 0x0FFFU;
        uint16_t b = (uint16_t)(src[i + 1] >> 4) & 0x0FFFU;
        dst[byte_pos++] = (uint8_t)(a & 0xFFU);
        dst[byte_pos++] = (uint8_t)(((a >> 8) & 0x0FU) | ((b & 0x0FU) << 4));
        dst[byte_pos++] = (uint8_t)((b >> 4) & 0xFFU);
    }
    if (i < n)   /* 홀수 개면 마지막 하나는 1.5바이트, 남는 반바이트는 0 */
    {
        uint16_t a = (uint16_t)(src[i] >> 4) & 0x0FFFU;
        dst[byte_pos++] = (uint8_t)(a & 0xFFU);
        dst[byte_pos++] = (uint8_t)((a >> 8) & 0x0FU);
    }
    return byte_pos;
}

/**@brief 12비트 값 n개를 담는 데 필요한 바이트 수. */
static uint16_t pack12_bytes(uint16_t n)
{
    return (uint16_t)(((uint32_t)n * 12U + 7U) / 8U);
}


/**@brief 논블로킹 전송. 보냈으면 true, 송신 큐가 꽉 찼으면 false.
 *
 * @details 스트리밍의 핵심. 예전 ble_send_bytes()처럼 여기서 기다려버리면
 *          전송이 막힐 때마다 센서 샘플링 박자가 같이 깨진다. 못 보내면
 *          그냥 false를 돌려주고, 다음 메인 루프에서 다시 시도한다.
 */
static bool ble_try_send(uint8_t const *p_data, uint16_t len)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return false;
    }

    uint16_t   length   = len;
    ret_code_t err_code = ble_nus_data_send(&m_nus, (uint8_t *)p_data, &length, m_conn_handle);

    if (err_code == NRF_SUCCESS)
    {
        return true;
    }
    /* RESOURCES = 송신 큐 만석(정상, 다음에 재시도).
     * INVALID_STATE / NOT_FOUND = 상대가 notify를 아직 안 켰음. */
    if ((err_code == NRF_ERROR_RESOURCES)     ||
        (err_code == NRF_ERROR_INVALID_STATE) ||
        (err_code == NRF_ERROR_NOT_FOUND))
    {
        return false;
    }

    APP_ERROR_CHECK(err_code);
    return false;
}


/**@brief IMU 링에서 한 패킷을 만들어 보낸다. 보낼 게 없으면 true(할 일 없음).
 *  @return 큐가 차서 막혔으면 false. */
static bool stream_send_imu_packet(uint8_t *pkt, uint16_t payload_max)
{
    if (m_imu_tail == m_imu_head)
    {
        return true;
    }

    /* --- 압축 경로: 12비트 팩 (프로파일 2/3/4) --- */
    uint8_t profile = m_profile;
    if ((profile == STREAM_PROFILE_IMU_PACK)   ||
        (profile == STREAM_PROFILE_BOTH)       ||
        (profile == STREAM_PROFILE_BOTH_ACCEL) ||
        (profile == STREAM_PROFILE_BOTH_MIC8))
    {
        uint8_t  fmt      = (profile == STREAM_PROFILE_BOTH_ACCEL) ?
                            IMU_PACK_FMT_ACCEL : IMU_PACK_FMT_6AXIS;
        uint16_t per_smpl = (fmt == IMU_PACK_FMT_ACCEL) ? 3U : 6U;   /* 12비트 값 개수 */
        uint16_t budget   = (uint16_t)(payload_max - PKT_HEADER2_BYTES);

        /* budget 바이트에 들어가는 최대 샘플 수. 샘플당 12*per_smpl 비트. */
        uint16_t room = (uint16_t)(((uint32_t)budget * 8U) / (12U * per_smpl));
        /* vals[]가 넘치지 않게 상한을 건다. payload_max는 244로 묶여 있어
         * 실제로는 항상 156값(6축 26샘플 / 가속도만 52샘플)에서 딱 맞지만,
         * MTU 상수를 나중에 키웠을 때 조용히 스택을 넘지 않도록 못박아 둔다. */
        if (room > (IMU_PACK_MAX_VALS / per_smpl))
        {
            room = (uint16_t)(IMU_PACK_MAX_VALS / per_smpl);
        }
        if (room == 0)
        {
            return true;   /* MTU가 너무 작다 — 아래 raw 경로도 못 쓴다 */
        }

        uint32_t avail = m_imu_head - m_imu_tail;
        uint16_t count = (avail < room) ? (uint16_t)avail : room;

        /* 링에서 꺼내 12비트 값 배열로 펼친다 */
        int16_t  vals[IMU_PACK_MAX_VALS];
        uint16_t n = 0;
        for (uint16_t i = 0; i < count; i++)
        {
            imu_sample_t const *p_s = &m_imu_ring[(m_imu_tail + i) & (IMU_RING_SAMPLES - 1U)];
            vals[n++] = p_s->ax;
            vals[n++] = p_s->ay;
            vals[n++] = p_s->az;
            if (fmt == IMU_PACK_FMT_6AXIS)
            {
                vals[n++] = p_s->gx;
                vals[n++] = p_s->gy;
                vals[n++] = p_s->gz;
            }
        }

        pkt_write_header2(pkt, PKT_MAGIC_IMU_PACK, m_imu_tail, count, fmt);
        uint16_t body = pack12(&pkt[PKT_HEADER2_BYTES], vals, n);

        if (!ble_try_send(pkt, (uint16_t)(PKT_HEADER2_BYTES + body)))
        {
            return false;
        }
        m_imu_tail += count;
        return true;
    }

    /* --- 기존 raw 경로 (프로파일 0/1) --- */
    uint16_t room  = (uint16_t)((payload_max - PKT_HEADER_BYTES) / sizeof(imu_sample_t));
    uint32_t avail = m_imu_head - m_imu_tail;
    uint16_t count = (avail < room) ? (uint16_t)avail : room;

    pkt_write_header(pkt, PKT_MAGIC_IMU, m_imu_tail, (uint8_t)count);
    for (uint16_t i = 0; i < count; i++)
    {
        imu_sample_t const *p_s = &m_imu_ring[(m_imu_tail + i) & (IMU_RING_SAMPLES - 1U)];
        memcpy(&pkt[PKT_HEADER_BYTES + i * sizeof(imu_sample_t)], p_s, sizeof(imu_sample_t));
    }

    if (!ble_try_send(pkt, (uint16_t)(PKT_HEADER_BYTES + count * sizeof(imu_sample_t))))
    {
        return false;
    }

    m_imu_tail += count;
    return true;
}


/**@brief 마이크 링에서 한 패킷을 만들어 보낸다. 보낼 게 없으면 true(할 일 없음).
 *  @return 큐가 차서 막혔으면 false. */
static bool stream_send_mic_packet(uint8_t *pkt, uint16_t payload_max)
{
    if (m_mic_tail == m_mic_head)
    {
        return true;
    }

    mic_chunk_t const *p_chunk = &m_mic_chunks[m_mic_tail & (MIC_CHUNK_COUNT - 1U)];

    /* --- 압축 경로: IMA ADPCM 4비트 (프로파일 1/3/4) ---
     *
     * 청크 하나(최대 256샘플 = 128바이트)를 통째로 한 패킷에 담는다.
     * 청크를 쪼개지 않는 이유는 인코더 상태가 청크 경계에서만 확정되면
     * 재시도 처리가 단순해지기 때문이다. 138바이트라 244 안에 넉넉히 들어간다. */
    uint8_t codec = mic_codec_for(m_profile);

    /* --- 압축 경로: 8비트 u-law (프로파일 5) ---
     *
     * 상태가 없으므로 ADPCM과 달리 헤더에 실을 것도, 재시도 시 되돌릴 것도 없다.
     * 한 샘플이 1바이트라 청크 256개가 한 패킷(244)에 안 들어간다. 기존 raw
     * 경로와 똑같이 m_mic_offset으로 청크를 나눠 보낸다. */
    if (codec == MIC_CODEC_ULAW8)
    {
        uint16_t room  = (uint16_t)(payload_max - PKT_HEADER2_BYTES);
        uint16_t left  = (uint16_t)(p_chunk->count - m_mic_offset);
        uint16_t count = (left < room) ? left : room;

        pkt_write_header2(pkt, PKT_MAGIC_MIC_ULAW,
                          p_chunk->index + m_mic_offset, count, 0);
        for (uint16_t i = 0; i < count; i++)
        {
            pkt[PKT_HEADER2_BYTES + i] =
                ulaw_encode_sample(p_chunk->samples[m_mic_offset + i]);
        }

        if (!ble_try_send(pkt, (uint16_t)(PKT_HEADER2_BYTES + count)))
        {
            return false;   // 큐 만석 -> 다음 루프에서 같은 위치부터 재시도
        }

        m_mic_offset = (uint16_t)(m_mic_offset + count);
        if (m_mic_offset >= p_chunk->count)
        {
            m_mic_offset = 0;
            m_mic_tail++;
        }
        return true;
    }

    if (codec == MIC_CODEC_ADPCM4)
    {
        uint16_t n = p_chunk->count;
        if ((uint16_t)(PKT_ADPCM_HDR_BYTES + (n + 1U) / 2U) > payload_max)
        {
            return true;   /* MTU가 비정상적으로 작다 — 이 경로로는 못 보낸다 */
        }

        /* 상태는 지역 복사본으로 굴리고, 전송에 성공했을 때만 커밋한다.
         * 큐가 차서 false를 돌려주고 나중에 같은 청크를 다시 인코딩할 때
         * 상태가 두 번 전진하면 수신 파형이 어긋난다. */
        int16_t pred = m_adpcm_pred;
        uint8_t idx  = m_adpcm_idx;

        pkt_write_header2(pkt, PKT_MAGIC_MIC_ADPCM, p_chunk->index, n, idx);
        pkt[8] = (uint8_t)((uint16_t)pred & 0xFFU);
        pkt[9] = (uint8_t)(((uint16_t)pred >> 8) & 0xFFU);

        uint8_t *body = &pkt[PKT_ADPCM_HDR_BYTES];
        for (uint16_t i = 0; i < n; i += 2U)
        {
            uint8_t lo = adpcm_encode_sample(p_chunk->samples[i], &pred, &idx);
            uint8_t hi = 0;
            if ((uint16_t)(i + 1U) < n)
            {
                hi = adpcm_encode_sample(p_chunk->samples[i + 1U], &pred, &idx);
            }
            body[i / 2U] = (uint8_t)(lo | (hi << 4));
        }

        uint16_t len = (uint16_t)(PKT_ADPCM_HDR_BYTES + (n + 1U) / 2U);
        if (!ble_try_send(pkt, len))
        {
            return false;   /* 상태를 커밋하지 않았으므로 다음에 그대로 다시 만든다 */
        }

        m_adpcm_pred = pred;
        m_adpcm_idx  = idx;
        m_mic_offset = 0;
        m_mic_tail++;
        return true;
    }

    /* --- 기존 raw PCM16 경로 (프로파일 0/2) --- */
    uint16_t           room    = (uint16_t)((payload_max - PKT_HEADER_BYTES) / sizeof(int16_t));
    uint16_t           left    = (uint16_t)(p_chunk->count - m_mic_offset);
    uint16_t           count   = (left < room) ? left : room;

    pkt_write_header(pkt, PKT_MAGIC_MIC, p_chunk->index + m_mic_offset, (uint8_t)count);
    memcpy(&pkt[PKT_HEADER_BYTES], &p_chunk->samples[m_mic_offset], count * sizeof(int16_t));

    if (!ble_try_send(pkt, (uint16_t)(PKT_HEADER_BYTES + count * sizeof(int16_t))))
    {
        return false;   // 큐 만석 -> 다음 루프에서 같은 위치부터 재시도
    }

    m_mic_offset = (uint16_t)(m_mic_offset + count);
    if (m_mic_offset >= p_chunk->count)
    {
        m_mic_offset = 0;
        m_mic_tail++;
    }
    return true;
}


/**@brief 한 IMU 패킷에 들어가는 샘플 수. WFQ 비용 계산에 쓴다.
 *
 * @details stream_send_imu_packet()의 room 계산과 반드시 같아야 한다. 어긋나면
 *          두 스트림의 배분 비율이 틀어진다(데이터가 깨지진 않는다).
 */
static uint16_t imu_samples_per_packet(void)
{
    uint16_t payload_max = (m_ble_nus_max_data_len < PKT_MAX_BYTES) ?
                           m_ble_nus_max_data_len : PKT_MAX_BYTES;

    switch (m_profile)
    {
        case STREAM_PROFILE_IMU_PACK:
        case STREAM_PROFILE_BOTH:
        case STREAM_PROFILE_BOTH_MIC8:
            /* 12비트 x 6축 = 샘플당 9바이트 */
            return (uint16_t)(((uint32_t)(payload_max - PKT_HEADER2_BYTES) * 8U) / (12U * 6U));
        case STREAM_PROFILE_BOTH_ACCEL:
            /* 12비트 x 3축 = 샘플당 4.5바이트 */
            return (uint16_t)(((uint32_t)(payload_max - PKT_HEADER2_BYTES) * 8U) / (12U * 3U));
        default:
            /* raw: 샘플당 12바이트 */
            return (uint16_t)((payload_max - PKT_HEADER_BYTES) / sizeof(imu_sample_t));
    }
}


/**@brief 한 마이크 패킷에 들어가는 샘플 수. WFQ 비용 계산에 쓴다.
 *
 * @details ADPCM은 청크를 통째로 한 패킷에 담고, u-law/raw는 payload에 들어가는
 *          만큼 쪼개 보낸다. 정확한 값이 아니어도 배분 비율에만 영향을 주므로
 *          청크 크기로 상한을 걸어 근사한다.
 */
static uint16_t mic_samples_per_packet(uint16_t payload_max)
{
    uint16_t chunk = (uint16_t)(PDM_BUFFER_SAMPLES / m_mic_decim);
    uint16_t n;

    switch (mic_codec_for(m_profile))
    {
        case MIC_CODEC_ADPCM4:  n = chunk; break;                            /* 청크 통째로 */
        case MIC_CODEC_ULAW8:   n = (uint16_t)(payload_max - PKT_HEADER2_BYTES); break;  /* 1B/샘플 */
        default:                n = (uint16_t)((payload_max - PKT_HEADER_BYTES) / 2U);   /* 2B/샘플 */
    }
    return (n > chunk) ? chunk : n;
}


/**@brief 링에 고인 IMU 데이터의 시간 길이(ms). */
static uint32_t imu_backlog_ms(void)
{
    if (m_imu_odr_hz == 0U) return 0U;
    return ((m_imu_head - m_imu_tail) * 1000UL) / m_imu_odr_hz;
}

/**@brief 링에 고인 마이크 데이터의 시간 길이(ms). */
static uint32_t mic_backlog_ms(void)
{
    return (m_mic_head - m_mic_tail) * MIC_CHUNK_MS;
}


/**@brief 지연 예산을 넘게 고인 것을 오래된 쪽부터 버린다.
 *
 * @details 링이 꽉 찬 채로 유지되면 지연 = 링 깊이가 된다. 그 상태에서 오래된 것을
 *          붙들고 있어봐야 어차피 링이 넘칠 때 버려진다 — 늦게, 그리고 지연까지
 *          얹어서 버려질 뿐이다. 그럴 바에는 지금 버리고 최신을 보내는 게 낫다.
 *
 *          두 스트림에 같은 예산을 걸어야 화면에서 둘이 어긋나지 않는다. 한쪽만
 *          제한하면 제한 안 한 쪽이 그만큼 뒤처져 보인다 — 지금 증상이 그거였다.
 *
 *          버린 만큼 절대 샘플 번호에 구멍이 나므로 수신측은 이걸 유실로 정확히
 *          센다. 조용히 사라지지 않는다.
 *
 * @warning 메인 루프에서만 부를 것. tail을 만진다.
 */
static void stream_trim_backlog(void)
{
    /* 'e' 이후 잔량 flush 중에는 버리면 안 된다. 그때는 센서가 이미 멈춘 뒤라
     * 새로 고이지 않고, 남은 건 전부 사용자가 녹음한 실제 데이터다. */
    if (m_stream_flushing)
    {
        return;
    }

    /* --- IMU --- */
    if (m_imu_odr_hz > 0U)
    {
        uint32_t max_samples = (m_imu_odr_hz * m_latency_budget_ms) / 1000UL;
        /* 패킷 하나도 못 채울 만큼 작으면 의미가 없다. 최소한 몇 패킷은 담기게. */
        uint32_t floor_samples = (uint32_t)imu_samples_per_packet() * 4U;
        if (max_samples < floor_samples) max_samples = floor_samples;
        /* 마이크 쪽과 같은 이유로 링 위로는 못 올라간다. 예산이 링보다 크면 이
         * 함수는 아무것도 안 하고 배수 쪽의 drop-oldest가 대신 자르는데, 그러면
         * 실제 상한이 예산이 아니라 링 깊이가 되어 lat 보고가 거짓말이 된다. */
        if (max_samples > (IMU_RING_SAMPLES - 1U)) max_samples = IMU_RING_SAMPLES - 1U;

        uint32_t lag = m_imu_head - m_imu_tail;
        if (lag > max_samples)
        {
            uint32_t drop = lag - max_samples;
            m_imu_tail    += drop;
            m_imu_trimmed += drop;
        }
    }

    /* --- 마이크 ---
     * head는 PDM ISR이, tail은 메인 루프가 만진다. 여기서 tail만 앞으로 미는 건
     * 안전하다(ISR은 tail을 안 읽고 head-tail 거리만 본다). 청크를 건너뛰면
     * 현재 청크 안 위치(m_mic_offset)는 반드시 0으로 되돌려야 한다. */
    {
        uint32_t max_chunks = m_latency_budget_ms / MIC_CHUNK_MS;
        if (max_chunks < 2U) max_chunks = 2U;
        /* 링보다 큰 상한은 의미가 없다 — 그 위는 ISR의 drop-newest가 먼저 걸리고,
         * 그러면 버려지는 게 '가장 오래된 것'이 아니라 '방금 들어온 것'이 되어
         * 지연이 링 깊이에 고정된다. 예산이 링을 넘지 않게 여기서 묶는다. */
        if (max_chunks > (MIC_CHUNK_COUNT - 1U)) max_chunks = MIC_CHUNK_COUNT - 1U;

        uint32_t lag = m_mic_head - m_mic_tail;
        if (lag > max_chunks)
        {
            uint32_t drop  = lag - max_chunks;
            m_mic_tail    += drop;
            m_mic_offset   = 0;
            m_mic_trimmed += drop * (PDM_BUFFER_SAMPLES / m_mic_decim);
        }
    }
}


/**@brief 링버퍼에 쌓인 것을 BLE로 흘려보낸다. 메인 루프가 매번 호출.
 *
 * @details **가중 공정 큐잉(WFQ).** 매번 '자기 레이트 대비 더 뒤처진' 스트림을
 *          골라 한 패킷씩 보낸다. 링크에 여유가 있으면 둘 다 100%가 나가고,
 *          모자라면 둘이 같은 비율로 유실된다.
 *
 *          이 함수의 역사가 곧 이 설계의 근거다. 앞선 두 정책은 모두 한쪽을
 *          굶겨 죽였다:
 *            - 라운드로빈(교대 상태가 호출마다 리셋) -> 사실상 IMU 우선, 마이크 91% 유실
 *            - 마이크 우선(전부 비운 뒤 IMU)         -> 링크가 모자라면 IMU 100% 유실.
 *              2026-07-24 폰 녹음이 정확히 이 모습이었다(마이크 무손실, IMU 0샘플).
 *
 *          ⚠️ "IMU 최소 1패킷 보장"처럼 호출 횟수로 공평성을 걸면 안 된다.
 *          이 함수는 초당 1000회 넘게 불리므로 호출당 1패킷만으로도 IMU 필요량이
 *          다 채워지고 마이크가 89% 죽는다. WFQ는 호출 횟수와 무관하게 "보낸
 *          패킷이 담은 스트림 시간"으로만 판단하므로 그 함정이 없다.
 *
 *          링크가 진짜로 모자라면 여기서 할 수 있는 일은 없다. 그때 고칠 곳은
 *          요구 대역폭이다(프로파일 3/4, 또는 IMU 3333Hz). PROGRESS.md §6.6.5/§6.6.6.
 *
 *          전송에 성공한 만큼만 tail을 전진시키므로 데이터가 유실되지 않는다.
 */
static void stream_service(void)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    uint16_t payload_max = (m_ble_nus_max_data_len < PKT_MAX_BYTES) ?
                           m_ble_nus_max_data_len : PKT_MAX_BYTES;
    /* IMU 한 샘플(12B)조차 못 담는 MTU면 아래 루프에서 count=0인 빈 패킷만
     * 반복해 tail이 영원히 안 움직인다. MTU 협상 전(기본 23)에도 20바이트라
     * 실제로 걸릴 일은 없지만, 걸리면 메인 루프가 통째로 멈추므로 막아 둔다. */
    if (payload_max < (PKT_HEADER_BYTES + sizeof(imu_sample_t)))
    {
        return;
    }

    uint8_t pkt[PKT_MAX_BYTES];

    /* 보내기 전에 지연 예산을 넘게 고인 것을 먼저 버린다. 이걸 송신 뒤에 하면
     * 이미 오래된 데이터를 한 번 실어보낸 뒤에 버리는 셈이라 대역만 낭비한다. */
    stream_trim_backlog();

    /* 가중 공정 큐잉. 매번 '더 뒤처진' 스트림을 골라 한 패킷씩 보낸다.
     *
     * 예전에는 마이크를 전부 비운 뒤에만 IMU를 보냈다. 링크에 여유가 있을 땐
     * 똑같이 동작하지만, 안드로이드가 화면 꺼짐 등으로 연결 간격을 늘려 대역이
     * 모자라지면 부족분이 100% IMU에 쏠렸다. 이제는 둘이 같은 비율로 유실된다.
     * (호스트 시뮬레이션: 링크 100pkt/s일 때 예전 IMU 14.6%/MIC 100% ->
     *  지금 IMU 31.4%/MIC 31.4%.)
     *
     * 비용은 "그 패킷이 담은 스트림 시간"이고, 나눗셈을 피하려고 상대 레이트를
     * 곱해서 비교한다. 자세한 근거는 m_vt_imu 선언부 주석 참고. */
    uint16_t imu_pkt_samples = imu_samples_per_packet();
    uint16_t mic_pkt_samples = mic_samples_per_packet(payload_max);
    uint16_t mic_hz          = (uint16_t)(MIC_BASE_RATE_HZ / m_mic_decim);

    int32_t cost_imu = (int32_t)imu_pkt_samples * (int32_t)mic_hz;
    int32_t cost_mic = (int32_t)mic_pkt_samples * (int32_t)m_imu_odr_hz;

    for (;;)
    {
        bool imu_ready = (m_imu_tail != m_imu_head);
        bool mic_ready = (m_mic_tail != m_mic_head);

        if (!imu_ready && !mic_ready)
        {
            /* 둘 다 비었다. 다음 구간이 공평하게 시작하도록 가상 시각을 리셋한다. */
            m_vt_imu = 0;
            m_vt_mic = 0;
            return;
        }

        /* 비어 있는 쪽은 크레딧을 쌓아두지 못하게 상대 시각으로 끌어올린다.
         * 안 그러면 잠깐 쉬었다 돌아온 스트림이 한동안 링크를 독점한다. */
        if (!imu_ready) m_vt_imu = m_vt_mic;
        if (!mic_ready) m_vt_mic = m_vt_imu;

        int32_t base = (m_vt_imu < m_vt_mic) ? m_vt_imu : m_vt_mic;
        m_vt_imu -= base;      /* 정규화: 둘 중 작은 쪽이 항상 0이 되어 누적 오버플로가 없다 */
        m_vt_mic -= base;

        /* 진행 없음 감지용. 전송에 "성공"했는데 tail이 안 움직이면 (예: MTU가 너무
         * 작아 count=0인 빈 패킷) 이 루프가 영원히 돌아 메인 루프가 통째로 멈춘다.
         * 위쪽 payload_max 검사로 실제로는 안 걸리지만, 여기서 도는 순간 보드가
         * 먹통이 되므로 값싼 보험을 하나 걸어둔다. */
        uint32_t imu_tail_before = m_imu_tail;
        /* u-law/raw는 청크를 쪼개 보내므로 성공해도 tail 대신 offset만 움직인다.
         * offset을 안 보면 정상 동작을 "진행 없음"으로 오판한다. */
        uint32_t mic_pos_before  = m_mic_tail + m_mic_offset;

        if (imu_ready && (!mic_ready || (m_vt_imu <= m_vt_mic)))
        {
            if (!stream_send_imu_packet(pkt, payload_max))
            {
                return;   /* 송신 큐 만석. 다음 호출에서 같은 자리부터 다시 판단한다 */
            }
            m_vt_imu += cost_imu;
        }
        else
        {
            if (!stream_send_mic_packet(pkt, payload_max))
            {
                return;
            }
            m_vt_mic += cost_mic;
        }

        if ((m_imu_tail == imu_tail_before) &&
            ((m_mic_tail + m_mic_offset) == mic_pos_before))
        {
            NRF_LOG_WARNING("stream_service: no progress - bailing out");
            return;
        }
    }
}


/**@brief 실제 협상된 링크 상태를 한 줄로 보낸다.
 *
 * @details 처리량이 안 나올 때 상대(호스트) 탓인지 우리 설정 탓인지 가르는 근거.
 *          interval_ms = units * 1.25. tx_queue는 우리가 SoftDevice에 요청한 깊이로,
 *          연결 이벤트를 꽉 채우려면 이게 이벤트당 패킷 수보다 커야 한다.
 */
static void link_report_send(void)
{
    char line[200];
    snprintf(line, sizeof(line),
             "LINK,interval_units=%u,interval_ms=%u.%02u,phy_tx=%u,phy_rx=%u,"
             "mtu_payload=%u,tx_queue=%u,tx_octets=%u,tx_time_us=%u,profile=%u\n",
             m_conn_interval_units,
             (unsigned)(m_conn_interval_units * 125U / 100U),
             (unsigned)((m_conn_interval_units * 125U) % 100U),
             m_phy_tx, m_phy_rx, m_ble_nus_max_data_len,
             (unsigned)HVN_TX_QUEUE_SIZE,
             m_tx_octets, m_tx_time_us, (unsigned)m_profile);
    ble_send_str(line);
}


/**@brief 1초마다 나가는 상태 한 줄.
 *
 * @details 지금까지 "IMU가 언제부터 안 나오기 시작했나"를 알 방법이 없었다.
 *          STAT은 'e' 이후에만 나가는데 앱은 'e' 직후 연결을 끊어서 그것조차
 *          못 받았고, 그 결과 40초짜리 녹음이 IMU 0샘플로 끝나도 원인을 가릴
 *          근거가 하나도 안 남았다(2026-07-24).
 *
 *          텍스트 한 줄(약 120바이트)이라 초당 1kbps도 안 된다. 이 비용으로
 *          다음 재현 때 §1(링크)인지 §2(센서)인지가 한 줄로 갈린다:
 *            - imu_prod가 안 늘어남           -> 센서/버스. stall/recover를 본다
 *            - imu_prod는 느는데 imu_drop 폭증 -> 링크. interval/phy를 본다
 *            - mic_drop이 같이 늘어남          -> 진짜 대역폭 부족. 레이트를 내린다
 */
static void health_report_send(void)
{
    /* ⚠️ ble_send_str()는 243자에서 말없이 자른다(MTU 244 - 매직 1바이트).
     * 그래서 두 가지를 지킨다: 필드 이름을 짧게 쓰고, 진단에 먼저 필요한 것을
     * 앞에 둔다. 혹시라도 잘리면 뒤쪽 참고용 필드만 사라지도록.
     *
     * 길이 실측(2026-07-28, lat 추가 후): 하루 연속 녹음에서 모든 샘플을 잃는
     * 최악 시나리오가 216자다. 카운터를 문자 그대로 uint32 최대로 채우면 256자로
     * 넘치지만 그 값은 도달할 수 없다 — trim/drop/prod는 전부 생산량 이하이고,
     * 16kHz 생산량이 uint32를 채우려면 3일이 걸린다. 그래도 넘칠 때를 대비해
     * 새 필드(lat)는 앞쪽 지연 그룹에 넣었다. 잘리는 건 항상 intv/phy 쪽이다. */
    char line[256];
    snprintf(line, sizeof(line),
             /* --- 먼저 봐야 할 것: 보드가 실제로 뭘 하고 있나 --- */
             "HEALTH,t=%lu,prof=%u,ihz=%u,iok=%u,"
             /* --- 지연: 링에 고여 있는 시간. 두 값의 차이 = 화면에서 보이는 어긋남.
              *     lat은 보드가 실제로 쓰고 있는 예산(0xA3). 앱이 보낸 값과 다르면
              *     그 쓰기가 유실된 것이다 — prof와 같은 이유로 여기 싣는다. --- */
             "lat=%lu,ilag=%lu,mlag=%lu,"
             /* --- 유실: trim은 지연 상한에 걸려 버린 것, drop은 링이 넘친 것 --- */
             "itrim=%lu,mtrim=%lu,idrop=%lu,mdrop=%lu,"
             /* --- 센서 건강 --- */
             "ovr=%lu,stall=%lu,recov=%lu,"
             /* --- 총 생산량과 링크 상태 --- */
             "iprod=%lu,mprod=%lu,intv=%u,phy=%u\n",
             (unsigned long)m_uptime_sec,
             /* ⚠️ 보드가 실제로 쓰고 있는 프로파일. 앱이 고른 값과 다를 수 있다.
              * 보드는 항상 프로파일 0(무압축)으로 부팅하고, 앱이 0xA2로 바꿔주기
              * 전까지는 그대로다. 그 쓰기가 유실되거나 거부되면 조용히 무압축으로
              * 남는데, 그러면 필요 대역이 632 -> 947 kbps로 뛰어 링크 상한(약
              * 600 kbps)의 1.5배가 된다. 큐가 영구히 꽉 차서 지연이 상한에 붙고
              * 유실이 계속 나는 상태가 정확히 이것이다.
              * RATE 줄에도 profile이 실려 나갔지만 앱이 파싱하지 않아 안 보였다. */
             (unsigned)m_profile,
             m_imu_odr_hz,
             m_imu_ok ? 1U : 0U,
             /* 링에 고인 시간 = 화면에 보이는 지연. 상한이 걸려 있으므로 둘 다
              * lat 이하로 유지되어야 한다. */
             (unsigned long)m_latency_budget_ms,
             (unsigned long)imu_backlog_ms(),
             (unsigned long)mic_backlog_ms(),
             /* 지연 상한에 걸려 버린 양. 0이 아니면 링크가 만성적으로 모자란
              * 것이므로 상한이 아니라 레이트나 프로파일을 내려야 한다. */
             (unsigned long)m_imu_trimmed,
             (unsigned long)m_mic_trimmed,
             (unsigned long)m_imu_dropped,
             (unsigned long)m_mic_dropped,
             (unsigned long)m_imu_fifo_ovr,
             (unsigned long)m_imu_stall_cnt,
             (unsigned long)m_imu_recover_cnt,
             (unsigned long)(m_imu_total + m_imu_head),
             (unsigned long)(m_mic_total + m_mic_produced),
             m_conn_interval_units, m_phy_tx);
    ble_send_str(line);
}


/**@brief 'e' 수신 후 마무리: 잔량을 끝까지 밀어내고 요약을 보낸다.
 *
 * @details 센서가 이미 멈춘 뒤라서 여기서는 기다려도 박자가 깨질 게 없다.
 *          다만 연결이 끊기거나 상대가 안 받아주는 경우를 대비해 상한을 둔다.
 */
static void stream_finish(void)
{
    m_stream_flushing = true;   /* 이 구간에서는 지연 상한을 끈다 */

    /* 잔량 flush. 한 바퀴에 최소 1패킷은 나가므로 상한은 넉넉히 잡는다. */
    uint32_t guard = MIC_CHUNK_COUNT * 8U + IMU_RING_SAMPLES + 64U;

    while (guard-- > 0)
    {
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            return;
        }
        if ((m_mic_tail == m_mic_head) && (m_imu_tail == m_imu_head))
        {
            break;   // 다 비웠다
        }
        stream_service();
        idle_state_handle();   // 송신 큐가 비워질 때까지 대기
    }

    /* 요약: 수신측이 자기가 센 것과 대조해 검증할 수 있도록 실제 생산량을 알려준다. */
    char summary[200];
    snprintf(summary, sizeof(summary),
             "STAT,mic_produced=%lu,mic_dropped=%lu,imu_produced=%lu,imu_dropped=%lu,"
             "imu_fifo_ovr=%lu,mic_err=%u,mic_t0_ok=%u,imu_t0_ok=%u,"
             "imu_stall=%lu,imu_recover=%lu,imu_ok=%u,"
             "imu_trim=%lu,mic_trim=%lu\n",
             /* 세션 전체 = 리셋으로 버려진 이전 구간들 + 현재 구간. 레이트나
              * 프로파일을 중간에 바꿔도 수신측이 센 총량과 대조할 수 있다. */
             (unsigned long)(m_mic_total + m_mic_produced),
             (unsigned long)m_mic_dropped,
             (unsigned long)(m_imu_total + m_imu_head),
             (unsigned long)m_imu_dropped,
             (unsigned long)m_imu_fifo_ovr,
             m_mic_error ? 1U : 0U,
             /* 어느 스트림이 첫 샘플을 냈는지. 둘 중 하나가 0이면 그 쪽이 아예
              * 시작되지 않은 것이고, 시작 경로를 건드린 변경을 의심하면 된다. */
             m_mic_t0_valid ? 1U : 0U,
             m_imu_t0_valid ? 1U : 0U,
             /* 세션 중 IMU가 몇 번 죽었고 몇 번 전원 사이클까지 갔는지.
              * imu_stall>0인데 imu_produced가 정상이면 워치독이 제 일을 한 것이고,
              * imu_recover가 계속 올라가면 하드웨어(전원 레일/납땜)를 봐야 한다. */
             (unsigned long)m_imu_stall_cnt,
             (unsigned long)m_imu_recover_cnt,
             m_imu_ok ? 1U : 0U,
             /* 지연 상한에 걸려 버린 양. imu_dropped(링 넘침)와는 원인이 달라
              * 따로 센다. 이 값이 크면 링크가 만성적으로 모자란 것이다. */
             (unsigned long)m_imu_trimmed,
             (unsigned long)m_mic_trimmed);
    ble_send_str(summary);

    link_report_send();
    ble_send_str("END\n");

    NRF_LOG_INFO("Stream finished. mic=%lu dropped=%lu",
                 (unsigned long)m_mic_produced, (unsigned long)m_mic_dropped);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}



/**@brief NUS TX로 텍스트 한 줄 전송 (송신버퍼가 찰 때까지 대기).
 *
 * @details 스트리밍 패킷과 섞여도 구분되도록 앞에 0x01 매직을 붙인다.
 *          수신측은 첫 바이트만 보고 텍스트/마이크/IMU를 갈라낸다.
 */
static void ble_send_str(const char *s)
{
    uint8_t  buf[PKT_MAX_BYTES];
    uint16_t len = (uint16_t)strlen(s);

    if (len > (PKT_MAX_BYTES - 1))
    {
        len = PKT_MAX_BYTES - 1;
    }

    buf[0] = PKT_MAGIC_TEXT;
    memcpy(&buf[1], s, len);
    ble_send_bytes(buf, (uint16_t)(len + 1U));
}

static void ble_send_bytes(uint8_t const *p_data, uint16_t len)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    ret_code_t err_code;
    do
    {
        uint16_t length = len;
        err_code = ble_nus_data_send(&m_nus, (uint8_t *)p_data, &length, m_conn_handle);

        if (err_code == NRF_ERROR_RESOURCES)
        {
            idle_state_handle();   // 송신 버퍼가 비워질 때까지 잠깐 대기
        }
        else if ((err_code != NRF_SUCCESS) &&
                 (err_code != NRF_ERROR_INVALID_STATE) &&
                 (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}

/**@brief 소프트웨어 비트뱅잉 SPI 1바이트 송수신 (GPIO 수동 토글).
 * mode3=1: CPOL=1(클럭 idle HIGH), CPHA=1(상승 에지 샘플). mode3=0: mode0.
 * 하드웨어 SPIM 블록을 우회하므로 어떤 핀 조합에서도 멈추지 않는다. */
/* 비트뱅잉 클럭 반주기. 이 값이 IMU 최고 레이트를 정하는 첫 번째 벽이다.
 *
 * 이력: 5us(=100kHz) → 한 샘플(주소1+데이터12=13바이트)에 1.04ms가 걸려 IMU가
 * ~960Hz에서 막혔다. 1us(=500kHz)로 줄여 208us/샘플이 됐고 420Hz를 얻었다.
 * 하지만 6664Hz는 초당 87KB를 읽어야 해서 500kHz로는 CPU를 통째로 먹는다.
 *
 * nrf_delay_us(1)은 반주기의 대부분이 순수 낭비다. GPIO 토글 자체가 이미
 * 수십 ns이므로 __NOP 몇 개만 끼우면 된다. NOP 8개(=125ns) + 레지스터 접근으로
 * 반주기 ~185ns → 약 2.7MHz. LSM6DSL 정격이 10MHz라 여전히 4배 가까운 여유가
 * 있고, 한 샘플은 208us → 약 39us로 줄어든다.
 *
 * 데이터가 깨지면(WHO_AM_I가 0x6A가 아니거나 정지 중력이 1.00g에서 벗어나면)
 * 이 개수부터 늘릴 것. 배선이 길거나 풀업이 약하면 여기가 먼저 무너진다. */
#define BB_HALF_CYCLE()  do { __NOP(); __NOP(); __NOP(); __NOP(); \
                              __NOP(); __NOP(); __NOP(); __NOP(); } while (0)

static uint8_t bb_txrx(uint32_t sck, uint32_t mosi, uint32_t miso, uint8_t out, int mode3)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--)
    {
        if (mode3)
        {
            nrf_gpio_pin_clear(sck);                 // 1번째 에지(하강): 데이터 세팅
            nrf_gpio_pin_write(mosi, (out >> i) & 1);
            BB_HALF_CYCLE();
            nrf_gpio_pin_set(sck);                   // 2번째 에지(상승): 샘플
            BB_HALF_CYCLE();
            if (nrf_gpio_pin_read(miso)) in |= (1 << i);
        }
        else
        {
            nrf_gpio_pin_write(mosi, (out >> i) & 1); // mode0: idle LOW
            BB_HALF_CYCLE();
            nrf_gpio_pin_set(sck);                    // 상승 에지: 샘플
            BB_HALF_CYCLE();
            if (nrf_gpio_pin_read(miso)) in |= (1 << i);
            nrf_gpio_pin_clear(sck);
            BB_HALF_CYCLE();
        }
    }
    return in;
}

/**@brief 비트뱅잉으로 LSM6DSL WHO_AM_I(0x0F) 읽기. CS active-low. */
static uint8_t bitbang_whoami(uint32_t sck, uint32_t mosi, uint32_t miso, uint32_t cs, int mode3)
{
    nrf_gpio_cfg_output(sck);
    nrf_gpio_cfg_output(mosi);
    nrf_gpio_cfg_output(cs);
    nrf_gpio_cfg_input(miso, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_pin_write(sck, mode3 ? 1 : 0);   // idle: mode3=HIGH, mode0=LOW
    nrf_gpio_pin_set(cs);                      // CS 비활성(HIGH)
    nrf_delay_us(20);

    nrf_gpio_pin_clear(cs);                    // CS 활성(LOW)
    nrf_delay_us(5);
    (void)bb_txrx(sck, mosi, miso, 0x0F | 0x80, mode3);   // 읽기 명령 (0x8F)
    uint8_t v = bb_txrx(sck, mosi, miso, 0x00, mode3);    // WHO_AM_I 값
    nrf_gpio_pin_set(cs);                      // CS 비활성
    nrf_gpio_pin_write(sck, mode3 ? 1 : 0);    // idle 복귀

    return v;
}

/* =====================================================================
 *  LSM6DSL IMU 드라이버 (비트뱅잉 SPI mode3) — 핀맵 확정 (2026-07-20)
 *  SCK=P0.03, MOSI=P0.05, MISO=P0.06, CS=P0.02, 전원=P0.23
 *  (교수님 JY_full_merged.hex 디스어셈블로 확정, WHO_AM_I=0x6A 검증됨)
 * ===================================================================== */
#define IMU_SCK   3
#define IMU_MOSI  5
#define IMU_MISO  6
#define IMU_CS    2
#define IMU_PWR   23

#define LSM6_WHO_AM_I  0x0F
#define LSM6_CTRL1_XL  0x10   /**< 가속도 ODR/FS */
#define LSM6_CTRL2_G   0x11   /**< 자이로 ODR/FS */
#define LSM6_CTRL3_C   0x12   /**< BDU/IF_INC/SW_RESET */
#define LSM6_OUTX_L_G  0x22   /**< 자이로X_L .. (0x22~0x2D: 자이로6 + 가속도6, 자동증가) */
#define LSM6_WHO_AM_I_VAL 0x6A

/* --- FIFO --- */
#define LSM6_FIFO_CTRL1   0x06   /**< 워터마크 하위 8비트 (안 씀) */
#define LSM6_FIFO_CTRL2   0x07   /**< 워터마크 상위 + FIFO_TEMP_EN */
#define LSM6_FIFO_CTRL3   0x08   /**< DEC_FIFO_GYRO[5:3] | DEC_FIFO_XL[2:0] */
#define LSM6_FIFO_CTRL4   0x09
#define LSM6_FIFO_CTRL5   0x0A   /**< ODR_FIFO[6:3] | FIFO_MODE[2:0] */
#define LSM6_FIFO_STATUS1 0x3A   /**< DIFF_FIFO[7:0] = 안 읽은 16비트 워드 수 */
#define LSM6_FIFO_STATUS2 0x3B   /**< DIFF_FIFO[10:8] | WATERMARK | OVER_RUN | FULL | EMPTY */
#define LSM6_FIFO_STATUS3 0x3C   /**< FIFO_PATTERN[7:0] = 다음에 나올 워드의 패턴 위치 */
#define LSM6_FIFO_STATUS4 0x3D   /**< FIFO_PATTERN[9:8] */
#define LSM6_FIFO_DATA_L  0x3E   /**< 여기서 연속으로 읽으면 FIFO가 계속 흘러나온다 */

#define LSM6_FIFO_MODE_BYPASS     0x00   /**< FIFO 정지 + 내용 비움 */
#define LSM6_FIFO_MODE_CONTINUOUS 0x06   /**< 꽉 차면 가장 오래된 것을 밀어냄 */
#define LSM6_FIFO_NO_DECIMATION   0x09   /**< 자이로 001, 가속도 001 = 둘 다 감쇠 없음 */
/* 한 샘플이 FIFO에서 차지하는 워드 수: Gx,Gy,Gz,Ax,Ay,Az */
#define LSM6_FIFO_WORDS_PER_SAMPLE 6U
#define LSM6_FIFO_STATUS2_OVERRUN  0x40

/**@brief IMU 고정 핀 초기화 + 전원 ON (mode3: SCK idle HIGH, CS active-low) */
static void lsm6_pins_init(void)
{
    /* ① 먼저 SPI 핀을 idle 상태로 확정한다. 전원보다 이게 앞이어야 한다.
     *
     * ⚠️ 2026-07-27 수정. 아래 전원 사이클을 추가하면서 이 블록이 전원 뒤에 있었다.
     *    그러면 레일이 올라오는 순간 CS/SCK/MOSI가 아직 설정 전이라 high-Z(입력)로
     *    떠 있다. LSM6DSL은 전원 상승 시점의 CS 상태로 인터페이스 모드를 정하는
     *    부품이라, CS가 풀업 없이 떠 있으면 SPI로 안 잡히는 상태로 latch될 수 있다.
     *    증상이 정확히 WHO_AM_I=0xFF/0x00이고, 그렇게 latch되면 재시도 10회로도
     *    안 풀린다(전원을 다시 사이클해야 풀린다).
     *
     * CS를 레일이 내려간 동안에도 HIGH로 구동하는 것은 의도한 것이다. 센서 입력에
     * 다이오드를 통한 미세 back-power가 생기지만 마이크로암페어 수준이고, 모드가
     * 잘못 latch되는 쪽이 훨씬 비싸다. 이게 걱정되면 CS를 PULLUP 입력으로 두고
     * 레일이 선 뒤 출력으로 승격시킬 것 — 어느 쪽이든 핵심은 "전원 상승 순간에
     * CS가 확실히 HIGH"다. */
    nrf_gpio_cfg_output(IMU_SCK);
    nrf_gpio_cfg_output(IMU_MOSI);
    nrf_gpio_cfg_output(IMU_CS);
    nrf_gpio_cfg_input(IMU_MISO, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(IMU_SCK);                        // mode3 idle HIGH
    nrf_gpio_pin_clear(IMU_MOSI);
    nrf_gpio_pin_set(IMU_CS);                         // CS 비활성 = HIGH를 먼저 확보

    /* ② 그 다음 센서 전원 레일(TPS7A0230, EN=P0.23)을 확실히 껐다가 켠다.
     * 켜기만 하면, recover/소프트 리셋 직후 LSM6DSL이 이상 상태로 latch됐을 때
     * 풀리지 않는다(WHO_AM_I가 0xFF/0x00만 나옴). 물리 전원을 못 끊는 상황에서도
     * EN을 내렸다 올리면 레귤레이터 출력이 끊겨 센서가 진짜 전원 사이클을 받는다.
     * 100ms는 출력 커패시터가 방전되기에 충분하다.
     * (2026-07-24: recover 후 IMU/마이크가 다 죽고 소프트 리셋으로 안 풀린 사례) */
    nrf_gpio_cfg_output(IMU_PWR);
    nrf_gpio_pin_clear(IMU_PWR);                      // 센서 전원 OFF (EN=LOW)
    nrf_delay_ms(100);                                // 레일 방전 대기
    nrf_gpio_pin_set(IMU_PWR);                        // 센서 전원 ON
    /* 20ms는 LSM6DSL의 전원 상승 + 부팅(t_boot)에 부족하다. 50ms로 잡으면
     * lsm6_init()의 첫 WHO_AM_I 시도가 대개 한 번에 성공한다. */
    nrf_delay_ms(50);
}

/**@brief 레지스터 1개 쓰기 (write: 최상위 비트=0) */
static void lsm6_write_reg(uint8_t reg, uint8_t val)
{
    nrf_gpio_pin_clear(IMU_CS);
    nrf_delay_us(2);
    (void)bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, reg & 0x7F, 1);
    (void)bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, val,        1);
    nrf_gpio_pin_set(IMU_CS);
    nrf_delay_us(2);
}

/**@brief 가속도계 + 자이로 ODR 설정. FIFO도 같은 ODR로 맞춘다.
 *
 * @details ODR 필드는 CTRL1_XL / CTRL2_G / FIFO_CTRL5에서 같은 인코딩을 쓴다.
 *          FIFO ODR이 센서 ODR보다 낮으면 샘플이 조용히 버려지므로 반드시 같이 바꾼다.
 *          FIFO를 쓰기 때문에 RTC 폴링 주기와 ODR을 맞출 필요가 없어졌다 —
 *          6664Hz처럼 app_timer로는 만들 수 없던 레이트가 그래서 가능해졌다.
 */
static bool lsm6_set_rate(uint16_t odr_hz)
{
    uint8_t odr_bits;

    switch (odr_hz)
    {
        case 104:  odr_bits = 0x40; break;
        case 208:  odr_bits = 0x50; break;
        case 416:  odr_bits = 0x60; break;
        case 833:  odr_bits = 0x70; break;
        case 1666: odr_bits = 0x80; break;
        case 3333: odr_bits = 0x90; break;
        case 6664: odr_bits = 0xA0; break;
        default: return false;
    }

    /* 측정 범위 유지: 가속도 ±16g(0x04), 자이로 ±2000dps(0x0C). */
    lsm6_write_reg(LSM6_CTRL1_XL, (uint8_t)(odr_bits | 0x04));
    lsm6_write_reg(LSM6_CTRL2_G,  (uint8_t)(odr_bits | 0x0C));
    m_imu_odr_hz   = odr_hz;
    m_imu_odr_bits = odr_bits;
    return true;
}

/**@brief FIFO를 비우고 연속 모드로 다시 켠다.
 *
 * @details bypass를 한 번 거쳐야 내용이 비워지고 패턴 카운터가 0으로 돌아간다.
 *          그래야 배수 쪽에서 "6워드 = 한 샘플" 경계를 믿을 수 있다.
 *          ODR_FIFO는 센서 ODR과 동일하게 둔다(감쇠 없음).
 */
static void lsm6_fifo_start(void)
{
    lsm6_write_reg(LSM6_FIFO_CTRL5, LSM6_FIFO_MODE_BYPASS);
    lsm6_write_reg(LSM6_FIFO_CTRL1, 0x00);
    lsm6_write_reg(LSM6_FIFO_CTRL2, 0x00);
    lsm6_write_reg(LSM6_FIFO_CTRL3, LSM6_FIFO_NO_DECIMATION);
    lsm6_write_reg(LSM6_FIFO_CTRL4, 0x00);
    /* ODR_FIFO는 [6:3]에 들어간다. CTRL1_XL의 ODR은 [7:4]이므로 한 칸 내린다. */
    lsm6_write_reg(LSM6_FIFO_CTRL5,
                   (uint8_t)((m_imu_odr_bits >> 1) | LSM6_FIFO_MODE_CONTINUOUS));
}

static void lsm6_fifo_stop(void)
{
    lsm6_write_reg(LSM6_FIFO_CTRL5, LSM6_FIFO_MODE_BYPASS);
}

/**@brief 레지스터 1개 읽기 (read: 최상위 비트=1) */
static uint8_t lsm6_read_reg(uint8_t reg)
{
    nrf_gpio_pin_clear(IMU_CS);
    nrf_delay_us(2);
    (void)bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, reg | 0x80, 1);
    uint8_t v = bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, 0x00, 1);
    nrf_gpio_pin_set(IMU_CS);
    nrf_delay_us(2);
    return v;
}

/* lsm6_read_multi()도 삭제됐다. 유일한 사용처가 lsm6_read_sample이었고,
 * FIFO 연속 읽기는 lsm6_fifo_drain 안에서 직접 CS를 잡는다. */

/**@brief IMU 초기화: 전원+핀, WHO_AM_I 확인, ODR/범위 설정. 성공 시 true. */
static bool lsm6_init(void)
{
    lsm6_pins_init();

    /* 전원 레일이 설 때까지 몇 번 다시 물어본다. 한 번에 포기하면 아래 CTRL 설정을
     * 건너뛰어 센서가 전원차단 상태로 남고, 그 세션 내내 IMU가 0만 뱉는다.
     * 실제로 겪었다: 데이터는 조용히 전부 0인데 스트리밍은 멀쩡해 보여서
     * 원인을 찾기 어려웠다. 리셋 한 번으로 정상 복귀했으므로 타이밍 문제다. */
    uint8_t id = 0;
    for (int attempt = 0; attempt < IMU_INIT_ATTEMPTS; attempt++)
    {
        nrf_delay_ms(50);
        id = lsm6_read_reg(LSM6_WHO_AM_I);
        if (id == LSM6_WHO_AM_I_VAL)
        {
            break;
        }
        NRF_LOG_WARNING("LSM6DSL WHO_AM_I=0x%02X, retry %d", id, attempt + 1);
    }

    if (id != LSM6_WHO_AM_I_VAL)
    {
        NRF_LOG_ERROR("LSM6DSL not found after %d attempts (WHO_AM_I=0x%02X, expect 0x6A)",
                      IMU_INIT_ATTEMPTS, id);
        return false;
    }

    lsm6_write_reg(LSM6_CTRL3_C, 0x01);     // SW_RESET
    nrf_delay_ms(20);
    lsm6_write_reg(LSM6_CTRL3_C, 0x44);     // BDU=1, IF_INC=1
    /* 측정 범위를 넓게 연다. ±2g/±245dps에서는 손톱 두드림 충격이 범위를 넘어
     * 값이 잘려나갔다(실측: 가속도 Y축의 12.4%가 ±2g에 붙음). 잘린 구간은 실제
     * 크기를 역산할 수 없으므로 "얼마나 세게 쳤나"를 못 본다.
     * 16비트라 범위를 넓혀도 분해능은 충분하다(±16g에서 0.488 mg/LSB).
     *   CTRL1_XL = ODR[7:4] | FS_XL[3:2]   ODR 416Hz=0110, FS ±16g=01  -> 0x64
     *   CTRL2_G  = ODR[7:4] | FS_G[3:2]    ODR 416Hz=0110, FS 2000dps=11 -> 0x6C
     * 값을 바꾸면 앱의 NailProtocol 변환 계수도 같이 바꿔야 한다. */
    (void)lsm6_set_rate(IMU_DEFAULT_ODR_HZ);
    NRF_LOG_INFO("LSM6DSL init OK (WHO_AM_I=0x6A)");
    return true;
}

/* lsm6_read_sample()은 삭제됐다. 샘플 단위 폴링 읽기는 FIFO 배수로 대체됐고,
 * 둘을 섞으면 같은 샘플을 두 번 세거나 FIFO 정렬이 깨진다. */

/**@brief FIFO에 안 읽고 쌓여 있는 16비트 워드 수. 오버런이면 카운터도 올린다. */
static uint16_t lsm6_fifo_diff(void)
{
    uint8_t lo = lsm6_read_reg(LSM6_FIFO_STATUS1);
    uint8_t hi = lsm6_read_reg(LSM6_FIFO_STATUS2);

    /* 오버런 = 우리가 늦게 빼가서 센서가 오래된 샘플을 덮어썼다는 뜻.
     * 몇 개를 잃었는지는 센서가 알려주지 않으므로 횟수만 센다. STAT/HEALTH로 나간다.
     *
     * 2026-07-27: 카운터만 올리고 끝내면 안 된다. ST 문서는 Continuous 모드에서
     * 오버런이 나면 FIFO를 Bypass로 내렸다 되돌려 정렬을 되찾으라고 한다.
     * 안 하면 패턴 위상이 어긋난 채로 배수가 이어져 축이 통째로 밀린 데이터가
     * 나오는데, 그럴듯해 보여서 안 들킨다. 재정렬은 이번 배수를 마친 뒤에
     * 한다(지금 당장 FIFO를 비우면 이미 유효한 샘플까지 버린다). */
    if (hi & LSM6_FIFO_STATUS2_OVERRUN)
    {
        m_imu_fifo_ovr++;
        m_imu_fifo_realign = true;
    }
    return (uint16_t)(((uint16_t)(hi & 0x07U) << 8) | lo);
}


/**@brief 레지스터만 다시 써서 센서를 되살려 본다. 지연 없음(1ms 미만).
 *
 * @details 전원은 멀쩡한데 설정만 날아간 경우(전원 글리치, 비트뱅 위상 어긋남,
 *          FIFO 데드락)를 여기서 잡는다. WHO_AM_I가 안 나오면 실패로 보고
 *          호출자가 전원 사이클로 올라간다.
 */
static bool lsm6_soft_recover(void)
{
    if (lsm6_read_reg(LSM6_WHO_AM_I) != LSM6_WHO_AM_I_VAL)
    {
        return false;   /* 버스나 전원 문제 — 레지스터를 써봐야 소용없다 */
    }

    lsm6_write_reg(LSM6_CTRL3_C, 0x44);            /* BDU=1, IF_INC=1 */
    (void)lsm6_set_rate(m_imu_odr_hz);             /* ODR/FS 재설정 */
    lsm6_fifo_start();                             /* bypass -> continuous */
    return true;
}


/**@brief 스톨을 감지했을 때 복구를 시작한다.
 *
 * @details 소프트 복구를 IMU_SOFT_RECOVER_MAX번까지 시도하고, 그래도 안 되면
 *          전원 사이클 상태기계를 띄운다. 전원 사이클은 여기서 시작만 하고
 *          진행은 imu_recover_service()가 메인 루프에서 나눠서 한다.
 */
static void imu_stall_recover_begin(void)
{
    m_imu_idle_drains = 0;
    m_imu_stall_cnt++;

    if (m_imu_soft_tries < IMU_SOFT_RECOVER_MAX)
    {
        m_imu_soft_tries++;
        if (lsm6_soft_recover())
        {
            NRF_LOG_WARNING("IMU stall -> soft recover OK (try %u)", m_imu_soft_tries);
            return;
        }
        NRF_LOG_WARNING("IMU stall -> soft recover failed (try %u)", m_imu_soft_tries);
    }

    /* 하드 복구: 전원 레일을 내린다. 나머지는 imu_recover_service()가 이어받는다.
     * 이 구간 동안 m_imu_ok를 내려 배수와 송신이 센서를 건드리지 않게 한다.
     * 마이크는 계속 돈다 — 그게 이 상태기계를 만든 이유다. */
    NRF_LOG_ERROR("IMU stall -> power cycle");
    m_imu_recover_cnt++;
    m_imu_ok           = false;
    m_imu_soft_tries   = 0;
    m_imu_rcv_stage    = IMU_RCV_WAIT_OFF;
    m_imu_rcv_deadline = (app_timer_cnt_get() + IMU_PWR_OFF_TICKS) & RTC_COUNTER_MASK;

    nrf_gpio_pin_set(IMU_CS);        /* 레일이 내려가는 동안 CS는 HIGH로 확정 */
    nrf_gpio_pin_set(IMU_SCK);
    nrf_gpio_cfg_output(IMU_PWR);
    nrf_gpio_pin_clear(IMU_PWR);     /* EN=LOW */
}


/**@brief 전원 사이클 상태기계를 한 칸 진행한다. 메인 루프에서 매번 호출.
 *
 * @details 절대 블로킹하지 않는다. 각 단계는 app_timer 틱으로 만료를 보고,
 *          만료 전이면 즉시 돌아가 메인 루프가 BLE와 마이크를 계속 돌리게 한다.
 */
static void imu_recover_service(void)
{
    if (m_imu_rcv_stage == IMU_RCV_NONE)
    {
        return;
    }

    /* 24비트 RTC라 뺄셈 후 마스크로 남은 시간을 본다. 아직 안 됐으면 그냥 돌아간다. */
    uint32_t remaining = (m_imu_rcv_deadline - app_timer_cnt_get()) & RTC_COUNTER_MASK;
    if (remaining != 0 && remaining < (RTC_COUNTER_MASK / 2))
    {
        return;
    }

    if (m_imu_rcv_stage == IMU_RCV_WAIT_OFF)
    {
        nrf_gpio_pin_set(IMU_PWR);   /* EN=HIGH. 센서 부팅 대기로 넘어간다 */
        m_imu_rcv_stage    = IMU_RCV_WAIT_ON;
        m_imu_rcv_deadline = (app_timer_cnt_get() + IMU_PWR_BOOT_TICKS) & RTC_COUNTER_MASK;
        return;
    }

    /* IMU_RCV_WAIT_ON: 부팅이 끝났을 시각. WHO_AM_I를 한 번만 확인한다.
     * 여기서 실패해도 재시도 루프를 돌리지 않는다 — 다음 스톨 감지(2초 뒤)가
     * 자연스럽게 재시도가 되고, 그 편이 메인 루프를 안 잡는다. */
    m_imu_rcv_stage = IMU_RCV_NONE;

    if (lsm6_read_reg(LSM6_WHO_AM_I) != LSM6_WHO_AM_I_VAL)
    {
        /* ⚠️ 여기서 그냥 돌아가면 안 된다. m_imu_ok가 false라 lsm6_fifo_drain()은
         * 첫 줄에서 return하고, 그러면 스톨 카운터가 다시는 안 올라간다 —
         * 워치독을 달아놓고도 "한 번 복구에 실패하면 영구 사망"이 된다.
         * (호스트 시뮬레이션에서 잡은 버그. 센서가 10초간 죽었다 살아나는
         *  시나리오에서 복구 실패 후 영영 안 돌아왔다.)
         * 그래서 재시도를 여기서 직접 건다. 2초 백오프로 계속 두드리면
         * 센서가 언제 살아나든 그 뒤 2초 안에 자동 복귀한다. */
        NRF_LOG_ERROR("IMU power cycle: WHO_AM_I still bad - retry in 2s");
        nrf_gpio_pin_clear(IMU_PWR);
        m_imu_rcv_stage    = IMU_RCV_WAIT_OFF;
        m_imu_rcv_deadline = (app_timer_cnt_get() + IMU_RETRY_TICKS) & RTC_COUNTER_MASK;
        return;
    }

    /* SW_RESET은 일부러 안 한다. 방금 전원 사이클을 했으므로 이미 리셋 상태이고,
     * SW_RESET을 걸면 완료까지 20ms를 기다려야 해서 메인 루프를 다시 붙잡는다. */
    lsm6_write_reg(LSM6_CTRL3_C, 0x44);   /* BDU=1, IF_INC=1 */
    (void)lsm6_set_rate(m_imu_odr_hz);
    m_imu_ok = true;
    if (m_recording)
    {
        lsm6_fifo_start();
    }
    /* 원점은 다시 재야 한다. 복구 구간만큼 IMU가 비어 있으므로 옛 t0로는
     * 마이크와 겹칠 수 없다. SYNC를 다시 보내 앱이 새 원점을 쓰게 한다. */
    m_imu_t0_valid    = false;
    m_imu_idle_drains = 0;
    m_vt_imu          = 0;
    m_vt_mic          = 0;
    NRF_LOG_INFO("IMU recovered (WHO_AM_I=0x6A)");
}

/**@brief 다음에 나올 워드가 한 샘플(6워드) 안에서 몇 번째인지. */
static uint16_t lsm6_fifo_pattern(void)
{
    uint8_t lo = lsm6_read_reg(LSM6_FIFO_STATUS3);
    uint8_t hi = lsm6_read_reg(LSM6_FIFO_STATUS4);
    return (uint16_t)(((uint16_t)(hi & 0x03U) << 8) | lo);
}

/**@brief 쌓인 FIFO를 한 번의 CS 구간에서 몰아 읽어 IMU 링에 넣는다.
 *
 * @details 메인 루프에서만 호출한다(비트뱅잉은 ISR 금지).
 *          FIFO_DATA_OUT에서 연속으로 읽으면 주소가 L↔H를 오가며 FIFO가 계속
 *          흘러나온다. 그래서 샘플 하나당 CS를 토글하던 비용이 사라진다.
 *
 *          시작 위치가 샘플 경계가 아닐 수 있어(패턴 != 0) 남는 워드를 먼저
 *          버린다. 이걸 안 하면 축이 통째로 밀려 그럴듯하지만 완전히 틀린
 *          데이터가 나온다.
 */
static void lsm6_fifo_drain(void)
{
    /* 복구 중이면 센서를 건드리지 않는다. 레일이 내려가 있는 동안 SPI를 때리면
     * 의미 없는 값을 읽고, 최악의 경우 back-power로 복구 자체를 방해한다. */
    if (!m_imu_ok || m_imu_rcv_stage != IMU_RCV_NONE)
    {
        return;
    }

    uint16_t words = lsm6_fifo_diff();
    if (words < LSM6_FIFO_WORDS_PER_SAMPLE)
    {
        /* 빈손. 정상이라면 최저 ODR 104Hz에서도 배수 1024Hz 기준 10번에 한 번은
         * 샘플이 나온다. 2048번(약 2초) 연속으로 빈손이면 센서가 죽은 것이다.
         * 이 검사가 없으면 여기서 조용히 return만 반복하며 영원히 안 돌아온다 —
         * 2026-07-24 폰 녹음 5건이 전부 그 상태였다. */
        if (m_recording && ++m_imu_idle_drains >= IMU_STALL_DRAINS)
        {
            imu_stall_recover_begin();
        }
        return;
    }
    m_imu_idle_drains = 0;
    m_imu_soft_tries  = 0;   /* 한 번이라도 정상 배수되면 복구 시도 횟수를 되돌린다 */

    uint16_t phase = (uint16_t)(lsm6_fifo_pattern() % LSM6_FIFO_WORDS_PER_SAMPLE);
    uint16_t skip  = phase ? (uint16_t)(LSM6_FIFO_WORDS_PER_SAMPLE - phase) : 0U;

    if (words < (uint16_t)(skip + LSM6_FIFO_WORDS_PER_SAMPLE))
    {
        return;   // 정렬하고 나면 온전한 샘플이 안 남는다. 다음 주기에 다시 본다
    }

    uint16_t samples = (uint16_t)((words - skip) / LSM6_FIFO_WORDS_PER_SAMPLE);
    if (samples > IMU_DRAIN_MAX_SAMPLES)
    {
        samples = IMU_DRAIN_MAX_SAMPLES;
    }

    /* 첫 배수가 IMU 스트림의 원점을 알려준다. FIFO에 쌓여 있던 것들은 지금 이 순간이
     * 아니라 그 이전에 센서 클럭이 찍은 샘플이므로, 가장 오래된 것(=샘플 0)은 지금에서
     * (samples-1)/ODR 만큼 앞이다. */
    if (!m_imu_t0_valid && samples > 0U && m_imu_odr_hz > 0U)
    {
        uint32_t span_ticks = ((uint32_t)(samples - 1U) * APP_TIMER_CLOCK_HZ) / m_imu_odr_hz;
        m_imu_t0       = (app_timer_cnt_get() - span_ticks) & RTC_COUNTER_MASK;
        m_imu_t0_valid = true;
        if (m_mic_t0_valid) m_sync_report = true;
    }

    nrf_gpio_pin_clear(IMU_CS);
    nrf_delay_us(2);
    (void)bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, LSM6_FIFO_DATA_L | 0x80, 1);

    for (uint16_t i = 0; i < (uint16_t)(skip * 2U); i++)
    {
        (void)bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, 0x00, 1);
    }

    for (uint16_t n = 0; n < samples; n++)
    {
        uint8_t b[12];
        for (uint16_t i = 0; i < 12; i++)
        {
            b[i] = bb_txrx(IMU_SCK, IMU_MOSI, IMU_MISO, 0x00, 1);
        }

        /* 링이 꽉 찼으면 가장 오래된 샘플을 밀어낸다(생산·소비가 같은 컨텍스트라
         * 경합 없음). head는 그대로 절대 샘플 번호로 쓰인다. */
        if ((m_imu_head - m_imu_tail) >= IMU_RING_SAMPLES)
        {
            m_imu_tail++;
            m_imu_dropped++;
        }

        /* FIFO 순서는 0x22부터의 연속 읽기와 같다: 자이로 3축 → 가속도 3축. */
        imu_sample_t *s = &m_imu_ring[m_imu_head & (IMU_RING_SAMPLES - 1U)];
        s->gx = (int16_t)(b[0]  | (b[1]  << 8));
        s->gy = (int16_t)(b[2]  | (b[3]  << 8));
        s->gz = (int16_t)(b[4]  | (b[5]  << 8));
        s->ax = (int16_t)(b[6]  | (b[7]  << 8));
        s->ay = (int16_t)(b[8]  | (b[9]  << 8));
        s->az = (int16_t)(b[10] | (b[11] << 8));
        m_imu_head++;
    }

    nrf_gpio_pin_set(IMU_CS);
    nrf_delay_us(2);

    /* 오버런이 있었으면 이제 FIFO를 재정렬한다. 위에서 읽은 샘플은 이미 링에
     * 안전하게 들어갔고, 여기서 버리는 건 우리가 못 따라간 잔량뿐이다. */
    if (m_imu_fifo_realign)
    {
        m_imu_fifo_realign = false;
        lsm6_fifo_start();
    }
}

/**@brief 센서 WHO_AM_I 스캔을 실행하고 결과를 BLE(NUS)로 전송한다. (PC가 'w' 명령) */
static void scan_dump_over_ble(void)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Not connected - cannot scan.");
        return;
    }

    // 센서 전원 ON (P0.23 = TPS LDO enable)
    nrf_gpio_cfg_output(IMU_PWR);
    nrf_gpio_pin_set(IMU_PWR);
    nrf_delay_ms(150);
    ble_send_str("SCAN (expect WHO_AM_I=0x6A)\n");

    // 확정 핀맵(SCK3/MOSI5/MISO6/CS2, mode3)으로 WHO_AM_I를 읽어 센서 연결 점검
    uint8_t id = bitbang_whoami(IMU_SCK, IMU_MOSI, IMU_MISO, IMU_CS, 1);

    char line[72];
    snprintf(line, sizeof(line), "SCK%d MOSI%d MISO%d CS%d m3(bb) = 0x%02X%s\n",
             IMU_SCK, IMU_MOSI, IMU_MISO, IMU_CS, id,
             (id == 0x6A) ? "  <<< OK" : "  (연결 확인 필요)");
    ble_send_str(line);
    ble_send_str("END\n");
}

int main(void)
{
    /* ISP1807의 nRF52840 Rev3는 구형 SDK에서 부팅 시 APPROTECT가
     * 자동 활성화될 수 있다. 개발/RTT 진단 중에는 소프트웨어 해제한다. */
    *((volatile uint32_t *)0x40000558UL) = 0x5A;

    bool erase_bonds;

    // Initialize.
    // uart_init();   // UART 끔: P0.06(MISO) 등 센서 핀을 UART가 점유하지 않도록
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    // printf("\r\nUART started.\r\n");   // UART 껐으므로 제거
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();

    // IMU(LSM6DSL) 초기화 — 전원 ON + WHO_AM_I 확인 + ODR 설정
    /* 실패해도 스트리밍은 계속된다(마이크는 살아 있으므로). 대신 상태를 들고
     * 있다가 RATE 줄에 실어 보내, 수신측이 "IMU가 0인 이유"를 알 수 있게 한다. */
    m_imu_ok = lsm6_init();
    mic_init();

    // Enter main loop.
    for (;;)
    {
        /* IMU 레지스터를 건드리는 일은 전부 여기(메인 루프)에서 한다.
         * BLE 이벤트 핸들러에서 비트뱅잉을 하면 SoftDevice 타이밍이 깨진다. */
        if (m_mic_rate_request != 0)
        {
            uint8_t requested = m_mic_rate_request;
            m_mic_rate_request = 0;

            /* PDM 하드웨어 레이트는 항상 16000Hz다. 이 값은 소프트웨어 데시메이션에만
             * 쓰이므로 PDM을 세웠다 켤 필요가 없다(그게 예전 BUSY 리셋의 원인이었다).
             * 다음 PDM 버퍼부터 새 계수가 적용된다. */
            uint16_t decim = mic_decim_from_khz(requested);
            if (decim != 0)
            {
                m_mic_decim = decim;
                stream_reset_all();
                NRF_LOG_INFO("=> Mic rate set: %u kHz (decim=%u)", requested, decim);
            }
            else
            {
                NRF_LOG_WARNING("=> Mic rate %u kHz rejected", requested);
            }
            m_rate_report = true;   // 성공이든 실패든 현재 레이트를 알려준다
        }

        if (m_imu_rate_request != 0)
        {
            uint16_t requested = m_imu_rate_request;
            m_imu_rate_request = 0;

            if (m_imu_ok && lsm6_set_rate(requested))
            {
                stream_reset_all();
                NRF_LOG_INFO("=> IMU rate set: odr=%u Hz", m_imu_odr_hz);
            }
            else
            {
                NRF_LOG_WARNING("=> IMU rate %u Hz rejected", requested);
            }
            m_rate_report = true;   // 성공이든 실패든 현재 레이트를 알려준다
        }

        if (m_profile_request != 0xFF)
        {
            uint8_t requested = m_profile_request;
            m_profile_request = 0xFF;

            if (requested <= STREAM_PROFILE_MAX)
            {
                m_profile = requested;
                stream_reset_all();
                NRF_LOG_INFO("=> Profile set: %u", requested);
            }
            else
            {
                NRF_LOG_WARNING("=> Profile %u rejected", requested);
            }
            m_rate_report = true;   // 성공이든 실패든 현재 프로파일을 알려준다
        }

        if (m_latency_request != 0)
        {
            uint16_t requested = m_latency_request;
            m_latency_request = 0;

            /* 링 리셋이 필요 없다 — 예산은 '얼마나 오래 붙들고 있을지'만 정하고
             * 인덱스 체계는 건드리지 않는다. 그래서 녹음 도중에 바꿔도 CSV의
             * 절대 번호가 끊기지 않는다(레이트·프로파일 변경과 다른 점). */
            if (requested >= STREAM_LATENCY_MIN_MS && requested <= STREAM_LATENCY_MAX_MS)
            {
                m_latency_budget_ms = requested;
                NRF_LOG_INFO("=> Latency budget set: %lu ms",
                             (unsigned long)m_latency_budget_ms);
            }
            else
            {
                NRF_LOG_WARNING("=> Latency budget %u ms rejected", requested);
            }
            /* 성공이든 실패든 현재 값은 HEALTH의 lat= 로 매초 나간다. 앱이 보낸
             * 값과 다르면 그 쓰기가 유실됐거나 거부된 것이다. */
        }

        if (m_stream_start_request)
        {
            m_stream_start_request = false;

            /* 's' 한 번에 해야 할 일이 전부 여기 모여 있다. 순서가 곧 원점 정렬이다:
             * 먼저 상태를 깨끗이 하고(누적 카운터까지), 그 다음 두 스트림을 연달아
             * 켠다. 사이에 BLE 이벤트가 끼어도 링 인덱스는 이미 안정된 뒤다. */
            m_imu_fifo_ovr    = 0;
            m_imu_stall_cnt   = 0;
            m_imu_recover_cnt = 0;
            m_imu_trimmed     = 0;
            m_mic_trimmed     = 0;
            m_stream_flushing = false;   /* 직전 세션의 flush 상태가 남으면 상한이 안 걸린다 */
            m_health_ticks    = 0;
            m_uptime_sec      = 0;
            m_health_report   = false;
            m_recording       = true;

            stream_reset_all();   // m_recording=true 이후여야 FIFO도 같이 비운다

            /* stream_reset_all()이 직전 세션의 잔여 생산량을 누적해 버리므로,
             * 세션 카운터는 반드시 그 뒤에 0으로 놓는다. */
            m_mic_total = 0;
            m_imu_total = 0;

            mic_record_start();

            ret_code_t err_code = app_timer_start(m_sample_timer,
                                                  IMU_DRAIN_PERIOD_TICKS,
                                                  NULL);
            APP_ERROR_CHECK(err_code);
            m_rate_report = true;   // 앱이 WAV 헤더를 쓰려면 실제 레이트를 알아야 한다
            NRF_LOG_INFO("=> Streaming STARTED");
        }

        if (m_sample_due)
        {
            m_sample_due = false;
            // 실제 센서 읽기는 메인 루프(thread 모드)에서 수행 (ISR 블로킹 방지)
            if (m_recording)
            {
                lsm6_fifo_drain();
            }
        }

        /* IMU 전원 사이클 복구를 한 칸 진행한다. 매 바퀴 부르지만 복구 중이
         * 아니면 즉시 돌아가므로 비용은 없다. 여기서 블로킹하지 않는 것이
         * 핵심 — 그래야 복구하는 동안에도 마이크와 BLE가 계속 돈다. */
        imu_recover_service();

        if (m_health_report)
        {
            m_health_report = false;
            health_report_send();
        }

        if (m_link_report)
        {
            m_link_report = false;
            link_report_send();    // 협상이 끝난 시점의 실제 링크 상태
        }

        if (m_sync_report)
        {
            m_sync_report = false;

            /* 같은 RTC로 잰 두 원점의 차이. 양수면 IMU 샘플 0이 마이크 샘플 0보다
             * 늦게 찍혔다는 뜻이다. 24비트 카운터라 뺄셈 후 부호를 되살린다. */
            int32_t diff_ticks = (int32_t)((m_imu_t0 - m_mic_t0) & RTC_COUNTER_MASK);
            if (diff_ticks > (int32_t)(RTC_COUNTER_MASK / 2))
            {
                diff_ticks -= (int32_t)(RTC_COUNTER_MASK + 1);
            }

            int32_t diff_us =
                (int32_t)(((int64_t)diff_ticks * 1000000) / APP_TIMER_CLOCK_HZ);

            char sync_line[64];
            snprintf(sync_line, sizeof(sync_line), "SYNC,imu_minus_mic_us=%ld\n",
                     (long)diff_us);
            ble_send_str(sync_line);
            NRF_LOG_INFO("=> stream origin offset: %ld us", (long)diff_us);
        }

        if (m_rate_report)
        {
            m_rate_report = false;
            char rate_line[96];
            snprintf(rate_line, sizeof(rate_line),
                     "RATE,mic_hz=%lu,imu_hz=%u,imu_drain_hz=%lu,imu_ok=%u,profile=%u\n",
                     (unsigned long)(MIC_BASE_RATE_HZ / m_mic_decim),
                     m_imu_odr_hz,
                     (unsigned long)(APP_TIMER_CLOCK_HZ / IMU_DRAIN_PERIOD_TICKS),
                     m_imu_ok ? 1U : 0U,
                     (unsigned)m_profile);
            ble_send_str(rate_line);
        }

        stream_service();          // 쌓인 만큼 실시간으로 흘려보낸다 (논블로킹)

        if (m_stop_request)
        {
            m_stop_request = false;

            /* 'e' 핸들러는 m_recording만 내렸다. 실제 정지는 시작과 대칭으로 여기서.
             * 생산을 먼저 다 끊은 뒤에 flush해야 stream_finish()의 종료 조건
             * (양쪽 링이 빔)이 뒤늦게 들어온 샘플로 다시 깨지지 않는다. */
            app_timer_stop(m_sample_timer);
            mic_record_stop();
            lsm6_fifo_drain();     // 센서에 남은 잔량까지 링으로 옮기고
            lsm6_fifo_stop();
            stream_finish();       // 잔량 flush + 상태 요약
        }
        if (m_scan_request)
        {
            m_scan_request = false;
            scan_dump_over_ble();  // 'w' 명령으로 요청된 센서 스캔 실행 + BLE 전송
        }
        idle_state_handle();
    }
}


/**
 * @}
 */
