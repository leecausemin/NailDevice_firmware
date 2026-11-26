/* main.c의 스케줄러/워치독 로직만 떼어내 호스트에서 돌려보는 하네스.
 * 목적: (1) 링크가 느려질 때 두 스트림이 비례 배분되는지
 *       (2) 마이크가 예전처럼 89% 죽지 않는지
 *       (3) 스톨 워치독이 마이크를 멈추지 않고 IMU를 되살리는지
 * 알고리즘은 main.c와 동일하게 옮겼다. 값이 다르면 main.c를 고친 것이다.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ---- main.c와 동일한 상수 ---- */
#define APP_TIMER_CLOCK_HZ      16384U
#define IMU_DRAIN_PERIOD_TICKS  16U
#define DRAIN_HZ                (APP_TIMER_CLOCK_HZ / IMU_DRAIN_PERIOD_TICKS)  /* 1024 */
#define IMU_DRAIN_MAX_SAMPLES   24U
#define MIC_CHUNK_COUNT         64U
#define IMU_RING_SAMPLES        8192U
#define IMU_CREDIT_ONE          1000
#define IMU_CREDIT_MAX          4000
#define IMU_STALL_DRAINS        2048U
#define IMU_SOFT_RECOVER_MAX    2U
#define PDM_BUFFER_SAMPLES      256
#define MIC_RATE_HZ             16000
static int STREAM_LATENCY_BUDGET_MS = 1000;
#define MIC_CHUNK_MS            (1000*PDM_BUFFER_SAMPLES/MIC_RATE_HZ)
static int g_trim_on;
static double g_imu_lag_sum, g_mic_lag_sum, g_off_sum, g_imu_lag_max, g_off_max; static long g_lag_n;
static uint32_t m_imu_trimmed, m_mic_trimmed;

/* 프로파일 5(BothMic8): IMU 12bit 6축 26샘플/패킷, 마이크 u-law 236샘플/패킷 */
#define IMU_SAMPLES_PER_PKT     26
#define MIC_SAMPLES_PER_PKT     236

/* ---- 상태 ---- */
static uint32_t m_imu_head, m_imu_tail, m_imu_dropped;
static uint32_t m_mic_head, m_mic_tail, m_mic_produced, m_mic_dropped;
static int32_t  m_imu_credit;
static uint32_t m_imu_idle_drains, m_imu_stall_cnt, m_imu_recover_cnt;
static uint8_t  m_imu_soft_tries;
static bool     m_imu_ok;
static int      m_rcv_stage;          /* 0=none 1=wait_off 2=wait_on */
static long     m_rcv_deadline_tick;

static uint32_t g_imu_sent, g_mic_sent;   /* 실제 전송된 샘플 수 */

/* 하드웨어 모형: 센서가 살아있는 구간에만 FIFO가 찬다 */
static double  g_fifo_accum;
static bool    g_sensor_alive;
static long    g_sensor_dead_from, g_sensor_dead_to;  /* 틱 구간 */
static long    g_stall_period_ticks, g_stall_len_ticks;  /* 링크가 주기적으로 멈춤 */

static void reset_all(void)
{
    memset(&m_imu_head, 0, 0);
    m_imu_head = m_imu_tail = m_imu_dropped = 0;
    m_mic_head = m_mic_tail = m_mic_produced = m_mic_dropped = 0;
    m_imu_credit = 0;
    m_imu_idle_drains = m_imu_stall_cnt = m_imu_recover_cnt = 0;
    m_imu_soft_tries = 0;
    m_imu_ok = true;
    m_rcv_stage = 0;
    g_imu_sent = g_mic_sent = 0;
    g_fifo_accum = 0;
    g_sensor_alive = true;
}

/* ---- lsm6_fifo_drain()의 링/크레딧 부분 ---- */
static void drain(int odr_hz, long tick)
{
    if (!m_imu_ok || m_rcv_stage != 0) return;

    /* 센서가 죽어 있으면 FIFO가 안 찬다 = words 0 */
    if (g_sensor_alive) g_fifo_accum += (double)odr_hz / DRAIN_HZ;

    uint32_t samples = (uint32_t)g_fifo_accum;
    if (samples < 1) {
        /* main.c: words < 6워드(=1샘플)이면 빈손 */
        if (++m_imu_idle_drains >= IMU_STALL_DRAINS) {
            /* imu_stall_recover_begin() */
            m_imu_idle_drains = 0;
            m_imu_stall_cnt++;
            if (m_imu_soft_tries < IMU_SOFT_RECOVER_MAX) {
                m_imu_soft_tries++;
                if (g_sensor_alive) return;   /* 소프트 복구 성공 */
            }
            m_imu_recover_cnt++;
            m_imu_ok = false;
            m_imu_soft_tries = 0;
            m_rcv_stage = 1;
            m_rcv_deadline_tick = tick + (100L * DRAIN_HZ) / 1000;  /* 100ms */
        }
        return;
    }
    m_imu_idle_drains = 0;
    m_imu_soft_tries  = 0;

    if (samples > IMU_DRAIN_MAX_SAMPLES) samples = IMU_DRAIN_MAX_SAMPLES;
    g_fifo_accum -= samples;

    for (uint32_t n = 0; n < samples; n++) {
        if ((m_imu_head - m_imu_tail) >= IMU_RING_SAMPLES) { m_imu_tail++; m_imu_dropped++; }
        m_imu_head++;
    }

    m_imu_credit += (int32_t)((samples * IMU_CREDIT_ONE) / IMU_SAMPLES_PER_PKT);
    if (m_imu_credit > IMU_CREDIT_MAX) m_imu_credit = IMU_CREDIT_MAX;
}

/* ---- imu_recover_service() ---- */
static void recover_service(long tick)
{
    if (m_rcv_stage == 0) return;
    if (tick < m_rcv_deadline_tick) return;

    if (m_rcv_stage == 1) {
        m_rcv_stage = 2;
        m_rcv_deadline_tick = tick + (60L * DRAIN_HZ) / 1000;   /* 60ms */
        return;
    }
    if (!g_sensor_alive) {
        /* main.c와 동일: 복구 실패 시 2초 백오프로 직접 재시도를 건다.
         * m_imu_ok가 false라 drain()이 스톨 카운터를 못 올리기 때문. */
        m_rcv_stage = 1;
        m_rcv_deadline_tick = tick + 2L * DRAIN_HZ;
        m_imu_recover_cnt++;
        return;
    }
    m_rcv_stage = 0;
    m_imu_ok = true;
    m_imu_idle_drains = 0;
}

/* ---- 마이크 생산 (PDM ISR) ---- */
static void mic_produce(long tick)
{
    /* 256샘플 청크가 16ms마다 = DRAIN_HZ/62.5 틱마다 */
    static double acc;
    acc += (double)MIC_RATE_HZ / DRAIN_HZ;
    while (acc >= PDM_BUFFER_SAMPLES) {
        acc -= PDM_BUFFER_SAMPLES;
        if ((m_mic_head - m_mic_tail) < MIC_CHUNK_COUNT) m_mic_head++;
        else m_mic_dropped += PDM_BUFFER_SAMPLES;
        m_mic_produced += PDM_BUFFER_SAMPLES;
    }
}

static void trim_backlog(int odr_hz)
{
    if (!g_trim_on) return;
    uint32_t max_s = (uint32_t)((long)odr_hz * STREAM_LATENCY_BUDGET_MS / 1000);
    uint32_t floor_s = IMU_SAMPLES_PER_PKT * 4;
    if (max_s < floor_s) max_s = floor_s;
    uint32_t lag = m_imu_head - m_imu_tail;
    if (lag > max_s) { uint32_t d = lag - max_s; m_imu_tail += d; m_imu_trimmed += d; }

    uint32_t max_c = (uint32_t)STREAM_LATENCY_BUDGET_MS / MIC_CHUNK_MS;
    if (max_c < 2) max_c = 2;
    uint32_t lc = m_mic_head - m_mic_tail;
    if (lc > max_c) { uint32_t d = lc - max_c; m_mic_tail += d; m_mic_trimmed += d*PDM_BUFFER_SAMPLES; }
}

/* ---- stream_service(). budget = 이번 틱에 링크가 받아줄 패킷 수 ---- */
/* 가중 공정 큐잉(WFQ). 각 스트림에 '가상 시각'을 두고, 패킷을 보낼 때마다
 * 그 패킷이 담은 '스트림 시간'만큼 앞으로 민다. 항상 가상 시각이 뒤처진 쪽을
 * 보내면 두 스트림이 자기 레이트에 비례해서 대역을 나눠 갖는다.
 * 나눗셈을 피하려고 서로의 레이트를 곱해 둔다:
 *   imu 1패킷 = imu_samples/imu_hz 초  ->  비용 imu_samples*mic_hz
 *   mic 1패킷 = mic_samples/mic_hz 초  ->  비용 mic_samples*imu_hz          */
static int32_t v_imu, v_mic;

static void stream_service_new(int *budget, int odr_hz)
{
    int32_t cost_imu = (int32_t)IMU_SAMPLES_PER_PKT * MIC_RATE_HZ;
    int32_t cost_mic = (int32_t)PDM_BUFFER_SAMPLES  * odr_hz;

    for (;;) {
        if (*budget <= 0) return;
        uint32_t imu_lag    = m_imu_head - m_imu_tail;
        uint32_t mic_chunks = m_mic_head - m_mic_tail;
        if (imu_lag == 0 && mic_chunks == 0) { v_imu = v_mic = 0; return; }

        /* 비어 있는 쪽은 크레딧을 쌓아두지 못하게 상대 시각으로 끌어올린다.
         * 안 그러면 잠깐 쉬었다 돌아온 스트림이 한동안 독점한다. */
        if (imu_lag == 0)    v_imu = v_mic;
        if (mic_chunks == 0) v_mic = v_imu;
        int32_t m = (v_imu < v_mic) ? v_imu : v_mic;   /* 정규화(오버플로 방지) */
        v_imu -= m; v_mic -= m;

        if (imu_lag != 0 && (mic_chunks == 0 || v_imu <= v_mic)) {
            uint32_t n = imu_lag > IMU_SAMPLES_PER_PKT ? IMU_SAMPLES_PER_PKT : imu_lag;
            m_imu_tail += n; g_imu_sent += n; (*budget)--;
            v_imu += cost_imu;
        } else {
            m_mic_tail++; g_mic_sent += PDM_BUFFER_SAMPLES; (*budget)--;
            v_mic += cost_mic;
        }
    }
}

/* ---- 예전 스케줄러 (마이크 우선) ---- */
static void stream_service_old(int *budget)
{
    while (m_mic_tail != m_mic_head) {
        if (*budget <= 0) return;
        m_mic_tail++; g_mic_sent += PDM_BUFFER_SAMPLES; (*budget)--;
    }
    while (m_imu_tail != m_imu_head) {
        if (*budget <= 0) return;
        uint32_t n = m_imu_head - m_imu_tail;
        if (n > IMU_SAMPLES_PER_PKT) n = IMU_SAMPLES_PER_PKT;
        m_imu_tail += n; g_imu_sent += n; (*budget)--;
    }
}

/* 마이크 1패킷 = 236샘플이지만 청크(256)를 소비하므로 근사. 대역 비교가 목적이라 충분. */

static void run(int seconds, int odr_hz, double link_pkt_per_s, bool use_new,
                long dead_from_s, long dead_to_s,
                double *imu_pct, double *mic_pct)
{
    reset_all(); v_imu = v_mic = 0;
    g_imu_lag_sum=g_mic_lag_sum=g_off_sum=g_imu_lag_max=g_off_max=0; g_lag_n=0; m_imu_trimmed=m_mic_trimmed=0;
    g_sensor_dead_from = dead_from_s * DRAIN_HZ;
    g_sensor_dead_to   = dead_to_s   * DRAIN_HZ;
    double budget_acc = 0;
    long total = (long)seconds * DRAIN_HZ;

    for (long tick = 0; tick < total; tick++) {
        g_sensor_alive = !(dead_to_s > dead_from_s &&
                           tick >= g_sensor_dead_from && tick < g_sensor_dead_to);
        mic_produce(tick);
        drain(odr_hz, tick);
        recover_service(tick);

        trim_backlog(odr_hz);
        {   double il = 1000.0*(m_imu_head-m_imu_tail)/odr_hz;
            double ml = 1000.0*(m_mic_head-m_mic_tail)*PDM_BUFFER_SAMPLES/MIC_RATE_HZ;
            g_imu_lag_sum+=il; g_mic_lag_sum+=ml; g_off_sum+=fabs(il-ml); g_lag_n++;
            if(il>g_imu_lag_max)g_imu_lag_max=il;
            if(fabs(il-ml)>g_off_max)g_off_max=fabs(il-ml); }
        int link_up = 1;
        if (g_stall_period_ticks > 0 && (tick % g_stall_period_ticks) < g_stall_len_ticks) link_up = 0;
        budget_acc += link_up ? (link_pkt_per_s / DRAIN_HZ) : 0.0;
        int budget = (int)budget_acc;
        budget_acc -= budget;
        if (budget > 0) {
            if (use_new) stream_service_new(&budget, odr_hz);
            else         stream_service_old(&budget);
        }
    }
    double imu_expect = (double)odr_hz * seconds;
    /* 센서가 죽어 있던 구간은 애초에 안 나오는 게 맞으므로 기대치에서 뺀다 */
    if (dead_to_s > dead_from_s) imu_expect -= (double)odr_hz * (dead_to_s - dead_from_s);
    *imu_pct = 100.0 * g_imu_sent / imu_expect;
    *mic_pct = 100.0 * g_mic_sent / ((double)MIC_RATE_HZ * seconds);
}

int main(void)
{
    printf("=== 최종 확인: 상한 1000ms, 낮은 레이트 포함 (10초마다 1초 멈춤, 링크 여유 130%%) ===\n\n");
    printf("%-9s | %-22s | %-22s\n","IMU ODR","상한 없음","상한 1000ms");
    printf("%-9s | %-9s %-12s | %-9s %-12s\n","","IMU수신률","최대 지연","IMU수신률","최대 지연");
    printf("---------------------------------------------------------------------\n");
    int odrs[]={6664,3333,1666,416};
    for(unsigned i=0;i<sizeof(odrs)/sizeof(odrs[0]);i++){
        int demand = odrs[i]/IMU_SAMPLES_PER_PKT + MIC_RATE_HZ/MIC_SAMPLES_PER_PKT;
        double a,x; double lag_off, rate_off;
        g_stall_period_ticks=10*DRAIN_HZ; g_stall_len_ticks=1*DRAIN_HZ;
        g_trim_on=0; run(60,odrs[i],demand*1.30,true,0,0,&a,&x); rate_off=a; lag_off=g_imu_lag_max;
        g_trim_on=1; run(60,odrs[i],demand*1.30,true,0,0,&a,&x);
        printf("%-9d | %8.1f%% %9.0f ms | %8.1f%% %9.0f ms\n",
               odrs[i], rate_off, lag_off, a, g_imu_lag_max);
    }
    printf("\n=== 링크가 안정적일 때 (멈춤 없음) ===\n");
    for(unsigned i=0;i<sizeof(odrs)/sizeof(odrs[0]);i++){
        int demand = odrs[i]/IMU_SAMPLES_PER_PKT + MIC_RATE_HZ/MIC_SAMPLES_PER_PKT;
        double a,x; g_stall_period_ticks=0; g_trim_on=1;
        run(30,odrs[i],demand*1.05,true,0,0,&a,&x);
        printf("%5d Hz: IMU %.1f%%  MIC %.1f%%  지연 %.0f ms  버린양 %lu\n",
               odrs[i],a,x,g_imu_lag_max,(unsigned long)m_imu_trimmed);
    }
    return 0;
}
