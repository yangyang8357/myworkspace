#include "xil_types.h"
#include "xil_io.h"
#include <math.h>
#include <string.h>

// ����ʱ����س���
#define CLOCK_FREQ_HZ       125000000  // ʱ�ӻ���Ƶ�� (125MHz)
#define NS_PER_SECOND       1000000000 // ÿ��������
#define STEP_COUNTER_BITS   32         // ����������λ��
#define STEP_COUNTER_MAX    0xFFFFFFFF // �������������ֵ
#define STEP_COUNTER_DEFAULT 0x00800000 // ����������Ĭ��ֵ
#define NS_PER_STEP         8          // ÿ�����������������

// ����ʱ�����ڶ�Ӧ��������
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// ��Ϣ����ö��
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// ����ʱ��ͬ���������ṹ��
typedef struct {
    double time_offset;          // ʱ��ƫ�������ӻ�-���������룩
    UINT64 last_sync_time;       // �ϴ�ͬ��ʱ��
    double slow_threshold;       // �ӻ���������ʱ����ֵ��1�� = 1e9���룩
    double step_levels[3];       // ��������ֵ
    double step_thresholds[3];   // �����л���ֵ
    double smooth_factor;        // ƽ������
    double max_freq_adjust;      // ���Ƶ�ʵ�����
    double base_frequency;       // ����Ƶ�ʣ�Hz��
    double current_frequency;    // ��ǰƵ��
    UINT32 adjustment_count;     // ��������ͳ��
    double adjustment_history[100]; // ���100�ε�����ʷ
    
    // �ȶ��Լ��ָ��
    double stability_mean;       // ����ƽ��ֵ
    double stability_std_dev;    // ������׼��
    int    stability_count;      // ��Ч������
    int    stability_warning;    // �ȶ��Ծ����־
} TimeSyncController;

// SequenceID���ṹ��
typedef struct {
    UINT16 expected_seq_id[6];   // ÿ����Ϣ������������һ��SequenceID
    UINT16 last_valid_seq_id[6]; // ÿ����Ϣ�������һ����Ч��SequenceID
    UINT32 missed_count[6];      // ÿ����Ϣ���Ͷ�ʧ����Ϣ��
    UINT32 duplicate_count[6];   // ÿ����Ϣ�����ظ�����Ϣ��
    UINT32 invalid_count[6];     // ÿ����Ϣ������Ч����Ϣ��
    UINT64 last_checked_time[6]; // �ϴμ��ʱ��
} SeqIdChecker;

// TX Buffer״̬�ṹ��
typedef struct {
    BOOL isBusy[5];              // ÿ��TX Buffer��æ״̬
    UINT32 msgLength[5];         // ÿ������������Ϣ����
    UINT64 lastTxTime[5];        // ÿ���������������ʱ��
    UINT32 txCount[5];           // ÿ���������ķ��ͼ���
} TXBufferStatus;

// ϵͳͳ�ƽṹ��
typedef struct {
    UINT32 total_tx_messages;    // ���͵�����Ϣ��
    UINT32 total_rx_messages;    // ���յ�����Ϣ��
    UINT32 tx_error_count;       // ���ʹ������
    UINT32 rx_error_count;       // ���մ������
    UINT32 tx_bytes;             // ���͵����ֽ���
    UINT32 rx_bytes;             // ���յ����ֽ���
    UINT32 ptp_message_types[6]; // ����PTP��Ϣ����
    UINT64 last_stats_time;      // �ϴ�ͳ��ʱ��
} SystemStats;

// ȫ�ֱ�������
extern UINT32 TSN_BASEADDR;      // TSN����������ַ
TimeSyncController g_TimeSyncCtrl;
SeqIdChecker g_SeqIdChecker;
TXBufferStatus g_TxBufferStatus;
SystemStats g_Stats;

// I/O��������
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
}

// ��ȡϵͳʱ�䣨���룩
UINT64 GetSystemTime() {
    UINT32 high = Read32(TSN_BASEADDR + RTC_SECONDS_MSB_FIELD_OFFSET);
    UINT32 low = Read32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET);
    UINT32 step_counter = Read32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET);
    
    // ������������ֵת��Ϊʵ������ֵ
    UINT64 nanoseconds = (UINT64)step_counter * NS_PER_STEP;
    
    // ���������벿��
    UINT64 seconds = ((UINT64)high << 32) | low;
    UINT64 total_nanoseconds = (seconds * NS_PER_SECOND) + nanoseconds;
    
    return total_nanoseconds;
}

// ��ʼ��ʱ��ͬ��������
void InitTimeSyncController() {
    g_TimeSyncCtrl.slow_threshold = 1000000000;  // 1�루���룩
    
    // �༶��������
    g_TimeSyncCtrl.step_levels[0] = 0.0001;      // �󲽳���100ppm��
    g_TimeSyncCtrl.step_levels[1] = 0.00001;     // �в�����10ppm��
    g_TimeSyncCtrl.step_levels[2] = 0.000001;    // С������1ppm��
    
    g_TimeSyncCtrl.step_thresholds[0] = 5000000000;  // 5��
    g_TimeSyncCtrl.step_thresholds[1] = 2000000000;  // 2��
    g_TimeSyncCtrl.step_thresholds[2] = 1000000000;  // 1��
    
    // ƽ����������
    g_TimeSyncCtrl.smooth_factor = 200;          // ƽ������
    g_TimeSyncCtrl.max_freq_adjust = 0.00005;    // ���Ƶ�ʵ����ʣ�50ppm��
    
    // ʱ�ӿ��Ʋ��� - ����125MHzʱ��
    g_TimeSyncCtrl.base_frequency = CLOCK_FREQ_HZ;  // ����Ƶ�ʣ�125MHz��
    g_TimeSyncCtrl.current_frequency = g_TimeSyncCtrl.base_frequency;
    
    // ��ʼ��״̬
    g_TimeSyncCtrl.time_offset = 0;
    g_TimeSyncCtrl.last_sync_time = 0;
    g_TimeSyncCtrl.adjustment_count = 0;
    memset(g_TimeSyncCtrl.adjustment_history, 0, sizeof(g_TimeSyncCtrl.adjustment_history));
    
    // ��ʼ���ȶ���ָ��
    g_TimeSyncCtrl.stability_mean = 0;
    g_TimeSyncCtrl.stability_std_dev = 0;
    g_TimeSyncCtrl.stability_count = 0;
    g_TimeSyncCtrl.stability_warning = 0;
}

// ƽ��ʱ�ӵ�����ͨ��Ƶ��΢����
void ApplySmoothAdjustment(double offset) {
    double freq_adjust_ratio = offset / g_TimeSyncCtrl.smooth_factor;
    
    // ��������������
    if (freq_adjust_ratio > g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = g_TimeSyncCtrl.max_freq_adjust;
    } else if (freq_adjust_ratio < -g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = -g_TimeSyncCtrl.max_freq_adjust;
    }
    
    // �����µ�Ƶ��
    double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
    
    // Ӧ��Ƶ�ʵ���
    g_TimeSyncCtrl.current_frequency = new_frequency;
    
    // ת��Ϊ����������ֵ
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / new_frequency) / NS_PER_STEP);
    
    // д�벽���������Ĵ���
    WriteRegister(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);
    
    // ��¼������ʷ
    UpdateAdjustmentHistory(freq_adjust_ratio);
}

// �༶��������
void ApplyMultiStepAdjustment(double offset) {
    double abs_offset = fabs(offset);
    double step_ratio = 0;
    
    // ����ƫ������Сѡ����ʵĲ���
    if (abs_offset >= g_TimeSyncCtrl.step_thresholds[0]) {
        step_ratio = g_TimeSyncCtrl.step_levels[0];
    } else if (abs_offset >= g_TimeSyncCtrl.step_thresholds[1]) {
        step_ratio = g_TimeSyncCtrl.step_levels[1];
    } else {
        step_ratio = g_TimeSyncCtrl.step_levels[2];
    }
    
    // ������Ҫ������Ƶ��
    double freq_adjust_ratio = abs_offset * step_ratio;
    
    // ��������������
    if (freq_adjust_ratio > g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = g_TimeSyncCtrl.max_freq_adjust;
    }
    
    // �����µ�Ƶ�ʣ�����ʱ��׷������ʱ�䣩
    double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
    
    // Ӧ��Ƶ�ʵ���
    g_TimeSyncCtrl.current_frequency = new_frequency;
    
    // ת��Ϊ����������ֵ
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / new_frequency) / NS_PER_STEP);
    
    // д�벽���������Ĵ���
    WriteRegister(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);
    
    // ��¼������ʷ
    UpdateAdjustmentHistory(freq_adjust_ratio);
}

// ִ��ʱ��ͬ������
void PerformTimeSync(double measured_offset) {
    g_TimeSyncCtrl.time_offset = measured_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // ����ѡ��
    if (measured_offset < 0) {
        // ���1���ӻ�ʱ�����������offsetΪ����
        ApplySmoothAdjustment(measured_offset);
    } else if (measured_offset < g_TimeSyncCtrl.slow_threshold) {
        // ���2���ӻ�ʱ������������δ������ֵ
        ApplySmoothAdjustment(measured_offset);
    } else {
        // ���3���ӻ�ʱ�����������ҳ�����ֵ
        ApplyMultiStepAdjustment(measured_offset);
    }
    
    g_TimeSyncCtrl.last_sync_time = GetSystemTime();
}

// ��ʼ��SequenceID�����
void InitSeqIdChecker(void) {
    memset(&g_SeqIdChecker, 0, sizeof(SeqIdChecker));
}

// ���SequenceID��Ч��
BOOL CheckSequenceID(UINT8 msg_type, UINT16 seq_id) {
    if (msg_type >= 6) {
        g_SeqIdChecker.invalid_count[msg_type]++;
        return FALSE;
    }
    
    // ����Ƿ���Ԥ�ڵ�SequenceID
    if (seq_id == g_SeqIdChecker.expected_seq_id[msg_type]) {
        // ��Ч��˳����ȷ
        g_SeqIdChecker.last_valid_seq_id[msg_type] = seq_id;
        g_SeqIdChecker.expected_seq_id[msg_type]++;
        return TRUE;
    } else if (seq_id < g_SeqIdChecker.expected_seq_id[msg_type]) {
        // �ظ���Ϣ
        g_SeqIdChecker.duplicate_count[msg_type]++;
        return FALSE;
    } else {
        // ��ʧ��Ϣ
        g_SeqIdChecker.missed_count[msg_type] += (seq_id - g_SeqIdChecker.expected_seq_id[msg_type]);
        g_SeqIdChecker.last_valid_seq_id[msg_type] = seq_id;
        g_SeqIdChecker.expected_seq_id[msg_type] = seq_id + 1;
        return TRUE;
    }
}

// ��ʼ��TX Buffer״̬
void InitTXBufferStatus(void) {
    memset(&g_TxBufferStatus, 0, sizeof(TXBufferStatus));
}

// ���TX Buffer�Ƿ�æ
BOOL IsTXBufferBusy(UINT32 buffer_num) {
    if (buffer_num > 4) return TRUE; // ��Ч���������
    return g_TxBufferStatus.isBusy[buffer_num];
}

// ���TX BufferΪæ״̬
void MarkTXBufferBusy(UINT32 buffer_num, UINT32 msg_length) {
    if (buffer_num > 4) return; // ��Ч���������
    
    g_TxBufferStatus.isBusy[buffer_num] = TRUE;
    g_TxBufferStatus.msgLength[buffer_num] = msg_length;
    g_TxBufferStatus.lastTxTime[buffer_num] = GetSystemTime();
    g_TxBufferStatus.txCount[buffer_num]++;
}

// ���TX BufferΪ����״̬
void MarkTXBufferFree(UINT32 buffer_num) {
    if (buffer_num > 4) return; // ��Ч���������
    
    g_TxBufferStatus.isBusy[buffer_num] = FALSE;
}

// ��ʼ��ϵͳͳ��
void InitSystemStats(void) {
    memset(&g_Stats, 0, sizeof(SystemStats));
    g_Stats.last_stats_time = GetSystemTime();
}

// ����ϵͳͳ��
void UpdateSystemStats(UINT8 msg_type, UINT32 msg_length, BOOL is_tx) {
    if (is_tx) {
        g_Stats.total_tx_messages++;
        g_Stats.tx_bytes += msg_length;
    } else {
        g_Stats.total_rx_messages++;
        g_Stats.rx_bytes += msg_length;
    }
    
    // ͳ�Ƹ���PTP��Ϣ
    if (msg_type < 6) {
        g_Stats.ptp_message_types[msg_type]++;
    }
}

// ����ϵͳͳ��
void ResetSystemStats(void) {
    memset(&g_Stats, 0, sizeof(SystemStats));
    g_Stats.last_stats_time = GetSystemTime();
}

// ����ϵͳ����
void HandleSystemError(char *error_msg) {
    // �������־��¼�����������
}

// ���ʱ��ͬ���ȶ���
void MonitorTimeSyncStability(void) {
    // �������100�ε�����ƽ��ֵ�ͱ�׼��
    double sum = 0;
    double sum_sq = 0;
    int count = 0;
    
    for (int i = 0; i < 100; i++) {
        if (g_TimeSyncCtrl.adjustment_history[i] != 0) {
            sum += g_TimeSyncCtrl.adjustment_history[i];
            sum_sq += g_TimeSyncCtrl.adjustment_history[i] * g_TimeSyncCtrl.adjustment_history[i];
            count++;
        }
    }
    
    if (count > 0) {
        double mean = sum / count;
        double variance = (sum_sq / count) - (mean * mean);
        double std_dev = sqrt(variance);
        
        // ����ͳ�ƽ�����ṹ��
        g_TimeSyncCtrl.stability_mean = mean;
        g_TimeSyncCtrl.stability_std_dev = std_dev;
        g_TimeSyncCtrl.stability_count = count;
        
        // �ɸ����ȶ���ָ�����ͬ������
        if (std_dev > 0.00001) {
            g_TimeSyncCtrl.stability_warning = 1;
        } else {
            g_TimeSyncCtrl.stability_warning = 0;
        }
    }
}

// ���µ�����ʷ
void UpdateAdjustmentHistory(double adjustment) {
    int index = g_TimeSyncCtrl.adjustment_count % 100;
    g_TimeSyncCtrl.adjustment_history[index] = adjustment;
}
