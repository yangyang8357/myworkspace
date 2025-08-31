#include "xil_types.h"
#include "xil_io.h"
#include "xparameters.h"
#include <math.h>
#include <string.h>

// ����ʱ����س���
#define CLOCK_FREQ_HZ       125000000  // ʱ�ӻ���Ƶ�� (125MHz)
#define NS_PER_SECOND       1000000000 // ÿ��������
#define STEP_COUNTER_BITS   32         // ����������λ��
#define STEP_COUNTER_MAX    0xFFFFFFFF // �������������ֵ
#define NS_PER_STEP         8          // ÿ�����������������
#define SYNC_INTERVAL       1000000000 // ͬ����Ϣ���ͼ�� (1��)
#define STABILITY_CHECK_INTERVAL 100   // �ȶ��Լ���� (100�ε���)

// �������˲�����
#define KF_Q                0.001      // ��������Э����
#define KF_R                0.1        // ��������Э����
#define KF_INIT_P           1.0        // ��ʼ�������Э����
#define KF_INIT_X           0.0        // ��ʼ״̬����

// ����ʱ�����ڶ�Ӧ��������
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// ��Ϣ����ö��
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// ͬ��״̬ö��
typedef enum {
    INITIALIZING,      // ��ʼ��״̬
    SYNCHRONIZING,    // ͬ����
    SYNCHRONIZED,     // ��ͬ��
    LOSING_SYNC,      // ͬ����ʧ
    RECOVERING,       // �ָ�ͬ��
    HOLDING,          // ����״̬������ʱ���źţ�
    FAULTY           // ����״̬
} SyncState;

// ͬ��״̬��ϸ��Ϣ�ṹ��
typedef struct {
    SyncState state;           // ��ǰͬ��״̬
    UINT32 state_duration;     // ��ǰ״̬����ʱ�䣨���룩
    UINT32 state_transitions;  // ״̬ת������
    UINT32 time_since_sync;    // ���ϴγɹ�ͬ��������ʱ�䣨���룩
    UINT32 sync_failure_count; // ͬ��ʧ�ܼ���
    double sync_accuracy;      // ͬ�����ȹ��ƣ����룩
    double offset_trend;       // ƫ�����ƣ�����/�룩
    BOOL is_locked;            // �Ƿ���������ʱ��
    BOOL is_synchronized;      // �Ƿ�ͬ��
    BOOL is_stable;            // ͬ���Ƿ��ȶ�
    char state_text[32];       // ״̬�ı�����
} SyncStatus;

// �������˲����ṹ��
typedef struct {
    double x;        // ״̬���ƣ�ʱ��ƫ�ƣ�
    double P;        // �������Э����
    double K;        // ����������
    double Q;        // ��������Э����
    double R;        // ��������Э����
    UINT64 last_time; // �ϴθ���ʱ��
} KalmanFilter;

// ����ʱ��ͬ���������ṹ��
typedef struct {
    double time_offset;          // ʱ��ƫ�������ӻ�-���������룩
    UINT64 last_sync_time;       // �ϴ�ͬ��ʱ��
    UINT64 last_announce_time;   // �ϴ��յ�Announce��Ϣʱ��
    double slow_threshold;       // �ӻ���������ʱ����ֵ�����룩
    double fast_threshold;       // �ӻ���������ʱ����ֵ�����룩
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
    
    // ͬ��״̬
    SyncState state;             // ��ǰͬ��״̬
    UINT32 state_start_time;     // ��ǰ״̬��ʼʱ��
    UINT32 state_duration;       // ��ǰ״̬����ʱ��
    UINT32 state_transitions;    // ״̬ת������
    SyncStatus status;           // ͬ��״̬��ϸ��Ϣ
    
    // �������˲���
    KalmanFilter kf;             // ʱ��ƫ�ƿ������˲���
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

// TxBuffer�ṹ�嶨�壨PTP��Ϣ��������
typedef struct
{
  union
  {
    struct
    {
      UINT8     frmLen[2];
      UINT8     cmd[6];
      UINT8     dstAddr[6]; //start of frmLen
      UINT8     srcAddr[6];
      UINT8     ethType[2];
      UINT8     messageType; // start of messageLength
      UINT8     versionPTP;
      UINT8     messageLength[2];
      UINT8     domainNumber;
      UINT8     minorSdoId;
      UINT8     flags[2];
      UINT8     correctionField[8];
      UINT8     messageTypeSpecific[4];
      UINT8     clockId[8];
      UINT8     portNum[2];
      UINT8     sequenceID[2];
      UINT8     controlFiled;
      UINT8     logMessageInterval;
      UINT8     originTimestamp[10];
      union
      {
        UINT8     reqPortId[10];
        UINT8     TLV[32];
        struct {
        	UINT8 currentUtcOffset[2];
        	UINT8 rsv;
        	UINT8 grandmasterPriority1;
        	UINT8 grandmasterClockclass;  		// grandmasterClockQuality[0];
        	UINT8 grandmasterClockAccuracy;		// grandmasterClockQuality[1];
        	UINT8 grandmasterClockVariance[2];	// grandmasterClockQuality[2]-[3];
        	UINT8 grandmasterPriority2;
        	UINT8 grandmasterIdentity[8];
        	UINT8 stepsRemoved[2];
        	UINT8 timeSource;
        	struct{
        		UINT8 tlvType[2];
        		UINT8 lengthFiled[2];
        		UINT8 pathSequence[][8];
        	};
        };
      };
    };
    UINT32 buf[64]; //ptp buffer 256�ֽ�
  };
} TxBuffer;

// ȫ�ֱ�������
UINT32 TSN_BASEADDR;             // TSN����������ַ
TimeSyncController g_TimeSyncCtrl;
SeqIdChecker g_SeqIdChecker;
TXBufferStatus g_TxBufferStatus;
SystemStats g_Stats;

// ����PTP��Ϣ������
TxBuffer SyncBuf;
TxBuffer FollowUpBuf;
TxBuffer AnnounceBuf;
TxBuffer PdelayReqBuf;
TxBuffer PdelayRespBuf;
TxBuffer RxBuffer;  // ���ջ�����

// I/O��������
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
}

// �ӼĴ�����ȡ���ݵ�TxBuffer��32λ��ȡ��
void ReadRegistersToBuffer(UINT32 base_addr, TxBuffer *buffer, UINT32 size) {
    UINT32 words = (size + 3) / 4;  // ������Ҫ��ȡ��32λ������
    
    for (UINT32 i = 0; i < words; i++) {
        buffer->buf[i] = Read32(base_addr + i * 4);
    }
}

// ��ȡϵͳʱ�䣨���룩
UINT64 GetSystemTime() {
    UINT32 high = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_MSB_FIELD_OFFSET);
    UINT32 low = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_LSB_FIELD_OFFSET);
    UINT32 step_counter = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_NANOSECONDS_FIELD_OFFSET);
    
    // ������������ֵת��Ϊʵ������ֵ
    UINT64 nanoseconds = (UINT64)step_counter * NS_PER_STEP;
    
    // ���������벿��
    UINT64 seconds = ((UINT64)high << 32) | low;
    UINT64 total_nanoseconds = (seconds * NS_PER_SECOND) + nanoseconds;
    
    return total_nanoseconds;
}

// д��Ĵ���
void WriteRegister(UINT32 addr, UINT32 data) {
    Write32(addr, data);
}

// ��ʼ���������˲���
void InitKalmanFilter(KalmanFilter *kf) {
    kf->x = KF_INIT_X;
    kf->P = KF_INIT_P;
    kf->Q = KF_Q;
    kf->R = KF_R;
    kf->last_time = GetSystemTime();
}

// ���¿������˲���
double UpdateKalmanFilter(KalmanFilter *kf, double measurement) {
    UINT64 current_time = GetSystemTime();
    double dt = (double)(current_time - kf->last_time) / NS_PER_SECOND;
    kf->last_time = current_time;
    
    // Ԥ�ⲽ��
    // ״̬Ԥ�⣨�������Ա仯��
    // ʱ��ƫ��Ԥ�⣨�ޱ仯ģ�ͣ�
    // ���Э����Ԥ��
    kf->P = kf->P + kf->Q;
    
    // ���²���
    // ���㿨��������
    kf->K = kf->P / (kf->P + kf->R);
    
    // ����״̬����
    kf->x = kf->x + kf->K * (measurement - kf->x);
    
    // �������Э����
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->x;
}

// ��ʼ��ϵͳӲ��
void InitSystemHardware() {
    // ����ϵͳʱ��
    // ��ʼ���жϿ�����
    // ��ʼ������ӿ�
    // ����Ӳ����ʼ��
}

// ��ʼ��ʱ��ͬ��������
void InitTimeSyncController() {
    g_TimeSyncCtrl.slow_threshold = 1000000000;  // 1�루���룩
    g_TimeSyncCtrl.fast_threshold = -1000000000; // -1�루���룩
    
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
    g_TimeSyncCtrl.last_announce_time = 0;
    g_TimeSyncCtrl.adjustment_count = 0;
    memset(g_TimeSyncCtrl.adjustment_history, 0, sizeof(g_TimeSyncCtrl.adjustment_history));
    
    // ��ʼ���ȶ���ָ��
    g_TimeSyncCtrl.stability_mean = 0;
    g_TimeSyncCtrl.stability_std_dev = 0;
    g_TimeSyncCtrl.stability_count = 0;
    g_TimeSyncCtrl.stability_warning = 0;
    
    // ��ʼ��ͬ��״̬
    g_TimeSyncCtrl.state = INITIALIZING;
    g_TimeSyncCtrl.state_start_time = GetSystemTime();
    g_TimeSyncCtrl.state_duration = 0;
    g_TimeSyncCtrl.state_transitions = 0;
    
    // ��ʼ��ͬ��״̬��ϸ��Ϣ
    UpdateSyncStatus();
    
    // ��ʼ���������˲���
    InitKalmanFilter(&g_TimeSyncCtrl.kf);
}

// ����ͬ��״̬��ϸ��Ϣ
void UpdateSyncStatus() {
    SyncStatus *status = &g_TimeSyncCtrl.status;
    
    // ���»���״̬��Ϣ
    status->state = g_TimeSyncCtrl.state;
    status->state_duration = g_TimeSyncCtrl.state_duration;
    status->state_transitions = g_TimeSyncCtrl.state_transitions;
    status->time_since_sync = (UINT32)((GetSystemTime() - g_TimeSyncCtrl.last_sync_time) / 1000000);
    status->sync_failure_count = g_TimeSyncCtrl.sync_failure_count;
    status->sync_accuracy = g_TimeSyncCtrl.sync_accuracy;
    
    // ����ƫ������
    if (g_TimeSyncCtrl.adjustment_count >= 10) {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < 10; i++) {
            if (g_TimeSyncCtrl.adjustment_history[i] != 0) {
                sum += g_TimeSyncCtrl.adjustment_history[i];
                count++;
            }
        }
        if (count > 0) {
            status->offset_trend = sum / count * 1000000000; // ת��Ϊ����/��
        } else {
            status->offset_trend = 0;
        }
    } else {
        status->offset_trend = 0;
    }
    
    // ����״̬��־
    status->is_locked = (g_TimeSyncCtrl.state == SYNCHRONIZED);
    status->is_synchronized = (g_TimeSyncCtrl.state == SYNCHRONIZED || 
                              g_TimeSyncCtrl.state == HOLDING);
    status->is_stable = (g_TimeSyncCtrl.stability_std_dev < 0.00001);
    
    // ����״̬�ı�����
    switch (g_TimeSyncCtrl.state) {
        case INITIALIZING:
            strcpy(status->state_text, "��ʼ��");
            break;
        case SYNCHRONIZING:
            strcpy(status->state_text, "ͬ����");
            break;
        case SYNCHRONIZED:
            strcpy(status->state_text, "��ͬ��");
            break;
        case LOSING_SYNC:
            strcpy(status->state_text, "ͬ����ʧ");
            break;
        case RECOVERING:
            strcpy(status->state_text, "�ָ�ͬ��");
            break;
        case HOLDING:
            strcpy(status->state_text, "����״̬");
            break;
        case FAULTY:
            strcpy(status->state_text, "����״̬");
            break;
        default:
            strcpy(status->state_text, "δ֪״̬");
            break;
    }
}

// �л�ͬ��״̬
void ChangeSyncState(SyncState new_state) {
    if (g_TimeSyncCtrl.state != new_state) {
        UINT64 current_time = GetSystemTime();
        
        // ���µ�ǰ״̬����ʱ��
        g_TimeSyncCtrl.state_duration = (UINT32)((current_time - g_TimeSyncCtrl.state_start_time) / 1000000);
        
        // ��¼״̬ת��
        g_TimeSyncCtrl.state = new_state;
        g_TimeSyncCtrl.state_start_time = current_time;
        g_TimeSyncCtrl.state_transitions++;
        
        // ����ͬ��״̬��ϸ��Ϣ
        UpdateSyncStatus();
        
        // ״̬ת����־��¼
        xil_printf("Sync state changed to: %s\r\n", g_TimeSyncCtrl.status.state_text);
    }
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
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_IVCR_OFFSET, step_counter_value);
    
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
    
    // ����ƫ�Ʒ������ʱ��
    if (offset > 0) {
        // �ӻ���������������ʱ��
        double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
        g_TimeSyncCtrl.current_frequency = new_frequency;
    } else {
        // �ӻ���������������ʱ��
        double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 - freq_adjust_ratio);
        g_TimeSyncCtrl.current_frequency = new_frequency;
    }
    
    // ת��Ϊ����������ֵ
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / g_TimeSyncCtrl.current_frequency) / NS_PER_STEP);
    
    // д�벽���������Ĵ���
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_IVCR_OFFSET, step_counter_value);
    
    // ��¼������ʷ
    UpdateAdjustmentHistory(freq_adjust_ratio * (offset > 0 ? 1 : -1));
}

// ʱ�����������ֱ���޸�ϵͳʱ�䣩
void ApplyTimeJump(double offset) {
    UINT64 current_time = GetSystemTime();
    UINT64 new_time = current_time - (UINT64)offset;  // ��ȥƫ������ʹ�ӻ�ʱ��׷������
    
    // �����µ��벿�ֺ����벿��
    UINT64 seconds = new_time / NS_PER_SECOND;
    UINT64 nanoseconds = new_time % NS_PER_SECOND;
    
    // ���㲽��������ֵ
    UINT32 step_counter_value = (UINT32)(nanoseconds / NS_PER_STEP);
    
    // д���µ�ʱ��ֵ
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_MSB_FIELD_OFFSET, (UINT32)(seconds >> 32));
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_LSB_FIELD_OFFSET, (UINT32)seconds);
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_NANOSECONDS_FIELD_OFFSET, step_counter_value);
    
    // ��¼������ʷ
    UpdateAdjustmentHistory(offset / 1000000000);  // ת��Ϊ��Ϊ��λ
    
    // ״̬ת��������Ǵ�ͬ����ʧ״̬�������䣬ת�����ָ�ͬ��״̬
    if (g_TimeSyncCtrl.state == LOSING_SYNC || g_TimeSyncCtrl.state == RECOVERING) {
        ChangeSyncState(RECOVERING);
    }
}

// ִ��ʱ��ͬ������
void PerformTimeSync(double measured_offset) {
    // ʹ�ÿ������˲����������ֵ
    double filtered_offset = UpdateKalmanFilter(&g_TimeSyncCtrl.kf, measured_offset);
    
    g_TimeSyncCtrl.time_offset = filtered_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // ����ͬ�����ȹ���
    g_TimeSyncCtrl.sync_accuracy = fabs(filtered_offset);
    
    // ����ͬ��״̬
    UpdateSyncStatus();
    
    // ״̬ת���߼�
    switch (g_TimeSyncCtrl.state) {
        case INITIALIZING:
            // �ӳ�ʼ��״̬��ʼͬ��
            ChangeSyncState(SYNCHRONIZING);
            break;
            
        case SYNCHRONIZING:
            // ����Ƿ��Ѵﵽͬ������
            if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold / 2) {
                ChangeSyncState(SYNCHRONIZED);
            }
            break;
            
        case SYNCHRONIZED:
            // ����Ƿ�ʧȥͬ��
            if (fabs(filtered_offset) > g_TimeSyncCtrl.slow_threshold * 2) {
                ChangeSyncState(LOSING_SYNC);
            } else if (g_TimeSyncCtrl.last_announce_time > 0 && 
                      (GetSystemTime() - g_TimeSyncCtrl.last_announce_time) > 3000000000) {
                // ��ʱ��δ�յ�Announce��Ϣ
                ChangeSyncState(HOLDING);
            }
            break;
            
        case LOSING_SYNC:
            // ���Իָ�ͬ��
            if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold) {
                ChangeSyncState(SYNCHRONIZED);
            } else {
                g_TimeSyncCtrl.sync_failure_count++;
                if (g_TimeSyncCtrl.sync_failure_count > 5) {
                    ChangeSyncState(RECOVERING);
                }
            }
            break;
            
        case RECOVERING:
            // ����Ƿ��ѻָ�ͬ��
            if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold) {
                ChangeSyncState(SYNCHRONIZED);
                g_TimeSyncCtrl.sync_failure_count = 0;
            }
            break;
            
        case HOLDING:
            // �ڱ���״̬�£��ȴ��µ�ͬ����Ϣ
            if (g_TimeSyncCtrl.last_announce_time > 0 && 
               (GetSystemTime() - g_TimeSyncCtrl.last_announce_time) < 1000000000) {
                ChangeSyncState(SYNCHRONIZING);
            }
            break;
            
        case FAULTY:
            // ����״̬����Ҫ�˹���Ԥ���Զ��ָ�����
            // ��������Զ��ָ��߼�
            if (g_TimeSyncCtrl.adjustment_count % 100 == 0) {
                ChangeSyncState(RECOVERING);
            }
            break;
    }
    
    // ʱ��ͬ������ѡ��
    if (filtered_offset > g_TimeSyncCtrl.slow_threshold) {
        // �ӻ���������������ֵ
        if (filtered_offset > 1000000000) {  // ����1�룬ִ��ʱ������
            ApplyTimeJump(filtered_offset);
        } else {  // С��1�룬ʹ�ö༶��������Ƶ��
            ApplyMultiStepAdjustment(filtered_offset);
        }
    } else if (filtered_offset < g_TimeSyncCtrl.fast_threshold) {
        // �ӻ���������������ֵ��ʹ�ö༶��������Ƶ��
        ApplyMultiStepAdjustment(filtered_offset);
    } else {
        // Сƫ�ƣ�ʹ��ƽ������
        ApplySmoothAdjustment(filtered_offset);
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
        g_SeqIdChecker.last_checked_time[msg_type] = GetSystemTime();
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
        g_SeqIdChecker.last_checked_time[msg_type] = GetSystemTime();
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

// ��ȡ���õ�TX Buffer
UINT32 GetAvailableTXBuffer(void) {
    for (UINT32 i = 0; i < 5; i++) {
        if (!IsTXBufferBusy(i)) {
            return i;
        }
    }
    return 0xFFFFFFFF; // û�п��û�����
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
    xil_printf("System Error: %s\r\n", error_msg);
    
    // ����������ش����л�������״̬
    ChangeSyncState(FAULTY);
}

// ���ʱ��ͬ���ȶ���
void MonitorTimeSyncStability(void) {
    // ÿSTABILITY_CHECK_INTERVAL�ε�������һ���ȶ��Լ��
    if (g_TimeSyncCtrl.adjustment_count % STABILITY_CHECK_INTERVAL != 0) {
        return;
    }
    
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
        
        // ����ͬ��״̬
        if (std_dev > 0.00001) {
            g_TimeSyncCtrl.stability_warning = 1;
            if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
                ChangeSyncState(LOSING_SYNC);
            }
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

// ����PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
BOOL ParsePTPMessage(TxBuffer *rx_buf, UINT8 *msg_type, UINT16 *seq_id, UINT64 *timestamp) {
    // ��ȡ��Ϣ����
    *msg_type = rx_buf->messageType;
    
    // ��ȡSequenceID
    *seq_id = (rx_buf->sequenceID[0] << 8) | rx_buf->sequenceID[1];
    
    // ��ȡʱ�����originTimestamp�ֶΣ�
    *timestamp = 0;
    for (int i = 0; i < 8; i++) {  // ȡǰ8�ֽ���Ϊ64λʱ���
        *timestamp |= (UINT64)rx_buf->originTimestamp[i] << (56 - i * 8);
    }
    
    return TRUE;
}

// ����PTP��Ϣ��ʹ��32λ�Ĵ�����ȡ��TxBuffer��
BOOL ReceivePTPMessage(TxBuffer *rx_buf) {
    // ������״̬�Ĵ������ж��Ƿ�������Ϣ
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_RX_STATUS_OFFSET);
    
    if ((status & 0x01) == 0) {  // ���û������Ϣ
        return FALSE;
    }
    
    // �ӽ��ռĴ�����ȡ���ݵ���������ʹ��32λ��ȡ��
    ReadRegistersToBuffer(TSN_BASEADDR + XPAR_TSN_0_RX_DATA_OFFSET, rx_buf, 256);
    
    // �������״̬��־
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RX_STATUS_OFFSET, status & ~0x01);
    
    return TRUE;
}

// ������յ���PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
void ProcessPTPMessage(TxBuffer *rx_buf) {
    UINT8 msg_type;
    UINT16 seq_id;
    UINT64 timestamp;
    
    // ����PTP��Ϣ
    if (!ParsePTPMessage(rx_buf, &msg_type, &seq_id, &timestamp)) {
        g_Stats.rx_error_count++;
        return;
    }
    
    // ���SequenceID��Ч��
    if (!CheckSequenceID(msg_type, seq_id)) {
        // ������ЧSequenceID
        return;
    }
    
    // ������Ϣ���ʹ���
    switch (msg_type) {
        case SYNC:
            // ����Sync��Ϣ
            ProcessSyncMessage(timestamp);
            break;
            
        case FOLLOWUP:
            // ����FollowUp��Ϣ
            ProcessFollowUpMessage(timestamp);
            break;
            
        case ANNOUNCE:
            // ����Announce��Ϣ
            ProcessAnnounceMessage(rx_buf);
            g_TimeSyncCtrl.last_announce_time = GetSystemTime();
            break;
            
        case PDELAY_REQ:
            // ����PdelayReq��Ϣ
            ProcessPdelayReqMessage(rx_buf);
            break;
            
        case PDELAY_RESP:
            // ����PdelayResp��Ϣ
            ProcessPdelayRespMessage(rx_buf);
            break;
            
        case PDELAY_FOLLOWUP:
            // ����PdelayFollowUp��Ϣ
            ProcessPdelayFollowUpMessage(rx_buf);
            break;
            
        default:
            break;
    }
    
    // ������Ϣ����
    UINT32 msg_length = (rx_buf->messageLength[0] << 8) | rx_buf->messageLength[1];
    
    // ����ϵͳͳ��
    UpdateSystemStats(msg_type, msg_length, FALSE);
}

// ����Sync��Ϣ
void ProcessSyncMessage(UINT64 sync_timestamp) {
    // ��¼Sync��Ϣ����ʱ��
    UINT64 local_receive_time = GetSystemTime();
    
    // ����Syncʱ���������FollowUp��Ϣ��ʹ��
}

// ����FollowUp��Ϣ
void ProcessFollowUpMessage(UINT64 precise_origin_timestamp) {
    // ����ʱ��ƫ��
    double offset = CalculateTimeOffset(precise_origin_timestamp);
    
    // ִ��ʱ��ͬ������
    PerformTimeSync(offset);
}

// ����Announce��Ϣ��ʹ��TxBuffer�ṹ�壩
void ProcessAnnounceMessage(TxBuffer *announce_buf) {
    // ����Announce��Ϣ�е�ʱ������ȼ���Ϣ
    // ���±���ʱ��ͬ��״̬
}

// ����PdelayReq��Ϣ��ʹ��TxBuffer�ṹ�壩
void ProcessPdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // ��¼PdelayReq��Ϣ����ʱ��
    // ׼��������PdelayResp��Ϣ
}

// ����PdelayResp��Ϣ��ʹ��TxBuffer�ṹ�壩
void ProcessPdelayRespMessage(TxBuffer *pdelay_resp_buf) {
    // ��¼PdelayResp��Ϣ����ʱ��
    // ����PdelayResp��Ϣ�е���Ϣ���ȴ�PdelayFollowUp��Ϣ
}

// ����PdelayFollowUp��Ϣ��ʹ��TxBuffer�ṹ�壩
void ProcessPdelayFollowUpMessage(TxBuffer *pdelay_followup_buf) {
    // ��ȡPdelayReq����ʱ���
    // ���������ӳٺ�ʱ��ƫ��
    // ִ��ʱ��ͬ������
}

// ����ʱ��ƫ��
double CalculateTimeOffset(UINT64 master_time) {
    // ��ȡ����ʱ��
    UINT64 local_time = GetSystemTime();
    
    // ����ʱ��ƫ�ƣ��ӻ�ʱ�� - ����ʱ�䣩
    double offset = (double)(local_time - master_time);
    
    return offset;
}

// ׼��Sync��Ϣ��ʹ��TxBuffer�ṹ�壩
void PrepareSyncMessage(TxBuffer *sync_buf) {
    // ��ջ�����
    memset(sync_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    sync_buf->frmLen[0] = 0x2C;  // ���ֽ�
    sync_buf->frmLen[1] = 0x00;  // ���ֽ�
    sync_buf->messageLength[0] = 0x2C;  // ���ֽ�
    sync_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    sync_buf->messageType = SYNC;
    sync_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    sync_buf->ethType[0] = 0x88;
    sync_buf->ethType[1] = 0xF7;
    
    // ���ù㲥��ַ
    memset(sync_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[SYNC];
    sync_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    sync_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[SYNC]++;
}

// ׼��FollowUp��Ϣ��ʹ��TxBuffer�ṹ�壩
void PrepareFollowUpMessage(TxBuffer *followup_buf, UINT64 sync_origin_timestamp) {
    // ��ջ�����
    memset(followup_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    followup_buf->frmLen[0] = 0x34;  // ���ֽ�
    followup_buf->frmLen[1] = 0x00;  // ���ֽ�
    followup_buf->messageLength[0] = 0x34;  // ���ֽ�
    followup_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    followup_buf->messageType = FOLLOWUP;
    followup_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    followup_buf->ethType[0] = 0x88;
    followup_buf->ethType[1] = 0xF7;
    
    // ���ù㲥��ַ
    memse    // ���ù㲥��ַ
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
    
    // ����preciseOriginTimestamp�ֶΣ���Sync��Ϣ�л�ȡ��
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// ׼��Announce��Ϣ��ʹ��TxBuffer�ṹ�壩
void PrepareAnnounceMessage(TxBuffer *announce_buf) {
    // ��ջ�����
    memset(announce_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    announce_buf->frmLen[0] = 0x48;  // ���ֽ�
    announce_buf->frmLen[1] = 0x00;  // ���ֽ�
    announce_buf->messageLength[0] = 0x48;  // ���ֽ�
    announce_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    announce_buf->messageType = ANNOUNCE;
    announce_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    announce_buf->ethType[0] = 0x88;
    announce_buf->ethType[1] = 0xF7;
    
    // ���ù㲥��ַ
    memset(announce_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[ANNOUNCE];
    announce_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    announce_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[ANNOUNCE]++;
    
    // ����Announce��Ϣ�ض��ֶ�
    announce_buf->controlFiled = 5;  // Announce��Ϣ��controlFieldֵΪ5
    
    // ����ʱ��Դ��Ϣ
    announce_buf->timeSource = 0x20;  // GPSʱ��Դ
}

// ׼��PdelayReq��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // ��ջ�����
    memset(pdelay_req_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_req_buf->frmLen[0] = 0x34;  // ���ֽ�
    pdelay_req_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_req_buf->messageLength[0] = 0x34;  // ���ֽ�
    pdelay_req_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_req_buf->messageType = PDELAY_REQ;
    pdelay_req_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_req_buf->ethType[0] = 0x88;
    pdelay_req_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ��ʱ��MAC
    // �˴�Ӧ����Ϊ��ʱ�ӵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_req_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_REQ];
    pdelay_req_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_req_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_REQ]++;
}

// ׼��PdelayResp��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayRespMessage(TxBuffer *pdelay_resp_buf, UINT8 *requesting_port_id) {
    // ��ջ�����
    memset(pdelay_resp_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_resp_buf->frmLen[0] = 0x3C;  // ���ֽ�
    pdelay_resp_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_resp_buf->messageLength[0] = 0x3C;  // ���ֽ�
    pdelay_resp_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_resp_buf->messageType = PDELAY_RESP;
    pdelay_resp_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_resp_buf->ethType[0] = 0x88;
    pdelay_resp_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ����˿ڵ�MAC��ַ
    // �˴�Ӧ����Ϊ����˿ڵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_resp_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_RESP];
    pdelay_resp_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_resp_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_RESP]++;
    
    // ��������˿�ID
    memcpy(pdelay_resp_buf->reqPortId, requesting_port_id, 10);
}

// ׼��PdelayFollowUp��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayFollowUpMessage(TxBuffer *pdelay_followup_buf, UINT8 *requesting_port_id, UINT64 request_receive_timestamp) {
    // ��ջ�����
    memset(pdelay_followup_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_followup_buf->frmLen[0] = 0x44;  // ���ֽ�
    pdelay_followup_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_followup_buf->messageLength[0] = 0x44;  // ���ֽ�
    pdelay_followup_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_followup_buf->messageType = PDELAY_FOLLOWUP;
    pdelay_followup_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_followup_buf->ethType[0] = 0x88;
    pdelay_followup_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ����˿ڵ�MAC��ַ
    // �˴�Ӧ����Ϊ����˿ڵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_followup_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP];
    pdelay_followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP]++;
    
    // ��������˿�ID
    memcpy(pdelay_followup_buf->reqPortId, requesting_port_id, 10);
    
    // ����requestReceiptTimestamp�ֶ�
    for (int i = 0; i < 8; i++) {
        pdelay_followup_buf->originTimestamp[i] = (request_receive_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// ����PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
BOOL SendPTPMessage(TxBuffer *tx_buf, UINT32 buffer_num) {
    // ���TX Buffer�Ƿ����
    if (IsTXBufferBusy(buffer_num)) {
        return FALSE;
    }
    
    // ���TX BufferΪæ״̬
    UINT32 msg_length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    MarkTXBufferBusy(buffer_num, msg_length);
    
    // ������д�뷢�ͻ�������ʹ��32λд�룩
    for (UINT32 i = 0; i < 64; i++) {
        Write32(TSN_BASEADDR + XPAR_TSN_0_TX_DATA_OFFSET + i * 4, tx_buf->buf[i]);
    }
    
    // ��������
    UINT32 control = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET);
    control |= (1 << buffer_num);  // ������Ӧ�������ķ���
    Write32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET, control);
    
    // ����ϵͳͳ��
    UpdateSystemStats(tx_buf->messageType, msg_length, TRUE);
    
    return TRUE;
}

// ��鷢��״̬
BOOL CheckTxStatus(UINT32 buffer_num) {
    if (buffer_num > 4) return FALSE; // ��Ч���������
    
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    return (status & (1 << buffer_num)) != 0;
}

// ��鲢���������
void ProcessTxComplete(void) {
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    
    for (UINT32 i = 0; i < 5; i++) {
        if ((status & (1 << i)) != 0) {
            // ������ɣ����״̬λ
            Write32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET, 1 << i);
            
            // ���TX BufferΪ����
            MarkTXBufferFree(i);
        }
    }
}

// ������
int main() {
    // ��ʼ��Ӳ��
    InitSystemHardware();
    
    // ��ʼ��TSN����������ַ
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;
    
    // ��ʼ��ʱ��ͬ��������
    InitTimeSyncController();
    
    // ��ʼ��SequenceID�����
    InitSeqIdChecker();
    
    // ��ʼ��TX Buffer״̬
    InitTXBufferStatus();
    
    // ��ʼ��ϵͳͳ��
    InitSystemStats();
    
    // ��ӡ��ʼ����Ϣ
    xil_printf("TSN Time Synchronization System initialized\r\n");
    
    // ��ѭ��
    while (1) {
        // ���������Ϣ
        if (ReceivePTPMessage(&RxBuffer)) {
            ProcessPTPMessage(&RxBuffer);
        }
        
        // ���������״̬
        ProcessTxComplete();
        
        // ���ʱ��ͬ���ȶ���
        MonitorTimeSyncStability();
        
        // ��鲢ִ�ж�ʱ����
        HandlePeriodicTasks();
        
        // ���ϵͳ״̬
        CheckSystemStatus();
        
        // ��ӡͬ��״̬��Ϣ��ÿ10��һ�Σ�
        static UINT32 last_print_time = 0;
        UINT32 current_time = GetSystemTime() / 1000000000; // ת��Ϊ��
        
        if (current_time - last_print_time >= 10) {
            PrintSyncStatus();
            last_print_time = current_time;
        }
    }
    
    return 0;
}

// ����ʱ����
void HandlePeriodicTasks() {
    static UINT64 last_sync_time = 0;
    static UINT64 last_announce_time = 0;
    static UINT64 last_pdelay_time = 0;
    
    UINT64 current_time = GetSystemTime();
    
    // ÿSYNC_INTERVAL����һ��Sync��Ϣ����Ϊ��ʱ��ʱ��
    if (current_time - last_sync_time >= SYNC_INTERVAL) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // ׼��Sync��Ϣ
            PrepareSyncMessage(&SyncBuf);
            
            // ��ȡ��ǰʱ����ΪSync��Ϣ��ʱ���
            UINT64 sync_timestamp = GetSystemTime();
            
            // ����Sync��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&SyncBuf, tx_buffer);
                
                // ��¼����ʱ�䣬���ں���FollowUp��Ϣ
                last_sync_time = current_time;
                
                // ׼��������FollowUp��Ϣ
                PrepareFollowUpMessage(&FollowUpBuf, sync_timestamp);
                tx_buffer = GetAvailableTXBuffer();
                if (tx_buffer != 0xFFFFFFFF) {
                    SendPTPMessage(&FollowUpBuf, tx_buffer);
                }
            }
        }
    }
    
    // ÿ5�뷢��һ��Announce��Ϣ����Ϊ��ʱ��ʱ��
    if (current_time - last_announce_time >= 5000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // ׼��Announce��Ϣ
            PrepareAnnounceMessage(&AnnounceBuf);
            
            // ����Announce��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&AnnounceBuf, tx_buffer);
                last_announce_time = current_time;
            }
        }
    }
    
    // ÿ10�뷢��һ��PdelayReq��Ϣ����Ϊ�߽�ʱ�ӻ��ʱ��ʱ��
    if (current_time - last_pdelay_time >= 10000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED || g_TimeSyncCtrl.state == SYNCHRONIZING) {
            // ׼��PdelayReq��Ϣ
            PreparePdelayReqMessage(&PdelayReqBuf);
            
            // ����PdelayReq��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&PdelayReqBuf, tx_buffer);
                last_pdelay_time = current_time;
            }
        }
    }
}

// ���ϵͳ״̬
void CheckSystemStatus() {
    // ���Ӳ��״̬
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET);
    
    // ���ʱ��״̬
    if ((status & 0x02) == 0) {  // ���ʱ�Ӷ�ʧ����
        HandleSystemError("Clock lock lost");
    }
    
    // �������ӿ�״̬
    if ((status & 0x04) == 0) {  // ���������·�Ͽ�
        HandleSystemError("Network link down");
    }
    
    // ���PTP����״̬
    if ((status & 0x08) != 0) {  // ���PTP�������
        HandleSystemError("PTP engine error");
        // ��������־
        Write32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET, 0x08);
    }
}

// ��ȡͬ��״̬��Ϣ
SyncStatus GetSyncStatus() {
    UpdateSyncStatus();
    return g_TimeSyncCtrl.status;
}

// ��ӡͬ��״̬��Ϣ
void PrintSyncStatus() {
    SyncStatus status = GetSyncStatus();
    
    xil_printf("=== ͬ��״̬��Ϣ ===\r\n");
    xil_printf("״̬: %s\r\n", status.state_text);
    xil_printf("״̬����ʱ��: %u ms\r\n", status.state_duration);
    xil_printf("״̬ת������: %u\r\n", status.state_transitions);
    xil_printf("���ϴ�ͬ��ʱ��: %u ms\r\n", status.time_since_sync);
    xil_printf("ͬ��ʧ�ܼ���: %u\r\n", status.sync_failure_count);
    xil_printf("ͬ������: %.2f ns\r\n", status.sync_accuracy);
    xil_printf("ƫ������: %.2f ns/s\r\n", status.offset_trend);
    xil_printf("����״̬: %s\r\n", status.is_locked ? "��" : "��");
    xil_printf("ͬ��״̬: %s\r\n", status.is_synchronized ? "��" : "��");
    xil_printf("�ȶ���: %s\r\n", status.is_stable ? "�ȶ�" : "���ȶ�");
    xil_printf("ʱ��Ƶ��: %.6f MHz\r\n", g_TimeSyncCtrl.current_frequency / 1000000);
    xil_printf("===================\r\n");
}t

    // ���ù㲥��ַ
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
    
    // ����preciseOriginTimestamp�ֶΣ���Sync��Ϣ�л�ȡ��
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// ׼��Announce��Ϣ��ʹ��TxBuffer�ṹ�壩
void PrepareAnnounceMessage(TxBuffer *announce_buf) {
    // ��ջ�����
    memset(announce_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    announce_buf->frmLen[0] = 0x48;  // ���ֽ�
    announce_buf->frmLen[1] = 0x00;  // ���ֽ�
    announce_buf->messageLength[0] = 0x48;  // ���ֽ�
    announce_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    announce_buf->messageType = ANNOUNCE;
    announce_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    announce_buf->ethType[0] = 0x88;
    announce_buf->ethType[1] = 0xF7;
    
    // ���ù㲥��ַ
    memset(announce_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[ANNOUNCE];
    announce_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    announce_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[ANNOUNCE]++;
    
    // ����Announce��Ϣ�ض��ֶ�
    announce_buf->controlFiled = 5;  // Announce��Ϣ��controlFieldֵΪ5
    
    // ����ʱ��Դ��Ϣ
    announce_buf->timeSource = 0x20;  // GPSʱ��Դ
}

// ׼��PdelayReq��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // ��ջ�����
    memset(pdelay_req_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_req_buf->frmLen[0] = 0x34;  // ���ֽ�
    pdelay_req_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_req_buf->messageLength[0] = 0x34;  // ���ֽ�
    pdelay_req_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_req_buf->messageType = PDELAY_REQ;
    pdelay_req_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_req_buf->ethType[0] = 0x88;
    pdelay_req_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ��ʱ��MAC
    // �˴�Ӧ����Ϊ��ʱ�ӵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_req_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_REQ];
    pdelay_req_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_req_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_REQ]++;
}

// ׼��PdelayResp��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayRespMessage(TxBuffer *pdelay_resp_buf, UINT8 *requesting_port_id) {
    // ��ջ�����
    memset(pdelay_resp_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_resp_buf->frmLen[0] = 0x3C;  // ���ֽ�
    pdelay_resp_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_resp_buf->messageLength[0] = 0x3C;  // ���ֽ�
    pdelay_resp_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_resp_buf->messageType = PDELAY_RESP;
    pdelay_resp_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_resp_buf->ethType[0] = 0x88;
    pdelay_resp_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ����˿ڵ�MAC��ַ
    // �˴�Ӧ����Ϊ����˿ڵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_resp_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_RESP];
    pdelay_resp_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_resp_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_RESP]++;
    
    // ��������˿�ID
    memcpy(pdelay_resp_buf->reqPortId, requesting_port_id, 10);
}

// ׼��PdelayFollowUp��Ϣ��ʹ��TxBuffer�ṹ�壩
void PreparePdelayFollowUpMessage(TxBuffer *pdelay_followup_buf, UINT8 *requesting_port_id, UINT64 request_receive_timestamp) {
    // ��ջ�����
    memset(pdelay_followup_buf, 0, sizeof(TxBuffer));
    
    // ����֡���Ⱥ���Ϣ����
    pdelay_followup_buf->frmLen[0] = 0x44;  // ���ֽ�
    pdelay_followup_buf->frmLen[1] = 0x00;  // ���ֽ�
    pdelay_followup_buf->messageLength[0] = 0x44;  // ���ֽ�
    pdelay_followup_buf->messageLength[1] = 0x00;  // ���ֽ�
    
    // ������Ϣ���ͺͰ汾
    pdelay_followup_buf->messageType = PDELAY_FOLLOWUP;
    pdelay_followup_buf->versionPTP = 0x02;  // PTPv2
    
    // ������̫������ΪPTP
    pdelay_followup_buf->ethType[0] = 0x88;
    pdelay_followup_buf->ethType[1] = 0xF7;
    
    // ����Ŀ���ַΪ����˿ڵ�MAC��ַ
    // �˴�Ӧ����Ϊ����˿ڵ�MAC��ַ��ʾ����ʹ�ù㲥��ַ
    memset(pdelay_followup_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP];
    pdelay_followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP]++;
    
    // ��������˿�ID
    memcpy(pdelay_followup_buf->reqPortId, requesting_port_id, 10);
    
    // ����requestReceiptTimestamp�ֶ�
    for (int i = 0; i < 8; i++) {
        pdelay_followup_buf->originTimestamp[i] = (request_receive_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// ����PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
BOOL SendPTPMessage(TxBuffer *tx_buf, UINT32 buffer_num) {
    // ���TX Buffer�Ƿ����
    if (IsTXBufferBusy(buffer_num)) {
        return FALSE;
    }
    
    // ���TX BufferΪæ״̬
    UINT32 msg_length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    MarkTXBufferBusy(buffer_num, msg_length);
    
    // ������д�뷢�ͻ�������ʹ��32λд�룩
    for (UINT32 i = 0; i < 64; i++) {
        Write32(TSN_BASEADDR + XPAR_TSN_0_TX_DATA_OFFSET + i * 4, tx_buf->buf[i]);
    }
    
    // ��������
    UINT32 control = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET);
    control |= (1 << buffer_num);  // ������Ӧ�������ķ���
    Write32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET, control);
    
    // ����ϵͳͳ��
    UpdateSystemStats(tx_buf->messageType, msg_length, TRUE);
    
    return TRUE;
}

// ��鷢��״̬
BOOL CheckTxStatus(UINT32 buffer_num) {
    if (buffer_num > 4) return FALSE; // ��Ч���������
    
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    return (status & (1 << buffer_num)) != 0;
}

// ��鲢���������
void ProcessTxComplete(void) {
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    
    for (UINT32 i = 0; i < 5; i++) {
        if ((status & (1 << i)) != 0) {
            // ������ɣ����״̬λ
            Write32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET, 1 << i);
            
            // ���TX BufferΪ����
            MarkTXBufferFree(i);
        }
    }
}

// ������
int main() {
    // ��ʼ��Ӳ��
    InitSystemHardware();
    
    // ��ʼ��TSN����������ַ
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;
    
    // ��ʼ��ʱ��ͬ��������
    InitTimeSyncController();
    
    // ��ʼ��SequenceID�����
    InitSeqIdChecker();
    
    // ��ʼ��TX Buffer״̬
    InitTXBufferStatus();
    
    // ��ʼ��ϵͳͳ��
    InitSystemStats();
    
    // ��ӡ��ʼ����Ϣ
    xil_printf("TSN Time Synchronization System initialized\r\n");
    
    // ��ѭ��
    while (1) {
        // ���������Ϣ
        if (ReceivePTPMessage(&RxBuffer)) {
            ProcessPTPMessage(&RxBuffer);
        }
        
        // ���������״̬
        ProcessTxComplete();
        
        // ���ʱ��ͬ���ȶ���
        MonitorTimeSyncStability();
        
        // ��鲢ִ�ж�ʱ����
        HandlePeriodicTasks();
        
        // ���ϵͳ״̬
        CheckSystemStatus();
        
        // ��ӡͬ��״̬��Ϣ��ÿ10��һ�Σ�
        static UINT32 last_print_time = 0;
        UINT32 current_time = GetSystemTime() / 1000000000; // ת��Ϊ��
        
        if (current_time - last_print_time >= 10) {
            PrintSyncStatus();
            last_print_time = current_time;
        }
    }
    
    return 0;
}

// ����ʱ����
void HandlePeriodicTasks() {
    static UINT64 last_sync_time = 0;
    static UINT64 last_announce_time = 0;
    static UINT64 last_pdelay_time = 0;
    
    UINT64 current_time = GetSystemTime();
    
    // ÿSYNC_INTERVAL����һ��Sync��Ϣ����Ϊ��ʱ��ʱ��
    if (current_time - last_sync_time >= SYNC_INTERVAL) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // ׼��Sync��Ϣ
            PrepareSyncMessage(&SyncBuf);
            
            // ��ȡ��ǰʱ����ΪSync��Ϣ��ʱ���
            UINT64 sync_timestamp = GetSystemTime();
            
            // ����Sync��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&SyncBuf, tx_buffer);
                
                // ��¼����ʱ�䣬���ں���FollowUp��Ϣ
                last_sync_time = current_time;
                
                // ׼��������FollowUp��Ϣ
                PrepareFollowUpMessage(&FollowUpBuf, sync_timestamp);
                tx_buffer = GetAvailableTXBuffer();
                if (tx_buffer != 0xFFFFFFFF) {
                    SendPTPMessage(&FollowUpBuf, tx_buffer);
                }
            }
        }
    }
    
    // ÿ5�뷢��һ��Announce��Ϣ����Ϊ��ʱ��ʱ��
    if (current_time - last_announce_time >= 5000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // ׼��Announce��Ϣ
            PrepareAnnounceMessage(&AnnounceBuf);
            
            // ����Announce��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&AnnounceBuf, tx_buffer);
                last_announce_time = current_time;
            }
        }
    }
    
    // ÿ10�뷢��һ��PdelayReq��Ϣ����Ϊ�߽�ʱ�ӻ��ʱ��ʱ��
    if (current_time - last_pdelay_time >= 10000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED || g_TimeSyncCtrl.state == SYNCHRONIZING) {
            // ׼��PdelayReq��Ϣ
            PreparePdelayReqMessage(&PdelayReqBuf);
            
            // ����PdelayReq��Ϣ
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&PdelayReqBuf, tx_buffer);
                last_pdelay_time = current_time;
            }
        }
    }
}

// ���ϵͳ״̬
void CheckSystemStatus() {
    // ���Ӳ��״̬
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET);
    
    // ���ʱ��״̬
    if ((status & 0x02) == 0) {  // ���ʱ�Ӷ�ʧ����
        HandleSystemError("Clock lock lost");
    }
    
    // �������ӿ�״̬
    if ((status & 0x04) == 0) {  // ���������·�Ͽ�
        HandleSystemError("Network link down");
    }
    
    // ���PTP����״̬
    if ((status & 0x08) != 0) {  // ���PTP�������
        HandleSystemError("PTP engine error");
        // ��������־
        Write32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET, 0x08);
    }
}

// ��ȡͬ��״̬��Ϣ
SyncStatus GetSyncStatus() {
    UpdateSyncStatus();
    return g_TimeSyncCtrl.status;
}

// ��ӡͬ��״̬��Ϣ
void PrintSyncStatus() {
    SyncStatus status = GetSyncStatus();
    
    xil_printf("=== ͬ��״̬��Ϣ ===\r\n");
    xil_printf("״̬: %s\r\n", status.state_text);
    xil_printf("״̬����ʱ��: %u ms\r\n", status.state_duration);
    xil_printf("״̬ת������: %u\r\n", status.state_transitions);
    xil_printf("���ϴ�ͬ��ʱ��: %u ms\r\n", status.time_since_sync);
    xil_printf("ͬ��ʧ�ܼ���: %u\r\n", status.sync_failure_count);
    xil_printf("ͬ������: %.2f ns\r\n", status.sync_accuracy);
    xil_printf("ƫ������: %.2f ns/s\r\n", status.offset_trend);
    xil_printf("����״̬: %s\r\n", status.is_locked ? "��" : "��");
    xil_printf("ͬ��״̬: %s\r\n", status.is_synchronized ? "��" : "��");
    xil_printf("�ȶ���: %s\r\n", status.is_stable ? "�ȶ�" : "���ȶ�");
    xil_printf("ʱ��Ƶ��: %.6f MHz\r\n", g_TimeSyncCtrl.current_frequency / 1000000);
    xil_printf("===================\r\n");
}
