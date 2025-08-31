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

// ����ʱ�����ڶ�Ӧ��������
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// ��Ϣ����ö��
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// ʱ��ͬ��״̬ö��
typedef enum {
    INITIALIZING,      // ��ʼ��״̬
    SYNCHRONIZING,    // ͬ����
    SYNCHRONIZED,     // ��ͬ��
    LOSING_SYNC,      // ͬ����ʧ
    RECOVERING        // �ָ�ͬ��
} SyncState;

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
    UINT32 sync_failure_count;   // ͬ��ʧ�ܼ���
    double sync_accuracy;        // ͬ�����ȹ���
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

// I/O��������
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
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
    g_TimeSyncCtrl.sync_failure_count = 0;
    g_TimeSyncCtrl.sync_accuracy = 0;
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

// ִ��ʱ��ͬ������
void PerformTimeSync(double measured_offset) {
    g_TimeSyncCtrl.time_offset = measured_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // ����ͬ�����ȹ���
    g_TimeSyncCtrl.sync_accuracy = fabs(measured_offset);
    
    // ״̬ת���߼�
    if (g_TimeSyncCtrl.state == INITIALIZING) {
        g_TimeSyncCtrl.state = SYNCHRONIZING;
    } else if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
        if (fabs(measured_offset) > g_TimeSyncCtrl.slow_threshold * 2) {
            g_TimeSyncCtrl.state = LOSING_SYNC;
            g_TimeSyncCtrl.sync_failure_count++;
        }
    } else if (g_TimeSyncCtrl.state == LOSING_SYNC) {
        if (fabs(measured_offset) < g_TimeSyncCtrl.slow_threshold) {
            g_TimeSyncCtrl.state = SYNCHRONIZED;
            g_TimeSyncCtrl.sync_failure_count = 0;
        } else {
            g_TimeSyncCtrl.sync_failure_count++;
            if (g_TimeSyncCtrl.sync_failure_count > 5) {
                g_TimeSyncCtrl.state = RECOVERING;
            }
        }
    } else if (g_TimeSyncCtrl.state == RECOVERING) {
        if (fabs(measured_offset) < g_TimeSyncCtrl.slow_threshold) {
            g_TimeSyncCtrl.state = SYNCHRONIZED;
            g_TimeSyncCtrl.sync_failure_count = 0;
        }
    }
    
    // ����ѡ��
    if (measured_offset < g_TimeSyncCtrl.fast_threshold) {
        // ���1���ӻ�ʱ��������������
        ApplyMultiStepAdjustment(measured_offset);
    } else if (measured_offset > g_TimeSyncCtrl.slow_threshold) {
        // ���2���ӻ�ʱ��������������
        ApplyMultiStepAdjustment(measured_offset);
    } else {
        // ���3��Сƫ�ƣ�ʹ��ƽ������
        ApplySmoothAdjustment(measured_offset);
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
                g_TimeSyncCtrl.state = LOSING_SYNC;
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

// ����PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
BOOL ReceivePTPMessage(TxBuffer *rx_buf) {
    // �������ӿ��Ƿ�������Ϣ
    // �����յ������ݸ��Ƶ�rx_buf
    // ������Ҫ����ʵ��Ӳ��ʵ��
    return FALSE;
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
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // ����SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // ����Sync��Ϣ�ľ�ȷ����ʱ�䣨originTimestamp�ֶΣ�
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
    
    // ����SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
}

// ����PTP��Ϣ��ʹ��TxBuffer�ṹ�壩
BOOL SendPTPMessage(UINT8 msg_type, TxBuffer *tx_buf) {
    // ��ȡ��Ϣ����
    UINT32 length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    
    // ��ȡ���õ�TX Buffer���
    UINT32 buffer_num;
    switch (msg_type) {
        case SYNC:
            buffer_num = 0;
            break;
        case FOLLOWUP:
            buffer_num = 1;
            break;
        case ANNOUNCE:
            buffer_num = 2;
            break;
        case PDELAY_REQ:
            buffer_num = 3;
            break;
        case PDELAY_RESP:
            buffer_num = 4;
            break;
        default:
            return FALSE;
    }
    
    // ���TX Buffer�Ƿ�æ
    if (IsTXBufferBusy(buffer_num)) {
        g_Stats.tx_error_count++;
        return FALSE;
    }
    
    // ���TX BufferΪæ״̬
    MarkTXBufferBusy(buffer_num, length);
    
    // ����Ϣ���Ƶ�Ӳ�����ͻ�����
    // ������Ҫ����ʵ��Ӳ��ʵ��
    
    // �������͹���
    // ������Ҫ����ʵ��Ӳ��ʵ��
    
    // ����ϵͳͳ��
    UpdateSystemStats(msg_type, length, TRUE);
    
    return TRUE;
}

// ����Ƿ���Ҫ����Sync��Ϣ
BOOL TimeToSendSync(void) {
    static UINT64 last_send_time = 0;
    UINT64 current_time = GetSystemTime();
    
    // ����Ƿ�ﵽ���ͼ��
    if (current_time - last_send_time >= SYNC_INTERVAL) {
        last_send_time = current_time;
        return TRUE;
    }
    
    return FALSE;
}

// ������
int main(void) {
    TxBuffer rx_buf;  // ���ջ�����
    
    // 1. ϵͳ��ʼ����Ӳ�����á��ж����õȣ�
    InitSystemHardware();
    
    // 2. ��ʼ��ȫ�ֱ����Ϳ�����
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;  // ����TSN����������ַ
    
    // 3. ��ʼ��������ģ��
    InitTimeSyncController();     // ��ʼ��ʱ��ͬ��������
    InitSeqIdChecker();           // ��ʼ��SequenceID�����
    InitTXBufferStatus();         // ��ʼ��TX Buffer״̬
    InitSystemStats();            // ��ʼ��ϵͳͳ��
    
    // 4. ��ѭ������ѯ���¼�������
    while (1) {
        // 4.1 ����PTP��Ϣ������
        if (ReceivePTPMessage(&rx_buf)) {
            // ������յ���PTP��Ϣ
            ProcessPTPMessage(&rx_buf);
        }
        
        // 4.2 ����Sync��Ϣ���������ʱ�ӣ�
        if (TimeToSendSync()) {
            // ��¼����ʱ��
            UINT64 send_time = GetSystemTime();
            
            // ׼��Sync��Ϣ
            PrepareSyncMessage(&SyncBuf);
            
            // ����Sync��Ϣ
            SendPTPMessage(SYNC, &SyncBuf);
            
            // ׼��������FollowUp��Ϣ
            PrepareFollowUpMessage(&FollowUpBuf, send_time);
            SendPTPMessage(FOLLOWUP, &FollowUpBuf);
        }
        
        // 4.3 ���ʱ��ͬ���ȶ���
        MonitorTimeSyncStability();
        
        // 4.4 ����ϵͳ����...
    }
    
    return 0;
}
