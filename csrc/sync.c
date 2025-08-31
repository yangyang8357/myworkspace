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

// ȫ�ֱ�������
UINT32 TSN_BASEADDR;             // TSN����������ַ
TimeSyncController g_TimeSyncCtrl;

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

// ��ȡ��ǰͬ��״̬
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
    xil_printf("===================\r\n");
}
