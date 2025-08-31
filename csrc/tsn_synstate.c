#include "xil_types.h"
#include "xil_io.h"
#include "xparameters.h"
#include <math.h>
#include <string.h>

// 定义时钟相关常量
#define CLOCK_FREQ_HZ       125000000  // 时钟基础频率 (125MHz)
#define NS_PER_SECOND       1000000000 // 每秒纳秒数
#define STEP_COUNTER_BITS   32         // 步长计数器位数
#define STEP_COUNTER_MAX    0xFFFFFFFF // 步长计数器最大值
#define NS_PER_STEP         8          // 每个步长代表的纳秒数
#define SYNC_INTERVAL       1000000000 // 同步消息发送间隔 (1秒)
#define STABILITY_CHECK_INTERVAL 100   // 稳定性检查间隔 (100次调整)

// 卡尔曼滤波参数
#define KF_Q                0.001      // 过程噪声协方差
#define KF_R                0.1        // 测量噪声协方差
#define KF_INIT_P           1.0        // 初始估计误差协方差
#define KF_INIT_X           0.0        // 初始状态估计

// 计算时钟周期对应的纳秒数
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// 消息类型枚举
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// 同步状态枚举
typedef enum {
    INITIALIZING,      // 初始化状态
    SYNCHRONIZING,    // 同步中
    SYNCHRONIZED,     // 已同步
    LOSING_SYNC,      // 同步丢失
    RECOVERING,       // 恢复同步
    HOLDING,          // 保持状态（无主时钟信号）
    FAULTY           // 故障状态
} SyncState;

// 同步状态详细信息结构体
typedef struct {
    SyncState state;           // 当前同步状态
    UINT32 state_duration;     // 当前状态持续时间（毫秒）
    UINT32 state_transitions;  // 状态转换次数
    UINT32 time_since_sync;    // 自上次成功同步以来的时间（毫秒）
    UINT32 sync_failure_count; // 同步失败计数
    double sync_accuracy;      // 同步精度估计（纳秒）
    double offset_trend;       // 偏移趋势（纳秒/秒）
    BOOL is_locked;            // 是否锁定到主时钟
    BOOL is_synchronized;      // 是否同步
    BOOL is_stable;            // 同步是否稳定
    char state_text[32];       // 状态文本描述
} SyncStatus;

// 卡尔曼滤波器结构体
typedef struct {
    double x;        // 状态估计（时间偏移）
    double P;        // 估计误差协方差
    double K;        // 卡尔曼增益
    double Q;        // 过程噪声协方差
    double R;        // 测量噪声协方差
    UINT64 last_time; // 上次更新时间
} KalmanFilter;

// 智能时间同步控制器结构体
typedef struct {
    double time_offset;          // 时间偏移量（从机-主机，纳秒）
    UINT64 last_sync_time;       // 上次同步时间
    UINT64 last_announce_time;   // 上次收到Announce消息时间
    double slow_threshold;       // 从机慢于主机时的阈值（纳秒）
    double fast_threshold;       // 从机快于主机时的阈值（纳秒）
    double step_levels[3];       // 三级步长值
    double step_thresholds[3];   // 步长切换阈值
    double smooth_factor;        // 平滑因子
    double max_freq_adjust;      // 最大频率调整率
    double base_frequency;       // 基础频率（Hz）
    double current_frequency;    // 当前频率
    UINT32 adjustment_count;     // 调整次数统计
    double adjustment_history[100]; // 最近100次调整历史
    
    // 稳定性监控指标
    double stability_mean;       // 调整平均值
    double stability_std_dev;    // 调整标准差
    int    stability_count;      // 有效样本数
    int    stability_warning;    // 稳定性警告标志
    
    // 同步状态
    SyncState state;             // 当前同步状态
    UINT32 state_start_time;     // 当前状态开始时间
    UINT32 state_duration;       // 当前状态持续时间
    UINT32 state_transitions;    // 状态转换次数
    SyncStatus status;           // 同步状态详细信息
    
    // 卡尔曼滤波器
    KalmanFilter kf;             // 时间偏移卡尔曼滤波器
} TimeSyncController;

// SequenceID检测结构体
typedef struct {
    UINT16 expected_seq_id[6];   // 每个消息类型期望的下一个SequenceID
    UINT16 last_valid_seq_id[6]; // 每个消息类型最后一个有效的SequenceID
    UINT32 missed_count[6];      // 每个消息类型丢失的消息数
    UINT32 duplicate_count[6];   // 每个消息类型重复的消息数
    UINT32 invalid_count[6];     // 每个消息类型无效的消息数
    UINT64 last_checked_time[6]; // 上次检查时间
} SeqIdChecker;

// TX Buffer状态结构体
typedef struct {
    BOOL isBusy[5];              // 每个TX Buffer的忙状态
    UINT32 msgLength[5];         // 每个缓冲区的消息长度
    UINT64 lastTxTime[5];        // 每个缓冲区的最后发送时间
    UINT32 txCount[5];           // 每个缓冲区的发送计数
} TXBufferStatus;

// 系统统计结构体
typedef struct {
    UINT32 total_tx_messages;    // 发送的总消息数
    UINT32 total_rx_messages;    // 接收的总消息数
    UINT32 tx_error_count;       // 发送错误计数
    UINT32 rx_error_count;       // 接收错误计数
    UINT32 tx_bytes;             // 发送的总字节数
    UINT32 rx_bytes;             // 接收的总字节数
    UINT32 ptp_message_types[6]; // 各类PTP消息计数
    UINT64 last_stats_time;      // 上次统计时间
} SystemStats;

// TxBuffer结构体定义（PTP消息缓冲区）
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
    UINT32 buf[64]; //ptp buffer 256字节
  };
} TxBuffer;

// 全局变量声明
UINT32 TSN_BASEADDR;             // TSN控制器基地址
TimeSyncController g_TimeSyncCtrl;
SeqIdChecker g_SeqIdChecker;
TXBufferStatus g_TxBufferStatus;
SystemStats g_Stats;

// 声明PTP消息缓冲区
TxBuffer SyncBuf;
TxBuffer FollowUpBuf;
TxBuffer AnnounceBuf;
TxBuffer PdelayReqBuf;
TxBuffer PdelayRespBuf;
TxBuffer RxBuffer;  // 接收缓冲区

// I/O操作函数
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
}

// 从寄存器读取数据到TxBuffer（32位读取）
void ReadRegistersToBuffer(UINT32 base_addr, TxBuffer *buffer, UINT32 size) {
    UINT32 words = (size + 3) / 4;  // 计算需要读取的32位字数量
    
    for (UINT32 i = 0; i < words; i++) {
        buffer->buf[i] = Read32(base_addr + i * 4);
    }
}

// 获取系统时间（纳秒）
UINT64 GetSystemTime() {
    UINT32 high = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_MSB_FIELD_OFFSET);
    UINT32 low = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_LSB_FIELD_OFFSET);
    UINT32 step_counter = Read32(TSN_BASEADDR + XPAR_TSN_0_RTC_NANOSECONDS_FIELD_OFFSET);
    
    // 将步长计数器值转换为实际纳秒值
    UINT64 nanoseconds = (UINT64)step_counter * NS_PER_STEP;
    
    // 组合秒和纳秒部分
    UINT64 seconds = ((UINT64)high << 32) | low;
    UINT64 total_nanoseconds = (seconds * NS_PER_SECOND) + nanoseconds;
    
    return total_nanoseconds;
}

// 写入寄存器
void WriteRegister(UINT32 addr, UINT32 data) {
    Write32(addr, data);
}

// 初始化卡尔曼滤波器
void InitKalmanFilter(KalmanFilter *kf) {
    kf->x = KF_INIT_X;
    kf->P = KF_INIT_P;
    kf->Q = KF_Q;
    kf->R = KF_R;
    kf->last_time = GetSystemTime();
}

// 更新卡尔曼滤波器
double UpdateKalmanFilter(KalmanFilter *kf, double measurement) {
    UINT64 current_time = GetSystemTime();
    double dt = (double)(current_time - kf->last_time) / NS_PER_SECOND;
    kf->last_time = current_time;
    
    // 预测步骤
    // 状态预测（假设线性变化）
    // 时间偏移预测（无变化模型）
    // 误差协方差预测
    kf->P = kf->P + kf->Q;
    
    // 更新步骤
    // 计算卡尔曼增益
    kf->K = kf->P / (kf->P + kf->R);
    
    // 更新状态估计
    kf->x = kf->x + kf->K * (measurement - kf->x);
    
    // 更新误差协方差
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->x;
}

// 初始化系统硬件
void InitSystemHardware() {
    // 配置系统时钟
    // 初始化中断控制器
    // 初始化网络接口
    // 其他硬件初始化
}

// 初始化时间同步控制器
void InitTimeSyncController() {
    g_TimeSyncCtrl.slow_threshold = 1000000000;  // 1秒（纳秒）
    g_TimeSyncCtrl.fast_threshold = -1000000000; // -1秒（纳秒）
    
    // 多级步长参数
    g_TimeSyncCtrl.step_levels[0] = 0.0001;      // 大步长（100ppm）
    g_TimeSyncCtrl.step_levels[1] = 0.00001;     // 中步长（10ppm）
    g_TimeSyncCtrl.step_levels[2] = 0.000001;    // 小步长（1ppm）
    
    g_TimeSyncCtrl.step_thresholds[0] = 5000000000;  // 5秒
    g_TimeSyncCtrl.step_thresholds[1] = 2000000000;  // 2秒
    g_TimeSyncCtrl.step_thresholds[2] = 1000000000;  // 1秒
    
    // 平滑调整参数
    g_TimeSyncCtrl.smooth_factor = 200;          // 平滑因子
    g_TimeSyncCtrl.max_freq_adjust = 0.00005;    // 最大频率调整率（50ppm）
    
    // 时钟控制参数 - 基于125MHz时钟
    g_TimeSyncCtrl.base_frequency = CLOCK_FREQ_HZ;  // 基础频率（125MHz）
    g_TimeSyncCtrl.current_frequency = g_TimeSyncCtrl.base_frequency;
    
    // 初始化状态
    g_TimeSyncCtrl.time_offset = 0;
    g_TimeSyncCtrl.last_sync_time = 0;
    g_TimeSyncCtrl.last_announce_time = 0;
    g_TimeSyncCtrl.adjustment_count = 0;
    memset(g_TimeSyncCtrl.adjustment_history, 0, sizeof(g_TimeSyncCtrl.adjustment_history));
    
    // 初始化稳定性指标
    g_TimeSyncCtrl.stability_mean = 0;
    g_TimeSyncCtrl.stability_std_dev = 0;
    g_TimeSyncCtrl.stability_count = 0;
    g_TimeSyncCtrl.stability_warning = 0;
    
    // 初始化同步状态
    g_TimeSyncCtrl.state = INITIALIZING;
    g_TimeSyncCtrl.state_start_time = GetSystemTime();
    g_TimeSyncCtrl.state_duration = 0;
    g_TimeSyncCtrl.state_transitions = 0;
    
    // 初始化同步状态详细信息
    UpdateSyncStatus();
    
    // 初始化卡尔曼滤波器
    InitKalmanFilter(&g_TimeSyncCtrl.kf);
}

// 更新同步状态详细信息
void UpdateSyncStatus() {
    SyncStatus *status = &g_TimeSyncCtrl.status;
    
    // 更新基本状态信息
    status->state = g_TimeSyncCtrl.state;
    status->state_duration = g_TimeSyncCtrl.state_duration;
    status->state_transitions = g_TimeSyncCtrl.state_transitions;
    status->time_since_sync = (UINT32)((GetSystemTime() - g_TimeSyncCtrl.last_sync_time) / 1000000);
    status->sync_failure_count = g_TimeSyncCtrl.sync_failure_count;
    status->sync_accuracy = g_TimeSyncCtrl.sync_accuracy;
    
    // 计算偏移趋势
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
            status->offset_trend = sum / count * 1000000000; // 转换为纳秒/秒
        } else {
            status->offset_trend = 0;
        }
    } else {
        status->offset_trend = 0;
    }
    
    // 设置状态标志
    status->is_locked = (g_TimeSyncCtrl.state == SYNCHRONIZED);
    status->is_synchronized = (g_TimeSyncCtrl.state == SYNCHRONIZED || 
                              g_TimeSyncCtrl.state == HOLDING);
    status->is_stable = (g_TimeSyncCtrl.stability_std_dev < 0.00001);
    
    // 设置状态文本描述
    switch (g_TimeSyncCtrl.state) {
        case INITIALIZING:
            strcpy(status->state_text, "初始化");
            break;
        case SYNCHRONIZING:
            strcpy(status->state_text, "同步中");
            break;
        case SYNCHRONIZED:
            strcpy(status->state_text, "已同步");
            break;
        case LOSING_SYNC:
            strcpy(status->state_text, "同步丢失");
            break;
        case RECOVERING:
            strcpy(status->state_text, "恢复同步");
            break;
        case HOLDING:
            strcpy(status->state_text, "保持状态");
            break;
        case FAULTY:
            strcpy(status->state_text, "故障状态");
            break;
        default:
            strcpy(status->state_text, "未知状态");
            break;
    }
}

// 切换同步状态
void ChangeSyncState(SyncState new_state) {
    if (g_TimeSyncCtrl.state != new_state) {
        UINT64 current_time = GetSystemTime();
        
        // 更新当前状态持续时间
        g_TimeSyncCtrl.state_duration = (UINT32)((current_time - g_TimeSyncCtrl.state_start_time) / 1000000);
        
        // 记录状态转换
        g_TimeSyncCtrl.state = new_state;
        g_TimeSyncCtrl.state_start_time = current_time;
        g_TimeSyncCtrl.state_transitions++;
        
        // 更新同步状态详细信息
        UpdateSyncStatus();
        
        // 状态转换日志记录
        xil_printf("Sync state changed to: %s\r\n", g_TimeSyncCtrl.status.state_text);
    }
}

// 平滑时钟调整（通过频率微调）
void ApplySmoothAdjustment(double offset) {
    double freq_adjust_ratio = offset / g_TimeSyncCtrl.smooth_factor;
    
    // 限制最大调整幅度
    if (freq_adjust_ratio > g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = g_TimeSyncCtrl.max_freq_adjust;
    } else if (freq_adjust_ratio < -g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = -g_TimeSyncCtrl.max_freq_adjust;
    }
    
    // 计算新的频率
    double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
    
    // 应用频率调整
    g_TimeSyncCtrl.current_frequency = new_frequency;
    
    // 转换为步长计数器值
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / new_frequency) / NS_PER_STEP);
    
    // 写入步长计数器寄存器
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_IVCR_OFFSET, step_counter_value);
    
    // 记录调整历史
    UpdateAdjustmentHistory(freq_adjust_ratio);
}

// 多级步长调整
void ApplyMultiStepAdjustment(double offset) {
    double abs_offset = fabs(offset);
    double step_ratio = 0;
    
    // 根据偏移量大小选择合适的步长
    if (abs_offset >= g_TimeSyncCtrl.step_thresholds[0]) {
        step_ratio = g_TimeSyncCtrl.step_levels[0];
    } else if (abs_offset >= g_TimeSyncCtrl.step_thresholds[1]) {
        step_ratio = g_TimeSyncCtrl.step_levels[1];
    } else {
        step_ratio = g_TimeSyncCtrl.step_levels[2];
    }
    
    // 计算需要调整的频率
    double freq_adjust_ratio = abs_offset * step_ratio;
    
    // 限制最大调整幅度
    if (freq_adjust_ratio > g_TimeSyncCtrl.max_freq_adjust) {
        freq_adjust_ratio = g_TimeSyncCtrl.max_freq_adjust;
    }
    
    // 根据偏移方向调整时钟
    if (offset > 0) {
        // 从机慢于主机，加速时钟
        double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
        g_TimeSyncCtrl.current_frequency = new_frequency;
    } else {
        // 从机快于主机，减速时钟
        double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 - freq_adjust_ratio);
        g_TimeSyncCtrl.current_frequency = new_frequency;
    }
    
    // 转换为步长计数器值
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / g_TimeSyncCtrl.current_frequency) / NS_PER_STEP);
    
    // 写入步长计数器寄存器
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_IVCR_OFFSET, step_counter_value);
    
    // 记录调整历史
    UpdateAdjustmentHistory(freq_adjust_ratio * (offset > 0 ? 1 : -1));
}

// 时间跳变调整（直接修改系统时间）
void ApplyTimeJump(double offset) {
    UINT64 current_time = GetSystemTime();
    UINT64 new_time = current_time - (UINT64)offset;  // 减去偏移量，使从机时间追上主机
    
    // 计算新的秒部分和纳秒部分
    UINT64 seconds = new_time / NS_PER_SECOND;
    UINT64 nanoseconds = new_time % NS_PER_SECOND;
    
    // 计算步长计数器值
    UINT32 step_counter_value = (UINT32)(nanoseconds / NS_PER_STEP);
    
    // 写入新的时间值
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_MSB_FIELD_OFFSET, (UINT32)(seconds >> 32));
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_SECONDS_LSB_FIELD_OFFSET, (UINT32)seconds);
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RTC_NANOSECONDS_FIELD_OFFSET, step_counter_value);
    
    // 记录调整历史
    UpdateAdjustmentHistory(offset / 1000000000);  // 转换为秒为单位
    
    // 状态转换：如果是从同步丢失状态进行跳变，转换到恢复同步状态
    if (g_TimeSyncCtrl.state == LOSING_SYNC || g_TimeSyncCtrl.state == RECOVERING) {
        ChangeSyncState(RECOVERING);
    }
}

// 执行时间同步调整
void PerformTimeSync(double measured_offset) {
    // 使用卡尔曼滤波器处理测量值
    double filtered_offset = UpdateKalmanFilter(&g_TimeSyncCtrl.kf, measured_offset);
    
    g_TimeSyncCtrl.time_offset = filtered_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // 更新同步精度估计
    g_TimeSyncCtrl.sync_accuracy = fabs(filtered_offset);
    
    // 更新同步状态
    UpdateSyncStatus();
    
    // 状态转换逻辑
    switch (g_TimeSyncCtrl.state) {
        case INITIALIZING:
            // 从初始化状态开始同步
            ChangeSyncState(SYNCHRONIZING);
            break;
            
        case SYNCHRONIZING:
            // 检查是否已达到同步条件
            if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold / 2) {
                ChangeSyncState(SYNCHRONIZED);
            }
            break;
            
        case SYNCHRONIZED:
            // 检查是否失去同步
            if (fabs(filtered_offset) > g_TimeSyncCtrl.slow_threshold * 2) {
                ChangeSyncState(LOSING_SYNC);
            } else if (g_TimeSyncCtrl.last_announce_time > 0 && 
                      (GetSystemTime() - g_TimeSyncCtrl.last_announce_time) > 3000000000) {
                // 长时间未收到Announce消息
                ChangeSyncState(HOLDING);
            }
            break;
            
        case LOSING_SYNC:
            // 尝试恢复同步
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
            // 检查是否已恢复同步
            if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold) {
                ChangeSyncState(SYNCHRONIZED);
                g_TimeSyncCtrl.sync_failure_count = 0;
            }
            break;
            
        case HOLDING:
            // 在保持状态下，等待新的同步消息
            if (g_TimeSyncCtrl.last_announce_time > 0 && 
               (GetSystemTime() - g_TimeSyncCtrl.last_announce_time) < 1000000000) {
                ChangeSyncState(SYNCHRONIZING);
            }
            break;
            
        case FAULTY:
            // 故障状态，需要人工干预或自动恢复机制
            // 可以添加自动恢复逻辑
            if (g_TimeSyncCtrl.adjustment_count % 100 == 0) {
                ChangeSyncState(RECOVERING);
            }
            break;
    }
    
    // 时间同步策略选择
    if (filtered_offset > g_TimeSyncCtrl.slow_threshold) {
        // 从机慢于主机超过阈值
        if (filtered_offset > 1000000000) {  // 超过1秒，执行时间跳变
            ApplyTimeJump(filtered_offset);
        } else {  // 小于1秒，使用多级步长调整频率
            ApplyMultiStepAdjustment(filtered_offset);
        }
    } else if (filtered_offset < g_TimeSyncCtrl.fast_threshold) {
        // 从机快于主机超过阈值，使用多级步长调整频率
        ApplyMultiStepAdjustment(filtered_offset);
    } else {
        // 小偏移，使用平滑调整
        ApplySmoothAdjustment(filtered_offset);
    }
    
    g_TimeSyncCtrl.last_sync_time = GetSystemTime();
}

// 初始化SequenceID检测器
void InitSeqIdChecker(void) {
    memset(&g_SeqIdChecker, 0, sizeof(SeqIdChecker));
}

// 检查SequenceID有效性
BOOL CheckSequenceID(UINT8 msg_type, UINT16 seq_id) {
    if (msg_type >= 6) {
        g_SeqIdChecker.invalid_count[msg_type]++;
        return FALSE;
    }
    
    // 检查是否是预期的SequenceID
    if (seq_id == g_SeqIdChecker.expected_seq_id[msg_type]) {
        // 有效且顺序正确
        g_SeqIdChecker.last_valid_seq_id[msg_type] = seq_id;
        g_SeqIdChecker.expected_seq_id[msg_type]++;
        g_SeqIdChecker.last_checked_time[msg_type] = GetSystemTime();
        return TRUE;
    } else if (seq_id < g_SeqIdChecker.expected_seq_id[msg_type]) {
        // 重复消息
        g_SeqIdChecker.duplicate_count[msg_type]++;
        return FALSE;
    } else {
        // 丢失消息
        g_SeqIdChecker.missed_count[msg_type] += (seq_id - g_SeqIdChecker.expected_seq_id[msg_type]);
        g_SeqIdChecker.last_valid_seq_id[msg_type] = seq_id;
        g_SeqIdChecker.expected_seq_id[msg_type] = seq_id + 1;
        g_SeqIdChecker.last_checked_time[msg_type] = GetSystemTime();
        return TRUE;
    }
}

// 初始化TX Buffer状态
void InitTXBufferStatus(void) {
    memset(&g_TxBufferStatus, 0, sizeof(TXBufferStatus));
}

// 检查TX Buffer是否忙
BOOL IsTXBufferBusy(UINT32 buffer_num) {
    if (buffer_num > 4) return TRUE; // 无效缓冲区编号
    return g_TxBufferStatus.isBusy[buffer_num];
}

// 获取可用的TX Buffer
UINT32 GetAvailableTXBuffer(void) {
    for (UINT32 i = 0; i < 5; i++) {
        if (!IsTXBufferBusy(i)) {
            return i;
        }
    }
    return 0xFFFFFFFF; // 没有可用缓冲区
}

// 标记TX Buffer为忙状态
void MarkTXBufferBusy(UINT32 buffer_num, UINT32 msg_length) {
    if (buffer_num > 4) return; // 无效缓冲区编号
    
    g_TxBufferStatus.isBusy[buffer_num] = TRUE;
    g_TxBufferStatus.msgLength[buffer_num] = msg_length;
    g_TxBufferStatus.lastTxTime[buffer_num] = GetSystemTime();
    g_TxBufferStatus.txCount[buffer_num]++;
}

// 标记TX Buffer为空闲状态
void MarkTXBufferFree(UINT32 buffer_num) {
    if (buffer_num > 4) return; // 无效缓冲区编号
    
    g_TxBufferStatus.isBusy[buffer_num] = FALSE;
}

// 初始化系统统计
void InitSystemStats(void) {
    memset(&g_Stats, 0, sizeof(SystemStats));
    g_Stats.last_stats_time = GetSystemTime();
}

// 更新系统统计
void UpdateSystemStats(UINT8 msg_type, UINT32 msg_length, BOOL is_tx) {
    if (is_tx) {
        g_Stats.total_tx_messages++;
        g_Stats.tx_bytes += msg_length;
    } else {
        g_Stats.total_rx_messages++;
        g_Stats.rx_bytes += msg_length;
    }
    
    // 统计各类PTP消息
    if (msg_type < 6) {
        g_Stats.ptp_message_types[msg_type]++;
    }
}

// 重置系统统计
void ResetSystemStats(void) {
    memset(&g_Stats, 0, sizeof(SystemStats));
    g_Stats.last_stats_time = GetSystemTime();
}

// 处理系统错误
void HandleSystemError(char *error_msg) {
    // 可添加日志记录或错误码设置
    xil_printf("System Error: %s\r\n", error_msg);
    
    // 如果发生严重错误，切换到故障状态
    ChangeSyncState(FAULTY);
}

// 监控时间同步稳定性
void MonitorTimeSyncStability(void) {
    // 每STABILITY_CHECK_INTERVAL次调整进行一次稳定性检查
    if (g_TimeSyncCtrl.adjustment_count % STABILITY_CHECK_INTERVAL != 0) {
        return;
    }
    
    // 计算最近100次调整的平均值和标准差
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
        
        // 保存统计结果到结构体
        g_TimeSyncCtrl.stability_mean = mean;
        g_TimeSyncCtrl.stability_std_dev = std_dev;
        g_TimeSyncCtrl.stability_count = count;
        
        // 更新同步状态
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

// 更新调整历史
void UpdateAdjustmentHistory(double adjustment) {
    int index = g_TimeSyncCtrl.adjustment_count % 100;
    g_TimeSyncCtrl.adjustment_history[index] = adjustment;
}

// 解析PTP消息（使用TxBuffer结构体）
BOOL ParsePTPMessage(TxBuffer *rx_buf, UINT8 *msg_type, UINT16 *seq_id, UINT64 *timestamp) {
    // 提取消息类型
    *msg_type = rx_buf->messageType;
    
    // 提取SequenceID
    *seq_id = (rx_buf->sequenceID[0] << 8) | rx_buf->sequenceID[1];
    
    // 提取时间戳（originTimestamp字段）
    *timestamp = 0;
    for (int i = 0; i < 8; i++) {  // 取前8字节作为64位时间戳
        *timestamp |= (UINT64)rx_buf->originTimestamp[i] << (56 - i * 8);
    }
    
    return TRUE;
}

// 接收PTP消息（使用32位寄存器读取到TxBuffer）
BOOL ReceivePTPMessage(TxBuffer *rx_buf) {
    // 检查接收状态寄存器，判断是否有新消息
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_RX_STATUS_OFFSET);
    
    if ((status & 0x01) == 0) {  // 如果没有新消息
        return FALSE;
    }
    
    // 从接收寄存器读取数据到缓冲区（使用32位读取）
    ReadRegistersToBuffer(TSN_BASEADDR + XPAR_TSN_0_RX_DATA_OFFSET, rx_buf, 256);
    
    // 清除接收状态标志
    WriteRegister(TSN_BASEADDR + XPAR_TSN_0_RX_STATUS_OFFSET, status & ~0x01);
    
    return TRUE;
}

// 处理接收到的PTP消息（使用TxBuffer结构体）
void ProcessPTPMessage(TxBuffer *rx_buf) {
    UINT8 msg_type;
    UINT16 seq_id;
    UINT64 timestamp;
    
    // 解析PTP消息
    if (!ParsePTPMessage(rx_buf, &msg_type, &seq_id, &timestamp)) {
        g_Stats.rx_error_count++;
        return;
    }
    
    // 检查SequenceID有效性
    if (!CheckSequenceID(msg_type, seq_id)) {
        // 处理无效SequenceID
        return;
    }
    
    // 根据消息类型处理
    switch (msg_type) {
        case SYNC:
            // 处理Sync消息
            ProcessSyncMessage(timestamp);
            break;
            
        case FOLLOWUP:
            // 处理FollowUp消息
            ProcessFollowUpMessage(timestamp);
            break;
            
        case ANNOUNCE:
            // 处理Announce消息
            ProcessAnnounceMessage(rx_buf);
            g_TimeSyncCtrl.last_announce_time = GetSystemTime();
            break;
            
        case PDELAY_REQ:
            // 处理PdelayReq消息
            ProcessPdelayReqMessage(rx_buf);
            break;
            
        case PDELAY_RESP:
            // 处理PdelayResp消息
            ProcessPdelayRespMessage(rx_buf);
            break;
            
        case PDELAY_FOLLOWUP:
            // 处理PdelayFollowUp消息
            ProcessPdelayFollowUpMessage(rx_buf);
            break;
            
        default:
            break;
    }
    
    // 计算消息长度
    UINT32 msg_length = (rx_buf->messageLength[0] << 8) | rx_buf->messageLength[1];
    
    // 更新系统统计
    UpdateSystemStats(msg_type, msg_length, FALSE);
}

// 处理Sync消息
void ProcessSyncMessage(UINT64 sync_timestamp) {
    // 记录Sync消息接收时间
    UINT64 local_receive_time = GetSystemTime();
    
    // 保存Sync时间戳，后续FollowUp消息会使用
}

// 处理FollowUp消息
void ProcessFollowUpMessage(UINT64 precise_origin_timestamp) {
    // 计算时间偏移
    double offset = CalculateTimeOffset(precise_origin_timestamp);
    
    // 执行时间同步调整
    PerformTimeSync(offset);
}

// 处理Announce消息（使用TxBuffer结构体）
void ProcessAnnounceMessage(TxBuffer *announce_buf) {
    // 解析Announce消息中的时间和优先级信息
    // 更新本地时间同步状态
}

// 处理PdelayReq消息（使用TxBuffer结构体）
void ProcessPdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // 记录PdelayReq消息接收时间
    // 准备并发送PdelayResp消息
}

// 处理PdelayResp消息（使用TxBuffer结构体）
void ProcessPdelayRespMessage(TxBuffer *pdelay_resp_buf) {
    // 记录PdelayResp消息接收时间
    // 保存PdelayResp消息中的信息，等待PdelayFollowUp消息
}

// 处理PdelayFollowUp消息（使用TxBuffer结构体）
void ProcessPdelayFollowUpMessage(TxBuffer *pdelay_followup_buf) {
    // 提取PdelayReq发送时间戳
    // 计算往返延迟和时间偏移
    // 执行时间同步调整
}

// 计算时间偏移
double CalculateTimeOffset(UINT64 master_time) {
    // 获取本地时间
    UINT64 local_time = GetSystemTime();
    
    // 计算时间偏移（从机时间 - 主机时间）
    double offset = (double)(local_time - master_time);
    
    return offset;
}

// 准备Sync消息（使用TxBuffer结构体）
void PrepareSyncMessage(TxBuffer *sync_buf) {
    // 清空缓冲区
    memset(sync_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    sync_buf->frmLen[0] = 0x2C;  // 低字节
    sync_buf->frmLen[1] = 0x00;  // 高字节
    sync_buf->messageLength[0] = 0x2C;  // 低字节
    sync_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    sync_buf->messageType = SYNC;
    sync_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    sync_buf->ethType[0] = 0x88;
    sync_buf->ethType[1] = 0xF7;
    
    // 设置广播地址
    memset(sync_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[SYNC];
    sync_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    sync_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[SYNC]++;
}

// 准备FollowUp消息（使用TxBuffer结构体）
void PrepareFollowUpMessage(TxBuffer *followup_buf, UINT64 sync_origin_timestamp) {
    // 清空缓冲区
    memset(followup_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    followup_buf->frmLen[0] = 0x34;  // 低字节
    followup_buf->frmLen[1] = 0x00;  // 高字节
    followup_buf->messageLength[0] = 0x34;  // 低字节
    followup_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    followup_buf->messageType = FOLLOWUP;
    followup_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    followup_buf->ethType[0] = 0x88;
    followup_buf->ethType[1] = 0xF7;
    
    // 设置广播地址
    memse    // 设置广播地址
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
    
    // 设置preciseOriginTimestamp字段（从Sync消息中获取）
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// 准备Announce消息（使用TxBuffer结构体）
void PrepareAnnounceMessage(TxBuffer *announce_buf) {
    // 清空缓冲区
    memset(announce_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    announce_buf->frmLen[0] = 0x48;  // 低字节
    announce_buf->frmLen[1] = 0x00;  // 高字节
    announce_buf->messageLength[0] = 0x48;  // 低字节
    announce_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    announce_buf->messageType = ANNOUNCE;
    announce_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    announce_buf->ethType[0] = 0x88;
    announce_buf->ethType[1] = 0xF7;
    
    // 设置广播地址
    memset(announce_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[ANNOUNCE];
    announce_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    announce_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[ANNOUNCE]++;
    
    // 设置Announce消息特定字段
    announce_buf->controlFiled = 5;  // Announce消息的controlField值为5
    
    // 设置时间源信息
    announce_buf->timeSource = 0x20;  // GPS时间源
}

// 准备PdelayReq消息（使用TxBuffer结构体）
void PreparePdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // 清空缓冲区
    memset(pdelay_req_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_req_buf->frmLen[0] = 0x34;  // 低字节
    pdelay_req_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_req_buf->messageLength[0] = 0x34;  // 低字节
    pdelay_req_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_req_buf->messageType = PDELAY_REQ;
    pdelay_req_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_req_buf->ethType[0] = 0x88;
    pdelay_req_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为主时钟MAC
    // 此处应设置为主时钟的MAC地址，示例中使用广播地址
    memset(pdelay_req_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_REQ];
    pdelay_req_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_req_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_REQ]++;
}

// 准备PdelayResp消息（使用TxBuffer结构体）
void PreparePdelayRespMessage(TxBuffer *pdelay_resp_buf, UINT8 *requesting_port_id) {
    // 清空缓冲区
    memset(pdelay_resp_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_resp_buf->frmLen[0] = 0x3C;  // 低字节
    pdelay_resp_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_resp_buf->messageLength[0] = 0x3C;  // 低字节
    pdelay_resp_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_resp_buf->messageType = PDELAY_RESP;
    pdelay_resp_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_resp_buf->ethType[0] = 0x88;
    pdelay_resp_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为请求端口的MAC地址
    // 此处应设置为请求端口的MAC地址，示例中使用广播地址
    memset(pdelay_resp_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_RESP];
    pdelay_resp_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_resp_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_RESP]++;
    
    // 复制请求端口ID
    memcpy(pdelay_resp_buf->reqPortId, requesting_port_id, 10);
}

// 准备PdelayFollowUp消息（使用TxBuffer结构体）
void PreparePdelayFollowUpMessage(TxBuffer *pdelay_followup_buf, UINT8 *requesting_port_id, UINT64 request_receive_timestamp) {
    // 清空缓冲区
    memset(pdelay_followup_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_followup_buf->frmLen[0] = 0x44;  // 低字节
    pdelay_followup_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_followup_buf->messageLength[0] = 0x44;  // 低字节
    pdelay_followup_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_followup_buf->messageType = PDELAY_FOLLOWUP;
    pdelay_followup_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_followup_buf->ethType[0] = 0x88;
    pdelay_followup_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为请求端口的MAC地址
    // 此处应设置为请求端口的MAC地址，示例中使用广播地址
    memset(pdelay_followup_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP];
    pdelay_followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP]++;
    
    // 复制请求端口ID
    memcpy(pdelay_followup_buf->reqPortId, requesting_port_id, 10);
    
    // 设置requestReceiptTimestamp字段
    for (int i = 0; i < 8; i++) {
        pdelay_followup_buf->originTimestamp[i] = (request_receive_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// 发送PTP消息（使用TxBuffer结构体）
BOOL SendPTPMessage(TxBuffer *tx_buf, UINT32 buffer_num) {
    // 检查TX Buffer是否可用
    if (IsTXBufferBusy(buffer_num)) {
        return FALSE;
    }
    
    // 标记TX Buffer为忙状态
    UINT32 msg_length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    MarkTXBufferBusy(buffer_num, msg_length);
    
    // 将数据写入发送缓冲区（使用32位写入）
    for (UINT32 i = 0; i < 64; i++) {
        Write32(TSN_BASEADDR + XPAR_TSN_0_TX_DATA_OFFSET + i * 4, tx_buf->buf[i]);
    }
    
    // 启动发送
    UINT32 control = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET);
    control |= (1 << buffer_num);  // 启动对应缓冲区的发送
    Write32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET, control);
    
    // 更新系统统计
    UpdateSystemStats(tx_buf->messageType, msg_length, TRUE);
    
    return TRUE;
}

// 检查发送状态
BOOL CheckTxStatus(UINT32 buffer_num) {
    if (buffer_num > 4) return FALSE; // 无效缓冲区编号
    
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    return (status & (1 << buffer_num)) != 0;
}

// 检查并处理发送完成
void ProcessTxComplete(void) {
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    
    for (UINT32 i = 0; i < 5; i++) {
        if ((status & (1 << i)) != 0) {
            // 发送完成，清除状态位
            Write32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET, 1 << i);
            
            // 标记TX Buffer为空闲
            MarkTXBufferFree(i);
        }
    }
}

// 主函数
int main() {
    // 初始化硬件
    InitSystemHardware();
    
    // 初始化TSN控制器基地址
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;
    
    // 初始化时间同步控制器
    InitTimeSyncController();
    
    // 初始化SequenceID检测器
    InitSeqIdChecker();
    
    // 初始化TX Buffer状态
    InitTXBufferStatus();
    
    // 初始化系统统计
    InitSystemStats();
    
    // 打印初始化信息
    xil_printf("TSN Time Synchronization System initialized\r\n");
    
    // 主循环
    while (1) {
        // 处理接收消息
        if (ReceivePTPMessage(&RxBuffer)) {
            ProcessPTPMessage(&RxBuffer);
        }
        
        // 处理发送完成状态
        ProcessTxComplete();
        
        // 监控时间同步稳定性
        MonitorTimeSyncStability();
        
        // 检查并执行定时任务
        HandlePeriodicTasks();
        
        // 检查系统状态
        CheckSystemStatus();
        
        // 打印同步状态信息（每10秒一次）
        static UINT32 last_print_time = 0;
        UINT32 current_time = GetSystemTime() / 1000000000; // 转换为秒
        
        if (current_time - last_print_time >= 10) {
            PrintSyncStatus();
            last_print_time = current_time;
        }
    }
    
    return 0;
}

// 处理定时任务
void HandlePeriodicTasks() {
    static UINT64 last_sync_time = 0;
    static UINT64 last_announce_time = 0;
    static UINT64 last_pdelay_time = 0;
    
    UINT64 current_time = GetSystemTime();
    
    // 每SYNC_INTERVAL发送一次Sync消息（作为主时钟时）
    if (current_time - last_sync_time >= SYNC_INTERVAL) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // 准备Sync消息
            PrepareSyncMessage(&SyncBuf);
            
            // 获取当前时间作为Sync消息的时间戳
            UINT64 sync_timestamp = GetSystemTime();
            
            // 发送Sync消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&SyncBuf, tx_buffer);
                
                // 记录发送时间，用于后续FollowUp消息
                last_sync_time = current_time;
                
                // 准备并发送FollowUp消息
                PrepareFollowUpMessage(&FollowUpBuf, sync_timestamp);
                tx_buffer = GetAvailableTXBuffer();
                if (tx_buffer != 0xFFFFFFFF) {
                    SendPTPMessage(&FollowUpBuf, tx_buffer);
                }
            }
        }
    }
    
    // 每5秒发送一次Announce消息（作为主时钟时）
    if (current_time - last_announce_time >= 5000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // 准备Announce消息
            PrepareAnnounceMessage(&AnnounceBuf);
            
            // 发送Announce消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&AnnounceBuf, tx_buffer);
                last_announce_time = current_time;
            }
        }
    }
    
    // 每10秒发送一次PdelayReq消息（作为边界时钟或从时钟时）
    if (current_time - last_pdelay_time >= 10000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED || g_TimeSyncCtrl.state == SYNCHRONIZING) {
            // 准备PdelayReq消息
            PreparePdelayReqMessage(&PdelayReqBuf);
            
            // 发送PdelayReq消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&PdelayReqBuf, tx_buffer);
                last_pdelay_time = current_time;
            }
        }
    }
}

// 检查系统状态
void CheckSystemStatus() {
    // 检查硬件状态
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET);
    
    // 检查时钟状态
    if ((status & 0x02) == 0) {  // 如果时钟丢失锁定
        HandleSystemError("Clock lock lost");
    }
    
    // 检查网络接口状态
    if ((status & 0x04) == 0) {  // 如果网络链路断开
        HandleSystemError("Network link down");
    }
    
    // 检查PTP引擎状态
    if ((status & 0x08) != 0) {  // 如果PTP引擎出错
        HandleSystemError("PTP engine error");
        // 清除错误标志
        Write32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET, 0x08);
    }
}

// 获取同步状态信息
SyncStatus GetSyncStatus() {
    UpdateSyncStatus();
    return g_TimeSyncCtrl.status;
}

// 打印同步状态信息
void PrintSyncStatus() {
    SyncStatus status = GetSyncStatus();
    
    xil_printf("=== 同步状态信息 ===\r\n");
    xil_printf("状态: %s\r\n", status.state_text);
    xil_printf("状态持续时间: %u ms\r\n", status.state_duration);
    xil_printf("状态转换次数: %u\r\n", status.state_transitions);
    xil_printf("自上次同步时间: %u ms\r\n", status.time_since_sync);
    xil_printf("同步失败计数: %u\r\n", status.sync_failure_count);
    xil_printf("同步精度: %.2f ns\r\n", status.sync_accuracy);
    xil_printf("偏移趋势: %.2f ns/s\r\n", status.offset_trend);
    xil_printf("锁定状态: %s\r\n", status.is_locked ? "是" : "否");
    xil_printf("同步状态: %s\r\n", status.is_synchronized ? "是" : "否");
    xil_printf("稳定性: %s\r\n", status.is_stable ? "稳定" : "不稳定");
    xil_printf("时钟频率: %.6f MHz\r\n", g_TimeSyncCtrl.current_frequency / 1000000);
    xil_printf("===================\r\n");
}t

    // 设置广播地址
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
    
    // 设置preciseOriginTimestamp字段（从Sync消息中获取）
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// 准备Announce消息（使用TxBuffer结构体）
void PrepareAnnounceMessage(TxBuffer *announce_buf) {
    // 清空缓冲区
    memset(announce_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    announce_buf->frmLen[0] = 0x48;  // 低字节
    announce_buf->frmLen[1] = 0x00;  // 高字节
    announce_buf->messageLength[0] = 0x48;  // 低字节
    announce_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    announce_buf->messageType = ANNOUNCE;
    announce_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    announce_buf->ethType[0] = 0x88;
    announce_buf->ethType[1] = 0xF7;
    
    // 设置广播地址
    memset(announce_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[ANNOUNCE];
    announce_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    announce_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[ANNOUNCE]++;
    
    // 设置Announce消息特定字段
    announce_buf->controlFiled = 5;  // Announce消息的controlField值为5
    
    // 设置时间源信息
    announce_buf->timeSource = 0x20;  // GPS时间源
}

// 准备PdelayReq消息（使用TxBuffer结构体）
void PreparePdelayReqMessage(TxBuffer *pdelay_req_buf) {
    // 清空缓冲区
    memset(pdelay_req_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_req_buf->frmLen[0] = 0x34;  // 低字节
    pdelay_req_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_req_buf->messageLength[0] = 0x34;  // 低字节
    pdelay_req_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_req_buf->messageType = PDELAY_REQ;
    pdelay_req_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_req_buf->ethType[0] = 0x88;
    pdelay_req_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为主时钟MAC
    // 此处应设置为主时钟的MAC地址，示例中使用广播地址
    memset(pdelay_req_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_REQ];
    pdelay_req_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_req_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_REQ]++;
}

// 准备PdelayResp消息（使用TxBuffer结构体）
void PreparePdelayRespMessage(TxBuffer *pdelay_resp_buf, UINT8 *requesting_port_id) {
    // 清空缓冲区
    memset(pdelay_resp_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_resp_buf->frmLen[0] = 0x3C;  // 低字节
    pdelay_resp_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_resp_buf->messageLength[0] = 0x3C;  // 低字节
    pdelay_resp_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_resp_buf->messageType = PDELAY_RESP;
    pdelay_resp_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_resp_buf->ethType[0] = 0x88;
    pdelay_resp_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为请求端口的MAC地址
    // 此处应设置为请求端口的MAC地址，示例中使用广播地址
    memset(pdelay_resp_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_RESP];
    pdelay_resp_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_resp_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_RESP]++;
    
    // 复制请求端口ID
    memcpy(pdelay_resp_buf->reqPortId, requesting_port_id, 10);
}

// 准备PdelayFollowUp消息（使用TxBuffer结构体）
void PreparePdelayFollowUpMessage(TxBuffer *pdelay_followup_buf, UINT8 *requesting_port_id, UINT64 request_receive_timestamp) {
    // 清空缓冲区
    memset(pdelay_followup_buf, 0, sizeof(TxBuffer));
    
    // 设置帧长度和消息长度
    pdelay_followup_buf->frmLen[0] = 0x44;  // 低字节
    pdelay_followup_buf->frmLen[1] = 0x00;  // 高字节
    pdelay_followup_buf->messageLength[0] = 0x44;  // 低字节
    pdelay_followup_buf->messageLength[1] = 0x00;  // 高字节
    
    // 设置消息类型和版本
    pdelay_followup_buf->messageType = PDELAY_FOLLOWUP;
    pdelay_followup_buf->versionPTP = 0x02;  // PTPv2
    
    // 设置以太网类型为PTP
    pdelay_followup_buf->ethType[0] = 0x88;
    pdelay_followup_buf->ethType[1] = 0xF7;
    
    // 设置目标地址为请求端口的MAC地址
    // 此处应设置为请求端口的MAC地址，示例中使用广播地址
    memset(pdelay_followup_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP];
    pdelay_followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    pdelay_followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[PDELAY_FOLLOWUP]++;
    
    // 复制请求端口ID
    memcpy(pdelay_followup_buf->reqPortId, requesting_port_id, 10);
    
    // 设置requestReceiptTimestamp字段
    for (int i = 0; i < 8; i++) {
        pdelay_followup_buf->originTimestamp[i] = (request_receive_timestamp >> (56 - i * 8)) & 0xFF;
    }
}

// 发送PTP消息（使用TxBuffer结构体）
BOOL SendPTPMessage(TxBuffer *tx_buf, UINT32 buffer_num) {
    // 检查TX Buffer是否可用
    if (IsTXBufferBusy(buffer_num)) {
        return FALSE;
    }
    
    // 标记TX Buffer为忙状态
    UINT32 msg_length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    MarkTXBufferBusy(buffer_num, msg_length);
    
    // 将数据写入发送缓冲区（使用32位写入）
    for (UINT32 i = 0; i < 64; i++) {
        Write32(TSN_BASEADDR + XPAR_TSN_0_TX_DATA_OFFSET + i * 4, tx_buf->buf[i]);
    }
    
    // 启动发送
    UINT32 control = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET);
    control |= (1 << buffer_num);  // 启动对应缓冲区的发送
    Write32(TSN_BASEADDR + XPAR_TSN_0_TX_CONTROL_OFFSET, control);
    
    // 更新系统统计
    UpdateSystemStats(tx_buf->messageType, msg_length, TRUE);
    
    return TRUE;
}

// 检查发送状态
BOOL CheckTxStatus(UINT32 buffer_num) {
    if (buffer_num > 4) return FALSE; // 无效缓冲区编号
    
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    return (status & (1 << buffer_num)) != 0;
}

// 检查并处理发送完成
void ProcessTxComplete(void) {
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET);
    
    for (UINT32 i = 0; i < 5; i++) {
        if ((status & (1 << i)) != 0) {
            // 发送完成，清除状态位
            Write32(TSN_BASEADDR + XPAR_TSN_0_TX_STATUS_OFFSET, 1 << i);
            
            // 标记TX Buffer为空闲
            MarkTXBufferFree(i);
        }
    }
}

// 主函数
int main() {
    // 初始化硬件
    InitSystemHardware();
    
    // 初始化TSN控制器基地址
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;
    
    // 初始化时间同步控制器
    InitTimeSyncController();
    
    // 初始化SequenceID检测器
    InitSeqIdChecker();
    
    // 初始化TX Buffer状态
    InitTXBufferStatus();
    
    // 初始化系统统计
    InitSystemStats();
    
    // 打印初始化信息
    xil_printf("TSN Time Synchronization System initialized\r\n");
    
    // 主循环
    while (1) {
        // 处理接收消息
        if (ReceivePTPMessage(&RxBuffer)) {
            ProcessPTPMessage(&RxBuffer);
        }
        
        // 处理发送完成状态
        ProcessTxComplete();
        
        // 监控时间同步稳定性
        MonitorTimeSyncStability();
        
        // 检查并执行定时任务
        HandlePeriodicTasks();
        
        // 检查系统状态
        CheckSystemStatus();
        
        // 打印同步状态信息（每10秒一次）
        static UINT32 last_print_time = 0;
        UINT32 current_time = GetSystemTime() / 1000000000; // 转换为秒
        
        if (current_time - last_print_time >= 10) {
            PrintSyncStatus();
            last_print_time = current_time;
        }
    }
    
    return 0;
}

// 处理定时任务
void HandlePeriodicTasks() {
    static UINT64 last_sync_time = 0;
    static UINT64 last_announce_time = 0;
    static UINT64 last_pdelay_time = 0;
    
    UINT64 current_time = GetSystemTime();
    
    // 每SYNC_INTERVAL发送一次Sync消息（作为主时钟时）
    if (current_time - last_sync_time >= SYNC_INTERVAL) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // 准备Sync消息
            PrepareSyncMessage(&SyncBuf);
            
            // 获取当前时间作为Sync消息的时间戳
            UINT64 sync_timestamp = GetSystemTime();
            
            // 发送Sync消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&SyncBuf, tx_buffer);
                
                // 记录发送时间，用于后续FollowUp消息
                last_sync_time = current_time;
                
                // 准备并发送FollowUp消息
                PrepareFollowUpMessage(&FollowUpBuf, sync_timestamp);
                tx_buffer = GetAvailableTXBuffer();
                if (tx_buffer != 0xFFFFFFFF) {
                    SendPTPMessage(&FollowUpBuf, tx_buffer);
                }
            }
        }
    }
    
    // 每5秒发送一次Announce消息（作为主时钟时）
    if (current_time - last_announce_time >= 5000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
            // 准备Announce消息
            PrepareAnnounceMessage(&AnnounceBuf);
            
            // 发送Announce消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&AnnounceBuf, tx_buffer);
                last_announce_time = current_time;
            }
        }
    }
    
    // 每10秒发送一次PdelayReq消息（作为边界时钟或从时钟时）
    if (current_time - last_pdelay_time >= 10000000000) {
        if (g_TimeSyncCtrl.state == SYNCHRONIZED || g_TimeSyncCtrl.state == SYNCHRONIZING) {
            // 准备PdelayReq消息
            PreparePdelayReqMessage(&PdelayReqBuf);
            
            // 发送PdelayReq消息
            UINT32 tx_buffer = GetAvailableTXBuffer();
            if (tx_buffer != 0xFFFFFFFF) {
                SendPTPMessage(&PdelayReqBuf, tx_buffer);
                last_pdelay_time = current_time;
            }
        }
    }
}

// 检查系统状态
void CheckSystemStatus() {
    // 检查硬件状态
    UINT32 status = Read32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET);
    
    // 检查时钟状态
    if ((status & 0x02) == 0) {  // 如果时钟丢失锁定
        HandleSystemError("Clock lock lost");
    }
    
    // 检查网络接口状态
    if ((status & 0x04) == 0) {  // 如果网络链路断开
        HandleSystemError("Network link down");
    }
    
    // 检查PTP引擎状态
    if ((status & 0x08) != 0) {  // 如果PTP引擎出错
        HandleSystemError("PTP engine error");
        // 清除错误标志
        Write32(TSN_BASEADDR + XPAR_TSN_0_STATUS_OFFSET, 0x08);
    }
}

// 获取同步状态信息
SyncStatus GetSyncStatus() {
    UpdateSyncStatus();
    return g_TimeSyncCtrl.status;
}

// 打印同步状态信息
void PrintSyncStatus() {
    SyncStatus status = GetSyncStatus();
    
    xil_printf("=== 同步状态信息 ===\r\n");
    xil_printf("状态: %s\r\n", status.state_text);
    xil_printf("状态持续时间: %u ms\r\n", status.state_duration);
    xil_printf("状态转换次数: %u\r\n", status.state_transitions);
    xil_printf("自上次同步时间: %u ms\r\n", status.time_since_sync);
    xil_printf("同步失败计数: %u\r\n", status.sync_failure_count);
    xil_printf("同步精度: %.2f ns\r\n", status.sync_accuracy);
    xil_printf("偏移趋势: %.2f ns/s\r\n", status.offset_trend);
    xil_printf("锁定状态: %s\r\n", status.is_locked ? "是" : "否");
    xil_printf("同步状态: %s\r\n", status.is_synchronized ? "是" : "否");
    xil_printf("稳定性: %s\r\n", status.is_stable ? "稳定" : "不稳定");
    xil_printf("时钟频率: %.6f MHz\r\n", g_TimeSyncCtrl.current_frequency / 1000000);
    xil_printf("===================\r\n");
}
