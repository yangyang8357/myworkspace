#include "xil_types.h"
#include "xil_io.h"
#include <math.h>
#include <string.h>

// 定义时钟相关常量
#define CLOCK_FREQ_HZ       125000000  // 时钟基础频率 (125MHz)
#define NS_PER_SECOND       1000000000 // 每秒纳秒数
#define STEP_COUNTER_BITS   32         // 步长计数器位数
#define STEP_COUNTER_MAX    0xFFFFFFFF // 步长计数器最大值
#define STEP_COUNTER_DEFAULT 0x00800000 // 步长计数器默认值
#define NS_PER_STEP         8          // 每个步长代表的纳秒数

// 计算时钟周期对应的纳秒数
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// 消息类型枚举
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// 智能时间同步控制器结构体
typedef struct {
    double time_offset;          // 时间偏移量（从机-主机，纳秒）
    UINT64 last_sync_time;       // 上次同步时间
    double slow_threshold;       // 从机慢于主机时的阈值（1秒 = 1e9纳秒）
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

// 全局变量声明
extern UINT32 TSN_BASEADDR;      // TSN控制器基地址
TimeSyncController g_TimeSyncCtrl;
SeqIdChecker g_SeqIdChecker;
TXBufferStatus g_TxBufferStatus;
SystemStats g_Stats;

// I/O操作函数
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
}

// 获取系统时间（纳秒）
UINT64 GetSystemTime() {
    UINT32 high = Read32(TSN_BASEADDR + RTC_SECONDS_MSB_FIELD_OFFSET);
    UINT32 low = Read32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET);
    UINT32 step_counter = Read32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET);
    
    // 将步长计数器值转换为实际纳秒值
    UINT64 nanoseconds = (UINT64)step_counter * NS_PER_STEP;
    
    // 组合秒和纳秒部分
    UINT64 seconds = ((UINT64)high << 32) | low;
    UINT64 total_nanoseconds = (seconds * NS_PER_SECOND) + nanoseconds;
    
    return total_nanoseconds;
}

// 初始化时间同步控制器
void InitTimeSyncController() {
    g_TimeSyncCtrl.slow_threshold = 1000000000;  // 1秒（纳秒）
    
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
    g_TimeSyncCtrl.adjustment_count = 0;
    memset(g_TimeSyncCtrl.adjustment_history, 0, sizeof(g_TimeSyncCtrl.adjustment_history));
    
    // 初始化稳定性指标
    g_TimeSyncCtrl.stability_mean = 0;
    g_TimeSyncCtrl.stability_std_dev = 0;
    g_TimeSyncCtrl.stability_count = 0;
    g_TimeSyncCtrl.stability_warning = 0;
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
    WriteRegister(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);
    
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
    
    // 计算新的频率（加速时钟追赶主机时间）
    double new_frequency = g_TimeSyncCtrl.base_frequency * (1.0 + freq_adjust_ratio);
    
    // 应用频率调整
    g_TimeSyncCtrl.current_frequency = new_frequency;
    
    // 转换为步长计数器值
    UINT32 step_counter_value = (UINT32)((NS_PER_SECOND / new_frequency) / NS_PER_STEP);
    
    // 写入步长计数器寄存器
    WriteRegister(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);
    
    // 记录调整历史
    UpdateAdjustmentHistory(freq_adjust_ratio);
}

// 执行时间同步调整
void PerformTimeSync(double measured_offset) {
    g_TimeSyncCtrl.time_offset = measured_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // 策略选择
    if (measured_offset < 0) {
        // 情况1：从机时间快于主机（offset为负）
        ApplySmoothAdjustment(measured_offset);
    } else if (measured_offset < g_TimeSyncCtrl.slow_threshold) {
        // 情况2：从机时间慢于主机但未超过阈值
        ApplySmoothAdjustment(measured_offset);
    } else {
        // 情况3：从机时间慢于主机且超过阈值
        ApplyMultiStepAdjustment(measured_offset);
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
}

// 监控时间同步稳定性
void MonitorTimeSyncStability(void) {
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
        
        // 可根据稳定性指标调整同步策略
        if (std_dev > 0.00001) {
            g_TimeSyncCtrl.stability_warning = 1;
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
