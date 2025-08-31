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

// 计算时钟周期对应的纳秒数
#define NS_PER_CLOCK_CYCLE  ((double)NS_PER_SECOND / CLOCK_FREQ_HZ) // 8ns

// 消息类型枚举
#define SYNC            0
#define FOLLOWUP        1
#define ANNOUNCE        2
#define PDELAY_REQ      3
#define PDELAY_RESP     4
#define PDELAY_FOLLOWUP 5

// 时间同步状态枚举
typedef enum {
    INITIALIZING,      // 初始化状态
    SYNCHRONIZING,    // 同步中
    SYNCHRONIZED,     // 已同步
    LOSING_SYNC,      // 同步丢失
    RECOVERING        // 恢复同步
} SyncState;

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
    UINT32 sync_failure_count;   // 同步失败计数
    double sync_accuracy;        // 同步精度估计
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

// I/O操作函数
UINT32 Read32(UINT32 addr) {
    return Xil_In32(addr);
}

void Write32(UINT32 addr, UINT32 data) {
    Xil_Out32(addr, data);
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
    g_TimeSyncCtrl.sync_failure_count = 0;
    g_TimeSyncCtrl.sync_accuracy = 0;
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

// 执行时间同步调整
void PerformTimeSync(double measured_offset) {
    g_TimeSyncCtrl.time_offset = measured_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // 更新同步精度估计
    g_TimeSyncCtrl.sync_accuracy = fabs(measured_offset);
    
    // 状态转换逻辑
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
    
    // 策略选择
    if (measured_offset < g_TimeSyncCtrl.fast_threshold) {
        // 情况1：从机时间显著快于主机
        ApplyMultiStepAdjustment(measured_offset);
    } else if (measured_offset > g_TimeSyncCtrl.slow_threshold) {
        // 情况2：从机时间显著慢于主机
        ApplyMultiStepAdjustment(measured_offset);
    } else {
        // 情况3：小偏移，使用平滑调整
        ApplySmoothAdjustment(measured_offset);
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
                g_TimeSyncCtrl.state = LOSING_SYNC;
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

// 接收PTP消息（使用TxBuffer结构体）
BOOL ReceivePTPMessage(TxBuffer *rx_buf) {
    // 检查网络接口是否有新消息
    // 将接收到的数据复制到rx_buf
    // 这里需要根据实际硬件实现
    return FALSE;
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
    memset(followup_buf->dstAddr, 0xFF, 6);
    
    // 设置SequenceID
    UINT16 seq_id = g_SeqIdChecker.expected_seq_id[FOLLOWUP];
    followup_buf->sequenceID[0] = (seq_id >> 8) & 0xFF;
    followup_buf->sequenceID[1] = seq_id & 0xFF;
    
    // 设置Sync消息的精确发送时间（originTimestamp字段）
    for (int i = 0; i < 8; i++) {
        followup_buf->originTimestamp[i] = (sync_origin_timestamp >> (56 - i * 8)) & 0xFF;
    }
    
    // 更新SequenceID
    g_SeqIdChecker.expected_seq_id[FOLLOWUP]++;
}

// 发送PTP消息（使用TxBuffer结构体）
BOOL SendPTPMessage(UINT8 msg_type, TxBuffer *tx_buf) {
    // 获取消息长度
    UINT32 length = (tx_buf->messageLength[0] << 8) | tx_buf->messageLength[1];
    
    // 获取可用的TX Buffer编号
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
    
    // 检查TX Buffer是否忙
    if (IsTXBufferBusy(buffer_num)) {
        g_Stats.tx_error_count++;
        return FALSE;
    }
    
    // 标记TX Buffer为忙状态
    MarkTXBufferBusy(buffer_num, length);
    
    // 将消息复制到硬件发送缓冲区
    // 这里需要根据实际硬件实现
    
    // 启动发送过程
    // 这里需要根据实际硬件实现
    
    // 更新系统统计
    UpdateSystemStats(msg_type, length, TRUE);
    
    return TRUE;
}

// 检查是否需要发送Sync消息
BOOL TimeToSendSync(void) {
    static UINT64 last_send_time = 0;
    UINT64 current_time = GetSystemTime();
    
    // 检查是否达到发送间隔
    if (current_time - last_send_time >= SYNC_INTERVAL) {
        last_send_time = current_time;
        return TRUE;
    }
    
    return FALSE;
}

// 主函数
int main(void) {
    TxBuffer rx_buf;  // 接收缓冲区
    
    // 1. 系统初始化（硬件配置、中断设置等）
    InitSystemHardware();
    
    // 2. 初始化全局变量和控制器
    TSN_BASEADDR = XPAR_TSN_0_S00_AXI_BASEADDR;  // 设置TSN控制器基地址
    
    // 3. 初始化各功能模块
    InitTimeSyncController();     // 初始化时间同步控制器
    InitSeqIdChecker();           // 初始化SequenceID检测器
    InitTXBufferStatus();         // 初始化TX Buffer状态
    InitSystemStats();            // 初始化系统统计
    
    // 4. 主循环（轮询或事件驱动）
    while (1) {
        // 4.1 接收PTP消息并处理
        if (ReceivePTPMessage(&rx_buf)) {
            // 处理接收到的PTP消息
            ProcessPTPMessage(&rx_buf);
        }
        
        // 4.2 发送Sync消息（如果是主时钟）
        if (TimeToSendSync()) {
            // 记录发送时间
            UINT64 send_time = GetSystemTime();
            
            // 准备Sync消息
            PrepareSyncMessage(&SyncBuf);
            
            // 发送Sync消息
            SendPTPMessage(SYNC, &SyncBuf);
            
            // 准备并发送FollowUp消息
            PrepareFollowUpMessage(&FollowUpBuf, send_time);
            SendPTPMessage(FOLLOWUP, &FollowUpBuf);
        }
        
        // 4.3 监控时间同步稳定性
        MonitorTimeSyncStability();
        
        // 4.4 其他系统任务...
    }
    
    return 0;
}
