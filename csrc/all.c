/******************************************************************************
* Author :  YY
*
******************************************************************************/

 #include <stdio.h>
 #include "platform.h"
 #include "common.h"
 #include "xil_printf.h"
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


 #define	TSN_BASEADDR		(0x40000000)
 #define	DMA_BASEADDR_0		(0x40400000)
 #define	DMA_BASEADDR_1		(0x40410000)
 #define	BRAM_BASEADDR		(0x42000000)
 #define	IRQ_REG_BASEADDR	(0x44000000)
 //#define	INTC_BASEADDR		(0xa0000000)

 #define RCV_CFG_WORD_0_OFFSET (0x400)
 #define RCV_CFG_WORD_1_OFFSET (0x404)

 #define RX_PTP_BUFFER_BASEADDR	(TSN_BASEADDR + 0x10000)
 #define RX_PTP_BUFFER_OFFSET   	(0x100)

 #define TX_PTP_BUFFER_BASEADDR	(TSN_BASEADDR + 0x11000)
 #define TX_PTP_BUFFER_CTRL		(TSN_BASEADDR + 0x12000)

 #define TX_PTP_CTRL_OFFSET (0x12000)
 #define RX_PTP_CTRL_OFFSET (0x12004)

 #define RTC_NANOSECONDS_FIELD_OFFSET (0x12800)
 #define RTC_SECONDS_LSB_FIELD_OFFSET (0x12808)
 #define RTC_SECONDS_MSB_FIELD_OFFSET (0x1280C)

 #define RTC_IVCR_OFFSET (0x12810)
 #define RTC_IVCR_OFFSET (0x12810)
 #define CURRENT_RTC_NANOSECONDS_OFFSET (0x12814)
 #define CURRENT_RTC_SECONDS_LSB_OFFSET (0x12818)
 #define CURRENT_RTC_SECONDS_MSB_OFFSET (0x1281C)

 #define Syntonized_NANOSECONDS_FIELD_OFFSET (0x1282c)
 #define Syntonized_SECONDS_LSB_FIELD_OFFSET (0x12830)
 #define Syntonized_SECONDS_MSB_FIELD_OFFSET (0x12834)

#define INCRVALUE (0x00800000)
 #define SYNC                   0x0
 #define PDELAY_REQ             0x2
 #define PDELAY_RESP            0x3
 #define FOLLOWUP               0x8
 #define PDELAY_RESPFOLLOWUP    0xa
 #define ANNOUNCE               0xb
 #define SIGNALING              0xc

#define TWO_STEP_FLAG 0x1

 #define SYNC_BUFNUM                   0x0
 #define PDELAY_REQ_BUFNUM             0x1
 #define PDELAY_RESP_BUFNUM            0x2
 #define FOLLOWUP_BUFNUM               0x3
 #define PDELAY_RESPFOLLOWUP_BUFNUM    0x4
 #define ANNOUNCE_BUFNUM               0x5
 #define SIGNALING_BUFNUM              0x6

#define MM2S_DMACR_OFFSET    0x00
#define MM2S_DMASR_OFFSET	 0x04
#define MM2S_DA_OFFSET	 	 0x18
#define MM2S_LENGTH			 0x28

#define S2MM_DMACR_OFFSET    0x30
#define S2MM_DMASR_OFFSET	 0x34
#define S2MM_DA_OFFSET	 	 0x48
#define S2MM_LENGTH			 0x58
// tx ptp buffer

// 时间同步状态枚举
typedef enum {
    INITIALIZING,      // 初始化状态
    SYNCHRONIZING,    // 同步中
    SYNCHRONIZED,     // 已同步
    LOSING_SYNC,      // 同步丢失
    RECOVERING        // 恢复同步
} SyncState;


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
    UINT16 expected_seq_id[16];   // 每个消息类型期望的下一个SequenceID
    UINT16 last_valid_seq_id[16]; // 每个消息类型最后一个有效的SequenceID
    UINT32 missed_count[16];      // 每个消息类型丢失的消息数
    UINT32 duplicate_count[16];   // 每个消息类型重复的消息数
    UINT32 invalid_count[16];     // 每个消息类型无效的消息数
    UINT64 last_checked_time[16]; // 上次检查时间
} SeqIdChecker;

// TX Buffer状态结构体
typedef struct {
    BOOL isBusy[8];              // 每个TX Buffer的忙状态
    UINT32 msgLength[8];         // 每个缓冲区的消息长度
    UINT64 lastTxTime[8];        // 每个缓冲区的最后发送时间
    UINT32 txCount[8];           // 每个缓冲区的发送计数
} TXBufferStatus;

// 系统统计结构体
typedef struct {
    UINT32 total_tx_messages;    // 发送的总消息数
    UINT32 total_rx_messages;    // 接收的总消息数
    UINT32 tx_error_count;       // 发送错误计数
    UINT32 rx_error_count;       // 接收错误计数
    UINT32 tx_bytes;             // 发送的总字节数
    UINT32 rx_bytes;             // 接收的总字节数
    UINT32 ptp_message_types[16]; // 各类PTP消息计数
    UINT64 last_stats_time;      // 上次统计时间
} SystemStats;

// 全局变量声明
//extern UINT32 TSN_BASEADDR;      // TSN控制器基地址
TimeSyncController g_TimeSyncCtrl;
SeqIdChecker g_SeqIdChecker;
TXBufferStatus g_TxBufferStatus;
SystemStats g_Stats;




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

      }
    };

    UINT32 buf[64]; //ptp buffer 256
  };
}TxBuffer;
typedef struct
{
  union
  {
    struct
    {
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
      }
    }; 
    struct {
    UINT32   rsv[61];
    TimeStamp RxTS;
    }
    UINT32 buf[64]; //ptp buffer 256
  };
}RxBuffer;

// tx ptp buffer

RxBuffer   RxPdelayReqBuf;
RxBuffer   RxPdelayRespBuf;
RxBuffer   RxPdelayRespFollowUpBuf;
RxBuffer   RxSyncBuf;
RxBuffer   RxFollowUpBuf;
RxBuffer   RxAnnounceBuf;

TxBuffer   TxPdelayReqBuf;
TxBuffer   TxPdelayRespBuf;
TxBuffer   TxPdelayRespFollowUpBuf;
TxBuffer   TxSyncBuf;
TxBuffer   TxFollowUpBuf;
TxBuffer   TxAnnounceBuf;
 typedef struct
{
	UINT32 	MSB;
	UINT32 	LSB;
	UINT32  NS;
}TimeStamp;

TimeStamp g_PTP_TXBuffer_TS;
TimeStamp gRTCOffset;
TimeStamp gRTCOffsetSav;
TimeStamp gSyncInterval[2];

TimeStamp g_NearEnd_PdelayReqEgressT1;
TimeStamp g_NearEnd_PdelayReqIngressT2;
TimeStamp g_NearEnd_PdelayRespEgressT3;
TimeStamp g_NearEnd_PdelayRespIngressT4;
TimeStamp g_FarEnd_PdelayReqEgressT1;
TimeStamp g_FarEnd_PdelayReqIngressT2;
TimeStamp g_FarEnd_PdelayRespEgressT3;
TimeStamp g_FarEnd_PdelayRespIngressT4;

TimeStamp g_SyncTs;
TimeStamp g_NearEnd_Sync_TS[2];
TimeStamp g_FarEnd_Sync_TS[2];

TimeStamp RTC_Time;
TimeStamp UTC_Time;
TimeStamp RX_PTP_TimeStamp;
TimeStamp g_FarEnd_PdelayReqReciptTime;
TimeStamp Syntonized_Time;

TimeStamp FarEndReqRespOffset;

TimeStamp g_InitialTime;

double NearEndRatioOffset;
double NearEndRatioOffsetComp;
double FarEndRatioOffset;

//RTC offset;
double Ratio =1;
double RatioWindow[8] = {1,1,1,1,1,1,1,1};
double RatioSav[2048];
double RatioMeanSav[2048];
double RatioMean =1;
double IncrValueDouble = (double)0x00800000;
UINT32 SecondsInNs = 0x3B9ACA00;
UINT32 IncrValue = 0x00800000;
UINT32 P2PDelay[2];
UINT32 P2PDelayOffset;
UINT32 RatioEn = 0;
UINT32 SyncCnt = 0;
UINT32 Uerr;
UINT32 DelayMeasureStateM = 5;
UINT32 MinsIndex;

UINT32 NanoSecondTimer0 = 0;
UINT32 NanoSecondTimer1 = 0;

UINT32 TxStateBusy = 0;
UINT16 g_PdealyReqSequenceID = 0;
UINT16 g_SyncSequenceID = 0;
UINT16 g_AnnounceSequenceID = 0;
UINT16 g_FollowUpSequenceID = 0;
UINT16 FarEndSequenceID = 0;

UINT64 UTC_TimeInNS;
UINT64 PTP_TimeInNS;

UINT32 StepMode = 2;
UINT32 SequenceID = 0;
UINT32 RequestingPortID[3];
UINT32 MannualDelay = 0xF000000; // 100000000

UINT32 g_TxBusy = 0;

UINT32 TimePlus = 0;
UINT64 TimeOffset;
// init table
UINT32 	logMessageIntervalInit = 0;
UINT8 	DstAddrInit[6] = {0x01,0x80,0xc2,0x00,0x00,0x0e};
UINT8 	SrcAddrInit[6] = {0x00,0x00,0x02,0x01,0x01,0x03};
UINT8	clockIdInit[8] = {0x00,0x00,0x22,0xfa,0xeb,0x01,0x01,0x03};
UINT8	portNumInit[2] = {0x00,0x05};
UINT8 	MacAddrInit[6] = {0x00,0x00,0x02,0x01,0x01,0x03};

typedef struct
{
	UINT64  TimeOffset;
	UINT32 	TimePlus;
}TimeOffsetS;

TimeOffsetS TimeOffsetSav[1024];


// 初始化时间同步控制器
void InitTimeSyncController() {
    g_TimeSyncCtrl.fast_threshold = -1000000000; // -1秒（纳秒）
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
    Write32(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);

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
    Write32(TSN_BASEADDR + RTC_IVCR_OFFSET, step_counter_value);

    // 记录调整历史
    UpdateAdjustmentHistory(freq_adjust_ratio * (offset > 0 ? 1 : -1));
}



// 更新调整历史
void UpdateAdjustmentHistory(double adjustment) {
    // 将调整值添加到历史记录中（循环缓冲区）
    int index = g_TimeSyncCtrl.adjustment_count % 100;
    g_TimeSyncCtrl.adjustment_history[index] = adjustment;
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




double CalculateRatioMean()
{
	UINT32 cnt = 0;
	double Sum = 0;
	double Mean = 0;
	for(cnt = 0; cnt<8; cnt++)
	{
		Sum += RatioWindow[cnt];
	}

	Mean = Sum/cnt;
	return Mean;

}
void GetOriginTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 48 + Num*4);
	}
	PTP_TimeStamp->MSB =    TS[0] & 0xffff;   //low 16 bit

	PTP_TimeStamp->LSB =   (((TS[0] >> 16) & 0xff) <<24)    \
						+ (((TS[0] >> 24) & 0xff) <<16)     \
						+ (((TS[1] >> 0 ) & 0xff) <<8)      \
						+ (((TS[1] >> 8) & 0xff) );

	PTP_TimeStamp->NS  =   (((TS[1] >> 16) & 0xff) <<24)    \
						+ (((TS[1] >> 24) & 0xff) <<16)     \
						+ (((TS[2] >> 0 ) & 0xff) <<8)      \
						+ (((TS[2] >> 8) & 0xff) );
}

void GetCorrectionField(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number + 20 + Num*4);
	}
	PTP_TimeStamp->NS =   (((TS[0] >> 16) & 0xff) <<56)      \
							+ (((TS[0] >> 24) & 0xff) <<48)  \
							+ (((TS[1] >> 0 ) & 0xff) <<40)  \
							+ (((TS[1] >> 8 ) & 0xff) <<32)  \
							+ (((TS[1] >> 16) & 0xff) <<24)  \
							+ (((TS[1] >> 24) & 0xff) <<16);
}

void GetRxPTPTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number * 0x100+ 0xf4 + Num*4);
	}
	PTP_TimeStamp->MSB =    0;//TS[0] & 0xffff;   //low 16 bit
	PTP_TimeStamp->LSB =  TS[1];
	PTP_TimeStamp->NS  =  TS[2];
}

void GetTxPTPTimeStamp(UINT32 PTP_Number,TimeStamp *PTP_TimeStamp)
{
	UINT32 TS[3];

	for(int Num = 0; Num <3; Num++)
	{
		TS[Num] = Read32(TX_PTP_BUFFER_BASEADDR + PTP_Number * 0x100+ 0xf4 + Num*4);
	}
	PTP_TimeStamp->MSB =    0;//TS[0] & 0xffff;   //low 16 bit
	PTP_TimeStamp->LSB =  TS[1];
	PTP_TimeStamp->NS  =  TS[2];
}


void CopyTimeStamp(TimeStamp *SrcTS,TimeStamp *DstTS)
{
	DstTS->MSB =  SrcTS->MSB;
	DstTS->LSB =  SrcTS->LSB;
	DstTS->NS  =  SrcTS->NS;
}


void CopyTimeStampToBuf(TimeStamp *SrcTS,TxBuffer *DstBuf)
{
	DstBuf->originTimestamp[0] = ((SrcTS->MSB >> 8  ) & 0xff);
	DstBuf->originTimestamp[1] = ((SrcTS->MSB >> 0  ) & 0xff);
	DstBuf->originTimestamp[2] = ((SrcTS->LSB >> 24 ) & 0xff);
	DstBuf->originTimestamp[3] = ((SrcTS->LSB >> 16 ) & 0xff);
	DstBuf->originTimestamp[4] = ((SrcTS->LSB >> 8  ) & 0xff);
	DstBuf->originTimestamp[5] = ((SrcTS->LSB >> 0  ) & 0xff);
	DstBuf->originTimestamp[6] = ((SrcTS->NS  >> 24 ) & 0xff);
	DstBuf->originTimestamp[7] = ((SrcTS->NS  >> 16 ) & 0xff);
	DstBuf->originTimestamp[8] = ((SrcTS->NS  >> 8  ) & 0xff);
	DstBuf->originTimestamp[9] = ((SrcTS->NS  >> 0  ) & 0xff);
}

void CopyReqPortIdToBuf(UINT32 RequestingPortID[],TxBuffer *DstBuf)
{
	DstBuf->reqPortId[0] = ((RequestingPortID[0] >> 16 ) & 0xff );
	DstBuf->reqPortId[1] = ((RequestingPortID[0] >> 24 ) & 0xff );
	DstBuf->reqPortId[2] = ((RequestingPortID[1] >> 0  ) & 0xff );
	DstBuf->reqPortId[3] = ((RequestingPortID[1] >> 8  ) & 0xff );
	DstBuf->reqPortId[4] = ((RequestingPortID[1] >> 16 ) & 0xff );
	DstBuf->reqPortId[5] = ((RequestingPortID[1] >> 24 ) & 0xff );
	DstBuf->reqPortId[6] = ((RequestingPortID[2] >> 0  ) & 0xff );
	DstBuf->reqPortId[7] = ((RequestingPortID[2] >> 8  ) & 0xff );
	DstBuf->reqPortId[8] = ((RequestingPortID[2] >> 16 ) & 0xff );
	DstBuf->reqPortId[9] = ((RequestingPortID[2] >> 24 ) & 0xff );
}

void  ProcessTXPTP(UINT32 PTP_Number)
{
	UINT32 MssgType = 0xff;

	GetTxPTPTimeStamp(PTP_Number ,&g_PTP_TXBuffer_TS);
	MssgType = Read32(TX_PTP_BUFFER_BASEADDR + PTP_Number * 0x100 +12 + 8); //message type
	MssgType = ((MssgType >> 16) & 0xf);
	switch(MssgType)
	{
 		case  SYNC:  // sync
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_SyncTs);
			FollowUpMsg(FOLLOWUP_BUFNUM , &TxFollowUpBuf);
		break;
		case  PDELAY_REQ:   //near end send Pdelay_Req
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_NearEnd_PdelayReqEgressT1);
		break;
		case  PDELAY_RESP:  // far end recive pdelay_resp
			CopyTimeStamp(&g_PTP_TXBuffer_TS,&g_FarEnd_PdelayRespEgressT3);
			PdelayRespFollowUpMsg(PDELAY_RESPFOLLOWUP_BUFNUM, &TxPdelayRespFollowUpBuf);
		break;
		case  PDELAY_RESPFOLLOWUP:  //  pdelay_resp follow up

		break;
		case  ANNOUNCE:  //

		break;
		case  FOLLOWUP:  //
			AnnounceMsg(ANNOUNCE_BUFNUM , &TxAnnounceBuf);
		break;



		default: break;
	}
}

UINT64 GetTimeStampInNS(TimeStamp *EventTimeStamp)
{
    // 将步长计数器值转换为实际纳秒值
    UINT64 nanoseconds = (UINT64)(EventTimeStamp->NS);

    // 组合秒和纳秒部分
    UINT64 seconds = (UINT64)(EventTimeStamp->LSB);
    UINT64 total_nanoseconds = (seconds * NS_PER_SECOND) + nanoseconds;
    return total_nanoseconds;

}
 void  ProcessRXPTP(UINT32 PTP_Number)
 {
	UINT32 cnt = 0;
	UINT32 MssgType = 0xff;

	MssgType = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number * 0x100+12); //message type
	MssgType = ((MssgType >> 16) & 0xf);
	switch(MssgType)
	{
 		case    SYNC:  // 0x0  sync
		 CopyTimeStamp(&(gSyncInterval[1]),&(gSyncInterval[0]));
		 GetRxPTPTimeStamp(PTP_Number,&(gSyncInterval[1]));
		 CopyTimeStamp(&(gSyncInterval[1]),&RX_PTP_TimeStamp);
                 RecvPTPMsg(PTP_Number, RxSyncBuf);

		 if(MinsIndex == 1){
			 NearEndRatioOffset =(gSyncInterval[1].LSB - gSyncInterval[0].LSB)*0x3B9ACA00 \
					 	 	 	+ gSyncInterval[1].NS - gSyncInterval[0].NS - NearEndRatioOffsetComp;
		 }
		 else{
			 NearEndRatioOffset =(gSyncInterval[1].LSB - gSyncInterval[0].LSB)*0x3B9ACA00 \
					 	 	    + gSyncInterval[1].NS - gSyncInterval[0].NS + NearEndRatioOffsetComp;
		 }
		 break;
        case    PDELAY_REQ:   // 0x2 Pdelay_Req
                        RecvPTPMsg(PTP_Number,    RxPdelayReqBuf);
			GetRxPTPTimeStamp(PTP_Number,&g_FarEnd_PdelayReqIngressT2);
			FarEndSequenceID = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number *0x100+ 44);
			FarEndSequenceID = (FarEndSequenceID )&0xffff;
			RequestingPortID[0] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number *0x100+ 32);
			RequestingPortID[1] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number *0x100+ 36);
			RequestingPortID[2] = Read32(RX_PTP_BUFFER_BASEADDR + PTP_Number *0x100+ 40);
			// send PdelayResp
			PdelayRespMsg(PDELAY_RESP_BUFNUM, &TxPdelayRespBuf);

		break;
        case  PDELAY_RESP:  //  0x3 Pdelay_Resp
                        RecvPTPMsg(PTP_Number,    RxPdelayRespBuf);
			// T2
			if(StepMode == 1)
			{
				GetCorrectionField(PTP_Number,&FarEndReqRespOffset);
			}
			else
			{
				GetOriginTimeStamp(PTP_Number,&g_NearEnd_PdelayReqIngressT2);
				GetRxPTPTimeStamp(PTP_Number,&g_NearEnd_PdelayRespIngressT4);
				cnt++;
			}

		break;
        case  FOLLOWUP:   //0x8 Follow_up   for 2 step mode
                        RecvPTPMsg(PTP_Number, RxFollowUpBuf);
			GetOriginTimeStamp(PTP_Number,&UTC_Time);
			//GetRxPTPTimeStamp(PTP_Number,&RX_PTP_TimeStamp);

			//nano seconds use ptp
			if(UTC_Time.LSB > RX_PTP_TimeStamp.LSB ){
				if(UTC_Time.NS >= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS+ ( UTC_Time.NS - RX_PTP_TimeStamp.NS )& 0x3fffffff;
				}
				else if(UTC_Time.NS < RX_PTP_TimeStamp.NS) {
					gRTCOffset.LSB = gRTCOffset.LSB+ UTC_Time.LSB - RX_PTP_TimeStamp.LSB -1;
					gRTCOffset.NS =  gRTCOffset.NS + (0x3B9ACA00  - RX_PTP_TimeStamp.NS + UTC_Time.NS ) & 0x3fffffff;
				}
			}
			else  if (UTC_Time.LSB = RX_PTP_TimeStamp.LSB ){
				if(UTC_Time.NS >= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS+ ( UTC_Time.NS - RX_PTP_TimeStamp.NS )& 0x3fffffff;
				}
				else
				{
					gRTCOffset.LSB = gRTCOffset.LSB+  UTC_Time.LSB - RX_PTP_TimeStamp.LSB;
					gRTCOffset.NS = gRTCOffset.NS - (RX_PTP_TimeStamp.NS - UTC_Time.NS)& 0x3fffffff;
				}
			}
			else { // LSB<
				if(UTC_Time.NS <= RX_PTP_TimeStamp.NS){
					gRTCOffset.LSB = gRTCOffset.LSB - ( RX_PTP_TimeStamp.LSB - UTC_Time.LSB );
					gRTCOffset.NS = gRTCOffset.NS- (RX_PTP_TimeStamp.NS -  UTC_Time.NS)& 0x3fffffff;
				}
				else{
					gRTCOffset.LSB = gRTCOffset.LSB - ( RX_PTP_TimeStamp.LSB - UTC_Time.LSB -1) ;
					gRTCOffset.NS = gRTCOffset.NS - (0x3B9ACA00  + RX_PTP_TimeStamp.NS - UTC_Time.NS)& 0x3fffffff;
				}
			}

			if(P2PDelay[1] >= P2PDelay[0])
			{
				P2PDelayOffset = P2PDelay[1] - P2PDelay[0];
				if(gRTCOffset.NS + P2PDelayOffset >= 0x3B9ACA00){
					gRTCOffset.LSB += 1;
					gRTCOffset.NS =gRTCOffset.NS + P2PDelayOffset - 0x3B9ACA00;
				}
				else
				{
					gRTCOffset.NS += P2PDelayOffset;
				}
			}
			else {
				P2PDelayOffset = P2PDelay[0] - P2PDelay[1];
				if(gRTCOffset.NS - P2PDelayOffset >= 0){
					gRTCOffset.NS =gRTCOffset.NS - P2PDelayOffset;
				}
				else
				{
					gRTCOffset.LSB -= 1;
					gRTCOffset.NS = gRTCOffset.NS + 0x3B9ACA00 - P2PDelayOffset;
				}
			}
			Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, 0);
			Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET, 0); // clear off set
			Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, gRTCOffset.LSB);
			Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET, gRTCOffset.NS); // Write Nano Seconds last
			UTC_TimeInNS = (UINT64)(UTC_Time.LSB & 0xffff) * 0x3B9ACA00 + (UINT64)(UTC_Time.NS);
			PTP_TimeInNS = (UINT64)(RX_PTP_TimeStamp.LSB & 0xffff) * 0x3B9ACA00 + (UINT64)(RX_PTP_TimeStamp.NS);
			if(UTC_TimeInNS >= PTP_TimeInNS){
				TimePlus = 1;
				TimeOffset = UTC_TimeInNS - PTP_TimeInNS;
			}
			else{
				TimePlus = 0;
				TimeOffset = PTP_TimeInNS - UTC_TimeInNS;
			}
			TimeOffsetSav[SyncCnt].TimePlus = TimePlus;
			TimeOffsetSav[SyncCnt].TimeOffset = TimeOffset;
			if(gRTCOffset.LSB > gRTCOffsetSav.LSB ){
				MinsIndex = 1;
				NearEndRatioOffsetComp = (gRTCOffset.LSB -  gRTCOffsetSav.LSB) * 0x3B9ACA00  + gRTCOffset.NS - gRTCOffsetSav.NS;
			}
			else  if (gRTCOffset.LSB = gRTCOffsetSav.LSB ){
				if(gRTCOffset.NS >= gRTCOffsetSav.NS){
					MinsIndex = 1;
					NearEndRatioOffsetComp =  gRTCOffset.NS - gRTCOffsetSav.NS;
				}
				else
				{
					MinsIndex = 0;
					NearEndRatioOffsetComp =  gRTCOffsetSav.NS - gRTCOffset.NS;
				}
			}
			else { // LSB<
				MinsIndex = 0;
				NearEndRatioOffsetComp = (gRTCOffsetSav.LSB -  gRTCOffset.LSB) * 0x3B9ACA00  + gRTCOffsetSav.NS - gRTCOffset.NS;
			}

			CopyTimeStamp(&gRTCOffset,&gRTCOffsetSav);
			if(DelayMeasureStateM == 5)
			{
				DelayMeasureStateM=0;
			}
			cnt++;
			CopyTimeStamp(&(g_FarEnd_Sync_TS[1]),&(g_FarEnd_Sync_TS[0]));
			CopyTimeStamp(&UTC_Time,&(g_FarEnd_Sync_TS[1]));
			SyncCnt++;
			FarEndRatioOffset =	(g_FarEnd_Sync_TS[1].LSB - g_FarEnd_Sync_TS[0].LSB)*0x3B9ACA00 \
							   + g_FarEnd_Sync_TS[1].NS  - g_FarEnd_Sync_TS[0].NS;
			Ratio = FarEndRatioOffset/NearEndRatioOffset;
			if((Ratio >=1.1  )|| (Ratio <0.9))
			{
				Ratio =1;
			}
			if(SyncCnt >=2){
				RatioEn =1;
			}
			if(RatioEn == 1){
				RatioWindow[SyncCnt & 0x3] = Ratio;
				RatioMean = CalculateRatioMean();
				RatioSav[SyncCnt] = Ratio;
				RatioMeanSav[SyncCnt] = RatioMean;

				IncrValueDouble = IncrValueDouble * Ratio;
				IncrValue = (UINT32) IncrValueDouble;
				Write32(TSN_BASEADDR+ RTC_IVCR_OFFSET, IncrValue);    //incr value ctrl reg
			}
			 PdelayReqMsg(PDELAY_REQ_BUFNUM, &TxPdelayReqBuf);

			cnt++;
		break;
        case  PDELAY_RESPFOLLOWUP:    // 0xa Pdelay_Resp_Follow_Up
                        RecvPTPMsg(PTP_Number, RxPdelayRespFollowUpBuf);
			GetOriginTimeStamp(PTP_Number,&g_NearEnd_PdelayRespEgressT3);
			//T3
			P2PDelay[0] = P2PDelay[1];
			P2PDelay[1] = (g_NearEnd_PdelayRespIngressT4.LSB - g_NearEnd_PdelayReqEgressT1.LSB) * 0x3B9ACA00 \
					 	 + g_NearEnd_PdelayRespIngressT4.NS  - g_NearEnd_PdelayReqEgressT1.NS	\
					     - ((g_NearEnd_PdelayRespEgressT3.LSB  - g_NearEnd_PdelayReqIngressT2.LSB) * 0x3B9ACA00 \
					     + g_NearEnd_PdelayRespEgressT3.NS   - g_NearEnd_PdelayReqIngressT2.NS);
			P2PDelay[1] = (P2PDelay[1] /2);	//  divide by 2(TS[0] >> 16)
			if(P2PDelay[1] >= 0x800)
			{
				P2PDelay[1] = 0;
			}
			cnt++;
		break;
        case  ANNOUNCE:  //0xb announce
                        RecvPTPMsg(PTP_Number, RxAnnounceBuf);

		break;
		default: break;
	 }
 }


 UINT32 DmaSend(UINT32 Len,UINT32 DmaAddr,UINT32 LocalAddr)
 {
	 //ta dma
	 UINT32 Status = 0;
	 UINT32 Rslt = 0;

	 Status = Read32(DmaAddr + MM2S_DMASR_OFFSET);
	 Rslt = Status &0x3;
	 if(Rslt !=0)
	 {
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x4);  //ctrl
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x0);  //ctrl reset
		 Write32(DmaAddr + MM2S_DMACR_OFFSET, 0x1);  //start
		 Write32(DmaAddr + MM2S_DA_OFFSET, LocalAddr);  // Source addr  Low 32bit
		 Write32(DmaAddr + MM2S_LENGTH, Len);  // write length to start
		 return 1;
	 }
	 return 0;
 }

 UINT32 DmaRecv(UINT32 Len,UINT32 DmaAddr,UINT32 LocalAddr)
 {
	 //ta dma
	 UINT32 Status = 0;
	 UINT32 Rslt = 0;

	 Status = Read32(DmaAddr + S2MM_DMASR_OFFSET);
	 Rslt = Status &0x3;
	 if(Rslt !=0)
	 {
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x4);  //ctrl
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x0);  //ctrl reset
		 Write32(DmaAddr + S2MM_DMACR_OFFSET, 0x1);  //start
		 Write32(DmaAddr + S2MM_DA_OFFSET, LocalAddr);  // Source addr  Low 32bit
		 Write32(DmaAddr + S2MM_LENGTH, Len);  // write length to start
		 return 1;
	 }
	 return 0;
 }

 UINT32 GetDmaTransRecvLen(UINT32 BaseAddr)
 {
	 UINT32 RecvLen = 0;

	 RecvLen = Read32(BaseAddr + S2MM_LENGTH);
	 return RecvLen;

 }

 void CfgMacAddr(UINT8  MacAddr[])
 {
     UINT32 RWC0 = 0;
     UINT32 RWC1 = 0;

     //RWC0 = Read32( TSN_BASEADDR + RCV_CFG_WORD_0_OFFSET);
     RWC0 =   (MacAddr[0] << 0)  \
            | (MacAddr[1] << 8)  \
            | (MacAddr[2] << 16) \
            | (MacAddr[3] << 24);

     Write32( TSN_BASEADDR + RCV_CFG_WORD_0_OFFSET,RWC0);

     RWC1 = Read32( TSN_BASEADDR + RCV_CFG_WORD_1_OFFSET);
     RWC1 =  	(MacAddr[4]	<< 0) 	\
    		 | 	(MacAddr[5] << 8)	\
			 | 	(RWC1 & 0xffff0000) ;

     Write32( TSN_BASEADDR + RCV_CFG_WORD_1_OFFSET,RWC1);
 }

 void cfgLocalTime()
 {
	g_InitialTime.LSB = 0x00006866393f;
	g_InitialTime.NS  = 0x00000f;
	Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, 0);
	Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,0); // clear off set
	Write32(TSN_BASEADDR + RTC_SECONDS_LSB_FIELD_OFFSET, g_InitialTime.LSB);
	Write32(TSN_BASEADDR + RTC_NANOSECONDS_FIELD_OFFSET,g_InitialTime.NS); // Write Nano Seconds last
 }
 void PdelayReqMsg( UINT32 PTP_Number , TxBuffer * TxPdelayReqBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxPdelayReqBuf->frmLen[0] = 0x44;  // 68bytes+4crc

	 TxPdelayReqBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 TxPdelayReqBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	TxPdelayReqBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		TxPdelayReqBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxPdelayReqBuf->ethType[0]     = 0x88;
	TxPdelayReqBuf->ethType[1]     = 0xf7;
	TxPdelayReqBuf->messageType 	 = 0x10 | PDELAY_REQ;
	TxPdelayReqBuf->versionPTP  	 = 0x12;
	TxPdelayReqBuf->messageLength[0] = 0x00;
	TxPdelayReqBuf->messageLength[1] = 0x44;
	TxPdelayReqBuf->domainNumber   = 0x00;
	TxPdelayReqBuf->minorSdoId     = 0x00;
	TxPdelayReqBuf->flags[0]       = 0x02;
	TxPdelayReqBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    TxPdelayReqBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    TxPdelayReqBuf->portNum[i] = portNumInit[i];
    }
    TxPdelayReqBuf->logMessageInterval = logMessageIntervalInit;
	TxPdelayReqBuf->sequenceID[0] = ((g_PdealyReqSequenceID >> 8) & 0xff);
	TxPdelayReqBuf->sequenceID[1] = ((g_PdealyReqSequenceID >> 0) & 0xff);
	g_PdealyReqSequenceID++;

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4,TxPdelayReqBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}


 void PdelayRespMsg(UINT32 PTP_Number , TxBuffer * TxPdelayRespBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;


     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxPdelayRespBuf->frmLen[0] = 0x68;  // 4c

	 TxPdelayRespBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 TxPdelayRespBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	TxPdelayRespBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		TxPdelayRespBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxPdelayRespBuf->ethType[0]     = 0x88;
	TxPdelayRespBuf->ethType[1]     = 0xf7;
	TxPdelayRespBuf->messageType 	 = 0x10 | PDELAY_RESP; // response :0x3
	TxPdelayRespBuf->versionPTP  	 = 0x12;
	TxPdelayRespBuf->messageLength[0] = 0x00;
	TxPdelayRespBuf->messageLength[1] = 54;
	TxPdelayRespBuf->domainNumber   = 0x00;
	TxPdelayRespBuf->minorSdoId     = 0x00;
	TxPdelayRespBuf->flags[0]       = 0x02;
	TxPdelayRespBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    TxPdelayRespBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    TxPdelayRespBuf->portNum[i] = portNumInit[i];
    }
    TxPdelayRespBuf->logMessageInterval = logMessageIntervalInit;
	TxPdelayRespBuf->sequenceID[0] = ((FarEndSequenceID >> 0) & 0xff);
	TxPdelayRespBuf->sequenceID[1] = ((FarEndSequenceID >> 8) & 0xff);

	CopyTimeStampToBuf(&g_FarEnd_PdelayReqIngressT2,TxPdelayRespBuf);

	CopyReqPortIdToBuf(RequestingPortID,TxPdelayRespBuf);

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4,TxPdelayRespBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

 void PdelayRespFollowUpMsg(UINT32 PTP_Number , TxBuffer * TxPdelayRespFollowUpBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

	 g_TxBusy = 1;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxPdelayRespFollowUpBuf->frmLen[0] = 68;  // 68bytes+4crc

	 TxPdelayRespFollowUpBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 TxPdelayRespFollowUpBuf->cmd[1]  = 0x00;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	TxPdelayRespFollowUpBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		TxPdelayRespFollowUpBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxPdelayRespFollowUpBuf->ethType[0]     = 0x88;
	TxPdelayRespFollowUpBuf->ethType[1]     = 0xf7;
	TxPdelayRespFollowUpBuf->messageType 	 = 0x10 | PDELAY_RESPFOLLOWUP; // response follow up  : 0xa
	TxPdelayRespFollowUpBuf->versionPTP  	 = 0x12;
	TxPdelayRespFollowUpBuf->messageLength[0] = 0x00;
	TxPdelayRespFollowUpBuf->messageLength[1] = 54;
	TxPdelayRespFollowUpBuf->domainNumber   = 0x00;
	TxPdelayRespFollowUpBuf->minorSdoId     = 0x00;
	TxPdelayRespFollowUpBuf->flags[0]       = 0x00;
	//TxPdelayRespFollowUpBuf->flags[0]       = 0x00 | TWO_STEP_FLAG ;
	TxPdelayRespFollowUpBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    TxPdelayRespFollowUpBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    TxPdelayRespFollowUpBuf->portNum[i] = portNumInit[i];
    }
    TxPdelayRespFollowUpBuf->logMessageInterval = logMessageIntervalInit;

    for(i=0;i<2;i++){
    	TxPdelayRespFollowUpBuf->sequenceID[i] = TxPdelayRespBuf.sequenceID[i];
    }
	//TxPdelayRespFollowUpBuf->sequenceID[1] = ((FarEndSequenceID >> 8) & 0xff);
	CopyTimeStampToBuf(&g_FarEnd_PdelayRespEgressT3,TxPdelayRespFollowUpBuf);

	for(i = 0;i<10; i++){
		TxPdelayRespFollowUpBuf->reqPortId[i] = TxPdelayRespBuf.reqPortId[i];
	}

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, TxPdelayRespFollowUpBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

void SyncMsg(UINT32 PTP_Number , TxBuffer * TxSyncBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxSyncBuf->frmLen[0] = 0x44;  // 68bytes+4crc

	 TxSyncBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 TxSyncBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	TxSyncBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		TxSyncBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxSyncBuf->ethType[0]     = 0x88;
	TxSyncBuf->ethType[1]     = 0xf7;
	TxSyncBuf->messageType 	 = 0x10 | SYNC;
	TxSyncBuf->versionPTP  	 = 0x12;
	TxSyncBuf->messageLength[0] = 0x00;
	TxSyncBuf->messageLength[1] = 0x36;
	TxSyncBuf->domainNumber   = 0x00;
	TxSyncBuf->minorSdoId     = 0x00;
	TxSyncBuf->flags[0]       = 0x02;
	TxSyncBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    TxSyncBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    TxSyncBuf->portNum[i] = portNumInit[i];
    }
    TxSyncBuf->logMessageInterval = logMessageIntervalInit;
	TxSyncBuf->sequenceID[0] = ((g_SyncSequenceID >> 8) & 0xff);
	TxSyncBuf->sequenceID[1] = ((g_SyncSequenceID >> 0) & 0xff);
	g_FollowUpSequenceID  = g_SyncSequenceID;
    g_SyncSequenceID++;

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, TxSyncBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}


void FollowUpMsg(UINT32 PTP_Number , TxBuffer * TxFollowUpBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxFollowUpBuf->frmLen[0] = 0x6c;  // 68bytes+4crc

	 TxFollowUpBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
	 TxFollowUpBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
	 	TxFollowUpBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		TxFollowUpBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxFollowUpBuf->ethType[0]     = 0x88;
	TxFollowUpBuf->ethType[1]     = 0xf7;
	TxFollowUpBuf->messageType 	 = 0x10 | FOLLOWUP;
	TxFollowUpBuf->versionPTP  	 = 0x12;
	TxFollowUpBuf->messageLength[0] = 0x00;
	TxFollowUpBuf->messageLength[1] = 0x6c;//0x44;
	TxFollowUpBuf->domainNumber   = 0x00;
	TxFollowUpBuf->minorSdoId     = 0x00;
	TxFollowUpBuf->flags[0]       = 0x02;
	TxFollowUpBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
	    TxFollowUpBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
	    TxFollowUpBuf->portNum[i] = portNumInit[i];
    }
    TxFollowUpBuf->logMessageInterval = logMessageIntervalInit;
	TxFollowUpBuf->sequenceID[0] = ((g_FollowUpSequenceID >> 8) & 0xff);
	TxFollowUpBuf->sequenceID[1] = ((g_FollowUpSequenceID >> 0) & 0xff);

	CopyTimeStampToBuf(&g_SyncTs,TxFollowUpBuf);

	for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, TxFollowUpBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

void AnnounceMsg(UINT32 PTP_Number , TxBuffer * TxAnnounceBuf)
{
	 UINT32 i = 0;
	 UINT32 TxPtpAddr = 0;

     TxPtpAddr = TX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
     TxAnnounceBuf->frmLen[0] = 0x5a;  // 68bytes+4crc

     TxAnnounceBuf->cmd[0]  = 0x00;   //0:2step;1 :1step
     TxAnnounceBuf->cmd[1]  = 0x01;   // 1:update check sum

	 for(i = 0; i<6; i++){
		 TxAnnounceBuf->dstAddr[i] = DstAddrInit[i];
     }

	 for(i = 0; i<6; i++){
		 TxAnnounceBuf->srcAddr[i] = SrcAddrInit[i];
	}
	TxAnnounceBuf->ethType[0]     = 0x88;
	TxAnnounceBuf->ethType[1]     = 0xf7;
	TxAnnounceBuf->messageType 	 = 0x10 | ANNOUNCE;
	TxAnnounceBuf->versionPTP  	 = 0x12;
	TxAnnounceBuf->messageLength[0] = 0x00;
	TxAnnounceBuf->messageLength[1] = 0x4c;
	TxAnnounceBuf->domainNumber   = 0x00;
	TxAnnounceBuf->minorSdoId     = 0x00;
	TxAnnounceBuf->flags[0]       = 0x0c;
	TxAnnounceBuf->flags[1]       = 0x00;
	for(i = 0; i<8; i++){
		TxAnnounceBuf->clockId[i] = clockIdInit[i];
    }
    for(i = 0; i<2; i++){
    	TxAnnounceBuf->portNum[i] = portNumInit[i];
    }
    TxAnnounceBuf->logMessageInterval = logMessageIntervalInit;
    TxAnnounceBuf->sequenceID[0] = ((g_AnnounceSequenceID >> 8) & 0xff);
    TxAnnounceBuf->sequenceID[1] = ((g_AnnounceSequenceID >> 0) & 0xff);
	g_AnnounceSequenceID++;

	TxAnnounceBuf->currentUtcOffset[1] 	= 37;
	TxAnnounceBuf->grandmasterPriority1 	= 128;
	TxAnnounceBuf->grandmasterClockclass 	= 248;
	TxAnnounceBuf->grandmasterClockAccuracy = 0x23;
	TxAnnounceBuf->grandmasterClockVariance[0] = 0x43;
	TxAnnounceBuf->grandmasterClockVariance[1] = 0x6a;
	TxAnnounceBuf->grandmasterPriority2 = 128;

	for(i = 0; i<8; i++){
		TxAnnounceBuf->grandmasterIdentity[i] = clockIdInit[i];
    }
	TxAnnounceBuf->stepsRemoved[0] = 0;
	TxAnnounceBuf->stepsRemoved[1] = 0;
	TxAnnounceBuf->timeSource = 0xa0;  //internal osciliator
	TxAnnounceBuf->tlvType[1] = 0x8;
	TxAnnounceBuf->lengthFiled[1] = 0x8;
	for(i = 0; i<8; i++){
		TxAnnounceBuf->pathSequence[0][i] = clockIdInit[i];
    }

    for(i = 0;i<64;i++){
        Write32(TxPtpAddr + i*4, TxAnnounceBuf->buf[i]);
    }
	Write32(TX_PTP_BUFFER_CTRL, 0x1 << PTP_Number);
}

void RecvPTPMsg(UINT32 PTP_Number , RxBuffer * PTPRcvBuf)
{	
    UINT32 RxPtpAddr = 0;

    RxPtpAddr = RX_PTP_BUFFER_BASEADDR + PTP_Number*0x100;
    for(i = 0;i<64;i++){
        Read32(RxPtpAddr + i*4,  PTPRcvBuf->buf[i]);
    }
}

 int main()
 {
	// init_platform();

	 print("Hello World\n\r");
	 UINT32 PacketCnt = 0;
	 UINT32 cnt = 0;
	 UINT32 PtpNum = 0;
	 UINT32 RxPktNum = 0;
	 UINT32 Status = 0;
	 UINT32  PTP_Number = 0;
	 UINT32 PTP_Offset = 0;
	 UINT32 TxEn = 1;
	 UINT32 RxLen = 0;
	 UINT32 rtc,rtcSav;

	cleanup_platform();
	PackPayloadArpReqest(0x200,0x1234);
	PackPayloadArpReqest(0x00,0x8357);
	// Write32(TSN_BASEADDR + 0x444, 0x901); //preemption control
	// Write32(TSN_BASEADDR + 0x440, 0x1);    //preemption en
	Write32(TSN_BASEADDR + RTC_IVCR_OFFSET, INCRVALUE);    //incr value ctrl reg
	CfgMacAddr(MacAddrInit);
	cfgLocalTime();
//	while(1){
//		//CfgMacAddr(MacAddrInit);
//		DmaRecv(2048,DMA_BASEADDR_1,BRAM_BASEADDR);
//		Status = Read32(DMA_BASEADDR_1+S2MM_DMASR_OFFSET);
//		Status = Status &0x3;
//		if(Status !=0){
//			RxLen = GetDmaTransRecvLen(DMA_BASEADDR_1);
//		}
//
//
//		DmaRecv(2048,DMA_BASEADDR_0,BRAM_BASEADDR+2048);
//		Status = Read32(DMA_BASEADDR_0+S2MM_DMASR_OFFSET);
//		Status = Status &0x3;
//		if(Status !=0){
//			RxLen = GetDmaTransRecvLen(DMA_BASEADDR_0);
//		}
//	}

	while(1){
		//PDELAY_RESP_BUFNUM, &TxPdelayRespBuf

		rtc =Read32(TSN_BASEADDR + 0x1282c);
		rtc =Read32(TSN_BASEADDR + 0x12830);

		if((rtc != rtcSav) && (g_TxBusy == 0))
		{
			rtcSav = rtc;
			SyncMsg(SYNC_BUFNUM , &TxSyncBuf);
		}
		Status = Read32(IRQ_REG_BASEADDR + 0x4);   //ptp tx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + TX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Number = (PtpNum >>16); //mod 0x100
			ProcessTXPTP( PTP_Number);
			g_TxBusy = 0;

		}

		Status = Read32(IRQ_REG_BASEADDR + 0x0);   //ptp rx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + RX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Number = (PtpNum >>8) ; //mod 0x100
			ProcessRXPTP(PTP_Number);
			g_TxBusy = 1;
			// PdelayReqMsg Pdelay_Req(0x0);
		}
	}

	while(1){
		Status = Read32(IRQ_REG_BASEADDR + 0x0);   //ptp rx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + RX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Number = (PtpNum >>8) ; //mod 0x100
			ProcessRXPTP(PTP_Number);
			g_TxBusy = 1;
			// PdelayReqMsg Pdelay_Req(0x0);
		}


		Status = Read32(IRQ_REG_BASEADDR + 0x4);   //ptp tx interrupt status
		if(Status !=0)
		{
			PtpNum = Read32(TSN_BASEADDR + TX_PTP_CTRL_OFFSET); //rx ptp packet buffer control register
			PTP_Number = (PtpNum >>16) ; //mod 0x100
			ProcessTXPTP(PTP_Number);
			g_TxBusy = 0;
		//cnt = DmaTrans(64,DMA_BASEADDR_0);
		PacketCnt +=cnt;
		//DmaTrans(64,DMA_BASEADDR_1);
		}
		if(g_TxBusy == 0){
			//DmaSend(64,DMA_BASEADDR_1,BRAM_BASEADDR);
		}
	}
	while(1);
	print("Successfully ran Hello World application");

	return 0;
 }



 void PackPayloadArpReqest(UINT32 Offset,UINT32 Addr)
  {

 	 UINT32 i = 0;
 	 UINT32 cnt = 0;
 	 i = Offset;
  //	Write32(BRAM_BASEADDR +(i++), 0x57ffffff); //
  //	Write32(BRAM_BASEADDR +(i++), 0x06088357); //  type 0806 req   0835 reply
  //	Write32(BRAM_BASEADDR +(i++), 0x05040302); //
  //	Write32(BRAM_BASEADDR +(i++), 0x04800081); //
  //	Write32(BRAM_BASEADDR +(i++), 0x28292a2a); //
  //	Write32(BRAM_BASEADDR +(i++), 0x83578357); //
  //	for(cnt=0;cnt<4096;cnt++)
  //	{
  //		Write32(BRAM_BASEADDR +(i+cnt), cnt); //
  //	}


 	 Write8(BRAM_BASEADDR +(i++), 0xff); //DST MAC
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //
 	 Write8(BRAM_BASEADDR +(i++), 0xff); //

 	 Write8(BRAM_BASEADDR +(i++), 0x00); //SRC MAC
 	 Write8(BRAM_BASEADDR +(i++), 0x0C); //
 	 Write8(BRAM_BASEADDR +(i++), 0x29); //
 	 Write8(BRAM_BASEADDR +(i++), 0x70); //
 	 Write8(BRAM_BASEADDR +(i++), ((Addr>>8)& 0xff)); //
 	 Write8(BRAM_BASEADDR +(i++), (Addr& 0xff)); //


 	 Write8(BRAM_BASEADDR +(i++), 0x08); //
 	 Write8(BRAM_BASEADDR +(i++), 0x06); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x01); //
 	 Write8(BRAM_BASEADDR +(i++), 0x08); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x06); //
 	 Write8(BRAM_BASEADDR +(i++), 0x04); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x01); //

 	 Write8(BRAM_BASEADDR +(i++), 0x00); //SRC MAC
 	 Write8(BRAM_BASEADDR +(i++), 0x0C); //
 	 Write8(BRAM_BASEADDR +(i++), 0x29); //
 	 Write8(BRAM_BASEADDR +(i++), 0x70); //
 	 Write8(BRAM_BASEADDR +(i++), 0x2E); //
 	 Write8(BRAM_BASEADDR +(i++), 0x59); //

 	 Write8(BRAM_BASEADDR +(i++), 0xc0); //sender ip  192
 	 Write8(BRAM_BASEADDR +(i++), 0xa8); // 168
 	 Write8(BRAM_BASEADDR +(i++), 0x00); // 0
 	 Write8(BRAM_BASEADDR +(i++), 0xf9); // 249


 	 Write8(BRAM_BASEADDR +(i++), 0x00); //target mac
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //


 	 Write8(BRAM_BASEADDR +(i++), 0xc0); //target ip 192
 	 Write8(BRAM_BASEADDR +(i++), 0xa8); //168
 	 Write8(BRAM_BASEADDR +(i++), 0x00); //0
 	 Write8(BRAM_BASEADDR +(i++), 0xb4); //180


 	 for(cnt=0;cnt<128;cnt++)
 	 {
 		 Write8(BRAM_BASEADDR +(i+cnt),0); //
 	 }
  }

