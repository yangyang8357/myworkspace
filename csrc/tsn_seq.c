#include "xil_io.h"
#include "xparameters.h"
#include "xil_types.h"

// ============== 硬件寄存器定义 ==============
#define TSN_BASEADDR         XPAR_TSN_0_BASEADDR
#define RTC_IVCR_OFFSET      0x12810
#define RTC_SECONDS_OFFSET   0x12808
#define RTC_NANOSEC_OFFSET   0x12800
#define SYNC_RX_CTRL_OFFSET  0x12004
#define TX_PTP_CTRL_OFFSET   0x12000
#define RX_PTP_CTRL_OFFSET   0x12004

// ============== PTP 协议参数 ==============
#define JUMP_THRESHOLD_NS    1000000000LL  // 1秒跳变阈值
#define SYNC_SEQID_TIMEOUT   3             // SequenceID超时计数
#define MAX_SEQID            0xFFFF        // SequenceID最大值
#define INCRVALUE            0x00800000    // 默认时钟增量值
#define SYNC                 0x0
#define FOLLOWUP             0x8
#define PDELAY_REQ           0x2
#define PDELAY_RESP          0x3

// ============== 数据结构 ==============
typedef struct {
    u32 seconds;
    u32 nanoseconds;
} Timestamp;

typedef struct {
    u16 last_sync_seqid;      // 最近接收的Sync序列号
    u16 expected_followup;    // 期待的FollowUp序列号
    u16 next_sync_seqid;      // 下一个发送的Sync序列号
    u16 next_followup_seqid;  // 下一个发送的FollowUp序列号
    u32 seqid_timeout_cnt;    // 序列号超时计数器
} SeqIDManager;

typedef struct {
    u32 incr_value;           // 当前时钟增量值
    u8  sync_state;           // 同步状态
    Timestamp last_adjust;    // 最后调整时间
    Timestamp last_sync;      // 最后一次同步时间
} ClockController;

// ============== 全局变量 ==============
static SeqIDManager seq_mgr;
static ClockController clock_ctrl;

// ============== 硬件访问封装 ==============
static u32 ReadReg(u32 offset) {
    return Xil_In32(TSN_BASEADDR + offset);
}

static void WriteReg(u32 offset, u32 value) {
    Xil_Out32(TSN_BASEADDR + offset, value);
}

// ============== 时间操作 ==============
static Timestamp GetCurrentTime() {
    Timestamp ts;
    ts.seconds = ReadReg(RTC_SECONDS_OFFSET);
    ts.nanoseconds = ReadReg(RTC_NANOSEC_OFFSET);
    return ts;
}

static void SetTimeDirectly(Timestamp t) {
    WriteReg(RTC_SECONDS_OFFSET, t.seconds);
    WriteReg(RTC_NANOSEC_OFFSET, t.nanoseconds); // 注意：必须先写秒再写纳秒
}

static int64_t CalculateTimeDiff(Timestamp master, Timestamp slave) {
    return ((int64_t)master.seconds - (int64_t)slave.seconds) * 1000000000LL + 
           ((int64_t)master.nanoseconds - (int64_t)slave.nanoseconds);
}

// ============== SequenceID 管理 ==============
static bool ValidateSeqID(u16 received_id, u16 expected_id) {
    // 允许序列号循环递增
    if (expected_id == MAX_SEQID && received_id == 0) 
        return true;
    return received_id == expected_id;
}

static void UpdateSeqIDState(u16 new_sync_id) {
    if (!ValidateSeqID(new_sync_id, seq_mgr.last_sync_seqid + 1)) {
        seq_mgr.seqid_timeout_cnt++;
        if (seq_mgr.seqid_timeout_cnt > SYNC_SEQID_TIMEOUT) {
            // 序列号严重不同步，触发时钟重置
            clock_ctrl.sync_state = 0xFF; // 错误状态
        }
        return;
    }
    
    seq_mgr.last_sync_seqid = new_sync_id;
    seq_mgr.expected_followup = new_sync_id;
    seq_mgr.seqid_timeout_cnt = 0;
}

// ============== 时钟调整策略 ==============
static void AdjustClockFrequency(double ratio) {
    // 限制调整幅度在±0.1%范围内
    ratio = (ratio > 1.001) ? 1.001 : (ratio < 0.999) ? 0.999 : ratio;
    
    clock_ctrl.incr_value = (u32)(INCRVALUE * ratio);
    WriteReg(RTC_IVCR_OFFSET, clock_ctrl.incr_value);
}

static void SmoothAdjustment(int64_t offset_ns) {
    // 从机快于主机时只减速（ratio < 1.0）
    double ratio = (offset_ns > 0) ? 
        1.0 + (offset_ns / 1e10) :  // 从机慢，加速
        1.0 + (offset_ns / 1e11);   // 从机快，减速（幅度更小）
    
    AdjustClockFrequency(ratio);
}

static void JumpAdjustmentIfNeeded(Timestamp master, Timestamp slave) {
    int64_t offset = CalculateTimeDiff(master, slave);
    
    if (offset > JUMP_THRESHOLD_NS) {
        SetTimeDirectly(master); // 从机慢超过1秒：直接跳变
    } else {
        SmoothAdjustment(offset); // 否则平滑调整
    }
}

// ============== PTP消息处理 ==============
static void ProcessSyncMessage(u16 seq_id, Timestamp rx_time, Timestamp master_time) {
    // 1. 检查SequenceID连续性
    UpdateSeqIDState(seq_id);
    if (clock_ctrl.sync_state == 0xFF) {
        return; // 同步错误状态
    }
    
    // 2. 获取当前从机时间
    Timestamp slave_time = GetCurrentTime();
    
    // 3. 应用非对称调整策略
    if (slave_time.seconds > master_time.seconds || 
       (slave_time.seconds == master_time.seconds && 
        slave_time.nanoseconds > master_time.nanoseconds)) {
        // 从机快于主机：仅平滑减速
        SmoothAdjustment(CalculateTimeDiff(master_time, slave_time));
    } else {
        // 从机慢于主机：可能跳变或加速
        JumpAdjustmentIfNeeded(master_time, slave_time);
    }
    
    // 4. 更新最后调整时间
    clock_ctrl.last_adjust = GetCurrentTime();
    clock_ctrl.last_sync = master_time;
}

static void ProcessFollowUpMessage(u16 seq_id, Timestamp master_time) {
    // 验证FollowUp的SequenceID是否匹配最近的Sync
    if (seq_id != seq_mgr.expected_followup) {
        return;
    }
    
    // 更新时间戳
    clock_ctrl.last_sync = master_time;
}

// ============== 主处理循环 ==============
void TSN_PTPHandler() {
    // 检查RX中断状态
    u32 rx_status = ReadReg(IRQ_REG_BASEADDR);
    if (rx_status & 0x1) { // RX中断
        u32 ptp_num = ReadReg(RX_PTP_CTRL_OFFSET);
        u16 seq_id = (ptp_num >> 16) & 0xFFFF;
        u8 msg_type = (ptp_num >> 8) & 0xF;
        
        Timestamp rx_time, master_time;
        rx_time.seconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0xF8);
        rx_time.nanoseconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0xFC);
        
        if (msg_type == SYNC) {
            master_time.seconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0x30);
            master_time.nanoseconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0x34);
            ProcessSyncMessage(seq_id, rx_time, master_time);
        }
        else if (msg_type == FOLLOWUP) {
            master_time.seconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0x30);
            master_time.nanoseconds = ReadReg(RX_PTP_BUFFER_BASEADDR + 0x34);
            ProcessFollowUpMessage(seq_id, master_time);
        }
        
        // 清除中断
        WriteReg(IRQ_REG_BASEADDR, rx_status & ~0x1);
    }
    
    // 检查TX中断状态
    u32 tx_status = ReadReg(IRQ_REG_BASEADDR + 0x4);
    if (tx_status & 0x1) { // TX中断
        // 处理TX完成
        // 清除中断
        WriteReg(IRQ_REG_BASEADDR + 0x4, tx_status & ~0x1);
    }
}

// ============== 初始化函数 ==============
void TSN_TimeSyncInit() {
    // 初始化SequenceID管理
    seq_mgr.last_sync_seqid = 0;
    seq_mgr.expected_followup = 0;
    seq_mgr.next_sync_seqid = 1;
    seq_mgr.next_followup_seqid = 1;
    seq_mgr.seqid_timeout_cnt = 0;
    
    // 初始化时钟控制器
    clock_ctrl.incr_value = INCRVALUE;
    clock_ctrl.sync_state = 0;
    WriteReg(RTC_IVCR_OFFSET, INCRVALUE);
    
    // 初始化时间
    Timestamp init_time = {0, 0};
    SetTimeDirectly(init_time);
}

// ============== 主函数 ==============
int main() {
    // 初始化硬件平台
    init_platform();
    
    // 初始化TSN时间同步
    TSN_TimeSyncInit();
    
    // 主循环
    while(1) {
        TSN_PTPHandler();
    }
    
    // 清理
    cleanup_platform();
    return 0;
}