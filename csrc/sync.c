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

// 全局变量声明
UINT32 TSN_BASEADDR;             // TSN控制器基地址
TimeSyncController g_TimeSyncCtrl;

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

// 获取当前同步状态
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
    xil_printf("===================\r\n");
}
