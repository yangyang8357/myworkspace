// 执行时间同步调整
void PerformTimeSync(double measured_offset) {
    // 使用卡尔曼滤波器处理测量值
    double filtered_offset = UpdateKalmanFilter(&g_TimeSyncCtrl.kf, measured_offset);
    
    g_TimeSyncCtrl.time_offset = filtered_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // 更新同步精度估计
    g_TimeSyncCtrl.sync_accuracy = fabs(filtered_offset);
    
    // 状态转换逻辑
    if (g_TimeSyncCtrl.state == INITIALIZING) {
        g_TimeSyncCtrl.state = SYNCHRONIZING;
    } else if (g_TimeSyncCtrl.state == SYNCHRONIZED) {
        if (fabs(filtered_offset) > g_TimeSyncCtrl.slow_threshold * 2) {
            g_TimeSyncCtrl.state = LOSING_SYNC;
            g_TimeSyncCtrl.sync_failure_count++;
        }
    } else if (g_TimeSyncCtrl.state == LOSING_SYNC) {
        if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold) {
            g_TimeSyncCtrl.state = SYNCHRONIZED;
            g_TimeSyncCtrl.sync_failure_count = 0;
        } else {
            g_TimeSyncCtrl.sync_failure_count++;
            if (g_TimeSyncCtrl.sync_failure_count > 5) {
                g_TimeSyncCtrl.state = RECOVERING;
            }
        }
    } else if (g_TimeSyncCtrl.state == RECOVERING) {
        if (fabs(filtered_offset) < g_TimeSyncCtrl.slow_threshold) {
            g_TimeSyncCtrl.state = SYNCHRONIZED;
            g_TimeSyncCtrl.sync_failure_count = 0;
        }
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
}
