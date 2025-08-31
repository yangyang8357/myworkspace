// ִ��ʱ��ͬ������
void PerformTimeSync(double measured_offset) {
    // ʹ�ÿ������˲����������ֵ
    double filtered_offset = UpdateKalmanFilter(&g_TimeSyncCtrl.kf, measured_offset);
    
    g_TimeSyncCtrl.time_offset = filtered_offset;
    g_TimeSyncCtrl.adjustment_count++;
    
    // ����ͬ�����ȹ���
    g_TimeSyncCtrl.sync_accuracy = fabs(filtered_offset);
    
    // ״̬ת���߼�
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
}
