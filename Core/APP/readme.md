@Core 现在阐述我们的目的，我在Gimbal_Control定义了新的target_small和target_big的格式    //  float target_big = Virtual_Yaw_GetTarget_Big() + Chassis_GetTarget_Big;
同时，我们需要观察会不会影响virtual_postion的执行
    //  我目前的代码逻辑可能会存在着问题，记得审查会不会影响smallyaw的逻辑

我们的任务是什么，是解决机甲大师哨兵机器人的bigyaw的底盘跟随功能，我希望它的接口能如virtual_postion那样仅调出接口给target_small和target_big
,在次之前我们已经生成了一份Chassis_Follow文件在Core/old 文件夹下面，它可能不能完全满足我的工作，但它的格式是相对像virtual_postion的.
同时相对于
    //---------- 1. 获取实际编码（给虚拟层）----------
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;


你能获取的值只有freertos的任务

void StartCalTask(void *argument)
{
    /* USER CODE BEGIN StartCalTask */
    osDelay(100);
    
    int n = 0;
    /* Infinite loop */
    for (;;)
    {
        float yaw = Can_BMI088_Data.Yaw;
        CalTask_Yaw_Update(yaw);
        float delta = CalTask_Yaw_GetDelta();
        n++;
        if (n > 20)
        {
            printf("delta = %d\n", (int)(delta*100));
            n = 0;
        }
        osDelay(20);
    }
    /* USER CODE END StartCalTask */
}
中的delta，也请以delta为准，//  yaw轴的定义是-180度到180度
也请阅读CalTask_Yaw了解更多特性
现在请完成新的底盘跟随补偿的文件神生成
