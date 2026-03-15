这就是每次甩头后返回数据，这是编码真实值 请分析为什么 我已经撤回你的修改 不不不 让咱们继续沟通，在我们把机械挪到中值位置的时候应该是坐标（0，0），但当我们的位置不是初始坐标的时候，先从正数计算，我们应先读取一下当前smallyaw编码值并与机械中值对比，得出新的虚拟坐标值，并把该虚拟坐标值作为新的启动位置，进而不再摆头 而现在它还是会转向一个莫名其妙一直以来的位置

真的是如此吗，为什么开机会瞎转到一个特定的角度

咱们的路线不应该是在虚拟坐标初始化后，把参数 PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f);切换成 PID_PositionStructureInit(&SmallYaw_PositionPID, x);吗 而且 //---------- 虚拟坐标层初始化 ---------- // 传入当前编码，计算初始虚拟坐标（开机位置对应虚拟值，不转头） float current_small = (float)Can2_M6020_MotorStatus[1].Angle; float current_big = (float)Can2_M6020_MotorStatus[0].Angle; Virtual_Yaw_Init(current_small, current_big); 应该先于 //---------- Pitch初始化 ---------- PID_PositionStructureInit(&Pitch_PositionPID, 4074.0f); PID_PositionSetParameter(&Pitch_PositionPID, 0.5f, 0.0f, 0.0f); PID_PositionSetOUTRange(&Pitch_PositionPID, -400.0f, 400.0f); PID_PositionSetEkRange(&Pitch_PositionPID, -3.0f, 3.0f);

PID_PositionStructureInit(&Pitch_SpeedPID, 0.0f); PID_PositionSetParameter(&Pitch_SpeedPID, 50.0f, 0.0f, 0.0f); PID_PositionSetOUTRange(&Pitch_SpeedPID, -20000.0f, 20000.0f); PID_PositionSetEkRange(&Pitch_SpeedPID, -3.0f, 3.0f);

//---------- SmallYaw初始化 ---------- PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f); PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f); PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f); PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);

PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f); PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f); PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f); PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);

//---------- BigYaw初始化 ---------- PID_PositionStructureInit(&BigYaw_PositionPID, 0.0f); PID_PositionSetParameter(&BigYaw_PositionPID, 0.8f, 0.0f, 0.0f); PID_PositionSetOUTRange(&BigYaw_PositionPID, -6000.0f, 6000.0f); PID_PositionSetEkRange(&BigYaw_PositionPID, -5.0f, 5.0f);

PID_PositionStructureInit(&BigYaw_SpeedPID, 0.0f); PID_PositionSetParameter(&BigYaw_SpeedPID, 30.0f, 0.0f, 0.0f); PID_PositionSetOUTRange(&BigYaw_SpeedPID, -20000.0f, 20000.0f); PID_PositionSetEkRange(&BigYaw_SpeedPID