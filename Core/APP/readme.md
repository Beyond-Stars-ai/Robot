@Core 就物理模型而言计划有变化，现在我们需要重新审核yaw轴代码，
首先第一步，让我们回到@Core/Src freertos.c
我已经粗略地定义了
int16_t origin_BigYaw_count = 6537;
int16_t origin_SmallYaw_count = 2233;

int16_t now_BigYaw_count = 0;
int16_t now_SmallYaw_count = 0;

int16_t error_BigYaw_count = 0;
int16_t error_SmallYaw_count = 0;
分别为机械中值编码值，开机启动的编码值，和编码差值
接下来 请阅读 @Core/APP 请重新阅读 Gimbal_Yaw_Big./Gimbal_Yaw_Small./Gimbal_PoseCalc./Gimbal_Pitch./

然后回到咱们定义的new_Gimbal_Yaw
我预计整合yaw和pitch到一个文件中，意味着
Gimbal_Pitch
也会被添加，请重新起名
此外new_Gimbal_Yaw存在若干点瑕疵和机械上的考虑不足
1extern M6020_Motor Can2_M6020_MotorStatus[7];     // GM6020电机状态数组 - Can2定义是否正确，我们使用的要严格遵循Gimbal_Yaw_Big/Gimbal_Yaw_Small的配置，应该是can1吧
typedef enum {
    SMALLYAW_MODE_ENCODER = 0,      // 编码器模式：直接设置目标编码器值
    SMALLYAW_MODE_REMOTE,           // 遥控器模式：遥控器增量控制
    SMALLYAW_MODE_GYRO,             // 陀螺仪模式：预留
} SmallYaw_Mode_e
要么不设立
要么取消编码器模式：直接设置目标编码器值，我们没有这个模式，SMALLYAW_MODE_GYRO,             // 陀螺仪模式：预留也取消，我们底盘补偿了，所以typedef enum {
    SMALLYAW_MODE_ENCODER = 0,      // 编码器模式：直接设置目标编码器值
    SMALLYAW_MODE_REMOTE,           // 遥控器模式：遥控器增量控制
    SMALLYAW_MODE_GYRO,             // 陀螺仪模式：预留
} SmallYaw_Mode_e;没有意义了，只有遥控器模式


现在是机械上的疏忽，以我们的原始机械中值为例子，以俯视角（上视角为例子），bigyaw是顺时针编码增加，smallyaw是逆时针编码增加，以原始机械中值，也就是原始位置为例子，就是smallyaw逆时针向左边为正方向，bigyaw顺时针向右为正方向
此外还记得之前吗，smallyaw编码只根据遥控器转动，bigyaw编码只随小yaw轴编码转动而转动
所以我们需要根据它们的机械向性，定义上位端的虚拟编码值0，0
然后再用这个虚拟编码值0，0，来计算实际编码值，也就是我们之前定义的now_BigYaw_count = 0;
记得保留当前值和目标值的设置


好了我们只是初步达到我们的目标， 但有两个事实我们要意识到， 首先是在开机后，我们的小车会瞬间猛打头回归云台的机械中值，这不是我们所期望的， 其次我们的大yaw轴明显跟不上小yaw轴的速度，为什么，虽然确定编码了，但转动角度并不和谐，这让我想起了遥远的代码 // if(angle > smallyaw_x - 74 && angle < smallyaw_x + 8) { // SmallYaw_SpeedPID.OUT = 400; // } // else if(angle > smallyaw_x - 154 && angle < smallyaw_x - 74) { // SmallYaw_SpeedPID.OUT = -400;

现在我们进行问答环节

int16_t origin_BigYaw_count = 6537; int16_t origin_SmallYaw_count = 2233;

int16_t now_BigYaw_count = 0; int16_t now_SmallYaw_count = 0;

int16_t error_BigYaw_count = 0; int16_t error_SmallYaw_count = 0; 这是三组参数，你是怎么理解的， origin_被我理解为机械中值 now_被我理解为每次开机时的当前定位 error_被