#include "remote_control.h"

/* ----------------------- Sentry Guard Protocol Definition ---------------------- */
#define SENTRY_FRAME_HEAD       0xAA
#define SENTRY_FRAME_LENGTH     14      // 头(1) + ch0~4(10) + s0~1(2) + 长度尾(1)

void Message_Remote_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3

    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/**
 * @brief 哨兵机器人专用遥控器解析（帧头AA，固定长度14字节）同时ch上下界是-660~660，s是1~3
 * @param sbus_buf 接收缓冲区（volatile，与标准函数保持一致）
 * @param rc_ctrl 解析结果（填充 rc.ch[0~4] 和 rc.s[0~1]）
 * 
 * @note 帧格式：AA | ch0L ch0H | ch1L ch1H | ... | ch4L ch4H | s0 | s1 | 0x0E
 */
void Message_Remote_to_rc_guard(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    // 帧头校验
    if (sbus_buf[0] != SENTRY_FRAME_HEAD) {
        return;
    }
    
    // 尾帧校验（第13字节应为数据长度 14）
    if (sbus_buf[13] != SENTRY_FRAME_LENGTH) {
        return;
    }
    
    // 解析 ch[0] ~ ch[4]（小端格式）
    rc_ctrl->rc.ch[0] = (int16_t)(sbus_buf[1] | (sbus_buf[2] << 8));  //右摇杆 底盘左右
    rc_ctrl->rc.ch[1] = (int16_t)(sbus_buf[3] | (sbus_buf[4] << 8));  //右摇杆 底盘前后
    rc_ctrl->rc.ch[2] = (int16_t)(sbus_buf[5] | (sbus_buf[6] << 8));  //左摇杆 云台左右
    rc_ctrl->rc.ch[3] = (int16_t)(sbus_buf[7] | (sbus_buf[8] << 8));  //左摇杆 云台上下
    rc_ctrl->rc.ch[4] = (int16_t)(sbus_buf[9] | (sbus_buf[10] << 8)); //我暂时没有用上的
    
    // 解析开关 s[0], s[1]
    rc_ctrl->rc.s[0] = (char)sbus_buf[11];  //右开关
    rc_ctrl->rc.s[1] = (char)sbus_buf[12];  //左开关
}