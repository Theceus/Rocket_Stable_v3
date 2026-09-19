/*
 * @brief   模型火箭姿态自稳系统（横滚/俯仰舵机控制 + OLED显示）
 * @note    MCU: STM32F103C8T6, 舵机: PA6(TIM3_CH1), PA7(TIM3_CH2)
 *          OLED: I2C1(PB8 SCL, PB9 SDA), MPU6050: I2C2(PB10 SCL, PB11 SDA)
 *          Beeper: PB12 开漏输出 + 10k上拉到5V，驱动PNP高边开关，低电平响
 * @version 优化版：卡尔曼滤波（可选）、硬件/软件I2C可选、PID计算精简
 *          控制模式：0停用/1普通比例/2神经自适应PID
 *          开仓后姿态舵机锁死中立
 *          蜂鸣器三档：关仓未发射心跳式哔哔响，关仓已发射三连哔，开仓后持续响
 *          倾斜保险增加角速度条件，防止振动误触发
 *          最高点检测更灵敏，增加零速修正防止积分漂移
 *          新增：按钮1(PB0)按住哔哔响，听到2声后松手触发系统自检
 *          自检中短按按钮1需二次确认才退出
 */

#include "stm32f10x.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// ==================== 硬件引脚宏定义 ====================
#define SERVO1_GPIO_PORT        GPIOA
#define SERVO1_GPIO_PIN         GPIO_Pin_6
#define SERVO2_GPIO_PORT        GPIOA
#define SERVO2_GPIO_PIN         GPIO_Pin_7

#define OLED_I2C_PORT           GPIOB
#define OLED_SCL_PIN            GPIO_Pin_8
#define OLED_SDA_PIN            GPIO_Pin_9

#define MPU_I2C_PORT            GPIOB
#define MPU_SCL_PIN             GPIO_Pin_10
#define MPU_SDA_PIN             GPIO_Pin_11

#define MPU6050_ADDR            0x68
#define OLED_ADDR               0x3C

#define BUTTON_GPIO_PORT        GPIOA       // 按钮3（校准触发）
#define BUTTON_GPIO_PIN         GPIO_Pin_8

/* === SelfTest 新增：按钮1（自检触发） === */
#define BUTTON1_GPIO_PORT       GPIOB
#define BUTTON1_GPIO_PIN        GPIO_Pin_0
/* ========================================= */

#define BEEPER_GPIO_PORT        GPIOB
#define BEEPER_GPIO_PIN         GPIO_Pin_12

// ==================== 用户调参区域 ====================

// ---------- 控制模式 ----------
// 0: 停用姿态舵机（锁死中立）
// 1: 普通即时比例控制（仅比例，最快响应）
// 2: 神经自适应PID（积分+微分，消除静差）
#define CONTROL_MODE            0 

// ---------- 算法选项 ----------
#define USE_KALMAN_FILTER       1   // 1: 启用卡尔曼滤波（提高精度，略微增加CPU负载）
                                    // 0: 使用传统互补滤波（更轻量）
#define USE_HARDWARE_I2C        0   // 1: 使用硬件I2C（400kHz，需注意STM32F103硬件Bug）
                                    // 0: 使用软件I2C（稳定，优化了时序）

// ---------- 倾斜保险 ----------
#define TILT_DETECT_CONSECUTIVE 3   // 连续超阈值次数（防抖）
float tilt_angle_limit = 60.0f;     // 倾斜角阈值（度）
#define TILT_GYRO_THRESHOLD  300.0f // 角速度阈值(°/s)，超过此值视为振动，保险不触发

// ---------- 飞行控制 ----------
float safety_delay_sec = 3.0f;      // 发射后强制开仓延时（秒）
float startAcc = 5.0f;              // 起飞加速度阈值（m/s²）

// ---------- 舵机物理限制 & 方向 ----------
float servo_angle_limit = 30.0f;    // 姿态舵机最大偏转角度（度）
float deadband_angle = 5.0f;        // 死区角度（度）
#define SERVO1_DIR              1   // 横滚方向修正（1 或 -1）
#define SERVO2_DIR              1   // 俯仰方向修正

// ---------- PWM 频率 ----------
#define SERVO_PWM_FREQ_HZ       300 // 固定300Hz

// ---------- 神经自适应PID参数（仅模式2生效） ----------
float initial_gain = 2.0f;          // 初始比例增益
float initial_deriv = 0.02f;        // 初始微分增益

#define NEURO_LEARN_RATE_P_R     0.005f  // 横滚比例学习率
#define NEURO_LEARN_RATE_I_R     0.0001f // 横滚积分学习率
#define NEURO_LEARN_RATE_D_R     0.001f  // 横滚微分学习率
#define NEURO_LEARN_RATE_P_P     0.005f
#define NEURO_LEARN_RATE_I_P     0.0001f
#define NEURO_LEARN_RATE_D_P     0.001f

#define NEURO_INTEGRAL_LIMIT     6.0f    // 积分限幅（度）
#define NEURO_WEIGHT_LIMIT       5.0f    // 权值限幅

// ---------- 姿态解算 ----------
float comp_filter_alpha = 0.96f;        // 互补滤波权重（仅当 USE_KALMAN_FILTER=0 时生效）
float dt = 0.01f;                       // 固定控制周期（秒）
uint16_t calib_samples = 500;           // 校准采样次数
uint8_t mpu_dlpf_cfg = 0x03;            // MPU6050内部低通滤波（0x03=44Hz）

// ---------- MPU6050安装方向 ----------
int8_t accel_sign_x = 1;
int8_t accel_sign_y = 1;
int8_t accel_sign_z = 1;
int8_t gyro_sign_x  = 1;
int8_t gyro_sign_y  = 1;
int8_t gyro_sign_z  = 1;

// ---------- 舵机3（开仓） ----------
#define SERVO3_ANGLE_MAX        180.0f
#define SERVO3_ANGLE_DEFAULT    0.0f
#define SERVO3_ANGLE_OPEN       55.0f
#define SERVO3_DIR              -1

// ---------- 蜂鸣器心跳 ----------
#define BEEP_PERIOD_MS          1000    // 关仓慢滴周期
#define BEEP_DURATION_MS        100     // 每次持续时间

/* === SelfTest 新增：自检阈值与提示音参数 === */
#define SELFTEST_SAMPLES            50      // 自检采样次数
#define SELFTEST_ACCEL_MIN_G        0.80f   // 加速度模长下限(g)
#define SELFTEST_ACCEL_MAX_G        1.20f   // 加速度模长上限(g)
#define SELFTEST_GYRO_BIAS_LIMIT    10.0f   // 陀螺仪零偏模长阈值(°/s)
#define SELFTEST_HOLD_STILL_MS      1500    // 自检开始前静止提示时间(ms)
#define SELFTEST_ITEM_RESULT_MS     400     // 单项结果停留时间(ms)
#define SELFTEST_END_PASS_MS        2500    // 全部通过停留时间(ms)
#define SELFTEST_END_FAIL_MS        3000    // 有失败停留时间(ms)
#define SELFTEST_DOOR_CLOSED_MS     3000    // 未开仓提示停留时间(ms)

// 按钮1按住蜂鸣参数
#define B1_BEEP_ON_MS               80      // 每声蜂鸣持续时间
#define B1_BEEP_GAP_MS              420     // 两声蜂鸣之间的间隔
#define B1_BEEP_MIN_TRIGGER         2       // 至少听到2声才触发
#define B1_BEEP_MAX                 8       // 最多响8声

// 自检确认退出参数
#define SELFTEST_CONFIRM_WINDOW_MS  3000    // 二次确认等待时间
#define SELFTEST_SUPPRESS_MS        1000    // 按钮按住超过1秒视为误触

// 蜂鸣器自检专用提示音（不与原有三档重复）
#define BEEP_START_ON        50
#define BEEP_START_OFF       50
#define BEEP_START_CNT       3

#define BEEP_PASS_ON         60
#define BEEP_FAIL_ON         80
#define BEEP_FAIL_OFF        80
#define BEEP_FAIL_CNT        2

#define BEEP_END_OK_ON       200
#define BEEP_END_OK_OFF      100
#define BEEP_END_OK_CNT      2

#define BEEP_END_NG_ON       300
#define BEEP_END_NG_OFF      100
#define BEEP_END_NG_CNT      3

#define BEEP_NEED_OPEN_ON    500
#define BEEP_NEED_OPEN_OFF   200
#define BEEP_NEED_OPEN_CNT   2

#define BEEP_CANCEL_ON       400
#define BEEP_CANCEL_CNT      1
/* ============================================ */

// ---------- OLED 显示文字 ----------
const char* DISPLAY_ROCKET_NAME = " AHU.UniversityRocket";
const char* DISPLAY_DEVELOPER   = "Dev by TheceusSun";
const char* DISPLAY_CALIB_MSG   = "=> Calibrating...";
const char* DISPLAY_CALIB_DONE  = "=> Calib Done!   ";

// ==================== 硬件引脚（开仓舵机及按钮2） ====================
#define SERVO3_GPIO_PORT        GPIOA
#define SERVO3_GPIO_PIN         GPIO_Pin_11
#define BUTTON2_GPIO_PORT       GPIOB
#define BUTTON2_GPIO_PIN        GPIO_Pin_1

// ==================== 全局变量 ====================
volatile uint32_t sysTick_ms = 0;

float roll_angle = 0.0f;
float pitch_angle = 0.0f;
float gyro_rate_x_deg = 0.0f;
float gyro_rate_y_deg = 0.0f;

int16_t gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float accel_offset_angle_roll = 0.0f;
float accel_offset_angle_pitch = 0.0f;
uint8_t calibration_done = 0;

uint8_t tilt_triggered = 0;
uint32_t safety_deadline_ms = 0;

uint8_t servo3_state = 0;

float vertical_velocity = 0.0f;
float vertical_position = 0.0f;
float max_up_accel = 0.0f;
float max_height = 0.0f;
uint8_t launched = 0;
uint8_t apogee_reached = 0;

// 神经PID状态（仅模式2）
#if CONTROL_MODE == 2
static float neuro_wp_r = 0.0f, neuro_wi_r = 0.0f, neuro_wd_r = 0.0f;
static float neuro_prev_e_r = 0.0f, neuro_prev_u_r = 0.0f;
static float neuro_int_r = 0.0f;
static float neuro_wp_p = 0.0f, neuro_wi_p = 0.0f, neuro_wd_p = 0.0f;
static float neuro_prev_e_p = 0.0f, neuro_prev_u_p = 0.0f;
static float neuro_int_p = 0.0f;
#endif

// 卡尔曼滤波状态（仅当 USE_KALMAN_FILTER=1 时使用）
#if USE_KALMAN_FILTER
typedef struct {
    float x_est;      // 角度估计值
    float p_est;      // 估计误差协方差
    float q;          // 过程噪声协方差（角度随机游走）
    float r;          // 测量噪声协方差（加速度计角度噪声）
} KalmanState_t;

static KalmanState_t kalman_r = {0, 1.0f, 0.02f, 0.05f};
static KalmanState_t kalman_p = {0, 1.0f, 0.02f, 0.05f};
#endif

// ==================== 函数声明 ====================
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void I2C_User_Init(void);
void I2C_Write(uint8_t addr, uint8_t reg, uint8_t data);
void I2C_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t decimal);

void MPU6050_Init(void);
void MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6050_Calibrate(void);

void Attitude_Update(void);

void Servo_Init(void);
void Servo_SetAngle(uint8_t channel, float target_angle);
void Servo_Reset(void);

void Servo3_Init(void);
void Servo3_SetAngle(float angle);
void Servo3_Toggle(void);

void Button2_Init(void);
uint8_t Button2_IsPressed(void);

void Button_Init(void);
uint8_t Button_IsPressed(void);

void System_Init(void);

static void NeuroPID_Reset(void);

void Beeper_Init(void);
void Beeper_On(void);
void Beeper_Off(void);
void Beeper_Update(void);

/* === SelfTest 新增：函数声明 === */
void Button1_Init(void);
uint8_t Button1_IsDown(void);
uint8_t Button1_Process(void);   // 返回1表示当前占用蜂鸣器

void Beeper_PlayPattern(uint16_t on_ms, uint16_t off_ms, uint8_t count);
void FlightState_Reset(void);

uint8_t SelfTest_CheckCancel(void);
void SelfTest_Cancel(void);
void SelfTest_Run(void);
/* ==================================== */

// ==================== 延时函数 ====================
void delay_ms(uint32_t ms) {
    uint32_t start = sysTick_ms;
    while ((sysTick_ms - start) < ms);
}

void delay_us(uint32_t us) {
    uint32_t i;
    for (i = 0; i < us * 8; i++) {
        __NOP();
    }
}

// ==================== I2C 通信层 ====================
#if USE_HARDWARE_I2C
// ---------- 硬件 I2C（STM32F103 硬件 I2C，可能存在 Bug） ----------
static void I2C_HW_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 400000;

    I2C_Init(I2C1, &I2C_InitStruct);
    I2C_Init(I2C2, &I2C_InitStruct);
    I2C_Cmd(I2C1, ENABLE);
    I2C_Cmd(I2C2, ENABLE);
}

static uint8_t I2C_HW_Write(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    I2C_TypeDef *I2Cx = (dev_addr == OLED_ADDR) ? I2C1 : I2C2;
    uint32_t timeout = 10000;
    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) if (timeout-- == 0) return 1;
    I2C_Send7bitAddress(I2Cx, dev_addr, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) if (timeout-- == 0) return 1;
    I2C_SendData(I2Cx, reg);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) if (timeout-- == 0) return 1;
    I2C_SendData(I2Cx, data);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) if (timeout-- == 0) return 1;
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 0;
}

static uint8_t I2C_HW_Read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    I2C_TypeDef *I2Cx = (dev_addr == OLED_ADDR) ? I2C1 : I2C2;
    uint32_t timeout = 10000;
    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) if (timeout-- == 0) return 1;
    I2C_Send7bitAddress(I2Cx, dev_addr, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) if (timeout-- == 0) return 1;
    I2C_SendData(I2Cx, reg);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) if (timeout-- == 0) return 1;
    I2C_GenerateSTART(I2Cx, ENABLE);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) if (timeout-- == 0) return 1;
    I2C_Send7bitAddress(I2Cx, dev_addr, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) if (timeout-- == 0) return 1;
    while (len) {
        if (len == 1) I2C_AcknowledgeConfig(I2Cx, DISABLE);
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) if (timeout-- == 0) return 1;
        *buf++ = I2C_ReceiveData(I2Cx);
        len--;
    }
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 0;
}

void I2C_User_Init(void) { I2C_HW_Init(); }
#define I2C_Write(addr, reg, data) I2C_HW_Write(addr, reg, data)
#define I2C_Read(addr, reg, buf, len) I2C_HW_Read(addr, reg, buf, len)

#else
// ---------- 软件 I2C（优化时序） ----------
#define OLED_I2C_SCL_H()   GPIO_SetBits(OLED_I2C_PORT, OLED_SCL_PIN)
#define OLED_I2C_SCL_L()   GPIO_ResetBits(OLED_I2C_PORT, OLED_SCL_PIN)
#define OLED_I2C_SDA_H()   GPIO_SetBits(OLED_I2C_PORT, OLED_SDA_PIN)
#define OLED_I2C_SDA_L()   GPIO_ResetBits(OLED_I2C_PORT, OLED_SDA_PIN)

#define MPU_I2C_SCL_H()   GPIO_SetBits(MPU_I2C_PORT, MPU_SCL_PIN)
#define MPU_I2C_SCL_L()   GPIO_ResetBits(MPU_I2C_PORT, MPU_SCL_PIN)
#define MPU_I2C_SDA_H()   GPIO_SetBits(MPU_I2C_PORT, MPU_SDA_PIN)
#define MPU_I2C_SDA_L()   GPIO_ResetBits(MPU_I2C_PORT, MPU_SDA_PIN)

static void I2C_Delay(void) {
    uint32_t i = 4;
    while (i--) __NOP();
}

void I2C_User_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_I2C_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = MPU_SCL_PIN | MPU_SDA_PIN;
    GPIO_Init(MPU_I2C_PORT, &GPIO_InitStruct);
    GPIO_SetBits(OLED_I2C_PORT, OLED_SCL_PIN | OLED_SDA_PIN);
    GPIO_SetBits(MPU_I2C_PORT, MPU_SCL_PIN | MPU_SDA_PIN);
}

static void SW_I2C_Start(uint8_t dev_addr) {
    if (dev_addr == OLED_ADDR) {
        OLED_I2C_SDA_H(); OLED_I2C_SCL_H(); I2C_Delay();
        OLED_I2C_SDA_L(); I2C_Delay();
        OLED_I2C_SCL_L();
    } else {
        MPU_I2C_SDA_H(); MPU_I2C_SCL_H(); I2C_Delay();
        MPU_I2C_SDA_L(); I2C_Delay();
        MPU_I2C_SCL_L();
    }
}

static void SW_I2C_Stop(uint8_t dev_addr) {
    if (dev_addr == OLED_ADDR) {
        OLED_I2C_SDA_L(); OLED_I2C_SCL_H(); I2C_Delay();
        OLED_I2C_SDA_H(); I2C_Delay();
    } else {
        MPU_I2C_SDA_L(); MPU_I2C_SCL_H(); I2C_Delay();
        MPU_I2C_SDA_H(); I2C_Delay();
    }
}

static uint8_t SW_I2C_WaitAck(uint8_t dev_addr) {
    uint8_t ack;
    if (dev_addr == OLED_ADDR) {
        OLED_I2C_SDA_H(); I2C_Delay();
        OLED_I2C_SCL_H(); I2C_Delay();
        ack = GPIO_ReadInputDataBit(OLED_I2C_PORT, OLED_SDA_PIN);
        OLED_I2C_SCL_L();
    } else {
        MPU_I2C_SDA_H(); I2C_Delay();
        MPU_I2C_SCL_H(); I2C_Delay();
        ack = GPIO_ReadInputDataBit(MPU_I2C_PORT, MPU_SDA_PIN);
        MPU_I2C_SCL_L();
    }
    return ack;
}

static void SW_I2C_SendByte(uint8_t dev_addr, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (dev_addr == OLED_ADDR) {
            if (data & 0x80) OLED_I2C_SDA_H(); else OLED_I2C_SDA_L();
            I2C_Delay(); OLED_I2C_SCL_H(); I2C_Delay(); OLED_I2C_SCL_L();
        } else {
            if (data & 0x80) MPU_I2C_SDA_H(); else MPU_I2C_SDA_L();
            I2C_Delay(); MPU_I2C_SCL_H(); I2C_Delay(); MPU_I2C_SCL_L();
        }
        data <<= 1;
    }
    SW_I2C_WaitAck(dev_addr);
}

static uint8_t SW_I2C_ReadByte(uint8_t dev_addr) {
    uint8_t i, data = 0;
    if (dev_addr == OLED_ADDR) {
        OLED_I2C_SDA_H();
        for (i = 0; i < 8; i++) {
            data <<= 1;
            OLED_I2C_SCL_H(); I2C_Delay();
            if (GPIO_ReadInputDataBit(OLED_I2C_PORT, OLED_SDA_PIN)) data |= 0x01;
            OLED_I2C_SCL_L(); I2C_Delay();
        }
    } else {
        MPU_I2C_SDA_H();
        for (i = 0; i < 8; i++) {
            data <<= 1;
            MPU_I2C_SCL_H(); I2C_Delay();
            if (GPIO_ReadInputDataBit(MPU_I2C_PORT, MPU_SDA_PIN)) data |= 0x01;
            MPU_I2C_SCL_L(); I2C_Delay();
        }
    }
    return data;
}

void I2C_Write(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    SW_I2C_Start(dev_addr);
    SW_I2C_SendByte(dev_addr, dev_addr << 1);
    SW_I2C_SendByte(dev_addr, reg);
    SW_I2C_SendByte(dev_addr, data);
    SW_I2C_Stop(dev_addr);
}

void I2C_Read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t i;
    SW_I2C_Start(dev_addr);
    SW_I2C_SendByte(dev_addr, dev_addr << 1);
    SW_I2C_SendByte(dev_addr, reg);
    SW_I2C_Start(dev_addr);
    SW_I2C_SendByte(dev_addr, (dev_addr << 1) | 0x01);
    for (i = 0; i < len; i++) {
        if (i == len - 1) {
            buf[i] = SW_I2C_ReadByte(dev_addr);
        } else {
            buf[i] = SW_I2C_ReadByte(dev_addr);
            if (dev_addr == OLED_ADDR) {
                OLED_I2C_SDA_L(); I2C_Delay(); OLED_I2C_SCL_H(); I2C_Delay(); OLED_I2C_SCL_L();
            } else {
                MPU_I2C_SDA_L(); I2C_Delay(); MPU_I2C_SCL_H(); I2C_Delay(); MPU_I2C_SCL_L();
            }
        }
    }
    SW_I2C_Stop(dev_addr);
}
#endif // USE_HARDWARE_I2C

// ==================== OLED 驱动 ====================
void OLED_WriteCmd(uint8_t cmd) { I2C_Write(OLED_ADDR, 0x00, cmd); }
void OLED_WriteData(uint8_t data) { I2C_Write(OLED_ADDR, 0x40, data); }

void OLED_Init(void) {
    I2C_User_Init();
    delay_ms(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xB0);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0x7F);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0xF0);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0x22);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
    OLED_Clear();
}

void OLED_Clear(void) {
    uint8_t i, j;
    for (i = 0; i < 8; i++) {
        OLED_WriteCmd(0xB0 + i);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        for (j = 0; j < 128; j++) OLED_WriteData(0x00);
    }
}

void OLED_SetPos(uint8_t x, uint8_t y) {
    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(0x00 + (x & 0x0F));
    OLED_WriteCmd(0x10 + ((x >> 4) & 0x0F));
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str) {
    static const unsigned char F6x8[][6] = {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // sp
        { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 }, // !
        { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 }, // "
        { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 }, // #
        { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 }, // $
        { 0x00, 0x23, 0x13, 0x08, 0x64, 0x62 }, // %
        { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 }, // &
        { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 }, // '
        { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 }, // (
        { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 }, // )
        { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 }, // *
        { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 }, // +
        { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 }, // ,
        { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 }, // -
        { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 }, // .
        { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 }, // /
        { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E }, // 0
        { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 }, // 1
        { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 }, // 2
        { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 }, // 3
        { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 }, // 4
        { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5
        { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 }, // 6
        { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 }, // 7
        { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8
        { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E }, // 9
        { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 }, // :
        { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 }, // ;
        { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 }, // <
        { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 }, // =
        { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 }, // >
        { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 }, // ?
        { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E }, // @
        { 0x00, 0x7E, 0x11, 0x11, 0x11, 0x7E }, // A
        { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 }, // B
        { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 }, // C
        { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C }, // D
        { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 }, // E
        { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 }, // F
        { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A }, // G
        { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F }, // H
        { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 }, // I
        { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 }, // J
        { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 }, // K
        { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 }, // L
        { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F }, // M
        { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F }, // N
        { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E }, // O
        { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 }, // P
        { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E }, // Q
        { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 }, // R
        { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 }, // S
        { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 }, // T
        { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F }, // U
        { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F }, // V
        { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F }, // W
        { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 }, // X
        { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 }, // Y
        { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 }, // Z
        { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 }, // [
        { 0x00, 0x02, 0x04, 0x08, 0x10, 0x20 }, // "\"
        { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 }, // ]
        { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 }, // ^
        { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 }, // _
        { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 }, // `
        { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 }, // a
        { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 }, // b
        { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 }, // c
        { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F }, // d
        { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 }, // e
        { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 }, // f
        { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C }, // g
        { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 }, // h
        { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 }, // i
        { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 }, // j
        { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 }, // k
        { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 }, // l
        { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 }, // m
        { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 }, // n
        { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 }, // o
        { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 }, // p
        { 0x00, 0x18, 0x24, 0x24, 0x28, 0xFC }, // q
        { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 }, // r
        { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 }, // s
        { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 }, // t
        { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C }, // u
        { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C }, // v
        { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C }, // w
        { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 }, // x
        { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C }, // y
        { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 }, // z
        { 0x00, 0x08, 0x36, 0x41, 0x41, 0x00 }, // {
        { 0x00, 0x00, 0x00, 0x77, 0x00, 0x00 }, // |
        { 0x00, 0x00, 0x41, 0x41, 0x36, 0x08 }, // }
        { 0x00, 0x02, 0x01, 0x02, 0x04, 0x02 }, // ~
    };
    uint8_t c;
    while (*str) {
        c = *str - ' ';
        if (c > 94) c = 0;
        OLED_SetPos(x, y);
        for (uint8_t i = 0; i < 6; i++) OLED_WriteData(F6x8[c][i]);
        x += 6;
        if (x > 127) break;
        str++;
    }
}

void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t decimal) {
    char buf[16];
    sprintf(buf, "%.*f", decimal, num);
    OLED_ShowString(x, y, buf);
}

// ==================== MPU6050 驱动 ====================
void MPU6050_Init(void) {
    I2C_User_Init();
    delay_ms(10);
    I2C_Write(MPU6050_ADDR, 0x6B, 0x80);
    delay_ms(100);
    I2C_Write(MPU6050_ADDR, 0x6B, 0x01);
    delay_ms(10);
    I2C_Write(MPU6050_ADDR, 0x1C, 0x00);
    I2C_Write(MPU6050_ADDR, 0x1B, 0x00);
    I2C_Write(MPU6050_ADDR, 0x1A, mpu_dlpf_cfg);
}

void MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[14];
    I2C_Read(MPU6050_ADDR, 0x3B, buf, 14);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void MPU6050_Calibrate(void) {
    int16_t ax, ay, az, gx, gy, gz;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t i;
    for (i = 0; i < calib_samples; i++) {
        MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax * accel_sign_x; sum_ay += ay * accel_sign_y; sum_az += az * accel_sign_z;
        sum_gx += gx * gyro_sign_x; sum_gy += gy * gyro_sign_y; sum_gz += gz * gyro_sign_z;
        delay_ms(2);
    }
    accel_offset_angle_roll = atan2((float)sum_ay / calib_samples, (float)sum_az / calib_samples) * 57.2958f;
    accel_offset_angle_pitch = atan2(-(float)sum_ax / calib_samples,
                                     sqrt(((float)sum_ay / calib_samples) * ((float)sum_ay / calib_samples) +
                                          ((float)sum_az / calib_samples) * ((float)sum_az / calib_samples))) * 57.2958f;
    gyro_offset_x = sum_gx / calib_samples;
    gyro_offset_y = sum_gy / calib_samples;
    gyro_offset_z = sum_gz / calib_samples;
    calibration_done = 1;
}

// ==================== 卡尔曼滤波函数 ====================
#if USE_KALMAN_FILTER
static float Kalman_Update(KalmanState_t *k, float z) {
    float x_pred = k->x_est;
    float p_pred = k->p_est + k->q;
    float k_gain = p_pred / (p_pred + k->r);
    k->x_est = x_pred + k_gain * (z - x_pred);
    k->p_est = (1.0f - k_gain) * p_pred;
    return k->x_est;
}
#endif

// ==================== 姿态解算 ====================
void Attitude_Update(void) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    float ax, ay, az, gx, gy;
    float accel_roll, accel_pitch, gyro_rate_x, gyro_rate_y;
    static uint32_t last_time = 0;
    uint32_t now = sysTick_ms;
    float dt = 0.01f;
    static float prev_vel = 0.0f;

    if (last_time == 0) {
        last_time = now;
    } else {
        float real_dt = (now - last_time) / 1000.0f;
        if (real_dt > 0.02f) real_dt = 0.01f;
        else if (real_dt < 0.001f) real_dt = 0.01f;
        dt = real_dt;
    }
    last_time = now;

    MPU6050_ReadAll(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    ax = (float)ax_raw * accel_sign_x / 16384.0f;
    ay = (float)ay_raw * accel_sign_y / 16384.0f;
    az = (float)az_raw * accel_sign_z / 16384.0f;
    gx = ((float)gx_raw - gyro_offset_x) * gyro_sign_x;
    gy = ((float)gy_raw - gyro_offset_y) * gyro_sign_y;

    accel_roll  = atan2(ay, az) * 57.2958f - accel_offset_angle_roll;
    accel_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958f - accel_offset_angle_pitch;
    gyro_rate_x = gx / 131.0f;
    gyro_rate_y = gy / 131.0f;

#if USE_KALMAN_FILTER
    float cf_roll  = comp_filter_alpha * (roll_angle  + gyro_rate_x * dt) + (1.0f - comp_filter_alpha) * accel_roll;
    float cf_pitch = comp_filter_alpha * (pitch_angle + gyro_rate_y * dt) + (1.0f - comp_filter_alpha) * accel_pitch;
    roll_angle  = Kalman_Update(&kalman_r, cf_roll);
    pitch_angle = Kalman_Update(&kalman_p, cf_pitch);
#else
    roll_angle  = comp_filter_alpha * (roll_angle  + gyro_rate_x * dt) + (1.0f - comp_filter_alpha) * accel_roll;
    pitch_angle = comp_filter_alpha * (pitch_angle + gyro_rate_y * dt) + (1.0f - comp_filter_alpha) * accel_pitch;
#endif

    gyro_rate_x_deg = gyro_rate_x;
    gyro_rate_y_deg = gyro_rate_y;

    // ---------- 倾斜保险 ----------
    if (calibration_done) {
        float tilt_angle = sqrtf(roll_angle * roll_angle + pitch_angle * pitch_angle);
        float gyro_magnitude = sqrtf(gyro_rate_x_deg * gyro_rate_x_deg + gyro_rate_y_deg * gyro_rate_y_deg);

        static uint8_t tilt_exceed_counter = 0;

        if ((tilt_angle > tilt_angle_limit) && (gyro_magnitude < TILT_GYRO_THRESHOLD)) {
            tilt_exceed_counter++;
            if (tilt_exceed_counter >= TILT_DETECT_CONSECUTIVE && !tilt_triggered) {
                Servo3_SetAngle(SERVO3_ANGLE_OPEN);
                servo3_state = 1;
                tilt_triggered = 1;
                apogee_reached = 1;
                launched = 0;
                safety_deadline_ms = 0;
                tilt_exceed_counter = 0;
                roll_angle = 0.0f; pitch_angle = 0.0f;
                gyro_rate_x_deg = 0.0f; gyro_rate_y_deg = 0.0f;
                NeuroPID_Reset();
            }
        } else {
            tilt_exceed_counter = 0;
        }
    }

    // ---------- 垂直加速度与飞行检测 ----------
    static float az_filtered = 1.0f;
    az_filtered = 0.8f * az_filtered + 0.2f * az;
    float accel_up = (az_filtered - 1.0f) * 9.80665f;

    #define STATIC_ACCEL_THRESHOLD  0.5f
    static uint8_t static_counter = 0;

    if (!launched) {
        if (fabs(accel_up) < STATIC_ACCEL_THRESHOLD) {
            static_counter++;
            if (static_counter >= 10) {
                vertical_velocity = 0.0f;
                vertical_position = 0.0f;
                static_counter = 0;
            }
        } else {
            static_counter = 0;
        }

        static uint8_t accel_high_counter = 0;
        if (accel_up > startAcc) {
            accel_high_counter++;
            if (accel_high_counter >= 3) {
                launched = 1;
                apogee_reached = 0;
                vertical_velocity = 0.0f; vertical_position = 0.0f;
                max_up_accel = 0.0f; max_height = 0.0f;
                accel_high_counter = 0;
                safety_deadline_ms = sysTick_ms + (uint32_t)(safety_delay_sec * 1000.0f);
            }
        } else {
            accel_high_counter = 0;
        }
    } else {
        vertical_velocity += accel_up * dt;
        if (fabs(vertical_velocity) < 0.1f) {
            vertical_velocity = 0.0f;
        }
        vertical_position += vertical_velocity * dt;

        if (accel_up > max_up_accel && accel_up < 100.0f) max_up_accel = accel_up;
        if (vertical_position > max_height && vertical_position < 5000.0f) max_height = vertical_position;

        if (fabs(vertical_velocity) > 500.0f || vertical_position > 5000.0f || vertical_position < -500.0f) {
            launched = 0; vertical_velocity = 0.0f; vertical_position = 0.0f;
            max_up_accel = 0.0f; max_height = 0.0f;
            safety_deadline_ms = 0; prev_vel = 0.0f;
            return;
        }

        if (safety_deadline_ms != 0 && sysTick_ms >= safety_deadline_ms) {
            Servo3_SetAngle(SERVO3_ANGLE_OPEN);
            servo3_state = 1;
            apogee_reached = 1; launched = 0;
            safety_deadline_ms = 0; prev_vel = 0.0f;
            roll_angle = 0.0f; pitch_angle = 0.0f;
            gyro_rate_x_deg = 0.0f; gyro_rate_y_deg = 0.0f;
            NeuroPID_Reset();
            return;
        }

        if ((prev_vel > 0.05f) && (vertical_velocity <= 0.0f) && (accel_up < -0.5f)) {
            Servo3_SetAngle(SERVO3_ANGLE_OPEN);
            servo3_state = 1;
            apogee_reached = 1; launched = 0;
            safety_deadline_ms = 0;
            roll_angle = 0.0f; pitch_angle = 0.0f;
            gyro_rate_x_deg = 0.0f; gyro_rate_y_deg = 0.0f;
            NeuroPID_Reset();
        }
        prev_vel = vertical_velocity;
    }
}

// ==================== 神经PID复位 ====================
static void NeuroPID_Reset(void) {
#if CONTROL_MODE == 2
    neuro_int_r = 0.0f; neuro_int_p = 0.0f;
    neuro_prev_e_r = 0.0f; neuro_prev_e_p = 0.0f;
    neuro_prev_u_r = 0.0f; neuro_prev_u_p = 0.0f;
#endif
}

/* === SelfTest 新增：飞行状态复位 === */
void FlightState_Reset(void) {
    vertical_velocity = 0.0f;
    vertical_position = 0.0f;
    max_up_accel = 0.0f;
    max_height = 0.0f;
    launched = 0;
    apogee_reached = 0;
    safety_deadline_ms = 0;
    tilt_triggered = 0;

    roll_angle = 0.0f;
    pitch_angle = 0.0f;
    gyro_rate_x_deg = 0.0f;
    gyro_rate_y_deg = 0.0f;

    NeuroPID_Reset();
}
/* =================================== */

// ==================== 舵机控制 ====================
void Servo_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SERVO1_GPIO_PIN | SERVO2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO1_GPIO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 3332;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetAngle(uint8_t channel, float target_angle) {
    if (channel == 1) target_angle *= SERVO1_DIR;
    else if (channel == 2) target_angle *= SERVO2_DIR;
    if (target_angle > servo_angle_limit) target_angle = servo_angle_limit;
    if (target_angle < -servo_angle_limit) target_angle = -servo_angle_limit;
    uint16_t pulse;
    if (fabs(target_angle) < deadband_angle) pulse = 1500;
    else pulse = (uint16_t)(1500 + target_angle * (2000.0f / 180.0f));
    if (channel == 1) TIM_SetCompare1(TIM3, pulse);
    else if (channel == 2) TIM_SetCompare2(TIM3, pulse);
}

void Servo_Reset(void) {
    TIM_SetCompare1(TIM3, 1500);
    TIM_SetCompare2(TIM3, 1500);
}

void Servo3_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SERVO3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO3_GPIO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 3332;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
}

void Servo3_SetAngle(float angle) {
    angle *= SERVO3_DIR;
    if (angle > SERVO3_ANGLE_MAX) angle = SERVO3_ANGLE_MAX;
    if (angle < -SERVO3_ANGLE_MAX) angle = -SERVO3_ANGLE_MAX;
    uint16_t pulse = (uint16_t)(1500 + angle * (2000.0f / 180.0f));
    TIM_SetCompare4(TIM1, pulse);
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
}

void Servo3_Toggle(void) {
    servo3_state = !servo3_state;
    if (servo3_state == 0) Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
    else Servo3_SetAngle(SERVO3_ANGLE_OPEN);
}

// ==================== 蜂鸣器 ====================
void Beeper_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BEEPER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;   // 开漏输出，外部10k上拉到5V
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEPER_GPIO_PORT, &GPIO_InitStructure);

    Beeper_Off();   // 上电默认关闭
}

void Beeper_On(void)  { GPIO_ResetBits(BEEPER_GPIO_PORT, BEEPER_GPIO_PIN); } // 拉低，PNP导通
void Beeper_Off(void) { GPIO_SetBits(BEEPER_GPIO_PORT, BEEPER_GPIO_PIN); }   // 高阻，PNP截止

// 蜂鸣器模式定义
#define BEEP_MODE_SLOW      0   // 关仓未发射：滴——滴——滴——
#define BEEP_MODE_TRIPLE    1   // 关仓已发射：滴滴滴——滴滴滴——
#define BEEP_MODE_ON        2   // 开仓：滴——（持续）

static uint8_t  beep_mode_cur   = 0xFF;   // 当前模式
static uint8_t  beep_phase      = 0;      // 当前阶段
static uint32_t beep_phase_start = 0;     // 阶段开始时间

void Beeper_Update(void) {
    uint8_t target_mode;

    if (servo3_state == 1) {
        target_mode = BEEP_MODE_ON;
    } else if (launched) {
        target_mode = BEEP_MODE_TRIPLE;
    } else {
        target_mode = BEEP_MODE_SLOW;
    }

    if (target_mode != beep_mode_cur) {
        beep_mode_cur = target_mode;
        beep_phase = 0;
        beep_phase_start = sysTick_ms;
    }

    uint32_t elapsed = sysTick_ms - beep_phase_start;

    if (target_mode == BEEP_MODE_ON) {
        Beeper_On();
        return;
    }

    if (target_mode == BEEP_MODE_SLOW) {
        // 周期 1000ms：响 100ms，停 900ms
        if (beep_phase == 0) {
            Beeper_On();
            if (elapsed >= BEEP_DURATION_MS) {
                Beeper_Off();
                beep_phase = 1;
                beep_phase_start = sysTick_ms;
            }
        } else {
            Beeper_Off();
            if (elapsed >= (BEEP_PERIOD_MS - BEEP_DURATION_MS)) {
                beep_phase = 0;
                beep_phase_start = sysTick_ms;
            }
        }
    } else { // BEEP_MODE_TRIPLE
        // 序列：响100、停100、响100、停100、响100、停400（周期900ms）
        switch (beep_phase) {
            case 0: Beeper_On();  if (elapsed >= 100) { beep_phase = 1; beep_phase_start = sysTick_ms; } break;
            case 1: Beeper_Off(); if (elapsed >= 100) { beep_phase = 2; beep_phase_start = sysTick_ms; } break;
            case 2: Beeper_On();  if (elapsed >= 100) { beep_phase = 3; beep_phase_start = sysTick_ms; } break;
            case 3: Beeper_Off(); if (elapsed >= 100) { beep_phase = 4; beep_phase_start = sysTick_ms; } break;
            case 4: Beeper_On();  if (elapsed >= 100) { beep_phase = 5; beep_phase_start = sysTick_ms; } break;
            case 5: Beeper_Off(); if (elapsed >= 400) { beep_phase = 0; beep_phase_start = sysTick_ms; } break;
            default: beep_phase = 0; beep_phase_start = sysTick_ms; break;
        }
    }
}

/* === SelfTest 新增：阻塞式蜂鸣器播放 === */
void Beeper_PlayPattern(uint16_t on_ms, uint16_t off_ms, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        Beeper_On();
        delay_ms(on_ms);
        Beeper_Off();
        if (i < count - 1) delay_ms(off_ms);
    }
}
/* =========================================== */

// ==================== 按钮 ====================
void Button2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BUTTON2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON2_GPIO_PORT, &GPIO_InitStructure);
}

uint8_t Button2_IsPressed(void) {
    static uint32_t last_stable_ms = 0;
    static uint8_t last_state = 1;
    uint8_t current_state = GPIO_ReadInputDataBit(BUTTON2_GPIO_PORT, BUTTON2_GPIO_PIN);
    if (current_state != last_state) {
        last_stable_ms = sysTick_ms;
        last_state = current_state;
    }
    if (current_state == 0 && (sysTick_ms - last_stable_ms >= 20)) {
        while (GPIO_ReadInputDataBit(BUTTON2_GPIO_PORT, BUTTON2_GPIO_PIN) == 0);
        return 1;
    }
    return 0;
}

void Button_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStructure);
}

uint8_t Button_IsPressed(void) {
    static uint32_t last_stable_ms = 0;
    static uint8_t last_state = 1;
    uint8_t current_state = GPIO_ReadInputDataBit(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);
    if (current_state != last_state) {
        last_stable_ms = sysTick_ms;
        last_state = current_state;
    }
    if (current_state == 0 && (sysTick_ms - last_stable_ms >= 20)) {
        while (GPIO_ReadInputDataBit(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN) == 0);
        return 1;
    }
    return 0;
}

/* === SelfTest 新增：按钮1初始化、读电平、主循环处理 === */
void Button1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BUTTON1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON1_GPIO_PORT, &GPIO_InitStructure);
}

uint8_t Button1_IsDown(void) {
    return GPIO_ReadInputDataBit(BUTTON1_GPIO_PORT, BUTTON1_GPIO_PIN) == 0;
}

static uint32_t b1_next_beep_at   = 0;
static uint8_t  b1_pressed_flag   = 0;
static uint8_t  b1_beep_count     = 0;

/*
 * 按钮1按住逻辑：
 *   按下瞬间响第一声，之后每 500ms 响一声（80ms 响 + 420ms 停）。
 *   松手时若累计响过 >= 2 声 → 触发自检。
 *   返回值：1 表示当前正在占用蜂鸣器（主循环跳过 Beeper_Update）。
 */
uint8_t Button1_Process(void) {
    uint8_t cur = Button1_IsDown();

    if (cur && !b1_pressed_flag) {
        // 刚按下
        b1_pressed_flag = 1;
        b1_beep_count = 0;
        b1_next_beep_at = sysTick_ms;      // 立即响第一声
    }

    if (cur) {
        // 到时间就响一声
        if ((int32_t)(sysTick_ms - b1_next_beep_at) >= 0 && b1_beep_count < B1_BEEP_MAX) {
            Beeper_On();
            delay_ms(B1_BEEP_ON_MS);
            Beeper_Off();
            b1_beep_count++;
            b1_next_beep_at = sysTick_ms + B1_BEEP_GAP_MS;
        }
        return 1;   // 占用蜂鸣器
    } else {
        // 松手
        if (b1_pressed_flag) {
            b1_pressed_flag = 0;

            // 强制蜂鸣器状态机重新同步
            beep_mode_cur = 0xFF;
            beep_phase = 0;
            beep_phase_start = sysTick_ms;

            if (b1_beep_count >= B1_BEEP_MIN_TRIGGER && b1_beep_count <= B1_BEEP_MAX) {
                SelfTest_Run();
            }
        }
        return 0;
    }
}
/* =================================================== */

/* === SelfTest 新增：自检相关函数 === */

/*
 * 自检中检测取消：
 *   第一次短按按钮1 → 显示 "Press B1 confirm"，等 3 秒。
 *   3 秒内再按一次 → 返回 1，调用方执行取消。
 *   3 秒内没按 → 清提示，返回 0，自检继续。
 *   按钮按住超过 1 秒 → 视为误触，忽略，且本次按住不再触发确认。
 */
uint8_t SelfTest_CheckCancel(void) {
    static uint8_t suppress_until_release = 0;

    if (suppress_until_release) {
        if (Button1_IsDown()) return 0;
        suppress_until_release = 0;
        return 0;
    }

    if (!Button1_IsDown()) return 0;

    // 第一次按下，等待松开（最多 1 秒）
    uint32_t start = sysTick_ms;
    while (Button1_IsDown() && (sysTick_ms - start) < SELFTEST_SUPPRESS_MS) {
        delay_ms(10);
    }

    if (Button1_IsDown()) {
        // 按住超过 1 秒，视为误触
        suppress_until_release = 1;
        return 0;
    }

    // 在 1 秒内松开了 → 询问确认
    OLED_ShowString(0, 1, "Press B1 confirm");

    start = sysTick_ms;
    while ((sysTick_ms - start) < SELFTEST_CONFIRM_WINDOW_MS) {
        if (Button1_IsDown()) {
            while (Button1_IsDown()) delay_ms(10);
            OLED_ShowString(0, 1, "                ");
            return 1;
        }
        delay_ms(10);
    }

    // 超时未确认
    OLED_ShowString(0, 1, "                ");
    return 0;
}

void SelfTest_Cancel(void) {
    // 取消提示音
    Beeper_PlayPattern(BEEP_CANCEL_ON, 0, BEEP_CANCEL_CNT);

    // 舵机3复位到关仓位置
    Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
    servo3_state = 0;
	
    // 恢复姿态显示
    roll_angle = 0.0f;
    pitch_angle = 0.0f;
    gyro_rate_x_deg = 0.0f;
    gyro_rate_y_deg = 0.0f;

    // 强制蜂鸣器状态机重新同步
    beep_mode_cur = 0xFF;
    beep_phase = 0;
    beep_phase_start = sysTick_ms;

    OLED_Clear();
}

void SelfTest_Run(void) {
    // ---------- 前置条件：必须已开仓 ----------
    if (servo3_state != 1) {
        OLED_Clear();
        OLED_ShowString(0, 0, "Self Test");
        OLED_ShowString(0, 2, "Door is closed");
        OLED_ShowString(0, 3, "*Press Button2 to");
        OLED_ShowString(0, 4, " open it first");
        Beeper_PlayPattern(BEEP_NEED_OPEN_ON, BEEP_NEED_OPEN_OFF, BEEP_NEED_OPEN_CNT);
        delay_ms(SELFTEST_DOOR_CLOSED_MS);

        beep_mode_cur = 0xFF;
        beep_phase = 0;
        beep_phase_start = sysTick_ms;
        OLED_Clear();
        return;
    }

    // ---------- 开始信号 ----------
    Beeper_PlayPattern(BEEP_START_ON, BEEP_START_OFF, BEEP_START_CNT);

    OLED_Clear();
    OLED_ShowString(0, 0, "Self Test");
    OLED_ShowString(0, 1, "Hold still...");

    // 给用户 1.5 秒停手
    for (uint16_t i = 0; i < SELFTEST_HOLD_STILL_MS / 50; i++) {
        if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }
        delay_ms(50);
    }

    // 清掉静止提示，避免和 MPU 项重叠
    OLED_ShowString(0, 1, "                ");

    uint8_t fail_count = 0;
    char    fail_names[5][12];
    memset(fail_names, 0, sizeof(fail_names));

    int16_t ax, ay, az, gx, gy, gz;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    char buf[24];

    // ---------- 第1项：MPU6050 I2C / WHO_AM_I ----------
    OLED_ShowString(0, 2, "MPU6050: ...    ");
    uint8_t whoami = 0;
    I2C_Read(MPU6050_ADDR, 0x75, &whoami, 1);

    if (whoami == 0x00 || whoami == 0xFF) {
        OLED_ShowString(0, 2, "MPU6050: FAIL   ");
        Beeper_PlayPattern(BEEP_FAIL_ON, BEEP_FAIL_OFF, BEEP_FAIL_CNT);
        if (fail_count < 5) strcpy(fail_names[fail_count++], "MPU_I2C");
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    } else {
        sprintf(buf, "MPU6050: OK 0x%02X", (unsigned int)whoami);
        OLED_ShowString(0, 2, buf);
        Beeper_PlayPattern(BEEP_PASS_ON, 0, 1);
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    }

    // ---------- 第2项：加速度计模长 ----------
    if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }

    OLED_ShowString(0, 3, "Accel  : ...    ");
    sum_ax = sum_ay = sum_az = 0;
    sum_gx = sum_gy = sum_gz = 0;
    for (uint16_t i = 0; i < SELFTEST_SAMPLES; i++) {
        MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);
        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        delay_ms(2);
        if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }
    }
    float avg_ax = (float)sum_ax / SELFTEST_SAMPLES / 16384.0f;
    float avg_ay = (float)sum_ay / SELFTEST_SAMPLES / 16384.0f;
    float avg_az = (float)sum_az / SELFTEST_SAMPLES / 16384.0f;
    float accel_norm = sqrtf(avg_ax*avg_ax + avg_ay*avg_ay + avg_az*avg_az);

    if (accel_norm < SELFTEST_ACCEL_MIN_G || accel_norm > SELFTEST_ACCEL_MAX_G) {
        sprintf(buf, "Accel  : FAIL %.2fg", accel_norm);
        OLED_ShowString(0, 3, buf);
        Beeper_PlayPattern(BEEP_FAIL_ON, BEEP_FAIL_OFF, BEEP_FAIL_CNT);
        if (fail_count < 5) strcpy(fail_names[fail_count++], "ACC_NORM");
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    } else {
        sprintf(buf, "Accel  : OK %.2fg  ", accel_norm);
        OLED_ShowString(0, 3, buf);
        Beeper_PlayPattern(BEEP_PASS_ON, 0, 1);
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    }

    // ---------- 第3项：陀螺仪零偏 ----------
    if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }

    OLED_ShowString(0, 4, "Gyro   : ...    ");
    float avg_gx = (float)sum_gx / SELFTEST_SAMPLES / 131.0f;
    float avg_gy = (float)sum_gy / SELFTEST_SAMPLES / 131.0f;
    float avg_gz = (float)sum_gz / SELFTEST_SAMPLES / 131.0f;
    float gyro_norm = sqrtf(avg_gx*avg_gx + avg_gy*avg_gy + avg_gz*avg_gz);

    if (gyro_norm > SELFTEST_GYRO_BIAS_LIMIT) {
        // 超限，执行一次 MPU 校准
        OLED_ShowString(0, 4, "Gyro   : CAL... ");
        MPU6050_Calibrate();
        if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }

        // 重新采样
        sum_gx = sum_gy = sum_gz = 0;
        for (uint16_t i = 0; i < SELFTEST_SAMPLES; i++) {
            MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);
            sum_gx += gx; sum_gy += gy; sum_gz += gz;
            delay_ms(2);
            if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }
        }
        avg_gx = (float)sum_gx / SELFTEST_SAMPLES / 131.0f;
        avg_gy = (float)sum_gy / SELFTEST_SAMPLES / 131.0f;
        avg_gz = (float)sum_gz / SELFTEST_SAMPLES / 131.0f;
        gyro_norm = sqrtf(avg_gx*avg_gx + avg_gy*avg_gy + avg_gz*avg_gz);
    }

    if (gyro_norm > SELFTEST_GYRO_BIAS_LIMIT) {
        OLED_ShowString(0, 4, "Gyro   : FAIL   ");
        Beeper_PlayPattern(BEEP_FAIL_ON, BEEP_FAIL_OFF, BEEP_FAIL_CNT);
        if (fail_count < 5) strcpy(fail_names[fail_count++], "GYRO_BIAS");
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    } else {
        OLED_ShowString(0, 4, "Gyro   : OK     ");
        Beeper_PlayPattern(BEEP_PASS_ON, 0, 1);
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    }

    // ---------- 第4项：按钮卡死检测 ----------
    if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }

    OLED_ShowString(0, 5, "Button : ...    ");
    uint8_t btn1_stuck = (GPIO_ReadInputDataBit(BUTTON1_GPIO_PORT, BUTTON1_GPIO_PIN) == 0);
    uint8_t btn2_stuck = (GPIO_ReadInputDataBit(BUTTON2_GPIO_PORT, BUTTON2_GPIO_PIN) == 0);
    uint8_t btn3_stuck = (GPIO_ReadInputDataBit(BUTTON_GPIO_PORT,  BUTTON_GPIO_PIN)  == 0);

    if (btn1_stuck || btn2_stuck || btn3_stuck) {
        sprintf(buf, "Button : STUCK %d%d%d", (int)btn1_stuck, (int)btn2_stuck, (int)btn3_stuck);
        OLED_ShowString(0, 5, buf);
        Beeper_PlayPattern(BEEP_FAIL_ON, BEEP_FAIL_OFF, BEEP_FAIL_CNT);
        if (fail_count < 5) strcpy(fail_names[fail_count++], "BTN_STUCK");
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    } else {
        OLED_ShowString(0, 5, "Button : OK     ");
        Beeper_PlayPattern(BEEP_PASS_ON, 0, 1);
        delay_ms(SELFTEST_ITEM_RESULT_MS);
    }

    // ---------- 第5项：舵机3小幅抖动 ----------
    if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }

    OLED_ShowString(0, 6, "Servo3 : ...    ");
    for (uint8_t i = 0; i < 3; i++) {
        Servo3_SetAngle(SERVO3_ANGLE_OPEN + 5.0f);
        delay_ms(80);
        if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }
        Servo3_SetAngle(SERVO3_ANGLE_OPEN - 5.0f);
        delay_ms(80);
        if (SelfTest_CheckCancel()) { SelfTest_Cancel(); return; }
    }
    Servo3_SetAngle(SERVO3_ANGLE_OPEN);   // 回到开仓位置
    servo3_state = 1;
    OLED_ShowString(0, 6, "Servo3 : OK     ");
    Beeper_PlayPattern(BEEP_PASS_ON, 0, 1);
    delay_ms(SELFTEST_ITEM_RESULT_MS);

    // ---------- 汇总 ----------
    if (fail_count == 0) {
        OLED_ShowString(0, 7, "ALL PASS");
        Beeper_PlayPattern(BEEP_END_OK_ON, BEEP_END_OK_OFF, BEEP_END_OK_CNT);

        // 自检通过后清零飞行状态，回关仓待发射
        FlightState_Reset();
        Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
        servo3_state = 0;

        delay_ms(SELFTEST_END_PASS_MS);
    } else {
        sprintf(buf, "FAIL: %s", fail_names[0]);
        OLED_ShowString(0, 7, buf);
        Beeper_PlayPattern(BEEP_END_NG_ON, BEEP_END_NG_OFF, BEEP_END_NG_CNT);
        delay_ms(SELFTEST_END_FAIL_MS);
		
		// 自检失败也复位舵机3
        Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
        servo3_state = 0;
    }

    // 恢复蜂鸣器状态机
    beep_mode_cur = 0xFF;
    beep_phase = 0;
    beep_phase_start = sysTick_ms;

    OLED_Clear();
}

// ==================== 系统初始化 ====================
void System_Init(void) {
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    if (RCC_WaitForHSEStartUp() == SUCCESS) {
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);
    }
    SysTick_Config(72000);
    NVIC_SetPriority(SysTick_IRQn, 0x00);
}

void SysTick_Handler(void) { sysTick_ms++; }

// ==================== 主函数 ====================
int main(void) {
    System_Init();
    OLED_Init();
    OLED_ShowString(0, 0, DISPLAY_ROCKET_NAME);

    MPU6050_Init();
    Servo_Init();
    Button_Init();
    Servo3_Init();
    Button2_Init();
    Button1_Init();       /* === SelfTest 新增 === */
    Beeper_Init();

    OLED_ShowString(0, 4, DISPLAY_CALIB_MSG);
    MPU6050_Calibrate();

    roll_angle = 0.0f; pitch_angle = 0.0f;
    gyro_rate_x_deg = 0.0f; gyro_rate_y_deg = 0.0f;
    tilt_triggered = 0;

#if CONTROL_MODE == 2
    neuro_wp_r = initial_gain; neuro_wd_r = initial_deriv; neuro_wi_r = 0.0f;
    neuro_wp_p = initial_gain; neuro_wd_p = initial_deriv; neuro_wi_p = 0.0f;
    NeuroPID_Reset();
#endif

    OLED_ShowString(0, 4, DISPLAY_CALIB_DONE);

    uint32_t last_update = sysTick_ms;
    uint32_t last_display = sysTick_ms;

    while (1) {
        /* === SelfTest 新增：按钮1按住蜂鸣、松手触发自检 === */
        uint8_t b1_busy = Button1_Process();
        /* ================================================= */

        if (sysTick_ms - last_update >= 10) {
            last_update = sysTick_ms;
            Attitude_Update();

            if (servo3_state == 1) {
                Servo_SetAngle(1, 0.0f);
                Servo_SetAngle(2, 0.0f);
            } else {
#if CONTROL_MODE == 0
                Servo_SetAngle(1, 0.0f);
                Servo_SetAngle(2, 0.0f);
#elif CONTROL_MODE == 1
                float u_r = -roll_angle * initial_gain;
                float u_p = -pitch_angle * initial_gain;
                if (u_r > servo_angle_limit) u_r = servo_angle_limit;
                if (u_r < -servo_angle_limit) u_r = -servo_angle_limit;
                if (u_p > servo_angle_limit) u_p = servo_angle_limit;
                if (u_p < -servo_angle_limit) u_p = -servo_angle_limit;
                Servo_SetAngle(1, u_r);
                Servo_SetAngle(2, u_p);
#else // CONTROL_MODE == 2
                float e_r = -roll_angle;
                float e_p = -pitch_angle;
                float de_r = e_r - neuro_prev_e_r;
                float de_p = e_p - neuro_prev_e_p;

                if ((e_r * neuro_prev_e_r) < 0.0f) { neuro_int_r = 0.0f; neuro_prev_e_r = 0.0f; }
                if ((e_p * neuro_prev_e_p) < 0.0f) { neuro_int_p = 0.0f; neuro_prev_e_p = 0.0f; }

                neuro_int_r += e_r * dt;
                neuro_int_p += e_p * dt;
                if (neuro_int_r > NEURO_INTEGRAL_LIMIT) neuro_int_r = NEURO_INTEGRAL_LIMIT;
                if (neuro_int_r < -NEURO_INTEGRAL_LIMIT) neuro_int_r = -NEURO_INTEGRAL_LIMIT;
                if (neuro_int_p > NEURO_INTEGRAL_LIMIT) neuro_int_p = NEURO_INTEGRAL_LIMIT;
                if (neuro_int_p < -NEURO_INTEGRAL_LIMIT) neuro_int_p = -NEURO_INTEGRAL_LIMIT;

                float u_r = neuro_wp_r * e_r + neuro_wi_r * neuro_int_r + neuro_wd_r * de_r;
                float u_p = neuro_wp_p * e_p + neuro_wi_p * neuro_int_p + neuro_wd_p * de_p;
                if (u_r > servo_angle_limit) u_r = servo_angle_limit;
                if (u_r < -servo_angle_limit) u_r = -servo_angle_limit;
                if (u_p > servo_angle_limit) u_p = servo_angle_limit;
                if (u_p < -servo_angle_limit) u_p = -servo_angle_limit;

                float u_abs_r = neuro_prev_u_r > 0 ? neuro_prev_u_r : -neuro_prev_u_r;
                float u_abs_p = neuro_prev_u_p > 0 ? neuro_prev_u_p : -neuro_prev_u_p;
                float factor_r = e_r * u_abs_r * (2.0f * e_r - neuro_prev_e_r);
                float factor_p = e_p * u_abs_p * (2.0f * e_p - neuro_prev_e_p);

                neuro_wp_r += NEURO_LEARN_RATE_P_R * factor_r;
                neuro_wi_r += NEURO_LEARN_RATE_I_R * factor_r;
                neuro_wd_r += NEURO_LEARN_RATE_D_R * factor_r;
                neuro_wp_p += NEURO_LEARN_RATE_P_P * factor_p;
                neuro_wi_p += NEURO_LEARN_RATE_I_P * factor_p;
                neuro_wd_p += NEURO_LEARN_RATE_D_P * factor_p;

                if (neuro_wp_r > NEURO_WEIGHT_LIMIT) neuro_wp_r = NEURO_WEIGHT_LIMIT;
                else if (neuro_wp_r < -NEURO_WEIGHT_LIMIT) neuro_wp_r = -NEURO_WEIGHT_LIMIT;
                if (neuro_wi_r > NEURO_WEIGHT_LIMIT) neuro_wi_r = NEURO_WEIGHT_LIMIT;
                else if (neuro_wi_r < -NEURO_WEIGHT_LIMIT) neuro_wi_r = -NEURO_WEIGHT_LIMIT;
                if (neuro_wd_r > NEURO_WEIGHT_LIMIT) neuro_wd_r = NEURO_WEIGHT_LIMIT;
                else if (neuro_wd_r < -NEURO_WEIGHT_LIMIT) neuro_wd_r = -NEURO_WEIGHT_LIMIT;
                if (neuro_wp_p > NEURO_WEIGHT_LIMIT) neuro_wp_p = NEURO_WEIGHT_LIMIT;
                else if (neuro_wp_p < -NEURO_WEIGHT_LIMIT) neuro_wp_p = -NEURO_WEIGHT_LIMIT;
                if (neuro_wi_p > NEURO_WEIGHT_LIMIT) neuro_wi_p = NEURO_WEIGHT_LIMIT;
                else if (neuro_wi_p < -NEURO_WEIGHT_LIMIT) neuro_wi_p = -NEURO_WEIGHT_LIMIT;
                if (neuro_wd_p > NEURO_WEIGHT_LIMIT) neuro_wd_p = NEURO_WEIGHT_LIMIT;
                else if (neuro_wd_p < -NEURO_WEIGHT_LIMIT) neuro_wd_p = -NEURO_WEIGHT_LIMIT;

                neuro_prev_e_r = e_r;
                neuro_prev_e_p = e_p;
                neuro_prev_u_r = u_r;
                neuro_prev_u_p = u_p;

                Servo_SetAngle(1, u_r);
                Servo_SetAngle(2, u_p);
#endif
            }

            // 蜂鸣器统一状态机（按钮1按住时跳过，避免冲突）
            if (!b1_busy) {
                Beeper_Update();
            }
        }

        // OLED 刷新 200ms
        if (sysTick_ms - last_display >= 200) {
            last_display = sysTick_ms;
			OLED_ShowString(0, 0, DISPLAY_ROCKET_NAME);   // 防止被自检清屏后不恢复
            OLED_ShowString(0, 6, "R:");
            OLED_ShowFloat(12, 6, roll_angle, 1);
            OLED_ShowString(70, 6, "P:");
            OLED_ShowFloat(82, 6, pitch_angle, 1);

            char door_str[16];
            sprintf(door_str, "Door: %d", servo3_state);
            OLED_ShowString(0, 2, door_str);

            char flight_info[32];
            float display_accel = (max_up_accel > 100.0f || max_up_accel < 0.0f) ? 0.0f : max_up_accel;
            float display_height = (max_height > 5000.0f || max_height < 0.0f) ? 0.0f : max_height;
            sprintf(flight_info, "Amax:%.2f   Hmax:%.2f ", display_accel, display_height);
            OLED_ShowString(0, 7, flight_info);
        }

        // 校准按钮（按钮3）
        if (Button_IsPressed()) {
            OLED_Clear();
            OLED_ShowString(0, 0, DISPLAY_ROCKET_NAME);
            OLED_ShowString(0, 4, DISPLAY_CALIB_MSG);

            Servo_Reset();
            MPU6050_Calibrate();
            roll_angle = 0.0f; pitch_angle = 0.0f;
            gyro_rate_x_deg = 0.0f; gyro_rate_y_deg = 0.0f;
            tilt_triggered = 0;

            vertical_velocity = 0.0f; vertical_position = 0.0f;
            max_up_accel = 0.0f; max_height = 0.0f;
            launched = 0; apogee_reached = 0;

            Servo3_SetAngle(SERVO3_ANGLE_DEFAULT);
            servo3_state = 0;
            safety_deadline_ms = 0;

#if CONTROL_MODE == 2
            neuro_wp_r = initial_gain; neuro_wd_r = initial_deriv; neuro_wi_r = 0.0f;
            neuro_wp_p = initial_gain; neuro_wd_p = initial_deriv; neuro_wi_p = 0.0f;
            NeuroPID_Reset();
#endif

            Beeper_Off();
            beep_mode_cur = 0xFF;   // 强制下次 Beeper_Update 重新初始化

            OLED_ShowString(0, 4, DISPLAY_CALIB_DONE);
        }

        // 按钮2 手动开仓
        if (Button2_IsPressed()) {
            Servo3_Toggle();
            Beeper_Update();   // 立即切换蜂鸣器模式
        }
    }
}
