#include <FlexCAN_T4.h>

// ==================== 配置参数 ====================
#define LEFT_MOTOR_ID   65  // 0x41
#define RIGHT_MOTOR_ID  33  // 0x21
#define CAN_BAUD        1000000

#define CONTROL_FREQ_HZ     200
#define CONTROL_PERIOD_US   (1000000 / CONTROL_FREQ_HZ)

// ==================== 物理量映射范围 (AK60-6 v1.1) ====================
namespace MotorParams {
    constexpr float P_MIN = -12.5f;
    constexpr float P_MAX = 12.5f;
    constexpr float V_MIN = -25.0f;   // 233 rpm output ≈ 24.4 rad/s, rounded up
    constexpr float V_MAX = 25.0f;
    constexpr float T_MIN = -9.0f;    // AK60-6 v1.1 KV80 peak torque
    constexpr float T_MAX = 9.0f;
    constexpr float KP_MIN = 0.0f;
    constexpr float KP_MAX = 500.0f;
    constexpr float KD_MIN = 0.0f;
    constexpr float KD_MAX = 5.0f;
}

// ==================== 通信协议结构 ====================
#define HEAD_CONTROL    0xAA
#define HEAD_STATUS     0xCC
#define HEAD_ENABLE     0xBB  // enable/disable command from PC
#define FRAME_TAIL      0x55

#pragma pack(push, 1)
struct ControlFrame {
    uint8_t  head;
    uint16_t seq;
    float    left_torque;
    float    right_torque;
    float    torque_limit;
    uint8_t  crc8;
    uint8_t  tail;
};

struct EnableFrame {
    uint8_t  head;
    uint8_t  cmd;   // 0xFC = enable, 0xFD = disable
    uint8_t  crc8;
    uint8_t  tail;
};

struct StatusFrame {
    uint8_t  head;
    uint16_t seq;
    uint8_t  state;
    uint8_t  error_code;
    float    left_pos;
    float    left_vel;
    float    left_torque;
    float    right_pos;
    float    right_vel;
    float    right_torque;
    uint32_t teensy_time_us;
    uint8_t  crc8;
    uint8_t  tail;
};
#pragma pack(pop)

// ==================== 全局变量 ====================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> can1;
IntervalTimer controlTimer;

struct MotorState {
    uint8_t id;
    volatile float pos, vel, torque;
    float cmd_torque;
    bool enabled = false;
};

MotorState leftMotor, rightMotor;

volatile float shared_left_cmd  = 0;
volatile float shared_right_cmd = 0;
volatile float shared_limit     = 5.0f;
volatile uint16_t shared_seq    = 0;

// ISR -> loop() 状态帧传递
volatile bool statusReady = false;
StatusFrame pendingFrame;

// ==================== 转换工具函数 ====================
unsigned int floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

uint8_t calcCRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
        }
    }
    return crc;
}

// ==================== 电机使能/禁用 ====================
void enableMotor(uint8_t id) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = 8;
    for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
    msg.buf[7] = 0xFC;
    can1.write(msg);
}

void disableMotor(uint8_t id) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = 8;
    for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
    msg.buf[7] = 0xFD;
    can1.write(msg);
}

// ==================== CAN 通信子函数 ====================
void sendMITCommand(uint8_t id, float t_ff, float limit) {
    t_ff = constrain(t_ff, -limit, limit);

    uint16_t p_int  = floatToUint(0, MotorParams::P_MIN, MotorParams::P_MAX, 16);
    uint16_t v_int  = floatToUint(0, MotorParams::V_MIN, MotorParams::V_MAX, 12);
    uint16_t kp_int = 0;
    uint16_t kd_int = 0;
    uint16_t t_int  = floatToUint(t_ff, MotorParams::T_MIN, MotorParams::T_MAX, 12);

    CAN_message_t msg;
    msg.id  = id;
    msg.len = 8;
    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;
    msg.buf[2] = v_int >> 4;
    msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.buf[4] = kp_int & 0xFF;
    msg.buf[5] = kd_int >> 4;
    msg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg.buf[7] = t_int & 0xFF;
    can1.write(msg);
}

void receiveCAN() {
    CAN_message_t msg;
    while (can1.read(msg)) {
        MotorState* m = nullptr;
        if      (msg.id == LEFT_MOTOR_ID)  m = &leftMotor;
        else if (msg.id == RIGHT_MOTOR_ID) m = &rightMotor;

        if (m && msg.len >= 6) {
            uint16_t p_int = ((uint16_t)msg.buf[1] << 8) | msg.buf[2];
            uint16_t v_int = ((uint16_t)msg.buf[3] << 4) | (msg.buf[4] >> 4);
            uint16_t t_int = (((uint16_t)msg.buf[4] & 0xF) << 8) | msg.buf[5];

            m->pos    = uintToFloat(p_int, MotorParams::P_MIN, MotorParams::P_MAX, 16);
            m->vel    = uintToFloat(v_int, MotorParams::V_MIN, MotorParams::V_MAX, 12);
            m->torque = uintToFloat(t_int, MotorParams::T_MIN, MotorParams::T_MAX, 12);
        }
    }
}

// ==================== 核心中断函数 (200Hz) ====================
void onControlTimer() {
    float l_torque = shared_left_cmd;
    float r_torque = shared_right_cmd;
    float limit    = shared_limit;

    sendMITCommand(LEFT_MOTOR_ID,  l_torque, limit);
    sendMITCommand(RIGHT_MOTOR_ID, r_torque, limit);

    // 等待CAN回复 (两帧 ~260us + 电机处理余量)
    delayMicroseconds(700);

    receiveCAN();

    // 准备状态帧，由 loop() 发送，避免在ISR中阻塞USB
    if (!statusReady) {
        pendingFrame.head           = HEAD_STATUS;
        pendingFrame.seq            = shared_seq;
        pendingFrame.state          = 2;
        pendingFrame.error_code     = 0;
        pendingFrame.left_pos       = leftMotor.pos;
        pendingFrame.left_vel       = leftMotor.vel;
        pendingFrame.left_torque    = leftMotor.torque;
        pendingFrame.right_pos      = rightMotor.pos;
        pendingFrame.right_vel      = rightMotor.vel;
        pendingFrame.right_torque   = rightMotor.torque;
        pendingFrame.teensy_time_us = micros();
        pendingFrame.crc8           = calcCRC8((uint8_t*)&pendingFrame.seq, sizeof(StatusFrame) - 3);
        pendingFrame.tail           = FRAME_TAIL;
        statusReady = true;
    }
}

// ==================== 主程序入口 ====================
void setup() {
    Serial.begin(9600);  // Teensy USB 忽略波特率参数
    can1.begin();
    can1.setBaudRate(CAN_BAUD);

    leftMotor.id  = LEFT_MOTOR_ID;
    rightMotor.id = RIGHT_MOTOR_ID;

    delay(500);

    // 使能两个电机进入MIT模式
    enableMotor(LEFT_MOTOR_ID);
    delay(50);
    enableMotor(RIGHT_MOTOR_ID);
    delay(50);

    controlTimer.begin(onControlTimer, CONTROL_PERIOD_US);
}

void loop() {
    // 发送已准备好的状态帧
    if (statusReady) {
        Serial.write((uint8_t*)&pendingFrame, sizeof(StatusFrame));
        statusReady = false;
    }

    // 接收上位机指令
    while (Serial.available() > 0) {
        uint8_t header = Serial.peek();

        if (header == HEAD_CONTROL && Serial.available() >= (int)sizeof(ControlFrame)) {
            ControlFrame cf;
            Serial.readBytes((uint8_t*)&cf, sizeof(ControlFrame));
            if (cf.tail == FRAME_TAIL &&
                calcCRC8((uint8_t*)&cf.seq, sizeof(ControlFrame) - 3) == cf.crc8) {
                noInterrupts();
                shared_left_cmd  = cf.left_torque;
                shared_right_cmd = cf.right_torque;
                shared_limit     = cf.torque_limit;
                shared_seq       = cf.seq;
                interrupts();
            }
        } else if (header == HEAD_ENABLE && Serial.available() >= (int)sizeof(EnableFrame)) {
            EnableFrame ef;
            Serial.readBytes((uint8_t*)&ef, sizeof(EnableFrame));
            if (ef.tail == FRAME_TAIL &&
                calcCRC8(&ef.cmd, 1) == ef.crc8) {
                if (ef.cmd == 0xFC) {
                    enableMotor(LEFT_MOTOR_ID);
                    delay(10);
                    enableMotor(RIGHT_MOTOR_ID);
                } else if (ef.cmd == 0xFD) {
                    // 先清零力矩再禁用
                    noInterrupts();
                    shared_left_cmd  = 0;
                    shared_right_cmd = 0;
                    interrupts();
                    delay(20);
                    disableMotor(LEFT_MOTOR_ID);
                    delay(10);
                    disableMotor(RIGHT_MOTOR_ID);
                }
            }
        } else {
            Serial.read(); // 丢弃错位字节
        }
    }
}
