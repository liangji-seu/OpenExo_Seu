/*
 * SimpleDualMotor_WithLED.ino
 * 
 * 带LED状态指示的双CAN电机控制器 - Teensy 4.1
 * 
 * LED状态指示：
 *   - 系统禁用：蓝色常亮
 *   - 扭矩模式：绿色呼吸
 *   - 位置模式：青色呼吸
 *   - 正弦测试：彩虹渐变
 *   - 电机超时/错误：红色闪烁
 *   - 数据流模式：紫色常亮
 * 
 * 串口命令同 SimpleDualMotor_Advanced.ino
 */

#include <FlexCAN_T4.h>

// ==================== 配置参数 ====================
#define LEFT_MOTOR_ID   65
#define RIGHT_MOTOR_ID  33

#define SERIAL_BAUD     115200
#define CAN_BAUD        1000000

#define LOOP_FREQ_HZ    500
#define LOOP_PERIOD_US  (1000000 / LOOP_FREQ_HZ)

#define STREAM_FREQ_HZ  50
#define STREAM_PERIOD_MS (1000 / STREAM_FREQ_HZ)

// ==================== LED 引脚配置 ====================
// OpenExo Board Teensy 4.1 引脚定义
#define LED_R_PIN       14    // 红色LED引脚
#define LED_G_PIN       25    // 绿色LED引脚  
#define LED_B_PIN       24    // 蓝色LED引脚
#define LED_ACTIVE_LOW  true  // true: 低电平点亮 (OpenExo板子是低电平点亮)

// 如果只用Teensy板载LED（单色）
#define USE_BUILTIN_LED false
#define BUILTIN_LED_PIN 13

// ==================== 电机参数 ====================
namespace MotorParams {
    constexpr float P_MAX = 12.5f;
    constexpr float V_MAX = 50.0f;
    constexpr float I_MAX = 25.0f;
    constexpr float KP_MIN = 0.0f;
    constexpr float KP_MAX = 500.0f;
    constexpr float KD_MIN = 0.0f;
    constexpr float KD_MAX = 5.0f;
    constexpr float KT = 0.091f;
    constexpr float TORQUE_LIMIT_DEFAULT = 5.0f;
}

// ==================== 系统状态枚举 ====================
enum SystemStatus {
    STATUS_DISABLED,        // 系统禁用 - 蓝色常亮
    STATUS_TORQUE_MODE,     // 扭矩模式 - 绿色呼吸
    STATUS_POSITION_MODE,   // 位置模式 - 青色呼吸
    STATUS_SINE_TEST,       // 正弦测试 - 彩虹
    STATUS_ERROR,           // 错误 - 红色闪烁
    STATUS_STREAMING        // 数据流 - 紫色常亮
};

enum ControlMode {
    MODE_TORQUE,
    MODE_POSITION
};

// ==================== LED模式枚举 ====================
enum LedPattern {
    PATTERN_SOLID,
    PATTERN_BLINK,
    PATTERN_PULSE,
    PATTERN_RAINBOW
};

// ==================== 电机数据结构 ====================
struct MotorState {
    uint8_t id;
    bool enabled;
    float position;
    float velocity;
    float current;
    float torque_cmd;
    float position_cmd;
    float kp;
    float kd;
    int timeout_count;
    bool connected;
};

// ==================== LED控制类 ====================
class StatusLed {
public:
    StatusLed(int r_pin, int g_pin, int b_pin, bool active_low = false) {
        _r_pin = r_pin;
        _g_pin = g_pin;
        _b_pin = b_pin;
        _active_low = active_low;
        _brightness = 255;
        _pattern_start = 0;
        
        pinMode(_r_pin, OUTPUT);
        pinMode(_g_pin, OUTPUT);
        pinMode(_b_pin, OUTPUT);
        
        setColor(0, 0, 0);
    }
    
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        // 应用亮度
        r = (r * _brightness) / 255;
        g = (g * _brightness) / 255;
        b = (b * _brightness) / 255;
        
        if (_active_low) {
            analogWrite(_r_pin, 255 - r);
            analogWrite(_g_pin, 255 - g);
            analogWrite(_b_pin, 255 - b);
        } else {
            analogWrite(_r_pin, r);
            analogWrite(_g_pin, g);
            analogWrite(_b_pin, b);
        }
    }
    
    void setBrightness(uint8_t brightness) {
        _brightness = brightness;
    }
    
    void update(SystemStatus status) {
        unsigned long now = millis();
        
        switch (status) {
            case STATUS_DISABLED:
                // 蓝色常亮
                setColor(0, 0, 255);
                break;
                
            case STATUS_TORQUE_MODE:
                // 绿色呼吸
                pulse(0, 255, 0, 1000, now);
                break;
                
            case STATUS_POSITION_MODE:
                // 青色呼吸
                pulse(0, 255, 255, 1000, now);
                break;
                
            case STATUS_SINE_TEST:
                // 彩虹渐变
                rainbow(4000, now);
                break;
                
            case STATUS_ERROR:
                // 红色闪烁
                blink(255, 0, 0, 250, now);
                break;
                
            case STATUS_STREAMING:
                // 紫色常亮
                setColor(255, 0, 255);
                break;
        }
    }
    
private:
    int _r_pin, _g_pin, _b_pin;
    bool _active_low;
    uint8_t _brightness;
    unsigned long _pattern_start;
    
    void pulse(uint8_t r, uint8_t g, uint8_t b, int period_ms, unsigned long now) {
        float angle = (2.0f * PI * (now % period_ms)) / period_ms;
        float factor = (sin(angle) + 1.0f) / 2.0f;  // 0 到 1
        setColor(r * factor, g * factor, b * factor);
    }
    
    void blink(uint8_t r, uint8_t g, uint8_t b, int period_ms, unsigned long now) {
        bool on = ((now / period_ms) % 2) == 0;
        if (on) {
            setColor(r, g, b);
        } else {
            setColor(0, 0, 0);
        }
    }
    
    void rainbow(int period_ms, unsigned long now) {
        float angle = (360.0f * (now % period_ms)) / period_ms;
        
        // HSV to RGB (简化版，S=1, V=1)
        int h_i = (int)(angle / 60) % 6;
        float f = angle / 60.0f - (int)(angle / 60);
        uint8_t q = 255 * (1 - f);
        uint8_t t = 255 * f;
        
        switch (h_i) {
            case 0: setColor(255, t, 0); break;
            case 1: setColor(q, 255, 0); break;
            case 2: setColor(0, 255, t); break;
            case 3: setColor(0, q, 255); break;
            case 4: setColor(t, 0, 255); break;
            case 5: setColor(255, 0, q); break;
        }
    }
};

// ==================== 全局变量 ====================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

MotorState leftMotor;
MotorState rightMotor;
ControlMode currentMode = MODE_TORQUE;
SystemStatus systemStatus = STATUS_DISABLED;

#if USE_BUILTIN_LED
    // 仅使用板载LED
#else
    StatusLed statusLed(LED_R_PIN, LED_G_PIN, LED_B_PIN, LED_ACTIVE_LOW);
#endif

String inputBuffer = "";
unsigned long lastLoopTime = 0;
unsigned long lastStreamTime = 0;
unsigned long lastLedUpdate = 0;
bool systemEnabled = false;
bool streamEnabled = false;
float torqueLimit = MotorParams::TORQUE_LIMIT_DEFAULT;

bool sineTestActive = false;
float sineAmplitude = 0;
float sineFrequency = 0;
unsigned long sineStartTime = 0;

bool hasError = false;

// ==================== 函数声明 ====================
void initMotors();
void enableMotor(MotorState* motor);
void disableMotor(MotorState* motor);
void zeroMotor(MotorState* motor);
void sendMotorCommand(MotorState* motor);
bool readMotorResponse(MotorState* motor);
void processSerialCommand(String cmd);
void updateSystemStatus();
void printStatus();
void printHelp();
void streamData();
float floatToUint(float x, float x_min, float x_max, int bits);
float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits);

// ==================== 初始化 ====================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    
    #if USE_BUILTIN_LED
        pinMode(BUILTIN_LED_PIN, OUTPUT);
    #endif
    
    Serial.println("\n==========================================");
    Serial.println("  SimpleDualMotor with LED - Teensy 4.1");
    Serial.println("  Left Motor ID:  " + String(LEFT_MOTOR_ID));
    Serial.println("  Right Motor ID: " + String(RIGHT_MOTOR_ID));
    Serial.println("==========================================");
    
    // 初始化CAN总线
    can1.begin();
    can1.setBaudRate(CAN_BAUD);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    
    Serial.println("[OK] CAN bus initialized");
    
    // 初始化电机状态
    initMotors();
    
    Serial.println("[OK] Motors initialized");
    Serial.println("[OK] LED initialized");
    Serial.println("\nLED Status Guide:");
    Serial.println("  Blue solid    = Disabled");
    Serial.println("  Green pulse   = Torque mode");
    Serial.println("  Cyan pulse    = Position mode");
    Serial.println("  Rainbow       = Sine test");
    Serial.println("  Red blink     = Error/Timeout");
    Serial.println("  Purple solid  = Streaming");
    Serial.println("\nType 'H' for help\n");
    
    lastLoopTime = micros();
    lastStreamTime = millis();
    lastLedUpdate = millis();
}

void initMotors() {
    leftMotor.id = LEFT_MOTOR_ID;
    leftMotor.enabled = false;
    leftMotor.position = 0;
    leftMotor.velocity = 0;
    leftMotor.current = 0;
    leftMotor.torque_cmd = 0;
    leftMotor.position_cmd = 0;
    leftMotor.kp = 0;
    leftMotor.kd = 0;
    leftMotor.timeout_count = 0;
    leftMotor.connected = false;
    
    rightMotor.id = RIGHT_MOTOR_ID;
    rightMotor.enabled = false;
    rightMotor.position = 0;
    rightMotor.velocity = 0;
    rightMotor.current = 0;
    rightMotor.torque_cmd = 0;
    rightMotor.position_cmd = 0;
    rightMotor.kp = 0;
    rightMotor.kd = 0;
    rightMotor.timeout_count = 0;
    rightMotor.connected = false;
}

// ==================== 主循环 ====================
void loop() {
    // 处理串口输入
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                inputBuffer.trim();
                inputBuffer.toUpperCase();
                processSerialCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
    
    // 定时执行电机通信
    unsigned long now = micros();
    if (now - lastLoopTime >= LOOP_PERIOD_US) {
        lastLoopTime = now;
        
        // 正弦波测试
        if (sineTestActive && currentMode == MODE_TORQUE) {
            float t = (millis() - sineStartTime) / 1000.0f;
            float torque = sineAmplitude * sin(2.0f * PI * sineFrequency * t);
            leftMotor.torque_cmd = torque;
            rightMotor.torque_cmd = torque;
        }
        
        if (systemEnabled) {
            if (leftMotor.enabled) {
                sendMotorCommand(&leftMotor);
                leftMotor.connected = readMotorResponse(&leftMotor);
            }
            if (rightMotor.enabled) {
                sendMotorCommand(&rightMotor);
                rightMotor.connected = readMotorResponse(&rightMotor);
            }
        }
        
        // 检查错误状态
        hasError = (leftMotor.enabled && leftMotor.timeout_count > 50) ||
                   (rightMotor.enabled && rightMotor.timeout_count > 50);
    }
    
    // 更新LED状态 (每20ms更新一次)
    if (millis() - lastLedUpdate >= 20) {
        lastLedUpdate = millis();
        updateSystemStatus();
        
        #if USE_BUILTIN_LED
            // 简单模式：板载LED
            digitalWrite(BUILTIN_LED_PIN, systemEnabled ? HIGH : LOW);
        #else
            // RGB LED模式
            statusLed.update(systemStatus);
        #endif
    }
    
    // 数据流输出
    if (streamEnabled && (millis() - lastStreamTime >= STREAM_PERIOD_MS)) {
        lastStreamTime = millis();
        streamData();
    }
}

// ==================== 更新系统状态 ====================
void updateSystemStatus() {
    if (hasError) {
        systemStatus = STATUS_ERROR;
    } else if (!systemEnabled) {
        systemStatus = STATUS_DISABLED;
    } else if (sineTestActive) {
        systemStatus = STATUS_SINE_TEST;
    } else if (streamEnabled) {
        systemStatus = STATUS_STREAMING;
    } else if (currentMode == MODE_POSITION) {
        systemStatus = STATUS_POSITION_MODE;
    } else {
        systemStatus = STATUS_TORQUE_MODE;
    }
}

// ==================== 电机通信函数 ====================
void enableMotor(MotorState* motor) {
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;
    
    can1.write(msg);
    delay(1);
    can1.write(msg);
    
    motor->enabled = true;
    motor->timeout_count = 0;
    Serial.println("[ENABLE] Motor ID " + String(motor->id));
}

void disableMotor(MotorState* motor) {
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;
    
    can1.write(msg);
    
    motor->enabled = false;
    motor->torque_cmd = 0;
    motor->position_cmd = 0;
    motor->connected = false;
    Serial.println("[DISABLE] Motor ID " + String(motor->id));
}

void zeroMotor(MotorState* motor) {
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
    
    can1.write(msg);
    motor->position = 0;
    motor->position_cmd = 0;
    Serial.println("[ZERO] Motor ID " + String(motor->id));
}

void sendMotorCommand(MotorState* motor) {
    if (!motor->enabled) return;
    
    float p_des, v_des, kp, kd, i_ff;
    
    if (currentMode == MODE_POSITION) {
        p_des = constrain(motor->position_cmd, -MotorParams::P_MAX, MotorParams::P_MAX);
        v_des = 0;
        kp = constrain(motor->kp, MotorParams::KP_MIN, MotorParams::KP_MAX);
        kd = constrain(motor->kd, MotorParams::KD_MIN, MotorParams::KD_MAX);
        i_ff = 0;
    } else {
        p_des = 0;
        v_des = 0;
        kp = 0;
        kd = 0;
        float limited_torque = constrain(motor->torque_cmd, -torqueLimit, torqueLimit);
        i_ff = constrain(limited_torque / MotorParams::KT, -MotorParams::I_MAX, MotorParams::I_MAX);
    }
    
    uint32_t p_int = floatToUint(p_des, -MotorParams::P_MAX, MotorParams::P_MAX, 16);
    uint32_t v_int = floatToUint(v_des, -MotorParams::V_MAX, MotorParams::V_MAX, 12);
    uint32_t kp_int = floatToUint(kp, MotorParams::KP_MIN, MotorParams::KP_MAX, 12);
    uint32_t kd_int = floatToUint(kd, MotorParams::KD_MIN, MotorParams::KD_MAX, 12);
    uint32_t i_int = floatToUint(i_ff, -MotorParams::I_MAX, MotorParams::I_MAX, 12);
    
    CAN_message_t msg;
    msg.id = motor->id;
    msg.len = 8;
    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;
    msg.buf[2] = v_int >> 4;
    msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.buf[4] = kp_int & 0xFF;
    msg.buf[5] = kd_int >> 4;
    msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
    msg.buf[7] = i_int & 0xFF;
    
    can1.write(msg);
}

bool readMotorResponse(MotorState* motor) {
    CAN_message_t msg;
    unsigned long startTime = micros();
    const unsigned long timeout = 1000;
    
    while (micros() - startTime < timeout) {
        if (can1.read(msg)) {
            if (msg.buf[0] == motor->id) {
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
                
                motor->position = uintToFloat(p_int, -MotorParams::P_MAX, MotorParams::P_MAX, 16);
                motor->velocity = uintToFloat(v_int, -MotorParams::V_MAX, MotorParams::V_MAX, 12);
                motor->current = uintToFloat(i_int, -MotorParams::I_MAX, MotorParams::I_MAX, 12);
                motor->timeout_count = 0;
                
                return true;
            }
        }
    }
    
    motor->timeout_count++;
    return false;
}

// ==================== 数据转换函数 ====================
float floatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (x - offset) * ((float)((1 << bits) - 1)) / span;
}

float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ==================== 数据流输出 ====================
void streamData() {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(leftMotor.position, 4);
    Serial.print(",");
    Serial.print(leftMotor.velocity, 4);
    Serial.print(",");
    Serial.print(leftMotor.current, 4);
    Serial.print(",");
    Serial.print(leftMotor.torque_cmd, 4);
    Serial.print(",");
    Serial.print(rightMotor.position, 4);
    Serial.print(",");
    Serial.print(rightMotor.velocity, 4);
    Serial.print(",");
    Serial.print(rightMotor.current, 4);
    Serial.print(",");
    Serial.println(rightMotor.torque_cmd, 4);
}

// ==================== 串口命令处理 ====================
void processSerialCommand(String cmd) {
    if (cmd.length() == 0) return;
    
    // 多字命令
    if (cmd.startsWith("STREAM")) {
        if (cmd.indexOf("ON") > 0) {
            streamEnabled = true;
            Serial.println("[STREAM] ON");
        } else if (cmd.indexOf("OFF") > 0) {
            streamEnabled = false;
            Serial.println("[STREAM] OFF");
        }
        return;
    }
    
    if (cmd.startsWith("SINE")) {
        if (cmd.indexOf("OFF") > 0) {
            sineTestActive = false;
            leftMotor.torque_cmd = 0;
            rightMotor.torque_cmd = 0;
            Serial.println("[SINE] Stopped");
        } else {
            int idx1 = cmd.indexOf(',');
            int idx2 = cmd.indexOf(',', idx1 + 1);
            if (idx1 > 0 && idx2 > 0) {
                sineAmplitude = cmd.substring(idx1 + 1, idx2).toFloat();
                sineFrequency = cmd.substring(idx2 + 1).toFloat();
                sineStartTime = millis();
                sineTestActive = true;
                Serial.println("[SINE] Amp=" + String(sineAmplitude) + "Nm, Freq=" + String(sineFrequency) + "Hz");
            }
        }
        return;
    }
    
    if (cmd.startsWith("LIMIT")) {
        int idx = cmd.indexOf(',');
        if (idx > 0) {
            torqueLimit = cmd.substring(idx + 1).toFloat();
            Serial.println("[LIMIT] " + String(torqueLimit) + " Nm");
        }
        return;
    }
    
    if (cmd.startsWith("BRIGHT") || cmd.startsWith("LED")) {
        int idx = cmd.indexOf(',');
        if (idx > 0) {
            int brightness = cmd.substring(idx + 1).toInt();
            #if !USE_BUILTIN_LED
                statusLed.setBrightness(brightness);
            #endif
            Serial.println("[LED] Brightness=" + String(brightness));
        }
        return;
    }
    
    char firstChar = cmd.charAt(0);
    
    switch (firstChar) {
        case 'E':
            hasError = false;  // 清除错误状态
            if (cmd.indexOf(",L") > 0) {
                enableMotor(&leftMotor);
                systemEnabled = true;
            } else if (cmd.indexOf(",R") > 0) {
                enableMotor(&rightMotor);
                systemEnabled = true;
            } else {
                Serial.println("\n--- Enabling Both Motors ---");
                enableMotor(&leftMotor);
                enableMotor(&rightMotor);
                systemEnabled = true;
            }
            break;
            
        case 'D':
            sineTestActive = false;
            if (cmd.indexOf(",L") > 0) {
                disableMotor(&leftMotor);
            } else if (cmd.indexOf(",R") > 0) {
                disableMotor(&rightMotor);
            } else {
                Serial.println("\n--- Disabling Both Motors ---");
                disableMotor(&leftMotor);
                disableMotor(&rightMotor);
                systemEnabled = false;
            }
            break;
            
        case 'Z':
            Serial.println("\n--- Zeroing Motors ---");
            zeroMotor(&leftMotor);
            zeroMotor(&rightMotor);
            break;
            
        case 'S':
            Serial.println("\n--- Stop ---");
            sineTestActive = false;
            leftMotor.torque_cmd = 0;
            rightMotor.torque_cmd = 0;
            leftMotor.position_cmd = leftMotor.position;
            rightMotor.position_cmd = rightMotor.position;
            break;
            
        case 'T':
            processTorqueCommand(cmd);
            break;
            
        case 'P':
            processPositionCommand(cmd);
            break;
            
        case 'G':
            processGainCommand(cmd);
            break;
            
        case 'M':
            processModeCommand(cmd);
            break;
            
        case 'R':
            printStatus();
            break;
            
        case 'H':
        case '?':
            printHelp();
            break;
            
        case 'C':  // 清除错误
            hasError = false;
            leftMotor.timeout_count = 0;
            rightMotor.timeout_count = 0;
            Serial.println("[CLEAR] Error cleared");
            break;
            
        default:
            Serial.println("[ERROR] Unknown: " + cmd);
            break;
    }
}

void processTorqueCommand(String cmd) {
    int idx1 = cmd.indexOf(',');
    if (idx1 < 0) return;
    
    char side = cmd.charAt(idx1 + 1);
    int idx2 = cmd.indexOf(',', idx1 + 1);
    
    if (side == 'L' && idx2 > 0) {
        leftMotor.torque_cmd = cmd.substring(idx2 + 1).toFloat();
        Serial.println("[T] L=" + String(leftMotor.torque_cmd, 3) + "Nm");
    } 
    else if (side == 'R' && idx2 > 0) {
        rightMotor.torque_cmd = cmd.substring(idx2 + 1).toFloat();
        Serial.println("[T] R=" + String(rightMotor.torque_cmd, 3) + "Nm");
    }
    else if (side == 'B') {
        int idx3 = cmd.indexOf(',', idx2 + 1);
        leftMotor.torque_cmd = cmd.substring(idx2 + 1, idx3).toFloat();
        rightMotor.torque_cmd = cmd.substring(idx3 + 1).toFloat();
        Serial.println("[T] L=" + String(leftMotor.torque_cmd, 3) + ", R=" + String(rightMotor.torque_cmd, 3) + "Nm");
    }
}

void processPositionCommand(String cmd) {
    int idx1 = cmd.indexOf(',');
    if (idx1 < 0) return;
    
    char side = cmd.charAt(idx1 + 1);
    int idx2 = cmd.indexOf(',', idx1 + 1);
    
    if (side == 'L' && idx2 > 0) {
        leftMotor.position_cmd = cmd.substring(idx2 + 1).toFloat();
        Serial.println("[P] L=" + String(leftMotor.position_cmd, 3) + "rad");
    } 
    else if (side == 'R' && idx2 > 0) {
        rightMotor.position_cmd = cmd.substring(idx2 + 1).toFloat();
        Serial.println("[P] R=" + String(rightMotor.position_cmd, 3) + "rad");
    }
    else if (side == 'B') {
        int idx3 = cmd.indexOf(',', idx2 + 1);
        leftMotor.position_cmd = cmd.substring(idx2 + 1, idx3).toFloat();
        rightMotor.position_cmd = cmd.substring(idx3 + 1).toFloat();
        Serial.println("[P] L=" + String(leftMotor.position_cmd, 3) + ", R=" + String(rightMotor.position_cmd, 3) + "rad");
    }
}

void processGainCommand(String cmd) {
    int idx1 = cmd.indexOf(',');
    int idx2 = cmd.indexOf(',', idx1 + 1);
    
    if (idx1 > 0 && idx2 > 0) {
        float kp = cmd.substring(idx1 + 1, idx2).toFloat();
        float kd = cmd.substring(idx2 + 1).toFloat();
        
        leftMotor.kp = kp;
        leftMotor.kd = kd;
        rightMotor.kp = kp;
        rightMotor.kd = kd;
        
        Serial.println("[GAIN] Kp=" + String(kp, 2) + ", Kd=" + String(kd, 3));
    }
}

void processModeCommand(String cmd) {
    int idx = cmd.indexOf(',');
    if (idx < 0) return;
    
    char mode = cmd.charAt(idx + 1);
    if (mode == 'T') {
        currentMode = MODE_TORQUE;
        leftMotor.kp = 0;
        leftMotor.kd = 0;
        rightMotor.kp = 0;
        rightMotor.kd = 0;
        Serial.println("[MODE] Torque");
    } 
    else if (mode == 'P') {
        currentMode = MODE_POSITION;
        if (leftMotor.kp == 0) {
            leftMotor.kp = 50;
            leftMotor.kd = 1.0;
            rightMotor.kp = 50;
            rightMotor.kd = 1.0;
        }
        leftMotor.position_cmd = leftMotor.position;
        rightMotor.position_cmd = rightMotor.position;
        Serial.println("[MODE] Position (Kp=" + String(leftMotor.kp) + ", Kd=" + String(leftMotor.kd) + ")");
    }
}

void printStatus() {
    Serial.println("\n========== Status ==========");
    Serial.println("System: " + String(systemEnabled ? "ENABLED" : "DISABLED"));
    Serial.println("Mode: " + String(currentMode == MODE_TORQUE ? "TORQUE" : "POSITION"));
    Serial.println("Torque Limit: " + String(torqueLimit) + " Nm");
    Serial.println("Stream: " + String(streamEnabled ? "ON" : "OFF"));
    Serial.println("Sine Test: " + String(sineTestActive ? "ACTIVE" : "OFF"));
    Serial.println("Error: " + String(hasError ? "YES" : "NO"));
    
    const char* statusNames[] = {"DISABLED", "TORQUE", "POSITION", "SINE", "ERROR", "STREAMING"};
    Serial.println("LED Status: " + String(statusNames[systemStatus]));
    Serial.println();
    
    Serial.println("Left Motor (ID:" + String(leftMotor.id) + ") " + 
                   (leftMotor.connected ? "[OK]" : "[--]"));
    Serial.println("  Pos: " + String(leftMotor.position, 4) + " rad");
    Serial.println("  Vel: " + String(leftMotor.velocity, 4) + " rad/s");
    Serial.println("  Cur: " + String(leftMotor.current, 4) + " A");
    Serial.println("  Cmd: " + String(leftMotor.torque_cmd, 4) + " Nm");
    Serial.println();
    
    Serial.println("Right Motor (ID:" + String(rightMotor.id) + ") " + 
                   (rightMotor.connected ? "[OK]" : "[--]"));
    Serial.println("  Pos: " + String(rightMotor.position, 4) + " rad");
    Serial.println("  Vel: " + String(rightMotor.velocity, 4) + " rad/s");
    Serial.println("  Cur: " + String(rightMotor.current, 4) + " A");
    Serial.println("  Cmd: " + String(rightMotor.torque_cmd, 4) + " Nm");
    Serial.println("=============================\n");
}

void printHelp() {
    Serial.println("\n============ Commands ============");
    Serial.println("E / E,L / E,R  - Enable motors");
    Serial.println("D / D,L / D,R  - Disable motors");
    Serial.println("Z              - Zero position");
    Serial.println("S              - Stop");
    Serial.println("C              - Clear error");
    Serial.println();
    Serial.println("T,L,<v> / T,R,<v> / T,B,<l>,<r>  - Torque");
    Serial.println("P,L,<v> / P,R,<v> / P,B,<l>,<r>  - Position");
    Serial.println("G,<kp>,<kd>    - Set gains");
    Serial.println("M,T / M,P      - Mode switch");
    Serial.println();
    Serial.println("STREAM,ON/OFF  - Data streaming");
    Serial.println("SINE,<amp>,<f> - Sine test");
    Serial.println("SINE,OFF       - Stop sine");
    Serial.println("LIMIT,<val>    - Torque limit");
    Serial.println("LED,<0-255>    - LED brightness");
    Serial.println();
    Serial.println("R              - Status");
    Serial.println("H              - Help");
    Serial.println();
    Serial.println("LED Colors:");
    Serial.println("  Blue solid   = Disabled");
    Serial.println("  Green pulse  = Torque mode");
    Serial.println("  Cyan pulse   = Position mode");
    Serial.println("  Rainbow      = Sine test");
    Serial.println("  Red blink    = Error");
    Serial.println("  Purple       = Streaming");
    Serial.println("==================================\n");
}
