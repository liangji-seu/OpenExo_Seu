# SimpleDualMotor - Teensy 4.1 双CAN电机控制器

基于OpenExo项目简化的双电机控制代码，支持通过串口命令控制两个CAN总线电机。

## 硬件配置

- **微控制器**: Teensy 4.1
- **通信**: CAN总线 (1Mbps)
- **左电机CAN ID**: 65
- **右电机CAN ID**: 33
- **电机类型**: AK系列电机 (MIT mini cheetah协议)

## CAN接线

Teensy 4.1 CAN1 引脚：
- **CAN_TX**: Pin 22
- **CAN_RX**: Pin 23

需要CAN收发器 (如 MCP2562 或 SN65HVD230) 连接到电机。

```
Teensy 4.1          CAN Transceiver          Motor
Pin 22 (TX) -----> TXD          CANH -----> CANH
Pin 23 (RX) <----- RXD          CANL -----> CANL
3.3V ------------> VCC          GND ------> GND
GND -------------> GND
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `SimpleDualMotor.ino` | 基础版本，简洁易懂 |
| `SimpleDualMotor_Advanced.ino` | 增强版本，包含数据流、正弦测试等功能 |
| `motor_controller.py` | Python控制脚本 |

## 安装

### Arduino IDE 配置

1. 安装 Teensyduino：https://www.pjrc.com/teensy/td_download.html
2. 安装 FlexCAN_T4 库：
   - 打开 Arduino IDE
   - 工具 -> 管理库
   - 搜索 "FlexCAN_T4" 并安装

### 编译上传

1. 打开 `SimpleDualMotor.ino` 或 `SimpleDualMotor_Advanced.ino`
2. 选择开发板：Teensy 4.1
3. 选择串口
4. 点击上传

## 串口命令

### 基本控制

| 命令 | 说明 |
|------|------|
| `E` | 使能两个电机 |
| `E,L` / `E,R` | 单独使能左/右电机 |
| `D` | 禁用两个电机 |
| `D,L` / `D,R` | 单独禁用左/右电机 |
| `Z` | 位置归零 |
| `S` | 停止 (扭矩归零) |

### 扭矩控制

| 命令 | 说明 |
|------|------|
| `T,L,<val>` | 设置左电机扭矩 (Nm) |
| `T,R,<val>` | 设置右电机扭矩 (Nm) |
| `T,B,<l>,<r>` | 同时设置两个电机扭矩 |

### 位置控制

| 命令 | 说明 |
|------|------|
| `P,L,<val>` | 设置左电机位置 (rad) |
| `P,R,<val>` | 设置右电机位置 (rad) |
| `P,B,<l>,<r>` | 同时设置两个电机位置 |
| `G,<kp>,<kd>` | 设置位置控制增益 |

### 模式切换

| 命令 | 说明 |
|------|------|
| `M,T` | 切换到扭矩模式 |
| `M,P` | 切换到位置模式 |

### 高级功能 (仅增强版)

| 命令 | 说明 |
|------|------|
| `STREAM,ON` | 开启数据流输出 |
| `STREAM,OFF` | 关闭数据流输出 |
| `SINE,<amp>,<freq>` | 启动正弦波扭矩测试 |
| `SINE,OFF` | 停止正弦波测试 |
| `LIMIT,<val>` | 设置扭矩限制 (Nm) |

### 状态查询

| 命令 | 说明 |
|------|------|
| `R` | 读取当前状态 |
| `H` | 显示帮助信息 |

## 使用示例

### 串口终端使用

1. 打开串口终端 (115200 baud)
2. 发送命令：

```
# 使能电机
E

# 设置左电机扭矩 0.5 Nm
T,L,0.5

# 设置两个电机扭矩
T,B,1.0,-1.0

# 切换到位置模式
M,P

# 设置位置增益
G,100,2.0

# 移动左电机到 90度 (1.57 rad)
P,L,1.57

# 查看状态
R

# 停止
S

# 禁用电机
D
```

### Python脚本使用

```bash
# 安装依赖
pip install pyserial matplotlib

# 交互模式
python motor_controller.py

# 指定串口
python motor_controller.py --port COM3

# 列出可用串口
python motor_controller.py --list

# 实时绘图
python motor_controller.py --plot
```

Python 交互示例：
```python
>>> enable
>>> torque 0.5 0.5    # 两个电机各0.5 Nm
>>> status
>>> mode position
>>> gains 100 2
>>> pos 1.57 1.57
>>> stop
>>> disable
>>> quit
```

## 电机参数修改

如果使用不同型号的电机，需要修改代码中的参数：

```cpp
// AK60 参数
#define V_MAX   50.0f    // 最大速度 (rad/s)
#define I_MAX   15.0f    // 最大电流 (A)
#define KT      0.091f   // 扭矩常数 (Nm/A)

// AK70 参数
#define V_MAX   15.5f
#define I_MAX   23.2f
#define KT      1.3f     // 0.13 * 10 (减速比)

// AK80 参数
#define V_MAX   8.0f
#define I_MAX   25.0f
#define KT      0.819f   // 0.091 * 9 (减速比)
```

## 修改电机ID

修改代码开头的定义：

```cpp
#define LEFT_MOTOR_ID   65    // 左电机CAN ID
#define RIGHT_MOTOR_ID  33    // 右电机CAN ID
```

## 数据流格式

开启数据流后 (`STREAM,ON`)，输出CSV格式：

```
time,L_pos,L_vel,L_cur,L_cmd,R_pos,R_vel,R_cur,R_cmd
```

| 字段 | 说明 | 单位 |
|------|------|------|
| time | 时间戳 | ms |
| L_pos / R_pos | 位置 | rad |
| L_vel / R_vel | 速度 | rad/s |
| L_cur / R_cur | 电流 | A |
| L_cmd / R_cmd | 扭矩命令 | Nm |

## 故障排除

### 电机无响应

1. 检查CAN接线是否正确
2. 确认CAN总线波特率 (默认1Mbps)
3. 确认电机ID设置正确
4. 检查电机是否已上电

### 通信超时

- 状态显示 `[Disconnected]` 表示通信异常
- 检查CAN收发器连接
- 确认终端电阻 (120Ω)

### 扭矩不响应

1. 确认电机已使能 (`E` 命令)
2. 确认在扭矩模式 (`M,T`)
3. 检查扭矩限制设置 (`LIMIT,<val>`)

## 安全注意事项

⚠️ **警告**：电机可能产生较大扭矩，请确保：

1. 测试时电机固定牢固
2. 设置合理的扭矩限制
3. 紧急情况下可发送 `D` 立即禁用电机
4. 首次测试使用小扭矩值 (< 0.5 Nm)

## License

基于 OpenExo 项目修改，仅供学习和研究使用。
