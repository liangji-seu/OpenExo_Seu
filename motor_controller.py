#!/usr/bin/env python3
"""
motor_controller.py

Teensy 4.1 双电机控制器 Python 接口
通过串口发送命令控制两个CAN电机

用法:
    python motor_controller.py              # 交互模式
    python motor_controller.py --port COM3  # 指定串口
    python motor_controller.py --stream     # 启动数据流并绘图
"""

import serial
import serial.tools.list_ports
import time
import argparse
import threading
import sys
from collections import deque

class MotorController:
    def __init__(self, port=None, baudrate=115200):
        self.ser = None
        self.baudrate = baudrate
        self.streaming = False
        self.stream_thread = None
        self.data_buffer = {
            'time': deque(maxlen=1000),
            'L_pos': deque(maxlen=1000),
            'L_vel': deque(maxlen=1000),
            'L_cur': deque(maxlen=1000),
            'L_cmd': deque(maxlen=1000),
            'R_pos': deque(maxlen=1000),
            'R_vel': deque(maxlen=1000),
            'R_cur': deque(maxlen=1000),
            'R_cmd': deque(maxlen=1000),
        }
        
        if port:
            self.connect(port)
    
    def list_ports(self):
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        print("\n可用串口:")
        for p in ports:
            print(f"  {p.device}: {p.description}")
        return [p.device for p in ports]
    
    def connect(self, port):
        """连接到指定串口"""
        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=0.1)
            time.sleep(2)  # 等待Teensy重启
            self.ser.flushInput()
            print(f"[OK] 已连接到 {port}")
            return True
        except Exception as e:
            print(f"[ERROR] 连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.ser:
            self.stop_stream()
            self.ser.close()
            print("[OK] 已断开连接")
    
    def send_command(self, cmd):
        """发送命令"""
        if not self.ser:
            print("[ERROR] 未连接")
            return None
        
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.05)
        
        response = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            if line:
                response.append(line)
        
        return response
    
    def enable(self, side='both'):
        """使能电机"""
        if side == 'left':
            return self.send_command('E,L')
        elif side == 'right':
            return self.send_command('E,R')
        else:
            return self.send_command('E')
    
    def disable(self, side='both'):
        """禁用电机"""
        if side == 'left':
            return self.send_command('D,L')
        elif side == 'right':
            return self.send_command('D,R')
        else:
            return self.send_command('D')
    
    def zero(self):
        """位置归零"""
        return self.send_command('Z')
    
    def stop(self):
        """停止电机"""
        return self.send_command('S')
    
    def set_torque(self, left=None, right=None):
        """设置扭矩 (Nm)"""
        if left is not None and right is not None:
            return self.send_command(f'T,B,{left},{right}')
        elif left is not None:
            return self.send_command(f'T,L,{left}')
        elif right is not None:
            return self.send_command(f'T,R,{right}')
    
    def set_position(self, left=None, right=None):
        """设置位置 (rad)"""
        if left is not None and right is not None:
            return self.send_command(f'P,B,{left},{right}')
        elif left is not None:
            return self.send_command(f'P,L,{left}')
        elif right is not None:
            return self.send_command(f'P,R,{right}')
    
    def set_gains(self, kp, kd):
        """设置位置控制增益"""
        return self.send_command(f'G,{kp},{kd}')
    
    def set_mode(self, mode):
        """设置控制模式 ('torque' 或 'position')"""
        if mode.lower() == 'torque':
            return self.send_command('M,T')
        elif mode.lower() == 'position':
            return self.send_command('M,P')
    
    def set_torque_limit(self, limit):
        """设置扭矩限制 (Nm)"""
        return self.send_command(f'LIMIT,{limit}')
    
    def start_sine_test(self, amplitude, frequency):
        """启动正弦波测试"""
        return self.send_command(f'SINE,{amplitude},{frequency}')
    
    def stop_sine_test(self):
        """停止正弦波测试"""
        return self.send_command('SINE,OFF')
    
    def get_status(self):
        """获取状态"""
        return self.send_command('R')
    
    def start_stream(self, callback=None):
        """启动数据流"""
        self.streaming = True
        self.send_command('STREAM,ON')
        self.stream_thread = threading.Thread(target=self._stream_reader, args=(callback,))
        self.stream_thread.daemon = True
        self.stream_thread.start()
    
    def stop_stream(self):
        """停止数据流"""
        self.streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1)
        self.send_command('STREAM,OFF')
    
    def _stream_reader(self, callback):
        """数据流读取线程"""
        while self.streaming and self.ser:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line and ',' in line:
                        parts = line.split(',')
                        if len(parts) == 9:
                            try:
                                data = {
                                    'time': float(parts[0]),
                                    'L_pos': float(parts[1]),
                                    'L_vel': float(parts[2]),
                                    'L_cur': float(parts[3]),
                                    'L_cmd': float(parts[4]),
                                    'R_pos': float(parts[5]),
                                    'R_vel': float(parts[6]),
                                    'R_cur': float(parts[7]),
                                    'R_cmd': float(parts[8]),
                                }
                                for key, value in data.items():
                                    self.data_buffer[key].append(value)
                                if callback:
                                    callback(data)
                            except ValueError:
                                pass
            except:
                break
            time.sleep(0.001)


def interactive_mode(controller):
    """交互模式"""
    print("\n========== 交互模式 ==========")
    print("输入命令 (输入 'help' 查看帮助, 'quit' 退出)")
    print("===============================\n")
    
    while True:
        try:
            cmd = input(">>> ").strip()
            
            if not cmd:
                continue
            
            if cmd.lower() == 'quit' or cmd.lower() == 'exit':
                print("正在退出...")
                controller.disable()
                break
            
            if cmd.lower() == 'help':
                print("""
可用命令:
  enable [left|right]     - 使能电机
  disable [left|right]    - 禁用电机
  zero                    - 位置归零
  stop                    - 停止
  
  torque <left> [right]   - 设置扭矩 (Nm)
  pos <left> [right]      - 设置位置 (rad)
  gains <kp> <kd>         - 设置增益
  
  mode torque|position    - 切换模式
  limit <value>           - 设置扭矩限制
  
  sine <amp> <freq>       - 正弦测试
  sine off                - 停止正弦测试
  
  status                  - 查看状态
  stream on|off           - 数据流开关
  
  <任意命令>              - 直接发送原始命令
  quit                    - 退出
""")
                continue
            
            # 解析命令
            parts = cmd.lower().split()
            
            if parts[0] == 'enable':
                if len(parts) > 1:
                    controller.enable(parts[1])
                else:
                    controller.enable()
            
            elif parts[0] == 'disable':
                if len(parts) > 1:
                    controller.disable(parts[1])
                else:
                    controller.disable()
            
            elif parts[0] == 'zero':
                controller.zero()
            
            elif parts[0] == 'stop':
                controller.stop()
            
            elif parts[0] == 'torque':
                if len(parts) >= 3:
                    controller.set_torque(float(parts[1]), float(parts[2]))
                elif len(parts) == 2:
                    controller.set_torque(float(parts[1]), float(parts[1]))
            
            elif parts[0] == 'pos':
                if len(parts) >= 3:
                    controller.set_position(float(parts[1]), float(parts[2]))
                elif len(parts) == 2:
                    controller.set_position(float(parts[1]), float(parts[1]))
            
            elif parts[0] == 'gains' and len(parts) >= 3:
                controller.set_gains(float(parts[1]), float(parts[2]))
            
            elif parts[0] == 'mode' and len(parts) >= 2:
                controller.set_mode(parts[1])
            
            elif parts[0] == 'limit' and len(parts) >= 2:
                controller.set_torque_limit(float(parts[1]))
            
            elif parts[0] == 'sine':
                if len(parts) >= 3:
                    controller.start_sine_test(float(parts[1]), float(parts[2]))
                elif len(parts) == 2 and parts[1] == 'off':
                    controller.stop_sine_test()
            
            elif parts[0] == 'status':
                response = controller.get_status()
                if response:
                    for line in response:
                        print(line)
            
            elif parts[0] == 'stream':
                if len(parts) >= 2:
                    if parts[1] == 'on':
                        controller.start_stream(lambda d: print(f"L:{d['L_pos']:.3f} R:{d['R_pos']:.3f}"))
                    else:
                        controller.stop_stream()
            
            else:
                # 直接发送原始命令
                response = controller.send_command(cmd.upper())
                if response:
                    for line in response:
                        print(line)
        
        except KeyboardInterrupt:
            print("\n中断...")
            controller.disable()
            break
        except Exception as e:
            print(f"错误: {e}")


def main():
    parser = argparse.ArgumentParser(description='Teensy 双电机控制器')
    parser.add_argument('--port', '-p', type=str, help='串口名称')
    parser.add_argument('--list', '-l', action='store_true', help='列出可用串口')
    parser.add_argument('--stream', '-s', action='store_true', help='启动数据流')
    parser.add_argument('--plot', action='store_true', help='实时绘图 (需要matplotlib)')
    args = parser.parse_args()
    
    controller = MotorController()
    
    if args.list:
        controller.list_ports()
        return
    
    # 自动查找串口
    if not args.port:
        ports = controller.list_ports()
        teensy_ports = [p for p in ports if 'teensy' in p.lower() or 'usb' in p.lower()]
        if teensy_ports:
            args.port = teensy_ports[0]
            print(f"自动选择: {args.port}")
        elif ports:
            args.port = ports[0]
            print(f"使用第一个可用串口: {args.port}")
        else:
            print("[ERROR] 未找到可用串口")
            return
    
    if not controller.connect(args.port):
        return
    
    try:
        if args.plot:
            try:
                import matplotlib.pyplot as plt
                import matplotlib.animation as animation
                
                fig, axes = plt.subplots(2, 2, figsize=(12, 8))
                fig.suptitle('Motor Data')
                
                lines = {}
                
                def init():
                    for ax in axes.flat:
                        ax.set_xlim(0, 1000)
                    
                    axes[0, 0].set_title('Position (rad)')
                    axes[0, 1].set_title('Velocity (rad/s)')
                    axes[1, 0].set_title('Current (A)')
                    axes[1, 1].set_title('Torque Cmd (Nm)')
                    
                    lines['L_pos'], = axes[0, 0].plot([], [], 'b-', label='Left')
                    lines['R_pos'], = axes[0, 0].plot([], [], 'r-', label='Right')
                    lines['L_vel'], = axes[0, 1].plot([], [], 'b-', label='Left')
                    lines['R_vel'], = axes[0, 1].plot([], [], 'r-', label='Right')
                    lines['L_cur'], = axes[1, 0].plot([], [], 'b-', label='Left')
                    lines['R_cur'], = axes[1, 0].plot([], [], 'r-', label='Right')
                    lines['L_cmd'], = axes[1, 1].plot([], [], 'b-', label='Left')
                    lines['R_cmd'], = axes[1, 1].plot([], [], 'r-', label='Right')
                    
                    for ax in axes.flat:
                        ax.legend()
                        ax.grid(True)
                    
                    return lines.values()
                
                def update(frame):
                    buf = controller.data_buffer
                    if len(buf['time']) > 0:
                        x = list(range(len(buf['time'])))
                        
                        for key in ['L_pos', 'R_pos', 'L_vel', 'R_vel', 'L_cur', 'R_cur', 'L_cmd', 'R_cmd']:
                            lines[key].set_data(x, list(buf[key]))
                        
                        for ax in axes.flat:
                            ax.relim()
                            ax.autoscale_view()
                    
                    return lines.values()
                
                controller.start_stream()
                
                ani = animation.FuncAnimation(fig, update, init_func=init, 
                                             interval=100, blit=False)
                plt.tight_layout()
                plt.show()
                
            except ImportError:
                print("[ERROR] 需要安装 matplotlib: pip install matplotlib")
                interactive_mode(controller)
        else:
            interactive_mode(controller)
    
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
