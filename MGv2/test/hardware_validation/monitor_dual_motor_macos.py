#!/usr/bin/env python3
"""
MacOS Compatible Enhanced Dual Motor Monitor with Real-time GUI
Fixed version for macOS with proper matplotlib backend and interactive GUI
"""

import os
import sys
import platform

# 設置 macOS 相容的 matplotlib 後端
if platform.system() == 'Darwin':  # macOS
    try:
        import matplotlib
        # 嘗試使用不同的後端
        backends_to_try = ['Qt5Agg', 'TkAgg', 'MacOSX']
        backend_set = False
        
        for backend in backends_to_try:
            try:
                matplotlib.use(backend, force=True)
                print(f"使用 matplotlib 後端: {backend}")
                backend_set = True
                break
            except ImportError:
                continue
        
        if not backend_set:
            print("警告: 無法設置圖形後端，使用預設後端")
    except Exception as e:
        print(f"後端設置錯誤: {e}")
else:
    import matplotlib

import serial
import re
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
import threading

class EnhancedDualMotorGUI:
    def __init__(self, port='/dev/cu.usbserial-110', baudrate=115200, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False
        self.serial_lock = threading.Lock()
        
        # macOS 特殊設置
        if platform.system() == 'Darwin':
            os.environ['QT_MAC_WANTS_LAYER'] = '1'
        
        # Data storage for Motor 1
        self.data_m1 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        # Data storage for Motor 2
        self.data_m2 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        self.status_m1 = {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0}
        self.status_m2 = {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0}

        self.start_time = time.time()
        self.frame_count = 0

    def connect(self):
        """連接序列埠"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connected to {self.port}")
            time.sleep(1)
            return True
        except Exception as e:
            print(f"✗ Failed: {e}")
            return False

    def parse_line(self, line):
        """解析馬達狀態資料"""
        if not line.startswith('['):
            return None
        try:
            # 解析格式: [123] M:1 T:25 V:12.5 I:1.2 S:120 ACC:10 E:1234 A:45.6 ERR:0x00
            match = re.match(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(9)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'motor_id': int(match.group(2)),
                    'temp': int(match.group(3)),
                    'voltage': float(match.group(4)),
                    'current': float(match.group(5)),
                    'speed': int(match.group(6)),
                    'acceleration': int(match.group(7)),
                    'encoder': int(match.group(8)),
                    'angle': angle_val,
                    'error': match.group(10)
                }
        except Exception as e:
            print(f"解析錯誤: {e} (line: {line.strip()})")
        return None

    def read_serial(self):
        """讀取序列埠資料"""
        print("開始讀取序列埠資料...")
        while self.running:
            try:
                with self.serial_lock:
                    if self.ser and self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            data = self.parse_line(line)
                            if data:
                                current_time = time.time() - self.start_time
                                
                                if data['motor_id'] == 1:
                                    self.update_motor_data(self.data_m1, self.status_m1, data, current_time)
                                elif data['motor_id'] == 2:
                                    self.update_motor_data(self.data_m2, self.status_m2, data, current_time)
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"讀取錯誤: {e}")
                time.sleep(0.1)

    def update_motor_data(self, data_dict, status_dict, new_data, current_time):
        """更新馬達資料"""
        data_dict['time'].append(current_time)
        data_dict['temp'].append(new_data['temp'])
        data_dict['current'].append(new_data['current'])
        data_dict['speed'].append(new_data['speed'])
        data_dict['acceleration'].append(new_data['acceleration'])
        data_dict['angle'].append(new_data['angle'])
        
        # 更新狀態
        status_dict.update(new_data)

    def send_reset_command(self, motor_id=None):
        """發送重設命令"""
        try:
            with self.serial_lock:
                if motor_id:
                    cmd = f"RESET {motor_id}\n"
                else:
                    cmd = "RESET ALL\n"
                self.ser.write(cmd.encode())
                print(f"✓ Reset command sent: {cmd.strip()}")
        except Exception as e:
            print(f"重設命令錯誤: {e}")

    def setup_plots(self):
        """設置圖表"""
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.canvas.manager.set_window_title('Enhanced Dual Motor Monitor - macOS Compatible')
        
        # 創建網格佈局
        gs = gridspec.GridSpec(3, 4, hspace=0.3, wspace=0.3)
        
        # 按鈕區域
        self.ax_buttons = self.fig.add_subplot(gs[0, :])
        self.ax_buttons.text(0.1, 0.5, 'Reset M1', fontsize=12, ha='center', 
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral"))
        self.ax_buttons.text(0.3, 0.5, 'Reset M2', fontsize=12, ha='center',
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
        self.ax_buttons.text(0.5, 0.5, 'Reset ALL', fontsize=12, ha='center',
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen"))
        self.ax_buttons.text(0.7, 0.5, 'Status Info', fontsize=10, ha='left')
        self.ax_buttons.set_xlim(0, 1)
        self.ax_buttons.set_ylim(0, 1)
        self.ax_buttons.axis('off')
        
        # 溫度圖表
        self.ax_temp = self.fig.add_subplot(gs[1, 0])
        self.line_temp_m1, = self.ax_temp.plot([], [], 'r-', linewidth=2, label='Motor 1')
        self.line_temp_m2, = self.ax_temp.plot([], [], 'b-', linewidth=2, label='Motor 2')
        self.ax_temp.set_title('溫度 (°C)', fontweight='bold')
        self.ax_temp.set_ylim(0, 100)
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)
        
        # 電流圖表
        self.ax_current = self.fig.add_subplot(gs[1, 1])
        self.line_current_m1, = self.ax_current.plot([], [], 'r-', linewidth=2, label='Motor 1')
        self.line_current_m2, = self.ax_current.plot([], [], 'b-', linewidth=2, label='Motor 2')
        self.ax_current.set_title('電流 (A)', fontweight='bold')
        self.ax_current.set_ylim(-10, 10)
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)
        
        # 速度圖表 (左軸: dps, 右軸: rad/s)
        self.ax_speed = self.fig.add_subplot(gs[1, 2])
        self.ax_speed_rad = self.ax_speed.twinx()
        self.line_speed_m1, = self.ax_speed.plot([], [], 'r-', linewidth=2, label='Motor 1')
        self.line_speed_m2, = self.ax_speed.plot([], [], 'b-', linewidth=2, label='Motor 2')
        self.ax_speed.set_title('速度 (dps / rad/s)', fontweight='bold')
        self.ax_speed.set_ylim(-1000, 1000)
        self.ax_speed_rad.set_ylim(-17.45, 17.45)  # ±1000 dps in rad/s
        self.ax_speed.set_ylabel('dps', color='black')
        self.ax_speed_rad.set_ylabel('rad/s', color='gray')
        self.ax_speed.legend()
        self.ax_speed.grid(True, alpha=0.3)
        
        # 加速度圖表
        self.ax_accel = self.fig.add_subplot(gs[1, 3])
        self.line_accel_m1, = self.ax_accel.plot([], [], 'r-', linewidth=2, label='Motor 1')
        self.line_accel_m2, = self.ax_accel.plot([], [], 'b-', linewidth=2, label='Motor 2')
        self.ax_accel.set_title('加速度 (dps/s)', fontweight='bold')
        self.ax_accel.set_ylim(-1000, 1000)
        self.ax_accel.legend()
        self.ax_accel.grid(True, alpha=0.3)
        
        # 角度圖表
        self.ax_angle = self.fig.add_subplot(gs[2, :2])
        self.line_angle_m1, = self.ax_angle.plot([], [], 'r-', linewidth=2, label='Motor 1')
        self.line_angle_m2, = self.ax_angle.plot([], [], 'b-', linewidth=2, label='Motor 2')
        self.ax_angle.set_title('多圈角度 (°)', fontweight='bold')
        self.ax_angle.set_ylim(-360, 360)
        self.ax_angle.legend()
        self.ax_angle.grid(True, alpha=0.3)
        
        # 狀態顯示區域
        self.ax_status = self.fig.add_subplot(gs[2, 2:])
        self.ax_status.axis('off')
        
        # 設置點擊事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        """處理滑鼠點擊事件"""
        if event.inaxes == self.ax_buttons:
            x = event.xdata
            if x and 0.05 < x < 0.15:  # Reset M1
                self.send_reset_command(1)
            elif x and 0.25 < x < 0.35:  # Reset M2
                self.send_reset_command(2)
            elif x and 0.45 < x < 0.55:  # Reset ALL
                self.send_reset_command()

    def animate(self, frame):
        """動畫更新函數"""
        self.frame_count += 1
        
        # 更新溫度
        if len(self.data_m1['time']) > 0:
            self.line_temp_m1.set_data(list(self.data_m1['time']), list(self.data_m1['temp']))
        if len(self.data_m2['time']) > 0:
            self.line_temp_m2.set_data(list(self.data_m2['time']), list(self.data_m2['temp']))
        
        # 更新電流
        if len(self.data_m1['time']) > 0:
            self.line_current_m1.set_data(list(self.data_m1['time']), list(self.data_m1['current']))
        if len(self.data_m2['time']) > 0:
            self.line_current_m2.set_data(list(self.data_m2['time']), list(self.data_m2['current']))
        
        # 更新速度
        if len(self.data_m1['time']) > 0:
            self.line_speed_m1.set_data(list(self.data_m1['time']), list(self.data_m1['speed']))
        if len(self.data_m2['time']) > 0:
            self.line_speed_m2.set_data(list(self.data_m2['time']), list(self.data_m2['speed']))
        
        # 更新加速度
        if len(self.data_m1['time']) > 0:
            self.line_accel_m1.set_data(list(self.data_m1['time']), list(self.data_m1['acceleration']))
        if len(self.data_m2['time']) > 0:
            self.line_accel_m2.set_data(list(self.data_m2['time']), list(self.data_m2['acceleration']))
        
        # 更新角度
        if len(self.data_m1['time']) > 0:
            self.line_angle_m1.set_data(list(self.data_m1['time']), list(self.data_m1['angle']))
        if len(self.data_m2['time']) > 0:
            self.line_angle_m2.set_data(list(self.data_m2['time']), list(self.data_m2['angle']))
        
        # 自動調整 X 軸範圍
        all_times = list(self.data_m1['time']) + list(self.data_m2['time'])
        if all_times:
            max_time = max(all_times)
            min_time = max(0, max_time - 30)  # 顯示最近30秒
            
            for ax in [self.ax_temp, self.ax_current, self.ax_speed, self.ax_accel, self.ax_angle]:
                ax.set_xlim(min_time, max_time + 1)
        
        # 更新狀態顯示
        if self.frame_count % 10 == 0:  # 每10幀更新一次狀態
            self.update_status_display()
        
        return (self.line_temp_m1, self.line_temp_m2, self.line_current_m1, self.line_current_m2,
                self.line_speed_m1, self.line_speed_m2, self.line_accel_m1, self.line_accel_m2,
                self.line_angle_m1, self.line_angle_m2)

    def update_status_display(self):
        """更新狀態顯示"""
        self.ax_status.clear()
        self.ax_status.axis('off')
        
        # 馬達1狀態
        status_text_m1 = f"""馬達 1:
溫度: {self.status_m1.get('temp', 0)}°C
電壓: {self.status_m1.get('voltage', 0):.1f}V
電流: {self.status_m1.get('current', 0):.2f}A
速度: {self.status_m1.get('speed', 0)} dps
角度: {self.status_m1.get('angle', 0):.1f}°"""
        
        # 馬達2狀態
        status_text_m2 = f"""馬達 2:
溫度: {self.status_m2.get('temp', 0)}°C
電壓: {self.status_m2.get('voltage', 0):.1f}V
電流: {self.status_m2.get('current', 0):.2f}A
速度: {self.status_m2.get('speed', 0)} dps
角度: {self.status_m2.get('angle', 0):.1f}°"""
        
        self.ax_status.text(0.1, 0.5, status_text_m1, fontsize=10, ha='left', va='center',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7))
        
        self.ax_status.text(0.6, 0.5, status_text_m2, fontsize=10, ha='left', va='center',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
        
        # 連線狀態
        connection_status = "🟢 已連線" if self.running else "🔴 未連線"
        data_count = f"資料點: M1={len(self.data_m1['time'])}, M2={len(self.data_m2['time'])}"
        
        self.ax_status.text(0.1, 0.1, f"{connection_status}\n{data_count}", fontsize=9, ha='left', va='bottom')

    def start_monitoring(self):
        """啟動監控"""
        print("=" * 70)
        print("  Enhanced Dual Motor Monitor - macOS Compatible")
        print("  - 5 plots per motor: Temp, Current, Speed, Acceleration, Angle")
        print("  - Speed shown in dps (left) and rad/s (right)")
        print("  - Click buttons to reset motor angles")
        print("  - Auto-scaling time axis (last 30 seconds)")
        print("=" * 70)
        
        if not self.connect():
            return False
        
        self.running = True
        
        # 啟動序列埠讀取執行緒
        serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        serial_thread.start()
        
        # 設置圖表
        self.setup_plots()
        
        # 自動重設馬達角度
        print("Auto-resetting motor angles...")
        time.sleep(1)
        self.send_reset_command()
        print("Motor angles reset to zero!")
        
        # 啟動動畫
        print("Starting Enhanced Monitor with Full Data Display...")
        ani = FuncAnimation(self.fig, self.animate, interval=100, blit=False, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n停止監控...")
        finally:
            self.stop_monitoring()
        
        return True

    def stop_monitoring(self):
        """停止監控"""
        self.running = False
        if self.ser:
            self.ser.close()
        print("✅ 監控已停止")

def main():
    print("🔧 MacOS 增強雙馬達圖形化監控器")
    print("=" * 50)
    
    # 解析命令列參數
    port = '/dev/cu.usbserial-110'  # macOS 預設
    baudrate = 115200
    max_points = 100
    
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print("⚠️ 無效的 baud rate，使用預設值 115200")
    if len(sys.argv) >= 4:
        try:
            max_points = int(sys.argv[3])
        except ValueError:
            print("⚠️ 無效的 max_points，使用預設值 100")
    
    print(f"序列埠: {port}")
    print(f"Baud Rate: {baudrate}")
    print(f"最大資料點: {max_points}")
    print(f"圖表後端: {matplotlib.get_backend()}")
    
    monitor = EnhancedDualMotorGUI(port, baudrate, max_points)
    monitor.start_monitoring()

if __name__ == "__main__":
    main()