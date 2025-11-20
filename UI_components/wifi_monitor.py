#!/usr/bin/env python3
"""
ExoPulse WiFi Dual Motor Monitor
TCP網路版本的即時馬達監控系統

特色：
- 透過WiFi TCP連接ESP32
- 雙馬達即時圖表顯示
- 遠端校正命令
- 自動重連功能
- 跨平台支援 (macOS, Linux, Windows)
"""

import os
import platform
import sys
import socket
import threading
import time
import re
from collections import deque
import numpy as np

# macOS 相容性
if platform.system() == 'Darwin':
    try:
        import matplotlib
        matplotlib.use('Qt5Agg')
    except ImportError:
        try:
            import matplotlib
            matplotlib.use('TkAgg')
        except ImportError:
            pass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button

class WiFiMotorMonitor:
    def __init__(self, host='192.168.43.123', port=8888, max_points=100):
        self.host = host
        self.port = port
        self.max_points = max_points
        self.sock = None
        self.running = False
        self.connection_lock = threading.Lock()
        self.connection_status = "Disconnected"
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5

        # Motor 1 資料
        self.data_m1 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        # Motor 2 資料
        self.data_m2 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        self.status_m1 = {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0,
                         'speed': 0, 'acceleration': 0, 'angle': 0}
        self.status_m2 = {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0,
                         'speed': 0, 'acceleration': 0, 'angle': 0}

        self.start_time = time.time()

    def connect(self):
        """連接到ESP32 WiFi TCP server"""
        try:
            print(f"⚙ 正在連接 {self.host}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            self.connection_status = "Connected"
            self.reconnect_attempts = 0

            # 讀取歡迎訊息
            try:
                welcome = self.sock.recv(1024).decode('utf-8', errors='ignore')
                print(f"✓ {welcome.strip()}")
            except:
                pass

            print(f"✓ 已連接到 {self.host}:{self.port}")
            return True
        except Exception as e:
            self.connection_status = f"Error: {e}"
            print(f"✗ 連接失敗: {e}")
            print(f"   提示: 檢查ESP32電源並確認WiFi已連接")
            print(f"   確認IP位址符合ESP32序列埠輸出的IP")
            return False

    def reconnect(self):
        """嘗試重新連接"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            print(f"✗ 達到最大重連次數 ({self.max_reconnect_attempts}). 放棄連接.")
            return False

        self.reconnect_attempts += 1
        self.connection_status = f"Reconnecting... ({self.reconnect_attempts}/{self.max_reconnect_attempts})"
        print(f"\n⚠ 連接中斷! 嘗試重新連接 ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")

        # 關閉舊連接
        try:
            if self.sock:
                self.sock.close()
        except:
            pass

        time.sleep(2)

        # 嘗試重連
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            self.connection_status = "Connected (Reconnected)"
            self.reconnect_attempts = 0
            print(f"✓ 重新連接成功!")
            return True
        except Exception as e:
            self.connection_status = f"Reconnect failed: {e}"
            print(f"✗ 重新連接失敗: {e}")
            return False

    def send_command(self, command):
        """透過WiFi發送命令到ESP32"""
        try:
            with self.connection_lock:
                if self.sock:
                    self.sock.sendall((command + '\n').encode('utf-8'))
                    print(f"[CMD] 已發送: {command}")
                    return True
        except Exception as e:
            print(f"[ERR] 發送命令失敗: {e}")
            return False

    def parse_line(self, line):
        """解析馬達狀態資料行"""
        if not line.startswith('['):
            return None
        try:
            match = re.match(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(9)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'timestamp': int(match.group(1)),
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
            print(f"解析錯誤: {e} | 資料行: {line}")
        return None

    def read_data(self):
        """背景執行緒：從WiFi讀取資料"""
        buffer = ""
        while self.running:
            try:
                with self.connection_lock:
                    if self.sock:
                        data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                        if not data:
                            raise ConnectionResetError("伺服器關閉連接")
                        buffer += data

                # 處理完整的行
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # 跳過命令回應
                    if line.startswith('[OK]') or line.startswith('[ERR]') or line.startswith('[STATUS]'):
                        print(line)
                        continue

                    # 解析馬達資料
                    data = self.parse_line(line)
                    if data:
                        elapsed = time.time() - self.start_time
                        motor_id = data['motor_id']

                        if motor_id == 1:
                            self.status_m1.update(data)
                            self.data_m1['time'].append(elapsed)
                            self.data_m1['temp'].append(data['temp'])
                            self.data_m1['current'].append(data['current'])
                            self.data_m1['speed'].append(data['speed'])
                            self.data_m1['acceleration'].append(data['acceleration'])
                            self.data_m1['angle'].append(data['angle'])

                        elif motor_id == 2:
                            self.status_m2.update(data)
                            self.data_m2['time'].append(elapsed)
                            self.data_m2['temp'].append(data['temp'])
                            self.data_m2['current'].append(data['current'])
                            self.data_m2['speed'].append(data['speed'])
                            self.data_m2['acceleration'].append(data['acceleration'])
                            self.data_m2['angle'].append(data['angle'])

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ERR] 讀取錯誤: {e}")
                if self.running and not self.reconnect():
                    break
                time.sleep(1)

    # 校正命令
    def calibrate_motor1(self, event=None):
        self.send_command("CAL1")

    def calibrate_motor2(self, event=None):
        self.send_command("CAL2")

    def calibrate_both(self, event=None):
        self.send_command("CAL_BOTH")

    def clear_calibration(self, event=None):
        self.send_command("CLEAR_CAL")

    def request_status(self, event=None):
        self.send_command("STATUS")

    def update_plot(self, frame):
        """更新matplotlib圖表"""
        # 更新 Motor 1 圖表
        if len(self.data_m1['time']) > 0:
            t_m1 = list(self.data_m1['time'])

            self.line_m1_temp.set_data(t_m1, list(self.data_m1['temp']))
            self.ax_m1_temp.relim()
            self.ax_m1_temp.autoscale_view()

            self.line_m1_current.set_data(t_m1, list(self.data_m1['current']))
            self.ax_m1_current.relim()
            self.ax_m1_current.autoscale_view()

            self.line_m1_speed.set_data(t_m1, list(self.data_m1['speed']))
            self.ax_m1_speed.relim()
            self.ax_m1_speed.autoscale_view()

            self.line_m1_accel.set_data(t_m1, list(self.data_m1['acceleration']))
            self.ax_m1_accel.relim()
            self.ax_m1_accel.autoscale_view()

            self.line_m1_angle.set_data(t_m1, list(self.data_m1['angle']))
            self.ax_m1_angle.relim()
            self.ax_m1_angle.autoscale_view()

        # 更新 Motor 2 圖表
        if len(self.data_m2['time']) > 0:
            t_m2 = list(self.data_m2['time'])

            self.line_m2_temp.set_data(t_m2, list(self.data_m2['temp']))
            self.ax_m2_temp.relim()
            self.ax_m2_temp.autoscale_view()

            self.line_m2_current.set_data(t_m2, list(self.data_m2['current']))
            self.ax_m2_current.relim()
            self.ax_m2_current.autoscale_view()

            self.line_m2_speed.set_data(t_m2, list(self.data_m2['speed']))
            self.ax_m2_speed.relim()
            self.ax_m2_speed.autoscale_view()

            self.line_m2_accel.set_data(t_m2, list(self.data_m2['acceleration']))
            self.ax_m2_accel.relim()
            self.ax_m2_accel.autoscale_view()

            self.line_m2_angle.set_data(t_m2, list(self.data_m2['angle']))
            self.ax_m2_angle.relim()
            self.ax_m2_angle.autoscale_view()

        # 更新狀態文字
        status_color = 'green' if 'Connected' in self.connection_status else 'red'
        self.status_text.set_text(f'狀態: {self.connection_status}', color=status_color)

        m1_text = f'M1: T={self.status_m1["temp"]}°C | I={self.status_m1["current"]:.2f}A | S={self.status_m1["speed"]}dps | A={self.status_m1["angle"]:.2f}°'
        m2_text = f'M2: T={self.status_m2["temp"]}°C | I={self.status_m2["current"]:.2f}A | S={self.status_m2["speed"]}dps | A={self.status_m2["angle"]:.2f}°'
        self.data_text.set_text(f'{m1_text}\n{m2_text}')

        return (self.line_m1_temp, self.line_m1_current, self.line_m1_speed,
                self.line_m1_accel, self.line_m1_angle,
                self.line_m2_temp, self.line_m2_current, self.line_m2_speed,
                self.line_m2_accel, self.line_m2_angle,
                self.status_text, self.data_text)

    def run(self):
        """啟動GUI"""
        if not self.connect():
            print("連接失敗. 結束...")
            return

        self.running = True

        # 啟動資料讀取執行緒
        read_thread = threading.Thread(target=self.read_data, daemon=True)
        read_thread.start()

        # 設定matplotlib圖表
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('ExoPulse WiFi 雙馬達監控系統', fontsize=16, fontweight='bold')

        gs = gridspec.GridSpec(6, 2, figure=self.fig, hspace=0.4, wspace=0.3)

        # Motor 1 圖表 (左欄)
        self.ax_m1_temp = self.fig.add_subplot(gs[0, 0])
        self.ax_m1_current = self.fig.add_subplot(gs[1, 0])
        self.ax_m1_speed = self.fig.add_subplot(gs[2, 0])
        self.ax_m1_accel = self.fig.add_subplot(gs[3, 0])
        self.ax_m1_angle = self.fig.add_subplot(gs[4, 0])

        # Motor 2 圖表 (右欄)
        self.ax_m2_temp = self.fig.add_subplot(gs[0, 1])
        self.ax_m2_current = self.fig.add_subplot(gs[1, 1])
        self.ax_m2_speed = self.fig.add_subplot(gs[2, 1])
        self.ax_m2_accel = self.fig.add_subplot(gs[3, 1])
        self.ax_m2_angle = self.fig.add_subplot(gs[4, 1])

        # 初始化線條
        self.line_m1_temp, = self.ax_m1_temp.plot([], [], 'r-', label='Motor 1')
        self.line_m1_current, = self.ax_m1_current.plot([], [], 'b-')
        self.line_m1_speed, = self.ax_m1_speed.plot([], [], 'g-')
        self.line_m1_accel, = self.ax_m1_accel.plot([], [], 'm-')
        self.line_m1_angle, = self.ax_m1_angle.plot([], [], 'c-')

        self.line_m2_temp, = self.ax_m2_temp.plot([], [], 'r-', label='Motor 2')
        self.line_m2_current, = self.ax_m2_current.plot([], [], 'b-')
        self.line_m2_speed, = self.ax_m2_speed.plot([], [], 'g-')
        self.line_m2_accel, = self.ax_m2_accel.plot([], [], 'm-')
        self.line_m2_angle, = self.ax_m2_angle.plot([], [], 'c-')

        # 設定標籤
        for ax, title in [(self.ax_m1_temp, '溫度 (°C)'),
                          (self.ax_m1_current, '電流 (A)'),
                          (self.ax_m1_speed, '速度 (dps)'),
                          (self.ax_m1_accel, '加速度 (dps²)'),
                          (self.ax_m1_angle, '角度 (°)'),
                          (self.ax_m2_temp, '溫度 (°C)'),
                          (self.ax_m2_current, '電流 (A)'),
                          (self.ax_m2_speed, '速度 (dps)'),
                          (self.ax_m2_accel, '加速度 (dps²)'),
                          (self.ax_m2_angle, '角度 (°)')]:
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('時間 (s)', fontsize=8)
            ax.grid(True, alpha=0.3)

        # 狀態與控制區域
        ax_status = self.fig.add_subplot(gs[5, :])
        ax_status.axis('off')

        self.status_text = ax_status.text(0.02, 0.8, '狀態: 連接中...', fontsize=12, verticalalignment='top')
        self.data_text = ax_status.text(0.02, 0.5, '', fontsize=10, verticalalignment='top', family='monospace')

        # 新增按鈕
        ax_cal1 = plt.axes([0.1, 0.02, 0.1, 0.03])
        ax_cal2 = plt.axes([0.22, 0.02, 0.1, 0.03])
        ax_both = plt.axes([0.34, 0.02, 0.12, 0.03])
        ax_clear = plt.axes([0.48, 0.02, 0.12, 0.03])
        ax_status_btn = plt.axes([0.62, 0.02, 0.1, 0.03])

        btn_cal1 = Button(ax_cal1, '校正 M1')
        btn_cal2 = Button(ax_cal2, '校正 M2')
        btn_both = Button(ax_both, '校正兩者')
        btn_clear = Button(ax_clear, '清除校正')
        btn_status = Button(ax_status_btn, '狀態')

        btn_cal1.on_clicked(self.calibrate_motor1)
        btn_cal2.on_clicked(self.calibrate_motor2)
        btn_both.on_clicked(self.calibrate_both)
        btn_clear.on_clicked(self.clear_calibration)
        btn_status.on_clicked(self.request_status)

        # 動畫
        ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

        plt.show()

        self.running = False
        if self.sock:
            self.sock.close()

if __name__ == '__main__':
    print("ExoPulse WiFi 雙馬達監控系統")
    print("=" * 50)

    # 從命令列取得ESP32 IP或使用預設值
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.43.123'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8888

    print(f"目標: {host}:{port}")
    print("請確認ESP32已連接到WiFi 'ExoPulse'")
    print("=" * 50)

    monitor = WiFiMotorMonitor(host=host, port=port)
    monitor.run()
