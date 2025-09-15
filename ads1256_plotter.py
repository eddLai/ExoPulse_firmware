#!/usr/bin/env python3
"""
ADS1256 Real-time Data Plotter with Batch Processing
讀取 ESP32 硬體 UART 輸出的 ADS1256 批次資料並進行圖像化顯示
支援 ESP32 內部緩衝的高速採樣資料
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import time
import re
import argparse

class ADS1256BatchPlotter:
    def __init__(self, port, baudrate=115200, buffer_size=2000):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        
        # 數據緩存，每個通道一個 deque
        self.data_buffers = {}
        self.time_buffers = {}
        
        # 圖表設置
        self.fig, self.ax = plt.subplots(figsize=(15, 10))
        self.lines = {}
        self.colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        
        # 串口連接
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"已連接到 {port}，波特率：{baudrate}")
            time.sleep(2)  # 等待連接穩定
        except Exception as e:
            print(f"無法連接到串口 {port}: {e}")
            exit(1)
            
        # 設置圖表
        self.setup_plot()
        
        # 統計信息
        self.total_samples = 0
        self.start_time = time.time()
        self.last_update_time = time.time()
        
        # 批次處理狀態
        self.in_batch = False
        self.batch_size = 0
        self.batch_count = 0
        
    def setup_plot(self):
        """設置圖表樣式"""
        self.ax.set_title('ADS1256 High-Speed Data (ESP32 Buffered)', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Voltage (V)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, 10)  # 顯示最近 10 秒
        self.ax.set_ylim(-2.5, 2.5)  # ADS1256 的電壓範圍
        
        # 添加圖例區域
        self.ax.legend(loc='upper right')
        
    def parse_batch_header(self, line):
        """解析批次標頭：BATCH:32"""
        match = re.match(r'BATCH:(\d+)', line.strip())
        if match:
            return int(match.group(1))
        return None
        
    def parse_data_line(self, line):
        """解析資料行：CH0:123456,0.0781,1234567890"""
        match = re.match(r'CH(\d+):(-?\d+),(-?\d+\.\d+),(\d+)', line.strip())
        if match:
            channel = int(match.group(1))
            raw_value = int(match.group(2))
            voltage = float(match.group(3))
            timestamp = int(match.group(4))
            return channel, raw_value, voltage, timestamp
        return None
        
    def add_data_point(self, channel, voltage, timestamp):
        """添加新的數據點"""
        # 將 ESP32 微秒時間戳轉換為相對時間（秒）
        current_time = timestamp / 1000000.0  # 微秒轉秒
        
        # 如果是新通道，創建新的緩存和線條
        if channel not in self.data_buffers:
            self.data_buffers[channel] = deque(maxlen=self.buffer_size)
            self.time_buffers[channel] = deque(maxlen=self.buffer_size)
            color = self.colors[channel % len(self.colors)]
            line, = self.ax.plot([], [], label=f'Channel {channel}', 
                               color=color, linewidth=1.5, alpha=0.8)
            self.lines[channel] = line
            self.ax.legend()
            
        # 添加數據點
        self.data_buffers[channel].append(voltage)
        self.time_buffers[channel].append(current_time)
        self.total_samples += 1
        
    def process_serial_data(self):
        """處理串口數據"""
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue
                    
                # 跳過註釋行
                if line.startswith('#'):
                    print(f"Info: {line}")
                    continue
                
                # 處理批次標頭
                if line.startswith('BATCH:'):
                    batch_size = self.parse_batch_header(line)
                    if batch_size:
                        self.in_batch = True
                        self.batch_size = batch_size
                        self.batch_count = 0
                        print(f"接收批次，大小：{batch_size}")
                    continue
                
                # 處理批次結束
                if line == 'END_BATCH':
                    if self.in_batch:
                        print(f"批次完成，實際接收：{self.batch_count} 個樣本")
                        self.in_batch = False
                    continue
                
                # 處理數據行
                if self.in_batch:
                    parsed = self.parse_data_line(line)
                    if parsed:
                        channel, raw_value, voltage, timestamp = parsed
                        self.add_data_point(channel, voltage, timestamp)
                        self.batch_count += 1
                        
                        # 可選：打印部分數據到控制台
                        if self.batch_count <= 3 or self.batch_count % 10 == 0:
                            print(f"  CH{channel}: {raw_value} ({voltage:.4f}V) @{timestamp}")
                else:
                    # 非批次模式的單一數據行（向後兼容）
                    match = re.match(r'CH(\d+):(-?\d+),(-?\d+\.\d+)', line)
                    if match:
                        channel = int(match.group(1))
                        raw_value = int(match.group(2))
                        voltage = float(match.group(3))
                        current_time = time.time() - self.start_time
                        self.add_data_point(channel, voltage, current_time * 1000000)  # 轉換為微秒
                        
        except Exception as e:
            print(f"處理串口數據時發生錯誤: {e}")
        
    def update_plot(self, frame):
        """更新圖表（由 animation 調用）"""
        # 處理新數據
        self.process_serial_data()
        
        # 更新統計信息
        current_time = time.time()
        if current_time - self.last_update_time > 2.0:  # 每2秒更新一次統計
            elapsed = current_time - self.start_time
            if elapsed > 0:
                sample_rate = self.total_samples / elapsed
                print(f"統計：總樣本 {self.total_samples}，平均速率 {sample_rate:.1f} SPS")
            self.last_update_time = current_time
        
        # 更新所有線條
        if self.time_buffers:
            # 計算時間窗口
            all_times = []
            for time_buffer in self.time_buffers.values():
                if time_buffer:
                    all_times.extend(time_buffer)
            
            if all_times:
                latest_time = max(all_times)
                time_window = 10  # 顯示最近 10 秒
                
                # 更新 X 軸範圍
                self.ax.set_xlim(max(0, latest_time - time_window), latest_time + 1)
                
                # 更新每個通道的線條
                for channel, line in self.lines.items():
                    if (channel in self.data_buffers and 
                        channel in self.time_buffers and 
                        self.data_buffers[channel] and 
                        self.time_buffers[channel]):
                        
                        # 只顯示時間窗口內的數據
                        times = list(self.time_buffers[channel])
                        voltages = list(self.data_buffers[channel])
                        
                        # 過濾時間窗口
                        filtered_times = []
                        filtered_voltages = []
                        for t, v in zip(times, voltages):
                            if t >= latest_time - time_window:
                                filtered_times.append(t)
                                filtered_voltages.append(v)
                        
                        line.set_data(filtered_times, filtered_voltages)
                
                # 自動調整 Y 軸範圍
                all_voltages = []
                for channel in self.data_buffers:
                    if self.data_buffers[channel]:
                        # 只考慮當前時間窗口內的數據
                        times = list(self.time_buffers[channel])
                        voltages = list(self.data_buffers[channel])
                        for t, v in zip(times, voltages):
                            if t >= latest_time - time_window:
                                all_voltages.append(v)
                
                if all_voltages:
                    y_min = min(all_voltages) - 0.1
                    y_max = max(all_voltages) + 0.1
                    # 限制Y軸範圍避免過度縮放
                    y_min = max(y_min, -5.0)
                    y_max = min(y_max, 5.0)
                    self.ax.set_ylim(y_min, y_max)
                    
        return list(self.lines.values())
        
    def start_plotting(self):
        """開始實時繪圖"""
        print("開始實時繪圖... (按 Ctrl+C 停止)")
        print("等待 ESP32 數據...")
        
        # 創建動畫
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n停止繪圖")
        finally:
            self.ser.close()
            
    def save_data(self, filename):
        """保存數據到文件"""
        with open(filename, 'w') as f:
            f.write("Time,Channel,Voltage\n")
            
            for channel in sorted(self.data_buffers.keys()):
                if channel in self.time_buffers:
                    times = list(self.time_buffers[channel])
                    voltages = list(self.data_buffers[channel])
                    
                    for t, v in zip(times, voltages):
                        f.write(f"{t:.6f},{channel},{v:.6f}\n")
                        
        print(f"數據已保存到 {filename}")

def main():
    parser = argparse.ArgumentParser(description='ADS1256 批次資料實時繪圖工具')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0', 
                       help='串口設備 (預設: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='波特率 (預設: 115200)')
    parser.add_argument('--buffer', type=int, default=2000,
                       help='數據緩存大小 (預設: 2000)')
    parser.add_argument('--save', '-s', 
                       help='保存數據到文件')
    
    args = parser.parse_args()
    
    # 創建繪圖器
    plotter = ADS1256BatchPlotter(args.port, args.baudrate, args.buffer)
    
    try:
        # 開始繪圖
        plotter.start_plotting()
    finally:
        # 如果指定了保存文件，則保存數據
        if args.save:
            plotter.save_data(args.save)

if __name__ == "__main__":
    main()