#!/usr/bin/env python3
"""
Real-time CAN Data Plotter
Purpose: Collect data from ESP32 and plot real-time graphs
Author: GitHub Copilot
Date: 2025-11-20
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import argparse
import sys
import time
from datetime import datetime
import csv

class CANDataPlotter:
    def __init__(self, serial_port='/dev/cu.usbserial-110', baud_rate=115200, max_points=100):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.max_points = max_points
        
        # Data storage
        self.timestamps = deque(maxlen=max_points)
        self.temperatures = deque(maxlen=max_points)
        self.currents = deque(maxlen=max_points)
        self.speeds = deque(maxlen=max_points)
        self.encoders = deque(maxlen=max_points)
        self.accelerations = deque(maxlen=max_points)
        self.status_valid = deque(maxlen=max_points)
        self.accel_valid = deque(maxlen=max_points)
        
        # Statistics
        self.start_time = None
        self.data_count = 0
        self.valid_status_count = 0
        self.valid_accel_count = 0
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        
        # Serial connection
        self.ser = None
        
        # Setup matplotlib
        self.setup_plots()
        
    def setup_plots(self):
        """Setup matplotlib plots"""
        plt.style.use('dark_background')
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 10))
        self.fig.suptitle('Real-time CAN Motor Data', fontsize=16, color='white')
        
        # Individual plots
        self.axes[0, 0].set_title('Temperature (°C)')
        self.axes[0, 1].set_title('Current (Raw)')
        self.axes[0, 2].set_title('Speed (dps)')
        self.axes[1, 0].set_title('Encoder Position')
        self.axes[1, 1].set_title('Acceleration (dps/s)')
        self.axes[1, 2].set_title('Data Validity')
        
        # Set colors and styles
        for ax in self.axes.flat:
            ax.grid(True, alpha=0.3)
            ax.set_facecolor('#1e1e1e')
        
        plt.tight_layout()
        
    def connect_serial(self):
        """Connect to ESP32 via serial"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for connection
            print(f"✅ Connected to {self.serial_port} at {self.baud_rate} baud")
            
            # Wait for DATA_START marker
            print("🔍 Waiting for ESP32 to start data transmission...")
            while True:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"📝 {line}")
                if 'DATA_START' in line:
                    print("🚀 Data transmission started!")
                    break
                    
        except Exception as e:
            print(f"❌ Failed to connect to serial port: {e}")
            sys.exit(1)
            
    def setup_csv_logging(self):
        """Setup CSV file for data logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"can_data_{timestamp}.csv"
        
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Timestamp', 'Temperature', 'Current', 'Speed', 
            'Encoder', 'Acceleration', 'Status_Valid', 'Accel_Valid'
        ])
        print(f"📊 Logging data to {filename}")
        
    def parse_data_line(self, line):
        """Parse CSV data line from ESP32"""
        try:
            parts = line.split(',')
            if len(parts) != 8:
                return None
                
            data = {
                'timestamp': int(parts[0]),
                'temperature': int(parts[1]),
                'current': int(parts[2]),
                'speed': int(parts[3]),
                'encoder': int(parts[4]),
                'acceleration': int(parts[5]),
                'status_valid': bool(int(parts[6])),
                'accel_valid': bool(int(parts[7]))
            }
            return data
            
        except (ValueError, IndexError) as e:
            return None
            
    def update_data(self, data):
        """Update data collections"""
        if self.start_time is None:
            self.start_time = data['timestamp']
            
        # Convert timestamp to relative time in seconds
        relative_time = (data['timestamp'] - self.start_time) / 1000.0
        
        # Update data collections
        self.timestamps.append(relative_time)
        self.temperatures.append(data['temperature'])
        self.currents.append(data['current'])
        self.speeds.append(data['speed'])
        self.encoders.append(data['encoder'])
        self.accelerations.append(data['acceleration'])
        self.status_valid.append(1 if data['status_valid'] else 0)
        self.accel_valid.append(1 if data['accel_valid'] else 0)
        
        # Update statistics
        self.data_count += 1
        if data['status_valid']:
            self.valid_status_count += 1
        if data['accel_valid']:
            self.valid_accel_count += 1
            
        # Log to CSV
        if self.csv_writer:
            self.csv_writer.writerow([
                data['timestamp'], data['temperature'], data['current'],
                data['speed'], data['encoder'], data['acceleration'],
                data['status_valid'], data['accel_valid']
            ])
            self.csv_file.flush()
            
    def update_plots(self, frame):
        """Update matplotlib plots (called by animation)"""
        # Read new data from serial
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check for special commands
                if 'DATA_END' in line:
                    print("📋 Data collection ended by ESP32")
                    return
                    
                # Parse data line
                data = self.parse_data_line(line)
                if data:
                    self.update_data(data)
                    
        except Exception as e:
            print(f"⚠️ Serial read error: {e}")
            
        # Update plots if we have data
        if len(self.timestamps) < 2:
            return
            
        # Clear all axes
        for ax in self.axes.flat:
            ax.clear()
            ax.grid(True, alpha=0.3)
            ax.set_facecolor('#1e1e1e')
            
        times = list(self.timestamps)
        
        # Plot temperature
        self.axes[0, 0].plot(times, list(self.temperatures), 'r-', linewidth=2)
        self.axes[0, 0].set_title('Temperature (°C)', color='white')
        self.axes[0, 0].set_ylabel('°C', color='white')
        
        # Plot current
        self.axes[0, 1].plot(times, list(self.currents), 'g-', linewidth=2)
        self.axes[0, 1].set_title('Current (Raw)', color='white')
        self.axes[0, 1].set_ylabel('Raw Value', color='white')
        
        # Plot speed
        self.axes[0, 2].plot(times, list(self.speeds), 'b-', linewidth=2)
        self.axes[0, 2].set_title('Speed (dps)', color='white')
        self.axes[0, 2].set_ylabel('dps', color='white')
        
        # Plot encoder
        self.axes[1, 0].plot(times, list(self.encoders), 'c-', linewidth=2)
        self.axes[1, 0].set_title('Encoder Position', color='white')
        self.axes[1, 0].set_ylabel('Raw Value', color='white')
        
        # Plot acceleration
        self.axes[1, 1].plot(times, list(self.accelerations), 'm-', linewidth=2)
        self.axes[1, 1].set_title('Acceleration (dps/s)', color='white')
        self.axes[1, 1].set_ylabel('dps/s', color='white')
        
        # Plot validity
        self.axes[1, 2].plot(times, list(self.status_valid), 'y-', linewidth=2, label='Status')
        self.axes[1, 2].plot(times, list(self.accel_valid), 'orange', linewidth=2, label='Acceleration')
        self.axes[1, 2].set_title('Data Validity', color='white')
        self.axes[1, 2].set_ylabel('Valid (1) / Invalid (0)', color='white')
        self.axes[1, 2].legend()
        self.axes[1, 2].set_ylim(-0.1, 1.1)
        
        # Set x-labels for bottom row
        for ax in self.axes[1, :]:
            ax.set_xlabel('Time (seconds)', color='white')
            
        # Update title with statistics
        status_rate = (self.valid_status_count / self.data_count * 100) if self.data_count > 0 else 0
        accel_rate = (self.valid_accel_count / self.data_count * 100) if self.data_count > 0 else 0
        
        title = f'Real-time CAN Motor Data | Samples: {self.data_count} | Status: {status_rate:.1f}% | Accel: {accel_rate:.1f}%'
        self.fig.suptitle(title, fontsize=14, color='white')
        
    def run(self):
        """Start data collection and plotting"""
        print("🔌 Connecting to ESP32...")
        self.connect_serial()
        
        print("📁 Setting up data logging...")
        self.setup_csv_logging()
        
        print("📈 Starting real-time plotting...")
        print("📋 Press Ctrl+C to stop")
        
        # Start animation
        try:
            ani = animation.FuncAnimation(
                self.fig, self.update_plots, interval=50, blit=False
            )
            plt.show()
            
        except KeyboardInterrupt:
            print("\n🛑 Stopping data collection...")
            
        finally:
            if self.ser:
                self.ser.close()
            if self.csv_file:
                self.csv_file.close()
            print("✅ Data collection completed")

def main():
    parser = argparse.ArgumentParser(description='Real-time CAN Data Plotter')
    parser.add_argument('--port', default='/dev/cu.usbserial-110', 
                       help='Serial port (default: /dev/cu.usbserial-110)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--points', type=int, default=100,
                       help='Maximum points to display (default: 100)')
    
    args = parser.parse_args()
    
    plotter = CANDataPlotter(args.port, args.baud, args.points)
    plotter.run()

if __name__ == '__main__':
    main()