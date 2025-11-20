#!/usr/bin/env python3
"""
Motor 1 診斷工具
檢查為什麼 Motor 1 沒有資料輸出
"""

import serial
import time
import re

def diagnose_motor1(port='/dev/ttyUSB0', baudrate=115200, duration=10):
    """診斷 Motor 1 連接狀態"""

    print("=" * 60)
    print("Motor 1 診斷工具")
    print("=" * 60)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Duration: {duration}s")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ 已連接到 {port}")
        time.sleep(2)
    except Exception as e:
        print(f"✗ 無法連接: {e}")
        return

    motor1_count = 0
    motor2_count = 0
    total_lines = 0
    error_lines = []
    debug_lines = []

    start_time = time.time()

    print("\n開始監控...\n")

    while time.time() - start_time < duration:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                total_lines += 1

                # 檢查 Motor 1 資料
                if ' M:1 ' in line:
                    motor1_count += 1
                    print(f"✓ Motor 1: {line}")

                # 檢查 Motor 2 資料
                elif ' M:2 ' in line:
                    motor2_count += 1
                    print(f"  Motor 2: {line}")

                # 收集 DEBUG 訊息
                elif '[DEBUG]' in line or '[ERROR]' in line:
                    debug_lines.append(line)
                    print(f"⚠ DEBUG: {line}")

                # 收集其他訊息
                else:
                    if 'Motor 1' in line or 'M:1' in line:
                        error_lines.append(line)
                        print(f"⚠ 其他: {line}")

            except Exception as e:
                print(f"✗ 解析錯誤: {e}")

    ser.close()

    # 輸出診斷報告
    print("\n" + "=" * 60)
    print("診斷報告")
    print("=" * 60)
    print(f"總共接收行數:     {total_lines}")
    print(f"Motor 1 資料數:   {motor1_count}")
    print(f"Motor 2 資料數:   {motor2_count}")
    print(f"DEBUG 訊息數:     {len(debug_lines)}")
    print("=" * 60)

    if motor1_count == 0:
        print("\n❌ 問題確認: Motor 1 完全沒有資料輸出!")
        print("\n可能原因:")
        print("1. Motor 1 未連接 CAN 總線")
        print("2. Motor 1 的 CAN ID 不是 1")
        print("3. Motor 1 電源未開啟")
        print("4. Motor 1 CAN 通訊故障")
        print("5. MCU 讀取 Motor 1 時超時 (檢查 DEBUG 訊息)")

        if debug_lines:
            print("\nDEBUG 訊息:")
            for line in debug_lines[:10]:  # 顯示前10條
                print(f"  {line}")

    elif motor1_count < motor2_count / 2:
        print(f"\n⚠️  警告: Motor 1 資料量異常少 ({motor1_count} vs Motor 2 {motor2_count})")
        print("可能是通訊不穩定或間歇性故障")

    else:
        print(f"\n✓ Motor 1 運作正常! 接收到 {motor1_count} 筆資料")

    print("=" * 60)

if __name__ == '__main__':
    diagnose_motor1(duration=10)
