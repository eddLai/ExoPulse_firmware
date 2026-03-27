#!/usr/bin/env python3
"""
最簡單的雙向測試：發送6筆 TEST 請求，檢查回應
"""

import socket
import time
import re

ESP32_IP = "10.16.241.20"
ESP32_PORT = 8888

print("=" * 60)
print("簡單雙向傳輸測試 - 發送6筆資料")
print("=" * 60)

try:
    # 連接
    print(f"\n連接到 {ESP32_IP}:{ESP32_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    sock.connect((ESP32_IP, ESP32_PORT))
    print("✓ 連接成功!")

    # 等待一下讓連接穩定
    time.sleep(1)

    # 發送6筆測試資料
    print("\n開始測試...")
    success = 0
    all_responses = ""

    for i in range(1, 7):
        # 發送 TEST 請求
        test_msg = f"[TEST] seq:{i}\n"
        print(f"\n{i}. 發送: {test_msg.strip()}")

        sock.send(test_msg.encode('utf-8'))

        # 累積接收資料（500ms）
        start_time = time.time()
        chunk_data = ""

        while time.time() - start_time < 0.5:
            try:
                sock.settimeout(0.1)
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                chunk_data += data
                all_responses += data
            except socket.timeout:
                continue

        # 檢查這一輪是否收到對應的 TEST_REPLY
        if f"[TEST_REPLY] seq:{i}" in chunk_data:
            # 提取回應訊息
            match = re.search(rf'\[TEST_REPLY\] seq:{i}[^\n]*', chunk_data)
            if match:
                print(f"   ✓ 收到回應: {match.group(0)}")
                success += 1
        else:
            print(f"   ✗ 沒有收到回應 seq:{i}")

        time.sleep(0.2)  # 短暫間隔

    # 發送完畢後，再等待1秒收集延遲的回應
    print("\n等待延遲的回應...")
    time.sleep(1)
    sock.settimeout(0.1)
    try:
        while True:
            data = sock.recv(4096).decode('utf-8', errors='ignore')
            if data:
                all_responses += data
            else:
                break
    except socket.timeout:
        pass

    # 最終統計：重新檢查所有收到的回應
    print("\n" + "=" * 60)
    test_replies = re.findall(r'\[TEST_REPLY\][^\n]*', all_responses)

    if test_replies:
        print(f"收到的所有 TEST_REPLY ({len(test_replies)} 筆):")
        for reply in test_replies:
            print(f"  {reply}")

        print("\n" + "=" * 60)

        # 檢查每個 seq 是否收到
        received_seqs = set()
        for reply in test_replies:
            match = re.search(r'seq:(\d+)', reply)
            if match:
                received_seqs.add(int(match.group(1)))

        missing_seqs = set(range(1, 7)) - received_seqs

        if len(received_seqs) == 6:
            print("✓ 全部測試通過！雙向傳輸正常！")
            print(f"✓ 所有 6 個序列都收到回應")
        else:
            print(f"⚠ 收到 {len(received_seqs)}/6 個回應")
            print(f"✓ 收到的序列: {sorted(received_seqs)}")
            if missing_seqs:
                print(f"✗ 遺失的序列: {sorted(missing_seqs)}")
    else:
        print("✗ 沒有收到任何 TEST_REPLY")

    print("=" * 60)

    sock.close()

except Exception as e:
    print(f"\n✗ 錯誤: {e}")
