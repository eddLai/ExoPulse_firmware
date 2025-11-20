# ExoPulse 雙馬達監控系統資料流分析報告

## 📊 **系統架構總覽**

```
ESP32 韌體 → 序列埠 → Python 監控程式 → 即時圖形界面
    ↓           ↓         ↓            ↓
CAN 馬達    USB/串口   資料解析      視覺化顯示
```

---

## 🔄 **完整資料流程**

### 1. **資料來源 (ESP32 韌體)**
- **馬達控制器**: LK-TECH M 系列馬達
- **通訊協議**: CAN Bus (1 Mbps)
- **馬達 ID**: Motor 1 (ID=1), Motor 2 (ID=2)
- **更新頻率**: 每 100ms (10 Hz)

### 2. **序列埠傳輸格式**
```
[時間戳] M:馬達ID T:溫度 V:電壓 I:電流 S:速度 ACC:加速度 E:編碼器 A:角度 ERR:錯誤碼
```

**實例資料格式:**
```
[12345] M:1 T:25 V:12.5 I:1.2 S:120 ACC:15 E:1234 A:45.6 ERR:0x00
[12346] M:2 T:27 V:12.4 I:-0.8 S:-95 ACC:-12 E:5678 A:-23.1 ERR:0x00
```

### 3. **Python 資料處理流程**

#### 📥 **資料接收與解析**
```python
def parse_line(self, line):
    # 正規表示式解析
    pattern = r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)'
    
    # 提取欄位:
    # group(2) = motor_id    (馬達 ID: 1 或 2)
    # group(3) = temp        (溫度: -40~85°C)
    # group(4) = voltage     (電壓: 0~30V)
    # group(5) = current     (電流: -10~10A)
    # group(6) = speed       (速度: -1000~1000 dps)
    # group(7) = acceleration (加速度: -1000~1000 dps/s)
    # group(8) = encoder     (編碼器值: 0~65535)
    # group(9) = angle       (角度: -∞~∞ degrees 或 'ovf')
```

#### 🗄️ **資料儲存結構**
```python
# Motor 1 資料結構
self.data_m1 = {
    'time': deque(maxlen=100),        # 時間戳記 (相對於啟動時間)
    'temp': deque(maxlen=100),        # 溫度 (°C)
    'current': deque(maxlen=100),     # 電流 (A)
    'speed': deque(maxlen=100),       # 速度 (dps)
    'acceleration': deque(maxlen=100), # 加速度 (dps/s)
    'angle': deque(maxlen=100),       # 角度 (degrees)
}

# 即時狀態
self.status_m1 = {
    'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0, 
    'speed': 0, 'acceleration': 0, 'angle': 0
}
```

---

## 🎯 **Motor 1 角度 (Angle) 資料流詳細分析**

### **📈 角度資料特性**
- **資料類型**: 多圈絕對角度
- **單位**: degrees (度)
- **範圍**: -∞ ~ +∞ (支援多圈旋轉)
- **精度**: 0.1° 或更高
- **溢位處理**: 當角度值過大時顯示 "ovf"

### **🔄 角度資料流程**
```
LK-TECH 馬達編碼器 → CAN Bus → ESP32 → 序列埠 → Python 解析 → 圖表顯示
     ↓                ↓         ↓        ↓         ↓         ↓
14位絕對編碼器    CAN幀傳輸   韌體處理   USB傳輸   字串解析   即時繪圖
```

### **💾 角度資料處理**
```python
# 1. 接收原始角度字串
angle_str = match.group(9)  # 例: "45.6" 或 "ovf"

# 2. 溢位保護
angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)

# 3. 儲存到緩衝區 (最多100個數據點)
self.data_m1['angle'].append(angle_val)

# 4. 即時狀態更新
self.status_m1['angle'] = angle_val
```

### **📊 角度視覺化**
```python
# 圖表配置
self.ax1_angle.set_title('Motor 1 - Angle (Multi-turn)', color='cyan')
self.ax1_angle.set_ylabel('degrees', color='cyan')

# 自動縮放 Y 軸
angle_data = list(self.data_m1['angle'])
angle_min, angle_max = min(angle_data), max(angle_data)
margin = (angle_max - angle_min) * 0.1 + 1
self.ax1_angle.set_ylim(angle_min - margin, angle_max + margin)

# 顯示最近 30 秒資料
self.ax1_angle.set_xlim(max(0, t1[-1] - 30), t1[-1] + 1)
```

---

## ⚡ **Motor 1 加速度 (Acceleration) 資料流詳細分析**

### **📈 加速度資料特性**
- **資料類型**: 角加速度
- **單位**: dps/s (degrees per second squared)
- **範圍**: -1000 ~ +1000 dps/s
- **更新來源**: CAN 命令 0x33
- **精度**: 整數值

### **🔄 加速度資料流程**
```
馬達控制器計算 → CAN命令0x33 → ESP32接收 → 序列埠傳輸 → Python處理 → 圖表顯示
      ↓              ↓          ↓         ↓          ↓         ↓
速度變化率檢測    CAN總線傳輸   韌體解析    USB串口    字串解析   即時監控
```

### **💾 加速度資料處理**
```python
# 1. 接收原始加速度值
acceleration = int(match.group(7))  # 例: 15, -12, 0

# 2. 儲存到緩衝區 (deque 自動管理大小)
self.data_m1['acceleration'].append(acceleration)

# 3. 即時狀態更新
self.status_m1['acceleration'] = acceleration

# 4. 自動縮放圖表範圍
if len(self.data_m1['acceleration']) > 0:
    accel_data = list(self.data_m1['acceleration'])
    if max(accel_data) - min(accel_data) > 0:
        accel_min, accel_max = min(accel_data), max(accel_data)
        margin = (accel_max - accel_min) * 0.2 + 10
        self.ax1_accel.set_ylim(accel_min - margin, accel_max + margin)
```

### **📊 加速度視覺化特色**
```python
# 動態範圍調整
self.ax1_accel.set_title('Motor 1 - Acceleration', color='cyan')
self.ax1_accel.set_ylabel('dps/s', color='cyan')

# 智慧縮放: 根據實際數據範圍自動調整
# - 如果數據變化大: 使用實際範圍 + 20% 邊界
# - 如果數據變化小: 使用預設範圍 ±100
```

---

## 🔧 **控制命令流程**

### **重設角度功能**
```python
def reset_motor_angle(self, motor_id):
    with self.serial_lock:  # 線程安全
        if motor_id == 1:
            self.ser.write(b"RESET_M1\n")   # 重設馬達1
        elif motor_id == 2:
            self.ser.write(b"RESET_M2\n")   # 重設馬達2  
        elif motor_id == 0:
            self.ser.write(b"RESET_ALL\n")  # 重設所有馬達
```

### **CAN 命令對應**
- **RESET_M1** → CAN 命令 0x95 發送至馬達 ID 1
- **RESET_M2** → CAN 命令 0x95 發送至馬達 ID 2
- **RESET_ALL** → CAN 命令 0x95 廣播至所有馬達

---

## 📊 **效能特性分析**

### **資料更新頻率**
- **硬體採樣**: 10 Hz (每 100ms)
- **圖形更新**: ~10 FPS (FuncAnimation)
- **緩衝區大小**: 100 個數據點 (約 10 秒歷史)
- **顯示窗口**: 30 秒滑動視窗

### **記憶體使用**
```python
# 每個馬達 6 個參數 × 100 個數據點 × 8 bytes = 4.8KB
# 雙馬達總計: 約 10KB 資料緩衝
# 圖形渲染: matplotlib 動態分配
```

### **線程安全設計**
```python
self.serial_lock = threading.Lock()  # 保護序列埠存取
```
- **讀取線程**: 專門處理序列埠資料接收
- **GUI 線程**: 負責圖形更新和使用者互動
- **鎖機制**: 防止多線程存取衝突

---

## 🎨 **視覺化設計**

### **Motor 1 專屬配色**
- **主色**: Cyan (青色) - 清晰易辨識
- **圖表背景**: 深色主題
- **網格**: 30% 透明度
- **線條寬度**: 2px

### **多軸顯示**
```python
# 速度雙軸顯示
self.ax1_speed.set_ylabel('rad/s', color='cyan')       # 左軸: 弧度/秒
self.ax1_speed_deg.set_ylabel('°/s', color='yellow')   # 右軸: 角度/秒
```

---

## 🚀 **系統優勢**

1. **即時性**: 100ms 延遲的高頻監控
2. **多參數**: 6 個關鍵參數同時顯示
3. **線程安全**: 穩定的多線程架構
4. **自適應**: 動態 Y 軸縮放
5. **互動性**: 一鍵重設角度功能
6. **跨平台**: macOS/Linux/Windows 相容

---

## 📋 **總結**

這個監控系統提供了完整的馬達狀態監控，特別針對 **Motor 1 的角度和加速度** 提供了：

- **高精度角度追蹤**: 支援多圈絕對定位
- **即時加速度監控**: 動態顯示運動狀態變化
- **智慧視覺化**: 自動縮放確保最佳顯示效果
- **可靠控制**: 線程安全的重設功能

整個系統從硬體感測到軟體顯示，形成了完整的閉環監控解決方案。

---

**📝 報告生成日期**: 2025年11月20日  
**🔧 系統版本**: Enhanced Dual Motor Monitor v1.0  
**🖥️ 測試平台**: macOS (Qt5Agg 後端)