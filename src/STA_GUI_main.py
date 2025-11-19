#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sta_gui.py – STA swing-control GUI  (rev-7.0, 2025-05-04)

• 非阻塞 Wi-Fi / UART 取樣，主執行緒不卡
• 即時曲線：角度 / 速度 / 電流（0.01 A → A）
• 3-D 飛機姿態示意 (roll, pitch, yaw)
• 記憶體友善：deque(maxlen) + draw_idle()
"""

##############################################################################
# Imports
##############################################################################
import socket, threading, queue, time, sys
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# — 3rd-party ---------------------------------------------------------------
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
import numpy as np                                   # noqa: E402

# — pyserial（可選） —
try:
    import serial                                    # type: ignore
    from serial.tools import list_ports              # type: ignore
except ImportError:
    serial = None
    list_ports = None

socket.setdefaulttimeout(2)                          # 全域 2-秒 timeout

##############################################################################
# Connection / protocol parameters
##############################################################################
ESP32_IP    = "10.154.48.200"
ESP32_PORT  = 8080
UDP_PORT    = 5000
HEADER      = bytes([0xAA, 0x55])
SPEED_SCALE = 200
BUF_SIZE    = 1024

##############################################################################
# CRC-8 (XOR) + helpers
##############################################################################
def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c

def odc_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    body = HEADER + bytes([cmd_id, len(payload)]) + payload
    return body + bytes([crc8(body)])

def make_motor_packet(mode1: str, val1: int, mode2: str, val2: int):
    cmd = f"X {mode1} {val1} {mode2} {val2}\r\n"
    return odc_packet(0x01, cmd.encode()), cmd.strip()

def make_led_packet():
    return odc_packet(0x10)

##############################################################################
# GUI class
##############################################################################
class STAControllerGUI(tk.Tk):
    """Tkinter-based swing / IMU controller with live plots."""

    HIST_LEN = 300            # 約 30 s 滑動窗口
    PLOT_HZ  = 10             # UI 更新頻率 (Hz)

    # ───────────────────────── Init ──────────────────────────
    def __init__(self):
        super().__init__()

        # 1. runtime attributes ------------------------------------------------
        self.ser           = None
        self.recording     = False
        self.log_lines     = []
        self.src_var       = tk.StringVar(value="wifi")       # Wi-Fi default
        self.foot_var      = tk.StringVar(value="both")

        self.tcp_sock      = None
        self.stop_udp_evt  = threading.Event()
        self.stop_uart_evt = threading.Event()
        self.udp_queue     = queue.Queue()

        # 資料緩衝 --------------------------------------------------------------
        self.motor_hist = {                       # deque(maxlen) => O(1) 記憶體
            k: deque(maxlen=self.HIST_LEN) for k in
            ("R_ang","R_spd","R_cur","L_ang","L_spd","L_cur")
        }
        self.imu_state = {"roll":0.0, "pitch":0.0, "yaw":0.0}

        # 2. GUI ---------------------------------------------------------------
        self.in_demo = False
        self.title("STA Swing Controller")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.resizable(False, False)
        self._build_widgets()

        # 3. I/O  --------------------------------------------------------------
        if self.src_var.get() == "wifi":
            self._connect_tcp_async()
            self._start_udp_listener()
        else:
            self._start_uart_listener()

        # 4. after-loop pump ---------------------------------------------------
        self._pump_id = self.after(int(1000/self.PLOT_HZ), self._pump_udp_queue)
        self.src_var.trace_add("write", lambda *_, s=self: s._on_src_change())

    # ──────────── async TCP connect (non-blocking) ────────────
    def _connect_tcp_async(self):
        def worker():
            try:
                sock = socket.create_connection((ESP32_IP, ESP32_PORT), timeout=2)
                self.after(0, self._tcp_ready, sock)
            except Exception as exc:
                self.after(0, self._log, f"⚠️ TCP connect failed: {exc}")
        threading.Thread(target=worker, daemon=True).start()

    def _tcp_ready(self, sock: socket.socket):
        self.tcp_sock = sock
        self._log(f"✓ TCP connected {ESP32_IP}:{ESP32_PORT}")

    # ────────────────────── UI build ────────────────────────
    def _build_widgets(self):
        pad = {"padx": 6, "pady": 4}
        frm = ttk.Frame(self); frm.grid(row=0, column=0, **pad)

        # ── Angle / Speed inputs ────────────────────────────────────────────
        ttk.Label(frm, text="Angle A (deg):").grid(row=0, column=0, **pad)
        self.entry_a = ttk.Entry(frm, width=6); self.entry_a.insert(0, "-30")
        self.entry_a.grid(row=0, column=1, **pad)

        ttk.Label(frm, text="Angle B (deg):").grid(row=0, column=2, **pad)
        self.entry_b = ttk.Entry(frm, width=6); self.entry_b.insert(0, "30")
        self.entry_b.grid(row=0, column=3, **pad)

        ttk.Label(frm, text="Speed (deg/s):").grid(row=1, column=0, **pad)
        self.entry_speed = ttk.Entry(frm, width=6); self.entry_speed.insert(0, "25")
        self.entry_speed.grid(row=1, column=1, **pad)

        # ── Foot radio ──────────────────────────────────────────────────────
        ttk.Label(frm, text="Foot:").grid(row=1, column=2, sticky="e", **pad)
        for i, foot in enumerate(("left", "right", "both")):
            ttk.Radiobutton(frm, text=foot, value=foot, variable=self.foot_var)\
                .grid(row=1, column=3+i, sticky="w", **pad)

        # ── Source radio & COM 選單 ────────────────────────────────────────
        ttk.Label(frm, text="Source:").grid(row=2, column=0, sticky="e", **pad)
        ttk.Radiobutton(frm, text="WiFi (UDP)", value="wifi", variable=self.src_var)\
            .grid(row=2, column=1, sticky="w", **pad)
        ttk.Radiobutton(frm, text="UART", value="uart", variable=self.src_var,
            state="disabled" if serial is None else "normal")\
            .grid(row=2, column=2, sticky="w", **pad)

        if serial is not None and list_ports is not None:
            ports = [p.device for p in list_ports.comports()] or ["No COM"]
        else:
            ports = ["No COM (pyserial missing)"]
        self.combo_com = ttk.Combobox(frm, values=ports, width=12, state="readonly")
        self.combo_com.set("/dev/ttyUSB0" if "/dev/ttyUSB0" in ports else ports[0])
        self.combo_com.grid(row=2, column=3, **pad)

        # ── Buttons ─────────────────────────────────────────────────────────
        btns = ttk.Frame(frm); btns.grid(row=3, column=0, columnspan=6, **pad)
        ttk.Button(btns, text="Run Swing",  command=self._start_swing_thread)\
            .grid(row=0, column=0, **pad)
        ttk.Button(btns, text="Toggle LED", command=self._toggle_led)\
            .grid(row=0, column=1, **pad)
        ttk.Button(btns, text="Demo Mode",  command=self._run_demo_mode)\
            .grid(row=0, column=3, **pad)
        self.btn_rec = ttk.Button(btns, text="Start Record", command=self._toggle_record)
        self.btn_rec.grid(row=0, column=2, **pad)

        # ── IMU log ─────────────────────────────────────────────────────────
        self.text_imu = scrolledtext.ScrolledText(self, width=86, height=14,
                                                  font=("Consolas", 10), state="disabled")
        self.text_imu.grid(row=1, column=0, sticky="nsew", padx=6, pady=(4,0))

        self.demo_status = ttk.Label(frm, text="Demo Mode: OFF", foreground="gray")
        self.demo_status.grid(row=4, column=0, columnspan=4, sticky="w", padx=6, pady=4)

        # ── Live plots ──────────────────────────────────────────────────────
        self._init_plot_area(row=2)

    # ── 建立 matplotlib Figure 及三條曲線 + 3-D 飛機 ──────────────────
    def _init_plot_area(self, row=2):
        fig = plt.Figure(figsize=(7.8, 3), dpi=100)
        self.axes = {
            "ang":  fig.add_subplot(131),
            "spd":  fig.add_subplot(132),
            "cur":  fig.add_subplot(133),
        }
        self.line2hist = {
            "ang_R": "R_ang", "ang_L": "L_ang",
            "spd_R": "R_spd", "spd_L": "L_spd",
            "cur_R": "R_cur", "cur_L": "L_cur",
        }
        for ax, ttl, ylim in (
            (self.axes["ang"], "Angle (°)",  (-120, 120)),
            (self.axes["spd"], "Speed (°/s)",(-120, 120)),
            (self.axes["cur"], "Current (A)",(-3, 3)),
        ):
            ax.set_title(ttl); ax.set_xlim(0, self.HIST_LEN); ax.set_ylim(*ylim)
            ax.grid(True)

        # Angle 線 (左右腳)
        self.lines = {
            "ang_R": self.axes["ang"].plot([], [], "r-")[0],
            "ang_L": self.axes["ang"].plot([], [], "b-")[0],
            "spd_R": self.axes["spd"].plot([], [], "r-")[0],
            "spd_L": self.axes["spd"].plot([], [], "b-")[0],
            "cur_R": self.axes["cur"].plot([], [], "r-")[0],
            "cur_L": self.axes["cur"].plot([], [], "b-")[0],
        }

        # 3-D 飛機放到另一個 Toplevel，可選擇要/不要
        air_top = tk.Toplevel(self); air_top.title("IMU Attitude (roll-pitch-yaw)")
        air_top.resizable(False, False)
        fig3d = plt.Figure(figsize=(3,3), dpi=100)
        ax3d = fig3d.add_subplot(111, projection='3d')
        ax3d.set_xlim(-1,1); ax3d.set_ylim(-1,1); ax3d.set_zlim(-1,1)
        ax3d.set_box_aspect([1,1,1]); ax3d.axis("off")
        self.air_body, = ax3d.plot([], [], [], "k-", lw=2)
        self.air_wing, = ax3d.plot([], [], [], "g-", lw=2)
        self.air_canvas = FigureCanvasTkAgg(fig3d, master=air_top)
        self.air_canvas.get_tk_widget().pack(fill="both", expand=1)

        self.canvas = FigureCanvasTkAgg(fig, master=self)
        self.canvas.get_tk_widget().grid(row=row, column=0, padx=6, pady=6)

    # ─────────────────── Network helpers ────────────────────
    def _start_uart_listener(self):
        if serial is None:
            self._log("⚠️ pyserial not installed; UART disabled")
            return

        def _uart_loop():
            try:
                port = self.combo_com.get()
                if port.startswith("No COM"):
                    self.udp_queue.put("⚠️ No COM port selected; UART not started")
                    return

                if self.ser and self.ser.is_open:
                    try: self.ser.close(); self._log("🔌 Closed existing UART")
                    except Exception as e: self._log(f"⚠️ UART close failed: {e}")

                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.udp_queue.put(f"✓ UART listening on {port}")

                while not self.stop_uart_evt.is_set():
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        msg = f"[UART] {line}"
                        self.udp_queue.put(msg)
                        if self.recording:
                            self.log_lines.append(f"{datetime.now().isoformat()} {msg}")

                self.ser.close(); self.ser = None
            except Exception as exc:
                self.udp_queue.put(f"UART error: {exc}")

        threading.Thread(target=_uart_loop, daemon=True).start()

    def _start_udp_listener(self):
        def _listener():
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                udp.bind(("", UDP_PORT)); udp.settimeout(0.5)
                self.udp_queue.put(f"✓ UDP listening on :{UDP_PORT}")
                while not self.stop_udp_evt.is_set():
                    try:
                        data, addr = udp.recvfrom(BUF_SIZE)
                        line = data.decode(errors="ignore").strip()
                        msg = f"[IMU {addr[0]}] {line}"
                        self.udp_queue.put(msg)
                        if self.recording:
                            self.log_lines.append(f"{datetime.now().isoformat()} {msg}")
                    except socket.timeout:
                        continue
            finally:
                udp.close()
        threading.Thread(target=_listener, daemon=True).start()

    # ───────────────────── Parsing  & Plot ──────────────────
    def _parse_line(self, raw: str):
        """
        入: 'X 0.0 0 0 -0.4 15 22 -56.4 -82.5 37.8'
        存入 motor_hist, imu_state
        """
        try:
            toks = raw.split()
            if toks[0] != "X" or len(toks) < 10:
                return
            vals = list(map(float, toks[1:]))
            R_ang,R_spd,R_cur,L_ang,L_spd,L_cur,roll,pitch,yaw = vals
            self.motor_hist["R_ang"].append(R_ang)
            self.motor_hist["R_spd"].append(R_spd)
            self.motor_hist["R_cur"].append(R_cur*0.01)
            self.motor_hist["L_ang"].append(L_ang)
            self.motor_hist["L_spd"].append(L_spd)
            self.motor_hist["L_cur"].append(L_cur*0.01)
            self.imu_state.update(roll=roll,pitch=pitch,yaw=yaw)
        except Exception:
            pass

    def _update_plots(self):
        xs = range(len(self.motor_hist["R_ang"]))
        for ln_key, line in self.lines.items():
            buf = self.motor_hist[self.line2hist[ln_key]]
            line.set_data(xs, list(buf))
        for ax in self.axes.values():
            ax.set_xlim(0, self.HIST_LEN)
        self.canvas.draw_idle()
        self._update_aircraft()

    def _update_aircraft(self):
        r = np.deg2rad(self.imu_state["roll"])
        p = np.deg2rad(self.imu_state["pitch"])
        y = np.deg2rad(self.imu_state["yaw"])
        Rx = np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
        Ry = np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
        Rz = np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
        Rm = Rz @ Ry @ Rx
        body_pts = np.array([[ 1, 0, 0], [-1, 0, 0]]).T               # 機身
        wing_pts = np.array([[0, -0.6, 0.6], [0,0,0],[0,0,0]]).T      # 左右機翼
        b = (Rm @ body_pts).T
        w = (Rm @ wing_pts).T
        self.air_body.set_data(b[:,0], b[:,1]); self.air_body.set_3d_properties(b[:,2])
        self.air_wing.set_data(w[:,0], w[:,1]); self.air_wing.set_3d_properties(w[:,2])
        self.air_canvas.draw_idle()

    # ───────────────────── Logging & UI ────────────────────
    def _log(self, s: str):
        if threading.current_thread() is not threading.main_thread():
            self.after(0, self._log, s); return
        if self.recording:
            self.log_lines.append(f"{datetime.now().isoformat()} {s}")
        self.text_imu.configure(state="normal")
        self.text_imu.insert("end", s + "\n")
        self.text_imu.configure(state="disabled"); self.text_imu.yview_moveto(1.0)

    def _pump_udp_queue(self):
        while not self.udp_queue.empty():
            msg = self.udp_queue.get()
            self._log(msg)
            if "] X " in msg:                          # [IMU …] X 0 …
                self._parse_line(msg.split("]",1)[1].strip())
        self._update_plots()
        self._pump_id = self.after(int(1000/self.PLOT_HZ), self._pump_udp_queue)

    # ─────────────────── Send helpers ─────────────────────
    def _send(self, pkt: bytes, desc: str = ""):
        src = self.src_var.get()
        if src == "wifi":
            if not self.tcp_sock:
                self._log("⚠️ TCP not connected"); return
            try:
                self.tcp_sock.sendall(pkt); self._log(f"→ [TCP] {desc}")
            except Exception as exc:
                self._log(f"TCP send error: {exc}")
        elif src == "uart":
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(pkt); self._log(f"→ [UART] {desc}")
                else:
                    self._log("⚠️ UART port not open")
            except Exception as exc:
                self._log(f"UART send error: {exc}")

    def _toggle_led(self):
        self._send(make_led_packet(), "LED toggle (0x10)")

    def _send_stop(self):
        self._send(*make_motor_packet("E", 0, "E", 0))

    # ─────────────── Swing logic (thread) ────────────────
    def _start_swing_thread(self):
        threading.Thread(target=self._run_swing, daemon=True).start()

    def _run_swing(self):
        def _deg2centideg(x):  return int(round(x * 100))

        try:
            a = float(self.entry_a.get()); b = float(self.entry_b.get())
        except ValueError:
            messagebox.showwarning("Input", "角度必須為數值"); return
        foot = self.foot_var.get()

        def get_angles(pos):
            if foot == "right":        return _deg2centideg(pos), 0
            elif foot == "left":       return 0, _deg2centideg(pos)
            else:                      return _deg2centideg(pos), -_deg2centideg(pos)

        # → 往右
        self._send(*make_motor_packet("A", *get_angles(b))); self._log(f"→ Swing to {b}°")
        time.sleep(2)
        # ← 往左
        self._send(*make_motor_packet("A", *get_angles(a))); self._log(f"← Swing back to {a}°")
        time.sleep(2)
        self._send_stop(); self._log("✓ Swing cycle complete")

    # ─────────────── Demo & Record ────────────────
    def _run_demo_mode(self):
        if not self.in_demo:
            self._send(*make_motor_packet("G", 0, "G", 0))
            self._log("🚶 Demo mode started (G)")
            self.demo_status.configure(text="Demo Mode: ON", foreground="green")
            self.in_demo = True
        else:
            self._send_stop()
            self._log("🛑 Demo mode stopped")
            self.demo_status.configure(text="Demo Mode: OFF", foreground="gray")
            self.in_demo = False

    def _toggle_record(self):
        if not self.recording:
            self.recording = True; self.log_lines.clear()
            self.btn_rec.configure(text="Stop Record")
            self._log("★ Recording started")
        else:
            self.recording = False
            fname = datetime.now().strftime("sta_log_%Y%m%d_%H%M%S.txt")
            try:
                with open(fname,"w",encoding="utf-8") as f:
                    f.write("\n".join(self.log_lines))
                self._log(f"★ Log saved → {fname}")
            except Exception as exc:
                self._log(f"Save error: {exc}")
            self.btn_rec.configure(text="Start Record")

    # ─────────────── Window close ────────────────
    def _on_close(self):
        if not messagebox.askokcancel("Quit", "確定退出程式？"): return
        self.after_cancel(self._pump_id); self.stop_udp_evt.set(); self.stop_uart_evt.set()
        if self.tcp_sock:
            try: self._send_stop(); self.tcp_sock.close()
            except: pass
        if self.ser and self.ser.is_open:
            try: self.ser.close(); self._log("🔌 UART port closed on exit")
            except: pass
        if self.recording: self._toggle_record()
        self.destroy()

        # ──────────────────── Source 切換 ─────────────────────
    def _on_src_change(self):
        """Wi-Fi ↔︎ UART 切換時重新啟動對應 listener。"""
        src = self.src_var.get()

        # 先停掉現有 listener
        if src == "wifi":
            self.stop_uart_evt.set(); self.stop_udp_evt.clear()
        else:  # uart
            self.stop_udp_evt.set();  self.stop_uart_evt.clear()

        # 再啟動目標 listener
        if src == "wifi":
            self._log("🔁 Switched to Wi-Fi mode")
            self._connect_tcp_async()
            self._start_udp_listener()
        else:
            self._log("🔁 Switched to UART mode")
            self._start_uart_listener()

##############################################################################
# Main
##############################################################################
if __name__ == "__main__":
    STAControllerGUI().mainloop()
