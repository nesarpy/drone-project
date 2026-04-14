import serial
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from collections import deque
import csv
import time
import os
import threading
import queue
from datetime import datetime

# ===== CONFIG =====

PORT = "COM18"
BAUD = 250000
MAX_POINTS = 100
FLUSH_EVERY = 25          # rows before CSV flush
RENDER_INTERVAL = 0.05    # seconds between redraws (~20 fps)
MOTOR_WARN_DELTA = 150    # µs deviation from average to trigger warning

# ===== SERIAL =====

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# ===== LOGGING =====

os.makedirs("logs", exist_ok=True)
filename = f"logs/flight_log_{datetime.now().strftime('%d-%m-%Y__%H%M')}.csv"
file = open(filename, mode="w", newline="")
writer = csv.writer(file)
writer.writerow(["time_ms", "throttle", "pitch", "roll", "yaw", "m1", "m2", "m3", "m4", "batV", "armed"])
print(f"Logging to {filename}")

# ===== THREAD COORDINATION =====

data_queue = queue.Queue()
stop_event = threading.Event()

# ===== DATA BUFFERS =====

pitch_data    = deque(maxlen=MAX_POINTS)
roll_data     = deque(maxlen=MAX_POINTS)
yaw_data      = deque(maxlen=MAX_POINTS)
throttle_data = deque(maxlen=MAX_POINTS)
m1_data       = deque(maxlen=MAX_POINTS)
m2_data       = deque(maxlen=MAX_POINTS)
m3_data       = deque(maxlen=MAX_POINTS)
m4_data       = deque(maxlen=MAX_POINTS)

# ===== PARSER =====

def parse_line(line: str):
    # Expected: ARM: 0 | T: 1499 | P: 0.10 | R: -0.01 | Y: 0.22 | M: 1000 1000 1000 1000 | BatV: 13.50
    try:
        parts    = [p.strip() for p in line.split("|")]
        throttle = int(parts[1].split(":")[1])
        pitch    = float(parts[2].split(":")[1])
        roll     = float(parts[3].split(":")[1])
        yaw      = float(parts[4].split(":")[1])

        m1, m2, m3, m4 = map(int, parts[5].split(":")[1].split())
        batV     = float(parts[6].split(":")[1])

        return throttle, pitch, roll, yaw, m1, m2, m3, m4, batV
    except (IndexError, ValueError):
        return None

# ===== SERIAL READER THREAD =====

row_count = 0

def serial_reader():
    global row_count
    while not stop_event.is_set():
        try:
            raw  = ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
        except serial.SerialException:
            break

        if "T:" not in line:
            continue

        result = parse_line(line)
        if result is None:
            continue

        ts = int(time.time() * 1000)
        throttle, _, _, _, m1, m2, m3, m4, _ = result
        armed = 1 if is_armed(throttle, m1, m2, m3, m4) else 0
        writer.writerow([ts, *result, armed])
        row_count += 1
        if row_count % FLUSH_EVERY == 0:
            file.flush()

        # Drop stale queue entries to avoid lag
        if data_queue.qsize() > 10:
            try:
                data_queue.get_nowait()
            except queue.Empty:
                pass

        data_queue.put(result)

reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# ===== PLOT SETUP =====

plt.ion()
plt.style.use("dark_background")

PANEL_BG      = "#131620"
MOTOR_COLORS  = ["#ef5350", "#42a5f5", "#66bb6a", "#ffa726"]

fig = plt.figure(figsize=(12, 8), facecolor="#0d0f14")
fig.suptitle("FLIGHT TELEMETRY", fontsize=13, color="#e0e0e0",
             fontfamily="monospace", y=0.995)

gs     = gridspec.GridSpec(2, 2, figure=fig,
                           left=0.07, right=0.97,
                           top=0.92,  bottom=0.08,
                           hspace=0.45, wspace=0.3)
ax_att = fig.add_subplot(gs[0, :])
ax_thr = fig.add_subplot(gs[1, 0])
ax_mot = fig.add_subplot(gs[1, 1])

for ax in (ax_att, ax_thr, ax_mot):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors="#555", labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor("#2a2d3a")

# Attitude panel
line_pitch, = ax_att.plot([], [], color="#4fc3f7", lw=1.5, label="Pitch")
line_roll,  = ax_att.plot([], [], color="#81c784", lw=1.5, label="Roll")
line_yaw,   = ax_att.plot([], [], color="#ffb74d", lw=1.5, label="Yaw")
ax_att.axhline(0, color="#2a2d3a", lw=0.8, ls="--")
ax_att.set_ylim(-45, 45)
ax_att.set_xlim(0, MAX_POINTS)
ax_att.set_title("Attitude  (Pitch / Roll / Yaw)", color="#aaa",
                 fontsize=9, fontfamily="monospace", pad=6)
ax_att.set_ylabel("degrees", color="#555", fontsize=8)
ax_att.legend(loc="upper right", fontsize=8, framealpha=0.2)

# Throttle panel
line_throttle, = ax_thr.plot([], [], color="#ce93d8", lw=1.5)
ax_thr.set_ylim(900, 2100)
ax_thr.set_xlim(0, MAX_POINTS)
ax_thr.set_title("Throttle", color="#aaa", fontsize=9, fontfamily="monospace", pad=6)
ax_thr.set_ylabel("µs", color="#555", fontsize=8)
ax_thr.axhline(1000, color="#2a2d3a", lw=0.7, ls=":")

# Motor panel
line_m1, = ax_mot.plot([], [], color=MOTOR_COLORS[0], lw=1.2, label="M1")
line_m2, = ax_mot.plot([], [], color=MOTOR_COLORS[1], lw=1.2, label="M2")
line_m3, = ax_mot.plot([], [], color=MOTOR_COLORS[2], lw=1.2, label="M3")
line_m4, = ax_mot.plot([], [], color=MOTOR_COLORS[3], lw=1.2, label="M4")
ax_mot.set_ylim(900, 2100)
ax_mot.set_xlim(0, MAX_POINTS)
ax_mot.set_title("Motors", color="#aaa", fontsize=9, fontfamily="monospace", pad=6)
ax_mot.legend(loc="upper right", fontsize=7, framealpha=0.2, ncol=2, handlelength=1)

hud_text  = fig.text(0.03, 0.96, "", fontsize=9, color="#00e5ff",
                     fontfamily="monospace", va="top")
warn_text = fig.text(0.03, 0.93, "", fontsize=9, color="#ff5252",
                     fontfamily="monospace", va="top")

# ===== HELPERS =====

def attitude_ylim_guard(ax, *datasets):
    """Expand attitude axis if data exceeds current limits."""
    lo, hi   = ax.get_ylim()
    all_vals = [v for d in datasets for v in d]
    if not all_vals:
        return
    dmin, dmax = min(all_vals), max(all_vals)
    if dmin < lo + 2 or dmax > hi - 2:
        ax.set_ylim(min(lo, dmin - 10), max(hi, dmax + 10))

def motor_warning(m1, m2, m3, m4) -> str:
    """Return a warning string if any motor deviates too far from the average."""
    avg    = (m1 + m2 + m3 + m4) / 4
    deltas = [abs(m - avg) for m in (m1, m2, m3, m4)]
    worst  = max(deltas)
    if worst > MOTOR_WARN_DELTA:
        bad = ["M1", "M2", "M3", "M4"][deltas.index(worst)]
        return f"⚠ MOTOR IMBALANCE  {bad} Δ{int(worst)}µs"
    return ""

def is_armed(throttle, m1, m2, m3, m4) -> bool:
    return throttle >= 1050 and (m1 + m2 + m3 + m4) / 4 >= 1050

# ===== RENDER LOOP =====

last_result = None

try:
    while True:
        new_data = False

        # Drain the queue, keeping only the latest values in the buffers
        while not data_queue.empty():
            try:
                result = data_queue.get_nowait()
            except queue.Empty:
                break

            throttle, pitch, roll, yaw, m1, m2, m3, m4, batV = result
            pitch_data.append(pitch);      roll_data.append(roll)
            yaw_data.append(yaw);          throttle_data.append(throttle)
            m1_data.append(m1);            m2_data.append(m2)
            m3_data.append(m3);            m4_data.append(m4)
            last_result = result
            new_data    = True

        if new_data and last_result is not None:
            throttle, pitch, roll, yaw, m1, m2, m3, m4, batV = last_result
            x = range(len(pitch_data))

            line_pitch.set_data(x, pitch_data)
            line_roll.set_data(x,  roll_data)
            line_yaw.set_data(x,   yaw_data)
            attitude_ylim_guard(ax_att, pitch_data, roll_data, yaw_data)

            line_throttle.set_data(x, throttle_data)

            line_m1.set_data(x, m1_data);  line_m2.set_data(x, m2_data)
            line_m3.set_data(x, m3_data);  line_m4.set_data(x, m4_data)

            n = len(pitch_data)
            for ax in (ax_att, ax_thr, ax_mot):
                ax.set_xlim(0, max(MAX_POINTS, n))

            state = "ARMED" if is_armed(throttle, m1, m2, m3, m4) else "DISARMED"
            hud_text.set_text(
                f"{state} │ T:{throttle:4d}  "
                f"P:{pitch:+6.1f}°  R:{roll:+6.1f}°  Y:{yaw:+6.1f}°  │  "
                f"V:{batV:4.2f}V │ "
                f"M1:{m1} M2:{m2} M3:{m3} M4:{m4}"
            )
            warn_text.set_text(motor_warning(m1, m2, m3, m4))
            fig.canvas.draw_idle()

        plt.pause(RENDER_INTERVAL)

except KeyboardInterrupt:
    print(f"\nStopped — {row_count} rows saved to {filename}")
    stop_event.set()
    file.close()
    ser.close()