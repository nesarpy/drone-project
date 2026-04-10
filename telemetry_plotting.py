import serial
import matplotlib.pyplot as plt
from collections import deque
import csv
import time
from datetime import datetime

# ===== SERIAL =====
PORT = "COM18"
BAUD = 250000

ser = serial.Serial(PORT, BAUD, timeout=0.01)

# ===== LOGGING SETUP =====
filename = f"logs/flight_log_{datetime.now().strftime('%d-%m-%Y__%H%M')}.csv"

file = open(filename, mode="w", newline="")
writer = csv.writer(file)

writer.writerow([
    "time_ms",
    "throttle",
    "pitch",
    "roll",
    "yaw",
    "m1",
    "m2",
    "m3",
    "m4"
])

print(f"Logging to {filename}")

# ===== DATA STORAGE =====
max_points = 80

pitch_data = deque(maxlen=max_points)
roll_data = deque(maxlen=max_points)
yaw_data = deque(maxlen=max_points)

throttle_data = deque(maxlen=max_points)
m1_data = deque(maxlen=max_points)
m2_data = deque(maxlen=max_points)
m3_data = deque(maxlen=max_points)
m4_data = deque(maxlen=max_points)

# ===== PLOTTING =====
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

# Attitude
line_pitch, = ax1.plot([], [], label="Pitch")
line_roll,  = ax1.plot([], [], label="Roll")
line_yaw,   = ax1.plot([], [], label="Yaw")
ax1.set_ylim(-40, 40)
ax1.set_title("Pitch / Roll / Yaw")
ax1.legend()

# Motors
line_throttle, = ax2.plot([], [], label="Throttle")
line_m1, = ax2.plot([], [], label="M1")
line_m2, = ax2.plot([], [], label="M2")
line_m3, = ax2.plot([], [], label="M3")
line_m4, = ax2.plot([], [], label="M4")
ax2.set_ylim(900, 2100)
ax2.set_title("Throttle & Motors")
ax2.legend()

text_display = fig.text(0.02, 0.95, "", fontsize=12)

# ===== PARSER =====
def parse_line(line):
    try:
        parts = line.split("|")

        throttle = int(parts[1].split(":")[1])
        pitch = float(parts[2].split(":")[1])
        roll  = float(parts[3].split(":")[1])
        yaw   = float(parts[4].split(":")[1])

        motors = parts[5].split(":")[1].strip().split(" ")
        m1, m2, m3, m4 = map(int, motors)

        return throttle, pitch, roll, yaw, m1, m2, m3, m4
    except:
        return None

# ===== MAIN LOOP =====
try:
    while True:
        ser.reset_input_buffer()

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if "T:" not in line:
            continue

        result = parse_line(line)

        if not result:
            continue

        throttle, pitch, roll, yaw, m1, m2, m3, m4 = result

        # ===== LOGGING =====
        current_time = int(time.time() * 1000)

        writer.writerow([
            current_time,
            throttle,
            pitch,
            roll,
            yaw,
            m1,
            m2,
            m3,
            m4
        ])

        # flush every ~0.5 sec
        if current_time % 500 < 20:
            file.flush()

        # ===== STORE DATA =====
        pitch_data.append(pitch)
        roll_data.append(roll)
        yaw_data.append(yaw)

        throttle_data.append(throttle)
        m1_data.append(m1)
        m2_data.append(m2)
        m3_data.append(m3)
        m4_data.append(m4)

        x = range(len(pitch_data))

        # ===== UPDATE GRAPHS =====
        line_pitch.set_data(x, pitch_data)
        line_roll.set_data(x, roll_data)
        line_yaw.set_data(x, yaw_data)

        line_throttle.set_data(x, throttle_data)
        line_m1.set_data(x, m1_data)
        line_m2.set_data(x, m2_data)
        line_m3.set_data(x, m3_data)
        line_m4.set_data(x, m4_data)

        ax1.set_xlim(0, max_points)
        ax2.set_xlim(0, max_points)

        # ===== TEXT DISPLAY =====
        text_display.set_text(
            f"T:{throttle}  P:{pitch:.1f}  R:{roll:.1f}  Y:{yaw:.1f}   "
            f"M:{m1},{m2},{m3},{m4}"
        )

        fig.canvas.draw_idle()
        plt.pause(0.02)
except KeyboardInterrupt:
    print("\nStopping and saving log...")
    file.close()
    ser.close()
    print(f"Log saved: {filename}")