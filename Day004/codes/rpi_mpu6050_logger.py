import time
import math
import csv
import threading
import sys
from mpu6050 import mpu6050

#Installs
# sudo apt-get update
# sudo apt-get install python3-pip i2c-tools
# pip3 install mpu6050-raspberrypi
# --- CONFIGURATION ---
SENSOR_ADDR = 0x68  # Default MPU6050 address
SAMPLE_RATE_HZ = 100  # Target Loop Speed (100Hz)
DT = 1.0 / SAMPLE_RATE_HZ
ALPHA = 0.2  # Smoothing Factor (0.1=Slow/Smooth, 0.9=Fast/Noisy)
FUSION_ALPHA = 0.96  # Complementary Filter (Trust Gyro 96%, Accel 4%)

# Global Flags 
is_running = True
is_recording = False
log_filename = "mpu6050_log.csv"


# --- HELPER FUNCTIONS ---

def calculate_roll_pitch(ax, ay, az):
    """
    Calculates Roll and Pitch from Accelerometer (Gravity Vector).
    """
    # Avoid division by zero
    hypot = math.sqrt(ay * ay + az * az)
    if hypot == 0: hypot = 0.0001

    # Roll: Rotation around X-axis
    roll_acc = math.degrees(math.atan2(ay, az))

    # Pitch: Rotation around Y-axis
    pitch_acc = math.degrees(math.atan2(-ax, hypot))

    return roll_acc, pitch_acc


def input_thread():
    """ Runs in background to check for 's' key """
    global is_recording, is_running
    print("\n--- CONTROLS ---")
    print("Press 's' + ENTER to Toggle Recording")
    print("Press 'q' + ENTER to Quit")
    print("----------------")

    while is_running:
        try:
            cmd = sys.stdin.readline().strip()
            if cmd.lower() == 's':
                is_recording = not is_recording
                if is_recording:
                    print(f"\n[RECORDING STARTED] >>> {log_filename}")
                else:
                    print("\n[RECORDING STOPPED]")
            elif cmd.lower() == 'q':
                is_running = False
                print("\nQuitting...")
        except:
            break


# --- MAIN EXECUTION ---

def main():
    global is_running

    print("Initializing MPU6050...")
    try:
        sensor = mpu6050(SENSOR_ADDR)
        print("Sensor Connected!")
    except Exception as e:
        print(f"Error connecting to MPU6050: {e}")
        return

    # Create CSV Header
    with open(log_filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "ax_s", "ay_s", "az_s", "gx_s", "gy_s", "gz_s", "roll", "pitch", "yaw"])

    # Start Input Listener
    t = threading.Thread(target=input_thread)
    t.daemon = True
    t.start()

    # Variables for Filtering
    # Init smoothed values to 0
    ax_s, ay_s, az_s = 0.0, 0.0, 0.0
    gx_s, gy_s, gz_s = 0.0, 0.0, 0.0

    # Orientation State
    roll, pitch, yaw = 0.0, 0.0, 0.0

    # Initialize "Previous" time for accurate DT calculation
    last_time = time.time()
    start_time = time.time()

    print("Sensor Loop Running...")

    while is_running:
        current_time = time.time()
        elapsed = current_time - last_time

        # Enforce Loop Timing
        if elapsed < DT:
            continue

        last_time = current_time
        dt_actual = elapsed  # Use actual time elapsed for integration precision

        try:
            # 1. READ RAW DATA
            # Library returns dictionary {'x': val, 'y': val, 'z': val}
            accel_data = sensor.get_accel_data()
            gyro_data = sensor.get_gyro_data()

            # MPU6050 Lib usually returns m/s^2 for Accel and deg/s for Gyro
            # We convert Accel to Gs (divide by 9.81) for standard calculations
            ax_raw = accel_data['x'] / 9.81
            ay_raw = accel_data['y'] / 9.81
            az_raw = accel_data['z'] / 9.81

            gx_raw = gyro_data['x']
            gy_raw = gyro_data['y']
            gz_raw = gyro_data['z']

            # 2. SMOOTHING (Low Pass Filter)
            # smoothed = (alpha * new) + ((1-alpha) * old)
            ax_s = (ALPHA * ax_raw) + ((1.0 - ALPHA) * ax_s)
            ay_s = (ALPHA * ay_raw) + ((1.0 - ALPHA) * ay_s)
            az_s = (ALPHA * az_raw) + ((1.0 - ALPHA) * az_s)

            gx_s = (ALPHA * gx_raw) + ((1.0 - ALPHA) * gx_s)
            gy_s = (ALPHA * gy_raw) + ((1.0 - ALPHA) * gy_s)
            gz_s = (ALPHA * gz_raw) + ((1.0 - ALPHA) * gz_s)

            # 3. COMPUTE ANGLES
            # A. Accel Only (Noisy but no drift)
            roll_acc, pitch_acc = calculate_roll_pitch(ax_s, ay_s, az_s)

            # B. Sensor Fusion (Complementary Filter)
            # Combine Gyro Integration (precise) with Accel (stable reference)
            roll = (FUSION_ALPHA * (roll + gx_s * dt_actual)) + ((1.0 - FUSION_ALPHA) * roll_acc)
            pitch = (FUSION_ALPHA * (pitch + gy_s * dt_actual)) + ((1.0 - FUSION_ALPHA) * pitch_acc)

            # C. Yaw (Gyro only - will drift because no compass)
            yaw = yaw + (gz_s * dt_actual)

            # 4. LOGGING & DISPLAY
            timestamp = current_time - start_time

            # Console Display (Update ~5 times a second)
            if int(timestamp * 10) % 2 == 0:
                status = "[REC]" if is_recording else "[IDLE]"
                # Using carriage return \r to overwrite line
                sys.stdout.write(f"\r{status} R:{roll:6.1f} P:{pitch:6.1f} Y:{yaw:6.1f}   ")
                sys.stdout.flush()

            if is_recording:
                with open(log_filename, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        f"{timestamp:.3f}",
                        f"{ax_s:.3f}", f"{ay_s:.3f}", f"{az_s:.3f}",
                        f"{gx_s:.3f}", f"{gy_s:.3f}", f"{gz_s:.3f}",
                        f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}"
                    ])

        except Exception as e:
            # I2C errors happen occasionally, just skip this frame
            pass


if __name__ == "__main__":
    main()
