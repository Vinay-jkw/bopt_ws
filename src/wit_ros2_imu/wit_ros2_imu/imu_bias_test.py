#!/usr/bin/env python3

import serial
import time
import csv
import os
import math
import statistics
from datetime import datetime


# ============================================================
# CONFIGURATION
# ============================================================

PORT = "/dev/ttyUSB0"
BAUDRATE = 230400

# Total experiment duration
TEST_DURATION = 300.0

# IMPORTANT:
# IMU must remain COMPLETELY STATIONARY during this period.
CALIBRATION_TIME = 60.0

OUTPUT_DIR = "/home/jkw/imu_test_ws/imu_test_results"


# ============================================================
# GYRO CALIBRATION
# ============================================================

GYRO_SCALE = 1.0

# Keep scale correction disabled for now.
# We will determine scale only using a physical angle reference.
#
# gyro_corrected = (gyro_raw - bias) * GYRO_SCALE


# ============================================================
# FILTER
# ============================================================

# FILTER IS OFF FOR THIS TEST.
#
# We are testing:
#
# RAW
# vs
# BIAS CORRECTED
#
ENABLE_LPF = False

LPF_CUTOFF_HZ = 5.0


# ============================================================
# BIAS CALIBRATION
# ============================================================

# During the 60-second stationary calibration,
# only samples below this threshold are accepted.
#
# This prevents an accidental large movement from being
# included in the bias calculation.

BIAS_STATIONARY_THRESHOLD = 2.0

MIN_BIAS_SAMPLES = 100


# ============================================================
# WITMOTION PROTOCOL
# ============================================================

HEADER = 0x55

ACCEL_ID = 0x51
GYRO_ID = 0x52
ANGLE_ID = 0x53
MAG_ID = 0x54


# ============================================================
# GLOBAL SENSOR STATE
# ============================================================

angle_yaw_deg = 0.0

packet_count = {
    "accel": 0,
    "gyro": 0,
    "angle": 0,
    "mag": 0,
    "invalid": 0,
}


# ============================================================
# CHECKSUM
# ============================================================

def checksum_ok(packet):

    if len(packet) != 11:
        return False

    return (sum(packet[:10]) & 0xFF) == packet[10]


# ============================================================
# SIGNED INT16
# ============================================================

def signed_int16(low, high):

    value = low | (high << 8)

    if value >= 32768:
        value -= 65536

    return value


# ============================================================
# PACKET DECODER
# ============================================================

def decode_packet(packet):

    global angle_yaw_deg

    if len(packet) != 11:
        return None

    if packet[0] != HEADER:
        return None

    if not checksum_ok(packet):
        packet_count["invalid"] += 1
        return None

    packet_id = packet[1]

    x = signed_int16(packet[2], packet[3])
    y = signed_int16(packet[4], packet[5])
    z = signed_int16(packet[6], packet[7])


    # --------------------------------------------------------
    # ACCELEROMETER
    # --------------------------------------------------------

    if packet_id == ACCEL_ID:

        ax = x / 32768.0 * 16.0 * 9.80665
        ay = y / 32768.0 * 16.0 * 9.80665
        az = z / 32768.0 * 16.0 * 9.80665

        packet_count["accel"] += 1

        return {
            "type": "accel",
            "ax": ax,
            "ay": ay,
            "az": az,
        }


    # --------------------------------------------------------
    # GYROSCOPE
    # --------------------------------------------------------

    elif packet_id == GYRO_ID:

        # WIT/WitMotion standard conversion:
        #
        # raw / 32768 * 2000 deg/s

        gx = x / 32768.0 * 2000.0
        gy = y / 32768.0 * 2000.0
        gz = z / 32768.0 * 2000.0

        packet_count["gyro"] += 1

        return {
            "type": "gyro",
            "gx": gx,
            "gy": gy,
            "gz": gz,
        }


    # --------------------------------------------------------
    # EULER ANGLE
    # --------------------------------------------------------

    elif packet_id == ANGLE_ID:

        roll = x / 32768.0 * 180.0
        pitch = y / 32768.0 * 180.0
        yaw = z / 32768.0 * 180.0

        angle_yaw_deg = yaw

        packet_count["angle"] += 1

        return {
            "type": "angle",
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }


    # --------------------------------------------------------
    # MAGNETOMETER
    # --------------------------------------------------------

    elif packet_id == MAG_ID:

        packet_count["mag"] += 1

        return {
            "type": "mag",
            "mx": x,
            "my": y,
            "mz": z,
        }

    return None


# ============================================================
# EULER YAW UNWRAP
# ============================================================

def unwrap_yaw(previous_unwrapped, current_raw):

    if previous_unwrapped is None:
        return current_raw

    delta = current_raw - previous_unwrapped

    while delta > 180.0:
        delta -= 360.0

    while delta < -180.0:
        delta += 360.0

    return previous_unwrapped + delta


# ============================================================
# LOW PASS FILTER
# ============================================================

class LowPassFilter:

    def __init__(self, cutoff_hz):

        self.cutoff_hz = cutoff_hz
        self.previous = None


    def update(self, value, dt):

        if dt <= 0.0:
            return value

        if self.previous is None:

            self.previous = value
            return value

        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)

        alpha = dt / (tau + dt)

        filtered = (
            self.previous
            + alpha * (value - self.previous)
        )

        self.previous = filtered

        return filtered


# ============================================================
# READ PACKET
# ============================================================

def read_packet(ser):

    # Search for packet header 0x55

    while True:

        byte = ser.read(1)

        if not byte:
            return None, None

        if byte[0] == HEADER:
            break


    # Read remaining 10 bytes

    remaining = ser.read(10)

    if len(remaining) != 10:
        return None, None

    packet = bytes([HEADER]) + remaining

    timestamp = time.monotonic()

    return packet, timestamp


# ============================================================
# MAIN
# ============================================================

def main():

    global angle_yaw_deg

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    timestamp_name = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    csv_file = os.path.join(
        OUTPUT_DIR,
        f"imu_bias_test_{timestamp_name}.csv"
    )

    report_file = os.path.join(
        OUTPUT_DIR,
        f"imu_bias_test_{timestamp_name}_report.txt"
    )


    # ========================================================
    # HEADER
    # ========================================================

    print()
    print("=" * 75)
    print("HIWONDER IMU - 100 Hz BIAS CHARACTERIZATION")
    print("=" * 75)
    print()

    print(f"Port                : {PORT}")
    print(f"Baudrate            : {BAUDRATE}")
    print(f"Total duration      : {TEST_DURATION:.1f} s")
    print(f"Stationary calib.   : {CALIBRATION_TIME:.1f} s")
    print(f"Gyro scale          : {GYRO_SCALE}")
    print(f"LPF enabled         : {ENABLE_LPF}")
    print()


    print("TEST PROCEDURE")
    print("--------------")
    print("1. IMU must remain COMPLETELY STATIONARY")
    print(f"   for the first {CALIBRATION_TIME:.0f} seconds.")
    print()
    print("2. The program will calculate the Z-axis gyro bias.")
    print()
    print("3. After calibration, perform random movement.")
    print()
    print("4. Do NOT change the IMU mounting.")
    print()
    print("5. Do NOT rotate the IMU during calibration.")
    print()
    print("6. LPF is OFF.")
    print()
    print("7. Gyro scale correction is OFF.")
    print()


    input("Press ENTER to start...")


    # ========================================================
    # OPEN SERIAL
    # ========================================================

    try:

        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            timeout=0.2
        )

    except Exception as e:

        print()
        print(f"ERROR opening serial port: {e}")
        return


    print()
    print("Serial port opened.")
    print()


    # ========================================================
    # CSV
    # ========================================================

    columns = [

        "time_s",
        "calibration",

        "dt_s",

        # Raw gyro
        "gyro_x_raw_dps",
        "gyro_y_raw_dps",
        "gyro_z_raw_dps",

        # Calibration
        "gyro_z_bias_corrected_dps",
        "gyro_z_scaled_dps",

        # Filter
        "gyro_z_filtered_dps",

        # Integrated yaw
        "gyro_yaw_raw_deg",
        "gyro_yaw_corrected_deg",
        "gyro_yaw_filtered_deg",

        # Euler
        "euler_yaw_deg",
        "euler_yaw_unwrapped_deg",

        # Error
        "yaw_difference_filtered_deg",
        "yaw_difference_raw_deg",
        "yaw_difference_corrected_deg",

    ]


    # ========================================================
    # STATE
    # ========================================================

    start_time = time.monotonic()

    previous_gyro_time = None

    previous_gyro_raw = None
    previous_gyro_corrected = None
    previous_gyro_filtered = None

    gyro_yaw_raw = 0.0
    gyro_yaw_corrected = 0.0
    gyro_yaw_filtered = 0.0

    previous_euler_yaw = None
    euler_yaw_unwrapped = None


    # ========================================================
    # BIAS CALIBRATION STATE
    # ========================================================

    bias_samples = []

    bias_locked = False
    bias_z = 0.0

    calibration_start = start_time


    # ========================================================
    # FILTER
    # ========================================================

    lpf = LowPassFilter(LPF_CUTOFF_HZ)


    # ========================================================
    # DEBUG
    # ========================================================

    debug_gyro_samples = 0
    DEBUG_GYRO_PRINT_COUNT = 20


    # ========================================================
    # CSV FILE
    # ========================================================

    with open(csv_file, "w", newline="") as f:

        writer = csv.writer(f)

        writer.writerow(columns)

        last_print_second = -1


        # ====================================================
        # MAIN LOOP
        # ====================================================

        while True:

            elapsed = time.monotonic() - start_time

            if elapsed >= TEST_DURATION:
                break


            packet, packet_time = read_packet(ser)

            if packet is None:
                continue


            decoded = decode_packet(packet)

            if decoded is None:
                continue


            # =================================================
            # EULER ANGLE
            # =================================================

            if decoded["type"] == "angle":

                current_yaw = decoded["yaw"]

                euler_yaw_unwrapped = unwrap_yaw(
                    euler_yaw_unwrapped,
                    current_yaw
                )

                previous_euler_yaw = current_yaw

                continue


            # =================================================
            # ONLY PROCESS GYRO PACKETS
            # =================================================

            if decoded["type"] != "gyro":
                continue


            gyro_x = decoded["gx"]
            gyro_y = decoded["gy"]
            gyro_z = decoded["gz"]


            # =================================================
            # DEBUG RAW GYRO
            # =================================================

            if debug_gyro_samples < DEBUG_GYRO_PRINT_COUNT:

                print(
                    f"RAW GYRO {debug_gyro_samples + 1:02d}: "
                    f"X={gyro_x:+.6f} "
                    f"Y={gyro_y:+.6f} "
                    f"Z={gyro_z:+.6f} deg/s"
                )

                debug_gyro_samples += 1


            # =================================================
            # STATIONARY CALIBRATION
            # =================================================

            calibration = elapsed < CALIBRATION_TIME


            if calibration:

                # Only accept low-rate Z samples.
                #
                # IMU MUST be physically stationary here.

                if abs(gyro_z) < BIAS_STATIONARY_THRESHOLD:

                    bias_samples.append(gyro_z)


                # Don't integrate calibration samples.

                previous_gyro_time = None
                previous_gyro_raw = None
                previous_gyro_corrected = None
                previous_gyro_filtered = None

                continue


            # =================================================
            # LOCK BIAS ONCE
            # =================================================

            if not bias_locked:

                if len(bias_samples) >= MIN_BIAS_SAMPLES:

                    bias_z = statistics.mean(bias_samples)

                    bias_std = (
                        statistics.stdev(bias_samples)
                        if len(bias_samples) > 1
                        else 0.0
                    )

                    bias_min = min(bias_samples)
                    bias_max = max(bias_samples)

                    bias_locked = True

                    print()
                    print("=" * 60)
                    print("BIAS CALIBRATION COMPLETE")
                    print("=" * 60)
                    print(
                        f"Accepted samples : {len(bias_samples)}"
                    )
                    print(
                        f"Gyro Z bias      : {bias_z:+.9f} deg/s"
                    )
                    print(
                        f"Gyro Z std       : {bias_std:.9f} deg/s"
                    )
                    print(
                        f"Gyro Z min       : {bias_min:+.9f} deg/s"
                    )
                    print(
                        f"Gyro Z max       : {bias_max:+.9f} deg/s"
                    )
                    print("=" * 60)
                    print()
                    print(">>> NOW START RANDOM MOVEMENT <<<")
                    print()

                else:

                    print(
                        f"\rWaiting for calibration samples: "
                        f"{len(bias_samples)}",
                        end="",
                        flush=True
                    )

                    continue


            # =================================================
            # TIMING
            # =================================================

            if previous_gyro_time is None:

                dt = 0.0

            else:

                dt = packet_time - previous_gyro_time

                if dt <= 0.0 or dt > 1.0:

                    dt = 0.0


            previous_gyro_time = packet_time


            # =================================================
            # BIAS CORRECTION
            # =================================================

            gyro_z_corrected = gyro_z - bias_z


            # =================================================
            # SCALE
            # =================================================

            gyro_z_scaled = (
                gyro_z_corrected * GYRO_SCALE
            )


            # =================================================
            # LOW PASS
            # =================================================

            if ENABLE_LPF and dt > 0.0:

                gyro_z_filtered = lpf.update(
                    gyro_z_scaled,
                    dt
                )

            else:

                # For THIS experiment, this should happen.

                gyro_z_filtered = gyro_z_scaled


            # =================================================
            # INTEGRATION
            # =================================================

            if dt > 0.0:

                # --------------------------------------------
                # RAW
                # --------------------------------------------

                if previous_gyro_raw is not None:

                    gyro_yaw_raw += (
                        0.5
                        * (
                            previous_gyro_raw
                            + gyro_z
                        )
                        * dt
                    )


                # --------------------------------------------
                # CORRECTED
                # --------------------------------------------

                if previous_gyro_corrected is not None:

                    gyro_yaw_corrected += (
                        0.5
                        * (
                            previous_gyro_corrected
                            + gyro_z_scaled
                        )
                        * dt
                    )


                # --------------------------------------------
                # FILTERED
                # --------------------------------------------

                if previous_gyro_filtered is not None:

                    gyro_yaw_filtered += (
                        0.5
                        * (
                            previous_gyro_filtered
                            + gyro_z_filtered
                        )
                        * dt
                    )


            previous_gyro_raw = gyro_z
            previous_gyro_corrected = gyro_z_scaled
            previous_gyro_filtered = gyro_z_filtered


            # =================================================
            # YAW ERRORS
            # =================================================

            if euler_yaw_unwrapped is not None:

                yaw_difference_filtered = (
                    gyro_yaw_filtered
                    - euler_yaw_unwrapped
                )

                yaw_difference_raw = (
                    gyro_yaw_raw
                    - euler_yaw_unwrapped
                )

                yaw_difference_corrected = (
                    gyro_yaw_corrected
                    - euler_yaw_unwrapped
                )

            else:

                yaw_difference_filtered = float("nan")
                yaw_difference_raw = float("nan")
                yaw_difference_corrected = float("nan")


            # =================================================
            # WRITE CSV
            # =================================================

            writer.writerow([

                f"{elapsed:.6f}",

                1 if calibration else 0,

                f"{dt:.6f}",

                f"{gyro_x:.9f}",
                f"{gyro_y:.9f}",
                f"{gyro_z:.9f}",

                f"{gyro_z_corrected:.9f}",
                f"{gyro_z_scaled:.9f}",

                f"{gyro_z_filtered:.9f}",

                f"{gyro_yaw_raw:.9f}",
                f"{gyro_yaw_corrected:.9f}",
                f"{gyro_yaw_filtered:.9f}",

                f"{angle_yaw_deg:.9f}",

                (
                    f"{euler_yaw_unwrapped:.9f}"
                    if euler_yaw_unwrapped is not None
                    else ""
                ),

                f"{yaw_difference_filtered:.9f}",
                f"{yaw_difference_raw:.9f}",
                f"{yaw_difference_corrected:.9f}",

            ])


            # =================================================
            # TERMINAL STATUS
            # =================================================

            current_second = int(elapsed)

            if current_second != last_print_second:

                last_print_second = current_second

                print(
                    f"\rTime: {elapsed:6.1f}s | "
                    f"Z raw: {gyro_z:+8.3f} | "
                    f"Z corrected: {gyro_z_corrected:+8.3f} | "
                    f"Raw yaw: {gyro_yaw_raw:+9.2f} | "
                    f"Corrected yaw: {gyro_yaw_corrected:+9.2f}",
                    end="",
                    flush=True
                )


    # ========================================================
    # CLOSE
    # ========================================================

    ser.close()

    print()
    print()
    print("=" * 75)
    print("TEST COMPLETE")
    print("=" * 75)
    print()


    # ========================================================
    # FINAL STATISTICS
    # ========================================================

    print(f"Gyro packets : {packet_count['gyro']}")
    print(f"Angle packets: {packet_count['angle']}")
    print(f"Accel packets: {packet_count['accel']}")
    print(f"Mag packets  : {packet_count['mag']}")
    print(f"Invalid      : {packet_count['invalid']}")
    print()

    print(
        f"Estimated gyro Z bias : "
        f"{bias_z:+.9f} deg/s"
    )

    print(
        f"Accepted bias samples : "
        f"{len(bias_samples)}"
    )

    print(
        f"Gyro scale factor     : "
        f"{GYRO_SCALE:.6f}"
    )

    print(
        f"LPF enabled           : "
        f"{ENABLE_LPF}"
    )

    print()

    print(f"CSV saved    : {csv_file}")
    print(f"Report saved : {report_file}")


    # ========================================================
    # REPORT
    # ========================================================

    with open(report_file, "w") as report:

        report.write("=" * 75 + "\n")
        report.write(
            "HIWONDER IMU - BIAS CHARACTERIZATION REPORT\n"
        )
        report.write("=" * 75 + "\n\n")


        report.write(
            f"Port              : {PORT}\n"
        )

        report.write(
            f"Baudrate          : {BAUDRATE}\n"
        )

        report.write(
            f"Test duration     : {TEST_DURATION:.1f} s\n"
        )

        report.write(
            f"Calibration time   : {CALIBRATION_TIME:.1f} s\n"
        )

        report.write(
            f"Gyro scale factor : {GYRO_SCALE:.6f}\n"
        )

        report.write(
            f"LPF enabled       : {ENABLE_LPF}\n"
        )

        report.write(
            f"LPF cutoff        : {LPF_CUTOFF_HZ:.2f} Hz\n\n"
        )


        report.write("BIAS CALIBRATION\n")
        report.write("-" * 50 + "\n")

        report.write(
            f"Accepted samples : "
            f"{len(bias_samples)}\n"
        )

        report.write(
            f"Bias Z           : "
            f"{bias_z:+.9f} deg/s\n"
        )


        if len(bias_samples) > 1:

            report.write(
                f"Bias Z std       : "
                f"{statistics.stdev(bias_samples):.9f} deg/s\n"
            )

            report.write(
                f"Bias Z min       : "
                f"{min(bias_samples):+.9f} deg/s\n"
            )

            report.write(
                f"Bias Z max       : "
                f"{max(bias_samples):+.9f} deg/s\n"
            )


        report.write("\n")

        report.write("PACKET COUNTS\n")
        report.write("-" * 50 + "\n")

        for key, value in packet_count.items():

            report.write(
                f"{key:<15}: {value}\n"
            )


        report.write("\n")

        report.write("PROCESSING\n")
        report.write("-" * 50 + "\n")

        report.write(
            "Raw gyro:\n"
            "gyro_z_raw\n\n"
        )

        report.write(
            "Bias correction:\n"
            "gyro_z_corrected = gyro_z_raw - bias_z\n\n"
        )

        report.write(
            "Scale:\n"
            "gyro_z_scaled = gyro_z_corrected * scale\n\n"
        )

        report.write(
            "For this experiment:\n"
            "scale = 1.0\n"
            "LPF = disabled\n\n"
        )

        report.write(
            "Integration:\n"
            "trapezoidal integration using actual "
            "gyro packet timestamps\n\n"
        )


        report.write("INTERPRETATION\n")
        report.write("-" * 50 + "\n")

        report.write(
            "This experiment is intended to characterize "
            "stationary gyro-Z bias and compare raw versus "
            "bias-corrected gyro integration.\n\n"
        )

        report.write(
            "The Euler yaw output is NOT treated as independent "
            "physical ground truth.\n"
        )

        report.write(
            "Absolute yaw accuracy requires an external physical "
            "reference.\n"
        )


    print()
    print("Next step:")
    print("Keep the IMU completely stationary for the first")
    print(f"{CALIBRATION_TIME:.0f} seconds.")
    print("After the calibration message appears, perform")
    print("the random-motion test.")
    print()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
