#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time


processes = []


def start_node(package, executable):

    print(f"Starting {package}/{executable}")

    command = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/Desktop/Ankit/Test/APDS/install/setup.bash && "
        "export PYTHONPATH="
        "/home/ankit/Desktop/Ankit/Test/APDS/src/ppts:"
        "$PYTHONPATH && "
        f"ros2 run {package} {executable}"
    )

    process = subprocess.Popen(
        ["bash", "-c", command],
        preexec_fn=os.setsid,
    )

    processes.append(process)

    return process

def stop_nodes():

    print("\nStopping nodes...")

    for process in processes:

        if process.poll() is None:

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGINT,
                )
            except ProcessLookupError:
                pass

    # Give processes time to shut down gracefully
    for _ in range(20):
        if all(
            process.poll() is not None
            for process in processes
        ):
            break

        time.sleep(0.1)


def main():

    print("=" * 60)
    print("          APDS LIDAR PIPELINE")
    print("=" * 60)

    try:

        # ==================================================
        # 1. Start LiDAR replay node
        # ==================================================

        lidar_process = start_node(
            "apds_lidar_clustering",
            "lidar_dummy.py",
        )

        # Give the LiDAR node time to initialize
        time.sleep(2)

        # ==================================================
        # 2. Start PPTS processing node
        # ==================================================

        ppts_process = start_node(
            "ppts",
            "ppts_visualize_cluster.py",
        )

        print()
        print("========================================")
        print("Both nodes are running")
        print("========================================")
        print(f"LiDAR PID : {lidar_process.pid}")
        print(f"PPTS PID  : {ppts_process.pid}")
        print()
        print("Press Ctrl+C to stop both nodes.")
        print("========================================")

        while True:

            # ----------------------------------------------
            # Check LiDAR node
            # ----------------------------------------------

            if lidar_process.poll() is not None:

                print(
                    f"\nLiDAR node stopped "
                    f"(exit code: {lidar_process.returncode})"
                )

                break

            # ----------------------------------------------
            # Check PPTS node
            # ----------------------------------------------

            if ppts_process.poll() is not None:

                print(
                    f"\nPPTS node stopped "
                    f"(exit code: {ppts_process.returncode})"
                )

                break

            time.sleep(1)

    except KeyboardInterrupt:

        print("\nCtrl+C received.")

    finally:

        stop_nodes()

        print("Pipeline stopped.")


if __name__ == "__main__":
    main()