#!/bin/bash

set -u

# ============================================================
# MULTI-ROBOT BOPT CONTINUOUS PICK / DROP TEST
#
# Robot 1: Pickup B0, Drop C0
# Robot 2: Pickup B0, Drop A0 (starts after Robot 1 clears B0)
# ============================================================

PALLET1_NAME="pallet_1"
PALLET2_NAME="pallet_2"

PALLET_SDF="$HOME/bopt_ws/src/bopt_description/models/pallet/model.sdf"

# Spawn location for B0
PALLET_X="6.10"
PALLET_Y="0.10"
PALLET_Z="0.05"
PALLET_YAW="0.0"

PICKUP_STATION="B0"
# ============================================================
# DESPAWN PALLET
# ============================================================

despawn_pallet()
{
    local P_NAME="$1"
    echo "[INFO] Removing pallet: $P_NAME"

    ign service \
        -s /world/empty_world/remove \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$P_NAME\", type: MODEL" \
        >/dev/null 2>&1 || true

    sleep 2
    return 0
}

# ============================================================
# SPAWN PALLET
# ============================================================

spawn_pallet()
{
    local P_NAME="$1"
    echo "[INFO] Spawning pallet: $P_NAME at B0"

    ros2 run ros_gz_sim create \
        -name "$P_NAME" \
        -file "$PALLET_SDF" \
        -x "$PALLET_X" \
        -y "$PALLET_Y" \
        -z "$PALLET_Z" \
        -Y "$PALLET_YAW"

    sleep 3
    return 0
}

# ============================================================
# WORKFLOW COMMANDS
# ============================================================

pickup_pallet()
{
    local ROBOT="$1"
    local STATION="$2"
    echo "[INFO] $ROBOT going to pickup at $STATION"
    ros2 run workflow_node workflow_node 1 "$STATION" Pickup --ros-args -r __ns:=/$ROBOT
}

drop_pallet()
{
    local ROBOT="$1"
    local STATION="$2"
    echo "[INFO] $ROBOT going to drop at $STATION"
    ros2 run workflow_node workflow_node 1 "$STATION" Drop --ros-args -r __ns:=/$ROBOT
}

# ============================================================
# CLEANUP WHEN CTRL+C
# ============================================================

cleanup()
{
    echo ""
    echo "[INFO] CTRL+C received. Cleaning up pallets..."
    despawn_pallet "$PALLET1_NAME"
    despawn_pallet "$PALLET2_NAME"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================================================
# MAIN LOOP
# ============================================================

echo "[INFO] Initial cleanup of any existing pallets..."
despawn_pallet "$PALLET1_NAME"
despawn_pallet "$PALLET2_NAME"

CYCLE=1

# Station assignments
R1_DROP="C0"
R2_DROP="A0"

while true
do
    echo "############################################################"
    echo "# STARTING CYCLE $CYCLE"
    echo "############################################################"

    # --------------------------------------------------------
    # 1. ROBOT 1 PICKUP
    # --------------------------------------------------------
    spawn_pallet "$PALLET1_NAME"
    
    pickup_pallet "robot001" "$PICKUP_STATION"
    if [ $? -ne 0 ]; then
        echo "[FATAL] Robot 1 Pickup failed."
        exit 1
    fi

    # --------------------------------------------------------
    # 2. SEND ROBOT 1 TO DROP (Background)
    # --------------------------------------------------------
    drop_pallet "robot001" "$R1_DROP" &
    R1_PID=$!

    # Wait for Robot 1 to physically clear the B0 spawn area
    # before we spawn the next pallet to avoid Gazebo collisions.
    echo "[INFO] Waiting 15 seconds for Robot 1 to clear B0..."
    sleep 15

    # --------------------------------------------------------
    # 3. ROBOT 2 PICKUP
    # --------------------------------------------------------
    spawn_pallet "$PALLET2_NAME"
    
    pickup_pallet "robot002" "$PICKUP_STATION"
    if [ $? -ne 0 ]; then
        echo "[FATAL] Robot 2 Pickup failed."
        exit 1
    fi

    # --------------------------------------------------------
    # 4. SEND ROBOT 2 TO DROP (Background)
    # --------------------------------------------------------
    drop_pallet "robot002" "$R2_DROP" &
    R2_PID=$!

    # --------------------------------------------------------
    # 5. WAIT FOR BOTH ROBOTS TO FINISH DROPPING
    # --------------------------------------------------------
    echo "[INFO] Waiting for both robots to reach their drop stations..."
    wait $R1_PID
    wait $R2_PID

    # --------------------------------------------------------
    # 6. DESPAWN PALLETS
    # --------------------------------------------------------
    despawn_pallet "$PALLET1_NAME"
    despawn_pallet "$PALLET2_NAME"

    # Swap drop stations for variety if needed (Optional)
    # TEMP=$R1_DROP
    # R1_DROP=$R2_DROP
    # R2_DROP=$TEMP

    echo "############################################################"
    echo "# CYCLE $CYCLE COMPLETED"
    echo "############################################################"
    
    CYCLE=$((CYCLE + 1))
    sleep 3

done
