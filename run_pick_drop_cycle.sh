#!/bin/bash

set -u

# ============================================================
# BOPT CONTINUOUS PICK / DROP TEST
#
# Pickup : Pickup3
# Drop   : Drop3
#
# Flow:
#   Spawn pallet at Pickup3
#   Pickup Pickup3
#   Drop Drop3
#   Despawn pallet
#   Spawn fresh pallet at Pickup3
#   Repeat
# ============================================================

PALLET_NAME="pallet_2"

PALLET_SDF="$HOME/bopt_ws/src/bopt_description/models/pallet/model.sdf"

# Ignition Gazebo world name (fg_warehouse.world -> fg_warehouse_world)
WORLD_NAME="fg_warehouse_world"

# ------------------------------------------------------------
# Pallet spawn pose at Pickup3
# (dock_station_end_line: x=123.71, y=45.51, yaw=0.0 => pose_z=0.0, pose_w=1.0)
# ------------------------------------------------------------

PALLET_X="28.0"
PALLET_Y="28.064"
PALLET_Z="0.05"
PALLET_YAW="0.0"

# ------------------------------------------------------------
# Stations
# ------------------------------------------------------------

PICKUP_STATION="Pickup3"

DROP_STATION="Drop3"

CYCLE=1


# ============================================================
# DESPAWN PALLET
# ============================================================

despawn_pallet()
{
    echo ""
    echo "============================================================"
    echo " Removing pallet: $PALLET_NAME"
    echo "============================================================"

    ign service \
        -s /world/$WORLD_NAME/remove \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$PALLET_NAME\", type: MODEL"

    RESULT=$?

    if [ $RESULT -ne 0 ]; then
        echo "[ERROR] Failed to remove pallet."
        return 1
    fi

    echo "[OK] Pallet removal requested."

    sleep 2

    return 0
}


# ============================================================
# SPAWN PALLET
# ============================================================

spawn_pallet()
{
    echo ""
    echo "============================================================"
    echo " Spawning pallet"
    echo " Name    : $PALLET_NAME"
    echo " Pickup3 : x=$PALLET_X y=$PALLET_Y"
    echo "============================================================"

    ros2 run ros_gz_sim create \
        -name "$PALLET_NAME" \
        -file "$PALLET_SDF" \
        -x "$PALLET_X" \
        -y "$PALLET_Y" \
        -z "$PALLET_Z" \
        -Y "$PALLET_YAW"

    RESULT=$?

    if [ $RESULT -ne 0 ]; then
        echo "[ERROR] Failed to spawn pallet."
        return 1
    fi

    echo "[OK] Pallet spawned."

    sleep 3

    return 0
}


# ============================================================
# PICKUP
# ============================================================

pickup_pallet()
{
    echo ""
    echo "------------------------------------------------------------"
    echo " Cycle $CYCLE"
    echo " Pickup station: $PICKUP_STATION"
    echo "------------------------------------------------------------"

    ros2 run workflow_node workflow_node 1 "$PICKUP_STATION" Pickup

    RESULT=$?

    if [ $RESULT -ne 0 ]; then
        echo "[ERROR] Pickup failed."
        return 1
    fi

    echo "[OK] Pickup completed."

    sleep 2

    return 0
}


# ============================================================
# DROP
# ============================================================

drop_pallet()
{
    echo ""
    echo "------------------------------------------------------------"
    echo " Cycle $CYCLE"
    echo " Drop station: $DROP_STATION"
    echo "------------------------------------------------------------"

    ros2 run workflow_node workflow_node 1 "$DROP_STATION" Drop

    RESULT=$?

    if [ $RESULT -ne 0 ]; then
        echo "[ERROR] Drop failed at $DROP_STATION."
        return 1
    fi

    echo "[OK] Drop completed at $DROP_STATION."

    sleep 3

    return 0
}


# ============================================================
# CLEANUP WHEN CTRL+C
# ============================================================

cleanup()
{
    echo ""
    echo "============================================================"
    echo " CTRL+C received"
    echo " Cleaning up pallet..."
    echo "============================================================"

    ign service \
        -s /world/$WORLD_NAME/remove \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$PALLET_NAME\", type: MODEL" \
        >/dev/null 2>&1 || true

    echo "[INFO] Script stopped."

    exit 0
}

trap cleanup SIGINT SIGTERM


# ============================================================
# START
# ============================================================

echo ""
echo "############################################################"
echo "# BOPT CONTINUOUS PICK / DROP TEST"
echo "############################################################"
echo "# Pickup : $PICKUP_STATION"
echo "# Drop   : $DROP_STATION"
echo "# Pallet : $PALLET_NAME"
echo "# World  : $WORLD_NAME"
echo "############################################################"


# ------------------------------------------------------------
# Make sure no old pallet remains
# ------------------------------------------------------------

echo ""
echo "[INFO] Removing any existing pallet..."

ign service \
    -s /world/$WORLD_NAME/remove \
    --reqtype ignition.msgs.Entity \
    --reptype ignition.msgs.Boolean \
    --timeout 3000 \
    --req "name: \"$PALLET_NAME\", type: MODEL" \
    >/dev/null 2>&1 || true

sleep 2


# ------------------------------------------------------------
# Initial spawn
# ------------------------------------------------------------

spawn_pallet

if [ $? -ne 0 ]; then
    echo "[FATAL] Initial spawn failed."
    exit 1
fi


# ============================================================
# MAIN LOOP
# ============================================================

while true
do

    echo ""
    echo ""
    echo "############################################################"
    echo "# STARTING CYCLE $CYCLE"
    echo "#"
    echo "# Pickup : $PICKUP_STATION"
    echo "# Drop   : $DROP_STATION"
    echo "############################################################"


    # --------------------------------------------------------
    # 1. PICKUP
    # --------------------------------------------------------

    pickup_pallet

    if [ $? -ne 0 ]; then
        echo ""
        echo "[FATAL] Pickup failed."
        echo "[FATAL] Continuous test stopped."
        exit 1
    fi


    # --------------------------------------------------------
    # 2. DROP
    # --------------------------------------------------------

    drop_pallet

    if [ $? -ne 0 ]; then
        echo ""
        echo "[FATAL] Drop failed."
        echo "[FATAL] Continuous test stopped."
        exit 1
    fi


    # --------------------------------------------------------
    # 3. DESPAWN
    # --------------------------------------------------------

    despawn_pallet

    if [ $? -ne 0 ]; then
        echo ""
        echo "[FATAL] Could not despawn pallet."
        echo "[FATAL] Continuous test stopped."
        exit 1
    fi


    # --------------------------------------------------------
    # 4. RESPAWN AT Pickup3
    # --------------------------------------------------------

    echo ""
    echo "[INFO] Respawning fresh pallet at $PICKUP_STATION..."

    spawn_pallet

    if [ $? -ne 0 ]; then
        echo ""
        echo "[FATAL] Respawn failed."
        echo "[FATAL] Continuous test stopped."
        exit 1
    fi


    # --------------------------------------------------------
    # 5. NEXT CYCLE
    # --------------------------------------------------------

    echo ""
    echo "############################################################"
    echo "# CYCLE $CYCLE COMPLETED"
    echo "# Next: Pickup=$PICKUP_STATION  Drop=$DROP_STATION"
    echo "############################################################"

    CYCLE=$((CYCLE + 1))

    sleep 2

done