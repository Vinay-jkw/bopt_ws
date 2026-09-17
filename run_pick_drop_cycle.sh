#!/bin/bash

set -u

# ============================================================
# BOPT CONTINUOUS PICK / DROP TEST
#
# Pickup : B0
# Drops  : C0 -> A0 -> C0 -> A0 -> ...
#
# Flow:
#   Spawn pallet at B0
#   Pickup B0
#   Drop C0/A0
#   Despawn pallet
#   Spawn fresh pallet at B0
#   Repeat
# ============================================================

PALLET_NAME="pallet_2"

PALLET_SDF="$HOME/bopt_ws/src/bopt_description/models/pallet/model.sdf"

# ------------------------------------------------------------
# Pallet spawn pose at B0
# ------------------------------------------------------------

PALLET_X="6.10"
PALLET_Y="0.08"
PALLET_Z="0.05"
PALLET_YAW="0.0"

# ------------------------------------------------------------
# Stations
# ------------------------------------------------------------

PICKUP_STATION="B0"

DROP_STATIONS=("C0" "A0")

DROP_INDEX=0
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
        -s /world/empty_world/remove \
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
    echo " Name : $PALLET_NAME"
    echo " B0   : x=$PALLET_X y=$PALLET_Y"
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
    local DROP_STATION="$1"

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
        -s /world/empty_world/remove \
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
echo "# Pickup : B0"
echo "# Drops  : C0 -> A0 -> C0 -> A0 -> ..."
echo "# Pallet : $PALLET_NAME"
echo "############################################################"


# ------------------------------------------------------------
# Make sure no old pallet remains
# ------------------------------------------------------------

echo ""
echo "[INFO] Removing any existing pallet..."

ign service \
    -s /world/empty_world/remove \
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

    DROP_STATION="${DROP_STATIONS[$DROP_INDEX]}"

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

    drop_pallet "$DROP_STATION"

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
    # 4. CHANGE DROP LOCATION
    #
    # C0 -> A0
    # A0 -> C0
    # --------------------------------------------------------

    if [ "$DROP_INDEX" -eq 0 ]; then
        DROP_INDEX=1
    else
        DROP_INDEX=0
    fi


    # --------------------------------------------------------
    # 5. RESPAWN AT B0
    # --------------------------------------------------------

    echo ""
    echo "[INFO] Respawning fresh pallet at B0..."

    spawn_pallet

    if [ $? -ne 0 ]; then
        echo ""
        echo "[FATAL] Respawn failed."
        echo "[FATAL] Continuous test stopped."
        exit 1
    fi


    # --------------------------------------------------------
    # 6. NEXT CYCLE
    # --------------------------------------------------------

    echo ""
    echo "############################################################"
    echo "# CYCLE $CYCLE COMPLETED"
    echo "# Next drop: ${DROP_STATIONS[$DROP_INDEX]}"
    echo "############################################################"

    CYCLE=$((CYCLE + 1))

    sleep 2

done