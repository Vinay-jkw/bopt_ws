#!/bin/bash

set -u

# ============================================================
# BOPT MULTI-ROBOT CONTINUOUS PICK / DROP TEST
#
# Robots : robot001  ->  Pickup3  ->  Drop3
#          robot002  ->  Pickup4  ->  Drop4   (edit below)
#
# Flow per robot (in parallel):
#   Spawn pallet at pickup station
#   Pickup  (via workflow_node under that robot's namespace)
#   Drop    (via workflow_node under that robot's namespace)
#   Despawn pallet
#   Respawn fresh pallet
#   Repeat
# ============================================================

PALLET_SDF="$HOME/bopt_ws/src/bopt_description/models/pallet/model.sdf"
WORLD_NAME="fg_warehouse_world"

# ============================================================
# PER-ROBOT CONFIGURATION
#
# Add or remove entries to match your num_robots.
# Fields:
#   ROBOT_NAME     : ROS namespace  (must match multi_robot_spawn)
#   PALLET_NAME    : unique Gazebo model name per robot
#   PICKUP_STATION : station ID understood by workflow_node
#   DROP_STATION   : station ID understood by workflow_node
#   PALLET_X/Y/Z   : Gazebo spawn coordinates at the pickup station
#   PALLET_YAW     : spawn yaw (radians)
# ============================================================

declare -A ROBOT_NAME
declare -A PALLET_NAME
declare -A PICKUP_STATION
declare -A DROP_STATION
declare -A PALLET_X
declare -A PALLET_Y
declare -A PALLET_Z
declare -A PALLET_YAW

# --- Robot 1 ---
ROBOT_NAME[0]="robot001"
PALLET_NAME[0]="pallet_r001"
PICKUP_STATION[0]="Pickup3"
DROP_STATION[0]="Drop3"
PALLET_X[0]="26.5"
PALLET_Y[0]="29.5"
PALLET_Z[0]="0.05"
PALLET_YAW[0]="0.0"

# --- Robot 2 ---
ROBOT_NAME[1]="robot002"
PALLET_NAME[1]="pallet_r002"
PICKUP_STATION[1]="Pickup4"
DROP_STATION[1]="Drop4"
PALLET_X[1]="26.5"
PALLET_Y[1]="31.5"
PALLET_Z[1]="0.05"
PALLET_YAW[1]="0.0"

NUM_ROBOTS=2


# ============================================================
# HELPER : despawn a pallet by name
# ============================================================

despawn_pallet()
{
    local name="$1"
    echo "[${name}] Removing pallet from Gazebo..."

    ign service \
        -s /world/$WORLD_NAME/remove \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$name\", type: MODEL"

    local result=$?
    if [ $result -ne 0 ]; then
        echo "[${name}][ERROR] Failed to remove pallet."
        return 1
    fi
    echo "[${name}][OK] Pallet removed."
    sleep 2
    return 0
}


# ============================================================
# HELPER : spawn a pallet
# ============================================================

spawn_pallet()
{
    local name="$1"
    local px="$2"
    local py="$3"
    local pz="$4"
    local pyaw="$5"

    echo "[${name}] Spawning pallet at x=$px y=$py..."

    ros2 run ros_gz_sim create \
        -name "$name" \
        -file "$PALLET_SDF" \
        -x "$px" \
        -y "$py" \
        -z "$pz" \
        -Y "$pyaw"

    local result=$?
    if [ $result -ne 0 ]; then
        echo "[${name}][ERROR] Failed to spawn pallet."
        return 1
    fi
    echo "[${name}][OK] Pallet spawned."
    sleep 3
    return 0
}


# ============================================================
# HELPER : run workflow_node under a robot namespace
# Usage:  run_workflow <robot_name> <task_id> <station> <action>
# ============================================================

run_workflow()
{
    local robot="$1"
    local task_id="$2"
    local station="$3"
    local action="$4"

    echo "[${robot}] workflow_node  task=$task_id  station=$station  action=$action"

    ROS_NAMESPACE="$robot" \
        ros2 run workflow_node workflow_node \
            "$task_id" "$station" "$action" \
            --ros-args -r __ns:=/$robot

    local result=$?
    if [ $result -ne 0 ]; then
        echo "[${robot}][ERROR] workflow_node failed (station=$station action=$action)."
        return 1
    fi
    echo "[${robot}][OK] $action at $station completed."
    return 0
}


# ============================================================
# PER-ROBOT LOOP  (runs in a background subshell)
# ============================================================

robot_loop()
{
    local idx=$1
    local robot="${ROBOT_NAME[$idx]}"
    local pallet="${PALLET_NAME[$idx]}"
    local pickup="${PICKUP_STATION[$idx]}"
    local drop="${DROP_STATION[$idx]}"
    local px="${PALLET_X[$idx]}"
    local py="${PALLET_Y[$idx]}"
    local pz="${PALLET_Z[$idx]}"
    local pyaw="${PALLET_YAW[$idx]}"
    local cycle=1

    echo ""
    echo "============================================================"
    echo " [$robot] Starting loop"
    echo "   Pickup : $pickup"
    echo "   Drop   : $drop"
    echo "   Pallet : $pallet"
    echo "============================================================"

    # Remove any leftover pallet silently
    ign service \
        -s /world/$WORLD_NAME/remove \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$pallet\", type: MODEL" \
        >/dev/null 2>&1 || true

    sleep 2

    # Initial spawn
    spawn_pallet "$pallet" "$px" "$py" "$pz" "$pyaw" || {
        echo "[$robot][FATAL] Initial pallet spawn failed — stopping this robot."
        exit 1
    }

    while true; do

        echo ""
        echo "------------------------------------------------------------"
        echo " [$robot]  CYCLE $cycle"
        echo "   Pickup : $pickup   Drop : $drop"
        echo "------------------------------------------------------------"

        # 1. Pickup
        run_workflow "$robot" "$cycle" "$pickup" "Pickup" || {
            echo "[$robot][FATAL] Pickup failed at cycle $cycle — stopping."
            exit 1
        }

        sleep 2

        # 2. Drop
        run_workflow "$robot" "$cycle" "$drop" "Drop" || {
            echo "[$robot][FATAL] Drop failed at cycle $cycle — stopping."
            exit 1
        }

        sleep 2

        # 3. Despawn
        despawn_pallet "$pallet" || {
            echo "[$robot][FATAL] Despawn failed at cycle $cycle — stopping."
            exit 1
        }

        # 4. Respawn
        echo "[$robot] Respawning pallet for next cycle..."
        spawn_pallet "$pallet" "$px" "$py" "$pz" "$pyaw" || {
            echo "[$robot][FATAL] Respawn failed at cycle $cycle — stopping."
            exit 1
        }

        echo "[$robot] Cycle $cycle DONE."
        cycle=$((cycle + 1))
        sleep 2

    done
}


# ============================================================
# CLEANUP  (CTRL+C)
# ============================================================

ROBOT_PIDS=()

cleanup()
{
    echo ""
    echo "============================================================"
    echo " CTRL+C received — stopping all robots"
    echo "============================================================"

    # Kill robot subshells
    for pid in "${ROBOT_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done

    # Remove all pallets
    for i in $(seq 0 $((NUM_ROBOTS - 1))); do
        local pallet="${PALLET_NAME[$i]}"
        ign service \
            -s /world/$WORLD_NAME/remove \
            --reqtype ignition.msgs.Entity \
            --reptype ignition.msgs.Boolean \
            --timeout 3000 \
            --req "name: \"$pallet\", type: MODEL" \
            >/dev/null 2>&1 || true
    done

    echo "[INFO] All robots stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM


# ============================================================
# START
# ============================================================

echo ""
echo "############################################################"
echo "# BOPT MULTI-ROBOT PICK / DROP TEST"
echo "# Robots : $NUM_ROBOTS"
for i in $(seq 0 $((NUM_ROBOTS - 1))); do
    echo "#   [${ROBOT_NAME[$i]}]  ${PICKUP_STATION[$i]} -> ${DROP_STATION[$i]}"
done
echo "############################################################"
echo ""

# Launch each robot loop in a background subshell
for i in $(seq 0 $((NUM_ROBOTS - 1))); do
    robot_loop "$i" &
    ROBOT_PIDS+=($!)
    echo "[INFO] Started robot loop for ${ROBOT_NAME[$i]} (PID=${ROBOT_PIDS[-1]})"
    sleep 2   # stagger starts to avoid simultaneous pallet spawns
done

echo ""
echo "[INFO] All robot loops running. Press Ctrl+C to stop."
echo ""

# Wait for all background loops
wait "${ROBOT_PIDS[@]}"
