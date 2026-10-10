#!/bin/bash

set -u

# ============================================================
# SYNCHRONIZED MULTI-ROBOT PICK / DROP CYCLE
#
# Requirements:
# - Pickup locations: 2, 4, 6, 8
# - Drop locations: 8, 6, 4, 2
# - Pallets spawn at Pickup locations before cycle
# - All robots must complete their tasks in strict sync
# ============================================================

PALLET_SDF="$HOME/bopt_ws/src/bopt_description/models/pallet/model.sdf"
WORLD_NAME="fg_warehouse_world"
NUM_ROBOTS=4

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
PICKUP_STATION[0]="Pickup2"
DROP_STATION[0]="Drop8"
PALLET_X[0]="26.5"
PALLET_Y[0]="26.36"
PALLET_Z[0]="0.05"
PALLET_YAW[0]="0.0"

# --- Robot 2 ---
ROBOT_NAME[1]="robot002"
PALLET_NAME[1]="pallet_r002"
PICKUP_STATION[1]="Pickup4"
DROP_STATION[1]="Drop6"
PALLET_X[1]="26.5"
PALLET_Y[1]="29.82"
PALLET_Z[1]="0.05"
PALLET_YAW[1]="0.0"

# --- Robot 3 ---
ROBOT_NAME[2]="robot003"
PALLET_NAME[2]="pallet_r003"
PICKUP_STATION[2]="Pickup6"
DROP_STATION[2]="Drop4"
PALLET_X[2]="26.5"
PALLET_Y[2]="33.30"
PALLET_Z[2]="0.05"
PALLET_YAW[2]="0.0"

# --- Robot 4 ---
ROBOT_NAME[3]="robot004"
PALLET_NAME[3]="pallet_r004"
PICKUP_STATION[3]="Pickup8"
DROP_STATION[3]="Drop2"
PALLET_X[3]="26.5"
PALLET_Y[3]="36.60"
PALLET_Z[3]="0.05"
PALLET_YAW[3]="0.0"

# HELPER: Despawn all pallets
despawn_all_pallets() {
    echo "======================================"
    echo " DESPAWNING ALL PALLETS"
    echo "======================================"
    for i in $(seq 0 $((NUM_ROBOTS - 1))); do
        local name="${PALLET_NAME[$i]}"
        ign service -s /world/$WORLD_NAME/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 3000 --req "name: \"$name\", type: MODEL" >/dev/null 2>&1 || true
    done
    sleep 2
}

# HELPER: Spawn all pallets
spawn_all_pallets() {
    echo "======================================"
    echo " SPAWNING PALLETS AT PICKUP LOCATIONS"
    echo "======================================"
    for i in $(seq 0 $((NUM_ROBOTS - 1))); do
        local name="${PALLET_NAME[$i]}"
        local px="${PALLET_X[$i]}"
        local py="${PALLET_Y[$i]}"
        local pz="${PALLET_Z[$i]}"
        local pyaw="${PALLET_YAW[$i]}"
        
        echo "[INFO] Spawning $name for ${ROBOT_NAME[$i]} at ${PICKUP_STATION[$i]} (Y=$py)"
        ros2 run ros_gz_sim create -name "$name" -file "$PALLET_SDF" -x "$px" -y "$py" -z "$pz" -Y "$pyaw" >/dev/null 2>&1 &
        sleep 1 # Stagger spawns to prevent Gazebo hanging
    done
    wait
    sleep 3
}

# HELPER: Run workflow for all robots concurrently
run_workflow_sync() {
    local task_id="$1"
    local action="$2" # "Pickup" or "Drop"
    
    echo "======================================"
    echo " STARTING ACTION: $action FOR ALL ROBOTS"
    echo "======================================"
    
    local pids=()
    
    local indices=""
    if [ "$action" == "Pickup" ]; then
        indices=$(seq 0 $((NUM_ROBOTS - 1)))
    else
        indices=$(seq $((NUM_ROBOTS - 1)) -1 0)
    fi
    
    for i in $indices; do
        local robot="${ROBOT_NAME[$i]}"
        local station=""
        
        if [ "$action" == "Pickup" ]; then
            station="${PICKUP_STATION[$i]}"
        else
            station="${DROP_STATION[$i]}"
        fi
        
        echo "[INFO] [${robot}] workflow_node task=$task_id station=$station action=$action"
        
        ROS_NAMESPACE="$robot" ros2 run workflow_node workflow_node "$task_id" "$station" "$action" --ros-args -r __ns:=/$robot -p use_sim_time:=true &
        pids+=($!)
        
        # Stagger departures to prevent traffic jams in narrow corridors!
        # When action is Drop, $i will be counting down, so checking $i isn't simple. 
        # Instead, just sleep 20s if it's not the LAST element in $indices.
        # But a simple way is just sleep 20 always and we'll wait anyway, but to be clean:
        echo "[INFO] Waiting 20 seconds before starting the next robot..."
        sleep 20
    done
    
    # Wait for ALL robots to finish their action
    local failed=0
    for pid in "${pids[@]}"; do
        wait "$pid" || failed=1
    done
    
    if [ $failed -ne 0 ]; then
        echo "[FATAL] One or more robots failed during $action. Aborting cycle."
        exit 1
    fi
    echo "[OK] ALL ROBOTS COMPLETED $action SUCCESSFULLY."
}

# ============================================================
# MAIN LOOP
# ============================================================

cleanup() {
    echo -e "\n[INFO] CTRL+C received. Stopping and cleaning up..."
    despawn_all_pallets
    exit 0
}
trap cleanup SIGINT SIGTERM

# Start clean
despawn_all_pallets

cycle=1
while true; do
    echo -e "\n######################################################"
    echo "#   CYCLE $cycle : SYNCHRONIZED MULTI-ROBOT PICK & DROP"
    echo -e "######################################################\n"
    
    spawn_all_pallets
    
    run_workflow_sync "$cycle" "Pickup"
    
    sleep 3
    
    run_workflow_sync "$cycle" "Drop"
    
    sleep 3
    
    despawn_all_pallets
    
    cycle=$((cycle + 1))
done
