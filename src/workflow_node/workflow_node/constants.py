# Fixed identifiers that do not change between deployments or configurations.
# Runtime tuning values (speeds, thresholds, radii) belong in the YAML config.

# ---------------------------------------------------------------------------
# Fork commands
# ---------------------------------------------------------------------------
FORK_CMD_UP = 'up'
FORK_CMD_DOWN = 'down'

# ---------------------------------------------------------------------------
# NMPC drive profiles
# ---------------------------------------------------------------------------
PROFILE_FAST = 'fast'
PROFILE_SLOW = 'slow'
PROFILE_PP = 'pp'
PROFILE_DD = 'dd'
PROFILE_DP = 'dp'
PROFILE_PARK = 'park'

# ---------------------------------------------------------------------------
# Action identifiers (as received from task allocator)
# ---------------------------------------------------------------------------
ACTION_PICKUP = 'Pickup'
ACTION_DROP = 'Drop'
ACTION_PARKING = 'Parking'
ACTION_WAIT = 'Wait'
ACTION_HOLD = 'hold'

# ---------------------------------------------------------------------------
# Safety field modes (published to /byd/safety)
# ---------------------------------------------------------------------------
FIELD_NORMAL = 'normal'
FIELD_PICKDROP = 'pickdrop'

# ---------------------------------------------------------------------------
# Conflict action types (received from /conflict_action topic)
# ---------------------------------------------------------------------------
CONFLICT_REROUTE = 'reroute'
CONFLICT_HALT = 'halt'
CONFLICT_RESUME = 'resume'

# ---------------------------------------------------------------------------
# MQTT topic names
# ---------------------------------------------------------------------------
MQTT_TOPIC_TASK_STATUS = 'machine/task/status'
MQTT_TOPIC_ERROR_STATUS = 'machine/error/status'
MQTT_TOPIC_ERROR_DETECTED = 'machine/error/detected'

# ---------------------------------------------------------------------------
# Operation state progression (string-compared in order)
# ---------------------------------------------------------------------------
OP_STATE_INIT = '0'
OP_STATE_MOVE_DONE = '1'
OP_STATE_DOCK_DONE = '2'
OP_STATE_ACTION_START = '3'
OP_STATE_ACTION_DRIVE = '4'
OP_STATE_FORK_DONE = '5'
OP_STATE_RETURN_DONE = '6'

# ---------------------------------------------------------------------------
# ROS2 parameter service path for nmpc halt/resume
# ---------------------------------------------------------------------------
ROBOT_CLIENT_SET_PARAMETERS = '/robot_client/set_parameters'
PARAM_SHOULD_HALT = 'should_halt'
