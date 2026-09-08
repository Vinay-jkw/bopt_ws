"""
Shared mutable runtime state for the workflow node process.

cancel_status and stop_event are module-level so that signal_handler (which
runs in the main thread) can set them and all callbacks / timer handlers that
import this module see the updated values immediately.

RuntimeState groups the per-task execution state that is shared between the
movement loop and the reroute/halt/resume callback.
"""

import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Process-wide cancellation flags — set by signal_handler and main()
# ---------------------------------------------------------------------------
cancel_status: bool = False
stop_event = threading.Event()


@dataclass
class RuntimeState:
    """Mutable state that evolves during a single task execution."""

    # Task identity
    task_id: Optional[str] = None

    # Workflow progress (string compared lexicographically, e.g. '0' < '3')
    operation_state: str = '0'

    # Error reporting
    error_status: Optional[str] = None

    # nmpc_controller subprocess handle (Popen) — shared between
    # movement_operation (starts it) and conflict_action_msg_handler (kills it)
    nmpc_fast_process: Optional[Any] = None

    # Whether the current fast-drive is heading toward the destination
    # (True) or handling a reroute detour (False)
    moving_to_destination: bool = True

    # The reroute item currently being executed (single point or polyline)
    current_reroute_item: Optional[Any] = None

    # Conflict message buffered before nmpc_fast_process is ready
    reroutor_startup_conflict: Optional[Any] = None

    # Queue of pending reroute waypoints populated by conflict_action_msg_handler
    reroute_locations: deque = field(default_factory=deque)

    # Lifecycle flag written by main() on clean shutdown
    sequence_complete: bool = False

    # Halt state (currently managed via parameter service, kept for future use)
    is_halted: bool = False
