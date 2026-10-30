from bopt_fleet_manager.fleet_monitor import pair_advisory


def test_clear_enters_near_at_stop_threshold():
    assert pair_advisory(4.0, 'CLEAR', 4.0, 4.5) == 'NEAR'


def test_near_state_holds_between_thresholds():
    assert pair_advisory(4.25, 'NEAR', 4.0, 4.5) == 'NEAR'


def test_near_state_clears_at_release_threshold():
    assert pair_advisory(4.5, 'NEAR', 4.0, 4.5) == 'CLEAR'


def test_clear_stays_clear_above_stop_threshold():
    assert pair_advisory(4.2, 'CLEAR', 4.0, 4.5) == 'CLEAR'
