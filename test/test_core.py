import math
import pytest
from door_monitor.core import DoorConfig, DoorMonitor, ScanObservation, classify_scan


def scan(value, x=2., y=0., **kwargs):
    return classify_scan([value]*101, -.5, .01, .05, 10., x, y, DoorConfig(), **kwargs)


def test_closed_open_and_missing_ranges():
    assert scan(2.).state == 'CLOSED'
    assert scan(3.).state == 'OPEN'
    assert scan(float('inf')).state == 'UNKNOWN'
    assert scan(float('nan')).state == 'UNKNOWN'
    assert scan(0.).state == 'UNKNOWN'


def test_door_bearing_and_scan_frame_are_used():
    assert scan(3., x=-2.).state == 'UNKNOWN'
    obs=classify_scan([3.]*101,.5,-.01,.05,10.,2.,0.,DoorConfig())
    assert obs.state=='OPEN'


def test_partial_coverage_and_too_close_are_unknown():
    assert scan(3.,x=.01).state == 'UNKNOWN'
    obs=classify_scan([3.]*30,0.,.01,.05,10.,2.,0.,DoorConfig())
    assert obs.state=='UNKNOWN'


def test_continuous_dwell_stale_and_recovery():
    monitor=DoorMonitor(DoorConfig(stable_sec=.3, stale_sec=.5))
    opened=scan(3.);closed=scan(2.)
    assert monitor.observe(0.,opened)=='TRANSITIONING'
    assert monitor.observe(.2,opened)=='TRANSITIONING'
    assert monitor.observe(.31,opened)=='OPEN'
    assert monitor.observe(.4,closed)=='TRANSITIONING'
    assert monitor.observe(.5,opened)=='TRANSITIONING'
    assert monitor.tick(1.1)=='UNKNOWN'
    assert monitor.observe(1.2,opened)=='TRANSITIONING'
    assert monitor.observe(1.51,opened)=='OPEN'


def test_missing_transform_observation_clears_open_state():
    monitor=DoorMonitor(DoorConfig(stable_sec=0.))
    assert monitor.observe(0.,scan(3.))=='OPEN'
    assert monitor.observe(.1,ScanObservation('UNKNOWN',reason='tf_unavailable'))=='UNKNOWN'


def test_clock_reset_and_stale_gap_do_not_complete_dwell():
    monitor=DoorMonitor(DoorConfig(stable_sec=.3,stale_sec=.5))
    assert monitor.observe(1.,scan(3.))=='TRANSITIONING'
    assert monitor.observe(2.,scan(3.))=='TRANSITIONING'
    assert monitor.observe(.5,scan(3.))=='TRANSITIONING'


@pytest.mark.parametrize('kwargs',[{'width_m':0.},{'stale_sec':-1.},{'stable_sec':float('nan')},{'min_valid_fraction':1.1}])
def test_invalid_configuration_fails(kwargs):
    with pytest.raises(ValueError):DoorConfig(**kwargs)
