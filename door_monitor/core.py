"""ROS-independent geometry and temporal filtering for a known doorway.

A nearer return in the doorway cone is treated as CLOSED/blocked, regardless
of whether it is a door leaf or a person. Invalid or missing evidence is UNKNOWN.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DoorConfig:
    width_m: float = .8
    clearance_m: float = .2
    stable_sec: float = .5
    stale_sec: float = .5
    min_valid_samples: int = 3
    min_valid_fraction: float = .6
    min_door_distance_m: float = .1

    def __post_init__(self):
        for key in ('width_m', 'stale_sec', 'min_door_distance_m'):
            value=getattr(self,key)
            if not math.isfinite(value) or value<=0:raise ValueError(f'{key} must be finite and positive')
        for key in ('clearance_m','stable_sec'):
            value=getattr(self,key)
            if not math.isfinite(value) or value<0:raise ValueError(f'{key} must be finite and nonnegative')
        if not isinstance(self.min_valid_samples,int) or isinstance(self.min_valid_samples,bool) or self.min_valid_samples<1:
            raise ValueError('min_valid_samples must be a positive integer')
        if not math.isfinite(self.min_valid_fraction) or not 0<self.min_valid_fraction<=1:
            raise ValueError('min_valid_fraction must be in (0,1]')


@dataclass(frozen=True)
class ScanObservation:
    state: str
    min_range_m: float | None = None
    threshold_m: float | None = None
    valid_samples: int = 0
    reason: str = ''


def classify_scan(ranges, angle_min, angle_increment, range_min, range_max,
                  door_x, door_y, config):
    """door_x/y are the known door center expressed in the scan frame, in metres.

    Only finite returns in the sensor's valid range count as evidence. Both
    sides of the doorway cone must be covered; no-return (+inf) is not proof
    that a door is open. Positive and negative angle increments are supported.
    """
    if (not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min,door_x,door_y))
            or angle_increment==0 or range_min<0 or math.isnan(range_max) or range_max<=range_min):
        return ScanObservation('UNKNOWN',reason='invalid_scan_geometry')
    distance=math.hypot(door_x,door_y)
    if distance<config.min_door_distance_m:
        return ScanObservation('UNKNOWN',reason='door_too_close')
    bearing=math.atan2(door_y,door_x)
    half=math.atan2(config.width_m/2,distance)
    selected=[]
    for index,value in enumerate(ranges):
        angle=angle_min+index*angle_increment-bearing
        delta=math.atan2(math.sin(angle),math.cos(angle))
        if abs(delta)<=half:
            selected.append((delta,value))
    if not selected:
        return ScanObservation('UNKNOWN',reason='door_outside_scan')
    angles=[a for a,_ in selected]
    tolerance=abs(angle_increment)*1.01
    if min(angles)>-half+tolerance or max(angles)<half-tolerance:
        return ScanObservation('UNKNOWN',reason='partial_door_coverage')
    valid=[r for _,r in selected if math.isfinite(r) and range_min<=r<=range_max]
    threshold=distance+config.clearance_m
    if len(valid)<config.min_valid_samples or len(valid)/len(selected)<config.min_valid_fraction:
        return ScanObservation('UNKNOWN',threshold_m=threshold,valid_samples=len(valid),reason='insufficient_valid_returns')
    nearest=min(valid)
    state='OPEN' if nearest>threshold else 'CLOSED'
    return ScanObservation(state,nearest,threshold,len(valid),'clear_beyond_door' if state=='OPEN' else 'door_or_obstacle')


class DoorMonitor:
    def __init__(self,config=DoorConfig()):
        self.config=config
        self.state='UNKNOWN'
        self.last_observation=None
        self.last_time=None
        self.candidate=None
        self.candidate_since=None

    def reset(self):
        self.state='UNKNOWN'
        self.candidate=self.candidate_since=None
        self.last_observation=None

    def tick(self,now):
        if not math.isfinite(now):
            self.reset();self.last_time=None;return self.state
        if (self.last_time is not None and now<self.last_time) or (
                self.last_observation is not None and now-self.last_observation>=self.config.stale_sec):
            self.reset()
        self.last_time=now
        return self.state

    def observe(self,now,observation):
        self.tick(now)
        if not math.isfinite(now):return self.state
        self.last_observation=now
        if observation.state not in ('OPEN','CLOSED'):
            self.reset();return self.state
        if self.candidate!=observation.state:
            self.candidate=observation.state
            self.candidate_since=now
        self.state=(self.candidate if now-self.candidate_since+1e-9>=self.config.stable_sec else 'TRANSITIONING')
        return self.state
