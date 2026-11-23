"""Synthetic closed -> open -> missing-scan sequence. No ROS or hardware needed."""
from .core import DoorConfig,DoorMonitor,classify_scan


def main():
    config=DoorConfig(stable_sec=.2,stale_sec=.5)
    monitor=DoorMonitor(config)
    for now,value in [(0.,2.),(.21,2.),(.3,3.),(.51,3.)]:
        obs=classify_scan([value]*101,-.5,.01,.05,10.,2.,0.,config)
        print(f'{now:.2f}s {monitor.observe(now,obs)}')
    print(f'1.10s {monitor.tick(1.1)} (scan missing)')
    assert monitor.state=='UNKNOWN'
    return 0


if __name__=='__main__':raise SystemExit(main())
