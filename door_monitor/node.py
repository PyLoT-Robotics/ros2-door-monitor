"""ROS 2 adapter: known doorway -> scan frame at scan time -> independent core."""
from dataclasses import asdict, fields
import json
import math
import time

from .core import DoorConfig, DoorMonitor, ScanObservation, classify_scan

# ROS dependencies stay inside main so importing the geometry never needs ROS.
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool
    from visualization_msgs.msg import Marker
    from tf2_ros import Buffer, TransformListener, TransformException

    class DoorNode(Node):
        def __init__(self):
            super().__init__('door_monitor')
            defaults=DoorConfig()
            config={f.name:self.declare_parameter(f.name,getattr(defaults,f.name)).value for f in fields(defaults)}
            self.monitor=DoorMonitor(DoorConfig(**config))
            self.reference_frame=self.declare_parameter('reference_frame','map').value
            self.door_x=float(self.declare_parameter('door_x',2.).value)
            self.door_y=float(self.declare_parameter('door_y',0.).value)
            self.enabled=self.declare_parameter('enabled',True).value
            self.max_scan_age=float(self.declare_parameter('max_scan_age_sec',.5).value)
            self.max_future=float(self.declare_parameter('max_future_sec',.1).value)
            if not all(math.isfinite(v) for v in (self.door_x,self.door_y,self.max_scan_age,self.max_future)) or self.max_scan_age<=0 or self.max_future<0:
                raise ValueError('Door position and scan-age limits must be finite; age > 0 and future >= 0')
            self.tf=Buffer();self.listener=TransformListener(self.tf,self)
            self.observation=ScanObservation('UNKNOWN',reason='no_scan')
            self.last_stamp=None
            self.last_ros_time=None
            self.state_pub=self.create_publisher(String,'~/state',10)
            self.open_pub=self.create_publisher(Bool,'~/is_open',10)
            self.debug_pub=self.create_publisher(String,'~/debug',10)
            self.marker_pub=self.create_publisher(Marker,'~/marker',10)
            topic=self.declare_parameter('scan_topic','/scan').value
            self.create_subscription(LaserScan,topic,self.scan,qos_profile_sensor_data)
            self.create_service(SetBool,'~/set_enabled',self.set_enabled)
            self.create_timer(.1,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def set_enabled(self,request,response):
            self.enabled=request.data
            self.monitor.reset();self.last_stamp=None
            self.observation=ScanObservation('UNKNOWN',reason='enabled' if self.enabled else 'disabled')
            self.publish()
            response.success=True;response.message=self.observation.reason
            return response

        def scan(self,msg):
            now=time.monotonic()
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            ros_now=self.get_clock().now().nanoseconds*1e-9
            if self.last_ros_time is not None and ros_now<self.last_ros_time:
                self.monitor.reset();self.last_stamp=None
            self.last_ros_time=ros_now
            if not self.enabled:
                self.observation=ScanObservation('UNKNOWN',reason='disabled')
            elif not msg.header.frame_id or stamp<=0 or ros_now-stamp>self.max_scan_age or stamp-ros_now>self.max_future:
                self.observation=ScanObservation('UNKNOWN',reason='invalid_or_stale_scan_stamp')
            elif self.last_stamp is not None and stamp<=self.last_stamp:
                self.observation=ScanObservation('UNKNOWN',reason='non_increasing_scan_stamp')
            else:
                self.last_stamp=stamp
                try:
                    t=self.tf.lookup_transform(msg.header.frame_id,self.reference_frame,Time.from_msg(msg.header.stamp)).transform
                    q=t.rotation
                    norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                    if not math.isfinite(norm) or norm<1e-12:raise ValueError('invalid_tf_rotation')
                    # Full quaternion rotation of the configured point (x,y,0).
                    x=self.door_x;y=self.door_y;s=2/norm
                    dx=(1-s*(q.y*q.y+q.z*q.z))*x+s*(q.x*q.y-q.w*q.z)*y+t.translation.x
                    dy=s*(q.x*q.y+q.w*q.z)*x+(1-s*(q.x*q.x+q.z*q.z))*y+t.translation.y
                    self.observation=classify_scan(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,dx,dy,self.monitor.config)
                except (TransformException,ValueError):
                    self.observation=ScanObservation('UNKNOWN',reason='tf_unavailable_or_invalid')
            self.monitor.observe(now,self.observation)
            self.publish()

        def publish(self):
            state=self.monitor.tick(time.monotonic())
            self.state_pub.publish(String(data=state))
            self.open_pub.publish(Bool(data=state=='OPEN'))
            debug=asdict(self.observation)
            debug.update(state=state,enabled=self.enabled)
            if self.monitor.last_observation is None and self.observation.state!='UNKNOWN':debug['reason']='stale_scan'
            self.debug_pub.publish(String(data=json.dumps(debug,allow_nan=False)))
            marker=Marker();marker.header.frame_id=self.reference_frame
            marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='door_monitor';marker.id=0;marker.type=Marker.SPHERE;marker.action=Marker.ADD
            marker.pose.position.x=self.door_x;marker.pose.position.y=self.door_y;marker.pose.orientation.w=1.
            marker.scale.x=marker.scale.y=marker.scale.z=self.monitor.config.width_m
            marker.color.a=.6;marker.color.g=1. if state=='OPEN' else .3;marker.color.r=0. if state=='OPEN' else 1.
            self.marker_pub.publish(marker)

    rclpy.init(args=args)
    node=None
    try:
        node=DoorNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
