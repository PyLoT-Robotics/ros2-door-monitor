"""Optional real DDS/TF test with synthetic scans; never connects to hardware."""
import os
import signal
import subprocess
import sys
import time
import pytest

rclpy=pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def test_real_node_tf_open_closed_and_stale():
    # Isolate generated topics and DDS from other robot workspaces.
    domain=94
    context=Context();rclpy.init(context=context,domain_id=domain)
    helper=Node('door_monitor_synthetic_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(helper)
    states=[]
    sub=helper.create_subscription(String,'/door_monitor/state',lambda m:states.append(m.data),10)
    pub=helper.create_publisher(LaserScan,'/door_monitor_test_scan',qos_profile_sensor_data)
    broadcaster=StaticTransformBroadcaster(helper)
    transform=TransformStamped()
    transform.header.frame_id='map';transform.child_frame_id='door_test_laser'
    transform.transform.rotation.w=1.
    # Sensor is at x=1 in map; door at x=3 is two metres ahead in scan frame.
    transform.transform.translation.x=1.
    broadcaster.sendTransform(transform)
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
    process=subprocess.Popen([sys.executable,'-c','from door_monitor.node import main; main()',
        '--ros-args','-p','scan_topic:=/door_monitor_test_scan','-p','door_x:=3.0',
        '-p','stable_sec:=0.1','-p','stale_sec:=0.3'],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
        start_new_session=True)

    def wait_for(expected,value=None,timeout=8.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            assert process.poll() is None, process.stdout.read().decode()
            if value is not None:
                msg=LaserScan();msg.header.frame_id='door_test_laser'
                msg.header.stamp=helper.get_clock().now().to_msg()
                msg.angle_min=-.5;msg.angle_max=.5;msg.angle_increment=.01
                msg.range_min=.05;msg.range_max=10.;msg.ranges=[value]*101
                pub.publish(msg)
            executor.spin_once(timeout_sec=.03)
            if expected in states:return
        pytest.fail(f'Expected {expected}; observed {states[-15:]}')
    try:
        wait_for('CLOSED',2.)
        states.clear();wait_for('OPEN',3.)
        states.clear();wait_for('UNKNOWN')
    finally:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
        output=process.stdout.read().decode();process.stdout.close()
        executor.shutdown();helper.destroy_node();rclpy.shutdown(context=context)
    assert process.returncode==0,output
