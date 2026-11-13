#!/usr/bin/env python3
"""Exercise attempt lifecycle, including an intentional crash in the simulator.

Requires the configured one-agent Zandvoort simulator and no other controller.
Restores the configured pose and stops all diagnostic commands on exit.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--allow-simulated-collision', action='store_true', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('validate_simulation_session', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    latest = {}
    events, odom_stamps, scan_stamps = [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def status(msg):
        latest['status'] = json.loads(msg.data)

    def clock(msg):
        latest['clock_ns'] = msg.clock.sec*1_000_000_000 + msg.clock.nanosec

    def sensor(key, stamps, msg):
        latest[key] = msg
        stamps.append(msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec)

    subscriptions = [
        node.create_subscription(String, '/sim/status', status, qos),
        node.create_subscription(String, '/sim/events', lambda m: events.append(json.loads(m.data)), 100),
        node.create_subscription(Bool, '/ego_racecar/collision', lambda m: latest.update(collision=m.data), qos),
        node.create_subscription(Clock, '/clock', clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        node.create_subscription(Odometry, '/ego_racecar/odom', lambda m: sensor('odom', odom_stamps, m), 10),
        node.create_subscription(LaserScan, '/scan', lambda m: sensor('scan', scan_stamps, m), 10),
    ]
    drive = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    clients = {name: node.create_client(kind, '/sim/'+name) for name, kind in
               [('reset', Trigger), ('pause', SetBool), ('end_attempt', Trigger)]}
    report = {'checks': {}}
    started = False

    def spin(seconds, speed=None):
        end = time.monotonic()+seconds
        next_publish = 0
        while time.monotonic()<end:
            if speed is not None and time.monotonic()>=next_publish:
                drive.publish(AckermannDriveStamped(drive=AckermannDrive(speed=float(speed))))
                next_publish=time.monotonic()+0.05
            rclpy.spin_once(node, timeout_sec=0.01)

    def call(name, paused=None):
        assert clients[name].wait_for_service(timeout_sec=3), name
        request=SetBool.Request(data=paused) if paused is not None else Trigger.Request()
        future=clients[name].call_async(request)
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert future.done(), 'Service timeout: '+name
        response=future.result()
        assert response.success, response.message
        spin(0.2)
        return response

    def check(name, condition):
        assert condition, name
        report['checks'][name]=True

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            spin(0.1)
            if all(k in latest for k in ['status', 'clock_ns', 'odom', 'scan', 'collision']):
                break
        check('single_clock_source', node.count_publishers('/clock')==1)
        check('no_other_controller', node.count_publishers('/drive')==1)
        check('required_topics', all(k in latest for k in ['status','clock_ns','odom','scan','collision']))
        started=True
        call('reset')
        initial=latest['status']
        spin(0.3)
        check('ready_clock_advances_without_starting_attempt', latest['clock_ns']>initial['sim_time_ns'] and latest['status']['attempt_id'] is None)
        spin(0.6, 0.5)
        check('nonzero_command_starts_attempt', latest['status']['state']=='running' and latest['status']['elapsed_sim_s']>0)
        call('pause', True)
        spin(0.2)
        paused=latest['status']; paused_clock=latest['clock_ns']
        clock_consumer_ns=node.get_clock().now().nanoseconds
        pose_before=(latest['odom'].pose.pose.position.x,latest['odom'].pose.pose.position.y)
        samples_before=len(odom_stamps)
        spin(0.6, 1.0)
        check('pause_freezes_physics_time', latest['clock_ns']==paused_clock and latest['status']['physics_steps']==paused['physics_steps'])
        check('pause_freezes_attempt_time', latest['status']['elapsed_sim_s']==paused['elapsed_sim_s'])
        check('ros_clock_consumer_matches_source', clock_consumer_ns==paused_clock and node.get_clock().now().nanoseconds==paused_clock)
        check('pause_ignores_commands', pose_before==(latest['odom'].pose.pose.position.x,latest['odom'].pose.pose.position.y))
        check('no_duplicate_sensor_stamps_while_paused', len(odom_stamps)==samples_before)
        report['pause']={'sim_time_ns':paused_clock,'elapsed_sim_s':paused['elapsed_sim_s'],'wall_wait_s':0.6}
        call('pause', False)
        spin(0.4, 0.0)
        check('resume_advances_clock', latest['clock_ns']>paused_clock)
        call('end_attempt')
        ended=latest['status'];spin(0.3, 1.0)
        check('manual_end_is_terminal', latest['status']['state']=='ended' and latest['status']['sim_time_ns']==ended['sim_time_ns'])
        check('manual_end_records_duration', ended['attempt_valid'] and ended['end_reason']=='manual_stop')
        report['manual_end']=ended
        call('reset')
        spin(0.4, 0.5)
        active=latest['status'];call('reset')
        spin(0.5)
        reset=latest['status']
        check('reset_changes_session_and_clears_attempt', reset['session_id']!=active['session_id'] and reset['attempt_id'] is None)
        check('reset_clears_old_speed_command', math.hypot(*reset['pose'][:2])<0.001 and max(abs(x) for x in reset['velocity'])<0.001)
        check('reset_marks_previous_attempt_invalid', any(e['event']=='attempt_ended' and e['attempt_id']==active['attempt_id'] and e['attempt_valid'] is False for e in events))
        # Face the nearby wall deliberately; this is not a race trial.
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.pose.pose.orientation.w=1.0
        pose_pub.publish(msg);spin(0.3)
        check('initialpose_is_a_new_session', latest['status']['session_id']!=reset['session_id'])
        deadline=time.monotonic()+5
        while latest['status']['state']!='collision' and time.monotonic()<deadline:
            spin(0.1, 1.0)
        collision=latest['status']
        check('real_backend_collision_invalidates_attempt', collision['state']=='collision' and collision['ego_collision'] and collision['attempt_valid'] is False)
        check('collision_topic_latched', latest['collision'] is True)
        spin(0.3, 1.0);call('pause',False)
        check('collision_cannot_resume', latest['status']['state']=='collision' and latest['status']['sim_time_ns']==collision['sim_time_ns'])
        check('collision_event_recorded', any(e['event']=='collision' and e['attempt_id']==collision['attempt_id'] for e in events))
        report['collision']=collision
        call('reset');spin(0.3)
        check('reset_clears_collision_and_stops_vehicle', latest['status']['state']=='ready' and not latest['collision'] and math.hypot(*latest['status']['pose'][:2])<0.001)
        check('sensor_stamps_use_physics_grid', all(s%10_000_000==0 for s in odom_stamps+scan_stamps))
        report['run_id']=latest['status']['run_id']
        report['final_status']=latest['status']
        report['events']=events
        report['status']='passed'
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'status':'passed','checks':report['checks'],'collision_elapsed_sim_s':collision['elapsed_sim_s']},indent=2))
    finally:
        if started:
            call('reset')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
