#!/usr/bin/env python3
"""Validate lap timing, pause/reset/late-start handling and one real ROS lap."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from f1tenth_controller.lap_panel import format_panel
from lap_validation import LapValidation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=480.)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep the ROS context alive through Ctrl+C so finally can stop/reset.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('validate_laps_ros')
    latest, checks, lap_events, trajectory = {}, {}, [], []
    reference = LapValidation()
    phase, start_ns, reference_finish = 'checks', None, None
    late_process, late_log = None, None
    cleanup, success = False, False
    report = {'result': 'interrupted'}
    wall_start = time.monotonic()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def on_sim(msg):
        nonlocal start_ns
        latest['sim'] = json.loads(msg.data)
        if phase == 'lap' and latest['sim']['state'] == 'running' and start_ns is None:
            start_ns = latest['sim']['sim_time_ns']-round(latest['sim']['elapsed_sim_s']*1e9)

    def on_pose(msg):
        nonlocal reference_finish
        if phase != 'lap' or start_ns is None or reference.complete:
            return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elapsed = (stamp-start_ns)/1e9
        if elapsed < 0:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        reference.update(x, y, elapsed)
        if not trajectory or elapsed-trajectory[-1]['time'] >= .099 or reference.complete:
            trajectory.append({'time': elapsed, 'x': x, 'y': y})
        if reference.complete:
            reference_finish = elapsed

    subscriptions = [
        node.create_subscription(String, '/sim/status', on_sim, qos),
        node.create_subscription(String, '/controller/status',
                                 lambda m: latest.update(controller=json.loads(m.data)), qos),
        node.create_subscription(String, '/laps/status',
                                 lambda m: latest.update(laps=json.loads(m.data)), qos),
        node.create_subscription(String, '/laps_late/status',
                                 lambda m: latest.update(late=json.loads(m.data)), qos),
        node.create_subscription(String, '/laps/events',
                                 lambda m: lap_events.append(json.loads(m.data)), 100),
        node.create_subscription(Odometry, '/ego_racecar/odom', on_pose, 100),
    ]
    clients = {key: node.create_client(kind, name) for key, name, kind in [
        ('enable', '/controller/enable', SetBool), ('pause', '/sim/pause', SetBool),
        ('reset', '/sim/reset', Trigger), ('end', '/sim/end_attempt', Trigger)]}

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def wait_for(predicate, timeout=5.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            spin(.05)
        assert predicate(), 'Condition timed out'

    def call(name, value=None):
        client = clients[name]
        assert client.wait_for_service(timeout_sec=5), name
        request = Trigger.Request() if value is None else SetBool.Request(data=value)
        future = client.call_async(request)
        wait_for(future.done)
        assert future.result().success, (name, future.result().message)
        spin(.3)

    def check(name, condition):
        assert condition, name
        checks[name] = True

    def stop_late_tracker():
        nonlocal late_process, late_log
        if late_process is not None:
            if late_process.poll() is None:
                os.killpg(late_process.pid, signal.SIGINT)
                try:
                    late_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(late_process.pid, signal.SIGTERM)
                    late_process.wait(timeout=3)
            late_process = None
        if late_log is not None:
            late_log.close()
            late_log = None

    try:
        wait_for(lambda: all(key in latest for key in ('sim', 'controller', 'laps')), 8.)
        check('tracker_available_before_attempt', latest['laps']['state'] == 'ready')
        check('controller_initially_disabled', not latest['controller']['armed'])
        cleanup = True
        call('reset')
        wait_for(lambda: latest['laps']['session_id'] == latest['sim']['session_id'])
        check('reset_starts_at_zero', latest['laps']['lap_count'] == 0 and latest['laps']['elapsed_sim_s'] == 0.)
        call('enable', True)
        wait_for(lambda: latest['laps']['state'] == 'running')
        spin(.5)
        check('attempt_start_uses_source_identity', latest['laps']['attempt_id'] == latest['sim']['attempt_id'] and latest['laps']['measurement_valid'] is True)
        call('pause', True)
        wait_for(lambda: latest['laps']['paused'])
        paused = latest['laps'].copy()
        spin(.6)
        check('pause_freezes_lap_and_total_timers', latest['laps']['elapsed_sim_s'] == paused['elapsed_sim_s'] and latest['laps']['current_lap_time_s'] == paused['current_lap_time_s'])
        check('pause_does_not_generate_laps', latest['laps']['lap_count'] == 0)
        call('pause', False)
        spin(.3)
        check('resume_advances_time_without_rearming_controller', latest['laps']['elapsed_sim_s'] > paused['elapsed_sim_s'] and not latest['controller']['armed'])
        call('enable', True)
        late_log = (output.parent/'late_tracker.log').open('w')
        command = ['ros2', 'run', 'f1tenth_controller', 'lap_tracker', '--ros-args',
                   '-r', '__node:=late_lap_tracker', '-r', '/laps/status:=/laps_late/status',
                   '-r', '/laps/events:=/laps_late/events', '-p', 'use_sim_time:=true',
                   '-p', 'display:=false', '-p',
                   'event_log_path:='+str(output.parent/'late_laps.jsonl')]
        late_process = subprocess.Popen(command, stdout=late_log, stderr=late_log,
                                        start_new_session=True)
        wait_for(lambda: 'late' in latest, 8.)
        check('late_tracker_requires_reset', latest['late']['state'] == 'invalid' and latest['late']['reason'] == 'attempt_start_not_observed' and latest['late']['lap_count'] == 0)
        late_status = latest['late'].copy()
        stop_late_tracker()
        call('enable', False)
        call('reset')
        wait_for(lambda: latest['laps']['session_id'] == latest['sim']['session_id'])
        check('new_session_clears_trial_measurements', latest['laps']['state'] == 'ready' and latest['laps']['lap_count'] == 0 and latest['laps']['lap_times_s'] == [] and not latest['laps']['pause_used'])
        phase = 'lap'
        call('enable', True)
        last_progress = 0.
        while time.monotonic()-wall_start < args.max_wall_seconds:
            spin(.1)
            if latest['laps']['state'] == 'invalid':
                raise AssertionError('Invalid lap measurement: '+str(latest['laps']))
            if latest['sim']['state'] not in ('ready', 'running'):
                raise AssertionError('Simulator terminated: '+str(latest['sim']))
            if not latest['controller']['armed']:
                raise AssertionError('Controller disarmed: '+str(latest['controller']))
            if reference.complete and latest['laps']['lap_count'] >= 1:
                break
            if time.monotonic()-last_progress > 30:
                last_progress = time.monotonic()
                print(json.dumps({'sim_s': latest['laps']['elapsed_sim_s'],
                                  'current_lap_s': latest['laps']['current_lap_time_s'],
                                  'lap_count': latest['laps']['lap_count'],
                                  'regions': latest['laps']['checkpoints_passed'],
                                  'measurement_valid': latest['laps']['measurement_valid']}), flush=True)
        check('full_lap_matches_independent_route_check', reference.complete and latest['laps']['lap_count'] == 1)
        measured = latest['laps'].copy()
        lap_time = measured['last_lap_time_s']
        check('crossing_time_matches_physics_sample', abs(lap_time-reference_finish) <= .025)
        check('last_best_and_completed_time_match_first_lap', measured['lap_times_s'] == [lap_time] and measured['best_lap_time_s'] == lap_time and measured['completed_time_s'] == lap_time)
        check('same_collision_free_attempt', measured['attempt_id'] == latest['sim']['attempt_id'] and latest['sim']['attempt_valid'] is True and measured['measurement_valid'] is True)
        matching = [e for e in lap_events if e['event'] == 'lap_completed' and e['attempt_id'] == measured['attempt_id']]
        check('exactly_one_lap_event', len(matching) == 1)
        phase = 'finished'
        call('enable', False)
        spin(.6)
        check('vehicle_stopped_after_test', abs(latest['sim']['velocity'][0]) < .02)
        call('end')
        wait_for(lambda: latest['laps']['state'] == 'ended')
        check('manual_end_preserves_completed_lap', latest['laps']['lap_times_s'] == [lap_time] and latest['laps']['measurement_valid'] is True)
        panel = format_panel(latest['laps'])
        check('panel_displays_count_and_times', 'Vueltas completadas: 1 / 10' in panel and 'Ultima:' in panel and 'Mejor:' in panel)
        report = {'result': 'passed', 'checks': checks, 'lap_time_sim_s': lap_time,
                  'reference_finish_sample_sim_s': reference_finish,
                  'route': reference.summary(), 'measured_at_finish': measured,
                  'final_state': latest, 'late_tracker': late_status,
                  'lap_events': lap_events, 'trajectory': trajectory, 'terminal_panel': panel}
        (output.parent/'panel.txt').write_text(panel+'\n')
        success = True
        print(json.dumps({'result': 'passed', 'checks': checks,
                          'lap_time_sim_s': lap_time, 'reference_sample_sim_s': reference_finish}, indent=2), flush=True)
    except KeyboardInterrupt:
        report = {'result': 'interrupted', 'checks': checks, 'latest': latest}
        raise
    except Exception as exc:
        report = {'result': 'failed', 'error': str(exc), 'checks': checks,
                  'latest': latest, 'lap_events': lap_events, 'trajectory': trajectory}
        raise
    finally:
        stop_late_tracker()
        if cleanup and not success:
            call('enable', False)
            call('reset')
        report['wall_elapsed_s'] = time.monotonic()-wall_start
        output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
