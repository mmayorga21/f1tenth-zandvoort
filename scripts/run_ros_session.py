#!/usr/bin/env python3
"""Supervise consecutive ROS laps and stop after the requested finish crossing."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from f1tenth_controller.session_guard import SessionGuard, SessionError
from lap_validation import LapValidation


def write_report(path, report):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(report, indent=2)+'\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, default=10)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-lap-sim-seconds', type=float, default=400.)
    parser.add_argument('--max-wall-seconds', type=float)
    args = parser.parse_args()
    if args.laps < 1 or args.max_lap_sim_seconds <= 0:
        parser.error('Session limits must be positive')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    wall_limit = args.max_wall_seconds or args.laps*450.+120.
    # Ctrl+C must leave the context alive until shutdown services are called.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('consecutive_lap_session')
    latest, received, events, references, progress = {}, {}, [], [], []
    phase, start_ns = 'initializing', None
    reference = None
    report = {'result': 'interrupted', 'mode': 'complete_ros_system',
              'requested_laps': args.laps,
              'started_at_utc': datetime.now(timezone.utc).isoformat()}
    cleanup_needed = False
    success = False
    wall_start = time.monotonic()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def receive(key, msg):
        nonlocal start_ns
        latest[key] = json.loads(msg.data)
        received[key] = time.monotonic()
        if key == 'sim' and phase == 'starting' and latest[key]['state'] == 'running':
            start_ns = latest[key]['sim_time_ns']-round(latest[key]['elapsed_sim_s']*1e9)

    def make_reference():
        config = latest['laps']['configuration']
        value = LapValidation((config['gate_x'], config['gate_y'], config['gate_yaw']))
        value.checkpoints = config['checkpoints']
        return value

    def on_pose(msg):
        nonlocal reference
        if phase not in ('starting', 'running') or start_ns is None or reference is None:
            return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elapsed = (stamp-start_ns)/1e9
        if elapsed < 0:
            return
        point = msg.pose.pose.position
        if reference.update(point.x, point.y, elapsed):
            references.append({'number': len(references)+1, 'finish_sample_sim_s': elapsed,
                               'route': reference.summary()})
            reference = make_reference()

    subscriptions = [node.create_subscription(String, topic,
                     lambda m, key=key: receive(key, m), qos)
                     for key, topic in [('sim', '/sim/status'), ('laps', '/laps/status'),
                                        ('controller', '/controller/status')]]
    subscriptions += [node.create_subscription(Odometry, '/ego_racecar/odom', on_pose, 100),
                      node.create_subscription(String, '/laps/events',
                                               lambda m: events.append(json.loads(m.data)), 100)]
    clients = {name: node.create_client(kind, path) for name, kind, path in [
        ('reset', Trigger, '/sim/reset'), ('enable', SetBool, '/controller/enable'),
        ('end', Trigger, '/sim/end_attempt')]}

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def wait_for(predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            spin(.05)
        if not predicate():
            raise SessionError('condition_timeout')

    def call(name, value=None):
        client = clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise SessionError('service_unavailable: '+name)
        future = client.call_async(Trigger.Request() if value is None else SetBool.Request(data=value))
        wait_for(future.done)
        response = future.result()
        if not response.success:
            raise SessionError(name+': '+response.message)
        spin(.15)

    def checkpoint(result='running', error=None):
        report.update(result=result, error=error, last_status=latest.copy(),
                      lap_progress=progress.copy(), independent_crossings=references.copy(),
                      wall_elapsed_s=time.monotonic()-wall_start,
                      updated_at_utc=datetime.now(timezone.utc).isoformat())
        write_report(output, report)

    try:
        wait_for(lambda: len(latest) == 3, 8.)
        if any(node.count_publishers(topic) != 1 for topic in ('/clock', '/drive', '/laps/status')):
            raise SessionError('duplicate_or_missing_publisher')
        cleanup_needed = True
        call('enable', False)
        call('reset')
        wait_for(lambda: latest['sim']['state'] == 'ready' and latest['laps']['state'] == 'ready'
                 and latest['sim']['session_id'] == latest['laps']['session_id'])
        if latest['laps']['lap_count'] != 0:
            raise SessionError('counter_not_reset')
        reference = make_reference()
        phase = 'starting'
        call('enable', True)
        wait_for(lambda: latest['sim']['state'] == 'running' and latest['laps']['state'] == 'running'
                 and latest['controller']['armed']
                 and latest['sim']['attempt_id'] == latest['laps']['attempt_id'])
        guard = SessionGuard(args.laps, latest['sim'], latest['laps'], latest['controller'],
                             max_lap_sim_s=args.max_lap_sim_seconds)
        report['initial_status'] = json.loads(json.dumps(latest))
        root = Path(__file__).resolve().parents[1]
        report['supervisor_sha256'] = hashlib.sha256(
            Path(__file__).read_bytes() +
            (root/'src/f1tenth_controller/f1tenth_controller/session_guard.py').read_bytes()).hexdigest()
        phase = 'running'
        previous_count, previous_print = 0, 0.
        checkpoint()
        print('SESION ROS INICIADA: '+str(args.laps)+' vueltas, sin pausas ni reinicios.', flush=True)
        while time.monotonic()-wall_start < wall_limit:
            spin(.05)
            reached = guard.observe(latest['sim'], latest['laps'], latest['controller'],
                                     received, time.monotonic())
            status = latest['laps']
            if status['lap_count'] != previous_count:
                previous_count = status['lap_count']
                row = {'lap': previous_count, 'time_sim_s': status['last_lap_time_s'],
                       'best_sim_s': status['best_lap_time_s'],
                       'total_completed_sim_s': status['completed_time_s']}
                progress.append(row)
                print(json.dumps(row), flush=True)
                checkpoint()
            if reached:
                target_status = json.loads(json.dumps(status))
                break
            if time.monotonic()-previous_print >= 45.:
                previous_print = time.monotonic()
                print(json.dumps({'lap_count': status['lap_count'],
                                  'current_lap_s': status['current_lap_time_s'],
                                  'regions': status['checkpoints_passed'],
                                  'total_sim_s': status['elapsed_sim_s']}), flush=True)
                checkpoint()
        else:
            raise SessionError('wall_time_limit')
        # Finish timing is captured above; braking and service latency add no
        # time to the completed-laps total used for comparison.
        call('enable', False)
        wait_for(lambda: len(references) >= args.laps, timeout=2.)
        phase = 'stopping'
        wait_for(lambda: abs(latest['sim']['velocity'][0]) < .02)
        call('end')
        wait_for(lambda: latest['laps']['state'] == 'ended')
        final = latest['laps']
        if (final['lap_count'] != args.laps or final['measurement_valid'] is not True
                or final['pause_used'] or final['lap_times_s'] != target_status['lap_times_s']
                or latest['sim']['ego_collision'] or latest['sim']['attempt_valid'] is not True):
            raise SessionError('final_session_not_valid')
        if len(references) != args.laps:
            raise SessionError('independent_route_count_disagrees')
        accumulated = 0.
        for duration, crossing in zip(final['lap_times_s'], references):
            accumulated += duration
            if abs(accumulated-crossing['finish_sample_sim_s']) > .025:
                raise SessionError('independent_crossing_time_disagrees')
        wait_for(lambda: sum(e['event'] == 'lap_completed' and e['attempt_id'] == final['attempt_id']
                             for e in events) >= args.laps, timeout=2.)
        completed_events = [e for e in events if e['event'] == 'lap_completed'
                            and e['attempt_id'] == final['attempt_id']]
        if len(completed_events) != args.laps:
            raise SessionError('lap_event_count_disagrees')
        report.update(target_crossing_status=target_status,
                      completed_time_sim_s=final['completed_time_s'],
                      best_lap_sim_s=final['best_lap_time_s'],
                      lap_times_sim_s=final['lap_times_s'],
                      lap_events=completed_events)
        success = True
        checkpoint('passed')
        print(json.dumps({'result': 'passed', 'laps': args.laps,
                          'total_sim_s': final['completed_time_s'],
                          'best_sim_s': final['best_lap_time_s'],
                          'lap_times_sim_s': final['lap_times_s']}, indent=2), flush=True)
    except KeyboardInterrupt:
        checkpoint('interrupted', 'user_or_process_interruption')
        raise
    except Exception as exc:
        checkpoint('failed', str(exc))
        raise
    finally:
        if cleanup_needed and not success:
            try:
                call('enable', False)
                if latest.get('sim', {}).get('state') == 'running':
                    call('end')
            except Exception as exc:
                report['cleanup_error'] = str(exc)
            checkpoint(report['result'], report.get('error'))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
