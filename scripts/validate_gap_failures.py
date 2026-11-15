#!/usr/bin/env python3
"""Inject temporary process freezes in the simulator to test watchdogs.

Only stops the explicitly named F1TENTH processes, then resumes them in finally.
Run after the normal controller is launched, disabled, with the car reset.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bridge-pid',type=int,required=True)
    p.add_argument('--controller-pid',type=int,required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    for pid,entry in [(args.bridge_pid,'/gym_bridge'),(args.controller_pid,'/gap_follower')]:
        command=Path(f'/proc/{pid}/cmdline').read_bytes().decode().replace('\0',' ')
        assert entry in command,(pid,command)
    rclpy.init();n=Node('validate_gap_failures');latest={};checks={};held=None
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[n.create_subscription(String,'/sim/status',lambda m:latest.update(sim=json.loads(m.data)),qos),
          n.create_subscription(String,'/controller/status',lambda m:latest.update(controller=json.loads(m.data)),qos),
          n.create_subscription(AckermannDriveStamped,'/drive',lambda m:latest.update(speed=m.drive.speed,drive_received=time.monotonic()),10)]
    enable=n.create_client(SetBool,'/controller/enable');reset=n.create_client(Trigger,'/sim/reset')

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)

    def call(client,value=None):
        assert client.wait_for_service(timeout_sec=5)
        future=client.call_async(Trigger.Request() if value is None else SetBool.Request(data=value))
        end=time.monotonic()+5
        while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        assert future.done() and future.result().success
        spin(.4)

    report={}
    try:
        end=time.monotonic()+5
        while not all(k in latest for k in ('sim','controller','speed')) and time.monotonic()<end:spin(.1)
        assert not latest['controller']['armed']
        call(reset);call(enable,True);spin(.4)
        assert latest['speed']>.1
        held=args.bridge_pid;os.kill(held,signal.SIGSTOP)
        started=time.monotonic()
        while latest['controller']['armed'] and time.monotonic()-started<1.:
            spin(.01)
        spin(.1)
        checks['scan_loss_disarms_controller']=not latest['controller']['armed'] and latest['controller']['reason']=='stale_scan'
        checks['scan_loss_publishes_zero']=latest['speed']==0.
        report['scan_loss']={'controller':latest['controller'],'observed_wall_delay_s':time.monotonic()-started}
        assert all(checks.values()),checks
        os.kill(held,signal.SIGCONT);held=None;spin(.5)
        call(enable,False);call(reset);call(enable,True);spin(.4)
        assert latest['speed']>.1
        held=args.controller_pid;os.kill(held,signal.SIGSTOP)
        started=time.monotonic()
        while latest['sim']['state']=='running' and time.monotonic()-started<1.:
            spin(.01)
        checks['controller_loss_invalidates_attempt']=latest['sim']['state']=='ended' and latest['sim']['end_reason']=='drive_timeout' and latest['sim']['attempt_valid'] is False
        checks['bridge_timeout_is_bounded']=time.monotonic()-started<.7
        report['controller_loss']={'sim':latest['sim'],'observed_wall_delay_s':time.monotonic()-started}
        assert all(checks.values()),checks
        os.kill(held,signal.SIGCONT);held=None;spin(.5)
        checks['resuming_controller_cannot_restart_terminal_attempt']=not latest['controller']['armed'] and latest['sim']['state']=='ended'
        assert all(checks.values()),checks
        report.update(result='passed',checks=checks)
        print(json.dumps({'result':'passed','checks':checks,
                          'scan_loss_delay_s':report['scan_loss']['observed_wall_delay_s'],
                          'controller_loss_delay_s':report['controller_loss']['observed_wall_delay_s']},indent=2))
    finally:
        if held is not None:os.kill(held,signal.SIGCONT)
        call(enable,False);call(reset)
        report['final_status']=latest
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n')
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
