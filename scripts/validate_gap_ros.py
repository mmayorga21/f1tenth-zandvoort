#!/usr/bin/env python3
"""Check activation/reset/pause, then independently verify one ROS-driven lap."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from lap_validation import LapValidation


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--max-wall-seconds',type=float,default=480.)
    args=parser.parse_args()
    rclpy.init();node=Node('validate_gap_ros')
    latest={};events=[];rows=[];checks={};mode='checks';start_ns=None
    validation=LapValidation()
    retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_sim(msg):
        nonlocal start_ns
        latest['sim']=json.loads(msg.data)
        if mode=='lap' and latest['sim']['state']=='running' and start_ns is None:
            start_ns=latest['sim']['sim_time_ns']-round(latest['sim']['elapsed_sim_s']*1e9)

    def on_odom(msg):
        if mode!='lap' or start_ns is None or validation.complete:
            return
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        elapsed=(stamp-start_ns)/1e9
        if elapsed<0:return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        validation.update(p.x,p.y,elapsed)
        if not rows or elapsed-rows[-1]['time']>=.099 or validation.complete:
            rows.append({'time':elapsed,'x':p.x,'y':p.y,'yaw':yaw,
                         'speed':msg.twist.twist.linear.x,'decision':latest.get('controller',{}).get('decision')})

    subs=[node.create_subscription(String,'/sim/status',on_sim,retained),
          node.create_subscription(String,'/controller/status',lambda m:latest.update(controller=json.loads(m.data)),retained),
          node.create_subscription(String,'/sim/events',lambda m:events.append(json.loads(m.data)),100),
          node.create_subscription(Odometry,'/ego_racecar/odom',on_odom,10)]
    clients={key:node.create_client(kind,name) for key,name,kind in [
        ('enable','/controller/enable',SetBool),('pause','/sim/pause',SetBool),
        ('reset','/sim/reset',Trigger),('end','/sim/end_attempt',Trigger)]}

    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)

    def call(name,value=None,expect=True):
        assert clients[name].wait_for_service(timeout_sec=5),name
        request=Trigger.Request() if value is None else SetBool.Request(data=value)
        future=clients[name].call_async(request);end=time.monotonic()+5
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert future.done(),'Service timeout: '+name
        result=future.result()
        if expect:assert result.success,(name,result.message)
        spin(.3)
        return result

    def check(name,condition):
        assert condition,name
        checks[name]=True

    report={};cleanup=False;wall_start=time.monotonic()
    try:
        deadline=time.monotonic()+8
        while not all(k in latest for k in ('sim','controller')) and time.monotonic()<deadline:spin(.1)
        check('topics_available',all(k in latest for k in ('sim','controller')))
        check('controller_starts_disabled',not latest['controller']['armed'])
        cleanup=True
        call('reset');spin(.5)
        check('disabled_vehicle_stationary',max(abs(v) for v in latest['sim']['velocity'])<.01)
        call('enable',True);spin(.6)
        check('explicit_enable_starts_motion',latest['controller']['armed'] and latest['sim']['state']=='running' and latest['sim']['velocity'][0]>.1)
        call('enable',False);spin(.5)
        check('disable_brakes_vehicle',not latest['controller']['armed'] and abs(latest['sim']['velocity'][0])<.02)
        call('reset');call('enable',True);spin(.4);call('pause',True)
        check('pause_disarms_controller',not latest['controller']['armed'])
        call('pause',False);spin(.4)
        check('resume_does_not_rearm',not latest['controller']['armed'])
        call('reset');call('enable',True);spin(.4);call('reset');spin(.4)
        check('reset_disarms_and_clears_commands',not latest['controller']['armed'] and math.hypot(*latest['sim']['pose'][:2])<.001)
        competing=node.create_publisher(AckermannDriveStamped,'/drive',10);spin(.6)
        result=call('enable',True,expect=False)
        check('competing_publisher_rejected',not result.success and result.message=='another_drive_publisher')
        node.destroy_publisher(competing);spin(.6)
        call('reset');spin(.5)
        mode='lap';call('enable',True)
        last_progress=0.
        while time.monotonic()-wall_start<args.max_wall_seconds:
            spin(.1)
            if latest['sim']['state'] not in ('ready','running'):
                raise AssertionError('Simulator terminated: '+str(latest['sim']))
            if not latest['controller']['armed']:
                raise AssertionError('Controller stopped: '+str(latest['controller']))
            if validation.complete:
                break
            if time.monotonic()-last_progress>30:
                last_progress=time.monotonic()
                print(json.dumps({'lap_sim_s':latest['sim']['elapsed_sim_s'],
                                  'checkpoints':validation.next_checkpoint,
                                  'pose':latest['sim']['pose'],'distance_m':validation.distance}),flush=True)
        check('full_lap_verified',validation.complete)
        check('lap_collision_free',latest['sim']['attempt_valid'] is True and not latest['sim']['ego_collision'])
        lap_time=rows[-1]['time'];attempt_id=latest['sim']['attempt_id']
        controller=latest['controller'];lap_state=latest['sim']
        mode='finished';call('enable',False);spin(.7)
        check('final_disable_brakes_vehicle',abs(latest['sim']['velocity'][0])<.02)
        call('end')
        report={'result':'lap_complete','lap_time_sim_s':lap_time,'validation':validation.summary(),
                'checks':checks,'controller':controller,'attempt_id':attempt_id,
                'lap_state':lap_state,'end_state':latest['sim'],'trajectory':rows,'events':events}
        print(json.dumps({k:v for k,v in report.items() if k not in ('trajectory','events','controller','lap_state','end_state')},indent=2),flush=True)
    except Exception as exc:
        report={'result':'failed','error':str(exc),'checks':checks,
                'latest':latest,'validation':validation.summary(),'trajectory':rows,'events':events}
        raise
    finally:
        if cleanup:
            mode='finished'
            call('enable',False,expect=False);call('reset',expect=False)
        report['wall_elapsed_s']=time.monotonic()-wall_start
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
