#!/usr/bin/env python3
"""Two measured 0.8 m squares; ROS odometry controls motion, truth is evaluation only."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path as RosPath
from std_srvs.srv import Trigger

OUT=Path(__file__).resolve().parent
def yaw(msg):
    q=msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def main():
    rclpy.init();node=rclpy.create_node('mapping_square_evaluation')
    state={'odom':None,'truth':None,'path':None}
    node.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m),20)
    node.create_subscription(Odometry,'/simulation/ground_truth',lambda m:state.update(truth=m),10)
    node.create_subscription(RosPath,'/localization/optimized_path',lambda m:state.update(path=m),10)
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def command(v=0,w=0):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pub.publish(msg)
    try:
        end=time.monotonic()+20
        while any(state[k] is None for k in ('odom','truth')) and time.monotonic()<end:spin(.1)
        assert all(state[k] is not None for k in ('odom','truth')),'Missing simulation data'
        start=state['truth'].pose.pose.position
        for lap in range(2):
            for side in range(4):
                p=state['odom'].pose.pose.position;x,y=p.x,p.y
                end=time.monotonic()+30
                while time.monotonic()<end:
                    p=state['odom'].pose.pose.position
                    distance=math.hypot(p.x-x,p.y-y)
                    if distance>=.80:break
                    command(min(.25,max(.08,(.8-distance)*1.5)));spin(.03)
                else:raise AssertionError('Straight segment timeout')
                command();spin(.35)
                last=yaw(state['odom']);turned=0;end=time.monotonic()+30
                while time.monotonic()<end:
                    now=yaw(state['odom']);turned+=math.remainder(now-last,2*math.pi);last=now
                    if turned>=math.pi/2:break
                    command(w=min(.75,max(.12,(math.pi/2-turned)*2)));spin(.03)
                else:raise AssertionError('Turn segment timeout')
                command();spin(.35)
                print('lap',lap+1,'side',side+1,'keyframes',len(state['path'].poses),flush=True)
        command();spin(1)
        end=state['truth'].pose.pose.position
        result={'truth_return_distance_m':math.hypot(end.x-start.x,end.y-start.y),'keyframes':len(state['path'].poses)}
        client=node.create_client(Trigger,'/save_map');assert client.wait_for_service(timeout_sec=3)
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=20)
        assert future.result() and future.result().success
        result['save']=future.result().message
        print(json.dumps(result),flush=True);(OUT/'simulation_results.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        command();spin(.2);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
