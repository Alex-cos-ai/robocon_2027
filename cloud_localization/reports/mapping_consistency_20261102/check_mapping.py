#!/usr/bin/env python3
"""Exercise mapping geometry, timestamp pairing and loops in the real ROS node."""
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from std_srvs.srv import Trigger

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
EXE=ROOT/'install/robocon_cloud_localization/lib/robocon_cloud_localization/loop_closure_pgo'
CONFIG=ROOT/'src/robocon_cloud_localization/config/localization.yaml'
MOUNT=np.diag([1.,-1.,-1.])
rng=np.random.default_rng(42)
uv=rng.uniform(-2.5,2.5,(1500,2))
world=np.concatenate([np.c_[uv,np.zeros(len(uv))],np.c_[np.full(len(uv),2.5),uv],
    np.c_[uv[:,0],np.full(len(uv),-2.5),uv[:,1]],rng.uniform([-.8,.1,.4],[-.1,.6,.8],(500,3))])

def rotation(yaw):
    c,s=np.cos(yaw),np.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def odom(stamp,translation,yaw=0,base=False):
    msg=Odometry();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    p=np.array(translation,float)
    # Give the wheel chassis pose that converts back to the desired lidar pose.
    if base:p=p-rotation(yaw)@np.array([-.1,0,.46])
    msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,p)
    if base:
        msg.pose.pose.orientation.z=math.sin(-yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        # Map frame is the first lidar frame, with downward Z. Its physical
        # chassis yaw has the opposite sign of map-frame lidar yaw.
        msg.pose.pose.position.y=-float(p[1]);msg.pose.pose.position.z=-float(p[2])
    else:
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
    return msg

def wheel(stamp,translation,yaw=0):
    # T_odom_base = T_odom_lidar_initial * T_map_lidar * inv(T_base_lidar).
    # Choose T_odom_lidar_initial = T_base_lidar so the first base pose is I.
    R=rotation(yaw);p=np.array(translation)
    Rb=MOUNT@R@MOUNT
    pb=np.array([-.1,0,.46])+MOUNT@p-Rb@np.array([-.1,0,.46])
    return odom(stamp,pb,-yaw)

def cloud(stamp,points):
    msg=PointCloud2();msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    msg.header.frame_id='lidar_link';msg.height=1;msg.width=len(points)
    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,k in enumerate(('x','y','z','intensity'))]
    msg.point_step=16;msg.row_step=16*len(points);msg.is_dense=True
    msg.data=np.c_[points,np.zeros(len(points))].astype('<f4').tobytes()
    return msg

class Probe:
    def __init__(self):
        self.node=rclpy.create_node('mapping_regression')
        self.lio=self.node.create_publisher(Odometry,'/fast_lio/position',10)
        self.wheel=self.node.create_publisher(Odometry,'/odom',10)
        self.body=self.node.create_publisher(PointCloud2,'/fast_lio/cloud_registered_body',10)
        self.raw=self.node.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
        self.path=None;self.map=None
        self.node.create_subscription(RosPath,'/localization/optimized_path',lambda m:setattr(self,'path',m),10)
        self.node.create_subscription(PointCloud2,'/localization/optimized_map',lambda m:setattr(self,'map',m),1)
    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.01)
    def case(self,name,local,loops):
        self.path=None;self.map=None
        args=[str(EXE),'--ros-args','--params-file',str(CONFIG),'-p','mode:=map','-p','use_sim_time:=false',
              '-p',f'map_directory:={OUT/name}','-p','keyframe_distance:=0.18','-p','keyframe_min_interval:=0.1',
              '-p','keyframe_yaw:=0.10','-p','mapping_sync_max_gap:=0.3',
              '-p',f'mapping_local_registration:={str(local).lower()}',
              '-p',f'loop_separation:={4 if loops else 1000}','-p','loop_min_time:=2.0',
              '-p','loop_min_interval:=1.0','-p','map_publish_rate:=4.0']
        with (OUT/f'{name}.log').open('w') as log:
            proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
            try:
                end=time.monotonic()+10
                while not self.body.get_subscription_count() and time.monotonic()<end:self.spin(.05)
                assert proc.poll() is None and self.body.get_subscription_count()
                self.spin(.2)
                # Out-and-back true path. Cumulative odometry error remains on return.
                positions=list(np.linspace(0,2.0,11))+list(np.linspace(1.8,0,10))
                for i,x in enumerate(positions):
                    stamp=10+i*.5
                    true=np.array([x,0.,0.]);drift=np.array([.018*i,.008*i,0.])
                    guess=true+drift;yaw=.003*i
                    points=(world-true)  # True yaw is zero throughout.
                    # Newer odometry arrives before the old scan. A latest-pose
                    # implementation would attach a 0.15 m/0.04 rad wrong pose.
                    self.wheel.publish(wheel(stamp,guess,yaw));self.lio.publish(odom(stamp,guess,yaw))
                    self.wheel.publish(wheel(stamp+.1,guess+[.15,0,0],yaw+.04))
                    self.lio.publish(odom(stamp+.1,guess+[.15,0,0],yaw+.04))
                    self.spin(.03)
                    expected=0 if self.path is None else len(self.path.poses)
                    self.body.publish(cloud(stamp,points))
                    end=time.monotonic()+6
                    while (self.path is None or len(self.path.poses)<=expected) and time.monotonic()<end:self.spin(.02)
                    assert self.path is not None and len(self.path.poses)>expected,(name,i)
                self.spin(.4)
                assert len(self.path.poses)==len(positions),(name,len(self.path.poses))
                p=self.path.poses[-1].pose.position
                endpoint=float(np.linalg.norm([p.x,p.y,p.z]))
                assert self.map is not None
                data=np.frombuffer(bytes(self.map.data),dtype=np.dtype({'names':['x','y','z'],
                    'formats':['<f4']*3,'offsets':[f.offset for f in self.map.fields if f.name in ('x','y','z')],
                    'itemsize':self.map.point_step}))
                xyz=np.c_[data['x'],data['y'],data['z']]
                rmse=float(np.sqrt(np.mean(cKDTree(world).query(xyz)[0]**2)))
                client=self.node.create_client(Trigger,'/save_map');assert client.wait_for_service(timeout_sec=2)
                future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(self.node,future,timeout_sec=10)
                assert future.result().success
                self.node.destroy_client(client)
                info=(OUT/name/'map_info.txt').read_text()
                count=int(info.split('loop_closures=')[1].splitlines()[0])
                local_count=int(info.split('local_registration_factors=')[1].splitlines()[0])
                return dict(case=name,endpoint_m=endpoint,map_rmse_m=rmse,loops=count,local_factors=local_count)
            finally:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
                self.spin(.3)

def main():
    rclpy.init();probe=Probe()
    try:
        control=probe.case('odometry_only',False,False);print(control,flush=True)
        loop=probe.case('loop_only',False,True);print(loop,flush=True)
        full=probe.case('local_and_loop',True,True);print(full,flush=True)
        assert loop['loops']>0,loop
        assert full['loops']>0 and full['local_factors']>0,full
        assert loop['endpoint_m']<control['endpoint_m']*.6,(control,loop)
        assert full['map_rmse_m']<control['map_rmse_m']*.6,(control,full)
        (OUT/'synthetic_results.json').write_text(json.dumps([control,loop,full],indent=2)+'\n')
        print('PASS mapping timestamp pairing, local registration, loop correction, optimized save',flush=True)
    finally:probe.node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
