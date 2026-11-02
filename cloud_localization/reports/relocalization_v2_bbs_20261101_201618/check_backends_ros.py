#!/usr/bin/env python3
"""Run the compiled localization node against generated PCDs and wrong poses."""
import importlib.util
import math
import sys
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('initial_alignment_probe',
    OUT.parent/'icp_initial_alignment_20261101_145322/check_initial_alignment.py')
probe_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe_module)
OUT = OUT/(sys.argv[1] if len(sys.argv)>1 else 'bbs_gpu_gicp')
OUT.mkdir(exist_ok=True)
probe_module.OUT = OUT


def pcd(path, xyz):
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n')
    path.write_bytes(header.encode()+np.column_stack([xyz,np.zeros(len(xyz))]).astype('<f4').tobytes())


def main():
    rclpy.init()
    probe = probe_module.Probe()
    backend = sys.argv[1] if len(sys.argv)>1 else 'bbs_gpu_gicp'
    original_case = probe.case
    def selected_case(name,path,callback,extra=()):
        return original_case(name,path,callback,('-p','localization_global_backend:='+backend,*extra))
    probe.case = selected_case
    source, target, translation = probe_module.source, probe_module.target, probe_module.translation
    try:
        with tempfile.TemporaryDirectory(prefix='global-icp-check-') as tmp:
            path = Path(tmp)/'target.pcd'
            pcd(path,target)

            def local_initialize_and_reject_jump():
                for _ in range(2):
                    assert np.linalg.norm(probe.scan(source))<1e-5
                aligned=probe.scan(source)
                assert np.linalg.norm(aligned-translation)<.025
                for _ in range(4):
                    result=probe.scan(source-[0,0,.50])
                    assert np.linalg.norm(result-aligned)<1e-5
                assert 'GICP tracking correction rejected' in (OUT/'local_tracking_gate.log').read_text()

            probe.case('local_tracking_gate',path,local_initialize_and_reject_jump)

            def reject_low_overlap():
                distant=np.random.default_rng(17).uniform([10,10,10],[15,15,15],(7000,3))
                partial=np.concatenate([source,distant])
                for _ in range(4):
                    assert np.linalg.norm(probe.scan(partial))<1e-5
                assert 'rejected: overlap=' in (OUT/'low_overlap.log').read_text()

            probe.case('low_overlap',path,reject_low_overlap,('-p','localization_global_enabled:=false'))

            def recover():
                end = time.monotonic()+35
                while time.monotonic()<end:
                    result=probe.scan(source)
                    if np.linalg.norm(result-translation)<.04 and probe.poses[-1].pose.covariance[0]<1.:
                        break
                else: raise AssertionError(f'Failed to recover from wrong XYZ/yaw: {result}')
                assert 'Global relocalization candidate:' in (OUT/'wrong_initial_pose.log').read_text()
                q=probe.poses[-1].pose.pose.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                assert abs(angle-probe_module.yaw)<.03,angle
                # Simulate an unobserved displacement, with odometry unchanged.
                displaced=source-[1.3,.2,0]
                expected=translation+probe_module.rotation@np.array([1.3,.2,0])
                end=time.monotonic()+35
                while time.monotonic()<end:
                    result=probe.scan(displaced)
                    if np.linalg.norm(result-expected)<.05 and probe.poses[-1].pose.covariance[0]<1.:break
                else:raise AssertionError(f'Failed to recover lost tracking: {result} expected {expected}')
                assert 'tracking lost' in (OUT/'wrong_initial_pose.log').read_text()
                print('Recovered wrong initial XYZ/yaw and lost tracking',flush=True)

            probe.case('wrong_initial_pose',path,recover,(
                '-p','initial_pose_x:=3.0','-p','initial_pose_y:=-3.0',
                '-p','initial_pose_yaw:=1.2','-p','localization_global_failure_frames:=4'))

            symmetric=Path(tmp)/'symmetric.pcd'
            pcd(symmetric,np.concatenate([target,target+[6.,0,0]]))

            def reject_ambiguity_and_manual_reset():
                end=time.monotonic()+35
                while time.monotonic()<end:
                    result=probe.scan(source)
                    log=(OUT/'ambiguous_map.log').read_text()
                    if 'ambiguous global matches' in log:break
                else:raise AssertionError('Expected ambiguity rejection')
                assert np.linalg.norm(result-translation)>1.
                pub=probe.node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
                probe.spin(.4)
                msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
                # RViz 2D tool supplies chassis XY/yaw with zero height/tilt.
                # The subscriber must preserve the lidar height/tilt estimate.
                base=translation+probe_module.rotation@np.array([.1,0,.46])
                msg.pose.pose.position.x=float(base[0]);msg.pose.pose.position.y=float(base[1])
                msg.pose.pose.orientation.z=math.sin(probe_module.yaw/2)
                msg.pose.pose.orientation.w=math.cos(probe_module.yaw/2)
                pub.publish(msg);probe.spin(.2)
                for _ in range(5):result=probe.scan(source)
                assert np.linalg.norm(result-translation)<.04,result
                assert 'Received RViz initial base pose' in (OUT/'ambiguous_map.log').read_text()
                client=probe.node.create_client(Trigger,'/relocalize')
                assert client.wait_for_service(timeout_sec=2)
                future=client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(probe.node,future,timeout_sec=3)
                assert future.result().success
                starts=(OUT/'ambiguous_map.log').read_text().count('Starting global backend=')
                probe.scan(source)
                assert (OUT/'ambiguous_map.log').read_text().count('Starting global backend=')>starts
                probe.node.destroy_client(client);probe.node.destroy_publisher(pub)
                print('Rejected ambiguous global match, accepted RViz reset; relocalize service available',flush=True)

            probe.case('ambiguous_map',symmetric,reject_ambiguity_and_manual_reset,(
                '-p','initial_pose_x:=3.0','-p','initial_pose_y:=-3.0',
                '-p','initial_pose_yaw:=1.2','-p','localization_global_failure_frames:=4'))
    finally:
        probe.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
