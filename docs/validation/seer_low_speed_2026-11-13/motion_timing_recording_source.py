import json,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('commissioning_motion_correlation_readonly');rows=[]
def cb(m,kind):
    p=m.pose.pose if kind=='odom' else m.pose
    r={'source':kind,'receive_wall_time':time.time(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'x':p.position.x,'y':p.position.y,'qz':p.orientation.z,'qw':p.orientation.w}
    if kind=='odom':r.update(vx=m.twist.twist.linear.x,vy=m.twist.twist.linear.y,w=m.twist.twist.angular.z)
    rows.append(r)
subs=[n.create_subscription(Odometry,'/odom',lambda m:cb(m,'odom'),qos_profile_sensor_data),n.create_subscription(PoseStamped,'/seer/localization/base_pose',lambda m:cb(m,'localization'),qos_profile_sensor_data)]
start=time.monotonic()
while time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.05)
Path('/tmp/agv_motion_correlation_20261113.json').write_text(json.dumps({'duration_s':time.monotonic()-start,'motion_sent':False,'rows':rows},indent=2))
print('只读对照记录完成：',len(rows),'条')
n.destroy_node();rclpy.shutdown()
