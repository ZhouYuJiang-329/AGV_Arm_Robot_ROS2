"""Replay captured scans into the real collision monitor in isolated domain 210."""
import argparse,copy,json,math,os,subprocess,time
from pathlib import Path
import yaml
os.environ['ROS_DOMAIN_ID']='210'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,PolygonStamped,Point32,TransformStamped
from tf2_ros import StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
parser=argparse.ArgumentParser();parser.add_argument('--profile',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
profile=yaml.safe_load(Path(args.profile).read_text())
fixtures={kind:json.loads((Path(__file__).resolve().parent / (kind+'_obstacle_scans.json')).read_text()) for kind in ('close','cleared')}
meta=copy.deepcopy(fixtures['close']);fixtures['close']['front']=fixtures['cleared']['front'];fixture=fixtures['close']
rclpy.init();node=rclpy.create_node('collision_stop_fixture_io')
static=StaticTransformBroadcaster(node);transforms=[]
for side,tx,ty,yaw in (('front',-.295,-.550,-135),('back',.295,.550,54)):
    t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='laser_'+side+'_link';t.header.stamp=node.get_clock().now().to_msg();t.transform.translation.x=tx;t.transform.translation.y=ty;t.transform.translation.z=-.24;t.transform.rotation.z=math.sin(math.radians(yaw)/2);t.transform.rotation.w=math.cos(math.radians(yaw)/2);transforms.append(t)
static.sendTransform(transforms)
scans={s:node.create_publisher(LaserScan,'/fixture/scan_'+s,qos_profile_sensor_data) for s in ('front','back')}
footprint=node.create_publisher(PolygonStamped,'/fixture/footprint',10)
outline=yaml.safe_load(profile['global_costmap']['global_costmap']['ros__parameters']['footprint'])
cmd_pub=None

def publish():
    stamp=node.get_clock().now().to_msg()
    for side,pub in scans.items():
        d=fixture[side];m=LaserScan();m.header.stamp=stamp;m.header.frame_id=d['frame_id'];m.angle_min=d['angle_min'];m.angle_increment=d['angle_increment'];m.angle_max=m.angle_min+m.angle_increment*(len(d['ranges'])-1);m.range_min=meta[side]['range_min'];m.range_max=meta[side]['range_max'];m.scan_time=.2;m.ranges=d['ranges'];pub.publish(m)
    p=PolygonStamped();p.header.stamp=stamp;p.header.frame_id='base_link';p.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in outline];footprint.publish(p)
    if cmd_pub is not None:
        m=Twist();m.linear.x=.2;cmd_pub.publish(m)
timer=node.create_timer(.1,publish)

def transition(name,which):
    c=node.create_client(ChangeState,'/'+name+'/change_state')
    if not c.wait_for_service(timeout_sec=5):raise RuntimeError('lifecycle not found')
    req=ChangeState.Request();req.transition.id=which;f=c.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=6)
    if not f.done() or not f.result().success:raise RuntimeError('transition failed')
    node.destroy_client(c)
results=[]
try:
    for index,kind in enumerate(('close','cleared')):
        fixture=fixtures[kind];name='collision_stop_fixture_'+str(index)
        params=copy.deepcopy(profile['collision_monitor']['ros__parameters']);params['stop_pub_timeout']=5.0;params['odom_frame_id']='base_link';params['cmd_vel_in_topic']='/fixture/input';params['cmd_vel_out_topic']='/fixture/output_'+str(index);params['state_topic']='/fixture/state_'+str(index);params['FootprintApproach']['footprint_topic']='/fixture/footprint'
        for side in ('front','back'):params[side]['topic']='/fixture/scan_'+side
        config=Path('/tmp/'+name+'.yaml');config.write_text(yaml.safe_dump({name:{'ros__parameters':params}}))
        cmd_pub=node.create_publisher(Twist,'/fixture/input',10);outputs=[];began=time.monotonic()
        sub=node.create_subscription(Twist,params['cmd_vel_out_topic'],lambda m:outputs.append((time.monotonic(),math.hypot(m.linear.x,m.linear.y))),10)
        stream=open('/tmp/'+name+'.log','w');proc=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor','--ros-args','-r','__node:='+name,'--params-file',str(config)],stdout=stream,stderr=subprocess.STDOUT)
        try:
            transition(name,Transition.TRANSITION_CONFIGURE);transition(name,Transition.TRANSITION_ACTIVATE)
            began=time.monotonic();end=began+3.5
            while time.monotonic()<end:
                if proc.poll() is not None:raise RuntimeError('monitor exited')
                rclpy.spin_once(node,timeout_sec=.02)
            values=[v for t,v in outputs if t-began>=1.5]
            if len(values)<5:raise RuntimeError('insufficient output')
            result={'fixture':kind,'samples':len(values),'min_output_speed':min(values),'max_output_speed':max(values),'mean_output_speed':sum(values)/len(values),'input_speed':.2,'domain':210,'hardware_connected':False}
            result['passed']=(max(values)<1e-6) if kind=='close' else (min(values)>.199)
            results.append(result);print(json.dumps(result),flush=True)
        finally:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            stream.close();node.destroy_subscription(sub);node.destroy_publisher(cmd_pub);cmd_pub=None
    Path(args.output).write_text(json.dumps(results,indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
if not all(r['passed'] for r in results):raise SystemExit(1)
