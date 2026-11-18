"""One-way real sensors 216 -> isolated Nav2 220. No hardware driver is started."""
import os,time,math,json,subprocess
from pathlib import Path
import yaml
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from agv_bringup_sim.real_navigation import EXECUTION_NODES
ROOT=Path.cwd();profile=ROOT/'docs/validation/seer_low_speed_2026-11-13/nav2_locked_arm_site.yaml'
src_ctx=Context();dst_ctx=Context();rclpy.init(context=src_ctx,domain_id=216);rclpy.init(context=dst_ctx,domain_id=220)
src=rclpy.create_node('commissioning_oneway_sensor_source',context=src_ctx);dst=rclpy.create_node('commissioning_nav2_dry_sink',context=dst_ctx)
se=SingleThreadedExecutor(context=src_ctx);de=SingleThreadedExecutor(context=dst_ctx);se.add_node(src);de.add_node(dst)
b=Buffer(node=src);listener=TransformListener(b,src)
subs=[];pubs=[];counts={};command_rows={};children=[];streams=[];clients=[];started=time.monotonic()

def pump(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        se.spin_once(timeout_sec=.005);de.spin_once(timeout_sec=.005)
        if any(p.poll() is not None for p in children):raise RuntimeError('a Nav2 child exited')
def until(predicate,timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return True
        pump(.025)
    return predicate()
for topic,cls,qos in (('/scan_front',LaserScan,qos_profile_sensor_data),('/scan_back',LaserScan,qos_profile_sensor_data),('/odom',Odometry,10),('/tf',TFMessage,100),('/tf_static',TFMessage,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))):
    pub=dst.create_publisher(cls,topic,qos);pubs.append(pub)
    def forward(m,p=pub,t=topic):
        counts[t]=counts.get(t,0)+1;p.publish(m)
    subs.append(src.create_subscription(cls,topic,forward,qos))
for topic in ('/navigation/cmd_vel_raw','/navigation/cmd_vel_smoothed','/navigation/cmd_vel_guard'):
    command_rows[topic]=[]
    def cb(m,t=topic):command_rows[t].append({'elapsed_s':time.monotonic()-started,'vx':m.linear.x,'vy':m.linear.y,'w':m.angular.z})
    subs.append(dst.create_subscription(Twist,topic,cb,10))
result={'hardware_connected':False,'source_domain':216,'test_domain':220,'bridge_direction':'216 to 220 only','profile':str(profile),'arrival_test':False}
try:
    pump(1.5)
    existing=set(dst.get_node_names())-{'commissioning_nav2_dry_sink'}
    if existing:raise RuntimeError('isolated domain occupied: '+str(existing))
    env=dict(os.environ,ROS_DOMAIN_ID='220',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def start(package,executable,name,extra=()):
        if not package.startswith('nav2_'):raise RuntimeError('only Nav2 binaries allowed')
        log=open('/tmp/agv_nav2_dry_'+name+'_20261113.log','w');streams.append(log)
        cmd=['/opt/ros/jazzy/lib/'+package+'/'+executable,'--ros-args','--params-file',str(profile),'-r','__node:='+name]+list(extra)
        children.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT))
    start('nav2_map_server','map_server','map_server',['-p','yaml_filename:='+str(ROOT/'maps/seer/20251120141209997_navigable.yaml')])
    start('nav2_planner','planner_server','planner_server')
    for package,name,remaps in EXECUTION_NODES:
        extra=[]
        for a,z in remaps:extra+=['-r',a+':='+z]
        start(package,name,name,extra)
    names=['map_server','planner_server']+[x[1] for x in EXECUTION_NODES]
    manager=Path('/tmp/agv_nav2_dry_manager_20261113.yaml');manager.write_text(yaml.safe_dump({'commissioning_dry_lifecycle':{'ros__parameters':{'use_sim_time':False,'autostart':True,'bond_timeout':0.,'node_names':names}}}))
    log=open('/tmp/agv_nav2_dry_lifecycle_20261113.log','w');streams.append(log)
    children.append(subprocess.Popen(['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager','--ros-args','-r','__node:=commissioning_dry_lifecycle','--params-file',str(manager)],env=env,stdout=log,stderr=subprocess.STDOUT))
    states={}
    for name in names:
        client=dst.create_client(GetState,'/'+name+'/get_state');clients.append(client)
        if not until(client.service_is_ready,10):raise RuntimeError(name+' lifecycle unavailable')
        active=False
        for _ in range(40):
            f=client.call_async(GetState.Request())
            if not until(f.done,2):raise RuntimeError(name+' state timeout')
            states[name]=f.result().current_state.label
            if f.result().current_state.id==3:active=True;break
            pump(.2)
        if not active:raise RuntimeError(name+' not active: '+states[name])
    result['lifecycle']=states
    if not until(lambda:counts.get('/scan_front',0)>5 and counts.get('/scan_back',0)>5,5):raise RuntimeError('bridge missing scans')
    unexpected=[x.node_name for x in dst.get_subscriptions_info_by_topic('/navigation/cmd_vel_guard') if x.node_name!='commissioning_nav2_dry_sink']
    if unexpected:raise RuntimeError('unexpected final output consumer '+str(unexpected))
    t=b.lookup_transform('map','base_link',rclpy.time.Time());q=t.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=dst.get_clock().now().to_msg();goal.pose.pose.position.x=t.transform.translation.x+.3*math.cos(yaw);goal.pose.pose.position.y=t.transform.translation.y+.3*math.sin(yaw);goal.pose.pose.orientation=q
    action=ActionClient(dst,NavigateToPose,'/navigate_to_pose')
    if not until(action.server_is_ready,5):raise RuntimeError('navigation action unavailable')
    f=action.send_goal_async(goal)
    if not until(f.done,5):raise RuntimeError('goal timeout')
    handle=f.result();result['goal_accepted']=handle.accepted
    if not handle.accepted:raise RuntimeError('goal rejected')
    result_future=handle.get_result_async();pump(3.)
    result['terminal_before_cancel']=result_future.done()
    cancel_time=time.monotonic()-started;cancel=handle.cancel_goal_async()
    if not until(cancel.done,5):raise RuntimeError('cancel response timeout')
    result['cancel_return_code']=cancel.result().return_code
    if not until(result_future.done,8):raise RuntimeError('cancelled result timeout')
    result['goal_status']=result_future.result().status;result['navigation_error_code']=getattr(result_future.result().result,'error_code',None)
    pump(1.5);result['cancel_elapsed_s']=cancel_time;result['bridge_counts']=counts;result['commands']=command_rows
    summary={}
    for topic,rows in command_rows.items():
        tail=[x for x in rows if x['elapsed_s']>=cancel_time+.7]
        summary[topic]={'samples':len(rows),'nonzero_samples':sum(math.hypot(x['vx'],x['vy'])>1e-5 or abs(x['w'])>1e-5 for x in rows),'last':rows[-1] if rows else None,'nonzero_after_cancel_settle':sum(math.hypot(x['vx'],x['vy'])>1e-5 or abs(x['w'])>1e-5 for x in tail)}
    result['summary']=summary
    result['passed']=result['goal_accepted'] and result['goal_status']==5 and summary['/navigation/cmd_vel_raw']['nonzero_samples']>0 and all(x['nonzero_after_cancel_settle']==0 for x in summary.values())
    print(json.dumps({k:v for k,v in result.items() if k!='commands'},indent=2),flush=True)
except Exception as e:
    result['error']=str(e);result['passed']=False;print('DRY TEST FAILED:',e,flush=True)
finally:
    for p in reversed(children):p.terminate()
    for p in children:
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    for f in streams:f.close()
    Path('/tmp/agv_nav2_realdata_dry_result_20261113.json').write_text(json.dumps(result,indent=2))
    se.shutdown();de.shutdown();src.destroy_node();dst.destroy_node();rclpy.shutdown(context=src_ctx);rclpy.shutdown(context=dst_ctx)
if not result.get('passed'):raise SystemExit(1)
