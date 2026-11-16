"""Exercise the running RViz kinematic demo through its actual ROS topics."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,PointStamped
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from std_msgs.msg import String


def main():
    rclpy.init();node=rclpy.create_node('rb_rviz_navigation_check');seen={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[]
    for topic,kind in [('status',String),('global_costmap',OccupancyGrid),('local_costmap',OccupancyGrid),('plan',RosPath),('trajectory',RosPath)]:
        subscriptions.append(node.create_subscription(kind,'/rb_navigation/'+topic,lambda m,key=topic:seen.update({key:m}),qos))
    initial=node.create_publisher(PoseWithCovarianceStamped,'/rb_navigation/initialpose',10)
    goal=node.create_publisher(PoseStamped,'/rb_navigation/goal_pose',10)
    stop=node.create_publisher(PointStamped,'/rb_navigation/stop',10)
    def wait_until(predicate,seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise AssertionError('ROS demo condition timed out: '+str(state()))
    def state():return json.loads(seen['status'].data) if 'status' in seen else {}
    def send_initial():
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.pose.position.x=.7;msg.pose.pose.position.y=-1.6;msg.pose.pose.orientation.w=1.
        initial.publish(msg)
    def send_goal():
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.position.x=.8;msg.pose.position.y=.8
        msg.pose.orientation.z=math.sin(math.pi/4);msg.pose.orientation.w=math.cos(math.pi/4)
        goal.publish(msg)
    try:
        wait_until(lambda:len(seen)==5 and initial.get_subscription_count()>0 and goal.get_subscription_count()>0)
        assert state()['mode']=='kinematic_mesh_demo' and state()['native_motion'] is False
        tools={topic:[p.node_name for p in node.get_publishers_info_by_topic('/rb_navigation/'+topic)]
               for topic in ['initialpose','goal_pose','stop']}
        assert all('rviz' in names for names in tools.values()),tools
        send_initial();wait_until(lambda:state().get('state')=='idle')
        send_goal();wait_until(lambda:state().get('state') in ('driving','aligning'))
        stop.publish(PointStamped());wait_until(lambda:state().get('state')=='cancelled')
        send_initial();wait_until(lambda:state().get('state')=='idle')
        started=time.monotonic();send_goal()
        wait_until(lambda:state().get('state') in ('driving','aligning'))
        wait_until(lambda:state().get('state')=='reached',50)
        final=state();error=math.dist(final['pose'][:2],[.8,.8])
        assert error<=.025 and abs(final['pose'][2]-math.pi/2)<=.04
        assert len(seen['plan'].poses)>2 and len(seen['trajectory'].poses)>10
        costs=set(seen['global_costmap'].data);assert {-1,0,99,100}<=costs
        assert seen['local_costmap'].info.width<seen['global_costmap'].info.width
        report=dict(scope='Real ROS roundtrip with RViz tools; kinematic static mesh demo',
            tool_publishers=tools,cancel_test='passed',result=final,goal_error_m=error,
            wall_seconds=time.monotonic()-started,plan_points=len(seen['plan'].poses),
            trajectory_points=len(seen['trajectory'].poses),global_cost_values=sorted(costs))
        target=Path(__file__).parent/'rviz_navigation_results.json';target.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
