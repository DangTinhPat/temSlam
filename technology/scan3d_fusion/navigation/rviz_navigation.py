"""RViz tools and displays for kinematic navigation on the existing scanned mesh.

ROS is an input/display boundary. No cmd_vel publisher or native backend exists
here; local costmap is a crop of the static mesh projection, not live sensing.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,PointStamped,Point,TransformStamped
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import TransformBroadcaster

from grid import NavigationGrid,FREE,OCCUPIED,UNKNOWN
from controller import Navigator

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_MESH=ROOT/'datasets/processed/scan3d_fusion/room-floor-preserved-20261112/final_mesh.stl'


def read_pose(message):
    if message.header.frame_id!='map':raise ValueError('Pose/goal must use the map frame')
    pose=message.pose.pose if isinstance(message,PoseWithCovarianceStamped) else message.pose
    q=pose.orientation;p=pose.position
    values=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    if (not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1)>.001
            or abs(q.x)>.001 or abs(q.y)>.001):
        raise ValueError('Require a finite XY pose and planar unit quaternion')
    return np.array([p.x,p.y,2*math.atan2(q.z,q.w)])


def grid_message(grid,*,inflated=False,crop=None):
    values=np.where(grid.cells==UNKNOWN,-1,np.where(grid.cells==OCCUPIED,100,0)).astype(np.int8)
    if inflated:values[(grid.cells==FREE)&grid.blocked]=99
    x0,y0,x1,y1=crop or (0,0,values.shape[1],values.shape[0])
    values=values[y0:y1,x0:x1]
    msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=grid.resolution
    msg.info.height,msg.info.width=values.shape
    msg.info.origin.position.x=float(grid.origin[0]+x0*grid.resolution)
    msg.info.origin.position.y=float(grid.origin[1]+y0*grid.resolution)
    msg.info.origin.position.z=.04 if inflated else .03
    msg.info.origin.orientation.w=1.;msg.data=values.ravel().tolist()
    return msg


def pose_stamped(xy,yaw=0.,height=.09):
    p=PoseStamped();p.header.frame_id='map'
    p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.position.z=height
    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
    return p


class NavigationNode(Node):
    def __init__(self,grid,mesh,initial_pose):
        super().__init__('rb_rviz_navigation_demo')
        self.grid,self.mesh=grid,Path(mesh)
        self.pose=np.asarray(initial_pose,float).copy()
        if self.pose.shape!=(3,) or not np.isfinite(self.pose).all() or not grid.segment_clear(self.pose[:2],self.pose[:2]):
            raise ValueError('Initial demo pose must lie in traversable map space')
        self.navigator=Navigator(grid);self.trail=[self.pose.tolist()];self.last_tick=time.monotonic()
        self.message='Ready: 2D Pose Estimate -> 2D Goal Pose';self.goal=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/rb_navigation/map',qos)
        self.cost_pub=self.create_publisher(OccupancyGrid,'/rb_navigation/global_costmap',qos)
        self.local_pub=self.create_publisher(OccupancyGrid,'/rb_navigation/local_costmap',qos)
        self.plan_pub=self.create_publisher(RosPath,'/rb_navigation/plan',qos)
        self.trail_pub=self.create_publisher(RosPath,'/rb_navigation/trajectory',qos)
        self.marker_pub=self.create_publisher(MarkerArray,'/rb_navigation/markers',qos)
        self.mesh_pub=self.create_publisher(Marker,'/rb_navigation/mesh',qos)
        self.state_pub=self.create_publisher(String,'/rb_navigation/status',qos)
        self.pose_pub=self.create_publisher(PoseStamped,'/rb_navigation/pose',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped,'/rb_navigation/initialpose',self.pose_callback,10)
        self.create_subscription(PoseStamped,'/rb_navigation/goal_pose',self.goal_callback,10)
        self.create_subscription(PointStamped,'/rb_navigation/stop',self.stop_callback,10)
        self.map_pub.publish(grid_message(grid));self.cost_pub.publish(grid_message(grid,inflated=True))
        marker=Marker();marker.header.frame_id='map';marker.ns='scanned_room';marker.type=Marker.MESH_RESOURCE
        marker.mesh_resource=self.mesh.resolve().as_uri();marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=marker.scale.z=1.
        marker.color.r,marker.color.g,marker.color.b,marker.color.a=.75,.72,.67,1.
        self.mesh_pub.publish(marker)
        self.create_timer(.05,self.tick);self.create_timer(.2,self.publish)
        self.publish()

    def pose_callback(self,message):
        self.navigator.cancel()
        try:
            pose=read_pose(message)
            if not self.grid.segment_clear(pose[:2],pose[:2]):raise ValueError('Initial pose is blocked or unknown')
            self.pose=pose;self.navigator=Navigator(self.grid);self.trail=[pose.tolist()];self.goal=None
            self.last_tick=time.monotonic();self.message='Initial pose set; choose 2D Goal Pose'
        except ValueError as error:self.message=str(error)
        self.get_logger().info(self.message);self.publish()

    def goal_callback(self,message):
        self.navigator.cancel()
        try:
            target=read_pose(message)
            result=self.navigator.set_goal(self.pose[:2],target[:2],goal_yaw=target[2])
            self.goal=target;self.last_tick=time.monotonic()
            self.message='Following global plan' if result['state']=='planning' else result['reason']
        except ValueError as error:self.message=str(error)
        self.get_logger().info(self.message);self.publish()

    def stop_callback(self,message):
        self.navigator.cancel();self.message='Stopped by operator; choose a new goal to continue'
        self.publish()

    def step(self,now):
        dt=now-self.last_tick;self.last_tick=now
        previous=self.navigator.state
        if previous in ('planning','driving','aligning'):
            pose=dict(xy=self.pose[:2].tolist(),yaw=float(self.pose[2]),stamp=now,usable=True,uncertainty_m=0.)
            result=self.navigator.update(pose,now,dict(stamp=now,clear=True,reason='Static mesh demo'),dt)
            angle=self.pose[2]+result['w']*dt/2
            self.pose[:2]+=result['v']*dt*np.array([math.cos(angle),math.sin(angle)])
            self.pose[2]=math.atan2(math.sin(self.pose[2]+result['w']*dt),math.cos(self.pose[2]+result['w']*dt))
            if np.linalg.norm(self.pose[:2]-self.trail[-1][:2])>.01 or abs(self.pose[2]-self.trail[-1][2])>.03:
                self.trail.append(self.pose.tolist());self.trail=self.trail[-5000:]
            self.message=result['reason'] or result['state']
            if result['state'] not in ('planning','driving','aligning'):
                self.get_logger().info(f"{result['state']}: {result['reason']}")

    def tick(self):
        self.step(time.monotonic())

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        current=pose_stamped(self.pose[:2],self.pose[2],.14305);current.header.stamp=stamp
        self.pose_pub.publish(current)
        transform=TransformStamped();transform.header=current.header;transform.child_frame_id='base_link'
        transform.transform.translation.x=float(self.pose[0]);transform.transform.translation.y=float(self.pose[1])
        transform.transform.translation.z=.14305;transform.transform.rotation=current.pose.orientation
        self.tf.sendTransform(transform)
        for points,pub in [(self.navigator.path,self.plan_pub),(self.trail,self.trail_pub)]:
            path=RosPath();path.header=current.header
            path.poses=[pose_stamped(p[:2],p[2] if len(p)>2 else 0.) for p in points]
            for p in path.poses:p.header.stamp=stamp
            pub.publish(path)
        row,col=self.grid.world_to_cell(self.pose[:2]);radius=int(math.ceil(1.5/self.grid.resolution))
        crop=(max(0,col-radius),max(0,row-radius),min(self.grid.cells.shape[1],col+radius+1),min(self.grid.cells.shape[0],row+radius+1))
        local=grid_message(self.grid,inflated=True,crop=crop);local.header.stamp=stamp;self.local_pub.publish(local)
        markers=[]
        def marker(name,kind,xyz,scale,color):
            m=Marker();m.header=current.header;m.ns=name;m.type=kind;m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,xyz)
            m.scale.x,m.scale.y,m.scale.z=map(float,scale)
            m.color.r,m.color.g,m.color.b,m.color.a=color;markers.append(m);return m
        body=marker('robot_body',Marker.CUBE,[*self.pose[:2],.14305],[.20,.16,.10],[.22,.40,.48,1.])
        body.pose.orientation=current.pose.orientation
        head=marker('robot_heading',Marker.ARROW,[*self.pose[:2],.24],[.32,.04,.05],[.85,.38,.12,1.])
        head.pose.orientation=current.pose.orientation
        footprint=marker('footprint',Marker.LINE_STRIP,[0,0,.08],[.012,0,0],[.1,.4,.45,1.])
        footprint.points=[Point(x=float(self.pose[0]+.25*math.cos(a)),y=float(self.pose[1]+.25*math.sin(a)),z=0.)
                          for a in np.linspace(0,2*math.pi,49)]
        status=marker('status',Marker.TEXT_VIEW_FACING,[.2,-2.4,.8],[0,0,.12],[.15,.19,.22,1.])
        status.text='KINEMATIC DEMO | STATIC MESH COSTMAP\n'+self.message+'\nPublish Point = STOP'
        goal=marker('goal',Marker.ARROW,[0,0,.12],[.35,.04,.06],[.8,.18,.24,1.])
        if self.goal is None:goal.action=Marker.DELETE
        else:
            goal.pose=pose_stamped(self.goal[:2],self.goal[2],.12).pose
        self.marker_pub.publish(MarkerArray(markers=markers))
        state=dict(state=self.navigator.state,message=self.message,pose=self.pose.tolist(),
                   goal=None if self.goal is None else self.goal.tolist(),
                   mode='kinematic_mesh_demo',costmap='static_mesh_projection',native_motion=False)
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh',type=Path,default=DEFAULT_MESH)
    parser.add_argument('--grid',type=Path,help='Use a saved grid instead of projecting the mesh again')
    parser.add_argument('--initial-pose',nargs=3,type=float,default=[.7,-1.6,0.],metavar=('X','Y','YAW_DEG'))
    args=parser.parse_args()
    if not args.mesh.is_file():parser.error('Mesh file not found')
    import trimesh
    from mesh_grid import build_mesh_grid
    grid=(NavigationGrid.load(args.grid) if args.grid is not None else
          build_mesh_grid(trimesh.load(args.mesh,force='mesh').triangles))
    initial=[*args.initial_pose[:2],math.radians(args.initial_pose[2])]
    rclpy.init();node=None
    try:
        node=NavigationNode(grid,args.mesh,initial)
        node.get_logger().info('READY: RViz pose/goal tools connected; kinematic static-map demo only')
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
