"""ROS display boundary for native live scan output."""
import json,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    session=Path(sys.argv[1]);rclpy.init();node=Node('rb_live_scan')
    cloud_pub=node.create_publisher(PointCloud2,'/scan3d/cloud',1)
    info_pub=node.create_publisher(Marker,'/scan3d/info',10)
    mesh_pub=node.create_publisher(Marker,'/rgb_tof/clay_mesh',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    last_cloud=0;last_mesh=''
    def tick():
        nonlocal last_cloud,last_mesh
        stamp=node.get_clock().now().to_msg()
        cloud=session/'cloud.npz'
        if cloud.exists() and cloud.stat().st_mtime_ns!=last_cloud:
            last_cloud=cloud.stat().st_mtime_ns
            with np.load(cloud,allow_pickle=False) as f: xyz=f['xyz'].astype('<f4');path=f['path']
            msg=PointCloud2();msg.header.frame_id='map';msg.header.stamp=stamp
            msg.height=1;msg.width=len(xyz);msg.point_step=12;msg.row_step=12*len(xyz);msg.is_dense=True
            msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
            msg.data=xyz.tobytes();cloud_pub.publish(msg)
            m=Marker();m.header=msg.header;m.ns='trajectory';m.id=0;m.type=Marker.LINE_STRIP
            m.pose.orientation.w=1.;m.scale.x=.015;m.color.g=1.;m.color.a=1.
            m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in path];info_pub.publish(m)
        texts=[]
        for stage in ['infer','slam','mesh']:
            try:
                state=json.loads((session/(stage+'_status.json')).read_text())
                texts.append(stage+': '+state['message'])
            except FileNotFoundError: texts.append(stage+': đang khởi động')
        text=Marker();text.header.frame_id='map';text.header.stamp=stamp;text.ns='status';text.id=1
        text.type=Marker.TEXT_VIEW_FACING;text.pose.orientation.w=1.;text.pose.position.z=1.2
        text.scale.z=.08;text.color.r=text.color.g=text.color.b=text.color.a=1.;text.text='\n'.join(texts)
        info_pub.publish(text)
        try:
            meta=json.loads((session/'mesh.json').read_text());filename=meta['file']
            if filename!=last_mesh:
                m=Marker();m.header=text.header;m.ns='surface';m.id=0;m.type=Marker.MESH_RESOURCE
                m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=1.
                m.color.r=.75;m.color.g=.72;m.color.b=.67;m.color.a=1.
                m.mesh_resource=Path(filename).resolve().as_uri();mesh_pub.publish(m);last_mesh=filename
        except FileNotFoundError: pass
    node.create_timer(.2,tick)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node();rclpy.shutdown()
if __name__=='__main__': main()
