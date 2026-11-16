"""Validate and display the exported diagnostic PLY product, without simulator geometry."""
import argparse
import json
from pathlib import Path

import numpy as np

FILES=dict(obstacle='obstacles.ply',uncertain='uncertain.ply',ground='ground.ply')
STATUS='DIAGNOSTIC\nUNVALIDATED\nUNKNOWN!=FREE'


def load_scan(folder):
    folder=Path(folder)
    report=json.loads((folder/'ground_report.json').read_text())
    if (report.get('ground_source')!='gravity_ransac_labels_v3'
            or report.get('map_quality_status')!='diagnostic/unvalidated'
            or not isinstance(report.get('pose_source'),str) or not report['pose_source'].strip()
            or report.get('classification_pose_source') not in ('synchronized wheel/IMU + body_from_camera',
                                                               'saved estimated Z-up poses; legacy fallback')
            or any(report.get(key) is not False for key in ('ground_truth_geometry','depth_modified','poses_modified'))
            or report.get('exported_files')!=FILES):
        raise ValueError('Require the classified diagnostic export with unchanged measured depth/poses')
    for name in ('depth','pose'):
        before=report.get('source_'+name+'_sha256','');after=report.get('output_'+name+'_sha256','')
        if (not isinstance(before,str) or len(before)!=64 or any(c not in '0123456789abcdef' for c in before)
                or before!=after):raise ValueError('Invalid or mismatched source/output checksums')
    layers={}
    for name,filename in FILES.items():
        count=report.get('layer_points',{}).get(name)
        if type(count) is not int or count<0:raise ValueError('Require nonnegative integer layer counts')
        header=('ply\nformat binary_little_endian 1.0\n'+f'element vertex {count}\n'
                'property float x\nproperty float y\nproperty float z\nend_header\n').encode('ascii')
        with (folder/filename).open('rb') as stream:
            if stream.read(len(header))!=header:raise ValueError('Unexpected PLY header or report count mismatch')
            payload=stream.read()
        if len(payload)!=count*12:raise ValueError('Truncated or oversized PLY payload')
        points=np.frombuffer(payload,dtype='<f4').reshape(-1,3)
        if not np.isfinite(points).all():raise ValueError('PLY points must be finite')
        layers[name]=points
    return layers,report


def view_config(layers):
    import yaml
    config=yaml.safe_load(Path(__file__).with_name('classified.rviz').read_text())
    points=np.concatenate(list(layers.values()))
    if len(points):
        lower,upper=points.min(0),points.max(0)
        view=config['Visualization Manager']['Views']['Current']
        view['Focal Point']=dict(zip('XYZ',map(float,(lower+upper)/2)))
        view['Distance']=max(.5,float(np.linalg.norm(upper-lower))*1.5)
    return config


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scan',type=Path)
    parser.add_argument('--write-config',type=Path,help='Validate files and write the launcher camera configuration')
    args=parser.parse_args();layers,report=load_scan(args.scan)
    if args.write_config:
        import yaml
        args.write_config.write_text(yaml.safe_dump(view_config(layers)))
        return
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import PointCloud2,PointField
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init();node=Node('rb_classified_scan')
    broadcaster=StaticTransformBroadcaster(node)
    origin=TransformStamped();origin.header.frame_id='map';origin.child_frame_id='rb_classified_origin'
    origin.header.stamp=node.get_clock().now().to_msg();origin.transform.rotation.w=1.
    broadcaster.sendTransform(origin)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    items=[]
    for name,points in layers.items():
        message=PointCloud2();message.header.frame_id='map'
        message.height=1;message.width=len(points);message.point_step=12;message.row_step=12*len(points)
        message.is_bigendian=False;message.is_dense=True
        message.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
        message.data=points.tobytes()
        items.append((node.create_publisher(PointCloud2,'/rb_classified/'+name,qos),message))
    points=np.concatenate(list(layers.values()))
    text=Marker();text.header.frame_id='map';text.ns='diagnostic_status';text.id=0
    text.type=Marker.TEXT_VIEW_FACING;text.pose.orientation.w=1.
    extent=float(np.linalg.norm(np.ptp(points,axis=0))) if len(points) else 1.
    if len(points):
        center=(points.min(0)+points.max(0))/2
        text.pose.position.x=float(center[0]);text.pose.position.y=float(center[1])
        text.pose.position.z=float(points[:,2].max())+.12*max(1.,extent)
    text.scale.z=min(.07,max(.04,.025*extent))
    text.color.r=.5;text.color.g=.12;text.color.b=.08;text.color.a=1.
    text.text=STATUS
    items.append((node.create_publisher(Marker,'/rb_classified/status',qos),text))
    def publish():
        for publisher,message in items:
            message.header.stamp=node.get_clock().now().to_msg();publisher.publish(message)
    publish();node.create_timer(2.,publish)
    node.get_logger().info(STATUS.replace('\n','; ')+f'; {args.scan.resolve()} | points {report["layer_points"]}')
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
