import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import time

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('create subscription')
        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.get_logger().info('create publisher')
        self.cube_pose_pub = self.create_publisher(PointStamped, '/transformed_pts', 1)
        #self.chatter = self.create_publisher(String, '/chat', 1)

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        if self.cube_pose is None:
            self.get_logger().info('call transform_cube_pose')
            self.cube_pose = self.transform_cube_pose(msg)

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frameassed to lookupTransform argument target_fram
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """
        # try:
        #     target_frame = 'base_link'
        #     tf_msg = self.tf_buffer.lookup_transform(
        #         target_frame,
        #         msg.header.frame_id,
        #         rclpy.time.Time()
        #     )

        #     time.sleep(0.02)
        # except:
        #     # self.transformed_points.publish(None)
        #     return None
        # tx = tf_msg.transform.translation.x
        # ty = tf_msg.transform.translation.y
        # tz = tf_msg.transform.translation.z
        # qx = tf_msg.transform.rotation.x
        # qy = tf_msg.transform.rotation.y
        # qz = tf_msg.transform.rotation.z
        # qw = tf_msg.transform.rotation.w
        
        # R00 = 1.0 - 2.0*(qy*qy + qz*qz)
        # R01 = 2.0*(qx*qy - qw*qz)
        # R02 = 2.0*(qx*qz + qw*qy)

        # target_frame = 'base_link'
        # tf_msg = self.tf_buffer.lookup_transform(
        #     target_frame,
        #     msg.header.frame_id,
        #     rclpy.time.Time()
        # )

        # tx = tf_msg.transform.translation.x
        # ty = tf_msg.transform.translation.y
        # tz = tf_msg.transform.translation.z
        # qx = tf_msg.transform.rotation.x
        # qy = tf_msg.transform.rotation.y
        # qz = tf_msg.transform.rotation.z
        # qw = tf_msg.transform.rotation.w
        
        # R00 = 1.0 - 2.0*(qy*qy + qz*qz)
        # R01 = 2.0*(qx*qy - qw*qz)
        # R02 = 2.0*(qx*qz + qw*qy)
        # R10 = 2.0*(qx*qy + qw*qz)
        # R11 = 1.0 - 2.0*(qx*qx + qz*qz)
        # R12 = 2.0*(qy*qz - qw*qx)
        # R20 = 2.0*(qx*qz - qw*qy)
        # R21 = 2.0*(qy*qz + qw*qx)
        # R22 = 1.0 - 2.0*(qx*qx + qy*qy)

        # px = msg.point.x
        # py = msg.point.y
        # pz = msg.point.z

        # x_t = R00*px + R01*py + R02*pz + tx
        # y_t = R10*px + R11*py + R12*pz + ty
        # z_t = R20*px + R21*py + R22*pz + tz

        # out = PointStamped()
        # out.header.stamp = msg.header.stamp
        # out.header.frame_id = target_frame
        # out.point.x = x_t
        # out.point.y = y_t
        # out.point.z = z_t

        tf_msg = None
        for i in range(5):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'base_link',
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.15)
                )
            except Exception as e:
                if i == 4:
                    self.get_logger().warn(f'TF lookup failed {e}')
                    return
                time.sleep(0.05)
        # try:
        #     tf_msg = self.tf_buffer.lookup_transform(
        #         'base link',
        #         msg.header.frame_id,
        #         msg.header.stamp,
        #         rclpy.duration.Duration(seconds=0.5)
        #     )
        # except Exception as e:
        #     self.get_logger().warn(f'TF lookup failed {e}')
        
        # t = tf_msg.transform.translation
        # q = tf_msg.transform.rotation
        # T = quaternion_matrix([q.x, q.y, q.z, q.w])
        # print('---------------------------------------------------')
        # self.get_logger().info('test 1' + T)
        # print('-------------------------------------------------')
        # T[0:3, 3] = [t.x, t.y, t.z]

        # p = np.array([msg.point.x, msg.point.y, msg.point.z, msg.point.w])
        # p_transformed = T.dot(p)

        # out = PointStamped()
        # out.header.stamp = self.get_clock().now().to_msg()
        # out.header.frame_id = 'base_link'
        # out.point.x = float(p_transformed[0])
        # out.point.y = float(p_transformed[1])
        # out.point.z = float(p_transformed[2])

        # tx = tf_msg.transform.translation.x
        # ty = tf_msg.transform.translation.y
        # tz = tf_msg.transform.translation.z
        # qx = tf_msg.transform.rotation.x
        # qy = tf_msg.transform.rotation.y
        # qz = tf_msg.transform.rotation.z
        # qw = tf_msg.transform.rotation.w
        
        # R00 = 1.0 - 2.0*(qy*qy + qz*qz)
        # R01 = 2.0*(qx*qy - qw*qz)
        # R02 = 2.0*(qx*qz + qw*qy)

        # target_frame = 'base_link'
        # tf_msg = self.tf_buffer.lookup_transform(
        #     target_frame,
        #     msg.header.frame_id,
        #     rclpy.time.Time()
        # )

        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z
        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w
        
        R00 = 1.0 - 2.0*(qy*qy + qz*qz)
        R01 = 2.0*(qx*qy - qw*qz)
        R02 = 2.0*(qx*qz + qw*qy)
        R10 = 2.0*(qx*qy + qw*qz)
        R11 = 1.0 - 2.0*(qx*qx + qz*qz)
        R12 = 2.0*(qy*qz - qw*qx)
        R20 = 2.0*(qx*qz - qw*qy)
        R21 = 2.0*(qy*qz + qw*qx)
        R22 = 1.0 - 2.0*(qx*qx + qy*qy)

        px = msg.point.x
        py = msg.point.y
        pz = msg.point.z

        x_t = R00*px + R01*py + R02*pz + tx
        y_t = R10*px + R11*py + R12*pz + ty
        z_t = R20*px + R21*py + R22*pz + tz

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'
        out.point.x = x_t
        out.point.y = y_t
        out.point.z = z_t

        self.cube_pose_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
