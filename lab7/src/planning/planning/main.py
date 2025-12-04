# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped 
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
import numpy as np
import sys

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pub = self.create_subscription(PointStamped, '/transformed_pts', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.current_plan = None
        self.joint_state = None

        self.ik_planner = IKPlanner()

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def cube_callback(self, cube_pose):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose

        # -----------------------------------------------------------
        # TODO: In the following section you will add joint angles to the job queue. 
        # Entries of the job queue should be of type either JointState or String('toggle_grip')
        # Think about you will leverage the IK planner to get joint configurations for the cube grasping task.
        # To understand how the queue works, refer to the execute_jobs() function below.
        # -----------------------------------------------------------

        # 1) Move to Pre-Grasp Position (gripper above the cube)
        '''
        Use the following offsets for pre-grasp position:
        x offset: 0.0
        y offset: -0.035 (Think back to lab 5, why is this needed?)
        z offset: +0.185 (to be above the cube by accounting for gripper length)
        '''
        # cx = cube_pose.point.x
        # cy = cube_pose.point.y
        # cz = cube_pose.point.z
        # pre_x = cx
        # pre_y = cy - 0.035
        # pre_z = cz + 0.185
        # pre_js = self.ik_planner.compute_ik(self.joint_state, pre_x, pre_y, pre_z)
        # self.job_queue.append(pre_js)
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            'base_link',          # target frame
            'tool0',  # source frame (or 'ee_link', depending on MoveIt setup)
            rclpy.time.Time()
        )
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        z = float(tf.transform.translation.z)
        if len(sys.argv) > 1:
            dx = float(sys.argv[1])
            dy = float(sys.argv[2])
            dz = float(sys.argv[3])
            delta = self.ik_planner.compute_ik(self.joint_state, x + dx, y + dy, z + dz)
            self.job_queue.append(delta)

            gripper = float(sys.argv[4])
            if gripper > 0.0:
                self.job_queue.append('toggle_grip')
        

        # 2) Move to Grasp Position (lower the gripper to the cube)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16.
        '''
        # grasp_z = cz + 0.19
        # grasp_js = self.ik_planner.compute_ik(self.joint_state, pre_x, pre_y, grasp_z)
        # self.job_queue.append(grasp_js)

        # # 3) Close the gripper. See job_queue entries defined in init above for how to add this action.
        # self.job_queue.append('toggle_grip')
        
        # # 4) Move back to Pre-Grasp Position
        # self.job_queue.append(pre_js)

        # 5) Move to release Position
        '''
        We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
        Which offset will you change to achieve this and in what direction?
        '''
        # release_pos_x = cx + 0.4
        # release_pos_y = pre_y
        # release_pos_z = pre_z
        # release_js = self.ik_planner.compute_ik(self.joint_state, release_pos_x, release_pos_y, release_pos_z)
        # self.job_queue.append(release_js)

        # 6) Release the gripper

        self.execute_jobs()


    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):

            traj = self.ik_planner.plan_to_joints(next_job)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return

            self.get_logger().info("Planned to position")

            self._execute_joint_trajectory(traj.joint_trajectory)
        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job

            
    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
