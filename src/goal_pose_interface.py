#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.task import Future
import tkinter as tk  # Importa tkinter para la interfaz gráfica

class GoalPoseNavigator(Node):
    def __init__(self):
        super().__init__('goal_pose_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

    def navigate_to_goal(self, x, y, z_orientation=0.0, w_orientation=1.0):
        self.get_logger().info(f'Navigating to goal: x={x}, y={y}')
        goal_pose = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=z_orientation, w=w_orientation)
            )
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Envía la meta y espera a que se complete
        goal_future = self._action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal result: {}'.format(result))
        if result:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info('Failed to reach the goal.')


def main(args=None):
    rclpy.init(args=args)
    navigator = GoalPoseNavigator()

    # Crear ventana principal
    root = tk.Tk()
    root.title("Control de Navegación del Robot")

    # Funciones que se llaman al presionar los botones
    def go_to_goal_1():
        navigator.navigate_to_goal(3.45, 1.42, 0.0, 1.0)  # Primer objetivo

    def go_to_goal_2():
        navigator.navigate_to_goal(2.30, -1.39, 0.62, 0.78)  # Segundo objetivo

    # Crear botones
    button_goal_1 = tk.Button(root, text="Ir al objetivo 1", command=go_to_goal_1)
    button_goal_2 = tk.Button(root, text="Ir al objetivo 2", command=go_to_goal_2)

    # Colocar botones en la ventana
    button_goal_1.pack(pady=10)
    button_goal_2.pack(pady=10)

    # Ejecutar la interfaz de Tkinter
    root.mainloop()

    # Finalizar rclpy
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
