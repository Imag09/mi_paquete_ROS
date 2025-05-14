#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from GazeTracking.gaze_tracking import GazeTracking
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class EyeTrackingNode(Node):
    def __init__(self):
        super().__init__('eye_tracking_node')
        self.gaze = GazeTracking()
        self.webcam = cv2.VideoCapture(0)

        # Action Client para enviar goal poses
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Timer para capturar imágenes cada 100ms
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        # Captura un nuevo cuadro de la webcam
        _, frame = self.webcam.read()

        # Envía este cuadro a GazeTracking para analizarlo
        self.gaze.refresh(frame)

        # Generar un marco anotado
        annotated_frame = self.gaze.annotated_frame()

        # Determinar el estado de la mirada
        if self.gaze.is_blinking():
            self.handle_blink()  # Manejar el parpadeo
        elif self.gaze.is_right():
            self.get_logger().info("Looking right, navigating to Goal Pose 2")
            self.navigate_to_goal(2.30, -1.39, 0.62, 0.78)  # Goal Pose 2
        elif self.gaze.is_left():
            self.get_logger().info("Looking left, navigating to Goal Pose 1")
            self.navigate_to_goal(3.45, 1.42)  # Goal Pose 1

        # Mostrar el cuadro anotado
        cv2.imshow("Gaze Tracking", annotated_frame)

        if cv2.waitKey(1) == 27:  # Salir al presionar 'ESC'
            self.webcam.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def handle_blink(self):
        # Maneja el parpadeo y envía una acción específica si es necesario
        self.get_logger().info("Blink detected!")
        # Aquí podrías implementar una acción específica al parpadear, si es necesario

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
    eye_tracking_node = EyeTrackingNode()

    # Ejecutar el nodo
    rclpy.spin(eye_tracking_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
