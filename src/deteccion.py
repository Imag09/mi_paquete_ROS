#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import os
import numpy as np
import tensorflow as tf
import time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolov5_detector')

        # Cargar los modelos YOLOv5 y H5
        model_path = '/home/asus/pro_ws/src/my_bot/src/OjosM.pt'
        h5_model_path = '/home/asus/pro_ws/src/my_bot/src/intento14.h5'

        if os.path.exists(model_path):
            self.get_logger().info(f"Cargando el modelo YOLOv5 desde {model_path}")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
            self.model.eval()
        else:
            self.get_logger().error(f"No se encontró el archivo del modelo YOLOv5: {model_path}")
            return

        if os.path.exists(h5_model_path):
            self.get_logger().info(f"Cargando el modelo H5 desde {h5_model_path}")
            self.h5_model = tf.keras.models.load_model(h5_model_path, compile=False)
        else:
            self.get_logger().error(f"No se encontró el archivo del modelo H5: {h5_model_path}")
            return

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Error al abrir la transmisión de video')
            return

        # Cliente de acción para enviar goal poses
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Esperando al servidor de acciones...')
        self._action_client.wait_for_server()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publisher_ = self.create_publisher(Image, 'yolov5_output', 10)

        self.gaze_start_time = None
        self.gaze_direction = None
        self.blink_count = 0

        self.prev_time = time.time()
        self.frame_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Fallo al capturar el fotograma')
            return

        # Inicializa la variable fps
        fps = 0.0

        # Calcular FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.prev_time >= 1.0:  # Cada segundo
            fps = self.frame_count / (current_time - self.prev_time)
            self.frame_count = 0
            self.prev_time = current_time

        resized_frame = cv2.resize(frame, (640, 480))
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        results = self.model(rgb_frame)
        bounding_boxes = results.xywh[0].cpu().numpy()

        for box in bounding_boxes:
            x_center, y_center, width, height, confidence, class_id = box
            x1, y1 = int(x_center - width / 2), int(y_center - height / 2)
            x2, y2 = int(x_center + width / 2), int(y_center + height / 2)
            roi = resized_frame[y1:y2, x1:x2]

            roi_resized = cv2.resize(roi, (96, 96))
            roi_normalized = roi_resized / 255.0
            roi_input = np.expand_dims(roi_normalized, axis=0)

            h5_prediction = self.h5_model.predict(roi_input)
            predicted_class = np.argmax(h5_prediction, axis=1)
            current_gaze_direction = self.get_gaze_direction(predicted_class[0])

            if current_gaze_direction != self.gaze_direction:
                self.gaze_direction = current_gaze_direction
                self.gaze_start_time = time.time()

            if (self.gaze_direction == 'Derecha' or self.gaze_direction == 'Izquierda') and time.time() - self.gaze_start_time > 3:
                self.get_logger().info(f"Mirada a la {self.gaze_direction} detectada durante 3 segundos, esperando parpadeo")
                self.blink_count += 1
                if self.blink_count == 4:
                    self.handle_blink()
                    self.blink_count = 0

            cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(resized_frame, current_gaze_direction, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Mostrar FPS en la imagen
        cv2.putText(resized_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        frame_bgr = cv2.cvtColor(resized_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('YOLOv5 Detection', frame_bgr)
        cv2.waitKey(1)

    def handle_blink(self):
        if self.gaze_direction == 'Derecha':
            self.send_goal_pose(3.45, 1.42, 0.0, 1.0)
        elif self.gaze_direction == 'Izquierda':
            self.send_goal_pose(2.30, -1.39, 0.62, 0.78)

    def send_goal_pose(self, x, y, z_orientation=0.0, w_orientation=1.0):
        self.get_logger().info(f'Enviando goal pose: x={x}, y={y}')
        goal_pose = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=z_orientation, w=w_orientation)
            )
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Enviar la goal pose de manera asíncrona
        goal_future = self._action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal pose rechazada')
            return

        self.get_logger().info('Goal pose aceptada')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal alcanzada con éxito')
        else:
            self.get_logger().info('Falló al alcanzar la goal pose')

    def get_gaze_direction(self, predicted_class):
        directions = {
            0: 'Centro',
            1: 'Cerrado',
            2: 'Derecha',
            3: 'Izquierda',
        }
        return directions.get(predicted_class, 'Desconocido')

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
