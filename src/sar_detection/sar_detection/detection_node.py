#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2 as cv
from ultralytics import YOLO
from std_msgs.msg import String
import json
import time
import os
import numpy as np

class DetectionNode(Node):
    qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     history = HistoryPolicy.KEEP_LAST,
                     depth =1)
    def __init__(self):
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
            )
        super().__init__('detectionnode')
        self.bridge = CvBridge()
        weights_path = os.path.expanduser(
            '~/soc_ws/src/sar_detection/sar_detection/weights/best.pt'
        ) 
        self.model = YOLO(weights_path)#loads the model once at startup
        self.confidence_threshold = 0.40
        self.person_class_id = 0 # in COCO dataset, class 0 is a person
        self.last_time = time.time()
        self.frame_count = 0
        self.camera_info = None

        #Initialising the publishers
        self.drone_detect_pub = self.create_publisher(String,
                                                    '/drone/detections',self.qos)

        self.drone_imag_pub = self.create_publisher(Image,
                                                    '/drone/detection_image', self.qos)

        self.bbox_pub = self.create_publisher(
            Detection2DArray,
            '/drone/detection_boxes',
            10
        )

        #Initialising the subscribers
        self.create_subscription(Image,'/camera/image_raw',self.image_callback,camera_qos)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.camera_info_callback,camera_qos)

    #Detection pipeline
    def camera_info_callback(self,msg):
        self.camera_info = msg

    def publish_detection_array(self, detections, header_stamp):
        array_msg = Detection2DArray()
        array_msg.header.stamp = header_stamp
        array_msg.header.frame_id = 'camera_optical_frame'

        detection_list = []

        for det in detections:
            detection = Detection2D()

            detection.bbox.center.position.x = det['cx']
            detection.bbox.center.position.y = det['cy']
            detection.bbox.size_x = float(det['x2'] - det['x1'])
            detection.bbox.size_y = float(det['y2'] - det['y1'])

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'person'
            hyp.hypothesis.score = det['confidence']
            
            detection.results = [hyp]
            detection_list.append(detection)

        array_msg.detections = detection_list
        self.bbox_pub.publish(array_msg)

    def get_fps(self):
        current_time = time.time()

        fps = 1.0 / (current_time - self.last_time)

        self.last_time = current_time

        return fps

    def image_callback(self,msg):
        self.frame_count += 1
        cv_image = self.bridge.imgmsg_to_cv2(msg,
                                             'bgr8')
        results = self.model(cv_image, 
                            conf=self.confidence_threshold)
        detections = []
        for box in results[0].boxes:
            if int(box.cls[0]) == self.person_class_id:

                #Extract the box co-ordinate
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                #Extract confidence
                confidence = float(box.conf[0])

                #Compute centroid
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                #Create detection dictionary
                detection_dict = {
                    'x1':x1,
                    'y1':y1,
                    'x2':x2,
                    'y2':y2,
                    'confidence':confidence,
                    'cx':cx,
                    'cy':cy
                }

                # Append to detections list
                detections.append(detection_dict)


                #Draw rectangle on the person
                cv.rectangle(cv_image,
                             (int(x1), int(y1)),
                             (int(x2),int(y2)),
                             (0, 255, 0), 
                             2)
                # Draw confidence text
                cv.putText(cv_image,
                           f'Person {confidence:.2f}',
                           (int(x1), int(y1) - 10),
                           cv.FONT_HERSHEY_SIMPLEX,
                           0.5,
                            (0, 255, 0),
                            2)

        fps = self.get_fps() 
        self.get_logger().info(
            f'Detected {len(detections)} persons | FPS: {fps:.1f}'
            )

        #Convering detection list to JSON string
        json_data = json.dumps(detections)

        #Converting cv_image back to ROS image
        detection_msg = String()
        detection_msg.data = json_data

        #Publish detections
        self.drone_detect_pub.publish(detection_msg)

        self.publish_detection_array(
            detections,
            self.get_clock().now().to_msg()
            )

        #Convert annoted image back to ROS
        ros_image = self.bridge.cv2_to_imgmsg(
                cv_image,
                encoding = 'bgr8'
            )
        
        #Publish debug image
        self.drone_imag_pub.publish(ros_image)

        # Log detections
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Detection FPS: {fps:.1f}'
            )


def main(args = None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
