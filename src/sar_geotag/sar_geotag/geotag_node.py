#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition,VehicleAttitude
from std_msgs.msg import String
from geopy.distance import distance
from geopy.point import Point
from sklearn.cluster import DBSCAN
import json
import math
import numpy as np
import os
import csv
import time

class GeotagNode(Node):
    qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     history = HistoryPolicy.KEEP_LAST,
                     depth =1)
    def __init__(self):
        super().__init__('geotagnode')
        # self.image_width = 640
        # self.image_height = 480
        # self.hfov = 1.3962634 # Horizontal FoV in radians
        # self.fx = (self.image_width / 2.0) / math.tan(self.hfov / 2.0) # Compute focal length
        # self.fy = self.fx # Compute square pixel
        # self.cx = self.image_width / 2.0 # Compute principal point
        # self.cy = self.image_height / 2.0
        self.image_width = 848
        self.image_height = 480

        self.fx = 454.6857718666893
        self.fy = 454.6857718666893
        self.cx = 424.5
        self.cy = 240.5
        self.home_lat = 47.397742 # PX4 default SITL home latitude
        self.home_lon = 8.545594 # PX4 default SITL home longitude
        # self.save_clustered_results()

        # State variables
        self.local_position = VehicleLocalPosition()
        self.attitude = VehicleAttitude()
        self.geotags = [] # Stores all the detection
        self.output_file = os.path.expanduser('~/soc_ws/geotags.csv') # Path to CSV output file
        self.geojson_file = os.path.expanduser('~/soc_ws/geotags.geojson')
        self.geojson_data = {
                    "type": "FeatureCollection",
                    "features": []
        }

        # Initialising the publishers
        self.geotag_pub = self.create_publisher(String,
                                                '/drone/geotags',self.qos) # publishes confirmed geotag as JSON

        # Initialising the subscribers 
        self.create_subscription(String,
                                 '/drone/detections',
                                 self.detection_callback,self.qos)

        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.local_position_callback,self.qos)

        self.create_subscription(VehicleAttitude,
                                 '/fmu/out/vehicle_attitude',
                                 self.attitude_callback,self.qos)
        
        self.create_timer(10.0, self.save_clustered_results)

        self.init_csv()
    
    def local_position_callback(self, msg):
        self.local_position = msg

    def attitude_callback(self, msg):
        self.attitude = msg

    def init_csv(self):
        with open(self.output_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['id', 'x_ned', 'y_ned', 'latitude','longitude','confidence', 'timestamp'])

        self.get_logger().info(f'Geotag CSV initialized')

    def quaternion_to_rotation_matrix(self,q):
        w, x, y, z = q
        rotation_matrix = np.array([[1 - 2*(y**2 + z**2),
                                   2*(x*y - w*z),
                                   2*(x*z + w*y)],
                                   [2*(x*y + w*z),
                                   1 - 2*(x**2 + z**2),
                                   2*(y*z - w*x)],
                                   [2*(x*z - w*y),
                                   2*(y*z + w*x),
                                   1 - 2*(x**2 + y**2)]])
        return rotation_matrix # standard quaternion to 3*3 matrix 

    def pixel_to_world(self, cx_pixel, cy_pixel, drone_x, drone_y, drone_z):
        ray_cam = np.array([(cx_pixel - self.cx) / self.fx,
                            (cy_pixel - self.cy) / self.fy,
                            1.0
                            ]) # Built ray in camera frame 
        
        R_cam_to_body = np.array([ # Camera is mounted pointing down
            #,so rotate ray from camera to body frame
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])

        ray_body = R_cam_to_body @ ray_cam

        # Get drone rotation matrix from attitude quaternion
        q = self.attitude.q # [w, x, y, z]
        R_body_to_ned = self.quaternion_to_rotation_matrix(q)
        ray_ned = R_body_to_ned @ ray_body

        if ray_ned[2] <= 0:
            return None # ray pointing up, no ground intersection
        
        t = -drone_z / ray_ned[2]
        world_x = drone_x + t * ray_ned[0]
        world_y = drone_y + t * ray_ned[1]

        return (world_x, world_y)

    def ned_to_gps(self,x_ned, y_ned):
        north_point = distance(meters=x_ned).destination(
            Point(self.home_lat, self.home_lon),
            bearing=0 # 0 degrees = north        
            )
        
        final_point = distance(meters=y_ned).destination(
            north_point,
            bearing=90 # 90 degrees = east
        )

        return (final_point.latitude, final_point.longitude)

    def cluster_detection(self): # Collects all geotag and groups the adjacent of them
        if len(self.geotags) < 2:
            return []
        
        positions = np.array([[g['x'], g['y']] for g in self.geotags])

        clustering = DBSCAN(eps=2.0, min_samples=2).fit(positions)
        labels = clustering.labels_
        result = []

        for label in np.unique(labels):

            # All geotags belonging to this cluster
            cluster_geotags = [
                g for g, l in zip(self.geotags, labels)
                if l == label
            ]

            average_x = np.mean([g['x'] for g in cluster_geotags])
            average_y = np.mean([g['y'] for g in cluster_geotags])

            average_confidence = np.mean(
                [g['confidence'] for g in cluster_geotags]
            )

            latest_timestamp = max(
                g['timestamp'] for g in cluster_geotags
            )
            
            result.append({
                'cluster_id': int(label),
                'x': float(average_x),
                'y': float(average_y),
                'confidence': float(average_confidence),
                'detection_count': len(cluster_geotags),
                'timestamp': latest_timestamp
            })

        return result
    
    def save_clustered_results(self):
        clusters = self.cluster_detection()
        if len(clusters) == 0:
            return 
        
        output = os.path.expanduser('~/soc_ws/clustered_geotags.csv')
        with open(output, 'w', newline = '') as file:
            writer = csv.writer(file)

            # Header
            writer.writerow([
                'cluster_id',
                'x_ned',
                'y_ned',
                'confidence',
                'detection_count',
                'timestamp'
            ])

            # Rows
            for c in clusters:
                writer.writerow([
                    c['cluster_id'],
                    c['x'],
                    c['y'],
                    c['confidence'],
                    c['detection_count'],
                    c['timestamp']
                ])

        # Log CSV save
        self.get_logger().info(
            f'Saved {len(clusters)} unique person locations'
        )

        # GeoJson output
        clustered_geojson = {
            "type": "FeatureCollection",
            "features" : []
        }

        for c in clusters:
            lat, lon = self.ned_to_gps(c['x'], c['y'])

            feature = {
                "type" : "Feature",
                "geometry" : {
                    "type": "Point",
                    "coordinates" :[
                        lon, lat
                    ]
                },
                "properties": {
                    "cluster_id": c['cluster_id'],
                    "confidence": c['confidence'],
                    "detection_count" : c['detection_count'],
                    "timestamp" : c['timestamp']
                }
            }

            clustered_geojson["features"].append(feature)

            # Save GeoJSON
            clustered_geojson_path = os.path.expanduser(
                '~/soc_ws/clustered_geotags.geojson'
            )

            with open(clustered_geojson_path, 'w') as f:
                json.dump(clustered_geojson, f, indent=2)

    def detection_callback(self,msg):
        # Parse JSON detections
        detection = json.loads(msg.data)

        # Current drone positions
        x = self.local_position.x
        y = self.local_position.y
        z = self.local_position.z

        #Drone not flying yet
        if z == 0:
            return 
        
        # Process each detection
        for det in detection:
            # Convert pixel coordinates to world coordinates
            world_coords = self.pixel_to_world(det['cx'],
                                 det['cy'], 
                                x, y, z)
        
            # Skip invalid results
            if world_coords is None:
                continue

            world_x, world_y = world_coords
            lat, lon = self.ned_to_gps(world_x, world_y)

            # Filter detections outside search area
            if abs(world_x) > 15.0 or abs(world_y) > 15.0:
                self.get_logger().warn(
                    f'Detection outside search area: x={world_x:.1f}, y={world_y:.1f} - skipping'
                )
                continue

            # Create geotag dictionary
            geotag = {
            'x' : world_x,
            'y' : world_y,
            'lat' : lat,
            'lon' : lon,
            'confidence' : det['confidence'],
            'timestamp' : time.time(),
            'coordinates' : [lon, lat] # GeoJson uses [longitude, latitude] order
            }

            # Store geotag
            self.geotags.append(geotag)

            # Append to CSV 
            self.append_to_csv(geotag)

            #Append to GeoJSON
            self.append_to_geojson(geotag, len(self.geotags))

            # Convert geotag to JSON
            json_data = json.dumps(geotag)

            # Convert ROS String message
            geotag_msg = String()
            geotag_msg.data = json_data

            # Publish geotag
            self.geotag_pub.publish(geotag_msg)

            self.save_clustered_results()

            # Log result
            self.get_logger().info(
                f'Geotag: x={world_x:.2f}, y={world_y:.2f}'
            )

    def append_to_geojson(self,geotag,detection_id):
        features = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [geotag['lon'], geotag['lat']]
    },
            "properties": {
                "id": detection_id,
                "confidence": geotag['confidence'],
                "timestamp": geotag['timestamp']
    }
}
        self.geojson_data['features'].append(features)

        with open(self.geojson_file, 'w') as file:
            json.dump(self.geojson_data, file, indent=2)
            
        self.get_logger().info(f'GeoJSON updated with {len(self.geojson_data["features"])} features')


    def append_to_csv(self,geotag):
        
        with open(self.output_file, 'a', newline='') as file:
            writer = csv.writer(file)

            writer.writerow([
                len(self.geotags),
                geotag['x'],
                geotag['y'],
                geotag['lat'],
                geotag['lon'],
                geotag['confidence'],
                geotag['timestamp']
            ])


def main(args=None):
    rclpy.init(args=args)
    node = GeotagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

