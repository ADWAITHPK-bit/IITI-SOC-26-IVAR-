#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)
import math


class OffboardControl(Node): #Used in almost all the PX4 projects 
    qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                     durability = DurabilityPolicy.TRANSIENT_LOCAL,
                     history = HistoryPolicy.KEEP_LAST,
                     depth =1)
    def __init__(self):
        super().__init__('offboardcontrol')
        #State variables
        self.offboard_setpoint_counter = 0 # counts how many setpoints sent before arming
        self.current_waypoint_idx = 0 #tracks which waypoint we're flying tp
        self.mission_complete = False
        self.takeoff_height = -5.0 #NED frame,negative is up
        self.coverage_altitude = -8.0
        self.offboard_pub = self.create_publisher(OffboardControlMode,
                                                  '/fmu/in/offboard_control_mode', self.qos) # OffboardControlMode

        self.traject_pub = self.create_publisher(TrajectorySetpoint,
                                                 '/fmu/in/trajectory_setpoint', self.qos) #TrajectorySetpoint

        self.vecomm_pub = self.create_publisher(VehicleCommand,
                                                '/fmu/in/vehicle_command',self.qos) #VehicleCommand
        self.path_pub = self.create_publisher(
            Path,
            '/drone/coverage_path',
            10
        )

        self.trail_pub = self.create_publisher(
            Path,
            '/drone/trajectory',
            10
        )
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.local_pos_sub = self.create_subscription(VehicleLocalPosition,
                                                    '/fmu/out/vehicle_local_position_v1',
                                                    self.local_position_callback,self.qos) # VehicleLocalPosition

        self.status_sub = self.create_subscription(VehicleStatus,
                                                       '/fmu/out/vehicle_status_v1',
                                                       self.vehicle_status_callback,self.qos) # VehicleStatus
        
        self.trail = Path()
        self.trail.header.frame_id = 'map'

        self.create_timer(0.1, self.control_loop)
        self.waypoints = self.generate_lawnmover_pattern()

    def local_position_callback(self,msg):
        self.local_position = msg

        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(msg.z)

        pose.pose.orientation.w = 1.0

        self.trail.poses.append(pose)
        self.trail.header.stamp = pose.header.stamp

        self.trail_pub.publish(self.trail)

    def vehicle_status_callback(self,msg):
        self.vehicle_status = msg

    def generate_lawnmover_pattern(self):
        search_area_x = 20.0 # total width of search area in meters
        search_area_y = 20.0 # total height
        strip_width = 4.0 # distance between parallel strips
        alt = self.coverage_altitude # altitude to fly at

        x_start = -(search_area_x / 2)
        x_end = +(search_area_x / 2)
        y_start = -(search_area_y / 2)
        y_end = +(search_area_y / 2)

        waypoints = []

        y = y_start 
        strip_num = 0

        while (y <= y_end):
            if (strip_num % 2 == 0):
                waypoints.append((x_start, y, alt))
                waypoints.append((x_end, y, alt))

            else:
                waypoints.append((x_end, y, alt))
                waypoints.append((x_start, y, alt))

            y = y + strip_width
            strip_num = strip_num + 1 
        
        waypoints.append((0.0, 0.0, self.coverage_altitude))
        waypoints.append((0.0, 0.0, self.takeoff_height))

        self.get_logger().info(f'Generated {len(waypoints)} waypoints')
        
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f'WP {i}: x={wp[0]:.1f}, y={wp[1]:.1f}, z={wp[2]:.1f}'
        )

        return waypoints
    
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.traject_pub.publish(msg)

    def publish_path(self,waypoints):
        path_msg = Path()

        path_msg.header.frame_id = 'map' # tells RViZ which coordinates frame the path is in
        path_msg.header.stamp = self.get_clock().now().to_msg() # timestamp

        poses = []

        for waypoint in waypoints:
            
            pose = PoseStamped()

            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0 # means no rotation
            poses.append(pose)
        
        path_msg.poses = poses

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published coverage path with {len(poses)} waypoints')
        

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vecomm_pub.publish(msg)

    def is_at_waypoint(self, x, y, z, threshold = 0.5):
        dx = self.local_position.x - x
        dy = self.local_position.y - y
        dz = self.local_position.z - z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        return dist < threshold
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1 = 1.0)

        self.get_logger().info(f'Arm command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        self.get_logger().info(f'Land command sent')

    def control_loop(self):   

        if self.mission_complete : 
            return 
        
        self.publish_offboard_mode()
    
        if self.offboard_setpoint_counter < 10:
            self.publish_trajectory_setpoint(0.0 ,
                                             0.0, 
                                             self.takeoff_height)

            self.offboard_setpoint_counter += 1
            return 
    
        if self.offboard_setpoint_counter == 10:

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                         param1 = 1.0, 
                                         param2 = 6.0)
            self.arm()
            self.publish_path(self.waypoints)
            self.offboard_setpoint_counter += 1
            return
        
        if self.current_waypoint_idx < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_idx]
            self.publish_trajectory_setpoint(wp[0], wp[1], wp[2])
            if self.is_at_waypoint(wp[0], wp[1], wp[2]):
                self.get_logger().info(f'Reached waypoint{self.current_waypoint_idx+1} / {len(self.waypoints)}')
                self.current_waypoint_idx += 1
        else:
            self.land()
            self.mission_complete = True
            
    


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                                        



        