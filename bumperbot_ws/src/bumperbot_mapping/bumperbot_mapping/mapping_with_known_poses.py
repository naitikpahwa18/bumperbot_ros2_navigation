#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer, LookupException
from tf_transformations import euler_from_quaternion

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35


class Pose:
    def __init__(self, px = 0, py = 0):
        self.x = px
        self.y = py

# Helper function to convert world coordinates to map grid indices
# We've converted to coordinates that were expressed within the TF2 Reference frame and converted them as poses on the map.
def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    if pose.x < 0 or pose.x >= map_info.width:
        return False
    if pose.y < 0 or pose.y >= map_info.height:
        return False
    return True

# Since, OccupancyGrid.data is a 1D array, we need to convert the 2D pose to a 1D cell index
def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

# Bresenham's Line Algorithm to get the cells between two poses
def bresenham(start: Pose, end: Pose):
    line = []
    dx = end.x - start.x
    dy = end.y - start.y
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1
    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy
    return line

# Mark 'Occupied' cells in 'Black' and 'Free' cells in 'White' based on the Inverse Sensor Model (Probabilistic Occupancy Grid Mapping)
def inverseSensorModel(pos_robot: Pose, pos_beam: Pose):
    occ_values = []
    line = bresenham(pos_robot, pos_beam)
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))    # Free space
    occ_values.append((line[-1], OCC_PROB))     # Occupied space
    return occ_values
    
# Convert probability to log-odds for Probabilistic Occupancy Grid Mapping
def prob2logodds(p):
    return math.log(p / (1 - p))

# Convert log-odds to probability for Probabilistic Occupancy Grid Mapping
def logodds2prob(l):
    try:
        return 1 - 1 / (1 + math.exp(l))
    except OverflowError:   
        return 0.0 if l < 0 else 1.0


class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").get_parameter_value().double_value
        height = self.get_parameter("height").get_parameter_value().double_value
        resolution = self.get_parameter("resolution").get_parameter_value().double_value

        # Initialize the occupancy grid map
        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        # Initialize the probabilistic map with prior probabilities
        self.probabilistic_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        # Publishers and Subscribers for map and laser scans
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # TF2 Buffer and Listener for pose transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, scan: LaserScan):
        try:
            # Get the transform from odom to base_footprint
            t = self.tf_buffer.lookup_transform("odom", scan.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().warn("Unable to transform between /odom and /base_footprint")
            return

        # Extract robot position from the transform and map it within the occupancy grid
        robot_pos = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)

        # Verify if robot_pos is within map bounds
        if not poseOnMap(robot_pos, self.map_.info):
            self.get_logger().warn("Robot position is out of map bounds")
            return

        # Prepare to transform each laser beam endpoint to odom frame cartesian coordinates
        roll, pitch, yaw = euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y,
                                                    t.transform.rotation.z, t.transform.rotation.w])
        
        for i in range(len(scan.ranges)):
            if math.isinf(scan.ranges[i]) or math.isnan(scan.ranges[i]):
                continue  # Skip invalid range readings
            
            # Calculate Cartesian Coordinates of the laser beam in the fixed map frame wrt odom frame using above info
            angle = scan.angle_min + i * scan.angle_increment + yaw
            px = scan.ranges[i] * math.cos(angle) + t.transform.translation.x
            py = scan.ranges[i] * math.sin(angle) + t.transform.translation.y

            beam_pose = coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_pose, self.map_.info):
                continue  # Skip if the beam pose is out of map bounds
            
            # Mark the cell corresponding to the laser beam endpoint as occupied
            poses = inverseSensorModel(robot_pos, beam_pose)
            for pose, value in poses:
                beam_cell = poseToCell(pose, self.map_.info)

                # Mark whether occupied or free (Probabilistic Mapping Approach)
                self.probabilistic_map_[beam_cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB) 
                # Mark whether occupied or free
                # self.map_.data[beam_cell] = value                   
                

        ## Convert robot pose to cell coordinates (for Testing purposes)
        # robot_cell = poseToCell(robot_pos, self.map_.info)
        # self.map_.data[robot_cell] = 100  # Mark robot position as occupied space

    def timer_callback(self):
        # Publish the occupancy grid map at regular intervals
        self.map_.header.stamp = self.get_clock().now().to_msg()

        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probabilistic_map_]  # Probabilistic Mapping Approach
        self.map_pub.publish(self.map_)

def main():
        rclpy.init()
        node = MappingWithKnownPoses("mapping_with_known_poses")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()