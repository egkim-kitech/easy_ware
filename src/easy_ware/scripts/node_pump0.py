#!/usr/bin/env python3
from pyroutelib3 import Router # Import the router
import rospy
import sys
import os
import utm
import math
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3


class PumpNode:
    def __init__(self):
        rospy.init_node("pump_osm")
        self.pub = rospy.Publisher(
            "route_pump_marker_array", MarkerArray, latch=True, queue_size=10
        )
        # Publisher for /node_route_complete
        self.route_complete_pub = rospy.Publisher('/node_route_complete', Int32, queue_size=10, latch=True)

        # 1. transform listener : this listener will transform pose from /world to /map frame
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/world", "/map", rospy.Time(), rospy.Duration(4.0))

        # 2. subscribe /initialpose : this subscriber will get pose from /initialpose topic (rviz)
        self.pose_x = None
        self.pose_y = None
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.posecallback)
        # 3. subscribe /move_base_simple/goal : this subscriber will get pose from /move_base_simple/goal topic (move_base)
        self.goal_x = None
        self.goal_y = None
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalcallback)
        # 4. get param from launch file, default is vector.osm
        self.current_path = os.path.dirname(os.path.realpath(__file__))
        self.osm_file = rospy.get_param("~vector_file_path", self.current_path + "/../map/second_map.osm")
        self.router = Router("car", self.osm_file) # Initialise it

        # 5. create a router object
        self.msg = None
        self.timer_interval = rospy.Duration(0.1)
        self.marker_life = self.timer_interval + rospy.Duration(1)
        rospy.Timer(self.timer_interval, self.timer_callback)

    def posecallback(self, msg):
        # get pose from msg
        pose = msg.pose.pose
        # get x,y from pose
        map_point = PointStamped()
        map_point.header.frame_id = "/map"
        map_point.header.stamp = rospy.Time(0)
        map_point.point.x = pose.position.x
        map_point.point.y = pose.position.y

        world_point = self.listener.transformPoint("/world", map_point)
        self.pose_x = world_point.point.x
        self.pose_y = world_point.point.y
        print(self.pose_x, self.pose_y)

    def goalcallback(self, msg):
        # get pose from msg
        pose = msg.pose
        # get x,y from pose
        map_point = PointStamped()
        map_point.header.frame_id = "/map"
        map_point.header.stamp = rospy.Time(0)
        map_point.point.x = pose.position.x
        map_point.point.y = pose.position.y

        world_point = self.listener.transformPoint("/world", map_point)
        self.goal_x = world_point.point.x
        self.goal_y = world_point.point.y
        print(self.goal_x, self.goal_y)

        route_x, route_y = self.get_route()
        if route_x and route_y:
            print(route_x, route_y)
            route_px, route_py = self.route_interpolation(route_x, route_y, 2.0)
            self.create_markers(route_px, route_py, msg)

    def create_markers(self, route_px, route_py, msg):
        self.msg = MarkerArray()
        index = 0
        for i in range(len(route_px) - 1):
            marker = Marker(
                header=msg.header,
                ns="route_pump_osm",
                id=index,
                type=Marker.ARROW,
                action=Marker.ADD,
                scale=Vector3(x=2.0, y=1.0, z=1.0),
                color=ColorRGBA(r=0.5, g=1.0, b=0, a=0.8),
                lifetime=self.marker_life,
            )
            marker.header.frame_id = "world"
            index += 1
            marker.pose.position.x = route_px[i]
            marker.pose.position.y = route_py[i]
            marker.pose.position.z = 0.5

            # Calculate orientation using a quaternion
            delta_x = route_px[i + 1] - route_px[i]
            delta_y = route_py[i + 1] - route_py[i]
            angle = math.atan2(delta_y, delta_x)

            # Convert yaw angle to quaternion
            quaternion = Quaternion()
            quaternion.z = math.sin(angle / 2.0)
            quaternion.w = math.cos(angle / 2.0)

            marker.pose.orientation = quaternion

            self.msg.markers.append(marker)

    def get_route(self):
        # Convert UTM coordinates to latitude and longitude for start and end points
        start_latlon = utm.to_latlon(self.pose_x, self.pose_y, 52, 'S')  # Assuming zone is 52, southern hemisphere
        end_latlon = utm.to_latlon(self.goal_x, self.goal_y, 52, 'S')  # Assuming zone is 52, southern hemisphere
        print("위도 경도:", start_latlon, end_latlon)

        start_nodes = self.find_nearby_nodes(start_latlon, 2, 5)
        end_node = self.router.findNode(end_latlon[0], end_latlon[1])
        
        for start_node in start_nodes:
            status, route = self.router.doRoute(start_node, end_node)  # Find the route - a list of OSM nodes
            if status == 'success':
                print('Success')
                routeLatLons = list(map(self.router.nodeLatLon, route))  # Get actual route coordinates
                route_x = []
                route_y = []
                for latlon in routeLatLons:
                    r = utm.from_latlon(latlon[0], latlon[1])
                    route_x.append(r[0])
                    route_y.append(r[1])

                # Publish '1' to /node_route_complete upon success
                self.route_complete_pub.publish(1)
                return route_x, route_y
        print('!!!Failed (Check for the connections and directions)!!!')
        return [], []

    def find_nearby_nodes(self, latlon, step_meters, steps):
        base_lat, base_lon = latlon
        node_ids = [self.router.findNode(base_lat, base_lon)]

        for i in range(-steps, steps + 1):
            if i == 0:
                continue
            offset = (i * step_meters) / 111320.0  # Roughly convert meters to degrees
            nearby_lat = base_lat + offset
            nearby_lon = base_lon + offset

            node_ids.append(self.router.findNode(nearby_lat, base_lon))
            node_ids.append(self.router.findNode(base_lat, nearby_lon))

        return node_ids

    def route_interpolation(self, route_utmx, route_utmy, length):
        route_x = []
        route_y = []
        for i in range(len(route_utmx) - 1):
            x = route_utmx[i]
            y = route_utmy[i]

            dx = route_utmx[i + 1] - route_utmx[i]
            dy = route_utmy[i + 1] - route_utmy[i]

            if math.sqrt(dx ** 2 + dy ** 2) > length:
                n = math.floor(math.sqrt(dx ** 2 + dy ** 2) / length)
                dx = dx / n
                dy = dy / n
                for j in range(n):
                    route_x.append(x + dx * (j + 1))
                    route_y.append(y + dy * (j + 1))
        return route_x, route_y

    def timer_callback(self, event):
        """Called periodically to refresh map visualization."""
        if self.msg is not None:
            now = rospy.Time()
            for m in self.msg.markers:
                m.header.stamp = now
            self.pub.publish(self.msg)


def main():
    viznode = PumpNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
