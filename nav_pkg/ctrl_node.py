import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.lookahead = 0.25          #meters ahead on the path to aim at
        self.goal_tolerance = 0.01     #stop within 1 cm of final waypoint
        self.v_max = 0.25              #m/s forward speed
        self.w_max = 1.0               #rad/s max angular speed
        self.k_ang = 1.5               #P-gain on heading error
        self.turn_in_place_thresh = math.radians(30)  #if off by >30°, rotate in place

        self.path = []                 #list of (x, y)
        self.path_idx = 0              #index of the waypoint we're currently tracking

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.create_timer(0.05, self.control_tick)  #20 Hz

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_idx = 0
        self.get_logger().info(f'Got new path with {len(self.path)} waypoints')

    def get_robot_pose(self):
        """Returns (x, y, yaw) in the map frame, or None if TF isn't ready."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        #quaternion yaw calculation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    def control_tick(self):
        if not self.path:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # goal detector
        gx, gy = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < self.goal_tolerance:
            self.cmd_pub.publish(TwistStamped())   
            self.path = []                  
            self.get_logger().info('Goal reached')
            return

        #advance path_idx past any waypoints that are behind/too close
        while self.path_idx < len(self.path) - 1:
            wx, wy = self.path[self.path_idx]
            if math.hypot(wx - rx, wy - ry) < self.lookahead:
                self.path_idx += 1
            else:
                break
        tx, ty = self.path[self.path_idx]

        #heading error
        target_angle = math.atan2(ty - ry, tx - rx)
        err = self.wrap(target_angle - ryaw)

        cmd = TwistStamped()
        if abs(err) > self.turn_in_place_thresh:
            #Off-heading — rotate in place
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * err))
        else:
            #Roughly pointed right — drive and steer
            cmd.twist.linear.x = self.v_max
            cmd.twist.angular.z = max(-self.w_max, min(self.w_max, self.k_ang * err))
        self.cmd_pub.publish(cmd)

    @staticmethod
    def wrap(a):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

