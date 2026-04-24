import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from scipy.ndimage import binary_dilation
import numpy as np
import heapq

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.map = None          # latest OccupancyGrid
        self.inflated = None     # numpy 2D array, True = blocked
        self.robot_pose = None   # (x, y) in world frame
        self.goal = None
        self.running = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.map_wp)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        # Start pose — get from /amcl_pose, or from TF (map -> base_link), or hardcode for testing
        #self.create.subscription(PoseWithCovarianceStamped, '/pose', self.map_wp, 10)
        # Goal pose — easiest is to click "2D Goal Pose" in RViz, which publishes to /goal_pose
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

    def map_cb(self, msg: OccupancyGrid):
        if not self.running:
            self.running = True
            self.map = msg
            # Reshape flat data into 2D (rows = height, cols = width)
            grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # Block anything >50 or unknown
            blocked = (grid > 25) #| (grid < 0)
            radius_cells=int(np.ceil(0.18 / msg.info.resolution))
            # Inflate by robot radius
            self.inflated = self.inflate(blocked, radius_cells)

            if self.goal is None or self.robot_pose is None:
                self.get_logger().warn('No goal or pose yet')
                self.running = False
                return
            path_cells = self.astar(self.world_to_grid(self.robot_pose),
                                    self.world_to_grid((self.goal.pose.position.x, self.goal.pose.position.y)))
            if path_cells is None:
                self.get_logger().warn('No path found')
                self.running = False
                return
            self.path_pub.publish(self.cells_to_path_msg(path_cells))
            self.running = False

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def map_wp(self):
        # in a timer:
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_pose = (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}')

    def inflate(self, blocked, radius_cells):
            """Expand occupied cells outward by `radius_cells` cells."""
            if radius_cells <= 0:
                return blocked
            # Build a circular structuring element
            y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
            kernel = (x*x + y*y) <= radius_cells*radius_cells
            return binary_dilation(blocked, structure=kernel)

    def world_to_grid(self, xy):
        """(x, y) in meters -> (col, row) integer cell indices."""
        x, y = xy
        info = self.map.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return (col, row)

    def grid_to_world(self, cell):
        """(col, row) -> (x, y) at the center of that cell, in meters."""
        col, row = cell
        info = self.map.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return (x, y)
    
    def cells_to_path_msg(self, cells):
        """Convert a list of (col, row) cells into a nav_msgs/Path in the map frame."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for c in cells:
            x, y = self.grid_to_world(c)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0   # no rotation info - follower can ignore
            path.poses.append(ps)
        return path

    def astar(self, start, goal):
        self.get_logger().warn(f"{start}")
        self.get_logger().warn(f"{goal}")
        h = lambda a, b: np.hypot(a[0]-b[0], a[1]-b[1])  
        open_set = [(h(start, goal), 0, start, None)]    # (f, g, cell, parent_idx)
        came_from = {}
        g_score = {start: 0}
        neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

        while open_set:
            f, g, cur, parent = heapq.heappop(open_set)
            if cur in came_from: continue
            came_from[cur] = parent
            if cur == goal:
                # reconstruct
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                return path[::-1]
            for dc, dr in neighbors:
                nb = (cur[0]+dc, cur[1]+dr)
                if not (0 <= nb[0] < self.map.info.width and 0 <= nb[1] < self.map.info.height):
                    continue
                if self.inflated[nb[1], nb[0]]:   # note: [row, col] = [y, x]
                    continue
                tentative = g + np.hypot(dc, dr)
                if tentative < g_score.get(nb, float('inf')):
                    g_score[nb] = tentative
                    heapq.heappush(open_set, (tentative + h(nb, goal), tentative, nb, cur))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()