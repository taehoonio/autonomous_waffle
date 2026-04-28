import rclpy
#Node is the parent class of all nodes so they automatically inherit all ROS2 node functionality
from rclpy.node import Node

#these are the message types of the topics the node publishes and subscribes to
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

#functions for robot position
from tf2_ros import Buffer, TransformListener
from scipy.ndimage import binary_dilation
import numpy as np
import heapq

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.map = None          
        self.inflated = None     
        self.robot_pose = None   
        self.goal = None
        self.running = False

        #Sets up robot transform data (position relative to the map)
	    #map_wp uses tf_buffer and is called every 0.1s (10 Hz)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.map_wp)

        #Subscriptions to the /map and /goal pose topics
	    #(MessageType, ‘/topic_name’, self.callback_function, queue)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        #Topic to publish to. This topic is created when the program runs
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

    def map_cb(self, msg: OccupancyGrid):
        #Lock: function doesn’t run unless running = False
	    #this ensures the function runs completely before it can run again 
        if not self.running:
            self.running = True
            self.map = msg
            # Reshape flat data into 2D (rows = height, cols = width)
            grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # Block anything >25
            blocked = (grid > 25)
            #Inflate obstacles by robot radius
            radius_cells=int(np.ceil(0.18 / msg.info.resolution))
            self.inflated = self.inflate(blocked, radius_cells)

            if self.goal is None or self.robot_pose is None:
                self.get_logger().warn('No goal or pose yet')
                self.running = False
                return
            
            #A* called with converted robot pos and goal pos passed
            path_cells = self.astar(self.world_to_grid(self.robot_pose),
                                    self.world_to_grid((self.goal.pose.position.x, self.goal.pose.position.y)))
            if path_cells is None:
                self.get_logger().warn('No path found')
                self.running = False
                return
            
            #pass to publishing function
            self.path_pub.publish(self.cells_to_path_msg(path_cells))
            #unlock
            self.running = False

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def map_wp(self):
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
        #(circle of r == radius_cells) = True
        kernel = (x*x + y*y) <= radius_cells*radius_cells
        return binary_dilation(blocked, structure=kernel)

    def world_to_grid(self, xy):
        """(x, y) in meters -> (col, row) integer cell indices."""
        x, y = xy
        info = self.map.info
        #distance from origin divided by resolution
	    #resolution = 5 cm / grid square
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
            ps.pose.orientation.w = 1.0   
            path.poses.append(ps)
        return path

    def astar(self, start, goal):
        self.get_logger().warn(f"{start}")
        self.get_logger().warn(f"{goal}")
        h = lambda a, b: np.hypot(a[0]-b[0], a[1]-b[1])  
        open_set = [(h(start, goal), 0, start, None)]    # (f, g, cell, parent_idx)
        came_from = {}
        g_score = {start: 0}
        neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)] #8 directions to look around

        while open_set:
            f, g, cur, parent = heapq.heappop(open_set) #extract square
            if cur in came_from: continue
            came_from[cur] = parent
            if cur == goal:
                #path constructed here once goal is reached
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                return path[::-1]
            for dc, dr in neighbors: #check surroundings
                nb = (cur[0]+dc, cur[1]+dr)
                if not (0 <= nb[0] < self.map.info.width and 0 <= nb[1] < self.map.info.height): #within map check
                    continue
                if self.inflated[nb[1], nb[0]]: #obstacle or not check  
                    continue
                tentative = g + np.hypot(dc, dr)
                if tentative < g_score.get(nb, float('inf')): #reduce down to neighbors with the least cost
                    g_score[nb] = tentative
                    heapq.heappush(open_set, (tentative + h(nb, goal), tentative, nb, cur)) #add to open set
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()