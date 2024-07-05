### ROS2 Container Installation in Docker Desktop
Go to Window PowerShell, enter below command. The installation will check your local repository, then go to twming/ros-humble-training Docker Hub to download, if not available. The installation may take 20 mins.
```
docker run --name "ros-humble-training" -it twming/ros-humble-training
```
To exit the container.
```
exit
```
To stop the container.
```
docker stop ros-humble-training
```
To start the container.
```
docker start ros-humble-training
```
To login the container.
```
docker exec -it ros-humble-training /bin/bash
```
To setup the ROS environment in docker
```
source bashrc
```

# ROS2 Training Tutorials v2.0
This is the activities and instructions to start.

### Activity 2.1: Workspace & Package
* Go to the dev_ws/src workspace
```
cd ~/dev_ws/src
```
* Create "my_node" python package
```
ros2 pkg create my_node  --build-type ament_python
```

### Activity 2.2: Colcon build
* Go to the dev_ws folder
```
cd ~/dev_ws
```
* Build the "my_node" package
```
colcon build
```
* Debug the build, if any error
  
### Activity 2.3: Topic Publisher
* Create “publisher.py” in my_node/my_node folder
* Define “MinimalPublisher” class and String “topic“
* Update new entry line in setup.py
```
"publisher=my_node.publisher:main",
```
* Colcon build, source setup.bash and run
* Use ros2 topic echo /topic to check the message.
```
ros2 run my_node publisher
```
> [!TIP]
> publisher.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.4: Topic Subscriber
* Create “subscriber.py” in my_node/my_node folder
* Define “MinimalSubscriber” class and String “topic“
* Update new entry line in setup.py
```
"subscriber=my_node.subscriber:main",
```
* Colcon build, source setup.bash and run
```
ros2 run my_node subscriber
```
* Run and test the output together with publisher
> [!TIP]
> subscriber.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.5: draw_circle.py
* Create “draw_circle.py” in my_node
* Complete the code.
* Update new entry line in setup.py
```
"draw_circle=my_node.draw_circle:main",
```
* Colcon build, source setup.bash and run
* Run and observe movement in turtlesim
> [!TIP]
> Node and Topic Information
```
node name: “DrawCircleNode”
topic name: “/turtle1/cmd_vel”
linear.x : 2.0
angular.z: 1.0
```
> [!TIP]
> draw_circle.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('__________')
        self.publisher_ = self.create_publisher(Twist, '__________', 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x=__________
        msg.angular.z=__________
        self.publisher_.publish(msg)
        self.get_logger().info("Twist %f %f" % (msg.linear.x,msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    drawnode = DrawCircleNode()
    rclpy.spin(drawnode)
    drawnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.6: pose_subscriber.py
* Create a new python file “pose_subscriber.py”
* Complete the code.
* Update new entry line in setup.py
```
"pose_subscriber=my_node.pose_subscriber:main",
```
* Colcon build, source setup.bash.
* Run and observe movement in turtlesim
> [!TIP]
> pose_subscriber.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('PoseSubscriberNode')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_call_back,
            10)
        self.subscription  # prevent unused variable warning

    def pose_call_back(self, pose:Pose):
        self.get_logger().info("Pose: ("+str(pose.x)+","+str(pose.y)+")")

def main(args=None):
    rclpy.init(args=args)
    subnode = PoseSubscriberNode()
    rclpy.spin(subnode)
    subnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.7: turtle_controller.py
* Create a new python file “turtle_controller.py”
* Complete the code.
* Update new entry line in setup.py
```
"turtle_controller=my_node.turtle_controller:main",
```
* Colcon build, source setup.bash.
* Run and observe movement in turtlesim
> [!TIP]
> turtle_controller.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('TurtleControllerNode')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose,'/turtle1/pose',self.pose_call_back,10)
        self.subscription  # prevent unused variable warning

    def pose_call_back(self, pose:Pose):
        cmd = Twist()
        if pose.x>9 or pose.x<2 or pose.y<2 or pose.y>9:
            cmd.linear.x=1.0
            cmd.angular.z=0.9
        else:
            cmd.linear.x=5.0
            cmd.angular.z=0.0
        self.publisher_.publish(cmd)
        self.get_logger().info("Twist %f %f" % (cmd.linear.x,cmd.angular.z))

def main(args=None):
    rclpy.init(args=args)
    turtlecontrollernode = TurtleControllerNode()
    rclpy.spin(turtlecontrollernode)
    turtlecontrollernode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.8: Create Launch File
* Create a folder “launch” in “my_node”
* Create “turtlesim_follow_launch.py” in "launch" folder
```
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])

```
* Add below line to setup.py header
```
import os
from glob import glob
```
* Add below line to setup.py data_files arryay
```
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
```
* Colcon build, source setup.bash.
* Run together with tele_op to observe movement in turtlesim

### Activity 2.9: Create Interface Package
* Create “my_interface” package using ROS command:
```
cd ~/dev_ws/src/
ros2 pkg create my_interface --build-type ament_cmake
```
* Add below line to package.xml
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
* Add below line to CMakeLists.txt
```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}

)
ament_export_dependencies(rosidl_default_runtime)
```
### Activity 2.10 Create TargetCoordinates.msg
* In the “my_interface” package, create “msg” folder.
* Create TargetCoordinates.msg, with below data type:
```
string id
int32 x
int32 y
```
* Update CMakeList.txt and package.xml
* Build the “my_interface” package
* Check the interface message using ros2 command.

### Activity 2.11: report_coordinate.py with TargetCoordinates
* Duplicate “publisher.py” to “report_coordinate.py”
* Modify the topic type to TargetCoordinates, rename to “coordinate”, publish below data:
> [!NOTE]
> Message Information
```
id: Peter
x: 45
y: 60
```
* Update new entry line in setup.py
```
"report_coordinate=my_node.report_coordinate:main", 
```
* Build and Run the report_coordinate.py 
* Echo the /coordinate topic
```
ros2 topic echo /coordinate
```

> [!TIP]
> report_coordinate.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interface.msg import TargetCoordinates

class Report_Coordinate(Node):
    def __init__(self):
        super().__init__('report_coordinate')
        self.publisher_ = self.create_publisher(TargetCoordinates, 'coordinate', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TargetCoordinates() 
        msg.id = "Peter"
        msg.x = 32
        msg.y = 48
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s : %i , %i"' % (msg.id,msg.x,msg.y) )

def main(args=None):
    rclpy.init(args=args)
    report_node = Report_Coordinate()
    rclpy.spin(report_node)
    report_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.12: Create AddTwoInts.srv
* In the “my_interface” package, create “srv” folder.
* Create AddTwoInts.srv, with request and response.
> [!NOTE]
> Service Information
```
int32 x
int32 y
---
int32 sum
```
* Update CMakeList.txt and package.xml
* Build the “my_interface” package
* Check the interface message using ros2 command.

### Activity 2.13: AddTwoInts Service Client and Serve
* Create “add_two_int_client.py” and “add_two_int_server.py” using above codes and AddTwoInts.srv
* Update new entry line in setup.py
```
"add_two_int_server=my_node.add_two_int_server:main",
"add_two_int_client=my_node.add_two_int_client:main",
```
* Build the “my_node” package
* Run each code in one terminal.
```
ros2 run my_node add_two_int_server.py
ros2 run my_node add_two_int_client.py 54 32
```
> [!TIP]
> add_two_int_server.py
```
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from my_interface.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.x + request.y
        self.get_logger().info('Incoming request\na: %i b: %i' % (request.x, request.y))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
> [!TIP]
> add_two_int_client.py
```
#!/usr/bin/env python3
import sys
from my_interface.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.x = a
        self.req.y = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 2.14: Create Fibonacci.action
* In the “my_interface” package, create “action” folder.
* Create Fibonacci.action, with goal, result and feedback.

> [!NOTE]
> Action Information
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
* Update CMakeList.txt and package.xml
* Build the “my_interface” package
* Check the interface message using ros2 command.

### Activity 2.15: Fibonacci Client and Server
* Create “fibonacci_client.py” and “fibonacci_server.py” using above codes and Fibonacci.action
* Update new entry line in setup.py
```
"fibonacci_server=my_node.fibonacci_server:main",
"fibonacci_client=my_node.fibonacci_client:main",
```
* Build the “my_node” package
* Run each code in one terminal.
```
ros2 run my_node fibonacci_server.py
ros2 run my_node fibonacci_client.py 12
```
> [!TIP]
> fibonacci_server.py
```
#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_interface.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

if __name__ == '__main__':
    main()
```
> [!TIP]
> fibonacci_client.py
```
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_interface.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```
### Optional: Launch File
* Create a launch file named "turtlesim_circle_launch.py" in launch folder
* Launch the turtlesim_node and turtle_move.py
> [!TIP]
> turtlesim_circle_launch.py
```
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='turtle_node'
            ),
            Node(
                package='my_pubsub',
                executable='turtle_move.py',
                name='turtle_circle'
            )
        ]
    )
```
### LiDAR, SLAM and Navigation
### Activity 3.1: Controlling Turtlebot3 Burger using Teleop Keyboard
Choose one to the world below, launch the Burger into the world and teleop it around the world
* Terminal 1: Launch Turtle World into Gazebo
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: Teleop Keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Use W/X/A/D to move forward, backward, left and right. S to stop the robot.

### Activity 3.2: LiDAR Scan Data in RViz
Explore the LiDAR data using RViz
* Terminal 1: Launch Turtle World into Gazebo
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: Launch RViz
```
ros2 run rviz2 rviz2
```
### Activity 3.3: Exploring the world using LiDAR and SLAM
Start the Turtle World, activate the SLAM node to collect the environment data using LiDAR, save the map to local drive
* Terminal 1: Turtle World
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: SLAM
```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
* Terminal 3: Teleop Keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```
* Terminal 4: Save Map
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
### Activity 3.4: Navigate using SLAM map
Start the Turtle World, load the map and navigation node, initialize the pose and navigate within the map.
* Terminal 1: Tu4tle World
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: Load the map
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=$HOME/map.yaml
```
* Terminal 3: Load RViz2, initialize pose and navigate
```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz 
```
### Activity 3.5: Turtlebot3 Control with Python
* Pull a new package (demo1) to your workspace
```
cd ~/dev_ws/src
git clone https://github.com/twming/ros2_training_tutorial
```
* Update new entry line in setup.py
```
"initial_pose=demo1.initial_pose:main",
"trajectory=demo1.trajectory:main",
"laser_data=demo1.laser_data:main",
"avoid_obstacle=demo1.avoid_obstacle:main",
"path_planning=demo1.path_planning:main",
"autonomous_exploring=demo1.autonomous_exploring:main",
```
* Build the demo1 package
```
cd ~/dev_ws
colcon build
```
* Source the demo1 package
```
source install/setup.bash
```
### Activity 3.5.1: Initialize Pose 
Start the Turtle World, load the map and navigation node, initialize the pose using python.
* Terminal 1: Turtle World
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: Navigation Node
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```
* Terminal 3: Initialize pose
```
ros2 run demo1 initial_pose
```
> [!TIP]
> initial_pose.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.first_pose_received = False

    def odom_callback(self, msg):
        if not self.first_pose_received:
            self.get_logger().info('Setting initial pose based on first received odometry data.')
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = "map"  # Or whatever frame you are using
            initial_pose.pose.pose = msg.pose.pose
            initial_pose.pose.pose.position.x=msg.pose.pose.position.x+2.0
            initial_pose.pose.pose.position.y=msg.pose.pose.position.y+0.5
            self.publisher.publish(initial_pose)
            self.first_pose_received = True
            self.get_logger().info('Initial pose has been set and published.')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 3.5.2: Move Turtlebot 
> [!TIP]
> trajectory.py
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.move = Twist()

    def publish_velocity(self):
        self.move.linear.x = 0.1
        self.move.angular.z = 0.2
        self.publisher.publish(self.move)
        self.get_logger().info(f'Publishing: linear x={self.move.linear.x} angular z={self.move.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 3.5.3: LiDAR data 
> [!TIP]
> laser_data.py
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserDataNode(Node):
    def __init__(self):
        super().__init__('laser_data_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def laser_callback(self, msg):
        print('s1 [0]:', msg.ranges[0])
        print('s2 [90]:', msg.ranges[90])
        print('s3 [180]:', msg.ranges[180])
        print('s4 [270]:', msg.ranges[270])
        print('s5 [359]:', msg.ranges[359])

def main(args=None):
    rclpy.init(args=args)
    laser_data_node = LaserDataNode()
    rclpy.spin(laser_data_node)
    laser_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 3.5.4: Avoid Obstacle 
> [!TIP]
> avoid_obstacle.py
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.move = Twist()

    def scan_callback(self, msg):
        print('s1 [0]:', msg.ranges[0])
        
        if msg.ranges[0] > 0.5:
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
        
        self.publisher.publish(self.move)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 3.5.5: Path Planning 
> [!TIP]
> path_planning.py
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status = GoalStatus.STATUS_EXECUTING
        self.waypoints = [
            [(2.0, 2.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(4.0, 0.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(2.0, -1.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(0.0, 0.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
        ]

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = pose[0][0]
        goal_msg.pose.pose.position.y = pose[0][1]
        goal_msg.pose.pose.position.z = pose[0][2]
        goal_msg.pose.pose.orientation.x = pose[1][0]
        goal_msg.pose.pose.orientation.y = pose[1][1]
        goal_msg.pose.pose.orientation.z = pose[1][2]
        goal_msg.pose.pose.orientation.w = pose[1][3]
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'Going for goal: {goal_msg.pose.pose}')
        
        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.status=GoalStatus.STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    
    # Loop to send goals
    while rclpy.ok():
        for pose in navigation_node.waypoints:
            navigation_node.status=GoalStatus.STATUS_EXECUTING
            navigation_node.send_goal(pose)
            while navigation_node.status != GoalStatus.STATUS_SUCCEEDED:
                rclpy.spin_once(navigation_node)
                
            navigation_node.get_logger().info(f' -------------------------------Start Next Goal -------------------------------')

    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Activity 3.5.6: Autonomous Exploring
> [!TIP]
> autonomous_exploring.py
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ExploringNode(Node):
    def __init__(self):
        super().__init__('exploring_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.front_dist = 0.0
        self.left_dist = 0.0
        self.right_dist = 0.0

    def scan_callback(self, msg):
        # Update distance measurements
        self.front_dist = msg.ranges[0]
        self.left_dist = msg.ranges[45]
        self.right_dist = msg.ranges[315]
        self.drive_logic()

    def drive_logic(self):
        move = Twist()
        if self.front_dist >= 0.5 and self.left_dist >= 0.5 and self.right_dist >= 0.5:
            move.linear.x = 0.3
            move.angular.z = 0.0
            self.get_logger().info('Going Straight')
        elif self.left_dist < 0.5:
            move.linear.x = 0.0
            move.angular.z = -1.0  # 60 degrees to right
            self.get_logger().info('Turning 60 Right')
        elif self.right_dist < 0.5:
            move.linear.x = 0.0
            move.angular.z = 1.0  # 60 degrees to left
            self.get_logger().info('Turning 60 Left')
        else:
            move.linear.x = 0.0
            move.angular.z = 1.57  # 90 degrees to left
            self.get_logger().info('Turning 90 Left')

        self.publisher_.publish(move)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExploringNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

# Reference
### Unix/Linux Commands
> [!TIP]
> Commonly use commands
```
ls              # list the items in the directory
cd my_folder    # change directory to my_folder
cd ..           # change directory to parent folder
cd ~            # change directory to /home/ros folder
mkdir my_folder # create new my_folder directory
rmdir my_folder # remove my_folder directory
rm file         # remove file
sudo apt-get install package_name # install package_name
```
### ROS Humble Installation in Ubuntu

Enable UTF-8 locale support
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
Enable Universe repository in Ubuntu
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add the ROS2 GPG key with apt
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add the repository to Ubuntu source list
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install ROS development tools
```
sudo apt update && sudo apt install ros-dev-tools
```
Install ROS Humble desktop
```
sudo apt install ros-humble-desktop
```

Source the setup file into the environment
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
