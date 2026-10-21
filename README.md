# ROS2 Container Installation in Docker Desktop
Go to Window Command Line/PowerShell, enter below command. The installation will check your local repository, then go to twming/ros-humble-training Docker Hub to download, if not available. The installation may take 20 mins.
```
docker run --name ros-humble-training -p 8080:8080 -v ros_data:/root twming/ros-humble-training:web
```
To stop the container.
```
docker stop ros-humble-training
```
To start the container.
```
docker start ros-humble-training
```

# ROS2 Training Tutorials v2.0
This are the activities and instructions to start.
- [ROS Client Library Python Programming](#ros-client-library-python-programming)
  - [Activity 2.1: Workspace & Package](#activity-21-workspace--package)
  - [Activity 2.2: Colcon build](#activity-22-colcon-build)
  - [Activity 2.3: Topic Publisher](#activity-23-topic-publisher)
  - [Activity 2.4: Topic Subscriber](#activity-24-topic-subscriber)
  - [Activity 2.5: draw_circle.py](#activity-25-draw_circlepy)
  - [Activity 2.6: pose_subscriber.py](#activity-26-pose_subscriberpy)
  - [Activity 2.7: turtle_controller.py](#activity-27-turtle_controllerpy)
  - [Activity 2.8: turtle_controller_with_pen.py](#activity-216-turtle_controller_with_penpy)
- [LiDAR, SLAM and Navigation](#lidar-slam-and-navigation)
  - [Activity 3.1: Controlling Turtlebot3 Burger using Teleop Keyboard](#activity-31-controlling-turtlebot3-burger-using-teleop-keyboard)
  - [Activity 3.2: LiDAR Scan Data in RViz](#activity-32-lidar-scan-data-in-rviz)
  - [Activity 3.3: Exploring the world using LiDAR and SLAM](#activity-33-exploring-the-world-using-lidar-and-slam)
  - [Activity 3.4: Navigate using SLAM map](#activity-34-navigate-using-slam-map)


# ROS Client Library Python Programming

### Activity 2.1: Workspace & Package
* Go to the dev_ws/src workspace
```
cd ~/dev_ws/src
```
* Create "my_node" python package
```
ros2 pkg create my_node --build-type ament_python
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
* Update new entry line in setup.py
```
"publisher=my_node.publisher:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run the publisher node
* Use ros2 topic echo /topic to check the message.
```
ros2 run my_node publisher
```

### Activity 2.4: Topic Subscriber
* Create “subscriber.py” in my_node/my_node folder
* Define “MinimalSubscriber” class and String “topic“
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
* Update new entry line in setup.py
```
"subscriber=my_node.subscriber:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run the subscriber node and test the output together with publisher node
```
ros2 run my_node subscriber
```

### Activity 2.5: draw_circle.py
* Create “draw_circle.py” in my_node
* Complete the code.
> [!TIP]
> Node and Topic Information
```
node name: "DrawCircleNode"
topic name: "/turtle1/cmd_vel"
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
* Update new entry line in setup.py
```
"draw_circle=my_node.draw_circle:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run and observe movement in turtlesim
```
ros2 run my_node draw_circle
```

### Activity 2.6: pose_subscriber.py
* Create a new python file “pose_subscriber.py”
* Complete the code.
> [!TIP]
> Node and Topic Information
```
node name: "PoseSubscriberNode"
topic name: "/turtle1/pose"
```
> [!TIP]
> pose_subscriber.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('__________')
        self.subscription = self.create_subscription(
            Pose,
            '__________',
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
* Update new entry line in setup.py
```
"pose_subscriber=my_node.pose_subscriber:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run and observe movement in turtlesim
```
ros2 run my_node pose_subscriber
```

### Activity 2.7: turtle_controller.py
* Create a new python file “turtle_controller.py”
* Complete the code.
> [!TIP]
> Node and Topic Information
```
node name: "TurtleControllerNode"
publisher topic: "/turtle1/cmd_vel"
subscriber topic: "/turtle1/pose"
turn: linear.x=1.0, angular.z=0.9
foward: linear.x=5.0, angular.z=0.0
```
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
        super().__init__('__________')
        self.publisher_ = self.create_publisher(Twist, '__________', 10)
        self.subscription = self.create_subscription(Pose,'__________',self.pose_call_back,10)
        self.subscription  # prevent unused variable warning

    def pose_call_back(self, pose:Pose):
        cmd = Twist()
        if pose.x>9 or pose.x<2 or pose.y<2 or pose.y>9:
            cmd.linear.x=__________
            cmd.angular.z=__________
        else:
            cmd.linear.x=__________
            cmd.angular.z=__________
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
* Update new entry line in setup.py
```
"turtle_controller=my_node.turtle_controller:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run and observe movement in turtlesim
```
ros2 run my_node turtle_controller
```

### Activity 2.8: turtle_controller_with_pen.py
* Create a new python file “turtle_controller_with_pen.py”
* Complete the code.
> [!TIP]
> Node and Topic Information
```
node name: "TurtleControllerWithPenNode"
publisher topic: "/turtle1/cmd_vel"
subscriber topic: "/turtle1/pose"
Red: 255,0,0,3,0
Green: 0,255,0,3,0
set_pen service: "/turtle1/set_pen"
```
> [!TIP]
> turtle_controller_with_pen.py
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerWithPenNode(Node):
    def __init__(self):
        super().__init__('__________')
        self.previous_x = 0.0
        self.publisher_ = self.create_publisher(Twist, '__________', 10)
        self.subscription = self.create_subscription(Pose,'__________',self.pose_call_back,10)
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
    
        if pose.x>5.5 and self.previous_x<=5.5:
            self.previous_x=pose.x
            self.get_logger().info("Set Pen to Red!")
            self.call_set_pen_service(__________)
        elif pose.x<=5.5 and self.previous_x>5.5:
            self.previous_x=pose.x
            self.get_logger().info("Set Pen to Green!")
            self.call_set_pen_service(__________)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "__________")
        while not client.wait_for_service(1.0):
            self.get_logger().log("Waiting for service ...")

        request=SetPen.Request()
        request.r = r
        request.g = g
        request.b = b 
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    turtlecontrollerwithpennode = TurtleControllerWithPenNode()
    rclpy.spin(turtlecontrollerwithpennode)
    turtlecontrollerwithpennode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
* Update new entry line in setup.py
```
"turtle_controller_with_pen=my_node.turtle_controller_with_pen:main",
```
* Colcon build and source setup.bash
```
cd ~/dev_ws
colcon build
source ~/dev_ws/install/setup.bash
```
* Run and observe movement in turtlesim
```
ros2 run my_node turtle_controller_with_pen
```

# LiDAR, SLAM and Navigation
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
* Terminal 1: Turtle World
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
* Terminal 2: Load the map
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=$HOME/map.yaml
```
> [!TIP]
> turtlebot3_navigation2 for ROS Jazzy
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```
* Terminal 3: Load RViz2, initialize pose and navigate
```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz 
```

