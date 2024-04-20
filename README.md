# ROS2 Installation in Ubuntu 22.04

This are the ROS installation steps. 
### ROS2 Iron Installation in Ubuntu
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
Install ROS Iron desktop
```
sudo apt install ros-iron-desktop
```
# Software/Packages Requirements
These are the required software and packages.
```
sudo apt install ros-iron-joint-state-publisher
sudo snap install --classic code
```

# ROS2 Training Tutorials v1.0
This is the activities and instructions to start.

### Activity 2.1: Workspace & Package
* Clone the repository http://github.com/twming/ros2_training_tutorial.
* Move "my_node" folder to ~/dev_ws/src
* Inspect the CMakeList.txt and package.xml files, check the package name and project name is “my_node”

### Activity 2.2: Colcon build
* Run colcon build on “my_node"
* Debug the build, if any error
  
### Activity 2.3: Topic Publisher
* Create “publisher.py” in “scripts”
* Define “MinimalPublisher” class and String “topic“
* Update new line in CMakeList.txt 
> “scripts/publisher.py” 
* Colcon build, source setup.bash and run.
* Use ros2 topic echo /topic to check the message.
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
* Create “subscriber.py” in “scripts”
* Define “MinimalSubscriber” class and String “topic“
* Update CMakeList.txt 
> “scripts/subscriber.py” 
* Colcon build and source setup.bash.
* Run and test the output together with publisher.py
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

### Activity 2.5: turtle_move.py
* Open “turtle_move.py” in “scripts”
* Complete the code
* Update CMakeList.txt 
> “scripts/turtle_move.py” 
* Colcon build, source setup.bash.
* Run and observe movement in turtlesim
> [!TIP]
> Node and Topic Information
```
node name: “turtle_move”
topic name: “/turtle1/cmd_vel”
linear.x : 0.5
angular.z: 0.5
```

### Activity 2.6: Create Launch File
* Create a folder “launch” in “my_node”
* Create “turtlesim_follow_launch.py”
* Update CMakeList.txt 
> “scripts/turtlesim_follow_launch.py” 
* Colcon build, source setup.bash.
* Run and observe movement in turtlesim

### Activity 2.7: Create Interface Package
* Create “my_interface” package using ROS command:
```
cd ~/dev_ws/src/
ros2 pkg create my_interface --build_type ament_cmake
```

### Activity 2.8 Create TargetCoordinates.msg
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

### Activity 2.9: report_coordinate.py with TargetCoordinates
* Duplicate “publisher.py” to “report_coordinate.py”
* Modify the topic type to TargetCoordinates, rename to “coordinate”, publish below data:
> [!NOTE]
> Message Information
```
id: Peter
x: 45
y: 60
```
* Run the report_coordinate.py 
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

### Activity 2.10: Create AddTwoInts.srv
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

### Activity 2.11: AddTwoInts Service Client and Serve
* Create “add_two_int_client.py” and “add_two_int_server.py” using above codes and AddTwoInts.srv
* Update CMakeList.txt and package.xml
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
from my_interface.srv import AddTwoInt

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInt, 'add_two_ints', self.add_two_ints_callback)

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
from my_interface.srv import AddTwoInt
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInt, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInt.Request()

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

### Activity 2.12: Create Fibonacci.action
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

### Activity 2.13: Fibonacci Client and Server
* Create “fibonacci_client.py” and “fibonacci_server.py” using above codes and Fibonacci.action
* Update CMakeList.txt and package.xml
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
### ROS Commands
| ROS2  | ROS |
| ------------- | ------------- |
| ros2 pkg list/executables | rospack list |
| ros2 pkg create --build-type ament_cmake/ament_python --node-name node pkg --dependencies rclcpp std_msgs | catkin_create_pkg pkg rospy std_msgs |
| ros2 node list/info  | rosnode list/info  |
| ros2 topic list/info/echo/pub  | rostopic list/info/echo/pub  |
| ros2 service list/type/call | rosservice list/type/call |
| ros2 action list/info/send_goal | rosaction list/info/send_goal |
| ros2 interface list/show | rosmsg list/show  |
| ros2 param list/get/set | rosparam list/get/set |
| ros2 run pkg node --ros-args -p param1:=value1 -p param2:=value2 | rosrun pkg node param1:=value1 | 
| ros2 run pkg node --ros-args --params-file /file.yaml | rosparam load file.yaml | 
| ros2 run pkg node --ros-args -r node_topic:=topic_to_map | rosrun pkg node node_topic:=topic_to_map| 
| ros2 run rqt_graph rqt_graph | rosrun rqt_graph rqt_graph |
| ros2 run rqt_console rqt_console | rosrun rqt_console rqt_console |
| ros2 run rqt_tf_tree rqt_tf_tree | rosrun rqt_tf_tree rqt_tf_tree |
| colcon build --packages-select pkg | catkin_make -DCATKIN_WHITELIST_PACKAGES="pkg" |

### Custom msg/srv/action
```
TargetCoordinates.msg
string id
int32 x
int32 y

AddTwoInt.srv
int32 x
int32 y
---
int32 sum

Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Thank you! That's all.
