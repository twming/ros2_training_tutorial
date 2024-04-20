# ROS2 Training Tutorials v1.0

This is the activities and instructions to start.

### Activity 2.1 Workspace & Package
* Clone the repository http://github.com/twming/ros2_training_tutorial.
* Move "my_node" folder to ~/dev_ws/src
* Inspect the CMakeList.txt and package.xml files, check the package name and project name is “my_node”

### Activity 2.2 Colcon build
* Run colcon build on “my_node"
* Debug the build, if any error
  
### Activity 2.3 Topic Publisher
* Create “publisher.py” in “scripts”
* Define “MinimalPublisher” class and String “topic“
* Update new line in CMakeList.txt 
       “scripts/publisher.py” 
* Colcon build, source setup.bash and run.
* Use ros2 topic echo /topic to check the message.


Create publisher.py
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

Create subscriber.py
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
report_coordinate.py
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
add_two_int_server.py
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

add_two_int_client.py
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

fibonacci_server.py
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
fibonacci_client.py
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

turtlesim_circle_launch.py
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

Custom msg/srv/action
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
