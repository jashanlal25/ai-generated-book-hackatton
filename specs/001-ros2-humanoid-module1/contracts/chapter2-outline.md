# Chapter 2 Outline: Building ROS 2 Nodes with Python

**Priority**: P2 | **Word Target**: 1,400-1,800 | **Code Examples**: 4

---

## Learning Objective

After reading this chapter, students can create functional publisher, subscriber, and service nodes using rclpy that execute without errors on ROS 2 Humble.

## Section Structure

### 2.1 Introduction to rclpy (150-200 words)
- Python client library for ROS 2
- Why Python for humanoid robots (AI/ML integration)
- Required imports and initialization pattern
- `rclpy.init()` and `rclpy.shutdown()`

### 2.2 Creating a Publisher Node (300-400 words)
- Node class structure
- `create_publisher()` method
- Timer-based publishing
- Message creation and population

**Code Example 1**: publisher.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidStatusPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_status_publisher')
        self.publisher_ = self.create_publisher(String, 'humanoid_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Humanoid status update #{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3 Creating a Subscriber Node (250-350 words)
- Subscription callback pattern
- Message handling
- Connecting to publisher

**Code Example 2**: subscriber.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HumanoidStatusSubscriber(Node):
    def __init__(self):
        super().__init__('humanoid_status_subscriber')
        self.subscription = self.create_subscription(
            String,
            'humanoid_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.4 Running Publisher and Subscriber Together (150-200 words)
- Terminal commands
- Observing message flow
- Common troubleshooting

**Diagram**: Humanoid Joint State Message Flow
```
┌─────────────────────┐          ┌─────────────────────┐
│  Terminal 1         │          │  Terminal 2         │
│                     │          │                     │
│  ros2 run ...       │          │  ros2 run ...       │
│  publisher.py       │          │  subscriber.py      │
│                     │          │                     │
│  [INFO] Publishing: │  ───▶    │  [INFO] Received:   │
│  "status update #0" │  topic   │  "status update #0" │
│  "status update #1" │  ───▶    │  "status update #1" │
└─────────────────────┘          └─────────────────────┘
```

### 2.5 Creating a Service Server (250-350 words)
- Service definition
- `create_service()` method
- Request-response handling

**Code Example 3**: service_server.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class JointCalculatorService(Node):
    def __init__(self):
        super().__init__('joint_calculator_service')
        self.srv = self.create_service(
            AddTwoInts,
            'calculate_joint_torque',
            self.calculate_callback)
        self.get_logger().info('Joint calculator service ready')

    def calculate_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointCalculatorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.6 Creating a Service Client (200-300 words)
- `create_client()` method
- Asynchronous call pattern
- Waiting for service availability

**Code Example 4**: service_client.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class JointCalculatorClient(Node):
    def __init__(self):
        super().__init__('joint_calculator_client')
        self.cli = self.create_client(AddTwoInts, 'calculate_joint_torque')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    client = JointCalculatorClient()
    future = client.send_request(10, 5)
    rclpy.spin_until_future_complete(client, future)
    result = future.result()
    client.get_logger().info(f'Result: {result.sum}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.7 Summary and Next Steps (100-150 words)
- Recap of rclpy patterns
- Common debugging tips
- Preview of Chapter 3 (URDF)

---

## Acceptance Criteria

- [ ] Word count: 1,400-1,800
- [ ] 4 complete, runnable code examples
- [ ] All code executes on ROS 2 Humble without modification
- [ ] 1 ASCII diagram included
- [ ] FK readability: grade 11-13
- [ ] No partial snippets or ellipsis

## Dependencies

- `rclpy`
- `std_msgs`
- `example_interfaces`

## Execution Commands

```bash
# Terminal 1: Run publisher
ros2 run <package_name> publisher

# Terminal 2: Run subscriber
ros2 run <package_name> subscriber

# Terminal 3: Run service server
ros2 run <package_name> service_server

# Terminal 4: Run service client
ros2 run <package_name> service_client
```
