---
title: First Steps with ROS
description: Introduction to the Robot Operating System for Physical AI applications
sidebar_position: 3
---

# First Steps with ROS

This chapter introduces the Robot Operating System (ROS) and its role in developing Physical AI applications for humanoid robotics.

## Chapter Overview
This chapter provides an introduction to ROS (Robot Operating System), the middleware framework that enables communication between different components of robotic systems. We'll explore how ROS facilitates the development of Physical AI systems by providing standardized interfaces for sensors, actuators, and algorithms. Through practical examples, you'll learn the fundamental concepts of ROS including nodes, topics, services, and parameters. By the end of this chapter, you'll have a foundational understanding of how to use ROS to build interconnected robotic systems.

## Why This Matters in Physical AI
ROS is essential for Physical AI because it provides the communication infrastructure that allows diverse components of a robotic system to work together seamlessly. In Physical AI systems, multiple sensors, actuators, and processing units must coordinate in real-time to achieve intelligent behavior. ROS provides standardized message formats, communication protocols, and tools that enable researchers and developers to focus on the intelligence aspects rather than low-level communication details. For humanoid robots, which integrate numerous sensors and actuators, ROS is particularly valuable for managing the complexity of sensorimotor integration.

## Core Concepts
ROS encompasses several fundamental concepts:

1. **Nodes**: Processes that perform computation. Nodes are the fundamental building blocks of ROS applications.

2. **Topics**: Named buses over which nodes exchange messages. Topics implement a publish/subscribe communication model.

3. **Messages**: Data structures that are passed between nodes. Messages define the format of data exchanged via topics.

4. **Services**: A request/reply communication model that allows nodes to send requests and receive responses.

5. **Parameters**: A centralized key-value parameter server that allows nodes to store and share configuration data.

6. **Launch Files**: XML files that allow you to start multiple nodes with a single command.

7. **TF (Transforms)**: A system for tracking coordinate frames and their relationships over time.

These concepts form the foundation for building distributed robotic systems.

## System Architecture or Mental Model
A typical ROS system architecture includes:

- **Master**: Coordinates communication between nodes, maintains naming registry
- **Nodes**: Individual processes that perform specific functions (sensor drivers, controllers, algorithms)
- **Parameter Server**: Centralized storage for configuration parameters
- **Communication Layer**: Implements publish/subscribe and service call communication patterns
- **Client Libraries**: APIs (roscpp, rospy) that allow nodes to communicate using ROS concepts

This architecture enables modular development where different components can be developed and tested independently.

## Hands-On Lab/Simulation
In this lab, you'll create a simple ROS system that demonstrates the publish/subscribe pattern.

### Objective
Create a ROS system with a publisher node that generates sensor data and a subscriber node that processes this data.

### Steps
1. Set up a ROS 2 workspace
2. Create a package for the lab
3. Implement a publisher node that publishes sensor data
4. Implement a subscriber node that processes the sensor data
5. Test the system in a simulated environment

### Expected Outcome
You should observe data flowing from the publisher to the subscriber, demonstrating the ROS communication model.

### ROS Package Structure
```
ros_physical_ai_lab/
├── src/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── ros_physical_ai_lab/
│       ├── publisher_node.py
│       └── subscriber_node.py
```

### Troubleshooting Tips
- Ensure your ROS environment is properly sourced
- Check that the topic names match between publisher and subscriber
- Verify that message types are compatible
- Use `ros2 topic list` and `ros2 topic echo` to debug communication

## Code Examples
Here's an example of a simple ROS 2 publisher and subscriber in Python:

```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading: {self.i} at {time.time()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

# subscriber_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process the sensor data
        self.get_logger().info(f'Received sensor data: {msg.data}')
        
        # Example processing: extract numeric value
        try:
            # Extract the number from the message
            parts = msg.data.split(':')
            if len(parts) > 1:
                value = int(parts[1].split()[0])
                # Process the value (e.g., check if it's within expected range)
                if value % 10 == 0:  # Log every 10th reading
                    self.get_logger().info(f'Checkpoint reading: {value}')
        except ValueError:
            self.get_logger().info('Could not parse sensor value')

def main(args=None):
    rclpy.init(args=args)
    data_processor = DataProcessor()
    rclpy.spin(data_processor)
    data_processor.destroy_node()
    rclpy.shutdown()
```

This example demonstrates the basic ROS communication pattern where one node publishes data and another subscribes to it.

## Common Mistakes & Debugging
1. **Topic Mismatch**: Publishers and subscribers must use the same topic name and message type.

2. **Node Lifecycle**: Forgetting to properly initialize or shutdown ROS nodes can cause resource leaks.

3. **Timing Issues**: Not accounting for message delivery delays in real-time systems.

4. **TF Frame Issues**: Incorrectly setting up coordinate frame relationships in the TF tree.

Debugging tip: Use `ros2 topic list`, `ros2 topic echo`, and `ros2 node list` to inspect your ROS system's state and identify communication issues.

## Industry & Research Context
ROS has become the de facto standard for robotics research and development:

- **Academic Research**: Most robotics research labs use ROS for prototyping and experimentation
- **Industrial Robotics**: Companies like Fanuc, ABB, and KUKA provide ROS interfaces
- **Autonomous Vehicles**: Many self-driving car platforms are built on ROS
- **Service Robotics**: Companies developing service robots often use ROS for system integration

ROS 2, the latest version, addresses many of the limitations of ROS 1, including improved security, real-time support, and better multi-robot systems support.

## Review Questions
1. What is the difference between ROS topics and services?
2. How do nodes communicate in a ROS system?
3. What is the role of the ROS Master?
4. Why is ROS particularly valuable for Physical AI systems?
5. What are the advantages of using launch files in ROS?

## Glossary
- **Node**: A process that performs computation in ROS
- **Topic**: A named bus over which nodes exchange messages
- **Message**: The data structure passed between nodes in ROS
- **Service**: A request/reply communication model in ROS
- **Parameter Server**: A centralized key-value store for configuration data
- **Launch File**: An XML file that starts multiple nodes with a single command
- **TF (Transforms)**: A system for tracking coordinate frames and their relationships
- **Master**: The ROS component that coordinates communication between nodes
- **Client Library**: APIs (like roscpp, rospy) that allow nodes to communicate using ROS concepts