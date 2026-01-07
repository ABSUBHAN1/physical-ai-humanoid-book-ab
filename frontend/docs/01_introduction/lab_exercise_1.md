---
title: Lab Exercise 1 - Physical AI Simulation
description: Hands-on lab to implement a basic Physical AI simulation
---

# Lab Exercise 1: Physical AI Simulation

## Objective
Implement a simple simulation that demonstrates the principles of Physical AI by creating an embodied agent that learns to navigate a physical environment using sensorimotor interactions.

## Prerequisites
- Basic Python programming knowledge
- Understanding of ROS concepts from Chapter 3
- Familiarity with simulation environments

## Learning Outcomes
By completing this lab, you will:
1. Implement a basic embodied agent in a simulated environment
2. Understand the sensorimotor loop in Physical AI systems
3. Compare embodied vs. abstract approaches to problem-solving
4. Experience the challenges of real-time physical interaction

## Environment Setup
1. Install Gazebo simulation environment
2. Set up ROS 2 (Humble Hawksbill or later)
3. Create a new ROS package for this lab

```bash
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws/src
ros2 pkg create --build-type ament_python physical_ai_lab1
cd physical_ai_lab1
```

## Implementation Steps

### Step 1: Create the Environment
Create a simple maze environment in Gazebo with obstacles and a goal location.

### Step 2: Design the Agent
Create a differential drive robot with:
- Laser range finder for obstacle detection
- Basic movement capabilities (forward, turn)
- Goal detection sensor

### Step 3: Implement the Control Algorithm
Implement a simple learning algorithm that allows the agent to navigate to the goal:
- Use sensorimotor contingencies to guide behavior
- Implement basic memory of successful actions
- Include random exploration to discover new strategies

### Step 4: Compare with Abstract Approach
Implement a traditional pathfinding algorithm (e.g., A*) to solve the same problem and compare performance.

## Code Template
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import random

class PhysicalAIAgent(Node):
    def __init__(self):
        super().__init__('physical_ai_agent')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Agent state
        self.laser_data = None
        self.position = None
        self.orientation = None
        self.goal_position = (5.0, 5.0)  # Example goal position
        
        # Learning parameters
        self.q_table = {}  # Simple Q-learning table
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.exploration_rate = 0.3
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def scan_callback(self, msg):
        self.laser_data = msg.ranges
    
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Extract orientation if needed
        
    def get_state(self):
        # Discretize sensor data to create a state representation
        if self.laser_data is None:
            return "unknown"
        
        front_clear = min(self.laser_data[330:30] + self.laser_data[330:]) < 1.0
        left_clear = min(self.laser_data[210:330]) < 1.0
        right_clear = min(self.laser_data[30:150]) < 1.0
        
        # Simple discretization of position relative to goal
        dx = self.goal_position[0] - self.position[0] if self.position else 0
        dy = self.goal_position[1] - self.position[1] if self.position else 0
        dist_to_goal = np.sqrt(dx**2 + dy**2)
        
        # Create a state string
        state = f"front_{front_clear}_left_{left_clear}_right_{right_clear}_dist_{int(dist_to_goal)}"
        return state
    
    def get_reward(self):
        # Calculate reward based on proximity to goal and obstacle avoidance
        if self.position is None:
            return 0
        
        dx = self.goal_position[0] - self.position[0]
        dy = self.goal_position[1] - self.position[1]
        dist_to_goal = np.sqrt(dx**2 + dy**2)
        
        # Positive reward for getting closer to goal
        reward = 1.0 / (dist_to_goal + 0.1)
        
        # Negative reward for being near obstacles
        if self.laser_data:
            min_dist = min(self.laser_data) if self.laser_data else float('inf')
            if min_dist < 0.5:
                reward -= 5.0  # Penalty for being too close to obstacles
        
        # Positive reward for reaching goal
        if dist_to_goal < 0.5:
            reward += 100.0
        
        return reward
    
    def choose_action(self):
        state = self.get_state()
        
        # Initialize Q-values for this state if not already present
        if state not in self.q_table:
            self.q_table[state] = {"forward": 0.0, "left": 0.0, "right": 0.0, "backward": 0.0}
        
        # Epsilon-greedy action selection
        if random.random() < self.exploration_rate:
            # Explore: choose random action
            return random.choice(["forward", "left", "right", "backward"])
        else:
            # Exploit: choose action with highest Q-value
            return max(self.q_table[state], key=self.q_table[state].get)
    
    def update_q_value(self, state, action, reward, next_state):
        if next_state not in self.q_table:
            self.q_table[next_state] = {"forward": 0.0, "left": 0.0, "right": 0.0, "backward": 0.0}
        
        current_q = self.q_table[state][action]
        max_next_q = max(self.q_table[next_state].values())
        
        # Q-learning update rule
        new_q = current_q + self.learning_rate * (reward + self.discount_factor * max_next_q - current_q)
        self.q_table[state][action] = new_q
    
    def control_loop(self):
        if self.laser_data is None or self.position is None:
            return
        
        # Get current state and reward
        current_state = self.get_state()
        reward = self.get_reward()
        
        # Choose action
        action = self.choose_action()
        
        # Execute action
        cmd = Twist()
        if action == "forward":
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        elif action == "left":
            cmd.linear.x = 0.2
            cmd.angular.z = 0.5
        elif action == "right":
            cmd.linear.x = 0.2
            cmd.angular.z = -0.5
        elif action == "backward":
            cmd.linear.x = -0.3
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        
        # Get next state and update Q-value
        next_state = self.get_state()
        if current_state in self.q_table:
            self.update_q_value(current_state, action, reward, next_state)

def main(args=None):
    rclpy.init(args=args)
    agent = PhysicalAIAgent()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Expected Results
- The embodied agent should learn to navigate to the goal while avoiding obstacles
- Compare the learning curve with the traditional pathfinding approach
- Observe how the agent adapts to changes in the environment

## Troubleshooting
- If the agent doesn't learn, try adjusting the learning parameters
- Ensure the simulation environment is properly configured
- Check that sensor data is being received correctly

## Extension Ideas
1. Add more complex environments with dynamic obstacles
2. Implement more sophisticated learning algorithms
3. Compare with traditional pathfinding algorithms
4. Add multiple agents to explore social aspects of Physical AI

## Submission Requirements
- Submit your complete ROS package
- Include a report comparing the embodied approach with the abstract approach
- Document any challenges encountered and how you addressed them