"""
Example: Simple Physical AI Agent for Chapter 1

This example demonstrates a basic Physical AI agent that learns to navigate
an environment using sensorimotor interactions.
"""
import numpy as np
import matplotlib.pyplot as plt
import random


class SimplePhysicalAIAgent:
    """
    A simple Physical AI agent that learns to navigate using sensorimotor interactions.
    """
    def __init__(self, environment_size=(10, 10)):
        self.env_size = environment_size
        self.position = [random.randint(0, environment_size[0]-1), 
                         random.randint(0, environment_size[1]-1)]
        self.goal = [environment_size[0]-1, environment_size[1]-1]  # Bottom-right corner
        self.q_table = {}  # State-action value table
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.exploration_rate = 0.3
        
        # Define possible actions: up, down, left, right
        self.actions = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    
    def get_state(self):
        """Get the current state representation."""
        return tuple(self.position)
    
    def get_reward(self):
        """Calculate reward based on current position."""
        if self.position == self.goal:
            return 100  # Large reward for reaching goal
        
        # Distance-based reward (closer to goal = higher reward)
        dist = np.sqrt((self.position[0] - self.goal[0])**2 + 
                       (self.position[1] - self.goal[1])**2)
        return max(0, 10 - dist)  # Higher reward when closer
    
    def choose_action(self):
        """Choose an action using epsilon-greedy policy."""
        state = self.get_state()
        
        # Initialize Q-values for this state if not already present
        if state not in self.q_table:
            self.q_table[state] = [0.0 for _ in self.actions]
        
        # Epsilon-greedy action selection
        if random.random() < self.exploration_rate:
            # Explore: choose random action
            return random.randint(0, len(self.actions)-1)
        else:
            # Exploit: choose action with highest Q-value
            return np.argmax(self.q_table[state])
    
    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value using Q-learning update rule."""
        if next_state not in self.q_table:
            self.q_table[next_state] = [0.0 for _ in self.actions]
        
        current_q = self.q_table[state][action]
        max_next_q = max(self.q_table[next_state])
        
        # Q-learning update rule
        new_q = current_q + self.learning_rate * (reward + 
                      self.discount_factor * max_next_q - current_q)
        self.q_table[state][action] = new_q
    
    def move(self, action_idx):
        """Move the agent according to the chosen action."""
        action = self.actions[action_idx]
        new_x = max(0, min(self.env_size[0]-1, self.position[0] + action[0]))
        new_y = max(0, min(self.env_size[1]-1, self.position[1] + action[1]))
        
        self.position = [new_x, new_y]
    
    def train(self, episodes=1000):
        """Train the agent for a specified number of episodes."""
        rewards_per_episode = []
        
        for episode in range(episodes):
            # Reset to random start position (not the goal)
            while self.position == self.goal:
                self.position = [random.randint(0, self.env_size[0]-1), 
                                random.randint(0, self.env_size[1]-1)]
            
            total_reward = 0
            steps = 0
            max_steps = 100  # Limit steps per episode to prevent infinite loops
            
            while self.position != self.goal and steps < max_steps:
                current_state = self.get_state()
                reward = self.get_reward()
                
                action = self.choose_action()
                self.move(action)
                
                next_state = self.get_state()
                next_reward = self.get_reward()
                
                # Update Q-value
                self.update_q_value(current_state, action, next_reward, next_state)
                
                total_reward += reward
                steps += 1
            
            rewards_per_episode.append(total_reward)
            
            # Reduce exploration rate over time
            if self.exploration_rate > 0.01:
                self.exploration_rate *= 0.995
        
        return rewards_per_episode
    
    def visualize_path(self):
        """Visualize the agent's path to the goal."""
        # Reset position for visualization
        self.position = [0, 0]
        
        path = [self.position[:]]  # Start position
        steps = 0
        max_steps = 100
        
        while self.position != self.goal and steps < max_steps:
            state = self.get_state()
            if state in self.q_table:
                action = np.argmax(self.q_table[state])
            else:
                # If state not in Q-table, move randomly
                action = random.randint(0, len(self.actions)-1)
            
            self.move(action)
            path.append(self.position[:])
            steps += 1
        
        # Create visualization
        grid = np.zeros(self.env_size)
        grid[self.goal[0], self.goal[1]] = 2  # Goal
        
        for pos in path:
            grid[pos[0], pos[1]] = 1  # Path
        
        plt.figure(figsize=(8, 8))
        plt.imshow(grid, cmap='viridis', origin='lower')
        plt.plot([p[1] for p in path], [p[0] for p in path], 'r.-', label='Agent Path')
        plt.plot(self.goal[1], self.goal[0], 'go', markersize=15, label='Goal')
        plt.plot(path[0][1], path[0][0], 'bo', markersize=10, label='Start')
        plt.legend()
        plt.title('Physical AI Agent Path Visualization')
        plt.show()


def main():
    """Main function to demonstrate the Physical AI agent."""
    print("Physical AI Agent Example")
    print("=" * 30)
    
    # Create agent
    agent = SimplePhysicalAIAgent(environment_size=(10, 10))
    
    print(f"Starting position: {agent.position}")
    print(f"Goal position: {agent.goal}")
    
    # Train the agent
    print("\nTraining agent...")
    rewards = agent.train(episodes=1000)
    
    print("Training completed!")
    print(f"Final position after training: {agent.position}")
    
    # Visualize the learned path
    print("\nVisualizing learned path...")
    agent.visualize_path()
    
    # Plot learning curve
    plt.figure(figsize=(10, 5))
    plt.plot(rewards)
    plt.title('Learning Curve: Total Reward per Episode')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()