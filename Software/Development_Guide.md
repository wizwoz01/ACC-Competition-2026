# 🛠️ Development Guide - ACC 2026

This guide outlines the development workflow for the Beach Autonomous Systems team.

## Table of Contents

1. [Environment Setup](#environment-setup)
2. [Development Workflow](#development-workflow)
3. [ROS 2 Development](#ros-2-development)
4. [Testing in QLabs](#testing-in-qlabs)
5. [Best Practices](#best-practices)

---

## Environment Setup

### Step 1: Clone Required Repositories

```bash
# Clone the official ROS technical resources
git clone https://github.com/quanser/student-competition-resources-ros.git

# Clone our team repository (if not already)
git clone <our-team-repo-url>
```

### Step 2: Start Docker Environment

```bash
cd student-competition-resources-ros

# Start the Docker container
docker compose up -d

# Enter the container
docker compose exec ros2_dev bash
```

### Step 3: Verify Installation

```bash
# Check ROS 2 is working
ros2 --version

# List available packages
ros2 pkg list

# Check QLabs connection (if applicable)
# Commands depend on QLabs setup
```

---

## Development Workflow

### Daily Workflow

1. **Start Environment**
   ```bash
   docker compose up -d
   docker compose exec ros2_dev bash
   ```

2. **Pull Latest Changes**
   ```bash
   git pull origin main
   ```

3. **Build Workspace**
   ```bash
   cd /workspace
   colcon build
   source install/setup.bash
   ```

4. **Develop and Test**
   - Make code changes
   - Build incrementally: `colcon build --packages-select <pkg>`
   - Test in QLabs

5. **Commit Changes**
   ```bash
   git add .
   git commit -m "Description of changes"
   git push origin <branch>
   ```

### Branch Strategy

```
main          # Stable, tested code
├── develop   # Integration branch
├── feature/* # New features
├── fix/*     # Bug fixes
└── test/*    # Experimental code
```

---

## ROS 2 Development

### Creating a New Package

```bash
# Python package
ros2 pkg create --build-type ament_python <package_name>

# C++ package
ros2 pkg create --build-type ament_cmake <package_name>
```

### Package Structure

```
my_package/
├── package.xml          # Package metadata
├── setup.py             # Python setup (ament_python)
├── my_package/          # Python source
│   ├── __init__.py
│   └── my_node.py
├── launch/              # Launch files
│   └── my_launch.py
├── config/              # Configuration files
│   └── params.yaml
└── resource/            # Package marker
    └── my_package
```

### Common ROS 2 Patterns

**Publisher Node:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Subscriber Node:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = MySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## Testing in QLabs

### Launching QLabs

Follow the instructions in the [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros) for launching the QLabs simulation environment.

### Testing Checklist

- [ ] Vehicle spawns correctly
- [ ] Sensors produce data
- [ ] Control commands work
- [ ] No major latency issues

### Debugging Tools

```bash
# View all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# View topic info
ros2 topic info /topic_name

# Check node status
ros2 node list
ros2 node info /node_name

# View TF tree
ros2 run tf2_tools view_frames

# RViz visualization
ros2 run rviz2 rviz2
```

### Recording Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /lidar/points

# Play back recording
ros2 bag play <bag_directory>
```

---

## Best Practices

### Code Quality

1. **Use Type Hints**
   ```python
   def process_image(image: np.ndarray) -> np.ndarray:
       ...
   ```

2. **Document Functions**
   ```python
   def detect_objects(frame):
       """
       Detect objects in camera frame.
       
       Args:
           frame: RGB image as numpy array
       
       Returns:
           List of detected objects with bounding boxes
       """
       ...
   ```

3. **Handle Errors**
   ```python
   try:
       result = risky_operation()
   except SpecificError as e:
       self.get_logger().error(f'Operation failed: {e}')
   ```

### Git Practices

1. **Meaningful Commits**
   ```
   feat: Add traffic light detection
   fix: Correct steering angle calculation
   docs: Update README with setup instructions
   refactor: Simplify path planning logic
   ```

2. **Pull Requests**
   - Create PRs for all changes
   - Request review from teammates
   - Test before merging

### Performance

1. **Profile Your Code**
   ```python
   import cProfile
   cProfile.run('my_function()')
   ```

2. **Optimize Hot Paths**
   - Use numpy for array operations
   - Avoid unnecessary copies
   - Consider threading for I/O

---

## Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Python Style Guide](https://peps.python.org/pep-0008/)
- [Git Best Practices](https://www.atlassian.com/git/tutorials)

---

*Beach Autonomous Systems - CSULB*

