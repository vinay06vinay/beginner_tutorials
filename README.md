## ROS2 programming exercises - Publisher/Subscriber
Using ROS2 Humble and Object oriented approach in C++, a simple publisher and subscriber is created which prints custom message.

### Building the ROS package
```bash
# Source to ros humble
source /opt/ros/humble/setup.bash
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
git clone https://github.com/vinay06vinay/beginner_tutorials.git

# Once files are checked, go back to the root directory of ros workspace
cd ..
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select beginner_tutorials
# After successfull build source the package
. install/setup.bash
# Run the publisher
ros2 run beginner_tutorials talker
# Run the subscriber
ros2 run beginner_tutorials listener 
```
### Results
```
# Publisher Results
[INFO] [1699153146.000965750] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 0'
[INFO] [1699153146.500920433] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 1'
[INFO] [1699153147.001005949] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 2'
[INFO] [1699153147.500934610] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 3'
[INFO] [1699153148.000974356] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 4'
[INFO] [1699153148.500936091] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 5'
[INFO] [1699153149.000891614] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 6'
[INFO] [1699153149.500927766] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 7'
[INFO] [1699153150.000933799] [minimal_publisher]: Publishing: 'Hello ROS2 Humble! 8'

# Subscriber Results
[INFO] [1699153146.001852585] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 0'
[INFO] [1699153146.501610260] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 1'
[INFO] [1699153147.001789681] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 2'
[INFO] [1699153147.501642832] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 3'
[INFO] [1699153148.001779946] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 4'
[INFO] [1699153148.501715481] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 5'
[INFO] [1699153149.001659035] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 6'
[INFO] [1699153149.501673057] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 7'
[INFO] [1699153150.001658566] [minimal_subscriber]: ROS2 Humble Heard: 'Hello ROS2 Humble! 8'

```

### CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to directory beginner_tutorials
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" ) >  results/cppcheck.txt

# Use the below command for cpp lint by moving to directory beginner_tutorials 
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt

## The results of both are present in results folder insider beginner_tutorials directory
```