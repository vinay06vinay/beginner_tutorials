## ROS2 programming exercises - Publisher/Subscriber
Using ROS2 Humble and Object oriented approach in C++ to achieve multiple functionalities as below
1. Creation of simple publisher and subscriber to print custom message
2. Modify the simple publsiher for creation of a service to perform request and response through a service

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

# Using launch file to run both talker and listener together with custom frequency (Terminal 1)
ros2 launch beginner_tutorials launch.py frequency:=1.0

# Calling the service with modified request message (Terminal 2)
ros2 service call /custom_service beginner_tutorials/srv/CustomService "{request_message: Humble}"

```
### Results
```bash
# Open RQT Console to check different error logs (Terminal 1)
ros2 run rqt_console rqt_console

# Check the log prints for log types: ERROR, INFO, DEBUG (Terminal 2)
ros2 launch beginner_tutorials launch.py frequency:=1.0
```
<p align="center">
<img width="50%" alt="LOG" src="results/ERROR,INFO,DEBUG_LOGS.png">
</p>

```bash
# Press Ctrl+C on Keyboard
# Check the log prints for log types: WARN (Terminal 2)
ros2 launch beginner_tutorials launch.py frequency:=101.0
```
<p align="center">
<img width="50%" alt="WARN LOG" src="results/WARN_LOG.png">
</p>


```bash
# Press Ctrl+C on Keyboard
# Check the log prints for log types: FATAL (Terminal 2)
ros2 launch beginner_tutorials launch.py frequency:=-1.0
```
<p align="center">
<img width="50%" alt="FATAL LOG" src="results/FATAL_LOG.png">
</p>

The Service Request and response are
<p align="center">
<img width="50%" alt="FATAL LOG" src="results/Service_Request_Response.png">
</p>


### CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to directory beginner_tutorials
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> results/cppcheck.txt

# Use the below command for cpp lint by moving to directory beginner_tutorials 
cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint.txt 

## The results of both are present in results folder insider beginner_tutorials directory
```