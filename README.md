## How to run
1. Open terminator with LRS layout. 
2. In 1st terminal launch gazebo: 
```
gazebo <path_to_world>/fei_lrs_gazebo.world
```
3. In 2nd terminal launch ArduPilot SITL: 
```
cd ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris --console -l 48.15084570555732,17.072729745416016,150,0
```
4. In 3rd termina launch mavros:
```
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555
```
5. In 4th terminal go to workspace and run this run main program:
```
colcon build --packages-select lrs
source install/local_setup.bash
ros2 run lrs dron
```

### How to run following tasks from terminal
1. Stop dron movement:
```
ros2 service call /stop_dron tutorial_interfaces/srv/StopDron "{}"
```
2. Resume dron movement:
```
ros2 service call /resume_dron tutorial_interfaces/srv/ResumeDron "{}"
```   
3. Make circle:
```
ros2 service call /make_circle tutorial_interfaces/srv/MakeCircle "{radius: 1.0, distance: 0.5}"
```  

