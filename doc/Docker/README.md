# ROS and docker containers 

Processo per creare il container con la roba di Bridge:

``` bash
sudo groupadd docker
sudo usermod -aG docker $USER

docker run -it -e "DISPLAY=$DISPLAY" -v "$HOME/.Xauthority:/root/.Xauthority:ro" --mount src="$(pwd)",target=/MY_DISK,type=bind  --name Bridge_control --network host osrf/ros:humble-desktop

sudo apt update && sudo apt upgrade

git switch Humble

rosdep install --from-paths src -y --ignore-src

sudo apt install ros-humble-moveit-configs-utils

colcon build --symlink-install --packages-ignore bridge_control_hybrid

source install/setup.sh

ros2 launch bridge_control_bringup bridge_moveit_planning.launch.py
```


%Fase di test
Per avviare il container già creato manualmente:
```bash
docker start Bridge_control && docker exec -it Bridge_control bash
./ros_entrypoint.sh
cd MY_DISK/GitHub/Bridge_ROS2_control/
source install/setup.sh
```


Per avviare il container già creato eseguendo automaticamente il planning (**no rebuild**):
```bash
docker start Bridge_control && docker exec -it Bridge_control bash -c "./ros_entrypoint.sh; cd MY_DISK/GitHub/Bridge_ROS2_control/; ./install/setup.sh; ros2 launch bridge_control_bringup bridge_moveit_planning.launch.py"
```


Per avviare il container già creato eseguendo automaticamente il planning (**rebuild**):
```bash
docker start Bridge_control && docker exec -it Bridge_control bash -c "./ros_entrypoint.sh; cd MY_DISK/GitHub/Bridge_ROS2_control/; colcon build --symlink-install --packages-ignore bridge_control_hybrid; ./install/setup.sh; ros2 launch bridge_control_bringup bridge_moveit_planning.launch.py"
```
