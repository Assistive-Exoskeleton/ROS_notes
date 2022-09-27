# ROS and docker containers 

## Docker basics

At any particular instance, a Docker container can be found in 6 possible states:

* **Created**: Docker assigns the created state to the containers that were never started ever since they were created. Hence, no CPU or memory is used by the containers in this state.
* **Running**: When we start a container having created a state using the docker start command, it attains the running state.
This state signifies that the processes are running in the isolated environment inside the container.
* **Restarting**: this state denotes that the container is under the process of restart. 
* **Exited** : This state is achieved when the process inside the container terminates. In this state, no CPU and memory are consumed by the container. A container in the exited state cannot be accessed using the docker exec command (execute a command in a container). However, we can start the container using the docker start or docker restart and then access it.
* **Paused**: Paused is the state of a Docker container that suspends all the processes for an indefinite time. A paused container consumes the same memory used while running the container, but the CPU is released completely. 
A Docker container can be paused using the docker pause command.
* **Dead**: The dead state of a Docker container means that the container is non-functioning. This state is achieved when we try to remove the container, but it cannot be removed because some resources are still in use by an external process. Containers in a dead state cannot be restarted. They can only be removed.

Initial commands are Create - Start - Run.

`Create` adds a writeable container on top of your image and sets it up for running whatever command you specified in your CMD. The container ID is reported back but it’s not started.

`Start` will start any stopped containers. This includes freshly created containers.

`Run` is a combination of create and start. It creates the container and starts it.


The syntax of the docker run command is as follows:
 
    $ docker run [OPTIONS] IMAGE[:TAG|@DIGEST] [COMMAND] [ARG...]


To list the running containers, simply execute the docker ps command:

    $ docker ps

To show all the containers (even containers not running), the command is:

    $ docker ps -a

To remove the container from the host, you can use the docker rm command. The syntax is as follows:

    $ docker rm [OPTIONS] CONTAINER [CONTAINER...]


Option `-it` allow to use an interactive shell. 

To run docker commands without sudo privileges, run:
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```
Then reboot. 

GUIs are shown only if you add 

    xhost +local:docker 

in the `.bashrc` file.  


## How to create a ROS2 container with moveit image

Commands to create a container with moveit2 and ros2 packages.  

``` bash
docker pull ubuntu:jammy

docker run -it -e "DISPLAY=$DISPLAY" -v "$HOME/.Xauthority:/root/.Xauthority:ro" --name MY_PROJECT --network host --privileged moveit/moveit2:humble-release
```
exit from container and copy your ros2 workspace with 

    docker cp FOLDER_TO_COPY MY_PROJECT:NEW_FOLDER_NAME

Restart your container with
    
    docker start MY_PROJECT -i

then move into project directory and 

```
sudo apt update && sudo apt upgrade

rosdep install --from-paths src -y --ignore-src

colcon build --symlink-install
```

To source your project at container start, in the running container type:

```bash
sudo apt install nano
cd /
nano ros_entrypoint.sh
```

paste "source /MY_PROJECT/install/setup.sh" before "exec '$@'". From now on, at each startup, container will automatically source ROS2 and your compiled project.  



## ROS2 container - developer mode

Follow this steps to create a container with mounted folder of host system. This allows to edit the project on the host machine and execute it in the container. **Useful when project is under developement**.  
Follow each step of standard container creation, but with this variation:
```bash
docker run -it -e "DISPLAY=$DISPLAY" -v "$HOME/.Xauthority:/root/.Xauthority:ro" --mount src="$(pwd)",target=/MY_DISK,type=bind --name MY_PROJECT --network host --privileged moveit/moveit2:humble-release
```

When starting container, ensure to cd into mounted folder.

Remember also to rebuilt the project everytime a file is modified and source the install/setup.sh file.  


## Docker images management
To create a new image from an existing container:

    docker commit MY_PROJECT REPOSITORY:TAG

If you want to export the image in a file:

    docker save IMAGE > IMAGE.tar

To load the image in another computer running docker:
    
    docker load < IMAGE.tar




