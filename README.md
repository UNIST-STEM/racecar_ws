# racecar_ws
This is set-up manual for unist racecar.  

## 0. Hardeware set up
   will be provided soon

## 1. Configure jetson nano environment
### 1) Jetpack
  <https://developer.nvidia.com/embedded/jetpack>
### 2) Change Jetson nano power mode to 5W
    sudo nvpmodel -m1

## 2. Install ros on jetson nano
### 1) Install ros-melodic-desktop:
 <http://wiki.ros.org/melodic/Installation/Ubuntu>

### 2) Configrure ros environment:  
  <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>

### 3) Add below scripts to the bash:
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


## 3. Install required ros packages:
### 1) rosserial
    sudo apt-get install ros-melodic-rosserial-arduino
    sudo apt-get install ros-melodic-rosserial
### 2) Joystick node
    sudo apt-get install ros-melodic-joy


## 4. Get source code in your catkin_ws dir(/home/user/catkin_ws/src/) and build
    cd ~/catkin_ws/src/
    git clone https://github.com/JEONGHA-SHIN/racecar_ws.git
    git clone https://github.com/JEONGHA-SHIN/ydlidar_ros-X2.git
    cd ~/catkin_ws && catkin_make


## 5. Configure serial interface environment
### 1) ydlidar
      roscd ydlidar/startup
      sudo chmod 777 ./*
      sudo sh initenv.sh
### 2) Arduino & joystick: Add following scripts to the bash:
      echo "sudo chmod 666 /dev/ttyACM0" >> ~/.bashrc
      echo "sudo chmod a+rw /dev/input/js0" >> ~/.bashrc
      echo "alias teleop='roslaunch racecar_ws teleop.launch'" >> ~/.bahsrc
      
## 6. Test your car with teleop:
    teleop
      
    
    
    
    
      

      
      
# racecar_ws
