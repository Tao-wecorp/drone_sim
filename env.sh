sudo apt-get install python-pip 
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-joint-state-publisher-gui ros-melodic-teleop-twist-keyboard -y

pip3 install imageio==2.6.1 tensorflow-gpu==1.14.0 keras==2.3.1 pyglet==1.3.0 keras-rl==0.4.2

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib" >> ~/.bashrc
source ~/.bashrc

catkin_make -DPYTHON_EXECUTABLE:FILEPATH=~/.virtualenvs/py3venv/bin/python