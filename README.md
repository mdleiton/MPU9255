ROS MPU9255 Node
================

Installation
------------

First install WiringPI:
	
	cd
	git clone git://git.drogon.net/wiringPi
	cd ~/wiringPi
	./build

Then clone this repository into your ROS workspace(src folder):

    git clone https://github.com/mdleiton/MPU9255.git
    
compile it:

    catkin_make

Review the published data with:

    rostopic echo /imu/data_raw

