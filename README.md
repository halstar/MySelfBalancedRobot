# MySelfBalancedRobot

A homemade self balanced robot built on a Raspberry Pi 3 B+ basis, with different regular sensors (IMU, ultrasonic), and a camera.

This repository gathers sources Python scripts to be executed on the Raspberry Pi.

**Note**: bluetooth may not be installed by default. The following commands might help! 

* sudo apt-get install bluetooth bluez libbluetooth-dev

* pip3 install pybluez

**Note**:  gpiozero & pigpio supports were partially tested. For the latter, the following commands might help!

* sudo pip3 install pigpio

* sudo pigpiod