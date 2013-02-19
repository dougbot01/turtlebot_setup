This Arduino sketch reads the voltage on A0, and will publish results as a ROS message of type LaptopChargeStatus on /turtlebot_lipo.

It uses Vmin as 0% battery and Vmax as 100%. The voltage is read through a voltage divider on LiPos with >5V, and the scale factor is Vmul.

Before compiling or uploading the sketch, you will have to install the turtlebot_lipo package and then run
$ rosrun rosserial_client make_library.py ~/sketchbook/libraries/ turtlebot_lipo
