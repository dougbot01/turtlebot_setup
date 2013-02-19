This ROS package takes output from an Arduino that is set up with a rosserial program

It accepts LaptopChargeStatus messages on the /turtlebot_lipo topic, reformats them, and publishes them to the /diagnostics topic as a DiagnosticArray

Using this method, an Arduino can read voltage from a LiPo battery powering a mini-PC and allow the battery status to appear in turtlebot_dashboard

