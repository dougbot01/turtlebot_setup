#!/usr/bin/env python

import roslib; roslib.load_manifest('turtlebot_lipo')
import rospy

from turtlebot_node.msg import LaptopChargeStatus
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

class MsgForwarder(object):
    def __init__(self):
        rospy.init_node('turtlebot_lipo_diag')

        self.sub = rospy.Subscriber('/turtlebot_lipo', LaptopChargeStatus, self.callback)
        self.pub = rospy.Publisher('/diagnostics', DiagnosticArray)

    def laptop_charge_to_diag(self, laptop_msg):
        rv = DiagnosticStatus()
        rv.level   = DiagnosticStatus.OK
        rv.message = 'OK'
        rv.name    = 'Laptop Battery'
        
        if not laptop_msg.present:
            rv.level = DiagnosticStatus.ERROR
            rv.message = 'Laptop battery missing'
            
        rv.values.append(KeyValue('Voltage (V)',          str(laptop_msg.voltage)))
        rv.values.append(KeyValue('Current (A)',          str(laptop_msg.rate)))
        rv.values.append(KeyValue('Charge (Ah)',          str(laptop_msg.charge)))
        rv.values.append(KeyValue('Capacity (Ah)',        str(laptop_msg.capacity)))
        rv.values.append(KeyValue('Design Capacity (Ah)', str(laptop_msg.design_capacity)))

        return rv

    def callback(self, msg):
        diag = DiagnosticArray()
        diag.header.stamp = rospy.get_rostime()
        diag_stat = self.laptop_charge_to_diag(msg)
        diag.status.append(diag_stat)
        self.pub.publish(diag)

if __name__ == '__main__':
    try:
        a = MsgForwarder()
        rospy.spin()
    except rospy.ROSInterruptException: pass


