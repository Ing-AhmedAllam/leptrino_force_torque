#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped

class TareWrench:
    def __init__(self):
        self.n = rospy.get_param("~avg_samples", 200)
        self.in_topic  = rospy.get_param("~in",  "/leptrino_force_torque/force_torque")
        self.out_topic = rospy.get_param("~out", "/leptrino_force_torque/force_torque_zeroed")

        self.count = 0
        self.sum_fx = self.sum_fy = self.sum_fz = 0.0
        self.sum_tx = self.sum_ty = self.sum_tz = 0.0
        self.offset_ready = False

        self.pub = rospy.Publisher(self.out_topic, WrenchStamped, queue_size=50)
        rospy.Subscriber(self.in_topic, WrenchStamped, self.cb, queue_size=100)

    def cb(self, msg: WrenchStamped):
        w = msg.wrench

        if not self.offset_ready:
            self.sum_fx += w.force.x;  self.sum_fy += w.force.y;  self.sum_fz += w.force.z
            self.sum_tx += w.torque.x; self.sum_ty += w.torque.y; self.sum_tz += w.torque.z
            self.count += 1
            if self.count >= self.n:
                self.off_fx = self.sum_fx / self.n
                self.off_fy = self.sum_fy / self.n
                self.off_fz = self.sum_fz / self.n
                self.off_tx = self.sum_tx / self.n
                self.off_ty = self.sum_ty / self.n
                self.off_tz = self.sum_tz / self.n
                self.offset_ready = True
                rospy.logwarn("Tare complete. Offsets: F(%.3f, %.3f, %.3f)  T(%.4f, %.4f, %.4f)",
                              self.off_fx, self.off_fy, self.off_fz, self.off_tx, self.off_ty, self.off_tz)
            return

        out = WrenchStamped()
        out.header = msg.header
        out.wrench.force.x  = w.force.x  - self.off_fx
        out.wrench.force.y  = w.force.y  - self.off_fy
        out.wrench.force.z  = w.force.z  - self.off_fz
        out.wrench.torque.x = w.torque.x - self.off_tx
        out.wrench.torque.y = w.torque.y - self.off_ty
        out.wrench.torque.z = w.torque.z - self.off_tz
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("leptrino_tare")
    TareWrench()
    rospy.spin()