#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Bool, Int32
import rospkg
import copy

class CollectData(object):
    def __init__(self):
        self.pub = rospy.Publisher("/subjectA/response_device", Bool, queue_size=1)
        rospy.Subscriber('leptrino_force_torque/force_torque', WrenchStamped, self.sensor_callback)
        rospy.Subscriber('/subjectA/record_device', String, self.response_callback)
        self.sensor_data = None
        self.save_to_file_flag = False
        self.initial_time = rospy.Time.now()
        self.file = None
        self.filepath = str(rospkg.RosPack().get_path('master_thesis_program')) + "/data/"
        self.prev_seq = -1
        self.lost_signal_stamp = rospy.Time.now()

    def sensor_callback(self, msg):
        self.sensor_data = msg

    def response_callback(self, msg):
        rospy.loginfo("receive response")
        self.file = open(self.filepath + str(msg.data), 'w')
        self.file.write("time[sec],force_x,force_y,force_z,torque_x,torque_y,torque_z\n")
        self.save_to_file_flag = True
        
    def lost_data(self):
        if self.sensor_data.header.seq != self.prev_seq:
            self.lost_signal_stamp = rospy.Time.now()
        elif self.sensor_data.header.seq == self.prev_seq and (rospy.Time.now() - self.lost_signal_stamp).to_sec() > 3:
            return True
        self.prev_seq = copy.copy(self.sensor_data.header.seq)
        return False

    def update(self):
        if self.sensor_data:
            time_sec = self.sensor_data.header.stamp
            force_x = self.sensor_data.wrench.force.x
            force_y = self.sensor_data.wrench.force.y
            force_z = self.sensor_data.wrench.force.z
            torque_x = self.sensor_data.wrench.torque.x
            torque_y = self.sensor_data.wrench.torque.y
            torque_z = self.sensor_data.wrench.torque.z
            if self.save_to_file_flag:
                self.file.write(str((time_sec - self.initial_time).to_sec())+",")
                self.file.write(str(force_x)+",")
                self.file.write(str(force_y)+",")
                self.file.write(str(force_z)+",")
                self.file.write(str(torque_x)+",")
                self.file.write(str(torque_y)+",")
                self.file.write(str(torque_z)+"\n")
                if (rospy.Time.now() - self.initial_time).to_sec() > 10:
                    self.save_to_file_flag = False
                    self.file.close()
                    rospy.loginfo("finished collecting data")
                    self.pub.publish(True)
                elif self.lost_data():
                    self.sensor_data = None
                    self.pub.publish(False)
            else:
                self.initial_time = rospy.Time.now()
        else:
            rospy.loginfo("no sensor data found")
        
if __name__ == '__main__':
    rospy.init_node('collect_data', anonymous=True)
    rate = rospy.Rate(1000) #1.2kHz
    node = CollectData()
    try:
        while not rospy.is_shutdown():
            node.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        node.file.close()
