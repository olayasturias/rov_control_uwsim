#!/usr/bin/env python

# Node that uses transform obtained in tf_transformer.py
# to take a point in the "base_laser" frame and transform
# it to a point in the "base_link" frame.

import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import Imu
import numpy as np
import time




class Sensortf(object):
    """ *Sensortf* class for transforming sensor frame

    This class rotates the velocity frame to a new desired frame.

    **Atributes**:

        .. data:: old_frame

            current frame of the velocity vector

        .. data:: new_frame

            new frame in which the velocity vector will be expresed

        .. data:: input_topic

            Topic where the velocity information is published.

        .. data:: output_topic

            Topic where the transformed velocity is published


    """
    def __init__(self, old_frame, new_frame, input_topic, rate):

        self.old_frame = old_frame
        self.new_frame = new_frame
        self.input_topic = input_topic

        rospy.on_shutdown(self.closenode)

        # Create TransformStamped message
        self.transf = geometry_msgs.msg.TransformStamped()
        # Create rov pose publisher
        self.rov_pose_pub = rospy.Publisher('/BlueRov2/pose',geometry_msgs.msg.Pose,queue_size=10)

        # initialize velocity array and spacing between velocities array
        self.acc_x = []
        self.mean_acc_x = []
        self.vel_x = []
        self.vel_rot_x = []
        self.x = []
        self.x.append(0.0)
        self.rot_x = []
        self.rot_x.append(0.0)

        self.acc_y = []
        self.mean_acc_y = []
        self.vel_y = []
        self.vel_rot_y = []
        self.y = []
        self.y.append(0.0)
        self.rot_y = []
        self.rot_y.append(0.0)

        self.acc_z = []
        self.mean_acc_z = []
        self.vel_z = []
        self.vel_rot_z = []
        self.z = []
        self.z.append(0.0)
        self.rot_z = []
        self.rot_z.append(0.0)

        self.t = []

        # Subscribe to input (IMU) topic
        imu_sub = rospy.Subscriber(self.input_topic, Imu, self.imu_subscriber)
        rospy.loginfo("Subscribed to %s topic", self.input_topic)


    def imu_subscriber(self, data):
        """ Callback function that is called every time a new DVL message arrives
        This function copies the linear velocity to a TwistStamped variable, the obtains the quaternion
        which defines the rotation, and applies the rotation to the vector.
        Once obtained the new rotated vector, publishes it via ROS topic

        :param velstamp: TwistStamped variable with the linear velocity in old_frame coordinates
        :param newvel: quaternion with the transformation between old_frame and new_frame
        :param world_vel: TwistStamped variable with the linear velocity in new_frame coordinates
        """
        # Update transform message values
        self.transf.header.stamp = rospy.Time.now()
        self.transf.header.frame_id = self.old_frame
        self.transf.child_frame_id = self.new_frame

        ## LINEAR COMPONENTS

        # Add linear velocities to the corresponding arrays
        self.acc_x.append(data.linear_acceleration.x-0.08648)
        self.acc_y.append(data.linear_acceleration.y-0.019508)
        self.acc_z.append(data.linear_acceleration.z-9.8159)
        self.t.append(rospy.get_time()) # in secs

        self.mean_acc_x.append(self.average_of_n(self.acc_x[len(self.acc_x)-1],
                                                            self.mean_acc_x,
                                                            5))
        self.mean_acc_y.append(self.average_of_n(self.acc_y[len(self.acc_y)-1],
                                                            self.mean_acc_y,
                                                            5))
        self.mean_acc_z.append(self.average_of_n(self.acc_z[len(self.acc_z)-1],
                                                            self.mean_acc_z,
                                                            5)+0.003145)
  
        avg_imu_pub = rospy.Publisher('/mavros/imu/avg',sensor_msgs.msg.Imu,queue_size=10)
        avg_imu_msg = sensor_msgs.msg.Imu()
        avg_imu_msg.linear_acceleration.x = self.mean_acc_x[len(self.mean_acc_x)-1]
        avg_imu_msg.linear_acceleration.y = self.mean_acc_y[len(self.mean_acc_y)-1]
        avg_imu_msg.linear_acceleration.z = self.mean_acc_z[len(self.mean_acc_z)-1]

        avg_imu_msg.header = data.header
        avg_imu_msg.orientation = data.orientation
        avg_imu_msg.angular_velocity = data.angular_velocity

        avg_imu_pub.publish(avg_imu_msg)


        # Integrate twice and obtain traslation

        # Do it with last two values
        dvx = [self.acc_x[len(self.acc_x)-1], self.acc_x[len(self.acc_x)-2]]
        dvy = [self.acc_y[len(self.acc_y)-1], self.acc_y[len(self.acc_y)-2]]
        dvz = [self.acc_z[len(self.acc_z)-1], self.acc_z[len(self.acc_z)-2]]
        dt = [self.t[len(self.t)-1], self.t[len(self.t)-2]]

        self.vel_x.append(np.trapz(y = dvx, x = dt))
        self.vel_y.append(np.trapz(y = dvy, x = dt))
        self.vel_z.append(np.trapz(y = dvz, x = dt))

        # Do it with last two values
        dx = [self.vel_x[len(self.vel_x)-1], self.vel_x[len(self.vel_x)-2]]
        dy = [self.vel_y[len(self.vel_y)-1], self.vel_y[len(self.vel_y)-2]]
        dz = [self.vel_z[len(self.vel_z)-1], self.vel_z[len(self.vel_z)-2]]
        dt = [self.t[len(self.t)-1], self.t[len(self.t)-2]]

        self.x.append(np.trapz(y = dx, x = self.t))
        self.y.append(np.trapz(y = dy, x = self.t))
        self.z.append(np.trapz(y = dz, x = self.t))


        # Add to transform values
        self.transf.transform.translation.x = self.x[len(self.x)-1]
        self.transf.transform.translation.y = self.y[len(self.y)-1]
        self.transf.transform.translation.z = self.z[len(self.z)-1]

        ## ORIENTATION COMPONENTS

        self.vel_rot_x.append(data.angular_velocity.x)
        self.vel_rot_y.append(data.angular_velocity.y)
        self.vel_rot_z.append(data.angular_velocity.z)
        # Integrate and obtain orientation
        self.rot_x.append(np.trapz(y = self.vel_rot_x, x = self.t))
        self.rot_y.append(np.trapz(y = self.vel_rot_y, x = self.t))
        self.rot_z.append(np.trapz(y = self.vel_rot_z, x = self.t))

        self.transf.transform.rotation.x = data.orientation.x
        self.transf.transform.rotation.y = data.orientation.y
        self.transf.transform.rotation.z = data.orientation.z
        self.transf.transform.rotation.w = data.orientation.w

        # Create broadcaster and broadcast transform
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(self.transf)

        # Publish ROV pose (for debugging)
        self.rov_pose = geometry_msgs.msg.Pose()
        self.rov_pose.position.x = self.x[len(self.x)-1]
        self.rov_pose.position.y = self.y[len(self.y)-1]
        self.rov_pose.position.z = self.z[len(self.z)-1]

        self.rov_pose.orientation.x = data.orientation.x
        self.rov_pose.orientation.y = data.orientation.y
        self.rov_pose.orientation.z = data.orientation.z
        self.rov_pose.orientation.w = data.orientation.w
        self.rov_pose_pub.publish(self.rov_pose)

    def average_of_n(self,last_value,vector,n):
        """ Takes the last n elements of a vector and computes the average """
        avg_vector = []
        # If the array is smaller than required, only mean of available values!
        if len(vector) < n:
            n = len(vector)

        for i in range (n-1,0,-1):
            avg_vector.append(vector[i])

        # add last measurement
        avg_vector.append(last_value)
        
        mean = np.mean(avg_vector)
        return mean
    


    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]


    def closenode(self):
        print 'bye bye'



if __name__ == '__main__':

    rospy.init_node("tf_publisher", log_level=rospy.DEBUG)
    rospy.loginfo("Starting node tf_publisher")

    # Read parameters from console (o launch file)

    old_frame    = rospy.get_param("~old_frame")
    new_frame    = rospy.get_param("~new_frame")
    input_topic  = rospy.get_param("~input_topic")

    rospy.loginfo("Transforming from %s to %s frame", old_frame, new_frame)

    # Do the TF transform magic
    transfdvl = Sensortf(old_frame, new_frame, input_topic, 300.0)

    rospy.spin()

    # Do even cooler stuff here (to do)
