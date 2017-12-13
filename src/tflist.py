#!/usr/bin/env python

# Node that uses transform obtained in tf_transformer.py
# to take a point in the "base_laser" frame and transform
# it to a point in the "base_link" frame.

import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs
from underwater_sensor_msgs.msg import DVL
from geometry_msgs.msg import Twist
import numpy as np
from math import sqrt
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
    def __init__(self, old_frame, new_frame, input_topic, output_topic, rate):

        self.old_frame = old_frame
        self.new_frame = new_frame
        self.input_topic = input_topic
        self.output_topic = output_topic

        rospy.on_shutdown(self.closenode)

        # Updating rate using ROS rating
        self.rate = rate
        self.rrate = rospy.Rate(self.rate)

        # Create object that reads transform
        self.dvltfBuffer = tf2_ros.Buffer()
        self.dvl_listener = tf2_ros.TransformListener(self.dvltfBuffer)

        # Subscribe to DVL sensor topic
        dvl_sub = rospy.Subscriber(self.input_topic, DVL, self.dvl_subscriber)
        rospy.loginfo("Subscribed to %s topic", self.input_topic)


    def dvl_subscriber(self, data):
        """ Callback function that is called every time a new DVL message arrives
        This function copies the linear velocity to a TwistStamped variable, the obtains the quaternion
        which defines the rotation, and applies the rotation to the vector.
        Once obtained the new rotated vector, publishes it via ROS topic

        :param velstamp: TwistStamped variable with the linear velocity in old_frame coordinates
        :param newvel: quaternion with the transformation between old_frame and new_frame
        :param world_vel: TwistStamped variable with the linear velocity in new_frame coordinates
        """
        velstamp  = self.vel_msg(self.old_frame, data.bi_x_axis, data.bi_y_axis, data.bi_z_axis)
        newvel    = self.transform_vel(self.new_frame, self.old_frame)
        tfvel     = self.rotate_vector_by_quaternion(velstamp, newvel)
        world_vel = self.vel_msg(self.new_frame, tfvel[0], tfvel[1], tfvel[2])

        pub = rospy.Publisher(self.output_topic, geometry_msgs.msg.TwistStamped, queue_size=10)
        pub.publish(world_vel)

        rospy.logdebug("The velocity in DVL coordinates is: \n %s", velstamp)
        rospy.logdebug("The transform between world and DVLSensor is: \n %s", newvel)
        rospy.logdebug("The velocity in world coordinates is: \n %s", world_vel)


    def pose_msg(self, position, orientation, frame):
        # creates PoseStamp msg
        # @param position: Point msg with position
        # @param orientation: Quaternion msg with orientation
        # @param frame: string with frame of the pose.
        # @return posestamp: posestamped msg.
        self.posestamp = geometry_msgs.msg.PoseStamped()
        self.posestamp.pose.position = position
        self.posestamp.pose.orientation = orientation
        self.posestamp.header.frame_id = frame
        self.posestamp.header.stamp = rospy.Time.now()

        return self.posestamp

    def vel_msg(self, frame, vx, vy, vz):
        """ Creates TwistStamped message with linear velocities *vx*,*vy*, *vz* in frame *frame*
        :return self.velstamp: TwistStamped message
        """
        self.velstamp = geometry_msgs.msg.TwistStamped()
        self.velstamp.header.frame_id = frame
        self.velstamp.header.stamp = rospy.Time.now()
        self.velstamp.twist.linear.x = vx
        self.velstamp.twist.linear.y = vy
        self.velstamp.twist.linear.z = vz
        return self.velstamp



    def rotate_vector_by_quaternion(self, v, q):
        """
        Function which rotates vector *v* with quaternion *q*

        :param v: TwistStamped variable with the linear velocity in old_frame coordinates
        :param q: quaternion with the transformation between old_frame and new_frame
        :return vout: rotated vector
        """
        # @param v: TwistStamped vector to rotate
        # @param q: quaternion

        # Vector part of quaternion
        u = np.array([q.transform.rotation.x, q.transform.rotation.y, q.transform.rotation.z])
        # Int part of quaternion
        s = np.array([q.transform.rotation.w])
        # Velocity vector
        v1 = np.array([v.twist.linear.x, v.twist.linear.y, v.twist.linear.z])

        # Optimized way to do the math
        vout = 2.0*np.dot(u, v1)*u\
               +(s*s-np.dot(u, u))*v1\
               +2.0*s*np.cross(u, v1)

        return vout


    def transform_point(self, new_base, old_base):
        # transform sensor pose to the pose of the base frame.
        # @param base_id string with the name of the base frame.
        # @param sensor_point position of the sensor on its own coordinates
        # @return transf_pose: PoseStamp message with the transf. pose

        # Wait until there's a transformation available
        # self.robot_listener.waitForTransform(old_base, new_base, rospy.Time(),
                                            # rospy.Duration(4.0))

        if self.dvl_listener.waitForTransform("/base_laser", "/base_link",
                                                rospy.Time.now(),
                                                rospy.Duration(4.0)):
            return self.dvl_listener.transformPose(new_base, self.posestamp)
        else:
            rospy.logwarn('Couldnt transform pose, out of time')


    def transform_vel(self, new_base, old_base):
        """
        This function obtains the transform between *new_base* and *old_base*
        :param new_base: new frame you want to express your vector in
        :param old_base: old frame in which your vector is actually expressed
        :return:
        """

        return self.dvltfBuffer.lookup_transform(new_base, old_base, rospy.Time())

    def closenode(self):
        """
        Do this when closing down
        :return:
        """
        rospy.loginfo("Shutting down Sensortf...")
        rospy.sleep(0.1)

if __name__ == '__main__':

    rospy.init_node("tf_dvl_listener", log_level=rospy.DEBUG)
    rospy.loginfo("Starting node tf_dvl_listener")

    # Read parameters from console (o launch file)

    old_frame    = rospy.get_param("~old_frame")
    new_frame    = rospy.get_param("~new_frame")
    input_topic  = rospy.get_param("~input_topic")
    output_topic = rospy.get_param("~output_topic")

    rospy.loginfo("Transforming from %s to %s frame", old_frame, new_frame)

    # Do the TF transform magic
    transfdvl = Sensortf(old_frame, new_frame, input_topic, output_topic, 300.0)

    rospy.spin()
