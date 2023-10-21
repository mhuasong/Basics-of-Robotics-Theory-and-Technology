#!/usr/bin/env python   
import rospy
import tf
import math
import geometry_msgs.msg

if __name__ == '__main__':
# Init the node
       rospy.init_node('circle_reference')
# Publisher for the reference
       ref_pub = rospy.Publisher('/mobile_base_controller/reference', geometry_msgs.msg.Point,queue_size=1)
# Define a transform Broadcaster
       br = tf.TransformBroadcaster()
       listener = tf.TransformListener()
# Define the node execution frequency
       rate = rospy.Rate(10.0)
       while not rospy.is_shutdown():
            t =rospy.Time.now().to_sec() * math.pi
            x = 2.0 * math.cos(t/70)
            y = 2.0 * math.sin(t/70)
# Create a child frame of odom for see the reference in RVIZ
            br.sendTransform([ x, y, 0.0],
                            [0.0, 0.0, 0.0, 1.0],
                            rospy.Time.now(),
                            "reference",
                            "odom")
    # publish the reference topic
            reference = geometry_msgs.msg.Point()
            reference.x = x
            reference.y = y
            ref_pub.publish(reference)

            rate.sleep() 

