#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
Converts leg_tracker PersonArray into people_msgs people. This conversion is lossy,
as covariance matrix, modality, detection ID and confidence are omitted during the conversion.
"""

import rospy
from people_msgs.msg import People
from leg_tracker.msg import PersonArray Person
from geometry_msgs.msg import PoseArray, Pose
import math
import tf

def newMessageReceived(personArray):
    people = People()
    people.header = personArray.header
    number=0
    for personArray in personArray.people:
        people.people.append(detectedPerson.people.pose)
        
        print number
        number=number+1
        """
        people.people.position.x=msg.people[0].pose.position.x
        people.people.position.y=msg.people[0].pose.position.y
        people.people.velocity.x=math.sin(tf.transformations.euler_from_quaternion([msg.people[0].pose.orientation.x, msg.people[0].pose.orientation.y, msg.people[0].pose.orientation.z, msg.people[0].pose.orientation.w])[2]
)
        people.people.velocity.y=math.cos(tf.transformations.euler_from_quaternion([msg.people[0].pose.orientation.x, msg.people[0].pose.orientation.y, msg.people[0].pose.orientation.z, msg.people[0].pose.orientation.w])[2]
)
        people.people.tagnames=msg.people[0].id
        people.people.tags=msg.people[0].id
        """

    pub.publish(people)


# Initialize node
rospy.init_node("leg_tracker_person_array_to_people_msgs_people")

# Create publisher and subscriber
inputTopic = rospy.resolve_name("/people_tracked")
outputTopic = rospy.resolve_name("/people")

sub = rospy.Subscriber(inputTopic, PersonArray, newMessageReceived, queue_size=5)
pub = rospy.Publisher(outputTopic, People, queue_size=5)

rospy.loginfo("Re-publishing people_msgs_people from %s as leg_tracker_person_array at %s" % (inputTopic, outputTopic) )
rospy.spin()
