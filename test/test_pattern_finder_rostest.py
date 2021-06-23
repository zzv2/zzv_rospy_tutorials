#!/usr/bin/env python
"""Tests for pattern finder ROS node."""

import unittest

import rospy
import rostest
from std_msgs.msg import (
    Int8,
    String,
)


class TestPatternFinderRostest(unittest.TestCase):

    def setUp(self):
        self.__test_int_pub = rospy.Publisher('/test_int', Int8, queue_size=1)
        rospy.Subscriber('/test_string', String,
                         self.__test_string_callback, queue_size=1)
        self.__test_string = None

    def __test_string_callback(self, string_msg):
        self.__test_string = string_msg

    def test_correct_sequence(self):
        """Test pattern finding with the correct sequence."""
        rospy.sleep(3)
        input_sequence = [8, 9, 10]
        rate = rospy.Rate(10)
        for input_int in input_sequence:
            int8_msg = Int8(data=input_int)
            self.__test_int_pub.publish(int8_msg)
            rate.sleep()

        rospy.sleep(1)
        self.assertIsNotNone(self.__test_string)
        self.assertEquals(self.__test_string.data, 'Found sequence.')


if __name__ == '__main__':
    rospy.init_node('test_pattern_finder_rostest')
    PKG = 'zzv_rospy_tutorials'
    rostest.rosrun(PKG, 'test_pattern_finder_rostest',
                   TestPatternFinderRostest)
