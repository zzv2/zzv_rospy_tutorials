"""Pattern finder module."""

from collections import namedtuple

import rospy
from std_msgs.msg import (
    String,
)


PatternFinderParams = namedtuple('PatternFinderParams', [
    'sequence',
])

PatternFinderPubs = namedtuple('PatternFinderPubs', [
    'test_string',
])


class PatternFinder(object):
    """Finds patters in a stream of numbers."""

    def __init__(self, params, pubs):
        """Construct a PatternFinder object.

        Args:
            params (namedtuple): Parameters for pattern finding.
            pubs (namedtuple): Publishers for pattern finding.
        """
        self.__params = params
        self.__pubs = pubs
        self.__history = []

    @staticmethod
    def sequence_equals(sequence1, sequence2):
        """Determine if the sequences are equal.

        Args:
            sequence1 (lists): First sequence to check for equality.
            sequence2 (lists): Second sequence to check for equality.
        """
        return sequence1 == sequence2

    def int_callback(self, int8_msg):
        """Handle integer callback.

        Args:
            int8_msg (std_msgs/Int8): Integer msg input.
        """
        self.__history.append(int8_msg.data)
        if len(self.__history) > len(self.__params.sequence):
            self.__history.pop(0)
        if self.sequence_equals(self.__history, self.__params.sequence):
            string_msg = String(data='Found sequence.')
            self.__pubs.test_string.publish(string_msg)
            rospy.logdebug(('Published string:', string_msg))

        rospy.logdebug(('int8_msg', int8_msg))
        rospy.logdebug(('self.__history', self.__history))
        rospy.logdebug(('self.__params.sequence', self.__params.sequence))
