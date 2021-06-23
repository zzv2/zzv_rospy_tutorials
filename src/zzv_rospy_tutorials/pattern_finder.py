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

    def int_callback(self, int8_msg):
        """Handle integer callback.
        
        Args:
            int8_msg (std_msgs/Int8): Integer msg input.
        """
        rospy.loginfo(('int8_msg', int8_msg))
        string_msg = String(data='Found sequence.')
        self.__pubs.test_string.publish(string_msg)
        rospy.loginfo(('Published string:', string_msg))
