"""Tests for data collection mask generator functions."""

import mock
import unittest

from std_msgs.msg import (
    Int8,
)
from zzv_rospy_tutorials.pattern_finder import (
    PatternFinder,
    PatternFinderParams,
    PatternFinderPubs,
)


class TestPatternFinder(unittest.TestCase):

    def setUp(self):
        sequence = [4, 5, 6]
        params = PatternFinderParams(
            sequence=sequence
        )

        test_string_pub = mock.Mock()
        test_string_pub.publish = self.string_callback
        pubs = PatternFinderPubs(
            test_string=test_string_pub
        )

        self.__pattern_finder = PatternFinder(params, pubs)

    def tearDown(self):
        pass

    def string_callback(self, test_string):
        self.__test_string = test_string

    def test_simple_model1(self):
        """Test mask generation for an object with a simple model."""
        input_sequence = [1, 2, 3]
        self.__pattern_finder.int_callback(Int8(data=input_sequence[0]))
        self.assertEquals(self.__test_string.data, 'Found sequence.')
