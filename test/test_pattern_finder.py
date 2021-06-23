"""Tests for pattern finder."""

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
        self.__test_string = None

    def tearDown(self):
        pass

    def string_callback(self, test_string):
        self.__test_string = test_string

    def test_sequence_equals(self):
        """Test determining if the sequences are equal."""
        self.assertFalse(PatternFinder.sequence_equals([1, 2, 3], [1, 2]))
        self.assertTrue(PatternFinder.sequence_equals([1, 2, 3], [1, 2, 3]))

    def test_int_callback_wrong_sequence(self):
        """Test handling integer callback with the wrong sequence."""
        input_sequence = [1, 2, 3]
        for input_int in input_sequence:
            self.__pattern_finder.int_callback(Int8(data=input_int))
        self.assertIsNone(self.__test_string)

    def test_int_callback_correct_sequence(self):
        """Test handling integer callback with the correct sequence."""
        input_sequence = [4, 5, 6]
        for input_int in input_sequence:
            self.__pattern_finder.int_callback(Int8(data=input_int))
        self.assertIsNotNone(self.__test_string)
        self.assertEquals(self.__test_string.data, 'Found sequence.')

    def test_int_callback_correct_with_extra(self):
        """Test handling integer callback with correct sequence and extra."""
        input_sequence = [3, 4, 5, 6]
        for input_int in input_sequence:
            self.__pattern_finder.int_callback(Int8(data=input_int))
        self.assertIsNotNone(self.__test_string)
        self.assertEquals(self.__test_string.data, 'Found sequence.')
