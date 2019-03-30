#!/usr/bin/env python
PKG = 'state_machine_circ'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest

# A sample python unit test
class TestBareBones(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):  # only functions with 'test_'-prefix will be run!
        self.assertEqual(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'state_machine_rostest', TestBareBones)