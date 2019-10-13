#!/usr/bin/env python

import rospy


if __name__ == '__main__':
    rospy.init_node('abort', anonymous=True)
    rospy.loginfo('abort')
