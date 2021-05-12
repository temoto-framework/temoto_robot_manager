#!/usr/bin/env python

import rospy
import xacro
import sys

try:
    from cStringIO import StringIO ## for Python 2
except ImportError:
    from io import StringIO ## for Python 3

def myhook():
    print("Removing robot_description from parameter server")
    if rospy.has_param("robot_description"):
        rospy.delete_param("robot_description")

rospy.on_shutdown(myhook)

def loader():
    rospy.init_node('urdf_loader', anonymous=True)

    print("Parsing xacro: ", sys.argv[1])
    sys.argv.insert(1, "--inorder")
    sys.stdout = xacro_out = StringIO()
    xacro.main()

    
    sys.stdout = sys.__stdout__
#    print(xacro_out.getvalue())
    rospy.set_param('robot_description', xacro_out.getvalue())

    rospy.spin()

if __name__ == '__main__':
    try:
        loader()
    except rospy.ROSInterruptException:
        pass 