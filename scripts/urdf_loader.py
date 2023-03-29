#!/usr/bin/env python3

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

    if (len(sys.argv) == 3):
        robot_description = sys.argv[2]
    else:
        robot_description = "robot_description"
    
    print(" *************** ++++++++++ ************ +++++++++++ **********")
    print(sys.argv)
    print(len(sys.argv))
    print(sys.argv[2])              # ns of robot_description
    sys.argv.pop(2)
    print(len(sys.argv))

    print("Parsing xacro: ", sys.argv[1])
    sys.argv.insert(1, "--inorder")
    print("After insert")
    sys.stdout = xacro_out = StringIO()
    print("stdout insert")
    xacro.main()
    print("After xacro main")

    
    sys.stdout = sys.__stdout__
#    print(xacro_out.getvalue())
    print("After sys stdout")
    
    # print(sys.argv[2]+'/robot_description')

    # rospy.set_param('robot_description', xacro_out.getvalue())
    rospy.set_param(robot_description, xacro_out.getvalue())
    print("After set param")

    rospy.spin()

if __name__ == '__main__':
    try:
        loader()
    except rospy.ROSInterruptException:
        pass 
