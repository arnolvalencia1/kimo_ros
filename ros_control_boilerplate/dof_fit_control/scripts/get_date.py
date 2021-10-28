#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

print ("============ generando plan 1")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.4
pose_target.position.y = 0.4
pose_target.position.z = 0.4
group.set_pose_target(pose_target)

plan1 = group.plan()

rospy.sleep(5)

moveit_commander.roscpp_shutdown()

#Puede obtener el marco de referencia para un determinado grupo ejecutando esta línea:
#print ("Reference frame: %s" % group.get_planning_frame())

#Puede obtener el enlace del efector final para un grupo determinado que ejecute esta línea:
#print ("End effector: %s" % group.get_end_effector_link())

#Puede obtener una lista con todos los grupos del robot así:
#print ("Robot Groups:")
#print (robot.get_group_names())

#Puede obtener los valores actuales de las articulaciones de esta manera:
#print ("Current Joint Values:")
#print (group.get_current_joint_values())

#También puede obtener la Pose actual del efector final del robot de esta manera:
#print ("Current Pose:")
#print (group.get_current_pose())

#Finalmente, puede verificar el estado general del robot de esta manera:
#print ("Robot State:")
#print (robot.get_current_state())