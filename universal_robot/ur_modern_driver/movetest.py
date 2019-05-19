#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time, sys
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

 
if sys.platform[:3] == 'win':
    import msvcrt
    def getkey():
        key = msvcrt.getch()
        return key
elif sys.platform[:3] == 'lin':
    import termios, sys, os
    TERMIOS = termios
 
    def getkey():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
        new[6][TERMIOS.VMIN] = 1
        new[6][TERMIOS.VTIME] = 0
        termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
        c = None
        try:
            c = os.read(fd, 1)
        finally:
            termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
        return c
 
'''while True:
    k = getkey().decode()
    print k=='a'
'''

rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
#gripper action
action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
robotiq_client.wait_for_server()

#rospy.logwarn("Client test: Starting sending goals")

#moveit_commander.roscpp_initialize(sys.argv)

#move)_group
#rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)

group.set_planner_id('RRTConnect')
#group.set_end_effector_link('tool0')
#group.set_pose_reference_frame('base_link')

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

referenceframe=group.get_pose_reference_frame()
print "============  pose frame:%s" % referenceframe

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print group.get_current_pose().pose
print ""
rospy.sleep(5)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "ee_link"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = -0.3 # slightly above the end effector
box_pose.pose.position.x= 0.3
box_name = "box"
#scene.add_box(box_name, box_pose, size=(0.1, 0.4, 0.4))
#onm=scene.get_known_object_names()


#print onm

##current pose
'''position: 
  x: -0.15883890231
  y: 0.451648478298
  z: 0.596839923892
orientation: 
  x: 0.0242705393267
  y: 0.643287386718
  z: 0.76475801414
  w: 0.0271561930158'''

'''scale=1.0
waypoints = []

wpose = group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold'''

'''joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
group.stop()
'''

while(1):
  key=getkey().decode()
  if key=='w':
    print "========up======="
    current_pose=group.get_current_pose().pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = current_pose.orientation.x
    pose_goal.orientation.y = current_pose.orientation.y
    pose_goal.orientation.z = current_pose.orientation.z
    pose_goal.orientation.w = current_pose.orientation.w
    pose_goal.position.x = current_pose.position.x
    pose_goal.position.y = current_pose.position.y+0.1
    pose_goal.position.z = current_pose.position.z
    group.set_pose_target(pose_goal)

    plan = group.plan()

    group.go(wait=True)

  if key=='a':
      print "========left======="
      current_pose=group.get_current_pose().pose
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w
      pose_goal.position.x = current_pose.position.x-0.1
      pose_goal.position.y = current_pose.position.y
      pose_goal.position.z = current_pose.position.z
      group.set_pose_target(pose_goal)

      plan = group.plan()

      group.go(wait=True)   


  if key=='s':
      print "========back======="
      current_pose=group.get_current_pose().pose
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w
      pose_goal.position.x = current_pose.position.x
      pose_goal.position.y = current_pose.position.y-0.1
      pose_goal.position.z = current_pose.position.z
      group.set_pose_target(pose_goal)

      plan = group.plan()

      group.go(wait=True)

  if key=='d':
      print "========right======="
      current_pose=group.get_current_pose().pose
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w
      pose_goal.position.x = current_pose.position.x+0.1
      pose_goal.position.y = current_pose.position.y
      pose_goal.position.z = current_pose.position.z
      group.set_pose_target(pose_goal)

      plan = group.plan()

      group.go(wait=True)

  if key=='z':
      print "========up======="
      current_pose=group.get_current_pose().pose
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w
      pose_goal.position.x = current_pose.position.x
      pose_goal.position.y = current_pose.position.y
      pose_goal.position.z = current_pose.position.z+0.1
      group.set_pose_target(pose_goal)

      plan = group.plan()

      group.go(wait=True)

  if key=='x':
      print "========down======="
      current_pose=group.get_current_pose().pose
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = current_pose.orientation.x
      pose_goal.orientation.y = current_pose.orientation.y
      pose_goal.orientation.z = current_pose.orientation.z
      pose_goal.orientation.w = current_pose.orientation.w
      pose_goal.position.x = current_pose.position.x
      pose_goal.position.y = current_pose.position.y
      pose_goal.position.z = current_pose.position.z-0.1
      group.set_pose_target(pose_goal)

      plan = group.plan()

      group.go(wait=True)    

  if key=='c':
      print "========close gripper======="
      Robotiq.close(robotiq_client,speed=0.1, force=0, block=True)

  if key=='o':
      print "========close gripper======="
      Robotiq.open(robotiq_client, block=True)
'''current_pose=group.get_current_pose().pose
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = current_pose.orientation.x
pose_goal.orientation.y = current_pose.orientation.y
pose_goal.orientation.z = current_pose.orientation.z
pose_goal.orientation.w = current_pose.orientation.w
pose_goal.position.x = current_pose.position.x
pose_goal.position.y = current_pose.position.y
pose_goal.position.z = current_pose.position.z-0.1
group.set_pose_target(pose_goal)

plan = group.plan()

group.go(wait=True)

group.stop()
group.clear_pose_targets()


#group.execute(plan, wait=True)'''
