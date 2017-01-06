#!/usr/bin/env python

from random import uniform 
import rospy, sys
import moveit_commander
#from control_msgs.msg import GripperCommand

class MoveItDemo:
   def __init__(self):
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the ROS node
    rospy.init_node('moveit_demo', anonymous = True)
  
    GRIPPER_OPEN = [0.09]
    GRIPPER_CLOSED = [-0.07]
    GRIPPER_NEUTRAL = [0.075]
     
    # Connect to the arm move group
    arm_controller = moveit_commander.MoveGroupCommander('arm')
    
    # Connect to the gripper move group
    #gripper_controller = moveit_commander.MoveGroupCommander('gripper')
    # Get the name of the end-effector link
    end_effector_link = arm_controller.get_end_effector_link()
   
    # Display the name of the end_effector link
    #rospy.loginfo("The end effector link is :" + str(end_effector_link))

    # Set a small tolerance on joint angles
    arm_controller.set_goal_joint_tolerance(0.000001)
    #gripper_controller.set_goal_joint_tolerance(0.0000001)

    # Start the arm target in "home" pose stored in the SRDF file
    arm_controller.set_named_target('home')

    # Plan a trajectory to the goal configuration
    traj = arm_controller.plan()   
    
    # Execute the planned trajectory
    arm_controller.execute(traj)

    # Pause for a moment
    rospy.sleep(1)
  
    # Set the gripper target to neutral position using a joint value target
    #gripper_controller.go()
    #rospy.sleep(1)
   
    # Set target joint values for the arm : joints are in the order they are appear in the kinectic tree
    #joint_positions = [-0.0424, 1.523, -1.179, -1.124, -0.947, -0.603, -2.881] 
    joint_positions = [uniform(-1,1) for i in range(6)]
    rospy.loginfo("The value of joints is :" + str(joint_positions))
    # Set the arm's goal configuration to be the joint positions
    arm_controller.set_joint_value_target(joint_positions)

    # Plan and execute the motion
    arm_controller.go()
    rospy.sleep(1)
   
    # Save This configuartion for later
    arm_controller.remember_joint_values('saved_config', joint_positions)

    # Close the gripper as if picking something up 
    #gripper_controller.set_joint_value_target(GRIPPER_CLOSED)
    #gripper_controller.go()
    #rospy.sleep(1)
  
    # Set the arm target to the named "staright" pose stored in the SRDF file
    arm_controller.set_named_target('resting')

    # Plan and execute the motion
    arm_controller.go()
    rospy.sleep(1)
 
    # Set the goal configuartion to the named configuration saved earlier
    arm_controller.set_named_target('saved_config')

    # Plan and execute the motion
    arm_controller.go()
    rospy.sleep(1)
  
    # Open the gripper as if letting something go 
    #gripper_controller.set_joint_value_target(GRIPPER_NEUTRAL)
    #gripper_controller.go()
    #rospy.sleep(1)
  
    # Cleanly shut down MoveIt
    moveit_commander.roscpp_shutdown()
 
    # Exit the script
    moveit_commander.os._exit(0)

if __name__ == "__main__":
   try:
      MoveItDemo()
   except rospy.ROSInterruptException :
      pass
  
  

