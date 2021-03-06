#! /usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint 

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
   def __init__(self):
       # Initialize the move_group API
       moveit_commander.roscpp_initialize(sys.argv)
      
       rospy.init_node('moveit_demo')
     
       # Initialize the move group for the arm
       arm = moveit_commander.MoveGroupCommander('arm')
    
       # Get the name of the end_effector link
       end_effector_link = arm.get_end_effector_link()
   
       # Set the reference frame for pose targets
       reference_frame = 'base_link'
      
       # Set the arm refernce frame accordingly
       arm.set_pose_reference_frame(reference_frame)
   
       # Allow replanning to increase the odds of a solution
       arm.allow_replanning(True)
  
       # Allow some leeway in position (meters) and orientation (radians)
       arm.set_goal_position_tolerance(0.000001)
       arm.set_goal_orientation_tolerance(0.0000005)
 
       # Start the arm in the "home" pose stored in the SRDF file
       arm.set_named_target('home')
       arm.go()
       rospy.sleep(2)
   
       # Set the target pose.
       target_pose = PoseStamped()
       target_pose.header.frame_id = reference_frame
       target_pose.header.stamp = rospy.Time.now()
       target_pose.pose.position.x = 0.20
       target_pose.pose.position.y = -0.1
       target_pose.pose.position.z = 0.85
       target_pose.pose.orientation.x = 0.0
       target_pose.pose.orientation.y = 0.0
       target_pose.pose.orientation.z = 0.0
       target_pose.pose.orientation.w = 1.0

       # Set the start state to the current_state
       arm.set_start_state_to_current_state()
      
       # Set the goal pose of the end effector to the stored pose 
       arm.set_pose_target(target_pose, end_effector_link)

       # Plan the trajectory to goal 
       traj = arm.plan()
    
       # Execute the planned trajectory
       arm.execute(traj)
       
       # Pause for a second
       rospy.sleep(1)

       # Shift the end_effector to the right 5cm
       arm.shift_pose_target(1, -0.05, end_effector_link)
       arm.go()
       rospy.sleep(1)
 
       # Rotate the end_effector 90 degrees 
       arm.shift_pose_target(3, -1.57, end_effector_link)   
       arm.go()
       rospy.sleep(1)
    
       # Store this pose as the new target_pose
       saved_target_pose = arm.get_current_pose(end_effector_link)
     
       # Move to the named pose "grasp"
       arm.set_named_target('grasp')
       arm.go()
       rospy.sleep(1)
       
       # Go back to the stored target
       arm.set_pose_target(saved_target_pose, end_effector_link) 
       arm.go()
       rospy.sleep(2)
       
       # Finish up in the resting position
       arm.set_named_target('home')
   
       # Shutdown MoveIt cleanly
       moveit_commander.roscpp_shutdown()

       # EXit MoveIt
       moveit_commander.os._exit(0)

if __name__ == '__main__':
     MoveItDemo()
