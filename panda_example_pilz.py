#!/usr/bin/python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion

import franka_gripper.msg
import actionlib

def grasp_client(width):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
    #client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal()
    goal.width = width
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 5

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult

def main():

    rospy.init_node('panda_example', anonymous=True)

    result = grasp_client(0.08)
    print("Success: ",result.success)
    print("Error message: ", result.error)
    result = grasp_client(0.04)
    print("Success: ",result.success)   
    
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name, ns="", robot_description="/robot_description")
    group.set_planner_id('LIN')
    group.set_max_acceleration_scaling_factor(1.0)
    group.set_pose_reference_frame('panda_link0')
    group.set_max_velocity_scaling_factor(1.0)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)


    # To print the reference frame of robot:
    #planning_frame = group.get_planning_frame()
    #print(planning_frame)

    # To print a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print(group_names)

    # To print the entire state of the robot:
    #robot_state = robot.get_current_state()
    #print(robot_state)

    group.clear_pose_targets()

    # Print current pose
    current_pose = group.get_current_pose().pose
    print(current_pose)

    x = input("Go?")
    # Move 10 cm above current pose
    goto_pose = group.get_current_pose().pose
    goto_pose.position.z += 0.1
    group.set_pose_target(goto_pose)
    test_plan = group.plan()
    group.execute(test_plan, wait=True)
    group.stop() 
    group.clear_pose_targets()

    x = input("Go?")
    # Move to specific pose
    goto = PoseStamped()
    goto.header.seq = 0
    goto.header.stamp = rospy.Time.now()
    goto.header.frame_id = "panda_link0"
    #tocka za zacetek programa
    goto.pose.position = Point(0.25, 0.15, 0.58)
    goto.pose.orientation = Quaternion(1.0, 0.0, 0.0, 0.0)
    group.set_pose_target(goto)
    group.set_max_velocity_scaling_factor(0.05)
    test_plan = group.plan()
    group.execute(test_plan, wait=True)
    group.stop() 
    group.clear_pose_targets()

    x = input("Go?")
    # Move to specific pose with try
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = 0.24
    pose_goal.position.y = 0.18
    pose_goal.position.z = 0.60
    group.set_pose_target(pose_goal)
    group.set_max_velocity_scaling_factor(1.0)
    print("Starting move")
    try:
        test_plan = group.plan()
        #plan = group.go(wait=True)
    except KeyboardInterrupt as e:
        print("Exception: " + str(e))
    print("planned")
    group.execute(test_plan, wait=True)
    group.stop() 
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print(current_pose)

if __name__ == "__main__":
    main()