#!/usr/bin/python

import sys
import rospy
import pickle
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from edit_img import edit_img

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
    goal.epsilon.inner = 0.01
    goal.epsilon.outer = 0.01
    goal.speed = 0.1
    goal.force = 100

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult

def transformstamped_to_pose(transform_stamped):
    """Convert from geometry_msgs/TransformStamped to geometry_msgs/Pose

    Args:
        transform_stamped (geometry_msgs/TransformStamped): The geometry_msgs/TransformStamped message to convert

    Returns:
        Pose: The geometry_msgs/Pose message
    """
    return Pose(position=transform_stamped.transform.translation, orientation=transform_stamped.transform.rotation)

def get_pose_from_tf(target, tf_buffer, relative_to='panda_link0'):
    """Return a Pose object to the desired TF frame

    Args:
        target (str): TF frame that we are interested in
        tf_buffer (tf2_ros.Buffer): A TF buffer. Don't forget to initialize it with the listener!!!
        relative_to (str, optional): Relative to ...

    Return:
        Pose: The TF frame expressed as geometry_msgs/Pose
    """

    rospy.sleep(0.1)
    target_transform = tf_buffer.lookup_transform(relative_to, target, rospy.Time(0))
    return transformstamped_to_pose(target_transform)

def main():

    rospy.init_node('panda_example', anonymous=True)  
    
    result = grasp_client(0.08)
    print("Success: ",result.success)
    print("Error message: ", result.error)
    result = grasp_client(0.00)
    print("Success: ",result.success)  
    
    tf_broad = tf2_ros.StaticTransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    ##### BERI PICKLE FILE########
    infile = open('tocke','rb')
    stored_poses = pickle.load(infile)
    infile.close()
    ###############################
    
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
    group.set_max_acceleration_scaling_factor(0.3)
    group.set_pose_reference_frame('panda_link0')
    group.set_max_velocity_scaling_factor(0.1)

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
    
    ########NASA KODA#######
    array = []
    rospy.sleep(0.1)
    
    for frame, joint in stored_poses.values():
        array.append(frame)
        print(joint, frame)
    print(array)
    tf_broad.sendTransform(array)
    rospy.sleep(0.2)
    
    state = False
    gripped = False
    offset = 0.1
    while True:
        polja = edit_img()
        finished = True
        # Ustvarimo path glede na zasedena polja
        for i,p in enumerate(polja,1):
            if p == 1:
                spot1 = "P"+str(i)
                spot2 = "O"+str(i)
                path = ["Home", "offset", spot1, "grip", "offset", "Home", "offset", spot2, "grip", "offset"]
            else:
                continue
            for j,p in enumerate(path):
                # tukaj delamo offsete definiranih tock pred in po pobiranju/odlaganju
                if p == "offset":
                    if not state:
                        goto_pose = get_pose_from_tf(path[j+1], tf_buffer)
                        state = True
                    else:
                        goto_pose = get_pose_from_tf(path[j-2], tf_buffer)
                        state = False
                    goto_pose.position.z += offset
                elif p == "grip":
                    if not gripped:
                        result = grasp_client(0.0241)
                        gripped = True
                    else:
                        result = grasp_client(0.00)
                        gripped = False
                    continue
                else:
                    goto_pose = get_pose_from_tf(p,tf_buffer)
                group.set_pose_target(goto_pose)
                test_plan = group.plan()
                group.execute(test_plan, wait=True)
                group.stop() 
                group.clear_pose_targets()
            
            x = raw_input("Loop finished. Press any key to continue, 'exit' to quit, 'photo' to take new photo: ")
            if x == "exit":
                return
            elif x == "photo":
                finished = False
                break
            
        group.set_pose_target(get_pose_from_tf("Home",tf_buffer))
        test_plan = group.plan()
        group.execute(test_plan, wait=True)
        group.stop() 
        group.clear_pose_targets()
        
        if finished:
            f = raw_input("Path finished. Press any key for new path, 'exit' to quit: ")
            if f == "exit":
                return

    ########################

if __name__ == "__main__":
    main()
    
    