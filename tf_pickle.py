from sensor_msgs.msg import JointState
import rospy
import pickle
import os
import tf2_ros
#from collections import OrderedDict
#dict = OrderedDict()

if __name__ == '__main__':

    rospy.init_node('tf_saver')
    rospy.loginfo("TF saver started!")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1)
    #stored_data = {}

    ##### FILL IN THE APPROPRIATE FILENAME. HINT: USE `raw_input()`
    
    infile = open("tocke",'rb')


    outfile = open("tocke.tmp",'wb')#File more obstajat
    reads=infile.read()
    outfile.write(reads)
    
    
    saved_data = {}
    #########################
    ##### STUDENT WRITES ####
    #########################

    name = "  "
    while not rospy.is_shutdown():
        name = raw_input("Transformation name(exit to quit): ")
        if (name == 'exit'):
            break
        transformation = tf_buffer.lookup_transform("panda_link0", "panda_link8", rospy.Time(0))
        transformation.child_frame_id = name
        joints = rospy.wait_for_message('/joint_states', JointState, rospy.Duration(1))
        print(name)
        print(transformation)
        print(joints)
        print("###############################################################")
        saved_data[name] = [transformation,joints]
        
    
    pickle.dump(saved_data, outfile)
    outfile.close()
    infile.close()
    import os 
    os.rename("tocke.tmp","tocke")


    #transformation = tf_buffer.lookup_transform(saved_frames[0], saved_frames[2], rospy.Time(0))
    #saved_data[raw_input("Ime transformacije: ")] = transformation
    # Note - from_frame and to_frame need to be defined!


    #########################