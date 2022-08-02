#!/usr/bin/env python3
import rospy
import yaml
import rospy

def get_yaml():
    with open("file.yaml", "r") as stream:
        try:
            data = (yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            data = exc
    return data

def write_yaml(data):
    """ A function to write YAML file"""

    with open('file.yaml', 'w') as stream:
        try:
            yaml.dump(data, stream)
        except yaml.YAMLError as exc:
            print(exc)


if rospy.has_param('/camel_amr_1000_001/amcl/initial_pose_a'):
    pose_a = rospy.get_param('/camel_amr_1000_001/amcl/initial_pose_a') 
    pose_x = rospy.get_param('/camel_amr_1000_001/amcl/initial_pose_x') 
    pose_y = rospy.get_param('/camel_amr_1000_001/amcl/initial_pose_y') 
    
    data = {'pose_a': pose_a,'pose_x': pose_x,'pose_y': pose_y}
    write_yaml(data)

else:
    rospy.set_param('/camel_amr_1000_001/amcl/initial_pose_a',get_yaml['pose_a'])
    rospy.set_param('/camel_amr_1000_001/amcl/initial_pose_x',get_yaml['pose_x'])
    rospy.set_param('/camel_amr_1000_001/amcl/initial_pose_y',get_yaml['pose_y'])

