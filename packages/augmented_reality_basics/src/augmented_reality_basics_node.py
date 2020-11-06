#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import yaml
import cv2

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class (initialises the node too)
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # retrieve variables
        self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        self.map_file = rospy.get_param(f'/{self.vehicle_name}/map_file')
        self.map_file_basename = self.map_file.replace('.yaml','')

        # read in map_file
        map_dir = f'/code/catkin_ws/src/augmented-reality-basics/packages/augmented_reality_basics/map/{self.map_file}'
        self.map_dict = self.read_yaml_file(map_dir)
        rospy.loginfo(self.map_dict)

        # construct subscriber to images
        image_sub_topic = f'/{self.vehicle_name}/camera_node/image/compressed'
        self.image_sub = rospy.Subscriber(image_sub_topic, CompressedImage, self.callback)
        rospy.loginfo(f'Subscribed to: {image_sub_topic}')

        # construct publisher for the images with the projected yaml file data
        image_pub_topic = f'/{self.vehicle_name}/{node_name}/{self.map_file_basename}/image/compressed'
        self.image_pub = rospy.Publisher(image_pub_topic, CompressedImage, queue_size=16)
        rospy.loginfo(f'Publishing to: {image_pub_topic}')


    def callback(self, data):
        """ Once recieving the image, project the map features onto it
        """
        self.augmented_image = ''
        
    def read_yaml_file(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown('Yaml file not found')
                return

    # def draw_segment(self, image, pt_x, pt_y, color):
    #     defined_colors = {
    #         'red': ['rgb', [1, 0, 0]],
    #         'green': ['rgb', [0, 1, 0]],
    #         'blue': ['rgb', [0, 0, 1]],
    #         'yellow': ['rgb', [1, 1, 0]],
    #         'magenta': ['rgb', [1, 0 , 1]],
    #         'cyan': ['rgb', [0, 1, 1]],
    #         'white': ['rgb', [1, 1, 1]],
    #         'black': ['rgb', [0, 0, 0]]}
    #     _color_type, [r, g, b] = defined_colors[color]
    #     cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
    #     return image

    def run(self):
        # publish image 8 times per second
        rate = rospy.Rate(8) 

        while not rospy.is_shutdown():


            # self.image_pub.publish(self.augmented_image)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    rospy.loginfo('Initialising node ...')
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    rospy.loginfo('Node initalised.')

    # run node
    node.run()
