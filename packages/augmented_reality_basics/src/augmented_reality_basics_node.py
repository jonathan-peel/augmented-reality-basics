#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class (initialises the node too)
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # retrieve variables
        self.vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        self.map_file = rospy.get_param(f'/{self.vehicle_name}/map_file')
        self.map_file_basename = self.map_file.replace('.yaml','')

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
