#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo
import yaml
import cv2
import numpy as np
import copy
from augmented_reality_basics_module import Augmenter

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
        rospy.loginfo(f'Yaml dict output from map is: {self.map_dict}')

        # find the calibration parameters
        self.camera_info = self.load_calibration_params()
        self.log(f'Camera info: {self.camera_info}')

        # construct publisher for the images with the projected yaml file data
        image_pub_topic = f'/{self.vehicle_name}/{node_name}/{self.map_file_basename}/image/compressed'
        self.image_pub = rospy.Publisher(image_pub_topic, CompressedImage, queue_size=16)
        rospy.loginfo(f'Publishing to: {image_pub_topic}')

        # construct subscriber to images
        image_sub_topic = f'/{self.vehicle_name}/camera_node/image/compressed'
        self.image_sub = rospy.Subscriber(image_sub_topic, CompressedImage, self.callback)
        rospy.loginfo(f'Subscribed to: {image_sub_topic}')


    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.
        Loads the intrinsic and extrinsic camera matrices.
        Args:
            filename (:obj:`str`): filename of calibration files.
        Returns:
            :obj:`CameraInfo`: a CameraInfo message object
        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


    def load_calibration_params(self):
        # Find the calibration parameters
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        rospy.loginfo(f'Looking for calibration {self.cali_file}')
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        original_camera_info = self.load_camera_info(self.cali_file)
        original_camera_info.header.frame_id = self.frame_id
        current_camera_info = copy.deepcopy(original_camera_info)
        # self.update_camera_params() # only used if camera is a different res?
        self.log("Using calibration file: %s" % self.cali_file)

        return current_camera_info


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


    def draw_segment(self, image, pt_x, pt_y, color):
        """ Draws a segment (line) between two points only.
            pt_x is a list of 2 points for each x coord in the line (as for y)

        """

        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}

        _color_type, [r, g, b] = defined_colors[color]
        new_image = cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
        return new_image

    def callback(self, image_data):
        """ Once recieving the image, save the data in the correct format
            is it okay to do a lot of computation in the callback?
        """
        np_arr = np.fromstring(image_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # OpenCV < 3
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:


        # draw on the image
        image_width = image_np.shape[1]
        image_height = image_np.shape[0]
        segments = self.map_dict['segments'] # a list of segments dicts
        points = self.map_dict['points'] # dict of points

        for segment in segments:
            # deconstruct map_dict into pt_x and pt_y
            segment_points_names = segment['points'] # eg [TL, TR]
            segment_points_coords = []
            segment_points_coords.append(points[segment_points_names[0]][1]) # eg [0, 0]
            segment_points_coords.append(points[segment_points_names[1]][1])
            # segment_points_coords now eg [[0, 0], [0, 1]]
            pt_x = [segment_points_coords[0][0], segment_points_coords[1][0]]
            pt_y = [segment_points_coords[0][1], segment_points_coords[1][1]]
            # pt_x is eg [0, 0]
            segment_color = segment['color'] # eg 'red'

            pt_x = [i * image_width for i in pt_x] # just so you can actually see it!
            pt_y = [i * image_height for i in pt_y] # just so you can actually see it!

            image_np = self.draw_segment(image_np, pt_x, pt_y, segment_color)

        augmented_image = image_np

        # make new CompressedImage to publish
        augmented_image_msg = CompressedImage()
        augmented_image_msg.header.stamp = rospy.Time.now()
        augmented_image_msg.format = "jpeg"
        augmented_image_msg.data = np.array(cv2.imencode('.jpg', augmented_image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(augmented_image_msg)


    def run(self):
        # publish image 8 times per second
        rate = rospy.Rate(8) 

        while not rospy.is_shutdown():
            # publish straight from the callback, like in the example? 
            # http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
            # self.image_pub.publish(self.augmented_image)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    rospy.loginfo('Initialising node ...')
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    rospy.loginfo('Node initalised.')

    # run node
    node.run()
