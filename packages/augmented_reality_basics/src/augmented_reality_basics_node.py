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
        rospy.loginfo(f'Camera info: {self.camera_info}')
        self.homography = self.load_extrinsics()
        rospy.loginfo(f'Homography: {self.homography}')

        # Initialise Augmenter class
        self.augmenter = Augmenter(self.homography, self.camera_info)

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
            calib_data = yaml.load(stream, Loader=yaml.Loader)
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


    def load_extrinsics(self):
        """
        Loads the homography matrix from the extrinsic calibration file.
        Returns:
            :obj:`numpy array`: the loaded homography matrix
        """
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        try:
            with open(cali_file,'r') as stream:
                calib_data = yaml.load(stream, Loader=yaml.Loader)
        except yaml.YAMLError:
            msg = 'Error in parsing calibration file %s ... aborting' % cali_file
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        return calib_data['homography']


    def read_yaml_file(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file, Loader=yaml.Loader)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown('Yaml file not found')
                return


    def callback(self, image_data):
        """ Once recieving the image, save the data in the correct format
            is it okay to do a lot of computation in the callback?
        """
        rospy.loginfo('Image recieved, running callback.')


        # Extract the image from the subscriber into a numpy array
        np_arr = np.fromstring(image_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # OpenCV < 3
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        # Undistort (rectify) image
        # undistorted_image = self.augmenter.process_image()

        # Draw map on image
        augmented_image = self.augmenter.render_segments(image_np, self.map_dict)

        # make new CompressedImage to publish
        augmented_image_msg = CompressedImage()
        augmented_image_msg.header.stamp = rospy.Time.now()
        augmented_image_msg.format = "jpeg"
        augmented_image_msg.data = np.array(cv2.imencode('.jpg', augmented_image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(augmented_image_msg)
        rospy.loginfo('Callback completed, publishing image')


if __name__ == '__main__':
    # create the node
    rospy.loginfo('Initialising node ...')
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    rospy.loginfo('Node initalised.')

    # keep node running, process callbacks sequentially
    rospy.spin()
