#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
# from image_geometry import PinholeCameraModel

class Augmenter():
    def __init__(self, homography, camera_info):
        # homography matrix
        self.H = [homography[0:3], homography[3:6], homography[6:9]]
        rospy.loginfo(f'Homography matrix: {self.H}')
        self.Hinv = np.linalg.inv(self.H)
        rospy.loginfo(f'Inverse homography matrix: {self.Hinv}')

        self.camera_info = camera_info
        rospy.loginfo('Augmenter class initialised.')

        # For the image rectification
        # self.ci = camera_info
        # self.pcm = PinholeCameraModel()
        # self.pcm.fromCameraInfo(self.ci)
        # self._rectify_inited = False
        # self._distort_inited = False

        
    def _init_rectify_maps(self):
        W = self.pcm.width
        H = self.pcm.height
        mapx = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapy = np.ndarray(shape=(H, W, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm.K, self.pcm.D, self.pcm.R,
                                                 self.pcm.P, (W, H),
                                                 cv2.CV_32FC1, mapx, mapy)
        self.mapx = mapx
        self.mapy = mapy
        self._rectify_inited = True


    def process_image(self, cv_image_raw, interpolation=cv2.INTER_NEAREST):
        ''' Undistort an image.
            To be more precise, pass interpolation= cv2.INTER_CUBIC
        '''
        if not self._rectify_inited:
            self._init_rectify_maps()
        #
        #        inter = cv2.INTER_NEAREST  # 30 ms
        #         inter = cv2.INTER_CUBIC # 80 ms
        #         cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        cv_image_rectified = np.empty_like(cv_image_raw)
        res = cv2.remap(cv_image_raw, self.mapx, self.mapy, interpolation,
                        cv_image_rectified)
        return res


    def ground2pixel(self, point):
        """ Take in points in the 'axle' reference frame, and run them in the 
            camera reference frame.
        """
        try:
            assert (point[2] == 0)
        except AssertionError:
            rospy.logwarn('Trying to project a point that is not on the ground, assuming z=0 anyway.')

        ground_point = np.array([point[0], point[1], 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]
        image_point = image_point[0:2]
        image_point = [int(i) for i in image_point]

        return image_point


    def render_segments(self, image_np, map_dict):
        """ Renders the segments from the map file onto the image.
            Inputs: image_data from the subscribed topic
        """

        # draw on the image
        # image_width = image_np.shape[1]
        # image_height = image_np.shape[0]
        rospy.loginfo(f'map_dict: {map_dict}')
        points = map_dict['points'] # dict of points
        rospy.loginfo(f'Points in original frame: {points}')
        segments = map_dict['segments'] # a list of segments dicts
        rospy.loginfo(f'Segments: {segments}')

        # translate all points into image frame
        points_image = {}
        for point_name, point_data in points.items():
            # eg) point_name = 'TL', point_data = ['axle', [0.3, 0.09, 0]]
            frame = point_data[0] # eg 'axle'
            point_coords_axle = point_data[1] # eg [0.3, 0.09, 0]
            
            if (frame == 'axle'):
                # update original points dict with points in image frame
                point_coords_image = self.ground2pixel(point_coords_axle)
                points_image[point_name] = ['image', point_coords_image]

            elif (frame == 'image'):
                points_image[point_name] = point_data # do nothing
            
        rospy.loginfo(f'Points in image frame: {points_image}')

        # deconstruct map_dict into its points for each segment and plot it
        for segment in segments:
            segment_points_names = segment['points'] # eg ['TL', 'TR']

            segment_points_coords = [points_image[segment_points_names[0]][1], \
                points_image[segment_points_names[1]][1]]
            # segment_points_coords now eg [[0, 0], [0, 1]]

            segment_color = segment['color'] # eg 'red'

            augmented_image = self.draw_segment(image_np, segment_points_coords, segment_color)

        return augmented_image


    def draw_segment(self, image, points_list, color):
        """ Draws a segment (line) between two points only.
            points_list is a list of the points: [[x1, y1], [x2, y2]]
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
        new_image = cv2.line(image, (tuple(points_list[0])), tuple(points_list[1]), (b * 255, g * 255, r * 255), 5)

        return new_image

