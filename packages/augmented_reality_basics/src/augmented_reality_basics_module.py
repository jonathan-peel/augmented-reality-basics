#!/usr/bin/env python3

import numpy as np
import cv2
import rospy

class Augmenter():
    def __init__(self):
        rospy.loginfo('Augmenter class initialised.')


    def process_image(self):
        pass

    def ground2pixel(self):
        pass

    def render_segments(self, image_data, map_dict):
        """ Renders the segments from the map file onto the image.
            Inputs: image_data from the subscribed topic
        """

        np_arr = np.fromstring(image_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # OpenCV < 3
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        # draw on the image
        image_width = image_np.shape[1]
        image_height = image_np.shape[0]
        segments = map_dict['segments'] # a list of segments dicts
        points = map_dict['points'] # dict of points

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

        return image_np


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
