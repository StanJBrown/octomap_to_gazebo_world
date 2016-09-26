#!/usr/bin/env python

import rospy
import cv2
import yaml
import os.path
import matplotlib.pyplot as plt
from createGazeboWorld import writeSvnHeaders
from createGazeboWorld import writeContourToWorldFile, closeWorldFile
from visvalingamWhyatt import VWSimplifier
import numpy as np

# def writeContourToDatfile(cnt, scale, f):
#     f.write("%i \n" % (len(cnt)))
#     for ii in xrange(len(cnt)):
#         x = cnt[ii][0][0]  # * scale
#         y = cnt[ii][0][1]  # * scale
#         f.write("%f %f \n" % (x, y))

def writeContourToDatfile(cnt, scale, f):
    f.write("%i \n" % (len(cnt)))
    for ii in xrange(len(cnt)):
        x = cnt[ii][0]  # * scale
        y = cnt[ii][1]  # * scale
        f.write("%f %f \n" % (x, y))


def getContourPnts(cnt):
    pnts = []
    for ii in xrange(len(cnt)):
        x = cnt[ii][0][0]  # * scale
        y = cnt[ii][0][1]  # * scale
        pnts.append([x,y])
    pnts = np.asarray(pnts, float)
    return pnts


def writeGazeboAndDatFiles(contours, scale, dat_file, gazebo_file):
    writeSvnHeaders(gazebo_file)
    wall_num = 0
    for i in xrange(len(contours)):
        # if i == 0:
        #     continue
        cnt = contours[i]
        # minlength = min(epsilon, 0.1*cv2.arcLength(cnt, True))
        # minlength = 0.01*cv2.arcLength(cnt, True)
        # apx = cv2.approxPolyDP(cnt, minlength, True)
        # if (cv2.arcLength(apx, True) < min_perimeter):
        #     continue
        pnts = getContourPnts(cnt)
        simplifier = VWSimplifier(pnts)
        numpnts = max(15, len(cnt) * 0.5)
        apx = simplifier.from_number(numpnts)
        if (len(apx) < 15):
            continue
        writeContourToDatfile(apx, scale, dat_file)
        wall_num = writeContourToWorldFile(apx, scale, gazebo_file, wall_num)


# todo: Make this take a .yaml file as an argument.
# Improve documentation
# pass contours using ros?
# ignore small contours?
# add tsp stuff
if __name__ == '__main__':
    occupancy_grid_name = rospy.get_param("occupancy_grid_name")
    occupancy_grid_yaml = occupancy_grid_name + '.yaml'

    print occupancy_grid_yaml
    with open(occupancy_grid_yaml, 'r') as f:
        configs = yaml.load(f)

    scale = configs["resolution"]
    map_name = configs["image"]
    occupancy_grid_file = os.path.dirname(occupancy_grid_yaml) + '/' + map_name

    output_contour_filename = rospy.get_param("contour_file")
    output_gazebo_filename = rospy.get_param("gazebo_world_file")

    # min_perimeter = 50
    # epsilon = 4
    # img = cv2.imread('FloorPlanImages/e5_rescaled2_inverted2.png')
    img = cv2.imread(occupancy_grid_file)
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 206, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(
        thresh,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE
    )

    contour_file = open(output_contour_filename, 'w+')
    gazebo_file = open(output_gazebo_filename, 'w+')

    writeGazeboAndDatFiles(contours, scale, contour_file, gazebo_file)
    closeWorldFile(gazebo_file)
    contour_file.close()


cv2.drawContours(thresh, contours, -1, (225, 225, 225), 3);
plt.imshow(thresh, interpolation="nearest")
plt.show()
