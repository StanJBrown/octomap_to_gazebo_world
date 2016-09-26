#!/usr/bin/env python

from string import Template
from numpy import arctan2, sqrt

def writeSvnHeaders(f):
    s = """<?xml version='1.0'?>
        <sdf version='1.4'>
            <world name='1_wall'>
                <light name='sun' type='directional'>
                    <cast_shadows>1</cast_shadows>
                    <pose>0 0 10 0 -0 0</pose>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.2 0.2 0.2 1</specular>
                    <attenuation>
                    <range>1000</range>
                    <constant>0.9</constant>
                    <linear>0.01</linear>
                    <quadratic>0.001</quadratic>
                    </attenuation>
                    <direction>-0.5 0.1 -0.9</direction>
                </light>
                <model name='ground_plane'>
                    <static>1</static>
                    <link name='link'>
                        <collision name='collision'>
                        <geometry>
                            <plane>
                                <normal>0 0 1</normal>
                                <size>1000 1000</size>
                            </plane>
                        </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>100</mu>
                                    <mu2>50</mu2>
                                </ode>
                            </friction>
                            <bounce/>
                                <contact>
                                    <ode/>
                                </contact>
                        </surface>
                        <max_contacts>10</max_contacts>
                        </collision>
                    <visual name='visual'>
                        <cast_shadows>0</cast_shadows>
                        <geometry>
                            <plane>
                                <normal>0 0 1</normal>
                                <size>10000 10000</size>
                            </plane>
                            </geometry>
                                <material>
                                <script>
                                        <uri>file://media/materials/scripts/gazebo.material</uri>
                                        <name>Gazebo/Grey</name>
                                        </script>
                                    </material>
                    </visual>
                    <velocity_decay>
                        <linear>0</linear>
                        <angular>0</angular>
                    </velocity_decay>
                    <self_collide>0</self_collide>
                        <kinematic>0</kinematic>
                        <gravity>1</gravity>
                    </link>
                </model>
                <physics type='ode'>
                    <max_step_size>0.01</max_step_size>
                    <real_time_factor>1</real_time_factor>
                    <real_time_update_rate>100</real_time_update_rate>
                    <gravity>0 0 -9.8</gravity>
                </physics>
            <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.7 0.7 0.7 1</background>
                <shadows>1</shadows>
            </scene>
    """
    f.write(s)


def midpoint(x1, x2):
    xout = x1 + (x2 - x1) / 2.0
    return xout


def getDirection(x1, x2, y1, y2):
    return arctan2((y2 - y1), (x2 - x1))


# def length(x1, x2):
#     return abs(x2 - x1)


def writeWall(point_1, point_2, i, scale, gazebo_file):
    s = Template("""<model name='Wall_$index'>
     <static>1</static>
    <link name='Wall_$index'>
    <collision name='Wall_Collision_$index'>
        <geometry>
          <box>
            <size>$length $width $height</size>
          </box>
        </geometry>
        <pose>0 0 1.25 0 -0 0</pose>
      </collision>
      <visual name='Wall_Visual_$index'>
        <pose>0 0 1.25 0 -0 0</pose>
        <geometry>
          <box>
            <size>$length $width $height</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
      <velocity_decay>
        <linear>0</linear>
        <angular>0</angular>
      </velocity_decay>
      <pose>$x_mid $y_mid 0 0 0 $direction</pose>
    </link>
     </model>""")

    x1 = point_1[0] * scale
    y1 = point_1[1] * scale

    x2 = point_2[0] * scale
    y2 = point_2[1] * scale

    mid_x = midpoint(x1, x2)
    mid_y = midpoint(y1, y2)

    cnt_direction = getDirection(x1, x2, y1, y2)
    cnt_length = sqrt((x1 - x2)**2 + (y1 - y2)**2)
    # print("x1 = $f, y1 = $f, y2= $f, x2 = %f", x1, y1, x2, y2)
    wall_string = s.substitute(index=i, length=cnt_length,
                               width=0.2, height=2.5,
                               x_mid=mid_x, y_mid=mid_y,
                               direction=cnt_direction)
    gazebo_file.write(wall_string)


def writeContourToWorldFile(contour, scale, gazebo_file, wall_num):
    for i in xrange(len(contour)):
        writeWall(contour[i-1], contour[i], wall_num, scale, gazebo_file)
        wall_num += 1
        # print len(contours)
        # print len(cnt)
        # for j in xrange(len(cnt)):
    return wall_num


def closeWorldFile(gazebo_file):
    s = """</world>
        </sdf>"""
    gazebo_file.write(s)
    gazebo_file.close()
