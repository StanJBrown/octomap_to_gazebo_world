cmake_minimum_required(VERSION 2.8.3)
project(octomap_to_gazebo)
find_package(catkin REQUIRED
    move_base_msgs
    actionlib
)


#Prepare the catkin_package
catkin_package(
    CATKIN_DEPENDS move_base_msgs actionlib
)


# Install python scripts into the correct location
# catkin_python_setup()

catkin_install_python(PROGRAMS
    python_scripts/generate_world_and_contours.py
    python_scripts/createGazeboWorld.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)
