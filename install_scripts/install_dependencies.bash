#!/bin/bash

install_dependencies()
{
   sudo apt-get update
   sudo apt-get install -y \
        subversion \
        cmake \
        libopencv-dev \
	    ros-indigo-navigation
}

install_dependencies
