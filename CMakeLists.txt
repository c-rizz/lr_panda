cmake_minimum_required(VERSION 2.8.3)
project(panda)

    # Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    panda_simulation)

catkin_package()