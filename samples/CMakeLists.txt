ADD_EXECUTABLE(kdtree0 kdtree0.cpp)
ADD_EXECUTABLE(octree0 octree0.cpp)
INCLUDE_DIRECTORIES(../ /usr/local/include /usr/local/include/eigen3)
TARGET_LINK_LIBRARIES(octree0 mibase)


