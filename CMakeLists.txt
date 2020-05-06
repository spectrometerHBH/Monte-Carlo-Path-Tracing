cmake_minimum_required(VERSION 3.15)
project(cg4)

set(CMAKE_CXX_STANDARD 14)

# specify which version and components you need
find_package(OGRE 1.11 REQUIRED COMPONENTS Bites RTShaderSystem)
# add the source files as usual
add_executable(cg4 main.cpp bvh.h brdf.h)
# this also sets the includes and pulls third party dependencies
target_link_libraries(cg4 OgreBites OgreRTShaderSystem)