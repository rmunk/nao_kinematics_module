# Copyright (C) 2011 Aldebaran Robotics
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

# Define the name of the project
project(kinematics)

# This include enable you to use qibuild framework
find_package(qibuild)


# Create an option to make is possible compiling the module
# as a remote executable, or as a local shared library
option(KINEMATICS_IS_REMOTE
  "module is compiled as a remote module (ON or OFF)"
  ON)

# Create a list of source files
set(_srcs
  3rd_party/NAOKinematics/NAOKinematics.h
  3rd_party/NAOKinematics/NAOKinematics.cpp
  kinematics_module.cpp
  kinematics_module.h
  main.cpp
)

if(KINEMATICS_IS_REMOTE)
  # Add a compile flag because code changes a little bit
  # when we are compiling an executable
  # This will let you use #ifdef KINEMATICS_IS_REMOTE
  # in the C++ code
  add_definitions( " -DKINEMATICS_IS_REMOTE ")

  # Create an executable
  qi_create_bin(kinematics ${_srcs})
else()
  # Create a plugin, that is a shared library, and make
  # sure it is built in lib/naoqi, so that the naoqi executable
  # can find it later
  qi_create_lib(kinematics SHARED ${_srcs} SUBFOLDER naoqi)
endif()

# Tell CMake that kinematics depends on ALCOMMON and
# ALPROXIES.
# This will set the libraries to link kinematics with,
# the include paths, and so on
qi_use_lib(kinematics ALCOMMON ALPROXIES BOOST)


# Also create a simple executable capable of creating
# a proxy to the kinematics module
qi_create_bin(test_kinematics test_kinematics.cpp)
qi_use_lib(test_kinematics ALCOMMON ALPROXIES)
