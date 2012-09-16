#=============================================================================#
# Author: QueezyTheGreat                                                      #
# Date:   16.09.2012                                                          #
#                                                                             #
# Description: Heptic Alarm Clock                                             #
#                                                                             #
#=============================================================================#
set(CMAKE_TOOLCHAIN_FILE arduino-cmake/cmake/ArduinoToolchain.cmake) # Arduino Toolchain


cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(heptic_alarm C CXX)

print_board_list()
print_programmer_list()

add_subdirectory(src)
