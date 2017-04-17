cmake_minimum_required(VERSION 2.8)
project(test_rags)

#There are lots of scripts with cmake
#for finding external libraries. 
#see /usr/local/share/cmake-2.6/Modules/Find*.cmake for more examples
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

#add_executable(test_scratch test_scratch.cpp)
#target_link_libraries( test_scratch ${Boost_LIBRARIES} )

add_executable(test_path_comparisons main_Planner_Comparisons.cpp)
target_link_libraries( test_path_comparisons ${Boost_LIBRARIES} )

add_executable(test_ND_set_binaries main_Log_ND_Set.cpp)
target_link_libraries( test_ND_set_binaries ${Boost_LIBRARIES} )
