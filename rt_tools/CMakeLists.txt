cmake_minimum_required(VERSION 3.8)

project(rt_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##########################
##  Find  dependences  ##
##########################
find_package(ament_cmake REQUIRED)
include(FetchContent)
FetchContent_Declare(
    cactus_rt
    GIT_REPOSITORY https://github.com/cactusdynamics/cactus-rt.git
    GIT_TAG        cce7512
)
FetchContent_MakeAvailable(cactus_rt)

###########
## Build ##
###########

set(library_name ${PROJECT_NAME}_core)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# Tracing Library
add_library(${library_name}
    src/tracing_rt.cpp)

target_include_directories(${library_name} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${library_name}
    PUBLIC cactus_rt
)

#############
## Install ##
#############

install(TARGETS ${library_name}
    EXPORT ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_package()

