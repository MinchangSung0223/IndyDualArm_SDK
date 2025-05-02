cmake_minimum_required(VERSION 3.0)
project(dualArm_example)


include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)
set(BUILD_SHARED_LIBS FALSE)
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()



find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)

add_executable(test_dualarm main.cpp ${RobotSimulator_SRCS} Robot.cpp)
target_link_libraries(test_dualarm PRIVATE 
IndyDualArm 
Eigen3::Eigen
 ${BULLET_LIBRARIES} 
  )

target_include_directories(test_dualarm PUBLIC 
${BULLET_INCLUDE_DIR}
${RobotSimulator_INCLUDE_DIR}	)
target_compile_definitions(test_dualarm PUBLIC -DBT_USE_DOUBLE_PRECISION )
install(TARGETS test_dualarm
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
