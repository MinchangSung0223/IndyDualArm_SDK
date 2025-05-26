###########################################################################
# CMakeLists.txt generated for component log6
# Product type: STATIC library
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(log6)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR ../../.. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
#set(MATLAB_ROOT C:/Program\ Files/MATLAB/R2024b CACHE PATH "")
set(MATLAB_ROOT /usr/local/MATLAB/R2024b CACHE PATH "")
# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "log6".
add_library(log6 STATIC ${START_DIR}/codegen/lib/log6/rt_nonfinite.cpp
    ${START_DIR}/codegen/lib/log6/rtGetNaN.cpp
    ${START_DIR}/codegen/lib/log6/rtGetInf.cpp
    ${START_DIR}/codegen/lib/log6/log6_initialize.cpp
    ${START_DIR}/codegen/lib/log6/log6_terminate.cpp
    ${START_DIR}/codegen/lib/log6/log6.cpp
    ${START_DIR}/codegen/lib/log6/logm.cpp
    ${START_DIR}/codegen/lib/log6/schur.cpp
    ${START_DIR}/codegen/lib/log6/xnrm2.cpp
    ${START_DIR}/codegen/lib/log6/ordeig.cpp
    ${START_DIR}/codegen/lib/log6/xdlaev2.cpp
    ${START_DIR}/codegen/lib/log6/eigStandard.cpp
    ${START_DIR}/codegen/lib/log6/norm.cpp
    ${START_DIR}/codegen/lib/log6/sqrtObo.cpp
    ${START_DIR}/codegen/lib/log6/power.cpp
    ${START_DIR}/codegen/lib/log6/checkCondition.cpp
    ${START_DIR}/codegen/lib/log6/xzlarf.cpp
    ${START_DIR}/codegen/lib/log6/xhseqr.cpp
    ${START_DIR}/codegen/lib/log6/xzlarfg.cpp
    ${START_DIR}/codegen/lib/log6/xdlanv2.cpp
    ${START_DIR}/codegen/lib/log6/xdsterf.cpp
    ${START_DIR}/codegen/lib/log6/xzgehrd.cpp
    ${START_DIR}/codegen/lib/log6/log.cpp
    ${START_DIR}/codegen/lib/log6/sqrt.cpp
    ${START_DIR}/codegen/lib/log6/sqrtmTri.cpp
    ${START_DIR}/codegen/lib/log6/sylvesterRecursive.cpp
    ${START_DIR}/codegen/lib/log6/xgetrf.cpp
    ${START_DIR}/codegen/lib/log6/atanh.cpp
    ${START_DIR}/codegen/lib/log6/xzlascl.cpp
    ${START_DIR}/codegen/lib/log6/xdlahqr.cpp
    ${START_DIR}/codegen/lib/log6/sqrtm2by2.cpp
    ${START_DIR}/codegen/lib/log6/insertionsort.cpp
    ${START_DIR}/codegen/lib/log6/log6_rtwutil.cpp)

# Set properties for target "log6".
set_target_properties(log6 PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/log6/$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/log6/$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/log6/$<0:>")

# Specify language features required for target "log6".
target_compile_features(log6 PUBLIC cxx_std_11)

# Specify compiler preprocessor definitions for target "log6".
target_compile_definitions(log6 PRIVATE -DMODEL=log6)

# Specify include directories for target "log6".
target_include_directories(log6 PUBLIC $<BUILD_INTERFACE:${START_DIR}/codegen/lib/log6>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/codegen/lib/log6>
    $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)

# Specify library link dependencies for target "log6". CMake generator 
# expressions are used to create a CMakeLists.txt file that supports 
# multiple platforms with differently named system library dependencies.
target_link_libraries(log6 PUBLIC $<$<BOOL:${FOUND_LIBM}>:m>)


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET log6 POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created STATIC library: $<TARGET_FILE:log6>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(log6)
endif()
