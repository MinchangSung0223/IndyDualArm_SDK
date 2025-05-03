###########################################################################
#  top-level CMakeLists.txt  —  IndyDualArm SDK
###########################################################################

cmake_minimum_required(VERSION 3.15)
project(IndyDualArm LANGUAGES C CXX)

# ───── 사용자 설정 ────────────────────────────────────────────────────
option(BUILD_SIM_EXE      "Build Simulink test executables" OFF)
set(MATLAB_ROOT "/usr/local/MATLAB/R2024b" CACHE PATH "MATLAB root dir")
# 설치 시 경로(prefix)는 cmake -DCMAKE_INSTALL_PREFIX=<dir> 로 지정가능
# ─────────────────────────────────────────────────────────────────────

# ▸ 컴파일러 공통 설정
set(CMAKE_C_STANDARD   99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)           # 모든 타겟에 -fPIC

# ▸ Eigen (header-only)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)       # apt install libeigen3-dev
# ─────────────────────────────────────────────────────────────────────

# ▸ codegen 라이브러리 포함
add_subdirectory(codegen/FD_grt_rtw)
add_subdirectory(codegen/FK_grt_rtw)
add_subdirectory(codegen/ID_grt_rtw)
add_subdirectory(codegen/JSTraj_grt_rtw)
add_subdirectory(codegen/TSTraj_grt_rtw)
add_subdirectory(codegen/HinfController_grt_rtw)
add_subdirectory(codegen/TaskSpaceController_grt_rtw)
add_subdirectory(codegen/FD_nom_grt_rtw)
add_subdirectory(codegen/ID_nom_grt_rtw)

# (각 하위 폴더는 STATIC 라이브러리만 내놓도록 add_executable 래핑됨)

# ▸ IndyDualArm 공유 라이브러리
add_library(IndyDualArm SHARED
    src/IndyDualArm.cpp
)
target_include_directories(IndyDualArm
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>              # 설치 후 경로
        ${Eigen3_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/FD_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/FD_nom_grt_rtw 
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/FK_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/ID_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/ID_nom_grt_rtw
 
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/JSTraj_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/TSTraj_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/HinfController_grt_rtw
        ${CMAKE_CURRENT_SOURCE_DIR}/codegen/TaskSpaceController_grt_rtw

        ${MATLAB_ROOT}/extern/include
        ${MATLAB_ROOT}/simulink/include
        ${MATLAB_ROOT}/rtw/c/src
        ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
        ${MATLAB_ROOT}/extern/physmod/glnxa64/mc/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pd/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_log/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_math/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/pm_st/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_discr/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/sm_ssci/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_core/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_dae/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_ds/include
        ${MATLAB_ROOT}/extern/physmod/glnxa64/ssc_sli/include
        ${MATLAB_ROOT}/extern/include/shared_robotics
        ${MATLAB_ROOT}/toolbox/shared/robotics/externalDependency/libccd/src
        ${MATLAB_ROOT}/toolbox/shared/robotics/externalDependency/libccd/src/ccd
        ${START_DIR}/indy7_dualArm_ForwardKinematics_grt_rtw
        ${MATLAB_ROOT}/extern/include
        ${MATLAB_ROOT}/simulink/include
        ${MATLAB_ROOT}/rtw/c/src
        ${MATLAB_ROOT}/rtw/c/src/ext_mode/common
        
)
target_link_libraries(IndyDualArm
    PUBLIC Eigen3::Eigen                          # header-only, INTERFACE link
    PRIVATE
        ID
        FD
        FK
        JSTraj
    TSTraj
    HinfController
    TaskSpaceController
    FD_nom
    ID_nom
)

set_target_properties(IndyDualArm PROPERTIES
    OUTPUT_NAME "IndyDualArm"
    SOVERSION   1
)
target_link_options(IndyDualArm
    PRIVATE "LINKER:-z,muldefs")            # GNU ld / gold / lld
# 또는
# PRIVATE "LINKER:--allow-multiple-definition")

# ▸ 설치 규칙 (SDK 배포용)
install(TARGETS IndyDualArm
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)    # 만약 STATIC 선택 시
install(DIRECTORY include/          DESTINATION include)

# ▸ 빌드 완료 메시지
message(STATUS "=== IndyDualArm SDK configured ===")
message(STATUS "  MATLAB_ROOT   : ${MATLAB_ROOT}")
message(STATUS "  BUILD_SIM_EXE : ${BUILD_SIM_EXE}")

