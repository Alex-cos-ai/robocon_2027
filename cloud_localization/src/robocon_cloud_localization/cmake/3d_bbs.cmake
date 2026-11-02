# Pinned upstream sources, built in this workspace without sudo/global installs.
set(BBS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/3d_bbs")
if(NOT EXISTS "${BBS_ROOT}/bbs3d/include/cpu_bbs3d/bbs3d.hpp")
  message(FATAL_ERROR "Missing 3D-BBS: git clone https://github.com/KOKIAOKI/3d_bbs.git ${BBS_ROOT}")
endif()
find_package(Eigen3 3.4 REQUIRED)
find_package(OpenMP REQUIRED)
add_library(robocon_bbs_cpu STATIC
  ${BBS_ROOT}/bbs3d/src/cpu_bbs3d/bbs3d.cpp
  ${BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  ${BBS_ROOT}/bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp)
target_include_directories(robocon_bbs_cpu SYSTEM PUBLIC ${BBS_ROOT}/bbs3d/include)
target_link_libraries(robocon_bbs_cpu PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX)
option(ROBOCON_BBS_CUDA "Build official CUDA 3D-BBS backend" ON)
if(ROBOCON_BBS_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    add_library(robocon_bbs_gpu STATIC
      ${BBS_ROOT}/bbs3d/src/gpu_bbs3d/bbs3d.cu
      ${BBS_ROOT}/bbs3d/src/gpu_bbs3d/calc_score.cu
      ${BBS_ROOT}/bbs3d/src/gpu_bbs3d/voxelmaps.cu
      ${BBS_ROOT}/bbs3d/src/gpu_bbs3d/voxelmaps_io.cu
      ${BBS_ROOT}/bbs3d/src/gpu_bbs3d/stream_manager/check_error.cu)
    set_target_properties(robocon_bbs_gpu PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
    target_include_directories(robocon_bbs_gpu SYSTEM PUBLIC ${BBS_ROOT}/bbs3d/include)
    target_compile_options(robocon_bbs_gpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
    target_link_libraries(robocon_bbs_gpu PUBLIC Eigen3::Eigen CUDA::cudart)
  else()
    message(WARNING "CUDA not found: bbs_cpu_gicp/grid_gicp available, bbs_gpu_gicp will report an error")
  endif()
endif()
