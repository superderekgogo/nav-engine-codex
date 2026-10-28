cmake_minimum_required(VERSION 3.24)
project(ins_rtk_nav VERSION 0.1.0 LANGUAGES C CXX)

set(NAVCORE_EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../eigen" CACHE PATH "Path to the existing Eigen source tree")

if(NOT EXISTS "${NAVCORE_EIGEN_DIR}/Eigen/Core")
  message(FATAL_ERROR "Eigen headers not found: ${NAVCORE_EIGEN_DIR}/Eigen/Core")
endif()

add_library(nav_core STATIC
  src/core/json.cpp
  src/core/config_loader.cpp
  src/navigation/state_estimator.cpp
)
target_include_directories(nav_core PUBLIC include)
target_include_directories(nav_core SYSTEM PUBLIC "${NAVCORE_EIGEN_DIR}")
target_compile_features(nav_core PUBLIC cxx_std_17)

add_library(navcore
  src/core/data_decode.cpp
  src/core/module_manager.cpp
  src/navigation/strapdown_mechanizer.cpp
  src/navigation/measurement_models.cpp
  src/navigation/gnss_preprocessor.cpp
  src/modules/dataset_modules.cpp
  src/modules/measurement_modules.cpp
  src/modules/gnss_preprocessor_module.cpp
  src/modules/ins_navigator.cpp
  src/modules/filter_manager_module.cpp
  src/modules/estimation_module.cpp
  src/modules/constraint_modules.cpp
  src/modules/runtime_modules.cpp
  src/adapters/rtklib/rtklib_adapter.cpp
  src/modules/rtk_solution_module.cpp
)
set(NAVCORE_RTKLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../RTKLIB-master/src" CACHE PATH "Path to the RTKLIB source directory")
if(NOT EXISTS "${NAVCORE_RTKLIB_DIR}/rtklib.h")
  message(FATAL_ERROR "RTKLIB headers not found: ${NAVCORE_RTKLIB_DIR}/rtklib.h")
endif()
add_library(rtklib_core STATIC
  "${NAVCORE_RTKLIB_DIR}/rtkcmn.c"
  "${NAVCORE_RTKLIB_DIR}/rtkpos.c"
  "${NAVCORE_RTKLIB_DIR}/geoid.c"
  "${NAVCORE_RTKLIB_DIR}/solution.c"
  "${NAVCORE_RTKLIB_DIR}/lambda.c"
  "${NAVCORE_RTKLIB_DIR}/sbas.c"
  "${NAVCORE_RTKLIB_DIR}/preceph.c"
  "${NAVCORE_RTKLIB_DIR}/pntpos.c"
  "${NAVCORE_RTKLIB_DIR}/ppp.c"
  "${NAVCORE_RTKLIB_DIR}/ppp_ar.c"
  "${NAVCORE_RTKLIB_DIR}/ephemeris.c"
  "${NAVCORE_RTKLIB_DIR}/rinex.c"
  "${NAVCORE_RTKLIB_DIR}/ionex.c"
  "${NAVCORE_RTKLIB_DIR}/qzslex.c"
  "${NAVCORE_RTKLIB_DIR}/rtcm3.c"
  "${NAVCORE_RTKLIB_DIR}/rtcm2.c"
  "${NAVCORE_RTKLIB_DIR}/rtcm3e.c"
  "${NAVCORE_RTKLIB_DIR}/rtcm.c"
  src/adapters/rtklib/wide_lane_narrow_lane.cpp
)
target_include_directories(rtklib_core PUBLIC "${NAVCORE_RTKLIB_DIR}")
target_include_directories(rtklib_core PRIVATE "${NAVCORE_EIGEN_DIR}")
target_compile_definitions(rtklib_core PRIVATE ENAGLO ENAQZS ENACMP EXTGSI)
target_compile_features(rtklib_core PRIVATE cxx_std_17)
target_link_libraries(navcore PUBLIC rtklib_core nav_core)
if(WIN32)
  target_link_libraries(navcore PUBLIC winmm)
endif()
target_include_directories(navcore PUBLIC include)
target_include_directories(navcore SYSTEM PUBLIC "${NAVCORE_EIGEN_DIR}")
target_include_directories(navcore PRIVATE "${NAVCORE_RTKLIB_DIR}")
target_compile_features(navcore PUBLIC cxx_std_17)
target_compile_definitions(navcore PRIVATE ENAGLO ENAQZS ENACMP)

# GMIS is a distinct full-state navigation library. It depends only on the
# domain-agnostic estimator and Eigen, never on the TMIS mechanization target.
add_library(nav_gmis STATIC
  src/gmis/gmis_lc_config.cpp
  src/gmis/gmis_lc_models.cpp
  src/gmis/gmis_lc_estimation.cpp
)
target_include_directories(nav_gmis PUBLIC include)
target_include_directories(nav_gmis SYSTEM PUBLIC "${NAVCORE_EIGEN_DIR}")
target_compile_features(nav_gmis PUBLIC cxx_std_17)
target_link_libraries(nav_gmis PUBLIC nav_core)

add_executable(gmis_lc_math_validation tools/gmis_lc_math_validation.cpp)
target_link_libraries(gmis_lc_math_validation PRIVATE nav_gmis)
add_executable(gmis_lc_app src/gmis/gmis_lc_app.cpp)
target_link_libraries(gmis_lc_app PRIVATE nav_gmis)
include(CTest)
if(BUILD_TESTING)
  add_test(NAME gmis_lc_math_validation
    COMMAND gmis_lc_math_validation "${CMAKE_CURRENT_SOURCE_DIR}/config/gmis/gmis_lc.json")
endif()

add_executable(ins_rtk_app src/main.cpp)
target_link_libraries(ins_rtk_app PRIVATE navcore)

if(MSVC)
  target_compile_options(nav_core PRIVATE /W4 /permissive-)
  target_compile_options(nav_gmis PRIVATE /W4 /permissive-)
  target_compile_options(gmis_lc_math_validation PRIVATE /W4 /permissive-)
  target_compile_options(gmis_lc_app PRIVATE /W4 /permissive-)
  target_compile_options(navcore PRIVATE /W4 /permissive-)
  target_compile_options(ins_rtk_app PRIVATE /W4 /permissive-)
else()
  target_compile_options(nav_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(nav_gmis PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(gmis_lc_math_validation PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(gmis_lc_app PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(navcore PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(ins_rtk_app PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ins_rtk_app gmis_lc_app RUNTIME DESTINATION bin)
