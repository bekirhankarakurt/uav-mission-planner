cmake_minimum_required(VERSION 3.16)
project(uav_mission_planner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(UAV_BUILD_CLI "Build the uav-plan command line tool" ON)
option(UAV_BUILD_TESTS "Build the C++ unit tests" ON)
option(UAV_BUILD_PYTHON "Build the Python extension module" OFF)
option(UAV_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(UAV_SANITIZE "Enable AddressSanitizer and UBSan" OFF)

if(UAV_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
  add_link_options(-fsanitize=address,undefined)
endif()

function(uav_set_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(UAV_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(UAV_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

add_library(uavcore STATIC
  src/geo.cpp
  src/json.cpp
  src/terrain.cpp
  src/zones.cpp
  src/planner.cpp
  src/profile.cpp
  src/mission.cpp
  src/export.cpp
  src/render.cpp
  src/benchmark.cpp)
target_include_directories(uavcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(uavcore PUBLIC UAV_VERSION="${PROJECT_VERSION}")
set_target_properties(uavcore PROPERTIES POSITION_INDEPENDENT_CODE ON)
uav_set_warnings(uavcore)

if(UAV_BUILD_CLI)
  add_executable(uav-plan cli/main.cpp)
  target_link_libraries(uav-plan PRIVATE uavcore)
  uav_set_warnings(uav-plan)
  if(NOT SKBUILD)
    install(TARGETS uav-plan RUNTIME DESTINATION bin)
  endif()
endif()

if(UAV_BUILD_TESTS)
  enable_testing()
  foreach(name geo json terrain zones planners profile export benchmark)
    add_executable(test_${name} tests/test_${name}.cpp)
    target_link_libraries(test_${name} PRIVATE uavcore)
    uav_set_warnings(test_${name})
    add_test(NAME ${name} COMMAND test_${name})
  endforeach()
  if(UAV_BUILD_CLI)
    add_test(NAME cli_help COMMAND uav-plan --help)
    add_test(NAME cli_plan COMMAND uav-plan plan --synthetic 6 --seed 3 --planner thetastar --smooth)
    add_test(NAME cli_bench COMMAND uav-plan benchmark --maps 2 --size 5 --seed 1 --rrt-iterations 800)
  endif()
endif()

if(UAV_BUILD_PYTHON)
  include(FetchContent)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(pybind11)
  pybind11_add_module(_core python/bindings.cpp)
  target_link_libraries(_core PRIVATE uavcore)
  install(TARGETS _core LIBRARY DESTINATION uav_planner)
endif()
