cmake_minimum_required(VERSION 3.20)
project(ironforge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(IFG_SANITIZE "" CACHE STRING "Sanitizers: 'address,undefined' or 'thread' (empty = none)")

add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wshadow)
if(IFG_SANITIZE)
  add_compile_options(-fsanitize=${IFG_SANITIZE} -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=${IFG_SANITIZE})
endif()

find_package(Threads REQUIRED)

add_library(ironforge
  src/math/math.cpp
  src/memory/allocators.cpp
  src/spatial/bvh.cpp
  src/spatial/octree.cpp
  src/scene/node.cpp
  src/jobs/job_system.cpp
  src/render/image.cpp
  src/render/rasterizer.cpp
  src/render/mesh.cpp
  src/script/lexer.cpp
  src/script/compiler.cpp
  src/script/vm.cpp
  src/script/value.cpp
  src/script/bindings.cpp
  src/physics/physics.cpp
  src/engine/demo_scene.cpp
)
target_include_directories(ironforge PUBLIC src)
target_link_libraries(ironforge PUBLIC Threads::Threads)

add_executable(ironforge_demo demo/main.cpp)
target_link_libraries(ironforge_demo PRIVATE ironforge)
target_compile_definitions(ironforge_demo PRIVATE IFG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(forge demo/forge_cli.cpp)
target_link_libraries(forge PRIVATE ironforge)

add_executable(ironforge_bench bench/bench.cpp)
target_link_libraries(ironforge_bench PRIVATE ironforge)
target_compile_definitions(ironforge_bench PRIVATE IFG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

enable_testing()
add_executable(ironforge_tests
  tests/test_main.cpp
  tests/test_math.cpp
  tests/test_memory.cpp
  tests/test_spatial.cpp
  tests/test_scene.cpp
  tests/test_jobs.cpp
  tests/test_render.cpp
  tests/test_script.cpp
  tests/test_physics.cpp
)
target_include_directories(ironforge_tests PRIVATE third_party)
target_link_libraries(ironforge_tests PRIVATE ironforge)
target_compile_definitions(ironforge_tests PRIVATE IFG_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
add_test(NAME unit COMMAND ironforge_tests)
