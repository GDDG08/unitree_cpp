cmake_minimum_required(VERSION 3.15)
project(${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Unitree SDK2 backend.
#
# Default: link the installed unitree_sdk2 + CycloneDDS (real robot).
#
# TEST ONLY: pass -DUSE_DUMMY_SDK=ON to build against the bundled header-only
# dummy SDK (dummy_sdk/), which needs no hardware or DDS and feeds simulated
# data. This is opt-in on purpose — a normal install without the SDK fails
# loudly rather than silently producing a non-functional module. See README.
# ---------------------------------------------------------------------------
option(USE_DUMMY_SDK "TEST ONLY: build against the bundled dummy unitree_sdk2 (no hardware/DDS)" OFF)

if(USE_DUMMY_SDK)
  message(WARNING "unitree_cpp: building against the bundled DUMMY SDK "
                  "(TEST ONLY — no hardware, simulated data).")
  set(SDK_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/dummy_sdk/include)
  set(SDK_LIBRARIES "")
else()
  find_library(UNITREE_SDK2_LIB NAMES unitree_sdk2 PATHS /usr/local/lib /usr/lib)
  find_path(UNITREE_SDK2_INCLUDE NAMES unitree/robot/channel/channel_factory.hpp PATHS /usr/local/include /usr/include)
  find_library(DDSC_LIB NAMES ddsc PATHS /usr/local/lib /usr/lib)
  find_library(DDSCXX_LIB NAMES ddscxx PATHS /usr/local/lib /usr/lib)
  find_path(DDSCXX_ROOT_INCLUDE dds/dds.hpp PATHS /usr/local/include/ddscxx /usr/include)

  if(NOT UNITREE_SDK2_LIB OR NOT UNITREE_SDK2_INCLUDE)
    message(FATAL_ERROR "Cannot find installed unitree_sdk2 library or headers in /usr/local. "
                        "For a hardware-free test build, pass -DUSE_DUMMY_SDK=ON.")
  endif()

  if(NOT DDSC_LIB OR NOT DDSCXX_LIB OR NOT DDSCXX_ROOT_INCLUDE)
    message(FATAL_ERROR "Cannot find ddscxx or headers")
  endif()

  message(STATUS "Found unitree_sdk2 lib: ${UNITREE_SDK2_LIB}")
  message(STATUS "Found unitree_sdk2 include: ${UNITREE_SDK2_INCLUDE}")
  message(STATUS "Found ddsc lib: ${DDSC_LIB}")
  message(STATUS "Found ddscxx lib: ${DDSCXX_LIB}")
  message(STATUS "Found ddscxx root include: ${DDSCXX_ROOT_INCLUDE}")

  # `dl` is provided by libSystem on Apple, so only link it on other platforms.
  set(SDK_INCLUDE_DIRS ${UNITREE_SDK2_INCLUDE} ${DDSCXX_ROOT_INCLUDE})
  set(SDK_LIBRARIES ${UNITREE_SDK2_LIB} ${DDSC_LIB} ${DDSCXX_LIB})
  if(NOT APPLE)
    list(APPEND SDK_LIBRARIES dl)
  endif()
endif()

# Add a library using FindPython's tooling (pybind11 also provides a helper like
# this)
python_add_library(unitree_cpp MODULE
  src/py_binding.cpp 
  src/unitree_controller.cpp
  WITH_SOABI
)


target_include_directories(unitree_cpp PRIVATE
  ${SDK_INCLUDE_DIRS}
)

target_link_libraries(unitree_cpp PRIVATE
  pybind11::headers
  ${SDK_LIBRARIES}
  Threads::Threads
)


# This is passing in the version as a define just as an example
target_compile_definitions(unitree_cpp PRIVATE VERSION_INFO=${PROJECT_VERSION})

# Let the Python layer warn loudly that this is a simulated, test-only build.
if(USE_DUMMY_SDK)
  target_compile_definitions(unitree_cpp PRIVATE USE_DUMMY_SDK)
endif()

# The install directory is the output (wheel) directory
install(TARGETS unitree_cpp DESTINATION unitree_cpp)


set_target_properties(unitree_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN:/usr/local/lib"
)

# ======================
# Debug C++ executable
# ======================
add_executable(debugcpp
  src/unitree_controller.cpp
)

target_include_directories(debugcpp PRIVATE
  ${SDK_INCLUDE_DIRS}
)

target_link_libraries(debugcpp PRIVATE
  ${SDK_LIBRARIES}
  Threads::Threads
)

target_compile_options(debugcpp PRIVATE
  -g -O0
)

set_target_properties(debugcpp PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN:/usr/local/lib"
)