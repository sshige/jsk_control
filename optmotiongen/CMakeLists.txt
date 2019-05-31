cmake_minimum_required(VERSION 2.8.3)
project(optmotiongen)

add_compile_options(-std=c++11)

set(USE_OPENMP true CACHE BOOL "set true to use OpenMP.") # set variablees

find_package(catkin REQUIRED COMPONENTS cmake_modules rostest roscpp roslib)
find_package(Eigen REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
if(USE_OPENMP)
  find_package(OpenMP REQUIRED)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include(FindPkgConfig)
pkg_check_modules(CnoidBody choreonoid-body)
if(NOT CnoidBody_FOUND)
  # choreonoid-body module is introduced only after choreonoid-1.7
  # Before choreonoid-1.6, we use choreonoid-body-plugin instead of choreonoid-body
  pkg_check_modules(CnoidBody choreonoid-body-plugin REQUIRED)
endif()
pkg_check_modules(CnoidBodyPlugin choreonoid-body-plugin REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib
  )

# for debug
# message("[Eigen_LIBRARIES] ${Eigen_LIBRARIES}")
# message("[Eigen_INCLUDE_DIRS] ${Eigen_INCLUDE_DIRS}")
# message("[Boost_LIBRARIES] ${Boost_LIBRARIES}")
# message("[Boost_INCLUDE_DIRS] ${Boost_INCLUDE_DIRS}")
# message("[OpenMP_C_FLAGS] ${OpenMP_C_FLAGS}")
# message("[OpenMP_CXX_FLAGS] ${OpenMP_CXX_FLAGS}")
# message("[OpenMP_EXE_LINKER_FLAGS] ${OpenMP_EXE_LINKER_FLAGS}")
# message("[CnoidBody_LIBRARIES] ${CnoidBody_LIBRARIES}")
# message("[CnoidBody_LIBRARY_DIRS] ${CnoidBody_LIBRARY_DIRS}")
# message("[CnoidBody_INCLUDE_DIRS] ${CnoidBody_INCLUDE_DIRS}")
# message("[CnoidBodyPlugin_LIBRARIES] ${CnoidBodyPlugin_LIBRARIES}")
# message("[CnoidBodyPlugin_LIBRARY_DIRS] ${CnoidBodyPlugin_LIBRARY_DIRS}")
# message("[CnoidBodyPlugin_INCLUDE_DIRS] ${CnoidBodyPlugin_INCLUDE_DIRS}")
# message("[catkin_LIBRARIES] ${catkin_LIBRARIES}")
# message("[catkin_INCLUDE_DIRS] ${catkin_INCLUDE_DIRS}")
# message("[CATKIN_DEVEL_PREFIX] ${CATKIN_DEVEL_PREFIX}")
# message("[CATKIN_PACKAGE_LIB_DESTINATION] ${CATKIN_PACKAGE_LIB_DESTINATION}")

configure_file(${PROJECT_SOURCE_DIR}/env-hooks/99.choreonoid_ik_plugin.bash.in ${PROJECT_SOURCE_DIR}/env-hooks/99.choreonoid_ik_plugin.bash @ONLY)
catkin_add_env_hooks(99.choreonoid_ik_plugin SHELLS bash DIRECTORY ${PROJECT_SOURCE_DIR}/env-hooks)

include_directories(${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CnoidBody_INCLUDE_DIRS} ${CnoidBodyPlugin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${CnoidBody_LIBRARY_DIRS} ${CnoidBodyPlugin_LIBRARY_DIRS})

add_executable(sample_ik src/sample_ik.cpp)
target_link_libraries(sample_ik ${Boost_LIBRARIES} ${CnoidBody_LIBRARIES} ${catkin_LIBRARIES})

add_library(CnoidSampleIKPlugin SHARED src/sample_ik_plugin.cpp)
target_link_libraries(sample_ik ${Boost_LIBRARIES} ${CnoidBodyPlugin_LIBRARIES} ${catkin_LIBRARIES})

# SHARED means create shared library
add_library(SimpleBodyGenerator SHARED src/SimpleBodyGenerator.cpp)
target_link_libraries(SimpleBodyGenerator ${Boost_LIBRARIES} ${CnoidBody_LIBRARIES} ${catkin_LIBRARIES})

add_library(EusChoreonoidConverter SHARED src/EusChoreonoidConverter.cpp)
target_link_libraries(EusChoreonoidConverter SimpleBodyGenerator)
