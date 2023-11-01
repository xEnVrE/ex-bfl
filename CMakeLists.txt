cmake_minimum_required(VERSION 3.16)

# project name
project(ex-bfl-bb-tracking)

# dependencies
find_package(BayesFilters REQUIRED)

# add executable
add_executable(${PROJECT_NAME}
               include/bboxFilter.h
               include/motionModel.h
               include/measurementModel.h
               include/module.h
               src/main.cpp
               src/bboxFilter.cpp
               src/motionModel.cpp
               src/measurementModel.cpp
               src/module.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include/)
target_link_libraries(${PROJECT_NAME} PRIVATE BayesFilters::BayesFilters)
