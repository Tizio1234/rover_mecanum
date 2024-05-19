cmake_minimum_required(VERSION 3.22)

set(SRCS
    ${CMAKE_CURRENT_LIST_DIR}/app_main.c
)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

project(app_main)
add_library(${PROJECT_NAME} STATIC)
target_sources(${PROJECT_NAME} PUBLIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
