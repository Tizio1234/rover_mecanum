cmake_minimum_required(VERSION 3.22)

set(SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/uart_comm_rb.c
)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

project(uart_comm_rb)
add_library(${PROJECT_NAME} STATIC)
target_sources(${PROJECT_NAME} PUBLIC ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})
