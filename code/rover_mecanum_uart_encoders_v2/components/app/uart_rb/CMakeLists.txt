cmake_minimum_required(VERSION 3.22)

project(uart_rb)

add_library(${PROJECT_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/uart_rb.c
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc) # Public include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src) # Private include directories

target_link_libraries(${PROJECT_NAME} PUBLIC stm32cubemx lwrb) # Public dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE) # Private dependencies
