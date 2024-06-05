cmake_minimum_required(VERSION 3.22)

project(app_main)

add_library(${PROJECT_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/app_main.c
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc) # Public include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src ${cjson_SOURCE_DIR}) # Private include directories

target_link_libraries(${PROJECT_NAME} PUBLIC) # Public dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE stm32cubemx lwpkt cjson uart_rb robot) # Private dependencies
