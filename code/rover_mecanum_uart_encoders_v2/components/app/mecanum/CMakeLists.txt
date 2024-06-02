cmake_minimum_required(VERSION 3.22)

project(mecanum)

add_library(${PROJECT_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/mecanum.c
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc) # Public include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src ${cjson_SOURCE_DIR}) # Private include directories

target_link_libraries(${PROJECT_NAME} PUBLIC stm32cubemx) # Public dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE) # Private dependencies
