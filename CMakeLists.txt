cmake_minimum_required(VERSION 3.16.0)

# set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/components/esp-idf-lib/components)
list(APPEND EXTRA_COMPONENT_DIRS

    # needs to be set as REQUIRES in src/CMakeLists.txt
    # "${CMAKE_CURRENT_LIST_DIR}/components/x"
    "${CMAKE_CURRENT_LIST_DIR}/components/esp-idf-lib/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Hydro_ESP32)
