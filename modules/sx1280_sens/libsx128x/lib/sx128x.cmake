# Sx1280 Library Definition

# Allow build to reference header files from this directory
include_directories(${CMAKE_CURRENT_LIST_DIR})

# Set source file for compilation
# You can add files to be compiled here
set(SX128x_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/sx1280.c
    ${CMAKE_CURRENT_LIST_DIR}/sx1280-hal.c
    ${CMAKE_CURRENT_LIST_DIR}/sx1280-radio.c
)

# Build Library
add_library(sx128x ${SX128x_SOURCES})

# Add files to documentation generation (if used)
set(DOC_FILES ${DOC_FILES} ${CMAKE_CURRENT_LIST_DIR})
