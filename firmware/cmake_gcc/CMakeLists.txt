# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.25")

# Project definition
project(
	rc_module_trx
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_rc_module_trx target,
# which contains the content of the SLC project
include(rc_module_trx.cmake)

add_executable(rc_module_trx
    # Add additional sources here
)

target_include_directories(rc_module_trx PUBLIC
    # Add additional include paths here
)

target_compile_definitions(rc_module_trx PUBLIC
    # Add additional macros here
)

target_compile_options(rc_module_trx PUBLIC
    # Set additional compiler flags here
)

target_link_options(rc_module_trx PUBLIC
    # Set additional linker flags here
)

# Link with the content defined in the SLC project
target_link_libraries(rc_module_trx PRIVATE
    slc
)

# Include managed project content if available
include(rc_module_trx_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(rc_module_trx PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET rc_module_trx
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:rc_module_trx>" "$<TARGET_FILE_DIR:rc_module_trx>/$<TARGET_FILE_BASE_NAME:rc_module_trx>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:rc_module_trx>" "$<TARGET_FILE_DIR:rc_module_trx>/$<TARGET_FILE_BASE_NAME:rc_module_trx>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:rc_module_trx>" "$<TARGET_FILE_DIR:rc_module_trx>/$<TARGET_FILE_BASE_NAME:rc_module_trx>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET rc_module_trx
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()