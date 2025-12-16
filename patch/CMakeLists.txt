file(GLOB PATCH_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.patch")

if(NOT PATCH_FILES)
    message(STATUS "No patch files found in ${CMAKE_CURRENT_SOURCE_DIR}")
    return()
endif()

find_program(PATCH_EXECUTABLE patch)
if(NOT PATCH_EXECUTABLE)
    message(FATAL_ERROR "Cannot find 'patch' executable required to apply patches")
endif()

foreach(PATCH_FILE IN LISTS PATCH_FILES)
    get_filename_component(PATCH_NAME "${PATCH_FILE}" NAME)

    message(STATUS "Checking patch: ${PATCH_NAME}")
    execute_process(
        COMMAND "${PATCH_EXECUTABLE}" -p0 -R --dry-run -i "${PATCH_FILE}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE PATCH_CHECK_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )

    if(PATCH_CHECK_RESULT EQUAL 0)
        message(STATUS "Patch already applied: ${PATCH_NAME}")
        continue()
    endif()

    message(STATUS "Applying source patch: ${PATCH_NAME}")

    execute_process(
        COMMAND "${PATCH_EXECUTABLE}" -p0 -N -i "${PATCH_FILE}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE PATCH_RESULT
    )

    if(NOT PATCH_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to apply patch ${PATCH_NAME}, exit code = ${PATCH_RESULT}")
    endif()
endforeach()
