# Include the current directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})


# # Add helper include path
# set(UAVLINK_HELPER_PATH ../../helper)
# include_directories(${UAVLINK_HELPER_PATH})



pybind11_add_module(uavlink_py uavlink-dynamic-mcs_py.cc)
set_target_properties(uavlink_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(uavlink_py PRIVATE ${libuavlink})



set(NR_SOURCES_uav
    v2x-kpi.cc
    uavlink-nr-v2x-uav.cc
    uav-base-scheduler_nr.cc
    nr-sl-ue-mac-scheduler-dynamic-mcs.cc
    uavlink-dynamic-mcs.cc
)


build_lib_example(
        NAME uavlink-nr-v2x-uav
        SOURCE_FILES
            ${NR_SOURCES_uav}
        LIBRARIES_TO_LINK
            ${libuavlink}
            ${libflow-monitor}
            ${libnr} # Link the nr library
            ${libstats} # Optional, if needed for statistics
)

# Build Python interface along with C++ lib
add_dependencies(uavlink-nr-v2x-uav uavlink_py)
