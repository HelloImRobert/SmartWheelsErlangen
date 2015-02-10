set(FILTER_NAME SWE_TrackControl)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    SWE_TrackControl.h
    SWE_TrackControl.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
