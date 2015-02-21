set(FILTER_NAME SWE_KIControl)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    #SWE_KIControl.h
    #SWE_KIControl.cpp

)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
