set(FILTER_NAME SWE_LaneDetection)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    SWE_LaneDetection.h
    SWE_LaneDetection.cpp
    SWE_CatmullRomSpline.h
    SWE_CatmullRomSpline.cpp
    SWE_CrossingAnalyzer.h
    SWE_CrossingAnalyzer.cpp
    SWE_BlobDescriptor.h
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
