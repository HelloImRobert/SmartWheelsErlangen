set(FILTER_NAME SWE_QRshield)

# include directories
include_directories(${OpenCV_INCLUDE_DIR}) 
 include_directories(${ZBAR_INCLUDE_DIR})

# declare our filter
adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

   SWE_QRshield.h
   SWE_QRshield.cpp

)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
target_link_libraries(${FILTER_NAME} ${ZBAR_LIBS})

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
