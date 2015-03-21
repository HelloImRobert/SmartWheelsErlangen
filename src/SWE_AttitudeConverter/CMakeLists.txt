set(FILTER_NAME SWE_attitudeConverter)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                SWE_attitudeConverter.h
                SWE_attitudeConverter.cpp
)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
