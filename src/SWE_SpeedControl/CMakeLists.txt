set(FILTER_NAME SWE_SpeedControl)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp	 
                stdafx.h
                SpeedControl.h
                SpeedControl.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/Smart_Wheels) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
