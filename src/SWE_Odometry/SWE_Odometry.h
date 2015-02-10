#ifndef _SWE_ODOMETRY_H_
#define _SWE_ODOMETRY_H_

#define OID_ADTF_SWE_ODOMETRY "adtf.aadc.SWE_odometry"


#include "math.h"
/*!
* Simple odometry based on bicycle model. The relative position and heading are updated whenever a new data sample arrives. Output of the accumulated position change in distance, angle and heading on trigger event (accumulated since last trigger).


----- Coordinate System -----

vector: *``` -> new car position = *
old car position (last trigger) = o

          ^ X
*         |
  `  alpha|
     `    |
        ` |
<---------o--------->
Y         |        -Y
          |
          |
          |
          V -X

*/
class SWE_Odometry : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_ODOMETRY, "SWE_Odometry", OBJCAT_DataFilter, "SWE_Odometry filter", 1, 0, 0, "Beta Version");

		/*!input pin for the steering angle */
		cInputPin m_oInputSteeringAngle;
		/*!input pin for the rpm of the left wheel */
		cInputPin m_oInputVelocityLeft;
		/*!input pin for the rpm of the right wheel */
		cInputPin m_oInputVelocityRight;
		/*!input pin for the rpm of the right wheel */
		cInputPin m_oInputTrigger;
		/*!output pin for the odometry data */
		cOutputPin m_oOutputOdometry;

    public:
        SWE_Odometry(const tChar* __info);
        virtual ~SWE_Odometry();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
    private:
	/*!gets the actual time as a tTimeStamp */
	tTimeStamp GetTime();
		
	/*! Coder Descriptor for the pins*/
	    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;

	/*! struct containing the odometry output data */
	typedef struct
	{
		tFloat32	 angle;			// angle in radians
		tFloat32	 distance; 		// distance in mm
		tFloat32   	 heading;		// heading in radians
	}odometryData;

	/*! Private member variables */
	tFloat32 m_steeringAngle;
	tFloat32 m_slippageAngle;
	tFloat32 m_velocityLeft;
	tFloat32 m_velocityRight;
	tFloat32 m_buffer;
	
	tTimeStamp m_currTimeStamp;
	tTimeStamp m_oldTimeStamp;
	tTimeStamp m_lastPinEvent;

	tFloat32 m_distanceX_sum;
	tFloat32 m_distanceY_sum;
	tFloat32 m_heading_sum;

	odometryData m_odometryData;

	/* SWE METHODS */

	/*! Helper method to send Media Sample */
    	tResult sendData(odometryData odometryData);


	/*! Odometry single step */
	void calcSingleOdometry(tTimeStamp timeIntervall);


	/*! Odometry output, reset*/
	void odometryOutput();
};



//*************************************************************************************************

#endif // _SWE_ODOMETRY_H_
