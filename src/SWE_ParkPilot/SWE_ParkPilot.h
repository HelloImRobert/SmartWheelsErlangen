#ifndef _SWE_PARKPILOT_H_
#define _SWE_PARKPILOT_H_

#include "stdafx.h"
#include "math.h"


#define OID_ADTF_SWE_PARKPILOT "adtf.swe.parkpilot"

/*!
* Intersection Point Calculator
*/
class cSWE_ParkPilot : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SWE_PARKPILOT, "SWE ParkPilot", OBJCAT_DataFilter, "ParkPilot", 1, 0,0, "pre alpha version");

    cInputPin m_inputParkTrigger;
    cInputPin m_ObjectData;

    //cOutputPin m_oOutputManipulated;

public:
    cSWE_ParkPilot(const tChar* __info);
    virtual ~cSWE_ParkPilot();

protected: // overwrites cFilter
    tResult Init(tInitStage eStage, __exception = NULL);
    tResult Start(__exception = NULL);
    tResult Stop(__exception = NULL);
    tResult Shutdown(tInitStage eStage, __exception = NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);




private:

    tFloat32    m_IRFrontRightCur;
    tFloat32    m_IRRearRightCur;
    tFloat32    m_IRFrontRightOld;
    tFloat32    m_IRRearRightOld;
    tFloat32    m_distCur;
    tFloat32    m_distEntry;
    tFloat32    m_lastIRshort;

    tInt        m_parkTrigger;

    tBool       m_searchActive;
    tBool       m_entry;
    tBool       m_entrySaved;
    tBool       m_minDistReached;


    /*! struct containing the odometry input data */
    typedef struct
    {
        tFloat32	 angle;			// angle in radians
        tFloat32	 distance; 		// distance in mm
        tFloat32   	 heading;		// heading in radians
        tFloat32     distanceSum;   // sum of driven distance
    }odometryData;

    odometryData m_odometryData;



    /*! creates all the input Pins*/
    tResult CreateInputPins(__exception = NULL);
    /*! creates all the output Pins*/
    tResult CreateOutputPins(__exception = NULL);

    // Park functions
    tResult searchRoutineAlongside();
    tResult searchRoutineCross();

    tResult parkRoutineAlongside(tFloat32 lotSize, tFloat32 distStartPark);
    tResult parkRoutineCross(tFloat32 lotSize, tFloat32 distStartPark);


    // Media Descriptors
    /*! Coder Descriptors for the pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDesc;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescPointsIn;
};

#endif // _SWE_PARKPILOT_H_
