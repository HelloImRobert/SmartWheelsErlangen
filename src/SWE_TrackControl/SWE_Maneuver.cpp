
#include "SWE_Maneuver.h"

#define STEER_NEUTRAL 0
#define STEER_LEFT 30
#define STEER_RIGHT -30

#define CAR_LENGTH 480

// maneuver elements
// all headings are relative to maneuver start
// all distances are relative values, relative to start of this maneuver element

// go straight
#define GS_DIST1 900

//turn left
#define TL_HEAD1 (tFloat32)((  -10.0  /3.141592)*180)
#define TL_DIST1 400
#define TL_HEAD2 (tFloat32)((  80.0  /3.141592)*180)

//turn right
#define TR_HEAD1 (tFloat32)((  -10.0  /3.141592)*180)


//along park, leave left
#define AL_DIST1 50
#define AL_HEAD1 (tFloat32)((  25.0  /3.141592)*180)
#define AL_HEAD2 (tFloat32)((  25.0  /3.141592)*180)

//along park, leave right


//cross park, leave left


//cross park, leave right




SWE_Maneuver::SWE_Maneuver(): m_state(0), m_currentManeuver(NO_MANEUVER)
{
}

SWE_Maneuver::~SWE_Maneuver(){}


tResult SWE_Maneuver::Start(maneuvers maneuver, tFloat32 heading, tInt32 distanceSum, tFloat32 stopLineDistance)
{
    tInt32 returnvalue = 0;

    m_currentManeuver = maneuver;
    m_state = 1;
    m_headingStart = heading;
    m_distanceStart = distanceSum;

    returnvalue = CalcStep(heading, distanceSum);

    return returnvalue;
}

tBool SWE_Maneuver::IsFinished()
{
    return (m_state == 0) ? true : false;
}

tResult SWE_Maneuver::CalcStep(tFloat32 heading, tInt32 distanceSum)
{

    switch(m_currentManeuver)
    {
    case TC_TURN_LEFT:
        m_state = StateMachine_TL(heading, distanceSum, m_state);
        break;
    case TC_STOP_AT_STOPLINE:
        m_state = StateMachine_STOPLINE(heading, distanceSum, m_state, -1);
        break;
    case TC_TURN_RIGHT:
        m_state = StateMachine_TR(heading, distanceSum, m_state);
        break;
    case TC_GO_STRAIGHT:
        m_state = StateMachine_GS(heading, distanceSum, m_state);
        break;
    case PP_LEAVE_ALONG_LEFT:
        m_state = StateMachine_AL(heading, distanceSum, m_state);
        break;
    case PP_LEAVE_ALONG_RIGHT:
        m_state = StateMachine_AR(heading, distanceSum, m_state);
        break;
    case PP_LEAVE_CROSS_LEFT:
        m_state = StateMachine_CL(heading, distanceSum, m_state);
        break;
    case PP_LEAVE_CROSS_RIGHT:
        m_state = StateMachine_CR(heading, distanceSum, m_state);
        break;
    default: // NO_MANEUVER
        m_state = 0;
        m_currentManeuver = NO_MANEUVER;
        m_steeringAngleOut = 0;
        m_gearOut = 0;
        RETURN_ERROR(1);
    }
    RETURN_NOERROR;
}

tResult SWE_Maneuver::reset()
{

    m_state = 0;
    m_currentManeuver = NO_MANEUVER;

    RETURN_NOERROR;
}

tInt32 SWE_Maneuver::StateMachine_STOPLINE(tFloat32 heading, tInt32 distanceSum, tInt32 state, tFloat32 stopLineDistance)//TODO
{

    static tInt32 startDistance;
    static tFloat32 startStopLineDistance;

    switch(state)
    {
    case 1:
        startDistance = distanceSum;
        state++;
        break;
    case 2:
        state++;
        break;
    default:
        state = 0;
    }

    m_steeringAngleOut = 0;

    return state;
}


tInt32 SWE_Maneuver::StateMachine_GS(tFloat32 heading, tInt32 distanceSum, tInt32 state) //TODO
{

    static tInt32 startDistance;
    //static tInt32 startHeading;

    switch(state)
    {
    case 1:
        startDistance = distanceSum;
        //startHeading = heading;

        m_steeringAngleOut = 0;
        m_gearOut = 1;

        state++;
        break;
    case 2:
        if ((distanceSum - startDistance) >= GS_DIST1)
            state = 0;
        break;
    default: //finished
        startDistance = 0;
        //startHeading = heading;
        state = 0;
        break;
    }

    return state;
}

tInt32 SWE_Maneuver::StateMachine_TL(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    static tInt32 startDistance;

    switch(state)
    {
    case 1:
        startDistance = distanceSum;
        m_steeringAngleOut = 0;
        m_gearOut = 1;

        state++;
        break;
    case 2:
        if ((distanceSum - startDistance) >= GS_DIST1)
            state = 0;

        break;
    default: //finished
        startDistance = 0;
        state = 0;
        break;
    }

    return state;
}

tInt32 SWE_Maneuver::StateMachine_TR(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    return state;
}

tInt32 SWE_Maneuver::StateMachine_AL(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    state = 0;
    return state;
}

tInt32 SWE_Maneuver::StateMachine_AR(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    state = 0;
    return state;
}

tInt32 SWE_Maneuver::StateMachine_CL(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    state = 0;
    return state;
}

tInt32 SWE_Maneuver::StateMachine_CR(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    state = 0;
    return state;
}
