
#include "SWE_Maneuver.h"

#define STEER_NEUTRAL 0
#define STEER_LEFT 100
#define STEER_RIGHT -100

#define CAR_LENGTH 480

// maneuver elements
// all headings are relative to maneuver start
// all distances are relative values, relative to start of this maneuver element

//stopline
#define STOPLINE_DISTANCE_STOP 20
#define STOPLINE_DISTANCE_CRAWL 300
#define STOPLINE_DISTANCE_SLOW  1500
#define STOPLINE_WAIT_TIME 2000000 //in microseconds

// go straight
#define GS_DIST1 900

//turn left
#define TL_HEAD1 (tFloat32)((  -10.0  / 180 ) * 3.141592) //degree to rad
#define TL_HEAD2 (tFloat32)((  10.0  / 180 ) * 3.141592)
#define TL_DIST1 450
#define TL_HEAD3 (tFloat32)((  80.0   / 180 ) * 3.141592)

//turn right
#define TR_HEAD1 (tFloat32)((  -80.0   / 180 ) * 3.141592)


/*
//along park, leave left
#define AL_DIST1 50
#define AL_HEAD1 (tFloat32)((  25.0   / 180 ) * 3.141592)
#define AL_HEAD2 (tFloat32)((  180.0   / 180 ) * 3.141592)

//along park, leave right


//cross park, leave left


//cross park, leave right

*/


SWE_Maneuver::SWE_Maneuver(ucom::cObjectPtr<IReferenceClock> &clock): m_state(0), m_currentManeuver(NO_MANEUVER), m_steeringAngleOut(STEER_NEUTRAL), m_gearOut(0), _clock(clock)
{
}

SWE_Maneuver::~SWE_Maneuver(){}


tResult SWE_Maneuver::Start(maneuvers maneuver, tFloat32 headingSum, tInt32 distanceSum, tFloat32 stopLineXLeft, tFloat32 stopLineXRight)
{
    tInt32 returnvalue = 0;
    tFloat stopLineDistance;

    stopLineDistance = ((stopLineXLeft + stopLineXRight) / 2.0) - CAR_LENGTH;

    if(stopLineDistance < 0)
        stopLineDistance = 0;

    m_currentManeuver = maneuver;
    m_state = 1; //start

    returnvalue = CalcStep_p(headingSum, distanceSum, stopLineDistance);

    return returnvalue;
}

tResult SWE_Maneuver::CalcStep(tFloat32 heading, tInt32 distanceSum)
{

    CalcStep_p(heading, distanceSum, 0); //stopline distance not needed except for state = 1

    RETURN_NOERROR;
}

tBool SWE_Maneuver::IsFinished()
{
    return (m_state == 0) ? true : false;
}

tResult SWE_Maneuver::CalcStep_p(tFloat32 heading, tInt32 distanceSum, tFloat32 stopLineDistance)
{

    switch(m_currentManeuver)
    {
    case TC_TURN_LEFT:
        m_state = StateMachine_TL(heading, distanceSum, m_state);
        break;
    case TC_STOP_AT_STOPLINE:
        m_state = StateMachine_STOPLINE(distanceSum, m_state, stopLineDistance);
        break;
    case TC_TURN_RIGHT:
        m_state = StateMachine_TR(heading, distanceSum, m_state);
        break;
    case TC_GO_STRAIGHT:
        m_state = StateMachine_GS(distanceSum, m_state);
        break;
        /*
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
        break; */
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
    m_steeringAngleOut = 0;
    m_gearOut = 0;

    RETURN_NOERROR;
}

tInt32 SWE_Maneuver::StateMachine_STOPLINE(tInt32 distanceSum, tInt32 state, tFloat32 stopLineDistance)
{

    static tInt32 startDistance;
    static tFloat32 startStopLineDistance = 0;
    static tTimeStamp startTime;

    //OPTIONAL: do not change gear
    m_gearOut = 3;
    m_steeringAngleOut = STEER_NEUTRAL;

    switch(state)
    {
    case 1:
        startDistance = distanceSum;
        startStopLineDistance = stopLineDistance;
        state++;
    case 2:
        if((startStopLineDistance - (distanceSum - startDistance)) <= STOPLINE_DISTANCE_SLOW) //slow down if distances reached
        {
            if((startStopLineDistance - (distanceSum - startDistance)) <= STOPLINE_DISTANCE_CRAWL)
            {
                if((startStopLineDistance - (distanceSum - startDistance)) <= STOPLINE_DISTANCE_STOP)
                {
                    m_gearOut = 0;
                    startTime = (_clock != NULL) ? _clock->GetTime () : cSystem::GetTime(); // in microseconds
                    state++;
                }
                else
                    m_gearOut = 1;
            }
            else
                m_gearOut = 2;
        }

        break;
    case 3:
        if( ((_clock != NULL) ? _clock->GetTime () : cSystem::GetTime()) - startTime >= STOPLINE_WAIT_TIME) //wait a little to come to rest
        {
            state = 0;
        }
        break;
    default:
        state = 0;
    }

    return state;
}


tInt32 SWE_Maneuver::StateMachine_GS(tInt32 distanceSum, tInt32 state) //go straight
{

    static tInt32 startDistance;

    switch(state)
    {
    case 1:
        startDistance = distanceSum;

        m_steeringAngleOut = 0;
        m_gearOut = 2;

        state++;
    case 2:
        if ((distanceSum - startDistance) >= GS_DIST1) //distance reached?
            state = 0;
        break;
    default: //finished
        startDistance = 0;
        state = 0;
        break;
    }

    return state;
}

tInt32 SWE_Maneuver::StateMachine_TL(tFloat32 heading, tInt32 distanceSum, tInt32 state)
{
    static tInt32 startDistance = 0;
    static tFloat32 startHeading = 0;


    switch(state)
    {
    case 1:
        startDistance = distanceSum;
        startHeading = heading;
        m_steeringAngleOut = STEER_RIGHT;//go a little to the right first
        m_gearOut = 1;

        state++;
    case 2:
        if ((heading - startHeading) <= TL_HEAD1)
        {
            m_steeringAngleOut = STEER_LEFT; //... then back to the left
            state++;
        }

        break;

    case 3:
        if ((heading - startHeading) >= TL_HEAD2)
        {
            m_gearOut = 2;
            m_steeringAngleOut = STEER_NEUTRAL;//then go straight
            state++;
        }

        break;
    case 4:
        if ((distanceSum - startDistance) >= TL_DIST1)
        {
            m_gearOut = 1;
            m_steeringAngleOut = STEER_LEFT; // final turn...
            state++;
        }

        break;
    case 5:
        if ( (heading - startHeading) >= TL_HEAD3)
        {
            m_steeringAngleOut = STEER_NEUTRAL; // finished
            state = 0;
        }

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
    //static tInt32 startDistance = 0;
    static tFloat32 startHeading = 0;


    switch(state)
    {
    case 1:
        //startDistance = distanceSum;
        startHeading = heading;
        m_steeringAngleOut = STEER_RIGHT;
        m_gearOut = 1;

        state++;
    case 2:
        if ((heading - startHeading) <= TR_HEAD1)
        {
            m_gearOut = 2;
            m_steeringAngleOut = STEER_NEUTRAL;
            state = 0;
        }

        break;
    default: //finished
        //startDistance = 0;
        state = 0;
        break;
    }

    return state;
}

/*! not needed (yet)
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
*/


