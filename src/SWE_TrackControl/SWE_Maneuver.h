#include "stdafx.h"


enum maneuvers
{
    NO_MANEUVER,
    TC_STOP_AT_STOPLINE,
    TC_GO_STRAIGHT,
    TC_TURN_LEFT,
    TC_TURN_RIGHT,
    PP_LEAVE_ALONG_LEFT,
    PP_LEAVE_ALONG_RIGHT,
    PP_LEAVE_CROSS_LEFT,
    PP_LEAVE_CROSS_RIGHT
};

class SWE_Maneuver
{
public:

    /*! constructor of the class
    */
    SWE_Maneuver();

    /*! destructor */
    ~SWE_Maneuver();

    /**
     * @brief init new maneuver
     * @param maneuver the maneuver to be executed
     * @param heading odometry: the current heading of the car
     * @param distanceSum odometry: the sum of the driven distance of the car
     * @return the neccessary steering angle
     */
    tResult Start(maneuvers maneuver, tFloat32 heading, tInt32 distanceSum, tFloat32 stopLineDistance = 0);

    /**
     * @brief tell me if current maneuver has finished
     * @return is it finished true or false
     */
    tBool IsFinished();

    /**
     * @brief tells you the currently needed gear for speed control
     * @return the gear
     */
    tFloat32 GetGear();

    /**
     * @brief tells you the currently needed steering angle
     * @return the steering angle
     */
    tFloat32 GetSteeringAngle();

    /**
     * @brief calculate new step or update for maneuver
     * @param heading odometry: the current heading of the car
     * @param distanceSum odometry: the sum of the driven distance of the car
     * @return the neccessary steering angle
     */
    tResult CalcStep(tFloat32 heading, tInt32 distanceSum);


    tInt32 StateMachine_STOPLINE(tFloat32 heading, tInt32 distanceSum, tInt32 state, tFloat32 stopLineDistance);


    tResult StateMachine_GS(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_TL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_TR(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_AL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_AR(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_CL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_CR(tFloat32 heading, tInt32 distanceSum, tInt32 state);

    /**
     * @brief reset
     * @return success
     */
    tResult reset();

    //DEBUG:
    tFloat32 debugvar;


private:

    tInt32 m_distanceStart;
    tInt32 m_state;

    tFloat32 m_headingStart;

    maneuvers m_currentManeuver;

    tFloat32 m_steeringAngleOut;
    tFloat32 m_gearOut;



};

