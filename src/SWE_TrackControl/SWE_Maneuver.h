#include "stdafx.h"
#include "math.h"


/*! SmartWheels-Erlangen (Robert de Temple)
 * This class implements an easy way to define car maneuvers
 * Once the maneuvers are started the neccessary steering angles and gears are updated according to the odometry data
 * A simple heuristic prevents the registration of distant stoplines when still approaching another
 */


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
    * @param clock reference to the adtf clock object
    */
    SWE_Maneuver(ucom::cObjectPtr<IReferenceClock> &clock);

    /*! destructor */
    ~SWE_Maneuver();

    /**
     * @brief init new maneuver
     * @param maneuver the maneuver to be executed
     * @param headingSum absolute summed heading from odometry: the current heading of the car (e.g. sum of all heading changes since start -> must count more than +/- PI)
     * @param distanceSum odometry: the sum of the driven distance of the car
     * @param stopLineXLeft x coordinate of left stop line point
     * @param stopLineXRight x coordinate of right stop line point
     * @return the neccessary steering angle
     */
    tResult Start(maneuvers maneuver, tFloat32 headingSum, tInt32 distanceSum, tFloat32 stopLineXLeft = 0, tFloat32 stopLineXRight = 0, tBool isRealStopline = true); //TODO don't accept wrong stoplines

    /**
     * @brief tell me if current maneuver has finished
     * @return the status of the object NO_MANEUVER = maneuver finished
     */
    maneuvers GetStatus();

    /**
     * @brief tells you the currently recommended gear for speed control
     * @return the gear
     */
    tFloat32 GetGear();

    /**
     * @brief tells you the currently needed steering angle
     * @return the steering angle
     */
    tFloat32 GetSteeringAngle();

    tFloat32 GetStoplineDistance();

    /**
     * @brief tells you if the stopline being approached is real or not
     * @return is stopline real?
     */
    tBool GetStoplineType();

    /**
     * @brief calculate new step or update for maneuver
     * @param heading odometry: the current heading of the car
     * @param distanceSum odometry: the sum of the driven distance of the car
     * @return the neccessary steering angle
     */
    tResult CalcStep(tFloat32 heading, tInt32 distanceSum);



    /**
     * @brief reset
     * @return success
     */
    tResult Reset();

    //DEBUG:
    tFloat32 debugvar;


private:

    tBool TestStoplineDistance(tFloat32 newDistance, tFloat32 distanceSum);

    tResult CalcStep_p(tFloat32 heading, tInt32 distanceSum, tFloat32 stopLineDistance);


    tInt32 StateMachine_STOPLINE(tInt32 distanceSum, tInt32 state, tFloat32 stopLineDistance);


    tResult StateMachine_GS(tInt32 distanceSum, tInt32 state);


    tResult StateMachine_TL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_TR(tFloat32 heading, tInt32 distanceSum, tInt32 state);



    //defunct as not needed (yet)
    tResult StateMachine_AL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_AR(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_CL(tFloat32 heading, tInt32 distanceSum, tInt32 state);


    tResult StateMachine_CR(tFloat32 heading, tInt32 distanceSum, tInt32 state);



    //--- member variables ---
    tInt32 m_state;

    maneuvers m_currentManeuver;

    tFloat32 m_steeringAngleOut;
    tFloat32 m_gearOut;

    ucom::cObjectPtr<IReferenceClock> &_clock;

    tBool m_stoplineType; // true if real stopline

    tFloat32 m_stoplineDistance;

    tFloat32 m_startStopLineDistance;
    tFloat32 m_startDistance;
    tFloat32 m_stoplineDistanceEst;




};

