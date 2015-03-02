/***************************************************************************
 * Project: RAPI                                                           *
 * Author: Lingkang Zhang (lingkangzhang@sfu.ca)                           *
 * $Id: $								   *
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/
#ifndef CBWANDER_H
#define CBWANDER_H

#include "RapiChatterbox"

#ifdef STAGE
#include "RapiStage"
#endif

using namespace Rapi;

typedef enum {STATE_STOP, STATE_SPINC, STATE_SPINCC, STATE_FORWARD, STATE_BACKWARD, STATE_CIRCLE, STATE_LEFT, STATE_RIGHT} tState;

#define FWD_VEL 0.05
#define FWD_TIME 1

// Redis keys
#define CMD_FORWARD "w"
#define CMD_BACKWARD "s"
#define CMD_LEFT "a"
#define CMD_RIGHT "d"
#define CMD_SPINC "z"
#define CMD_SPINCC "c"

/**
 * A controller for chatterbox that drives randomly in a rectangle area.
 * @author Lingkang Zhang
 */
class CWander : public ARobotCtrl
{
  public:
    /**
     * Default constructor
     * @param robot this controller controls
     */
    CWander( ARobot* robot, std::string cbID);
    /** Default destructor */
    ~CWander();

    void setTurnrate(float tr);

  protected:
    /**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);
    /**
     * The core behaviour of the robot.
     * This is called every time step
     */
    void run();
    /** Emergency stop routine */
    bool emergencyStop();
    /** Drivetrain */
    ADrivetrain2dof* mDrivetrain;
    /** Infrared sensors */
    //ARangeFinder* mIr;
    /** Power pack */
    //APowerPack* mPowerPack;
    /** Text display */
    ATextDisplay* mTextDisplay;
    /** Lights */
    ALights* mLights;
    /** Bumper */
    ABinarySensorArray* mBumper;
    /** Wheel drop */
    ABinarySensorArray* mWheelDrop;
    /** Low side driver */
    //ASwitchArray* mLowSideDriver;
    /** Laser range finder */
    //ARangeFinder* mLaser;
    /** Top fiducial */
    //AFiducialFinder* mTopFiducial;
    /** Front fiducial */
    //AFiducialFinder* mFrontFiducial;
    /** Photo sensor */
    AAnalogSensorArray* mPhoto;
    /** Create button */
    ABinarySensorArray* mButton;
    /** Cliff sensor */
    ABinarySensorArray* mCliff;
    /** Odometry */
    COdometry* mOdometry;
    /** Some limit */
    CLimit mLimit;
    /** Redis Client */
    CRedisClient* redis;
    /** Data logger */
    CDataLogger* mDataLogger;


  private:
    float mturnRate;
    float mVelocity;
    /** State of robot **/
    tState mState;
    /** Time since start of controller [s] */
    float mTime;
    /** Flag if we should run the main behaviour or not */
    bool mFgRun;
    /** Text for 7seg display */
    int mText;
    /** Flags if motors are enabled */
    bool mFgMotor;
    /** General purpose timer */
    float mTimer;
    /** Unique ID of the chatterbox for this controller */
    std::string mCBID;
    /** Min photo value */
    float mMinPhoto;
	/** Encounter the bound defined by the black tape on the ground **/
	bool mBound;
    bool mBoundFirstTime;
};

#endif
