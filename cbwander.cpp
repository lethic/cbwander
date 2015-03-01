/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Lingkang Zhang (lingkangzhang@sfu.ca)                          *
 * $Id: $
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
#include "cbwander.h"
#include "utilities.h"
#include <iostream>

//-----------------------------------------------------------------------------
CWander::CWander ( ARobot* robot, std::string cbID )
    : ARobotCtrl ( robot ), mCBID (cbID)
{
  //ADrivetrain2dof* drivetrain;
  mTime = 0.0;
  mTimer = 0.0;
  mMinPhoto = INFINITY;
  mText = 0;

//#ifndef STAGE
//  mState = STOP;
//#else
//  mState = DRIVE;
//#endif

  mState = STATE_STOP;
  mFgMotor = false;

  redis = CRedisClient::getInstance("hal",6379);

#ifndef STAGE
  mRobot->findDevice ( mDrivetrain, CB_DEVICE_DRIVE_TRAIN );
#else
  mRobot->findDevice ( mDrivetrain, "position:0" );//CB_DEVICE_DRIVE_TRAIN );
#endif
  mRobot->findDevice ( mTextDisplay, CB_DEVICE_TEXT_DISPLAY );
#ifndef STAGE
  mRobot->findDevice ( mLights, CB_DEVICE_LIGHTS );
  mRobot->findDevice ( mBumper, CB_DEVICE_BUMPER );
  mRobot->findDevice ( mWheelDrop, CB_DEVICE_WHEEL_DROP );
  mRobot->findDevice ( mButton, CB_DEVICE_BUTTON );
  mRobot->findDevice ( mPhoto, CB_DEVICE_PHOTO_SENSOR );
  mRobot->findDevice ( mCliff, CB_DEVICE_CLIFF );
#endif

  //mDrivetrain = ( CCBDrivetrain2dof* ) drivetrain;
  //mOdometry = mDrivetrain->getOdometry();

  // enable logging
  //mDataLogger = CDataLogger::getInstance( "chatterbox.log", OVERWRITE );
  //mDataLogger->setInterval( 0.1 );
  //mOdometry->setCoordinateSystemOffset( CPose2d(5.0, 2.5, PI/2.0) );
  //mOdometry->startLogging("");


  if ( rapiError->hasError() ) {
    rapiError->print();
    exit ( -1 );
  }
#ifndef STAGE
  ( ( CCBDrivetrain2dof* ) mDrivetrain )->setDefaultOIMode ( CB_MODE_FULL );

  // set up a heart beat with 1Hz
  mLights->setBlink ( DOT, true, 1.0 );
#endif

  mLimit.setLimit ( 0.0, 0.5 );

  // Set text display 'C' for constructor.
  mTextDisplay->setText ( "0" );
  //mCBID = std::string ("cbctrl");
  printf("ID: %s\n", mCBID.c_str());
}
//-----------------------------------------------------------------------------
void CWander::setTurnrate(float turnrate) {
  mturnRate = turnrate;
}

void Cwander::setVelocity(float velocity) {
  mVelocity = velocity;
}

CWander::~CWander()
{
  printf("Deleting robot\n");
#ifndef STAGE
  // Turn off lights
  mLights->setEnabled(false);
#endif
}
//-----------------------------------------------------------------------------
void CWander::updateData ( float dt )
{
  mTime += dt;
  mTimer += dt;

  std::string cmd = "";
  redis->get(mCBID, cmd); // get command from redis server
  //printf("command: %s\n", cmd.c_str());

  if ( ! emergencyStop() ) {
	//cout << "cmd is " << cmd << endl;
    switch (mState) {
      case STATE_STOP:
        if (cmd == CMD_FORWARD) {
	  mTimer = 0.0;
	  mState = STATE_FORWARD;
	  redis->set(mCBID, "none");
	  mDrivetrain->setVelocityCmd(FWD_VEL, 0.0);
	  printf("driving forward\n");
        } else if (cmd == CMD_BACKWARD) {
	  mTimer = 0.0;
	  mState = STATE_BACKWARD;
	  redis->set(mCBID, "none");
	  mDrivetrain->setVelocityCmd(-FWD_VEL, 0.0);
	  printf("driving backward\n");
	} else if (cmd == CMD_CIRCLE) {
          mDrivetrain->setVelocityCmd(mLimit.limit(FWD_VEL), D2R(mturnRate));
	  	  mState = STATE_CIRCLE;
	      redis->set(mCBID, "none");
      	  printf("driving circle\n");
	}
	  else if (cmd == CMD_LEFT){
		 mTimer = 0.0;
	  	 mDrivetrain->setVelocityCmd(FWD_VEL, D2R(mturnRate));
		 mState = STATE_LEFT;
		 redis->set(mCBID, "none");
		 printf("driving left\n"); 
	  }
	  else if (cmd == CMD_RIGHT){
	  	 mTimer = 0.0;
		 mDrivetrain->setVelocityCmd(mLimit.limit(FWD_VEL), D2R(-mturnRate));
		 mState = STATE_RIGHT;
		 redis->set(mCBID, "none");
		 printf("driving right\n");
	  }
      else if (cmd == CMD_SPINC){
	  	 mTimer = 0.0;
		 mDrivetrain->setVelocityCmd(0, D2R(-mturnRate*7));
		 mState = STATE_SPINCC;
		 redis->set(mCBID, "none");
		 printf("spinning clockwise\n");
	  }
	  else if(cmd == CMD_SPINCC){
	  	 mTimer = 0.0;
		 mDrivetrain->setVelocityCmd(0, D2R(mturnRate*7));
		 mState = STATE_SPINC;
		 redis->set(mCBID, "none");
		 printf("spinnning counter-clockwise\n");
	  }
	  else{
	  	cmd = CMD_STOP;
		mDrivetrain->setVelocityCmd(0.0, 0.0);
		redis->set(mCBID, "none");
		printf("stop\n");
	  }
      break;

      case STATE_FORWARD:
	if (mTimer > FWD_TIME) {
	  mDrivetrain->setVelocityCmd(0.0, 0.0);
	  mState = STATE_STOP;
	}
      break;
      case STATE_BACKWARD:
	if (mTimer > FWD_TIME) {
	  mDrivetrain->setVelocityCmd(0.0, 0.0);
	  mState = STATE_STOP;
	}
      break;
	  case STATE_LEFT:
	  if(mTimer > FWD_TIME) {
	  	mDrivetrain->setVelocityCmd(0.0, 0.0);
		mState = STATE_STOP;
	  }
	  break;
	  case STATE_RIGHT:
	  if(mTimer > FWD_TIME) {
	  	mDrivetrain->setVelocityCmd(0.0, 0.0);
		mState = STATE_STOP;
	  }
	  break;
	  case STATE_SPINC:
	  if(mTimer > FWD_TIME) {
	    mDrivetrain->setVelocityCmd(0.0, 0.0);
		mState = STATE_STOP;
	  }
	  case STATE_SPINCC:
	  if(mTimer > FWD_TIME){
	  	mDrivetrain->setVelocityCmd(0.0, 0.0);
		mState = STATE_STOP;
	  }
      case STATE_CIRCLE:
        if (cmd == CMD_STOP) {
	  mDrivetrain->setVelocityCmd(0.0, 0.0);
	  mState = STATE_STOP;
	  redis->set(mCBID, "stop");
	}
      break;
    }
  }
  else {
    printf("Emergency stop.\n");
  }

  if ( rapiError->hasError() ) {
    rapiError->print();
  }
}
//-----------------------------------------------------------------------------
bool CWander::emergencyStop()
{
  bool stop = false;

#ifndef STAGE
  // Check for wheel drop
  if ( mWheelDrop->isAnyTriggered() ) {
    stop = true;
  }

  // Check for cliffs

  if (stop) {
    mDrivetrain->stop();
    mLights->setLight ( ALL_LIGHTS, RED );
  }
  else {
    mLights->setLight ( ALL_LIGHTS, BLACK );
  }

#endif

  return stop;
}
