/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
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

#include "RapiChatterbox"
#include "cbwander.h"
#include <signal.h>
#include <string.h>

Rapi::CCBRobot* robot = NULL;
CDriveCircle* robotCtrl = NULL;

//-----------------------------------------------------------------------------
void quitSig(int signum)
{
  PRT_MSG0(4,"User requested ctrl-c");

  // set default signal handler
  if (signal(SIGINT, SIG_DFL) == SIG_ERR) {
     PRT_ERR1("Error resetting signal handler %s", strerror(errno));
  }

  // quit main thread
  robot->quit();
}
//------------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
  if (argc != 4) {
    printf("Please provide a chatterbox id, turnrate and velocity.\n");
    printf("drivecircle CHATTERBOX_ID TURNRATE VELOCITY\n");
    return 1;
  }

  // init general stuff
  ErrorInit ( 1, false );
  initRandomNumberGenerator();

  printf("-----------------------------------\n");
  printf("Chatterbox RAPI demo \n");
  printf("  build %s %s \n", __DATE__, __TIME__);
  printf("  compiled against RAPI version %s (%s) build %s\n", RAPI_VERSION(),
           RAPI_GIT_VERSION(), RAPI_BUILD() );
  printf("\n");
  printf("Press PLAY to start demo programm\n");
  printf("Once demo is running, enable motors with ADVANCE button\n");
  printf("lifting the robot up disables motors\n");

  if (signal(SIGINT, quitSig) == SIG_ERR) {
     PRT_ERR1("Error resetting signal handler %s", strerror(errno));
  }

  // create robot and its controller
  robot = new Rapi::CCBRobot ();
  if ( robot->init() == 0) {
    Rapi::rapiError->print();
    delete robot;
    exit(-1);
  }
  robotCtrl = new CDriveCircle ( robot, std::string(argv[1]) );
  robotCtrl->setTurnrate(atof(argv[2]));
  robotCtrl->setVelocity(atof(argv[3]));
  // blocking call
  robot->run();

  // clean up robot controller
  if (robotCtrl) {
    delete robotCtrl;
    robotCtrl = NULL;
  }

  // clean up robot
  if (robot)
    delete (robot);
  return 0;
}
