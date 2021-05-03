// This is the CMOOSApp example from the MOOS website documentation
// Included here for convenience
//
// Feb 10th, 2013

#ifndef PAPP_VAR_HEADER
#define PAPP_VAR_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
typedef int SOCKET;
 
using namespace std;

class pBrains : public CMOOSApp {
public:
  pBrains();
  virtual ~pBrains();

  bool OnNewMail (MOOSMSG_LIST & Mail);
  bool Iterate ();
  bool OnConnectToServer ();
  bool OnStartUp ();

private:
  // Variables for vehicle position
  double curWaypoint;
  double curX;
  double curY;
  double curHeading;
  unsigned int curState;

  // Socket connection variables
  SOCKET udpSock;
  struct timeval timeOut;
  fd_set masterFDS;
  fd_set readFDS;
};

#endif
