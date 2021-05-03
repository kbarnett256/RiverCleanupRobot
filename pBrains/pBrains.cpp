#include <iterator>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "pBrains.h"
#include "MBUtils.h"

#include <iostream>
 
using namespace std;

pBrains::pBrains()
{
  // Initialize the variables
  curWaypoint = 0;
  curX = 0;
  curY = 0;
  curState = 0;

}

pBrains::~pBrains()
{
    // Close the udp socket, if it was opened
    int status = shutdown(udpSock, SHUT_RDWR);
    if (status == 0)
    {
        status = close(udpSock);
    }
}

bool pBrains::OnNewMail (MOOSMSG_LIST & Mail) 
{
// process it
MOOSMSG_LIST::iterator q;
    for(q=Mail.begin(); q!=Mail.end(); q++) {
        CMOOSMsg &msg = *q;

        string key = msg.GetKey();

        // Set the current position, waypoint, and heading whenever they are changed in the MOOS database

        if (key == "NAV_X")
            curX = msg.GetDouble();
        if (key == "NAV_Y")
            curY = msg.GetDouble();
        if (key == "WPT_INDEX")
            curWaypoint = msg.GetDouble();
        if (key == "NAV_HEADING")
            curHeading = msg.GetDouble();

    }
    return(true);
}

bool pBrains::OnConnectToServer () {

    // Register to receive mail updates for the position, waypoint, and heading variables when they are changed
    Register("WPT_INDEX", 0);
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_HEADING", 0);
    return(true);

} 

bool pBrains::Iterate () {

    // Timeout of 0 seconds to make the select call non-blocking
        timeOut.tv_sec = 0;
        timeOut.tv_usec = 0;

        readFDS = masterFDS;

        // Check if the socket is readable
        if (select(udpSock + 1 , &readFDS, 0, 0, &timeOut)){

            // If the socket is readable, read it
            if (FD_ISSET(udpSock, &readFDS)) 
            {

                socklen_t fromLen = sizeof(struct sockaddr_in);
                struct sockaddr_in fromAddr;

                // Allocate a buffer for the message
                char pktBuf[64] = {};
                // Read the message from the socket
                int msgLen = recvfrom(udpSock, pktBuf, 64, 0, (struct sockaddr *)&fromAddr, &fromLen);

                // Make sure the message length is valid
                if (msgLen > 0)
                {
                    // Packet received, set behavior based on Kinect readings
                    bool objectDetected = false;

                    // Initialize the direction bia to zero
                    int dirBias = 0;

                    for (unsigned int i = 0; i < 512; ++i)
                    {
                        // Read each bit to see if it is set
                        bool bitSet = (pktBuf[i/8] >> (7 - (i % 8))) & 1;

                        // If the bit is set, add the column offset to the direction bias sum
                        if (bitSet) 
                        {
                            objectDetected = true;

                            // Choose a direction to turn based on the kinect readings. Objects on the extreme ends give a higher bias to avoid turning directly into unseen objects
                            // If the object is on the left side of the buffer, subtract the bias ranging from -256 to -1
                            if (i < 256)
                            {
                                dirBias = dirBias - (256 - i);
                            }
                            // If the object is on the right side of the buffer, add the bias ranging from 1 to 256
                            else 
                            {
                                dirBias = dirBias + i - 255;
                            }
                        }
                    }

                    // If an object was detected, change the desired heading variable and state
                    if (objectDetected)
                    {
                        double newHeading;
                        int intHeading;

                        // Deflect the heading 90 degrees left or right from the current heading depending on the Kinect readings
                        if (dirBias < 0){
                            newHeading = curHeading - 90.0;
                            if (newHeading < 0.0)
                            {
                                newHeading += 360.0;
                            }
                        }
                        else 
                        {
                            newHeading = curHeading + 90.0;
                            if (newHeading > 360.0)
                            {
                                newHeading -= 360.0;
                            }
                        }

                        intHeading = (int) (newHeading + 0.5);

                        char updateString[20] = {};

                        sprintf(updateString, "heading = %d", intHeading);

                        Notify("UPDATES_AVOIDH", updateString);
                        Notify("AVOID", "true");

                    }

                }
            }

        }
        
    return (true);
    
}

bool pBrains::OnStartUp () {

    // Register to receive mail updates for the position, waypoint, and heading variables when they are changed
    Register("WPT_INDEX", 0);
    Register("NAV_X", 0);
    Register("NAV_Y", 0);
    Register("NAV_HEADING",0);

    // Open a socket and start listening on port 9900
    udpSock = socket(AF_INET, SOCK_DGRAM, 0);

    FD_ZERO(&readFDS);
    FD_ZERO(&masterFDS);

    FD_SET(udpSock, &masterFDS);

    struct sockaddr_in server_addr;

    // Zero out the variable serv_addr
    memset((char *)&server_addr, sizeof(server_addr), 0);
    
    // Initialize the serv_addr
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    // Convert port number from host to network
    server_addr.sin_port = htons(9900);

    if (bind(udpSock, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
    {
        perror("ERROR on binding");
        exit(1);

    }



    return(true);

} 


