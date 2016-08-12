
#include "robot_localization/PingThread.h"
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include "std_msgs/String.h"

PingThread::PingThread()
{
    // Based on Ping node of TU Munich PTAM
   line1[0] = '\0';
   keepRunning = false;
   p500 = 25;
}

PingThread::~PingThread(void)
{
}

void PingThread::startSystem()
{
   keepRunning = true;
}

void PingThread::stopSystem()
{
   keepRunning = false;
}

double parsePingResult(std::string s)
{
   int pos = s.find("time=");
   int found = 0;
   float ms;
   if(pos != std::string::npos)
       found = sscanf(s.substr(pos).c_str(),"time=%f",&ms);

   if(found == 1 && pos != std::string::npos)
       return ms;
   else
       return 10000;
}

double PingThread::getDelay()
{
    // Returns ping of a navdata sized package
    return p500;
}

void PingThread::run()
{
   // Ping
   sprintf(pingCommand500,"ping -c 1 -s 500 -w 1 192.168.1.1");
   FILE *p;

   if(keepRunning)
   {
       // ping twice, with a sleep in between
       p = popen(pingCommand500,"r");
       fgets(line1, 200, p);
       fgets(line1, 200, p);
       pclose(p);

       // calculate new value
       double res500 = parsePingResult(line1);

       // clip between 10 and 1000.
       res500 = std::min(1000.0,std::max(10.0,res500));

       // set new value
       p500 = 0.7 * p500 + 0.3 * res500;
   }
   else
   {
       // set new value
       p500 = p500Default;
   }
}


