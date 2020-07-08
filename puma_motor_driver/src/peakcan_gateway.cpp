/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <stdio.h>
#include "puma_motor_driver/socketcan_gateway.h"
#include "ros/ros.h"

#include <windows.h>
#include <PCANBasic.h>

namespace puma_motor_driver
{

PeakCANGateway::PeakCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false)
{
}

bool PeakCANGateway::connect()
{
    
    TPCANStatus result = CAN_Initialize(PCAN_PCIBUS1, PCAN_BAUD_500K);
    if (result != PCAN_ERROR_OK)
    {
        throw std::runtime_error("CAN_Initialize failed.");
    }    
}


bool PeakCANGateway::isConnected()
{
  return is_connected_;
}


bool PeakCANGateway::recv(Message* msg)
{
}

void PeakCANGateway::queue(const Message& msg)
{
}

bool PeakCANGateway::sendAllQueued()
{
}

void PeakCANGateway::msgToFrame(Message* msg, can_frame* frame)
{
}

}  // namespace puma_motor_driver
