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
#include "puma_motor_driver/peakcan_gateway.h"
#include "ros/ros.h"

#include <windows.h>
#include <PCANBasic.h>

#include <map>

namespace puma_motor_driver
{

const std::map<std::string, uint16_t> paramToBusId = 
{
    {"Lan1", PCAN_LANBUS1}, 
    {"Lan2", PCAN_LANBUS2}, 
    {"Lan3", PCAN_LANBUS3}, 
    {"Lan4", PCAN_LANBUS4}, 
    {"Lan5", PCAN_LANBUS5}, 
    {"Lan6", PCAN_LANBUS6}, 
    {"Lan7", PCAN_LANBUS7}, 
    {"Lan8", PCAN_LANBUS8}, 
    {"Lan9", PCAN_LANBUS9}, 
    {"Lan10", PCAN_LANBUS10}, 
    {"Lan11", PCAN_LANBUS11}, 
    {"Lan12", PCAN_LANBUS12}, 
    {"Lan13", PCAN_LANBUS13}, 
    {"Lan14", PCAN_LANBUS14}, 
    {"Lan16", PCAN_LANBUS15}, 
    {"USB1", PCAN_USBBUS1}, 
    {"USB2", PCAN_USBBUS2}, 
    {"USB3", PCAN_USBBUS3}, 
    {"USB4", PCAN_USBBUS4}, 
    {"USB5", PCAN_USBBUS5}, 
    {"USB6", PCAN_USBBUS6}, 
    {"USB7", PCAN_USBBUS7}, 
    {"USB8", PCAN_USBBUS8}, 
    {"USB9", PCAN_USBBUS9}, 
    {"USB10", PCAN_USBBUS10}, 
    {"USB11", PCAN_USBBUS11}, 
    {"USB12", PCAN_USBBUS12}, 
    {"USB13", PCAN_USBBUS13}, 
    {"USB14", PCAN_USBBUS14}, 
    {"USB15", PCAN_USBBUS15}, 
    {"USB16", PCAN_USBBUS16}, 
    {"PCI1", PCAN_PCIBUS1}, 
    {"PCI2", PCAN_PCIBUS2}, 
    {"PCI3", PCAN_PCIBUS3}, 
    {"PCI4", PCAN_PCIBUS4}, 
    {"PCI5", PCAN_PCIBUS5}, 
    {"PCI6", PCAN_PCIBUS6}, 
    {"PCI7", PCAN_PCIBUS7}, 
    {"PCI8", PCAN_PCIBUS8}, 
    {"PCI9", PCAN_PCIBUS9}, 
    {"PCI10", PCAN_PCIBUS10}, 
    {"PCI11", PCAN_PCIBUS11}, 
    {"PCI12", PCAN_PCIBUS12}, 
    {"PCI13", PCAN_PCIBUS13}, 
    {"PCI14", PCAN_PCIBUS14}, 
    {"PCI15", PCAN_PCIBUS15}, 
    {"PCI16", PCAN_PCIBUS16},
};

PeakCANGateway::PeakCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false),
  busId_(PCAN_NONEBUS)
{
}

bool PeakCANGateway::connect()
{
    auto busId = paramToBusId.find(canbus_dev_);
    if (busId == paramToBusId.end()) 
    {
        throw std::runtime_error("canbus id not found in map");
    }

    busId_ = busId->second;

    TPCANStatus result = CAN_Initialize(busId_, PCAN_BAUD_500K);
    is_connected_ = (result == PCAN_ERROR_OK);
}


bool PeakCANGateway::isConnected()
{
  return is_connected_;
}

bool PeakCANGateway::recv(Message* msg)
{
    TPCANMsg received = {0};

    TPCANStatus result = CAN_Read(busId_, &received, NULL);
    if (result == PCAN_ERROR_QRCVEMPTY)
    {
        ROS_DEBUG("No more frames");
        return false;
    }
    else if (result == PCAN_ERROR_OK)
    {
        ROS_DEBUG("Recieved ID 0x%08x, data (%d)", (received.ID), received.LEN);
        msg->id = received.ID;
        msg->len = received.LEN;
        for (int i = 0; i < msg->len; i++)
        {
            msg->data[i] = received.DATA[i];
        }

        return true;
    }
    else
    {
        ROS_ERROR("Error reading from socketcan: %d", result);
    }

    return false;
}

void PeakCANGateway::queue(const Message& msg)
{
    ROS_DEBUG("Queuing ID 0x%08x, data (%d)", msg.id, msg.len);
    write_frames_[write_frames_index_].id = msg.id;
    write_frames_[write_frames_index_].len = msg.len;

    memmove(write_frames_[write_frames_index_].data, msg.data, msg.len);
        
    write_frames_index_++;

    if (write_frames_index_ > sizeof(write_frames_) / sizeof(write_frames_[0]))
    {
        throw std::runtime_error("Overflow of write_frames_index_ in PeakCANGateway.");
    }
}

bool PeakCANGateway::sendAllQueued()
{
  for (int i = 0; i < write_frames_index_; i++)
  {
    ROS_DEBUG("Writing ID 0x%08x, data (%d)", write_frames_[i].id, write_frames_[i].len);

    TPCANMsg request = {0};
    request.ID = write_frames_[i].id;
    request.MSGTYPE = PCAN_MESSAGE_STANDARD;
    request.LEN = write_frames_[i].len;
    memmove(request.DATA, write_frames_[i].data, request.LEN);

    TPCANStatus result = CAN_Write(busId_, &request);
    if (result != PCAN_ERROR_OK)
    {
        throw std::runtime_error("CAN_Write failed.");
    }
  }
  write_frames_index_ = 0;
  return true;
}

}