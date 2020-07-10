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

#include "ros/ros.h"
#include <string>
#include <stdio.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include "puma_motor_driver/slcan_gateway.h"
using boost::asio::ip::udp;
using boost::asio::ip::address;



namespace puma_motor_driver
{

SLCANGateway::SLCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false)
{
}

bool SLCANGateway::connect()
{
    socket_ = new boost::asio::ip::udp::socket(io_service_);
    endpoint_ = udp::endpoint(address::from_string(canbus_dev_), 11412);

    socket_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}


bool SLCANGateway::isConnected()
{
  return is_connected_;
}

bool SLCANGateway::recv(Message* msg)
{
  SLCanMsg slCanMsg;
  if (boost::asio::read(socket_, boost::asio::buffer(&slCanMsg, sizeof(SLCanMsg))))
  {
    decodedMsg(*msg, slCanMsg);
  }

  return false;
}

void SLCANGateway::queue(const Message& msg)
{
    ROS_DEBUG("Queuing ID 0x%08x, data (%d)", msg.id, msg.len);
    write_frames_[write_frames_index_].id = msg.id;
    write_frames_[write_frames_index_].len = msg.len;

    memmove(write_frames_[write_frames_index_].data, msg.data, msg.len);
        
    write_frames_index_++;

    if (write_frames_index_ > sizeof(write_frames_) / sizeof(write_frames_[0]))
    {
        throw std::runtime_error("Overflow of write_frames_index_ in SLCANGateway.");
    }
}

bool SLCANGateway::sendAllQueued()
{
  for (int i = 0; i < write_frames_index_; i++)
  {
    ROS_DEBUG("Writing ID 0x%08x, data (%d)", write_frames_[i].id, write_frames_[i].len);

    SLCanMsg request = {0};

    encodedMsg(request, write_frames_[i]);

    boost::system::error_code err;
    auto sent = socket_.send_to(boost::asio::buffer(&request, sizeof(request)), endpoint_, 0, err);
    // **WARNING* 
    // Return Result Ignored on other implementations
  }
  write_frames_index_ = 0;
  return true;
}

void SLCANGateway::encodedMsg(SLCanMsg& slCanMsg, const Message& msg)
{
  
}

void SLCANGateway::decodedMsg(Message& msg, const SLCanMsg& slCanMsg)
{

}

}