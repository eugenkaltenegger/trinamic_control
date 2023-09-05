// Copyright 2023 Eugen Kaltenegger

#ifndef TRINAMIC_CONTROL_TRINAMIC_MESSAGES_TRINAMIC_RESPONSE_H
#define TRINAMIC_CONTROL_TRINAMIC_MESSAGES_TRINAMIC_RESPONSE_H

#include <cstring>
#include <string>
#include <boost/format.hpp>

namespace trinamic_control
{
class TrinamicResponse
{
public:
  TrinamicResponse();
  ~TrinamicResponse();

  unsigned char* getBufferPointer();
  size_t getBufferSize();

  unsigned char getReplyAddress();
  unsigned char getModuleAddress();
  unsigned char getStatus();
  unsigned char getCommandNumber();
  int getValue();
  unsigned char getChecksum();

  unsigned char calculateChecksum();

  std::string toDebugString();

private:
  unsigned char buffer_[9]{};
  unsigned char* reply_address_ = &buffer_[0];
  unsigned char* module_address_ = &buffer_[1];
  unsigned char* status_ = &buffer_[2];
  unsigned char* command_number_ = &buffer_[3];
  unsigned char* checksum_ = &buffer_[8];
};
}  // namespace trinamic_control

#endif  // TRINAMIC_CONTROL_TRINAMIC_MESSAGES_TRINAMIC_RESPONSE_H