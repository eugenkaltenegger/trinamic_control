// Copyright 2023 Eugen Kaltenegger

#ifndef TRINAMIC_CONTROL_TRINAMIC_MESSAGE_TRINAMIC_COMMAND_H
#define TRINAMIC_CONTROL_TRINAMIC_MESSAGE_TRINAMIC_COMMAND_H

#include <cstring>
#include <string>
#include <boost/format.hpp>

namespace trinamic_control
{
class TrinamicCommand
{
public:
  TrinamicCommand();
  ~TrinamicCommand();

  TrinamicCommand(unsigned char command_number, unsigned char command_type);
  TrinamicCommand(unsigned char command_number, unsigned char command_type, int value);

  unsigned char* getBufferPointer();
  size_t getBufferSize();

  void setModuleAddress(unsigned char module_address);
  void setCommandNumber(unsigned char command_number);
  void setCommandType(unsigned char command_type);
  void setMotorNumber(unsigned char motor_number);
  void setValue(int value);
  void setChecksum(unsigned char checksum);

  unsigned char calculateChecksum();

  std::string toDebugString();

private:
  unsigned char buffer_[9]{};
  unsigned char* module_address_ = &buffer_[0];
  unsigned char* command_number_ = &buffer_[1];
  unsigned char* command_type_ = &buffer_[2];
  unsigned char* motor_number_ = &buffer_[3];
  unsigned char* checksum_ = &buffer_[8];
};
}  // namespace trinamic_control

#endif  // TRINAMIC_CONTROL_TRINAMIC_MESSAGE_TRINAMIC_COMMAND_H