// Copyright 2023 Eugen Kaltenegger

#include "trinamic_control/trinamic_message/trinamic_command.hpp"

#include <string>

using trinamic_control::TrinamicCommand;

TrinamicCommand::TrinamicCommand()
{
  memset(&this->buffer_, '\0', sizeof(this->buffer_));
}

TrinamicCommand::~TrinamicCommand() = default;

TrinamicCommand::TrinamicCommand(unsigned char command_number, unsigned char command_type)
{
  unsigned char module_address = 1;
  unsigned char motor_number = 0;
  int value = 0;
  this->setModuleAddress(module_address);
  this->setCommandNumber(command_number);
  this->setCommandType(command_type);
  this->setMotorNumber(motor_number);
  this->setValue(value);
  this->setChecksum(this->calculateChecksum());
}

TrinamicCommand::TrinamicCommand(unsigned char command_number, unsigned char command_type, int value)
{
  unsigned char module_address = 1;
  unsigned char motor_number = 0;
  this->setModuleAddress(module_address);
  this->setCommandNumber(command_number);
  this->setCommandType(command_type);
  this->setMotorNumber(motor_number);
  this->setValue(value);
  this->setChecksum(this->calculateChecksum());
}

unsigned char* TrinamicCommand::getBufferPointer()
{
  return this->buffer_;
}

size_t TrinamicCommand::getBufferSize()
{
  return sizeof(this->buffer_);
}

void TrinamicCommand::setModuleAddress(unsigned char module_address)
{
  *this->module_address_ = module_address;
}

void TrinamicCommand::setCommandNumber(unsigned char command_number)
{
  *this->command_number_ = command_number;
}

void TrinamicCommand::setCommandType(unsigned char command_type)
{
  *this->command_type_ = command_type;
}

void TrinamicCommand::setMotorNumber(unsigned char motor_number)
{
  *this->motor_number_ = motor_number;
}

void TrinamicCommand::setValue(int value)
{
  this->buffer_[4] = (value >> 24) & 0xFF;
  this->buffer_[5] = (value >> 16) & 0xFF;
  this->buffer_[6] = (value >>  8) & 0xFF;
  this->buffer_[7] = value & 0xFF;
}

void TrinamicCommand::setChecksum(unsigned char checksum)
{
  *this->checksum_ = checksum;
}

unsigned char TrinamicCommand::calculateChecksum()
{
  unsigned char checksum = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    checksum += this->buffer_[i];
  }
  return checksum;
}

std::string TrinamicCommand::toDebugString()
{
  std::string string;
  for (int i = 0; i < 9; i++)
  {
    // convert to HEX
    char hex_char[3];
    snprintf(hex_char, sizeof(hex_char), "%02X", this->buffer_[i]);
    // add HEX to string
    string += std::string(hex_char);
    // add whitespace to string (if another HEX follows)
    if (i != 8)
    {
      string += std::string(" ");
    }
  }
  return string;
}