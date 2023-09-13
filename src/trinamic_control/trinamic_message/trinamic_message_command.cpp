// Copyright 2023 Eugen Kaltenegger

#include <trinamic_control/trinamic_message/trinamic_message_command.hpp>

using trinamic_control::TrinamicMessageCommand;

TrinamicMessageCommand::TrinamicMessageCommand(unsigned char module_address,
                                 unsigned char command,
                                 unsigned char type,
                                 unsigned char id,
                                 int value)
{
  this->setModuleAddress(module_address);
  this->setCommand(command);
  this->setType(type);
  this->setId(id);
  this->setValue(value);
  this->setChecksum(this->calculateChecksum());
}


void TrinamicMessageCommand::setModuleAddress(unsigned char module_address)
{
  this->setByte1(module_address);
}

void TrinamicMessageCommand::setCommand(unsigned char command)
{
  this->setByte2(command);
}

void TrinamicMessageCommand::setType(unsigned char type)
{
  this->setByte3(type);
}

void TrinamicMessageCommand::setId(unsigned char id)
{
  this->setByte4(id);
}