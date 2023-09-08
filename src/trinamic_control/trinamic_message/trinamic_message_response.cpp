// Copyright 2022 Eugen Kaltenegger

#include <trinamic_control/trinamic_message/implementation/trinamic_response.hpp>

using trinamic_control::TrinamicResponse;

TrinamicResponse::TrinamicResponse(unsigned char response_address,
                             unsigned char module_address,
                             unsigned char status,
                             unsigned char command,
                             int value)
{
  this->setByte1(response_address);
  this->setByte2(module_address);
  this->setByte3(status);
  this->setByte4(command);
  this->setValue(value);
  this->setChecksum(this->calculateChecksum());
}

unsigned char TrinamicResponse::getResponseAddress()
{
  return this->getByte1();
}

unsigned char TrinamicResponse::getModuleAddress()
{
  return this->getByte2();
}

unsigned char TrinamicResponse::getStatus()
{
  return this->getByte3();
}

unsigned char TrinamicResponse::getCommand()
{
  return this->getByte4();
}

int TrinamicResponse::getValue()
{
  return TrinamicMessage::getValue();
}

unsigned char TrinamicResponse::getChecksum()
{
  return TrinamicMessage::getChecksum();
}