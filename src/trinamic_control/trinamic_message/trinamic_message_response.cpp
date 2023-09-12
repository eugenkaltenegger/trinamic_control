// Copyright 2022 Eugen Kaltenegger

#include <trinamic_control/trinamic_message/implementation/trinamic_response.hpp>

using trinamic_control::TrinamicMessageResponse;

TrinamicMessageResponse::TrinamicMessageResponse(unsigned char response_address,
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

unsigned char TrinamicMessageResponse::getResponseAddress()
{
  return this->getByte1();
}

unsigned char TrinamicMessageResponse::getModuleAddress()
{
  return this->getByte2();
}

unsigned char TrinamicMessageResponse::getStatus()
{
  return this->getByte3();
}

unsigned char TrinamicMessageResponse::getCommand()
{
  return this->getByte4();
}

int TrinamicMessageResponse::getValue()
{
  return TrinamicMessage::getValue();
}

unsigned char TrinamicMessageResponse::getChecksum()
{
  return TrinamicMessage::getChecksum();
}