// Copyright 2023 Eugen Kaltenegger

#ifndef TRINAMIC_CONTROL_TRINAMIC_RESPONSE_H
#define TRINAMIC_CONTROL_TRINAMIC_RESPONSE_H

#include <trinamic_control/trinamic_message/trinamic_message.hpp>

namespace trinamic_control
{
class TrinamicMessageResponse : public TrinamicMessage
{
public:
  TrinamicMessageResponse() = default;
  ~TrinamicMessageResponse() = default;
  TrinamicMessageResponse(unsigned char response_address,
                unsigned char module_address,
                unsigned char status,
                unsigned char command,
                int value);
  unsigned char getResponseAddress();
  unsigned char getModuleAddress();
  unsigned char getStatus();
  unsigned char getCommand();
  int getValue() override;
  unsigned char getChecksum() override;
};
}  // namespace trinamic_control


#endif //TRINAMIC_CONTROL_TRINAMIC_RESPONSE_H