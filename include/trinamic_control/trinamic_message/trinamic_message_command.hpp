// Copyright 2022 Eugen Kaltenegger

#ifndef TRINAMIC_CONTROL_TRINAMIC_COMMAND_H
#define TRINAMIC_CONTROL_TRINAMIC_COMMAND_H

#include <trinamic_control/trinamic_message/trinamic_message.hpp>

namespace trinamic_control
{
class TrinamicCommand : public TrinamicMessage
{
public:
  TrinamicCommand(unsigned char module_address,
                  unsigned char command,
                  unsigned char type,
                  unsigned char id,
                  int value);
  TrinamicCommand() = default;
  ~TrinamicCommand() = default;
protected:
  void setModuleAddress(unsigned char module_address);
  void setCommand(unsigned char command);
  void setType(unsigned char type);
  void setId(unsigned char id);
};
}  // namespace trinamic_control


#endif //TRINAMIC_CONTROL_TRINAMIC_COMMAND_H