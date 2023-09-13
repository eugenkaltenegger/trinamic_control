// Copyright 2022 Eugen Kaltenegger

#ifndef TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H
#define TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H

#include <fcntl.h>
#include <cerrno>
#include <termios.h>
#include <memory>
#include <cstring>
#include <string>
#include <unistd.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#define SET_AXIS_PARAMETER 5
#define GET_AXIS_PARAMETER 6

#define SUCCESS 100
#define WRONG_CHECKSUM 1
#define INVALID_COMMAND 2
#define WRONG_TYPE 3
#define INVALID_VALUE 4
#define CONFIGURATION_LOCKED 5
#define COMMAND_NOT_AVAILABLE 6

namespace trinamic_control
{
class TrinamicMessageCommand;
class TrinamicMessageResponse;
class TrinamicConnection
{
public:
  static std::shared_ptr<TrinamicConnection> getConnection(YAML::Node node);

  explicit TrinamicConnection(YAML::Node node);

  bool connect();
  bool disconnect();

  void write(TrinamicParameter trinamic_parmeter, int data);
  int read(TrinamicParameter trinamic_parameter);

private:
  TrinamicMessageResponse communicate(TrinamicMessageCommand command);
  //static std::unique_ptr<std::map<std::string, std::shared_ptr<TMCM1640Connection>>> connection_table_;
  //std::shared_ptr<GenericConnectionDescription> connection_description_;
  //int serial_port_{};
  //struct termios tty_{};
};
}  // namespace tuw_trinamic_ros_control

#endif  // TUW_TRINAMIC_ROS_CONTROL_TRINAMIC_CONNECTION_H