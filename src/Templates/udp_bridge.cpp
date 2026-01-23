/* This is an example file to test the UDP control
*/

// SDK includes
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// General includes
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.12.1", 8082)
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Safety safe;
  UDP udp;
  HighCmd cmd = {};
  HighState state = {};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
  udp.GetRecv(state);
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime += 2;
  //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
  cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
  cmd.gaitType = 0;
  cmd.speedLevel = 0;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;
}

int main(void)
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);

  std::string sysState = " ";

  while (1)
  {
    sysState = std::to_string(custom.state.bms.SOC);
    std::cout << "Battery: %" + sysState << std::endl;
    sysState = std::to_string(custom.state.imu.temperature);
    std::cout << "Temp(C): " + sysState << std::endl;
    sysState = std::to_string(custom.state.velocity[0]);
    std::cout << "Forward Speed: " + sysState + "m/s" << std::endl;
    custom.cmd.mode = 2;
    custom.cmd.gaitType = 1;
    custom.cmd.velocity[0] = 0.1f;
    custom.udp.SetSend(custom.cmd);

    sleep(1);
    sleep(1);
  };

  return 0;
}