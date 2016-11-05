#include <SnowBotSerial.hpp>
#include <iostream>

int main() {
  SnowBotSerial snowBot("/dev/ttyACM1" ,SerialStreamBuf::BAUD_9600);
  
  std::string message = "A+B=CR";
  std::string receivedMsg;

  bool success;
  
  while (true) {
    success = false;
    snowBot.writeProtocol(message);
    receivedMsg = snowBot.readProtocol(success);
    if (success) {
      std::cout << "I read: " << receivedMsg << std::endl;
    }
  }
  return 0;
}
