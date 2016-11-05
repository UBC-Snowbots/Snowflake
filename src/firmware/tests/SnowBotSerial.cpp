#include <SnowBotSerial.hpp>
using namespace LibSerial;

// The size of the message that is to be sent or read after the identifer byte.
const int SnowBotSerial::MSG_SIZE = 6;
// The identifier byte that is used in this serial protocol. Every
// message sent or received through serial must include this identifier
// byte in order for the receiver of the message to know when to begin
// interpreting the message.
const char SnowBotSerial::IDENTIFIER = 'B';
// The VTIME as defined by POSIX termios
const short SnowBotSerial::TIME_OUT = 1;

/*
  Constructs a SnowBotSerial object that has a serial port that is connected to port with
  baud rate specified by baud_rate.
  
  Precondition: port must be a valid port name.
*/
SnowBotSerial::SnowBotSerial(const std::string port, const SerialStreamBuf::BaudRateEnum baud_rate) {
  this->port = port;
  serial_stream.Open(port);

  this->baud_rate = baud_rate;
  serial_stream.SetBaudRate(baud_rate);
}

/*
  Reads data from the serial port using a read protocol that first finds and reads
  an IDENTIFIER byte before reading the next MSG_SIZE number of bytes.
  
  Postcondition: returns message with size MSG_SIZE and modifies success to be true,
    if the reading protocol was successful. Otherwise returns an empty string and
    modifies success to be false.
*/
std::string SnowBotSerial::readProtocol(bool& success) {
  /*
    If VMIN = 0 and VTIME > 0 then this is a pure timed read. This is an overall
    timer, not an intercharacter one. This means that if nothing is read in
    TIME_OUT * 1/10 of a sec then the control is returned back to the caller.
  */
  serial_stream.SetVMin(0);
  serial_stream.SetVTime(TIME_OUT);
  
  char ch = 0;
  serial_stream >> ch;

  if (ch != IDENTIFIER) {
    success = false;
    return "";
  }

  // sets the minimum number of characters that must be read from the stream 
  serial_stream.SetVMin(MSG_SIZE);

  std::string message;
  message.reserve(MSG_SIZE);
  for(std::size_t i = 0; i < MSG_SIZE; i++){
    message.push_back(serial_stream.get());
  }

  success = true;
  return message;
}

/*
  Sends a message to serial using a write protocol that first sends the IDENTIFIER byte
  before sending MSG_SIZE number of bytes.
  
  Postcondition: the serial would have sent from left to right order the bytes:
    IDENTIFIER, msgToSend[0], msgToSend[1], ... , msgToSend[5].
*/
void SnowBotSerial::writeProtocol(const std::string msgToSend) {
  serial_stream << IDENTIFIER;
  for (int i = 0; i < MSG_SIZE; i++) {
    serial_stream << msgToSend[i];
  }
}

SerialStreamBuf::BaudRateEnum SnowBotSerial::getBaudRate() {
  return baud_rate;
}

void SnowBotSerial::switchPort(const std::string newPort) {
  serial_stream.Close();
  port = newPort;
  serial_stream.Open(port);
}

std::string SnowBotSerial::getPort() {
  return port;
}
