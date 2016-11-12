#ifndef SNOWBOT_SERIAL
#define SNOWBOT_SERIAL

#include <SerialStream.h>
using namespace LibSerial;

class SnowBotSerial {
  public:
    // The size of the message that is to be sent or read after the identifer byte.
    static const int MSG_SIZE;
    // The identifier byte that is used in this serial protocol. Every
    // message sent or received through serial must include this identifier
    // byte in order for the receiver of the message to know when to begin
    // interpreting the message.
    static const char IDENTIFIER;
    // The VTIME as defined by POSIX termios
    static const short TIME_OUT;
    
    /*
      Constructs a SnowBotSerial object that has a serial port that is connected to port with
      baud rate specified by baud_rate.
  
      Precondition: port must be a valid port name.
    */
    SnowBotSerial(const std::string port, const SerialStreamBuf::BaudRateEnum baud_rate);
    
    /*
      Sends a message to serial using a write protocol that first sends the IDENTIFIER byte
      before sending MSG_SIZE number of bytes.
  
      Postcondition: the serial would have sent from left to right order the bytes:
        IDENTIFIER, msgToSend[0], msgToSend[1], ... , msgToSend[5].
    */
    void writeProtocol(const std::string msgToSend);
    /*
      Reads data from the serial port using a read protocol that first finds and reads
      an IDENTIFIER byte before reading the next MSG_SIZE number of bytes.
      
      Postcondition: returns message with size MSG_SIZE.
    */
    std::string readProtocol(bool& success);
    
    // Changes the serial port that this object is connected to newPort.
    void switchPort(const std::string newPort);

    // observer functions for reading the private fields
    std::string getPort();
    SerialStreamBuf::BaudRateEnum getBaudRate();
    
  private:
    // The main object that is used to connect to a serial port.
    SerialStream serial_stream;
    std::string port;
    SerialStreamBuf::BaudRateEnum baud_rate;
};

#endif
