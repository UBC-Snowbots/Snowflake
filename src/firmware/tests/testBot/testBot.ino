const int MSG_SIZE = 6;
const char IDENTIFIER = 'B';

char incomingByte = 0;
char message[MSG_SIZE];
boolean beginSendBack = false;
boolean error = false;

// Note: MAKE SURE NOT TO open Serial Monitor when using this code
// with c++ serial program or else output will be messed up
void setup() {
  Serial.begin(9600);
}

void loop() {
  // if there is an error stop executing instructions
  if (!error) {
    if (!beginSendBack && Serial.available() > MSG_SIZE) {
      incomingByte = Serial.read();
      // the first byte read must be the identifier byte before anything more is read
      if (incomingByte == IDENTIFIER) {
        delay(10);
        int chReceived = 0;
        while (chReceived < MSG_SIZE &&
          Serial.available() > 0) {
          message[chReceived] = Serial.read();
          chReceived++;
        }
        // the message that is received must be equal to the agreed upon MSG_SIZE
        // for this serial protocol
        if (chReceived != MSG_SIZE) {
          error = true;
          Serial.println("ERROR: DID NOT RECEIVE FULL MSG");
        }
        else {
          beginSendBack = true;
        }
      }
    }
 
    if (beginSendBack) {
      // the first byte sent must be the identifier byte for this serial protocol
      Serial.write(IDENTIFIER);
      for (int i = 0; i < MSG_SIZE; i++) {
        Serial.write(message[i]);
      }
      beginSendBack = false;
    }
  }
}


