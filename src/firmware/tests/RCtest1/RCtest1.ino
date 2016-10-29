int R1, R2, R3, R4, B2, B4, Mode = 0;

void setup() {
 Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT); 
  
}

void loop() {
 R1 = pulseIn(2, HIGH);
 R2 = pulseIn(3, HIGH);
 R3 = pulseIn(4, HIGH);
 R4 = pulseIn(5, HIGH); 
  Serial.print("R1: "); Serial.print(R1);Serial.print(" R2: "); Serial.print(R2);Serial.print(" R3: "); Serial.println(R3);Serial.println("Mode: ");Serial.println(Mode);
}
