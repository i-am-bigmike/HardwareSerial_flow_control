/* Flow control TX test 1.0.0
 * (C) 26-03-2019 Michele Bighignoli
 * 
*/

#define SerialSend Serial2

void setup() {
  pinMode(13, OUTPUT);
  for (byte i=0; i<3; i++); {
    digitalWrite(13,1);
    delay(250);
    digitalWrite(13,0);
    delay(250);
  }
  
  SerialSend.begin(500000, 3, 4);   // speed, cts, rts
}  

void loop() {
  SerialSend.write('{');  // Ready to start

  char c = '0';
  for (unsigned long i=1; i<=100000000; i++) {
    SerialSend.write(c);
    c++;

    if (c > 'z') {
      c = '0';
    }
  }

  SerialSend.flush();  // Test if flush works fine
  if (SerialSend.availableForWrite() == (SERIAL_TX_BUFFER_SIZE - 1)) {
    digitalWrite(13,1);  // Led on = error
  }
  
  SerialSend.write('}');  // Stop

  while (true) {}
}
