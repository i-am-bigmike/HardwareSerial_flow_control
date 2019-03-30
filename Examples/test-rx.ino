/* Flow control test tx 1.0.0
 * (C) 26-03-2019 Michele Bighignoli
 * 
*/

#define SerialRecv Serial2

bool rcvExit = false;
char c = '0';
char r;
unsigned long i = 0;
unsigned int m = 0;
unsigned long elapsedTime;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up");

  SerialRecv.begin(500000, 3, 4);   // speed, cts, rts

  // Wait for start signal
  while (SerialRecv.read() != '{') {
    
  }

  elapsedTime = millis();
}  

void loop() {

  if (!rcvExit) {
    while (SerialRecv.available() > 0) {
      r = SerialRecv.read();
      //delayMicroseconds(5);
      //delay(1);
      //Serial.print(r);

      if (r == '}') {
        rcvExit = true;
        break;
      } else {
        if (r != c) {
          Serial.print("Error ");
          Serial.println(String(r)+" should be "+String(c));
          Serial.println(i);
          rcvExit = true;
          break;
        }
      
        c++;
        if (c > 'z') {
          c = '0';
        }
    
        i++;
        m++;  // Using m and not i % 50000 speeds up the receiving
        if (m == 50000) {
          Serial.println(i);
          m = 0;
        }
      }
    }
  } else {
    elapsedTime = millis() - elapsedTime;
    Serial.print("Elapsed time: ");
    Serial.println(elapsedTime);
    while (true) {}
  }
  
}
