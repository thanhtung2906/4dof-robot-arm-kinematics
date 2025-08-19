#include <Servo.h>
Servo servoShoulder;
Servo servoElbow;
Servo servoBase;
void moveServoSlow(Servo &servo, int targetPos, int delayTime = 10) {
    int currentPos = servo.read();   // đọc vị trí hiện tại của servo
    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            servo.write(pos);
            delay(delayTime);        // chỉnh tốc độ (ms)
        }
    } else {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            servo.write(pos);
            delay(delayTime);
        }
    }
}

void setup() {
    Serial.begin(9600);
    servoShoulder.attach(5);
    servoElbow.attach(6);
    servoBase.attach(4);
    
  
    
}

void loop() {
    if (Serial.available()) {
        char id = Serial.read();      // ký tự định danh servo
        int pos = Serial.parseInt();  // góc servo

        if (id == 'A') moveServoSlow(servoShoulder, pos, 15);
        if (id == 'B') moveServoSlow(servoElbow, pos, 15);
        if (id == 'C') moveServoSlow(servoBase, pos, 15);
        
    }
}
