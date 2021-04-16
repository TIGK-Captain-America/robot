#include <MeAuriga.h>

#define STOP '0'
#define FORWARD '1'
#define TURN_RIGHT '2'
#define TURN_LEFT '3'
#define REVERSE '4'
#define AUTO '5'


MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultraSonic(PORT_10);

int sensorState = 0;
int robotSpeed = 60;

void autoDrive(void) {
    static char b = FORWARD;
    static int count = 0;
    static int sensorCollision;

    switch(b) {
        case FORWARD:
            if (ultraSonic.distanceCm() < 30 || lineFinder.readSensors() != 3) {
                b = REVERSE;
                sensorCollision = lineFinder.readSensors();
            }
            else {
                Encoder_1.setTarPWM(-robotSpeed+20);
                Encoder_2.setTarPWM(robotSpeed-20);
            }
            break;
        
        case TURN_RIGHT:
            if (count >= 3000) {
                b = FORWARD;
                count = 0;
            }
            Encoder_1.setTarPWM(robotSpeed);
            Encoder_2.setTarPWM(robotSpeed);
            count++;
            break;
        
        case TURN_LEFT:
            if (count >= 3000) {
            b = FORWARD;
            count = 0;
            }
            Encoder_1.setTarPWM(-robotSpeed);
            Encoder_2.setTarPWM(-robotSpeed);
            count++;
            break;
        
        case REVERSE:
            if (count >= 4000 || (lineFinder.readSensors() != 3 && count > 2000)) {
                if (sensorCollision == 1)
                    b = TURN_RIGHT;
                else
                    b = TURN_LEFT;
                
                count = 0;
            }
            Encoder_1.setTarPWM(robotSpeed);
            Encoder_2.setTarPWM(-robotSpeed);
            count++;
            break;
        
        default:
            break;
    }
}

void isr_process_encoder1(void) {
    if(digitalRead(Encoder_1.getPortB()) == 0) 
        Encoder_1.pulsePosMinus();
    else 
        Encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void) {
    if(digitalRead(Encoder_2.getPortB()) == 0)
        Encoder_2.pulsePosMinus();
    else
        Encoder_2.pulsePosPlus();
}

void setup() {
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    Serial.begin(115200);
    
    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

void loop() {
    static char a;

    if (Serial.available()) {
        a = Serial.read();
        Serial.readString();
    }

    switch(a) {
        case STOP:
            Encoder_1.setTarPWM(0);
            Encoder_2.setTarPWM(0);
            break;
        
        case FORWARD:
            if (ultraSonic.distanceCm() < 30 || lineFinder.readSensors() != 3)
                a = STOP;
            else {
                Encoder_1.setTarPWM(-robotSpeed);
                Encoder_2.setTarPWM(robotSpeed);
            }
            break;
        
        case TURN_RIGHT:
            Encoder_1.setTarPWM(robotSpeed);
            Encoder_2.setTarPWM(robotSpeed);
            break;
        
        case TURN_LEFT:
            Encoder_1.setTarPWM(-robotSpeed);
            Encoder_2.setTarPWM(-robotSpeed);
            break;
        
        case REVERSE:
            Encoder_1.setTarPWM(robotSpeed);
            Encoder_2.setTarPWM(-robotSpeed);
            break;

        case AUTO:
            autoDrive();
            break;
            
        default:
            break;
    }
  
  Encoder_1.loop();
  Encoder_2.loop();
}
