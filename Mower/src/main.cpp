#include <Arduino.h>
#include <MeAuriga.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeUltrasonicSensor UltraSonic(PORT_10);

#define MAX_SPEED 60
#define MIN_SPEED 40
#define STOP 0

void isr_process_encoder1() {
    if(digitalRead(Encoder_1.getPortB()) == 0) {
        Encoder_1.pulsePosMinus();
    }
    else {
        Encoder_1.pulsePosPlus();;
    }
}

void isr_process_encoder2() {
    if(digitalRead(Encoder_2.getPortB()) == 0) {
        Encoder_2.pulsePosMinus();
    }
    else {
        Encoder_2.pulsePosPlus();
    }
}

void setup()
{
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
    if(Serial.available()) {
        char a = Serial.read();
        Serial.println(a);
        switch(a) {
            case '0': //Stop
                Encoder_1.setTarPWM(STOP);
                Encoder_2.setTarPWM(STOP);
                break;
            case '1': //Forward
                Encoder_1.setTarPWM(-MAX_SPEED);
                Encoder_2.setTarPWM(MAX_SPEED);
                break;
            case '2': //Backward
                Encoder_1.setTarPWM(MAX_SPEED);
                Encoder_2.setTarPWM(-MAX_SPEED);
                break;
            case '3': //Right
                Encoder_1.setTarPWM(MAX_SPEED);
                Encoder_2.setTarPWM(MAX_SPEED);
                break;
            case '4': //Left
                Encoder_1.setTarPWM(-MAX_SPEED);
                Encoder_2.setTarPWM(-MAX_SPEED);
                break;
            default:
                break;
        }
    }

    delay(10);
    if (UltraSonic.distanceCm() < 20) {
        buzzerOn();
        delay(80);
        buzzerOff();
    }

    Encoder_1.loop();
    Encoder_2.loop();
}

