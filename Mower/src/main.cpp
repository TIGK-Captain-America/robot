#include <MeAuriga.h>
#include <Wire.h>

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
MeGyro gyro(1, 0x69);

int sensorState = 0;
int robotSpeed = 60;
int angle;
int startPos = 0;
float distance = 0;
const byte noLineDetection = 3;
const int collisionAvoidanceDistance = 10;


void printToRpi(float distance, int angle, bool collisionAvoidance);

void autoDrive(void)
{
    static char b = FORWARD;
    static int lineSensor;
    static float wheelTurns = 0;

    /*Serial.print("wheelTurns: ");
    Serial.println(wheelTurns);
    Serial.print("Line sensor: ");
    Serial.println(lineFinder.readSensors());*/
    wheelTurns = (float(Encoder_2.getCurPos() - startPos) / 360);
    switch (b)
    {
    case FORWARD:
        if (ultraSonic.distanceCm() < collisionAvoidanceDistance || lineFinder.readSensors() != noLineDetection)

        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
            if (ultraSonic.distanceCm() < collisionAvoidanceDistance)
                printToRpi(distance, angle, true);
            else
                printToRpi(distance, angle, false);
            b = REVERSE;
            lineSensor = lineFinder.readSensors();
            startPos = Encoder_2.getCurPos();
        }
        else
        {
            Encoder_1.setMotorPwm(-robotSpeed + 20);
            Encoder_2.setMotorPwm(robotSpeed - 20);
        }
        wheelTurns = 0;
        break;

    case TURN_RIGHT:
        if (wheelTurns >= 0.3)
        {
            b = FORWARD;
            wheelTurns = 0;
            startPos = Encoder_2.getCurPos();
        }
        Encoder_1.setMotorPwm(robotSpeed);
        Encoder_2.setMotorPwm(robotSpeed);
        break;

    case TURN_LEFT:
        if (wheelTurns <= -0.5)
        {
            b = FORWARD;
            wheelTurns = 0;
            startPos = Encoder_2.getCurPos();
        }
        Encoder_1.setMotorPwm(-robotSpeed);
        Encoder_2.setMotorPwm(-robotSpeed);
        break;

    case REVERSE:
        if (wheelTurns <= -1 || (lineFinder.readSensors() != noLineDetection && wheelTurns <= -0.5))
        {
            if (lineSensor == 1) 
                b = TURN_RIGHT;
            else 
                b = TURN_LEFT;
            
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
            angle = (angle + 180) % 360;
            printToRpi(distance, angle, false);
            startPos = Encoder_2.getCurPos();
            wheelTurns = 0;
        }
        Encoder_1.setMotorPwm(robotSpeed);
        Encoder_2.setMotorPwm(-robotSpeed);
        break;

    default:
        break;
    }
}

void isr_process_encoder1(void)
{
    if (digitalRead(Encoder_1.getPortB()) == 0)
        Encoder_1.pulsePosMinus();
    else
        Encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void)
{
    if (digitalRead(Encoder_2.getPortB()) == 0)
        Encoder_2.pulsePosMinus();
    else
        Encoder_2.pulsePosPlus();
}

void setup()
{
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    Serial.begin(115200);
    Serial2.begin(9600);
    gyro.begin();

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

void loop()
{
    gyro.update();
    angle = 360 - (gyro.getAngleZ() + 180);
    static char a;
    static char previousState;

    if (Serial.available())
    {
        a = Serial.read();
        Serial.readString();

        switch (a)
        {
        case STOP:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20; //20 = omkrets på hjul
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(0);
            Encoder_2.setMotorPwm(0);
            previousState = STOP;
            break;

        case FORWARD:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            startPos = Encoder_2.getCurPos();
            Encoder_1.setMotorPwm(-robotSpeed);
            Encoder_2.setMotorPwm(robotSpeed);
            previousState = FORWARD;
            break;

        case TURN_RIGHT:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(robotSpeed);
            Encoder_2.setMotorPwm(robotSpeed);
            previousState = TURN_RIGHT;
            break;

        case TURN_LEFT:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(-robotSpeed);
            Encoder_2.setMotorPwm(-robotSpeed);
            previousState = TURN_LEFT;
            break;

        case REVERSE:
            if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
                printToRpi(distance, angle, false);
            }
            startPos = Encoder_2.getCurPos();
            Encoder_1.setMotorPwm(robotSpeed);
            Encoder_2.setMotorPwm(-robotSpeed);
            previousState = REVERSE;
            break;

        default:
            break;
        }
    }

    if (a == AUTO)
    {
        autoDrive();
        previousState = AUTO;
    }
    if (ultraSonic.distanceCm() <= collisionAvoidanceDistance && previousState != AUTO && previousState == FORWARD)
    {
        if (previousState == REVERSE)
        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20; //20 = omkrets på hjul
            angle = (angle + 180) % 360;
            printToRpi(distance, angle, false);
        }
        else if (previousState == FORWARD)
        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * 20;
            printToRpi(distance, angle, true);
        }
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        previousState = STOP;
    }

    Encoder_1.loop();
    Encoder_2.loop();
}

void printToRpi(float distance, int angle, bool collisionAvoidance) {
    Serial2.print(distance);
    Serial2.print(",");
    Serial2.print(angle);
    Serial2.print(",");
    if (collisionAvoidance)
        Serial2.print("T");
    else
        Serial2.print("F");
}