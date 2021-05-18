#include <MeAuriga.h>
#include <Wire.h>

#define STOP '0'
#define FORWARD '1'
#define TURN_RIGHT '2'
#define TURN_LEFT '3'
#define REVERSE '4'
#define AUTO '5'

const byte noLineDetection = 3;
const int wheelDiameter = 20;
const int distanceBeforeCollision = 15;

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultraSonic(PORT_10);
MeGyro gyro(1, 0x69);

int robotSpeed = 60;
int angle;
int startPos = 0;
float distance = 0;
char driveCommand = AUTO;
char previousState = AUTO;

void printToRpi(float distance, int angle, bool collisionAvoidance);
void drive(void);
void autoDrive(void);
void manualDrive();
bool collisionAvoidance(void);

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

    /*
    char rpiReady = n;
    do {
        if (Serial2.available()) {
            rpiReady = Serial2.read();
            Serial2.readstring();
        }
    } while (rpiReady != r)
    */

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

}

void loop()
{
    gyro.update();
    drive();

    Encoder_1.loop();
    Encoder_2.loop();
}

void drive(void) {
    angle = 360 - (gyro.getAngleZ() + 180);

    if (Serial.available()) {
        driveCommand = Serial.read();
        Serial.readString(); //clear
        manualDrive();
    }
    else if (driveCommand == AUTO) {
        autoDrive();
        previousState = AUTO;
    }
    else if (collisionAvoidance())
    {
        if (previousState == REVERSE)
        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
            angle = (angle + 180) % 360;
            printToRpi(distance, angle, false);
        }
        else if (previousState == FORWARD)
        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
            printToRpi(distance, angle, true);
        }
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        previousState = STOP;
    }
}

void autoDrive(void)
{
    static int sensorState = 0;
    static char b = FORWARD;
    static int lineSensor;
    static float wheelTurns = 0;

    wheelTurns = (float(Encoder_2.getCurPos() - startPos) / 360);
    switch (b)
    {
    case FORWARD:
        if (collisionAvoidance() || lineFinder.readSensors() != noLineDetection)
        {
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
            printToRpi(distance, angle, collisionAvoidance());
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
            
            distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
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

void manualDrive(void) {
    switch (driveCommand) {
        case STOP:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter; 
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(0);
            Encoder_2.setMotorPwm(0);
            previousState = STOP;
            break;

        case FORWARD:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
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
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(robotSpeed);
            Encoder_2.setMotorPwm(robotSpeed);
            previousState = TURN_RIGHT;
            break;

        case TURN_LEFT:
            if (previousState == REVERSE)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
                angle = (angle + 180) % 360;
                printToRpi(distance, angle, false);
            }
            else if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
                printToRpi(distance, angle, false);
            }
            Encoder_1.setMotorPwm(-robotSpeed);
            Encoder_2.setMotorPwm(-robotSpeed);
            previousState = TURN_LEFT;
            break;

        case REVERSE:
            if (previousState == FORWARD)
            {
                distance = (float(Encoder_2.getCurPos() - startPos) / 360) * wheelDiameter;
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

bool collisionAvoidance() {
    return ultraSonic.distanceCm() < distanceBeforeCollision;
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