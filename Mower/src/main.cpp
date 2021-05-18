#include <MeAuriga.h>
#include <Wire.h>

#define STOP '0'
#define FORWARD '1'
#define TURN_RIGHT '2'
#define TURN_LEFT '3'
#define REVERSE '4'
#define AUTO '5'

const byte noLineDetection = 3;
const int distanceBeforeCollision = 15;

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultraSonic(PORT_10);
MeGyro gyro(1, 0x69);

int robotSpeed = 60;
float startPos = 0;
char driveCommand = AUTO;
char previousState = AUTO;

void drive(void);
void autoDrive(void);
void manualDrive(void);
bool collisionAvoidance(void);
float calculateDistance(float startPos, float endPos);
int calculateAngle(char direction);
void printPathToRpi(float startPos, char direction);

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

    /*char rpiReady = 'N';
    do {
        if (Serial2.available()) {
            rpiReady = Serial2.read();
        }
    } while (rpiReady != 'R');
    
    Serial.println("Ready for commands");*/

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

}

void loop()
{
    drive();

    Encoder_1.loop();
    Encoder_2.loop();
}

void drive(void) {
    gyro.update();

    if (Serial.available()) {
        driveCommand = Serial.read();
        Serial.readString(); //clear
        manualDrive();
    }
    else if (driveCommand == AUTO) {
        autoDrive();
        previousState = AUTO;
    }
    else if (collisionAvoidance() && previousState == FORWARD)
    {
        printPathToRpi(startPos, FORWARD);
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        previousState = STOP;
    }
}

void autoDrive(void)
{
    static char autoDriveState = FORWARD;
    static int lineSensor;
    float wheelTurns = (float(Encoder_2.getCurPos() - startPos) / 360);

    switch (autoDriveState)
    {
    case FORWARD:
        if (collisionAvoidance() || lineFinder.readSensors() != noLineDetection)
        {
            printPathToRpi(startPos, FORWARD);
            autoDriveState = REVERSE;
            lineSensor = lineFinder.readSensors();
            startPos = Encoder_2.getCurPos();
        }
        Encoder_1.setMotorPwm(-robotSpeed + 20);
        Encoder_2.setMotorPwm(robotSpeed - 20);
        break;

    case TURN_RIGHT:
        if (wheelTurns >= 0.3)
        {
            autoDriveState = FORWARD;
            startPos = Encoder_2.getCurPos();
        }
        Encoder_1.setMotorPwm(robotSpeed);
        Encoder_2.setMotorPwm(robotSpeed);
        break;

    case TURN_LEFT:
        if (wheelTurns <= -0.5)
        {
            autoDriveState = FORWARD;
            startPos = Encoder_2.getCurPos();
        }
        Encoder_1.setMotorPwm(-robotSpeed);
        Encoder_2.setMotorPwm(-robotSpeed);
        break;

    case REVERSE:
        if (wheelTurns <= -1 || (lineFinder.readSensors() != noLineDetection && wheelTurns <= -0.5))
        {
            if (lineSensor == 1) 
                autoDriveState = TURN_RIGHT;
            else 
                autoDriveState = TURN_LEFT;
            
            printPathToRpi(startPos, REVERSE);
            startPos = Encoder_2.getCurPos();
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
            if (previousState == REVERSE || previousState == FORWARD)
                printPathToRpi(startPos, previousState);
            startPos = Encoder_2.getCurPos();
            Encoder_1.setMotorPwm(0);
            Encoder_2.setMotorPwm(0);
            previousState = STOP;
            break;

        case FORWARD:
            if (previousState == REVERSE)
                printPathToRpi(startPos, REVERSE);
            startPos = Encoder_2.getCurPos();
            Encoder_1.setMotorPwm(-robotSpeed);
            Encoder_2.setMotorPwm(robotSpeed);
            previousState = FORWARD;
            break;

        case TURN_RIGHT:
            if (previousState == REVERSE || previousState == FORWARD)
                printPathToRpi(startPos, previousState);
            Encoder_1.setMotorPwm(robotSpeed);
            Encoder_2.setMotorPwm(robotSpeed);
            previousState = TURN_RIGHT;
            break;

        case TURN_LEFT:
            if (previousState == REVERSE || previousState == FORWARD)
                printPathToRpi(startPos, previousState);
            Encoder_1.setMotorPwm(-robotSpeed);
            Encoder_2.setMotorPwm(-robotSpeed);
            previousState = TURN_LEFT;
            break;

        case REVERSE:
            if (previousState == FORWARD)
                printPathToRpi(startPos, FORWARD);
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

float calculateDistance(float startPos, float currentPos) {
    int wheelDiameter = 20;
    return (float(currentPos - startPos) / 360) * wheelDiameter;
}

int calculateAngle(char direction) {
    int forwardAngle = 360 - (gyro.getAngleZ() + 180);

    if (direction == FORWARD) 
        return forwardAngle;
    else if (direction == REVERSE)
        return (forwardAngle + 180) % 360;
}

void printPathToRpi(float startPos, char direction) {
    Serial2.print(calculateDistance(startPos, Encoder_2.getCurPos()));
    Serial2.print(",");
    Serial2.print(calculateAngle(direction));
    Serial2.print(",");
    if (collisionAvoidance() && direction == FORWARD)
        Serial2.print("T");
    else
        Serial2.print("F");
}