/* * * * INCLUDES * * * */
#include <MeAuriga.h>
#include <Wire.h>

/* * * * DEFINES * * * */
#define STOP '0'
#define FORWARD '1'
#define TURN_RIGHT '2'
#define TURN_LEFT '3'
#define REVERSE '4'
#define AUTO '5'

/* * * * * * * CONSTANTS * * * * * * */
const byte noLineDetection = 3;
const byte collisionDistanceCm15 = 15;
const byte collisionDistanceCm30 = 30;
const byte collisionDistanceCm45 = 45;
const byte ultraSonicDifferenceCm = 2;
const int robotSpeed = 60;

/* * * * * GLOBAL VARIABLES * * * * */
float startPos = 0;
byte previousUltraSensorValue = 0;
char driveCommand = AUTO; //AUTO is standard, STOP when testing
bool flagUltraSonicCm30 = false;
bool flagUltraSonicCm45 = false;
MeEncoderOnBoard rightMotor(SLOT1);
MeEncoderOnBoard leftMotor(SLOT2);
MeLineFollower lineFinder(PORT_9);
MeUltrasonicSensor ultraSonic(PORT_10);
MeGyro gyro(1, 0x69);

/* * * * * * * * * * FUNCTIONS * * * * * * * * * */
void autoDrive(void);
void manualDrive(void);
void setMotorSpeed(int rightMotorSpeed, int leftMotorSpeed);
bool collisionAvoidance(void);
float calculateDistance();
int calculateAngle(char direction);
void printPathToRpi(char direction);
void waitForRpi(void);

void isr_process_encoder1(void) {
    if (digitalRead(rightMotor.getPortB()) == 0)
        rightMotor.pulsePosMinus();
    else
        rightMotor.pulsePosPlus();
}

void isr_process_encoder2(void) {
    if (digitalRead(leftMotor.getPortB()) == 0)
        leftMotor.pulsePosMinus();
    else
        leftMotor.pulsePosPlus();
}

/* * * * * setup * * * * * *
 * Initiate the mower. 
 * * * * * * * * * * * * * */
void setup(void) {
    attachInterrupt(rightMotor.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(leftMotor.getIntNum(), isr_process_encoder2, RISING);

    Serial.begin(115200);
    Serial2.begin(115200);
    gyro.begin();

    waitForRpi();

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

/* * * * loop * * * *
 * Handle driving.
 * * * * * * * * * */
void loop(void) {
    gyro.update();
    
    manualDrive();
    if (driveCommand == AUTO) {
        autoDrive();
    }

    rightMotor.loop();
    leftMotor.loop();
}

/* * * * * * * autoDrive * * * * * * * *
 * State machine to drive autonomously.
 * * * * * * * * * * * * * * * * * * * */
void autoDrive(void) {
    static char autoDriveState = FORWARD;
    static int lineSensor;
    float wheelTurns = (float(leftMotor.getCurPos() - startPos) / 360);

    switch (autoDriveState) {
        case FORWARD:
            if (collisionAvoidance() || lineFinder.readSensors() != noLineDetection) {
                printPathToRpi(FORWARD);
                autoDriveState = REVERSE;
                lineSensor = lineFinder.readSensors();
                flagUltraSonicCm30 = false;
                flagUltraSonicCm45 = false;
            }
            else if (ultraSonic.distanceCm() >= collisionDistanceCm30 
                    && ultraSonic.distanceCm() <= collisionDistanceCm30 + ultraSonicDifferenceCm
                    && flagUltraSonicCm30 == false){
                printPathToRpi(FORWARD);
                flagUltraSonicCm30 = true;
            }
            else if (ultraSonic.distanceCm() >= collisionDistanceCm45 
                    && ultraSonic.distanceCm() <= collisionDistanceCm45 + ultraSonicDifferenceCm
                    && flagUltraSonicCm45 == false){
                printPathToRpi(FORWARD);
                flagUltraSonicCm45 = true;
            }
            setMotorSpeed(-robotSpeed + 20, robotSpeed - 20);
            break;

        case TURN_RIGHT:
            if (wheelTurns >= 0.3) {
                autoDriveState = FORWARD;
                startPos = leftMotor.getCurPos();
            }
            setMotorSpeed(robotSpeed, robotSpeed);
            break;

        case TURN_LEFT:
            if (wheelTurns <= -0.5) {
                autoDriveState = FORWARD;
                startPos = leftMotor.getCurPos();
            }
            setMotorSpeed(-robotSpeed, -robotSpeed);
            break;

        case REVERSE:
            if (wheelTurns <= -1 || (lineFinder.readSensors() != noLineDetection && wheelTurns <= -0.5)) {
                if (lineSensor == 1) 
                    autoDriveState = TURN_RIGHT;
                else 
                    autoDriveState = TURN_LEFT;
                
                printPathToRpi(REVERSE);
            }
            setMotorSpeed(robotSpeed, -robotSpeed);
            break;

        default:
            break;
    }
}

/* * * * * * * * * manualDrive * * * * * * * * * * * * *
 * Handle incoming commands from app to manually drive.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void manualDrive(void) {
    static char previousState = AUTO;

    if (Serial.available()) {
        driveCommand = Serial.read();
        Serial.readString(); //clear
        switch (driveCommand) {
            case STOP:
                if (previousState == REVERSE || previousState == FORWARD)
                    printPathToRpi(previousState);
                startPos = leftMotor.getCurPos();
                setMotorSpeed(0, 0);
                previousState = STOP;
                break;

            case FORWARD:
                if (previousState == REVERSE)
                    printPathToRpi(previousState);
                startPos = leftMotor.getCurPos();
                setMotorSpeed(-robotSpeed, robotSpeed);
                previousState = FORWARD;
                break;

            case TURN_RIGHT:
                if (previousState == REVERSE || previousState == FORWARD)
                    printPathToRpi(previousState);
                setMotorSpeed(robotSpeed, robotSpeed);
                previousState = TURN_RIGHT;
                break;

            case TURN_LEFT:
                if (previousState == REVERSE || previousState == FORWARD)
                    printPathToRpi(previousState);
                setMotorSpeed(-robotSpeed, -robotSpeed);
                previousState = TURN_LEFT;
                break;

            case REVERSE:
                if (previousState == FORWARD)
                    printPathToRpi(previousState);
                startPos = leftMotor.getCurPos();
                setMotorSpeed(robotSpeed, -robotSpeed);
                previousState = REVERSE;
                break;

            case AUTO:
                previousState = AUTO;
                break;

            default:
                break;
        }
    }
    else if (previousState == FORWARD){
        if (collisionAvoidance()) {
            printPathToRpi(previousState);
            setMotorSpeed(0, 0);
            previousState = STOP;
        }
        else if (ultraSonic.distanceCm() >= collisionDistanceCm30 
                && ultraSonic.distanceCm() <= collisionDistanceCm30 + ultraSonicDifferenceCm
                && flagUltraSonicCm30 == false){
            printPathToRpi(previousState);
            flagUltraSonicCm30 = true;
        }
        else if(ultraSonic.distanceCm() >= collisionDistanceCm45 
                && ultraSonic.distanceCm() <= collisionDistanceCm45 + ultraSonicDifferenceCm
                && flagUltraSonicCm45 == false){
            printPathToRpi(previousState);
            flagUltraSonicCm45 = true;
        }
    }
    else if (previousState != AUTO){
        flagUltraSonicCm30 = false;
        flagUltraSonicCm45 = false;
    }
}

/* * * setMotorSpeed * * *
 * Sets the motors speed.
 * * * * * * * * * * * * */
void setMotorSpeed(int rightMotorSpeed, int leftMotorSpeed) {
    rightMotor.setMotorPwm(rightMotorSpeed);
    leftMotor.setMotorPwm(leftMotorSpeed);
}

/* * * * * collisionAvoidance * * * * * *
 * Checks if collision avoidance occurs.
 * * * * * * * * * * * * * * * * * * * */
bool collisionAvoidance(void) {
    return ultraSonic.distanceCm() < collisionDistanceCm15;
}

/* * * * * * * * calculateDistance * * * * * * * * * * *
 * Calculates distance between startPos and currentPos.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * */
float calculateDistance() {
    int wheelCircumference = 20;
    float distance = (float(leftMotor.getCurPos() - startPos) / 360) * wheelCircumference;
    startPos = leftMotor.getCurPos();
    return distance;
}

/* * * * * * * calculateAngle * * * * * * * *
 * Calculates angle depending on direction.
 * * * * * * * * * * * * * * * * * * * * * */
int calculateAngle(char direction) {
    int forwardAngle = 360 - (gyro.getAngleZ() + 180);

    if (direction == FORWARD) 
        return forwardAngle;
    else if (direction == REVERSE)
        return (forwardAngle + 180) % 360;
}

/* * * * * * * * * * * printPathToRpi * * * * * * * * * * * * * * * 
 * Prints distance, angle and if collision avoidance occurs to RPi.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void printPathToRpi(char direction) {
    String s = "";
    if (collisionAvoidance() && direction == FORWARD)
        s = (String)calculateDistance() + "," + (String)calculateAngle(direction) + "," + (String)ultraSonic.distanceCm() + "," + "T";
    else
        s = (String)calculateDistance() + "," + (String)calculateAngle(direction) + "," + (String)ultraSonic.distanceCm() + "," + "F";
    Serial2.print(s);
}

/* * * * * waitForRpi * * * * * *
 * Wait for the RPi to be ready.
 * * * * * * * * * * * * * * * */
void waitForRpi(void) {
    char rpiReady = 'N';
    do {
        Serial2.print("R");
        if (Serial2.available()) {
            rpiReady = Serial2.read();
        }
        delay(500);
    } while (rpiReady != 'R'); 
}