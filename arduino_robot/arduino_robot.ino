#include "myDebug.h"


// prh additions



#define WITH_RADIO  1
#define WITH_SONAR  0

#if WITH_RADIO
    #include <VirtualWire.h>
        // #define PIN_RECV_DATA   11
        // #define PIN_XMIT_DATA   12
        //      receive radio data (next to VCC) goes to pin 11
        //      xmit sends out on pin 12
#endif
#if WITH_SONAR
    #define SONAR_ECHO  3
    #define SONAR_TRIG  4

    #define SONAR_READ_MS     300       // upto 8 times per second
    int sonar_state = 0;
    uint32_t sonar_timer = 0;           // ms of last read
    uint32_t sonar_start_time = 0;      // us of pulse
    uint32_t sonar_micros = 0;          // us of echo

    void sonarISR()
    {
        // display(0,"sonarISR()",0);
        if (sonar_state == 1)
        {
            sonar_micros = micros() - sonar_start_time;
            sonar_state = 2;
        }
    }
#endif

bool prh_started = 0;
bool prh_running = 0;
float prh_direction_bias = 0.0;

void display_float(const char *s, float f)
    //  show a float with one decimal of precision
{
    int times_ten = (f * 10.0);
    display(0,"%s = %d.%d",s,times_ten/10,times_ten%10);
}


// prh - changed ino filename from AmBOT_final_nano to match arduino-self-balancing-robot-master project name


#include "PID_v1.h"                 // prh - changed from angle brackets
#include "LMotorController.h"       // prh - changed from angle brackets
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 183.4;  // prh was 175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
#if 0
    // originals:
    double Kp = 50;
    double Kd = 1.4;
    double Ki = 60;
#else
    double Kp = 80;
    double Kd = 1;
    double Ki = 15;
#endif
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.7;          // prh original value = 0.6
double motorSpeedFactorRight = 0.7;         // prh original value = 0.5

//MOTOR CONTROLLER
int ENA = 5;        // prh - changed to my pin settings (I moved ENB to 6 cuz radio made 10 not work for PWM)
int IN1 = 7;
int IN2 = 8;
int IN3 = 9;
int IN4 = 10;
int ENB = 6;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        //setup PID

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(4);   // prh 5);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // prh addition
    #if WITH_RADIO
        vw_setup(2000);
        vw_rx_start();
        display(0,"radio started",0);
    #endif
    #if WITH_SONAR
        pinMode(SONAR_TRIG,OUTPUT);
        attachInterrupt(digitalPinToInterrupt(SONAR_ECHO), sonarISR, FALLING);
    #endif

}






void loop()
{
    // prh - crude attempt at control

    int command = 0;

    #if WITH_RADIO
        byte wireless_bytes = sizeof(command);
        if (0)
        {
            warning(0,"waiting for radio",0);
            vw_wait_rx();
        }

        if (vw_get_message((byte *) &command, &wireless_bytes))
        {
            display(0,"RADIO received '%c' chr(%d)",command>=32?command:' ',command);
        }
    #endif

    if (!command && Serial.available())
    {
        command = Serial.read();
        display(0,"Got serial command: '%c'",command);
    }

    if (command == 'p')
    {
        Kp += 5;
        display_float("Kp",Kp);
        pid.SetTunings(Kp,Ki,Kd);
    }
    else if (command == 'P')
    {
        Kp -= 5;
        display_float("Kp",Kp);
        pid.SetTunings(Kp,Ki,Kd);
    }
    else if (command == 'i')
    {
        Ki += 5;
        display_float("Ki",Ki);
        pid.SetTunings(Kp,Ki,Kd);
    }
    else if (command == 'I')
    {
        Ki -= 5;
        display_float("Ki",Ki);
        pid.SetTunings(Kp,Ki,Kd);
    }
    else if (command == 'd')
    {
        Kd += 5;
        display_float("Kd",Kd);
        pid.SetTunings(Kp,Ki,Kd);
    }
    else if (command == 'D')
    {
        Kd -= 5;
        display_float("Kd",Kd);
        pid.SetTunings(Kp,Ki,Kd);
    }


    if (command == 's')
    {
        display(0,"%s",prh_running?"STOP":"START");
        prh_running = !prh_running;
        if (!prh_running)
            motorController.stopMoving();
        setpoint = originalSetpoint;
    }
    if (command == 'f')
    {
        prh_direction_bias += 0.2;
        float new_setpoint = originalSetpoint - prh_direction_bias;
        display_float("FORWARD",prh_direction_bias);
        setpoint = new_setpoint;
    }
    else if (command == 'b')
    {
        prh_direction_bias -= 0.2;
        float new_setpoint = originalSetpoint - prh_direction_bias;
        display_float("BACKWARD",prh_direction_bias);
        setpoint = new_setpoint;
    }



    // end of prh changes


    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors

        if (prh_running)
        {
            #if WITH_SONAR

                if (!sonar_state)
                {
                    if (millis() > sonar_timer + SONAR_READ_MS)
                    {
                        // display(0,"pulse sonar",0);
                        sonar_micros = 0;
                        digitalWrite(SONAR_TRIG, 0);
                        delayMicroseconds(2);
                        digitalWrite(SONAR_TRIG, 1);
                        delayMicroseconds(10);
                        digitalWrite(SONAR_TRIG, 0);
                        while (!digitalRead(SONAR_ECHO)) {}
                        sonar_start_time = micros();
                        sonar_state = 1;
                    }
                }
                else if (sonar_state == 2)
                {
                    int distance = sonar_micros * 0.034 / 2;        // distance in CM
                    display(0,"distance=%d duration=%d",distance,sonar_micros);
                    sonar_timer = millis();
                    sonar_state = 0;
                }
            #endif

            pid.Compute();
            motorController.move(output, MIN_ABS_SPEED);
        }

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);

        #endif
        input = ypr[1] * 180/M_PI + 180;

        // prh - fall and start detection

        int degrees = abs(ypr[1] * 180/M_PI);
        // display(0,"degrees %d",degrees);

        if (!prh_started && degrees<10)
        {
            display(0,"STARTED",0);
            prh_started = 1;
            prh_running = 1;
        }
        else if (prh_running && degrees>50)
        {
            display(0,"CRASH!!",0);
            prh_started = 0;
            prh_running = 0;
            motorController.stopMoving();
            setpoint = originalSetpoint;
        }

   }
}
