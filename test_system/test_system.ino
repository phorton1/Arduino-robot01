#include "myDebug.h"

// basic LM298 MPU6050 wiring diagram from https://maker.pro/arduino/projects/build-arduino-self-balancing-robot

#define WITH_RADIO  1
#define WITH_GYRO   0
#define WITH_SONAR  0
#define WITH_MOTOR  0



#if WITH_RADIO
    #include <VirtualWire.h>
        // #define PIN_RECV_DATA   11
        // #define PIN_XMIT_DATA   12
        //      receive radio data (next to VCC) goes to pin 11
        //      xmit sends out on pin 12
#endif

#if WITH_GYRO
    #include <Wire.h>
    #include "MPU6050.h"
        // Gyro uses SDA/SCL
        // #define SDA    A4
        // #define SCL    A5
#endif


#define DEFAULT_SPEED 128

#if WITH_MOTOR
    // right is on A12 left is on B34
    #define MOTOR_ENA  5
    #define MOTOR_IN1  7
    #define MOTOR_IN2  8
    #define MOTOR_IN3  9
    #define MOTOR_IN4  10
    #define MOTOR_ENB  6
#endif

#if WITH_SONAR
    #define SONAR_ECHO  3
    #define SONAR_TRIG  4
    #define SONAR_READ_MS     1000
    #define SONAR_INTERRUPT   1

    #if SONAR_INTERRUPT
        int sonar_state = 0;
        uint32_t sonar_start_time = 0;      // us of pulse
        uint32_t sonar_micros = 0;          // us of echo

        void sonarISR()
        {
            display(0,"sonarISR()",0);
            if (sonar_state == 1)
            {
                sonar_micros = micros() - sonar_start_time;
                sonar_state = 2;
            }
        }
    #endif

#endif

#if WITH_GYRO
    MPU6050 mpu;
        //
#endif



void setup()
{
    Serial.begin(115200);

    #if WITH_RADIO
        // conflicting with MOTOR on D10

        vw_setup(2000);
        vw_rx_start();
        display(0,"radio started",0);
    #endif


	#if WITH_MOTOR
        pinMode(MOTOR_ENA, OUTPUT);
        pinMode(MOTOR_IN1, OUTPUT);
        pinMode(MOTOR_IN2, OUTPUT);
        pinMode(MOTOR_ENB, OUTPUT);
        pinMode(MOTOR_IN4, OUTPUT);
        pinMode(MOTOR_ENB, OUTPUT);

        analogWrite(MOTOR_ENA,0);
        digitalWrite(MOTOR_IN1,0);
        digitalWrite(MOTOR_IN2,0);
        analogWrite(MOTOR_ENB,0);
        digitalWrite(MOTOR_IN3,0);
        digitalWrite(MOTOR_IN4,0);
    #endif

    #if WITH_SONAR
        pinMode(SONAR_TRIG,OUTPUT);
        pinMode(SONAR_ECHO,INPUT);
        #if SONAR_INTERRUPT
            attachInterrupt(digitalPinToInterrupt(SONAR_ECHO), sonarISR, FALLING); // CHANGE);
        #endif
    #endif

    pinMode(13,OUTPUT);
    digitalWrite(13,0);

    delay(1000);
    display(0,"robot01.ino setup()",0);

    #if WITH_GYRO
        int counter = 10;
        while(counter-- && !mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
        {
            my_error("Could not find a valid MPU6050 sensor, check wiring!",0);
            delay(500);
        }
        if (counter < 0)
            warning(0,"not starting gyro",0);
        else
            display(0,"gyro started",0);
    #endif


    display(0,"robot01.ino started",0);

}



void loop()
{
    static uint32_t flasher_time = 0;
    static int flasher = 0;
    if (millis() > flasher_time + 1000)
    {
        flasher = flasher ? 0 : 1;
        digitalWrite(13,flasher);
        flasher_time = millis();
    }


    #if WITH_SONAR
        static uint32_t sonar_time = 0;
        if (
            #if SONAR_INTERRUPT
                !sonar_state &&
            #endif
            millis() > sonar_time + SONAR_READ_MS)
        {
            // clear the trigger pin for 2 us then
            // sets the trigPin HIGH for 10 us
            //
            // delays on the order of 10 milliseconds
            // the maximum range is about 450cm  (4.5 meters)
            //      or about 30000us (30 ms)
            // with a delay 588us per cm
            //      20 cm (8 inches) == about 1200 us or 1.2ms
            //      100 cm (40 inches) == about 6000 us or 6ms
            //      200 cm (80 inches, 2m) == about 12000 us or 12ms
            //
            //  #define MAX_USEFUL_SONAR_DELAY 30000      // 30ms about 6 meters
            //
            // this code must NOT be run on time critical thread

            digitalWrite(SONAR_TRIG, 0);
            delayMicroseconds(2);
            digitalWrite(SONAR_TRIG, 1);
            delayMicroseconds(10);
            digitalWrite(SONAR_TRIG, 0);

            #if SONAR_INTERRUPT
                // wait for it to go high
                while (!digitalRead(SONAR_ECHO)) {}
                sonar_start_time = micros();
                sonar_state = 1;
            #else
                long duration = pulseIn(SONAR_ECHO,HIGH);   // default is 1 second (1,000,000 us)
                int distance = duration * 0.034 / 2;        // distance in CM
                display(0,"distance=%d duration=%d",distance,duration);
            #endif

            sonar_time = millis();
        }
        #if SONAR_INTERRUPT
            else if (sonar_state == 2)
            {
                int distance = sonar_micros * 0.034 / 2;        // distance in CM
                display(0,"distance=%d duration=%d",distance,sonar_micros);
                sonar_state = 0;
            }
        #endif




    #endif


    #if WITH_GYRO
        static uint32_t gyro_time = 0;
        if (millis() > gyro_time + 1000)
        {
            // Read normalized values
            Vector normAccel = mpu.readNormalizeAccel();

            // Calculate Pitch & Roll
            int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
            int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

            display(0,"pitch(%d) roll(%d)",pitch,roll);
            gyro_time = millis();
        }
    #endif

    char command = 0;

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


    static int left_motor = 0;
    static int right_motor = 0;
    static int global_direction = 1;

    if (!command && Serial.available())
    {
        command = Serial.read();
        display(0,"Got serial command: '%c'",command);
    }

    #if WITH_MOTOR

        int new_direction = global_direction;
        int new_left_motor = left_motor;
        int new_right_motor = right_motor;

        if (command == 'f')
        {
            display(0,"FORWARD",0);
            new_direction = 1;
        }
        else if (command == 'b')
        {
            display(0,"BACK",0);
            new_direction = -1;
        }

        if (new_direction != global_direction)
        {
            global_direction = new_direction;
            display(0,"new global direction %d",global_direction);
            if (new_left_motor)
                new_left_motor = global_direction;
            if (new_right_motor)
                new_right_motor = global_direction;
        }

        if (command == 'l')
        {
            if (new_left_motor)
                new_left_motor = 0;
            else
                new_left_motor = global_direction;
            display(0,"LEFT %d",new_left_motor);
        }
        else if (command == 'r')
        {
            if (new_right_motor)
                new_right_motor = 0;
            else
                new_right_motor = global_direction;
            display(0,"RIGHT %d",new_right_motor);
        }

        if (left_motor != new_left_motor)
        {
            left_motor = new_left_motor;
            if (left_motor)
            {
                analogWrite(MOTOR_ENB,DEFAULT_SPEED);
                digitalWrite(MOTOR_IN3,left_motor < 0 ? 1 : 0);
                digitalWrite(MOTOR_IN4,left_motor < 0 ? 0 : 1);
            }
            else
            {
                analogWrite(MOTOR_ENB,0);
                digitalWrite(MOTOR_IN3,0);
                digitalWrite(MOTOR_IN4,0);  // forward
            }
        }
        else if (right_motor != new_right_motor)
        {
            right_motor = new_right_motor;
            if (right_motor)
            {
                analogWrite(MOTOR_ENA,DEFAULT_SPEED);
                digitalWrite(MOTOR_IN1,right_motor < 0 ? 1 : 0);
                digitalWrite(MOTOR_IN2,right_motor < 0 ? 0 : 1);
            }
            else
            {
                analogWrite(MOTOR_ENA,0);
                digitalWrite(MOTOR_IN1,0);
                digitalWrite(MOTOR_IN2,0);  // forward
            }
        }

    #endif
}
