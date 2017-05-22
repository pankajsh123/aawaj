#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;
const int thumb_finger_flex = A0, index_finger_flex = A1, middle_finger_flex = A2, ring_finger_flex = A3, tiny_finger_flex = A6, thumb_up_contact = 13, index_up_contact = 12, middle_up_contact = 11;
const int ring_up_contact = 10, tiny_up_contact = 9, index_low_contact = 8, middle_low_contact = 7, ring_low_contact = 6, tiny_middle_contact = 5, ring_middle_contact = 4;
float angles[5], val;
int contactval[7], temp;
float _yaw, _pitch, _roll, ty, tp, tr;
float orientation_max[3], orientation_min[3];
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());
  while (Serial.available());                 // wait for data
  while (Serial.available() && Serial.read());
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(thumb_finger_flex, INPUT);
  pinMode(index_finger_flex, INPUT);
  pinMode(middle_finger_flex, INPUT);
  pinMode(ring_finger_flex, INPUT);
  pinMode(tiny_finger_flex, INPUT);
  pinMode(thumb_up_contact, OUTPUT);
  pinMode(index_up_contact, INPUT);
  pinMode(middle_up_contact, INPUT);
  pinMode(ring_up_contact, INPUT);
  pinMode(tiny_up_contact, INPUT);
  pinMode(index_low_contact, INPUT);
  pinMode(middle_low_contact, OUTPUT);
  pinMode(ring_low_contact, INPUT);
  pinMode(tiny_middle_contact, OUTPUT);
  pinMode(ring_middle_contact, INPUT);
}



//funtion to read flex sensor values and map the resistance value to angles between 0 to 90 degree

void readflex(float * angles)                    //read current reading if it is greater than previous assign this value othervise don't change
{
  val = analogRead(thumb_finger_flex);
  val = map(val, 110, 55, 0, 90);
  /*if (val < 25)
    val = 0;
  else if (val < 55)
    val = 0.5;
  else
    val = 1;*/
  val = val / 90;
  if (val > angles[0])
    angles[0] = val;

  val = analogRead(index_finger_flex);
  val = map(val, 103, 60, 0, 90);
 /* if (val < 23)
    val = 0;
  else if (val < 65)
    val = 0.5;
  else
    val = 1;*/
  val = val / 90;
  if (val > angles[1])
    angles[1] = val;

  val = analogRead(middle_finger_flex);
  val = map(val, 100, 40, 0, 90);
 /* if (val < 27)
    val = 0;
  else if (val < 65)
    val = 0.5;
  else
    val = 1;*/
  val = val / 90;
  if (val > angles[2])
    angles[2] = val;

  val = analogRead(ring_finger_flex);
  val = map(val, 100, 60, 0, 90);
 /* if (val < 18)
    val = 0;
  else if (val < 64)
    val = 0.5;
  else
    val = 1;*/
  val = val / 90;
  if (val > angles[3])
    angles[3] = val;

  val = analogRead(tiny_finger_flex);
  val = map(val, 100, 50, 0, 90);
 /* if (val < 32)
    val = 0;
  else if (val < 68)
    val = 0.5;
  else
    val = 1;*/
  val = val / 90; 
  if (val > angles[4])
    angles[4] = val;

}

//funtion to read contact sensor values
void readcontact(int * contactval)                  //read current reading if it is greater than pervious assign this value othervise don't change
{
  temp = digitalRead(index_up_contact);
  if (temp > contactval[0])
    contactval[0] = temp;
  temp = digitalRead(middle_up_contact);
  if (temp > contactval[1])
    contactval[1] = temp;
  temp = digitalRead(ring_up_contact);
  if (temp > contactval[2])
    contactval[2] = temp;
  temp = digitalRead(tiny_up_contact);
  if (temp > contactval[3])
    contactval[3] = temp;
  temp = digitalRead(index_low_contact);
  if (temp > contactval[4])
    contactval[4] = temp;
  temp = digitalRead(ring_low_contact);
  if (temp > contactval[5])
    contactval[5] = temp;
  temp = digitalRead(ring_middle_contact);
  if (temp > contactval[6])
    contactval[6] = temp;
}


void loop()
{
  digitalWrite(thumb_up_contact,HIGH);
  digitalWrite(middle_low_contact,HIGH);
  digitalWrite(tiny_middle_contact,HIGH);
  int start_ = 0;
  while(start_ == 0)
  {
        start_ = digitalRead(tiny_up_contact);
        if (!dmpReady)
          return;
        while (!mpuInterrupt && fifoCount < packetSize)
        {
        }
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
          mpu.resetFIFO();
         // Serial.println(F("FIFO overflow!"));
        }
        else if (mpuIntStatus & 0x02)
        {
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
          #ifdef OUTPUT_READABLE_YAWPITCHROLL
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                _yaw = (ypr[0] * 180 / M_PI);
                _yaw = _yaw / 200;
                _pitch = (ypr[1] * 180 / M_PI);
                _pitch = _pitch / 200;
                _roll = (ypr[2] * 180 / M_PI);
                _roll = _roll / 200;
          #endif
        }
  }
  ty = _yaw;
  tr = _roll;
  tp = _pitch;
  for (int i = 0; i < 5; i++)           //intializing values
    angles[i] = 0;
  for (int i = 0; i < 7; i++)
    contactval[i] = 0;
  for (int i = 0; i < 3; i++)
  {
    orientation_min[i] = 0;
    orientation_max[i] = 0;
  }
  if(start_ == 1)          //if thumb touches tiny finger start reading gestures,inititate gesture
  {
        String str = ""; 
        while(contactval[2]  != 1)      //read gestures until ring finger touches thumb ,terminator gesture
        {
            readflex(angles);
            readcontact(contactval);
            if (!dmpReady)
               return;
            while (!mpuInterrupt && fifoCount < packetSize)
            {
            }
            mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();
            fifoCount = mpu.getFIFOCount();
            if ((mpuIntStatus & 0x10) || fifoCount == 1024)
            {
                mpu.resetFIFO();
               // Serial.println(F("FIFO overflow!"));
            }
            else if (mpuIntStatus & 0x02)
            {
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
                #ifdef OUTPUT_READABLE_YAWPITCHROLL
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  _yaw = (ypr[0] * 180/M_PI);
                  _yaw = _yaw / 200;
                  _yaw = _yaw - ty;
                 _pitch = (ypr[1] * 180/M_PI);
                 _pitch = _pitch / 200;
                 _pitch = _pitch - tp;
                 _roll = (ypr[2] * 180/M_PI);
                 _roll = _roll / 200;
                 _roll = _roll - tr;
                if(_pitch > orientation_max[0])
                    orientation_max[0] = _pitch;
                if(_roll > orientation_max[1])
                    orientation_max[1] = _roll;
                if(_yaw > orientation_max[2])
                    orientation_max[2] = _yaw;
                if(_pitch < orientation_min[0])
                    orientation_min[0] = _pitch;
                if(_roll < orientation_min[1])
                    orientation_min[1] = _roll;
                if(_yaw < orientation_min[2])
                    orientation_min[2] = _yaw;
                #endif
             }
        }
        for(int i = 0; i < 5; i++)
            str = str + (String)angles[i] + " ";
        for(int i = 0; i < 7; i++)
            str = str + (String)contactval[i] + " ";
        for(int i = 0;i < 3;i++)
            str = str + (String)orientation_min[i] + " ";
        for(int i = 0;i < 3;i++)
            str = str + (String)orientation_max[i] + " ";
        Serial.print("0\n");
        delay(50);      
        if(str != "")
        {
            Serial.println(str);
            delay(800);
        }
    }
}
