#include <Kalman.h>

#include <Mouse.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Adafruit_MPR121.h"
/* IDEAS
 *  Use the IRQ on the capacitive sensor in order to wake up the arduino.
 *  Make sure the bluetooth connection stays alive.
 *  project the filter onto the plane with gravity so roll doesn't matter
 */


/* with default values the accelerometer range is  +- 2g and += 250 deg/second
 *  leaving us with 
 *  16384 per g
 *  131 deg/s
 */
#define VAL_TO_G 16384
#define VAL_TO_DEG 131
#define G_TO_MSS 9.8
#define US_TO_S 1000000
#define INDEX_PAD 1 //Not sure what this is going to be yet
#define TRUE 1

struct mouse_pair{
  int x;
  int y;
};

struct orientation{
  float yaw;
  float pitch;
  float roll;
};

struct orientation read_acc(void);
struct mouse_pair calc_xy(struct orientation);
void send_mouse_event(struct mouse_pair);
bool on_tick(void);
bool pinched(void);
void set_offsets(void);
void set_starting_angles(void);
//void get_angles(struct accel_data, double*, double*, double);
MPU6050 mpu;
Adafruit_MPR121 cap = Adafruit_MPR121();
Kalman kalmanX;
Kalman kalmanY;

bool dmp_ready;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
uint16_t packetSize;
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
VectorInt16 gyro;
bool been_pinched;
uint32_t timer;
void setup() {
  uint8_t dev_status;
  dmp_ready = 0; //Assume failure
  been_pinched = 0;
  Serial.begin(9600);
  
  //pinMode(INDEX_PAD, INPUT);
  cap.begin(0x5A);
  Serial.println("Setup complete");
  /* Setup for MPU6050, hopefully it doesn't interfere with the capacitive sensor */
  Wire.begin();
  /* Calculate and set gyro/acceleromter offsets */
  mpu.initialize(); // Sets gyro to GYRO_FS_250 and accel to ACCEL_FS_2 (250deg/s and 2g)
  dev_status = mpu.dmpInitialize(); //Begin dmp programming
  /* Supply gyro and accel offsets once I have them */
  if(dev_status == 0){
    //Can enable dmp now
    mpu.setDMPEnabled(TRUE);
    set_offsets();
    //Setup interupt once I have it
    dmp_ready = TRUE; //So we can actually use it in the main loop
    packetSize = mpu.dmpGetFIFOPacketSize();
    //set_starting_angles();
     
  } else {
    //Bad stuff
    Serial.print("DMP ERROR: "); Serial.println(dev_status);
  }
}

void loop() {
  int x;
  int y;
  struct orientation adat;
  struct mouse_pair mp;
  adat = read_acc();
  if(pinched() && dmp_ready){
        adat = read_acc();
        mp = calc_xy(adat);
        send_mouse_event(mp);
        been_pinched = 1;
  } else { been_pinched = 0; }
  //delay(10);
}

/* accel is in gs and gyro is in degrees/sec */
struct orientation read_acc(){
  float ypr[3]; 
  struct orientation orient;
  
  fifoCount = mpu.getFIFOCount();
  if(fifoCount == 1024 ){
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else {
    while( fifoCount < packetSize){fifoCount = mpu.getFIFOCount();} // Wait for correct data length
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  }
  orient.yaw = ypr[0]/PI * 180;
  orient.pitch = ypr[2] /PI * 180;
  orient.roll = ypr[1] /PI * 180;
  return orient;
}


struct mouse_pair calc_xy(struct orientation dat){
  struct mouse_pair mp = {0,0};
  static float prev_yaw;
  static float prev_pitch;
  mp.x = dat.yaw - prev_yaw;
  mp.y = -1 * (dat.pitch - prev_pitch);
  prev_yaw = dat.yaw;
  prev_pitch = dat.pitch;
  return mp;
}

// This function assumes th/at the state of the mouse begin and end are handled already
void send_mouse_event(struct mouse_pair mp){
  Mouse.move(mp.x,mp.y);
  return;
}

/* Only returns true if the index finger is pinched */
bool first_pinched(uint8_t fingers){
  #define FIRST_FINGER ( 1 << 1 )
  //Serial.println(fingers & FIRST_FINGER);
  return ( (fingers & FIRST_FINGER) == FIRST_FINGER );
}

uint8_t cap_sensor_read(){
  //return (1<<0); //Faked until can actually read the capacitative sensor
  //Serial.println(cap.touched());
  return cap.touched();
}

bool pinched(){
  uint8_t fingers = cap_sensor_read();
  //Serial.print("Fingers: "); Serial.println(fingers);
  return first_pinched(fingers); //&& second_pinched(fingers)
}

/* Set offsets gotten from running calibration MPU6050 sketch */
void set_offsets(){
  mpu.setXAccelOffset(-1357);
  mpu.setYAccelOffset(-3639);
  mpu.setZAccelOffset(1019);
  mpu.setXGyroOffset(-45);
  mpu.setYGyroOffset(62);
  mpu.setZGyroOffset(-47);
}


