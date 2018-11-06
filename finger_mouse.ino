#include <CapacitiveSensor.h>

#include <Mouse.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//#include "Adafruit_MPR121.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/* IDEAS
 *  Use the IRQ on the capacitive sensor in order to wake up the arduino.
 *  Make sure the bluetooth connection stays alive.
 */


#define VAL_TO_G 16384
#define VAL_TO_DEG 131
#define G_TO_MSS 9.8
#define US_TO_S 1000000
#define INDEX_PAD 1 //Not sure what this is going to be yet
#define TRUE 1
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
 //Trying to be like 1/2at^2 constant must always be negative but can be varied for sensitivity
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
void error(const __FlashStringHelper*err);
void setup_ble(void);
struct mouse_pair mouse_glide(struct mouse_pair);

MPU6050 mpu;


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

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
CapacitiveSensor cap(12,11);


void setup() {
  uint8_t dev_status;
  dmp_ready = 0; //Assume failure
  been_pinched = 0;
  //Serial.begin(9600);
  pinMode(12,OUTPUT);
  pinMode(11, INPUT);
  cap = CapacitiveSensor(11,12); 
  Serial.println("Setup complete");
  /* Setup for MPU6050, hopefully it doesn't interfere with the capacitive sensor */
  Wire.begin();
  setup_ble();
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
  static struct mouse_pair mp;
  static unsigned timer = 0;
  adat = read_acc();
  if(pinched() && dmp_ready){
        adat = read_acc();
        mp = calc_xy(adat);
        send_mouse_event(mp);
        been_pinched = 1;
  } else {
        //mp = mouse_glide(mp); //Not working well at the moment
	been_pinched = 0;
  }
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
  float tempx, tempy;
  tempx = dat.yaw - prev_yaw;
  tempy= -1 * (dat.pitch - prev_pitch);
  prev_yaw = dat.yaw;
  prev_pitch = dat.pitch;

  #define SENSITIVITY 30.0
  mp.x = tempx * SENSITIVITY;
  mp.y = tempy * SENSITIVITY;
  //Serial.println(mp.x);
  return mp;
}

// This function assumes th/at the state of the mouse begin and end are handled already
void send_mouse_event(struct mouse_pair mp){
  //Mouse.move(mp.x,mp.y);
  // Mouse moves according to the user's input
    ble.print(F("AT+BleHidMouseMove="));
    ble.print(mp.x);ble.print(",");ble.println(mp.y);

    //if( !ble.waitForOK() )
    //{
    //  Serial.println( F("FAILED!") );
    //}


}

/* Only returns true if the index finger is pinched */
bool first_pinched(uint8_t fingers){
  #define FIRST_FINGER ( 1 << 1 )
  //Serial.println(fingers & FIRST_FINGER);
  return ( (fingers & FIRST_FINGER) == FIRST_FINGER );
}

uint8_t cap_sensor_read(){
  long v = cap.capacitiveSensor(80); //Sensor resolution probably need to mess with this
  Serial.println(v);
  if( v > 100 ) { //Arbitrary number
    return ( 1 << 0 );
  }
  delay(100);
  return (0);
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

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup_ble(void){
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // This demo only available for firmware from 0.6.6
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
  }
  Serial.println(F("Enable HID Service (including Mouse): "));
  if (! ble.sendCommandCheckOK(F( "AT+BleHIDEn=On"  ))) {
    error(F("Failed to enable HID (firmware >=0.6.6?)"));
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Could not reset??"));
  }
}

/* This should only be called when not pressed, otherwise we are going to move a lot */
struct mouse_pair mouse_glide(struct mouse_pair mp){
	#define FRICTION ( (-0.07) * pow( ticks, 2) )
	/* Should die off exponentially */
        struct mouse_pair glide_vals;
	static unsigned ticks = 0;
	if(mp.x > 0){
		glide_vals.x = mp.x + FRICTION;
	} else {
		glide_vals.x = mp.x - FRICTION;
	}

	if(mp.y > 0){
		glide_vals.y = mp.y + FRICTION;
	} else {
		glide_vals.y = mp.y - FRICTION;
	}
	send_mouse_event(glide_vals);
        ticks++;
        return glide_vals;
}
