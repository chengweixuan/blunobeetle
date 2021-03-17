// CircularBuffer Libraries
#include <CircularBuffer.h>

// MegunoLink Libraries for Exponential Filter
#include "MegunoLink.h"
#include "Filter.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// class with packet data structures and functions
#include "packets.h"

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */
#define BAUD_RATE 115200
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define EMG_INPUT_PIN 0
#define SAMPLE_INTERVAL 50 // 40 millis = 25Hz

#define TEST_BEETLE 0
#define BLACK_BEETLE 1
#define EMG_BEETLE 2
#define NAKED_BEETLE 3
#define PLASTIC_BEETLE 4
#define WHITE_BEETLE 5

// Sensor Sampling and Sending Rate
long int last_read_time = 0; // Beetle last read timestamp (millis)


// ------- SENSOR GLOBALS --------- //
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// MegunoLink Exponential Filter Vars
long FilterWeight = 5;
ExponentialFilter<long> AccXFilter(FilterWeight, 0);
ExponentialFilter<long> AccYFilter(FilterWeight, 0);
ExponentialFilter<long> AccZFilter(FilterWeight, 0);

ExponentialFilter<long> GyroYawFilter(FilterWeight, 0);
ExponentialFilter<long> GyroPitchFilter(FilterWeight, 0);
ExponentialFilter<long> GyroRollFilter(FilterWeight, 0);

//CircularBuffer Buffer Vars
CircularBuffer<int, 50> AccXBuffer; //to detect left right acc, size 25, 1 second of AccX Data
CircularBuffer<int, 50> AccYBuffer; //to detect up down acc, size 25, 1 second of AccX Data
CircularBuffer<int, 50> AccZBuffer; //to detect forward backward acc, size 25, 1 second of AccX Data

CircularBuffer<int, 50> GyroYawBuffer; //to detect left right acc, size 25, 1 second of AccX Data
CircularBuffer<int, 50> GyroPitchBuffer; //to detect left right acc, size 25, 1 second of AccX Data
CircularBuffer<int, 50> GyroRollBuffer; //to detect left right acc, size 25, 1 second of AccX Data

//Feature thresholds
// Threshold: Arm Pointing Down
int gryo_roll_delta_threshold_max = 20; // difference btw current roll andgle and pointing down angle (90 degrees)e.g should be below 20 to be considered to be pointing downwards
int gryo_roll_delta_threshold_min = -5;

// EMG globals
float emg_mean = 0; // mean absolute value of emg during entire session
long long emg_samples = 0; //number of emg samples taken during entire session
int emg_startup_count = 0; // number of readings for the emg to calibrate and initialise
int fatigue_flag = 0; // 0 not fatigued; 1 fatigued
float fatigue_threshold = 0.25;

// Threshold: Detect if moving LEFT or RIGHT when IDLE
int acc_z_move_threshold = 500;
// ------- END OF SENSOR GLOBALS --------- //


// ------- BLE CONNECTION GLOBALS ------- //
// Globals for Current Sensor Reading 
int16_t AccX = 0;
int16_t AccY = 0;
int16_t AccZ = 0;

int16_t GyroYaw = 0;
int16_t GyroPitch = 0;
int16_t GyroRoll = 0;

// Features vars to be fit into one byte
uint8_t startPosition = 0;
uint8_t dancer_state = 0; // 0 is IDLE; 1 is Dancing
bool feature_arm_pointing_down = false;
uint8_t feature_left_right_null = 0; // 0 null; 1 left; 2 right packet 2 bits

bool moveLeft = false;
bool moveRight = false;

// Packet control Globals
bool connected = false;
bool dataFlag = false;
bool waitingForAck = false;
SensorData sensorData;
DataPacket dataPacket;
// ------- END OF BLE CONNECTION GLOBALS ------- //



// ================================================================
// ===               ACTIVITY DETECTION ROUTINE                ===
// ================================================================

void detectActivity() {

  // Feature_1: Arm Pointing Down
  GyroRollBuffer.push(90 - GyroRoll);
  
  int GyroRollBufferSize = GyroRollBuffer.size();
  for (int i = 0; i < GyroRollBufferSize - 1; i++) {
    if (GyroRollBuffer[i] > gryo_roll_delta_threshold_min && GyroRollBuffer[i] < gryo_roll_delta_threshold_max) {
      feature_arm_pointing_down = true;
    } else {
      feature_arm_pointing_down = false;
    }
  }

  // Feature_2: Detect if moving LEFT or RIGHT when IDLE
  if (dancer_state == 0) {
    AccXBuffer.push(AccX);
    AccZBuffer.push(AccZ);
  } else {
    AccXBuffer.clear();
    AccZBuffer.clear() ;
  }

  //Feature_2a: Detection of Acc spike pattern
  //    - AccZ (+ve) AND AccX (+ve) BEFORE AccZ (-ve) AND AccX (-ve) -> LEFT
  //    - AccZ (-ve) AND AccX (-ve) BEFORE AccZ (+ve) AND AccX (+ve) -> RIGHT

  bool acc_pos_peak_detected = false;
  bool acc_neg_peak_detected = false;

  bool move_left_detected = false;
  bool move_right_detected = false;

  int AccXBufferSize = AccXBuffer.size();
  int AccZBufferSize = AccZBuffer.size();
  
  for (int i = 0; i< min(AccXBufferSize, AccZBufferSize); i++) {
    if ((AccZBuffer[i] > acc_z_move_threshold) && (AccZBuffer[i] > 0 && AccXBuffer[i] > 0)) {
      acc_pos_peak_detected = true;
      if (acc_neg_peak_detected == true) {
        move_left_detected = false;
        move_right_detected = true;
      }
    } else if ((AccZBuffer[i] < -acc_z_move_threshold) && (AccZBuffer[i] < 0 && AccXBuffer[i] < 0)) {
      acc_neg_peak_detected = true;
      if (acc_pos_peak_detected == true) {
        move_left_detected = true;
        move_right_detected = false;
      }
    }
  }

  if (move_left_detected == true && move_right_detected == false) {
    feature_left_right_null = 1;
    moveLeft = true;
     
    // Serial.println(feature_left_right_null);
  } else if (move_left_detected == false && move_right_detected == true) {
    feature_left_right_null = 2;
    moveRight = true;
  }
  
  

  // State Transition: Dancer State Transition
  if (dancer_state == 0) {
    if (feature_arm_pointing_down == false) {
      dancer_state = 1; // IDLE -> DANCING
    }
  } else if (dancer_state == 1) {
    if (feature_arm_pointing_down == true) {
      dancer_state = 0; // DANCING -> IDLE
    }
  }

}

//void calibrateSensor(int beetleNo) {
//  switch (beetleNo)
//  {
//    case TEST_BEETLE:
//   
//  }
//
//  
//  //    // test beetle
////    mpu.setXAccelOffset(-150);
////    mpu.setYAccelOffset(52);
////    mpu.setZAccelOffset(1162);
////
////    mpu.setXGyroOffset(188);
////    mpu.setYGyroOffset(-8);
////    mpu.setZGyroOffset(87);
//
//    // black beetle
//    mpu.setXAccelOffset(-5568);
//    mpu.setYAccelOffset(-1729);
//    mpu.setZAccelOffset(1318);
//
//    mpu.setXGyroOffset(71);
//    mpu.setYGyroOffset(-19);
//    mpu.setZGyroOffset(-259);
////
////    // EMG beetle
////    mpu.setXAccelOffset(-5451);
////    mpu.setYAccelOffset(-2764);
////    mpu.setZAccelOffset(1660);
////
////    mpu.setXGyroOffset(120);
////    mpu.setYGyroOffset(-26);
////    mpu.setZGyroOffset(-12);
// 
////    // naked beetle
////    mpu.setXAccelOffset(-933);
////    mpu.setYAccelOffset(576);
////    mpu.setZAccelOffset(22);
////
////    mpu.setXGyroOffset(291);
////    mpu.setYGyroOffset(-25);
////    mpu.setZGyroOffset(-2);
//
////    // plastic beetle
////    mpu.setXAccelOffset(-2936);
////    mpu.setYAccelOffset(340);
////    mpu.setZAccelOffset(1270);
////
////    mpu.setXGyroOffset(61);
////    mpu.setYGyroOffset(-103);
////    mpu.setZGyroOffset(2);
//
//    // white beetle
//    mpu.setXAccelOffset(-1511);
//    mpu.setYAccelOffset(1864);
//    mpu.setZAccelOffset(1533);
//
//    mpu.setXGyroOffset(37);
//    mpu.setYGyroOffset(-12);
//    mpu.setZGyroOffset(43);
//}

void setupSensors() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(115200);
    // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
/*    
    pinMode(INTERRUPT_PIN, INPUT);
*/
    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity

//    // test beetle
//    mpu.setXAccelOffset(-150);
//    mpu.setYAccelOffset(52);
//    mpu.setZAccelOffset(1162);
//
//    mpu.setXGyroOffset(188);
//    mpu.setYGyroOffset(-8);
//    mpu.setZGyroOffset(87);

//    // black beetle
//    mpu.setXAccelOffset(-5568);
//    mpu.setYAccelOffset(-1729);
//    mpu.setZAccelOffset(1318);
//
//    mpu.setXGyroOffset(71);
//    mpu.setYGyroOffset(-19);
//    mpu.setZGyroOffset(-259);
//
//    // EMG beetle
//    mpu.setXAccelOffset(-5451);
//    mpu.setYAccelOffset(-2764);
//    mpu.setZAccelOffset(1660);
//
//    mpu.setXGyroOffset(120);
//    mpu.setYGyroOffset(-26);
//    mpu.setZGyroOffset(-12);
 
//    // naked beetle
//    mpu.setXAccelOffset(-1069);
//    mpu.setYAccelOffset(409);
//    mpu.setZAccelOffset(10);
//
//    mpu.setXGyroOffset(292);
//    mpu.setYGyroOffset(-8);
//    mpu.setZGyroOffset(1);

    // plastic beetle
    mpu.setXAccelOffset(-2994);
    mpu.setYAccelOffset(103);
    mpu.setZAccelOffset(1253);

    mpu.setXGyroOffset(63);
    mpu.setYGyroOffset(-99);
    mpu.setZGyroOffset(-6);

//    // white beetle
//    mpu.setXAccelOffset(-1511);
//    mpu.setYAccelOffset(1864);
//    mpu.setZAccelOffset(1533);
//
//    mpu.setXGyroOffset(37);
//    mpu.setYGyroOffset(-12);
//    mpu.setZGyroOffset(43);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
//        mpu.CalibrateAccel(6);
//        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
/*
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
*/
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void detectFatigue() {
//  int emg_mean = 0; // mean absolute value of emg during entire session
//  int emg_samples = 0; //number of emg samples taken during entire session
//  int emg_startup_count = 0; // number of readings for the emg to calibrate and initialise
  // TimePlot Plot;
  int EMGValue = analogRead(EMG_INPUT_PIN);
//  Plot.SendData("Raw-EMG", EMGValue/1000.0);
  if (emg_startup_count < 150) {
    emg_startup_count++;
  } else {
    emg_samples++;
    emg_mean = (emg_mean * (emg_samples - 1) + EMGValue/1000.0) / emg_samples;
//    Plot.SendData("EMG-Mean", emg_mean);
  }
  // Set flag: Dancer Fatigue Flag
  if (fatigue_flag == 0) {
    if (emg_mean >= fatigue_threshold) {
      fatigue_flag = 1; // not fatigued -> fatigued
//      Plot.SendData("fatigue_flag", fatigue_flag);
    }
  } else if (fatigue_flag == 1) {
    if (emg_mean < fatigue_threshold) {
      fatigue_flag = 0; // fatigued -> not fatigued
//      Plot.SendData("fatigue_flag", fatigue_flag);
    }
  }
  
}

void readSensors() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);

//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);

        AccXFilter.Filter(aaReal.x);
        AccYFilter.Filter(aaReal.y);
        AccZFilter.Filter(aaReal.z);

        GyroYawFilter.Filter(ypr[0] * 180/M_PI);
        GyroPitchFilter.Filter(ypr[1] * 180/M_PI);
        GyroRollFilter.Filter(ypr[2] * 180/M_PI);

        AccX = AccXFilter.Current();
        AccY = AccYFilter.Current();
        AccZ = AccZFilter.Current();
        
        GyroYaw = GyroYawFilter.Current();
        GyroPitch = GyroPitchFilter.Current();
        GyroRoll = GyroRollFilter.Current();
             
        detectActivity();

//        TimePlot Plot;
//        Plot.SendData("Filtered-AccX", AccXFilter.Current());
//        Plot.SendData("Filtered-AccY", AccYFilter.Current());
//        Plot.SendData("Filtered-AccZ", AccZFilter.Current());
//        Plot.SendData("Filtered-GyroYaw", GyroYawFilter.Current());
//        Plot.SendData("Filtered-GyroPitch", GyroPitchFilter.Current());
//        Plot.SendData("Filtered-GyroRoll", GyroRollFilter.Current());

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }

    detectFatigue();
}


// ================================================================
// ===                       PROGRAM SETUP                      ===
// ================================================================

void setup() {
  Serial.begin(BAUD_RATE);
  setupSensors();
  connected = false;
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  if (!dmpReady) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      delay(1000);
    }

  // handles BLE commands and handshake with laptop
  if (Serial.available() > 0) {
    uint8_t receivedByte = Serial.read();

    if (isValidCommandPacket(receivedByte)) {
      uint8_t command = getCommandType(receivedByte);
      
      if (command == HELLO) {
        connected = false;
        CommandPacket ackCommand = makeCommandPacket(ACK);
        Serial.write((byte*)&ackCommand, sizeof(ackCommand));
        Serial.flush();
        waitingForAck = true;
        delay(100);
        
      } else if (waitingForAck && (command == ACK)) {
        connected = true;
        waitingForAck = false;
      }
    }
  } else {

    if (millis() > last_read_time + SAMPLE_INTERVAL && connected) {
      last_read_time = millis();

      readSensors(); // update sensor values
      makePacket();
      Serial.write((byte*)&dataPacket, sizeof(dataPacket));
      Serial.flush();

      feature_left_right_null = 0;
    }
  }

}

void makePacket() {
  sensorData = getSensorData(AccX, AccY, AccZ, GyroYaw, GyroPitch, GyroRoll);
  uint16_t EMGint = (int)(emg_mean * 1000);
  dataPacket = makeDataPacket(startPosition, dancer_state, feature_left_right_null, sensorData, EMGint);
}

void makeTestPacket() {
  uint16_t AccX2 = 7;
  uint16_t AccY2 = 50;
  uint16_t AccZ2 = -269;
  uint16_t GyroYaw2 = 67;
  uint16_t GyroPitch2 = -20;
  uint16_t GyroRoll2 = 173;
  sensorData = getSensorData(AccX2, AccY2, AccZ2, GyroYaw2, GyroPitch2, GyroRoll2);
  dataPacket = makeDataPacket(1, 0, 1, getSensorData(AccX2, AccY2, AccZ2, GyroYaw2, GyroPitch2, GyroRoll2), 0);
}