#ifndef CLAREMPU_H
#define CLAREMPU_H

#include "ArduinoLog.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//To avoid duplicate definition errors in the MPU6050_6Axis_MotionApps20 library
//include declaration and definition in the same file

class ClareMpu {
    private:
        MPU6050 mpu;

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector

        uint8_t devStatus = -999;
    public:    
        ClareMpu();
        void setupMpu();
        void readState(float &w, float &x, float &y, float &z, float &ax, float &ay, float &az);
};

ClareMpu::ClareMpu(){
}

void ClareMpu::setupMpu() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    // verify connection
    if (mpu.testConnection()){
        Log.info(F("MPU6050 connection successful\n"));
    }else {
        Log.error(F("MPU6050 connection failed\n"));
    }

    // load and configure the DMP
    Log.info(F("Initializing DMP...\n"));
    devStatus = mpu.dmpInitialize();

    // Gyro offsets, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Log.info(F("Enabling DMP...\n"));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Log.info(F("DMP ready!\n"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Log.info(F("DMP Initialization failed (code %i)\n"), devStatus);
    }
}

void ClareMpu::readState(float &w, float &x, float &y, float &z, float &ax, float &ay, float &az) {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
        Log.error("MPU DMP not ready, status: %i\n", devStatus);
        return;
    } else{
        //Log.notice("DMP is ready\n");
    }

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        w = q.w;
        x = q.x;
        y = q.y;
        z = q.z;
        
        ax = aaWorld.x;
        ay = aaWorld.y;
        az = aaWorld.z;
    } else {
        Log.error("No packet\n");
    }
}
#endif


