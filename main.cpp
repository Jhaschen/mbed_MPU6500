
#include "mbed.h"
#include "MPU6050.h"


float sum = 0;
uint32_t sumCount = 0;
double dt;


   MPU6050 mpu6050;
   
   Timer t;

DigitalOut led1(LED1) ;

int main()
{


  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C   
  
  t.start();        
  
  
    
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68\n\r");
  
  if (whoami == 0x68) // WHO_AM_I should always be 0x68
  {  
    //printf("MPU6050 is online...");
    ThisThread::sleep_for(1s);
   

    
    mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    /*
    printf("x-axis self test: acceleration trim within : "); printf("%f", SelfTest[0]); printf("% of factory value \n\r");
    printf("y-axis self test: acceleration trim within : "); printf("%f", SelfTest[1]); printf("% of factory value \n\r");
    printf("z-axis self test: acceleration trim within : "); printf("%f", SelfTest[2]); printf("% of factory value \n\r");
    printf("x-axis self test: gyration trim within : "); printf("%f", SelfTest[3]); printf("% of factory value \n\r");
    printf("y-axis self test: gyration trim within : "); printf("%f", SelfTest[4]); printf("% of factory value \n\r");
    printf("z-axis self test: gyration trim within : "); printf("%f", SelfTest[5]); printf("% of factory value \n\r");
    */
    ThisThread::sleep_for(1s);

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
    {
    mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
    mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    mpu6050.initMPU6050(); printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    ThisThread::sleep_for(2s);
       }
    else
    {
    printf("Device did not the pass self-test!\n\r"); 
      }
    }
    else
    {
    printf("Could not connect to MPU6050: \n\r");
    printf("%#x \n",  whoami);
 
    while(1) ; // Loop forever if communication doesn't happen
  }



 while(1) {
  
  // If data ready bit set, all data registers have new data
  if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
    mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
    mpu6050.getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
    mpu6050.getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   

    tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
    temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
   }  
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
   
    
    if(lastUpdate - firstUpdate > 10000000.0f) {
     beta = 0.04;  // decrease filter gain after stabilized
     zeta = 0.015; // increasey bias drift gain after stabilized
    }
    
   // Pass gyro rate as rad/s
    mpu6050.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

    // Serial print and/or display at 0.5 s rate independent of data rates
    ThisThread::sleep_for(500ms);
  
    printf("ax = %.2f", ax); 
    printf(" ay = %.2f", ay); 
    printf(" az = %.2f  g\n\r",az); 

    printf("gx = %.2f", gx); 
    printf(" gy = %.2f", gy); 
    printf(" gz = %.2f  deg/s\n\r", gz); 
    
    printf(" temperature = %.1f  C\n\r", temperature); 
    
    
        
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

    //printf("q0 = %f\n\r", q[0]);
    //printf("q1 = %f\n\r", q[1]);
    //printf("q2 = %f\n\r", q[2]);
    //printf("q3 = %f\n\r", q[3]);      
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;

    printf("Yaw, Pitch, Roll: %.2f %.2f %.2f\n\r", yaw, pitch, roll);
    
 

    led1= !led1;
   
}
}
 
 