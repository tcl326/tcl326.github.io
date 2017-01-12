// Inlude the SFE_LSM9DS0 and other required libraries.
#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

// Initiate an instance of LSM9DS0
#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

uint32_t timer;

void setup()
{
  Serial.begin(9600); 
  uint16_t status = dof.begin();
  timer = micros();
}

//Initiate Kalman filter variables

float QAngle  =  0.01;
float QGyro   =  0.0003;
float RAngle  =  0.01;
float xBias = 0;
float P0 = 0;
float P1 = 0;
float P2 = 0;
float P3 = 0;
float y, S;
float K0, K1;
float xAngle = 0;
float yAngle = 0;
float pi = 3.141696;

//Kalman module
float kalmanCalculate(float newAngle, float newRate, double looptime, float xAngle)
{
  double dt = double(looptime);
  xAngle += dt*(newRate-xBias);
  P0 += -1*dt*(P2+P1)+QAngle*dt;
  P1 -= dt*P3;
  P2 -= dt*P3;
  P3 += QGyro*dt;
  y = newAngle-xAngle;
  S = P0+RAngle;
  K0 = P0/S;
  K1 = P2/S;
  xAngle +=  K0*y;
  xBias  +=  K1*y;
  P0 -= K0*P0;
  P1 -= K0*P1;
  P2 -= K1*P0;
  P3 -= K1*P1;
  return xAngle;
}

void loop()
{
  double looptime = (double)(micros()-timer)/1000000; // Calculate delta time
  timer = micros();
  dof.readGyro();
  dof.readAccel();
  float ay = dof.calcAccel(dof.ay);
  float ax = dof.calcAccel(dof.ax);
  float az = dof.calcAccel(dof.az);
  float roll = atan2(ay,sqrt(ax*ax)+(az*az))*180/pi;
  float pitch = atan2(-1*ax,az)*180/pi;
  float gz = dof.calcGyro(dof.gz);
  float gx = dof.calcGyro(dof.gx);
  float gy = dof.calcGyro(dof.gy);
  float rollRate = gz;
  float pitchRate = gy;
  xAngle = kalmanCalculate(roll,rollRate,looptime,xAngle);
  yAngle = kalmanCalculate(pitch,pitchRate,looptime,yAngle);

  Serial.print(xAngle);
  Serial.print(": ");
  Serial.println(yAngle);
}

