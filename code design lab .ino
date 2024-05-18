float vOUT = 0.0;
float vIN = 0.0;
float R1 = 221000.0;
float R2 = 95000.0;
float value = 0.0;
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
float Iref = 1.5, Vref = 10;

#include <PWM.h>
#include <PID_v1.h>

int32_t frequency = 25000;

double integral, previous, output = 0;
double Kp=2, Ki=0.9, Kd=0.5; 
double Input, Output,Setpoint=0.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(3,OUTPUT);
  InitTimersSafe();
  bool success = SetPinFrequencySafe(3,frequency);

  if(success) {
    pinMode(3, OUTPUT);
    pwmWrite(3,20);
  }
  myPID.SetMode(AUTOMATIC);
  delay(10000);

}

void loop() {
  float v,i;
  v = voltage_s();
  i = current_s();
  Serial.println(i);

  if (v >= 7.5 && v < 11){
    if(v>10 && i < Iref){
      Setpoint = Vref;
      Input = v;
    }
    else{
      Setpoint = Iref;
      Input = i;
      }
      myPID.Compute();
      pwmWrite(3,Output);
  }
  elseif(v >= 11){
    pwmWrite(3,0);
  }
  
}

float voltage_s(){
  Samples = 0;
  for(int i=0;i<150;i++){
    value = analogRead(A0);
    Samples = Samples + value;
  }
  value = Samples/150.0;
  vOUT = (value * 5.0) / 1024.0;
  vIN = vOUT / (R2/(R1+R2));
  return vIN;
}

float current_s(){
  Samples = 0;
  for (int x = 0; x < 150; x++){
  AcsValue = analogRead(A1);  
  Samples = Samples + AcsValue;
}

AvgAcs=Samples/150.0;
AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.066;
return AcsValueF;
}