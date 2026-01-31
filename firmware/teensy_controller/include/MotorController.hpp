#pragma once
#include <Arduino.h>

class Motor {
  Motor(int Vf_Pin_, int Vf_L_Pin_, int Vb_Pin_, int Vb_L_Pin_, float maxVoltage_) : Vf_Pin(Vf_Pin_), Vf_L_Pin(Vf_L_Pin_), Vb_Pin(Vb_Pin_), Vb_L_Pin(Vb_L_Pin_), maxVoltage(maxVoltage_){}
  ~Motor();
  
  void setVoltage(float voltage, int direction);

  private:
  int Vf_Pin;
  int Vf_L_Pin;
  int Vb_Pin;
  int Vb_L_Pin;
  float maxVoltage;
  int currentDirection = 0; // 1 = forward, 0 = stopped, -1 = backwards
  float deadtime = 100;
};