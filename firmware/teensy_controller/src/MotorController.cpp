#include <MotorController.hpp>

void Motor::setVoltage(float voltage, int direction){
  // 1 direction for forward, -1 direction for backwards

  float percentageOfMax = voltage/maxVoltage;

  if (percentageOfMax > 0.98){ // clamp it at most to 98% due to the fact the bootstrap will not actually allow us to achieve 100% duty cycle
    percentageOfMax = 0.98;
  } 

  float Vb_Voltage;
  float Vb_L_Voltage;
  float Vf_Voltage;
  float Vf_L_Voltage;

  if (direction == 1){
    if (currentDirection == -1){ 
      // if we're currently backwards, when switching states we need to be careful to avoid shoot through 

    }

  } else if (direction == -1){
    if (currentDirection == 1){
      // turn off forward path
      digitalWrite(Vf_Pin, LOW);
      digitalWrite(Vf_L_Pin, HIGH);

      delay(deadtime); // to avoid shootthrough
    }



    

  } else {
    // turn both low sides off (no path from vcc to gnd)
    digitalWrite(Vf_Pin, LOW);
    digitalWrite(Vb_Pin, LOW); 
    digitalWrite(Vf_L_Pin, HIGH); // active low -> off at HIGH voltage
    digitalWrite(Vb_L_Pin, HIGH); // active low -> off at HIGH voltage
  }

  currentDirection = direction; // update to new direction


}