#include <Arduino.h>
#include <math.h>

int Vf_pin = 2 ;
int Vf_L_pin = 3;
int Vb_pin = 4;
int Vb_L_pin = 5;
float PWM_period = 100e-6f;
float PWM_freq = 1.0f / PWM_period;
int a = 20;
bool forward = false;
bool backward = true;

void setup() {
    Serial.begin(9600);
    
    for (int i = 2; i < 6; i++){
        pinMode(i, OUTPUT);
        analogWriteFrequency(i, PWM_freq);
    }

    analogWrite(Vf_L_pin, 255); // forward high side off 
    analogWrite(Vf_pin, 0); // forward low side off
    analogWrite(Vb_pin, 255); // backward low side on
    analogWrite(Vb_L_pin, a); // backward low side PWM to reach desired voltage/speed
}

void loop() {
    a += 10;

    if (a > 255){
        a = 0;
        
        if (forward){ // switch to backwards
            analogWrite(Vf_pin, 0);
            analogWrite(Vf_L_pin, 255);
            delay(1);

            analogWrite(Vb_pin, 255);

        } else if (backward){ // switch to forwards
            analogWrite(Vb_pin, 0); // backward low side off
            analogWrite(Vb_L_pin, 255); // backward high side off
            delay(1); // dead time

            analogWrite(Vf_pin, 255);

        }

        forward = !forward;
        backward = !backward;

        delay(200);
        
        return;

    }

    if (forward){
        analogWrite(Vf_L_pin, a);
    } else {
        analogWrite(Vb_L_pin, a);
    }
    delay(200);



}