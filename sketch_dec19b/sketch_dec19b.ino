#include"pwm.h"

#define Motor_L_A_PIN 7
#define Motor_L_B_PIN 8
#define Motor_L_PWM_PIN 9
#define Motor_R_A_PIN 11
#define Motor_R_B_PIN 12
#define Motor_R_PWM_PIN 10

PwmOut Motor1(Motor_L_PWM_PIN);
PwmOut Motor2(Motor_R_PWM_PIN);

void setup() {
  // put your setup code here, to run once:
    digitalWrite(Motor_L_A_PIN, LOW);
    digitalWrite(Motor_L_B_PIN, LOW);
    digitalWrite(Motor_L_PWM_PIN, LOW);
    digitalWrite(Motor_R_A_PIN, LOW);
    digitalWrite(Motor_R_B_PIN, LOW);
    digitalWrite(Motor_R_PWM_PIN, LOW);

    pinMode(Motor_L_A_PIN, OUTPUT);
    pinMode(Motor_L_B_PIN, OUTPUT);
    pinMode(Motor_L_PWM_PIN, OUTPUT);
    pinMode(Motor_R_A_PIN, OUTPUT);
    pinMode(Motor_R_B_PIN, OUTPUT);
    pinMode(Motor_R_PWM_PIN, OUTPUT);

    digitalWrite(Motor_L_A_PIN, HIGH);
    digitalWrite(Motor_L_B_PIN, LOW);
    digitalWrite(Motor_R_A_PIN, HIGH);
    digitalWrite(Motor_R_B_PIN, LOW);

    Serial.begin(9600);
    Motor1.begin(4000, 0.f);

}

void loop() {
  // put your main code here, to run repeatedly:
  int i;

    for(i = 20; i < 50 ; i++){
      Motor1.pulse_perc(i);
      Serial.println(i);
      delay(1000);
    }
}
