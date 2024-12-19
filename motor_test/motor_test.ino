#include"pwm.h"
#include "FspTimer.h"

/*pin assign*/
#define ENC_L_PIN 2
#define ENC_R_PIN 3

#define Motor_L_A_PIN 7
#define Motor_L_B_PIN 8
#define Motor_L_PWM_PIN 9
#define Motor_R_A_PIN 11
#define Motor_R_B_PIN 12
#define Motor_R_PWM_PIN 10

#define TEST_PIN 13

/*定数*/
#define PI 3.141593
#define TIER_DIAMETER 35 //[mm]
#define ROBOT_WIDTH 104 //[mm]
#define ENC_SLIT 40
#define INTERRUPT_FREQ 50//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 100//[mm/s]

/*value*/
typedef struct enc_str{
    int count = 0;
    bool moved = false;
    bool rotate_forward = true;
} ENCODER;

volatile ENCODER enc[2];

FspTimer time_interrupt;
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
    digitalWrite(TEST_PIN, LOW);


    pinMode(Motor_L_A_PIN, OUTPUT);
    pinMode(Motor_L_B_PIN, OUTPUT);
    pinMode(Motor_L_PWM_PIN, OUTPUT);
    pinMode(Motor_R_A_PIN, OUTPUT);
    pinMode(Motor_R_B_PIN, OUTPUT);
    pinMode(Motor_R_PWM_PIN, OUTPUT);
    pinMode(TEST_PIN, OUTPUT);


    /*エンコーダ割り込み設定*/
    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), enc_counter_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), enc_counter_R, CHANGE);

    digitalWrite(Motor_L_A_PIN, HIGH);
    digitalWrite(Motor_L_B_PIN, LOW);
    digitalWrite(Motor_R_A_PIN, HIGH);
    digitalWrite(Motor_R_B_PIN, LOW);

    Serial.begin(9600);
    Motor1.begin(4000, 0.f);
}

void loop() {

    static int i = 19;
    static int counter = 0;
    unsigned long current_time = 0;
    static unsigned long start_time = 0;
    unsigned long diff_time = 0;
    int current_count = 0;
    static int prev_count = 0;
    int diff_count = 0;
    float speed[6];
    float ave_speed = 0;


    current_time = millis();
    current_count = enc[0].count;
    diff_time = current_time - start_time;
    diff_count = current_count - prev_count;
    
    if(diff_time > 2000){
        speed[counter] = diff_count * 1000 / diff_time;
        counter++;
        start_time = current_time;
        prev_count = current_count;
    }

    if(counter == 5){
        ave_speed = (speed[0] + speed[1] + speed[2] + speed[3] + speed[4]) / 5.f;
        Serial.print(i);
        Serial.print(":");
        Serial.println(ave_speed);
        if(i == 50) return;
        i++;
        counter = 0;
        Motor1.pulse_perc(i);
        delay(2000);
        start_time = millis();
        prev_count = enc[0].count;
    }
}

void enc_counter_L(){
    enc[0].moved = true;
    enc[0].count ++;
}

void enc_counter_R(){
    enc[1].moved = true;
    enc[1].count ++;
}

void timer_callback(timer_callback_args_t *arg){
    if(enc[0].moved){
        if(enc[0].rotate_forward){
            enc[0].count ++;
        }else{
            enc[0].count --;
        }
        enc[0].moved = false;
    }
    
    if(enc[1].moved){
        if(enc[1].rotate_forward){
            enc[1].count ++;
        }else{
            enc[1].count --;
        }
        enc[1].moved = false;
    }
}