#include "FspTimer.h"

/*pin assign*/
#define ENC_L_PIN 2
#define ENC_R_PIN 3

/*定数*/
#define PI 3.141593
#define TIER_DIAMETER 35 //[mm]
#define ROBOT_WIDTH 105 //[mm]


FspTimer time_interrupt;

/*value*/
typedef struct enc_str{
    int count = 0;
    bool rotate_forward =true;
} ENC_STATE;

typedef struct rbt_str{
    float headding = 0.f;
    float x = 0.f;
    float y = 0.f;
} ROBOT_STATE;

volatile ENC_STATE enc[2];
volatile ROBOT_STATE robot;

void setup() {
    /*IO設定*/
    pinMode(ENC_L_PIN, INPUT_PULLUP);
    pinMode(ENC_R_PIN, INPUT_PULLUP);

    /*エンコーダ割り込み設定*/
    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), enc_counter_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), enc_counter_R, CHANGE);

    /*タイマ割り込み設定*/
    /*使用可能なTimerの取得*/
    uint8_t type;
    int8_t ch = FspTimer::get_available_timer(type);
    if(ch < 0){
        return;
    }
    time_interrupt.begin(TIMER_MODE_PERIODIC, type, ch, 20.0f, 50.0f,timer_callback, nullptr);
    time_interrupt.setup_overflow_irq();
    time_interrupt.open();
    time_interrupt.start();

    Serial.begin(9600);
}

unsigned long pretime;

float rad1;

void loop() {
  unsigned long time;
  
  time = millis();
  if(time - pretime > 500){
    Serial.print(enc[0].count);
    Serial.print(",");
    Serial.print(enc[1].count);
    Serial.print(",");
    Serial.print(rad1);
    Serial.print(",");
    Serial.print(robot.headding);
    Serial.print(",");
    Serial.print(robot.x);
    Serial.print(",");
    Serial.println(robot.y);
    pretime = time;
  }


}

void enc_counter_L(){
    if(enc[0].rotate_forward){
        enc[0].count ++;
    }else{
        enc[0].count --;
    }
}

void enc_counter_R(){
    if(enc[1].rotate_forward){
        enc[1].count ++;
    }else{
        enc[1].count --;
    }
}

void timer_callback(timer_callback_args_t *arg){
    float enc_diff[2];
    float radius;
    float theta;
    float theta1;
    float x;
    float y;
    static int prev_enc[2];

    /*前回からの移動量*/
    enc_diff[0] = (enc[0].count - prev_enc[0]);
    enc_diff[1] = (enc[1].count - prev_enc[1]);
    /*エンコーダデータの更新*/
    prev_enc[0] = enc[0].count;
    prev_enc[1] = enc[1].count;


    /*エンコーダのカウントを移動距離[mm]に変換*/
    enc_diff[0] = TIER_DIAMETER * PI * enc_diff[0] / 12.f;
    enc_diff[1] = TIER_DIAMETER * PI * enc_diff[1] / 12.f;

    /*移動距離を位置と方向に変換*/
    theta = (enc_diff[0] - enc_diff[1])/ ROBOT_WIDTH;
    if(theta > 360) theta -= 360;
  
    if(enc_diff[0] == enc_diff[1]){
        robot.y += enc_diff[0];
    }else{
        radius = ROBOT_WIDTH * (enc_diff[0] + enc_diff[1]) / (enc_diff[0] - enc_diff[1]);
        rad1 = radius;
        theta1 = robot.headding * 2 * PI / 360;
        x = radius * (1 - cos(theta));
        y = radius * sin(theta);
        robot.x += y * sin(theta1) + x * cos(theta1);
        robot.y += y * cos(theta1) - x * sin(theta1);
        
    }
    robot.headding += 360 * theta / (2 * PI);
    if(robot.headding > 360) robot.headding -= 360;
}