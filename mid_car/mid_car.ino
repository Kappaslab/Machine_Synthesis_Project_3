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
    float v_L = 0.f;
    float v_R = 0.f;
} ROBOT_STATE;

volatile ENC_STATE enc[2];
volatile ROBOT_STATE robot;

void setup() {
    /*IO設定*/
    pinMode(ENC_L_PIN, INPUT_PULLUP);
    pinMode(ENC_R_PIN, INPUT_PULLUP);

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
    Serial.print(time);
    Serial.print(",");
    Serial.print(robot.v_L + robot.v_R);
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
    int enc_diff[2];
    float radius;
    float global_theta;
    float local_theta;
    float local_x;
    float local_y;
    static int prev_enc[2];

    /*前回からの移動量*/
    enc_diff[0] = (enc[0].count - prev_enc[0]);
    enc_diff[1] = (enc[1].count - prev_enc[1]);
    /*エンコーダデータの更新*/
    prev_enc[0] = enc[0].count;
    prev_enc[1] = enc[1].count;

    /*割り込みを許可*/
    interrupts();

    /*エンコーダのカウントを移動距離[mm]に変換*/
    robot.v_L = TIER_DIAMETER * PI * enc_diff[0] / 12.f;
    robot.v_R = TIER_DIAMETER * PI * enc_diff[1] / 12.f;

    /*移動距離を位置と方向に変換*/
    local_theta = (robot.v_L - robot.v_R)/ ROBOT_WIDTH;
    if(local_theta > 360) theta -= 360;
  
    if(robot.v_L == robot.v_R){
        robot.y += robot.v_L;
    }else{
        radius = ROBOT_WIDTH * (robot.v_L + robot.v_R) / (robot.v_L - robot.v_R);
        rad1 = radius;
        global_theta = robot.headding * 2 * PI / 360;
        local_x = radius * (1 - cos(local_theta));
        local_y = radius * sin(local_theta);
        robot.x += local_y * sin(global_theta) + local_x * cos(global_theta);
        robot.y += local_y * cos(global_theta) - local_x * sin(global_theta);
        
    }
    robot.headding += 360 * local_theta / (2 * PI);
    if(robot.headding > 360) robot.headding -= 360;
}