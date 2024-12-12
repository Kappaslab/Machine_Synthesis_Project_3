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
#define INTERRUPT_FREQ 20//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 100//[mm/s]


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
    float v = 0.f;
} ROBOT_STATE;

volatile ENC_STATE enc[2];
volatile ROBOT_STATE robot;

volatile bool flag_L;
volatile bool flag_R;

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

    /*タイマ割り込み設定*/
    /*使用可能なTimerの取得*/
    uint8_t type;
    int8_t ch = FspTimer::get_available_timer(type);
    if(ch < 0){
        return;
    }
    time_interrupt.begin(TIMER_MODE_PERIODIC, type, ch, INTERRUPT_FREQ, 50.0f,timer_callback, nullptr);
    time_interrupt.setup_overflow_irq();
    time_interrupt.open();
    time_interrupt.start();

    Serial.begin(9600);
}



float rad1;

void loop() {
    unsigned long current_time;
    static unsigned long pretime;
    static int step = 0;
    static int cup_num = 0;

    
    current_time = millis();
    if(current_time - pretime > 500){
        Serial.print(current_time);
        Serial.print(",");
        Serial.print(enc[0].count);
        Serial.print(",");
        Serial.print(enc[1].count);
        Serial.print(",");
        Serial.print(robot.headding);
        Serial.print(",");
        Serial.print(robot.x);
        Serial.print(",");
        Serial.println(robot.y);
        pretime = current_time;
    }
    

    if(robot.headding < 90){
        move(Motor_L_A_PIN, Motor_L_B_PIN, Motor_L_PWM_PIN, Motor_R_A_PIN, Motor_R_B_PIN, Motor_R_PWM_PIN, 100, 1);
    }else{
        move(Motor_L_A_PIN, Motor_L_B_PIN, Motor_L_PWM_PIN, Motor_R_A_PIN, Motor_R_B_PIN, Motor_R_PWM_PIN, 0, 0);
    }
    
}

void enc_counter_L(){
    flag_L = true;
}

void enc_counter_R(){
    flag_R = true;
}

void timer_callback(timer_callback_args_t *arg){
    float enc_diff[2];
    float radius;
    float global_theta;
    float local_theta;
    float local_x;
    float local_y;
    static int prev_enc[2];
    static int counter = 0;

    if(flag_L){
        if(enc[0].rotate_forward){
            enc[0].count ++;
        }else{
            enc[0].count --;
        }
        flag_L = false;
    }
    
    if(flag_R){
        if(enc[1].rotate_forward){
            enc[1].count ++;
        }else{
            enc[1].count --;
        }
        flag_R = false;
    }
    counter++;
    if(counter == 2){
        digitalWrite(TEST_PIN, HIGH);
        counter = 0;
        /*前回からの移動量*/
        enc_diff[0] = (enc[0].count - prev_enc[0]);
        enc_diff[1] = (enc[1].count - prev_enc[1]);
        /*エンコーダデータの更新*/
        prev_enc[0] = enc[0].count;
        prev_enc[1] = enc[1].count;

        /*割り込みを許可*/
        interrupts();

        /*エンコーダのカウントを移動距離[mm]に変換*/
        enc_diff[0] = TIER_DIAMETER * PI * enc_diff[0] / ENC_SLIT;
        enc_diff[1] = TIER_DIAMETER * PI * enc_diff[1] / ENC_SLIT;

        /*移動距離を位置と方向に変換*/
        local_theta = (enc_diff[0] - enc_diff[1])/ ROBOT_WIDTH;
        if(local_theta > 360) local_theta -= 360;
    
        if(enc_diff[0] == enc_diff[1]){
            robot.y += enc_diff[0];
        }else{
            radius = ROBOT_WIDTH * (enc_diff[0] + enc_diff[1]) / (enc_diff[0] - enc_diff[1]);
            rad1 = radius;
            global_theta = robot.headding * 2 * PI / 360;
            local_x = radius * (1 - cos(local_theta)) / 4 ; //何故か４で割らねばならない
            local_y = radius * sin(local_theta) / 4 ; //何故か４で割らねばならない
            robot.x += local_y * sin(global_theta) + local_x * cos(global_theta);
            robot.y += local_y * cos(global_theta) - local_x * sin(global_theta);
        }
        robot.headding += 360 * local_theta / (2 * PI * 1.25); //何故か1.25で割らねばならない
        if(robot.headding > 360) robot.headding -= 360;

        /*速度情報の更新*/
        robot.v_L = enc_diff[0] * 10;
        robot.v_R = enc_diff[1] * 10;
        robot.v = (robot.v_L + robot.v_R) * 10 / 2;
    }
}

void move(int L_a_pin, int L_b_pin, int L_pwm_pin, int R_a_pin, int R_b_pin, int R_pwm_pin, float velocity,float direction){
    float L_velocity;
    float R_velocity;
    int L_output;
    int R_output;

    /*速度の最大限を決定*/
    if(velocity > VELOCITY_MAX) velocity = VELOCITY_MAX;
    if(velocity < -VELOCITY_MAX) velocity = -VELOCITY_MAX;

    /*曲率の最大限を決定*/
    if(direction > DIRECTION_MAX) direction = DIRECTION_MAX;
    if(direction < -DIRECTION_MAX) direction = -DIRECTION_MAX;
    noInterrupts();
    /*車輪の移動速度に変換*/
    L_velocity = velocity;
    R_velocity = velocity;
    if(direction >= 0){
        R_velocity = R_velocity * (1 - 2 * direction);
    }else{
        L_velocity = L_velocity * (1 + 2 * direction);
    }
    /*絶対値をとる*/
    if(L_velocity < 0){
        enc[0].rotate_forward = false;
        L_velocity = -L_velocity;
    }else{
        enc[0].rotate_forward = true;
    }
    if(R_velocity < 0){
        enc[1].rotate_forward = false;
        R_velocity = -R_velocity;
    }else{
        enc[1].rotate_forward = true;
    }
    interrupts();

    /*いいかんじに速度を出力に変換*/
    L_output = map(L_velocity, 0, VELOCITY_MAX, 0 , 255);//仮
    R_output = map(R_velocity, 0, VELOCITY_MAX, 0 , 255);//仮

    /*最大値の制限*/
    if(L_output > 255) L_output = 255;
    if(R_output > 255) R_output = 255;

    /*出力*/
    digitalWrite(L_a_pin, enc[0].rotate_forward);
    digitalWrite(L_b_pin, !enc[0].rotate_forward);
    digitalWrite(R_a_pin, enc[1].rotate_forward);
    digitalWrite(R_b_pin, !enc[1].rotate_forward);
    analogWrite(L_pwm_pin, L_output);
    analogWrite(R_pwm_pin, R_output);
}