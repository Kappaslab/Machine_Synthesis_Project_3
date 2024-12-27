#include <WiFi.h>
#include "FspTimer.h"
#include"pwm.h"

/*pin assign*/
#define ENC_L_PIN 2
#define ENC_R_PIN 3

#define Motor_L_A_PIN 7
#define Motor_L_B_PIN 8
#define Motor_L_PWM_PIN 9
#define Motor_R_A_PIN 11
#define Motor_R_B_PIN 12
#define Motor_R_PWM_PIN 10

#define Servo_PWM_PIN 6

#define TEST_PIN 13

/*定数*/
#define PI 3.141593
#define TIER_RADIUS 18.5 //[mm]
#define ROBOT_WIDTH 104 //[mm]
#define ENC_SLIT 40
#define INTERRUPT_FREQ 500//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 55//[mm/s]
#define POWER_MAX 50//[mm/s]
#define WMA_NUM 3

/* Wi-Fi 設定 */
const char* ssid = "cafe_03";
const char* password = "123456789";
byte IP[] = { 192, 48, 56, 1 };
int PORT = 80;
int status = WL_IDLE_STATUS;

WiFiServer server(PORT);
FspTimer time_interrupt;
PwmOut MotorL(Motor_L_PWM_PIN);
PwmOut MotorR(Motor_R_PWM_PIN);
PwmOut Servo(Servo_PWM_PIN);

/*value*/
typedef struct enc_str{
    int count = 0;
    bool moved = false;
    bool rotate_forward = true;
    unsigned long time = 0;
    float omega[WMA_NUM] = {0.f};
    float WMA_omega = 0.f;
    float WMA_total = 0.f;
    float WMA_numerator = 0.f;
} ENCODER;

typedef struct rbt_str{
    float headding = 0.f;
    float x = 0.f;
    float y = 0.f;
    float rho = 0.f;
    float v = 0.f;
} ROBOT_STATE;

typedef struct {
    float target;
    float target_diff;
    float integral;
    float differential;
    float output;
    float kp;
    float ki;
    float kd;
    float max;
} PID_data;

volatile ENCODER enc[2];
volatile ROBOT_STATE robot;
volatile PID_data speed_data[2], angle_data;

void setup() {
    /*IO設定*/
    /*初期化*/
    digitalWrite(ENC_L_PIN, LOW);
    digitalWrite(ENC_R_PIN, LOW);

    digitalWrite(Motor_L_A_PIN, LOW);
    digitalWrite(Motor_L_B_PIN, LOW);
    digitalWrite(Motor_L_PWM_PIN, LOW);
    digitalWrite(Motor_R_A_PIN, LOW);
    digitalWrite(Motor_R_B_PIN, LOW);
    digitalWrite(Motor_R_PWM_PIN, LOW);

    digitalWrite(Servo_PWM_PIN, LOW);

    digitalWrite(TEST_PIN, LOW);

    /*ピンモード*/
    pinMode(ENC_L_PIN, INPUT_PULLUP);
    pinMode(ENC_R_PIN, INPUT_PULLUP);

    pinMode(Motor_L_A_PIN, OUTPUT);
    pinMode(Motor_L_B_PIN, OUTPUT);
    pinMode(Motor_L_PWM_PIN, OUTPUT);
    pinMode(Motor_R_A_PIN, OUTPUT);
    pinMode(Motor_R_B_PIN, OUTPUT);
    pinMode(Motor_R_PWM_PIN, OUTPUT);

    pinMode(Servo_PWM_PIN, OUTPUT);

    pinMode(TEST_PIN, OUTPUT);

    /*シリアル通信*/
    Serial.begin(9600);

    /*Wi-Fi通信*/
    Wifi_setup();

    /*エンコーダ割り込み設定*/
    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), enc_counter_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), enc_counter_R, CHANGE);

    /*タイマ割り込み設定*/
    if(time_interrupt_setup() == -1) return;

    /*PWMスタート*/
    MotorL.begin(5000, 0.f);
    MotorR.begin(5000, 0.f);
    Servo.begin(50, 0.f);
    Servo.suspend();
}

void loop(){
    WiFiClient client = server.available();

    Serial.println(enc[0].WMA_omega);
}

void enc_counter_L(){
    enc[0].moved = true;
}

void enc_counter_R(){
    enc[1].moved = true;
}

void timer_callback(timer_callback_args_t *arg){
    digitalWrite(TEST_PIN, HIGH);
    static int count = 0;
    float t = 0.1;
    float v_L = 0.f;
    float v_R = 0.f;
    float x_local = 0.f;
    float y_local = 0.f;
    float d_theta = 0.f;
    encorder_counter(0);
    encorder_counter(1);
    if(count == 0){
        /*カウンターリセット*/
        count = INTERRUPT_FREQ * t;

        /*状態推定*/
        /*速度算出*/
        v_L = enc[0].WMA_omega * TIER_RADIUS;
        v_R = enc[1].WMA_omega * TIER_RADIUS;
        robot.v = (v_L + v_R) / 2;
        /*ロボットの旋回曲率の算出*/
        robot.rho = 2 * (v_L - v_R) / (ROBOT_WIDTH * (v_L + v_R));
        /*自己位置推定*/
        if(v_L = v_R){
            robot.y += v_L * t;
        }else{
            d_theta = (v_L - v_R) / ROBOT_WIDTH;
            x_local = (1 - cos(d_theta * t)) / robot.rho;
            y_local = sin(d_theta * t) / robot.rho;
            /*グローバル変換*/
            robot.x += y_local * sin(robot.headding) + x_local * cos(robot.headding);
            robot.y += y_local * cos(robot.headding) - x_local * sin(robot.headding);
            robot.headding += d_theta;
        }

        /*各部PID*/
        angle_pid();
        if(angle_data.output >= 0){
            speed_data[1].target -= angle_data.output;
        }else{
            speed_data[0].target -= angle_data.output;
        }
        speed_pid(0);
        speed_pid(1);

        /*モータ出力*/
        motor_output(speed_data[0].output, speed_data[0].output);
    }
    count--;
    digitalWrite(TEST_PIN, LOW);
}

/*サーボ*/
void my_servo(int angle){
    Servo.resume();
    Servo.pulseWidth_us(map(angle,-90, 90,553, 2347));
}

/*エンコーダのカウントとパルスの間隔からの角速度の導出*/
void encorder_counter(int enc_num){
    int i;
    unsigned long prev_time;
    unsigned long diff_time;

    if(enc[enc_num].moved){
        for(i = WMA_NUM - 1; i > 0; i--){
            enc[enc_num].omega[i] = enc[enc_num].omega[i - 1];
        }
        /*角速度の計算*/
        prev_time = enc[enc_num].time;
        enc[enc_num].time = micros();
        diff_time = enc[enc_num].time - prev_time;
        enc[enc_num].omega[0] = 2000000 * PI / (diff_time * ENC_SLIT);
        /*パルスカウント*/
        if(enc[enc_num].rotate_forward){
            enc[enc_num].count ++;
        }else{
            enc[enc_num].count --;
            enc[enc_num].omega[0] *= -1;
        }
        /*加重移動平均*/
        enc[enc_num].WMA_total += enc[enc_num].omega[0] - enc[enc_num].omega[WMA_NUM - 1];
        enc[enc_num].WMA_numerator += WMA_NUM * enc[enc_num].omega[0] - enc[enc_num].WMA_total;
        enc[enc_num].WMA_omega = 2 * enc[enc_num].WMA_numerator / (WMA_NUM * (WMA_NUM + 1));
        /*フラグの初期化*/
        enc[enc_num].moved = false;
    }
}

/*Wifi通信のセットアップ*/
void Wifi_setup(){
    WiFi.config(IPAddress(IP));
    WiFi.beginAP(ssid, password);
    while (WiFi.status() != WL_AP_LISTENING) {
        delay(500);
        Serial.println("Starting AP...");
    }
    Serial.println("AP started");
    Serial.println(WiFi.softAPIP());
    server.begin();
}

/*タイマ割り込み設定*/
int time_interrupt_setup(){
    uint8_t type;
    int8_t ch = FspTimer::get_available_timer(type);
    if(ch < 0){
        Serial.println("Can't get availabletimer...");
        return -1;
    }
    time_interrupt.begin(TIMER_MODE_PERIODIC, type, ch, INTERRUPT_FREQ, 50.0f,timer_callback, nullptr);
    time_interrupt.setup_overflow_irq();
    time_interrupt.open();
    time_interrupt.start();
    return 0;
}

void move_data(float velocity,float direction){
    float L_velocity;
    float R_velocity;
    float ideal_rho;
    float L_omega;
    float R_omega;
    int L_output;
    int R_output;

    /*速度の最大限を決定*/
    if(velocity > VELOCITY_MAX) velocity = VELOCITY_MAX;
    if(velocity < -VELOCITY_MAX) velocity = -VELOCITY_MAX;

    /*曲率の最大限を決定*/
    if(direction > DIRECTION_MAX) direction = DIRECTION_MAX;
    if(direction < -DIRECTION_MAX) direction = -DIRECTION_MAX;

    /*車輪の移動速度に変換*/
    L_velocity = velocity;
    R_velocity = velocity;
    if(direction >= 0){
        R_velocity = R_velocity * (1 - 2 * direction);
    }else{
        L_velocity = L_velocity * (1 + 2 * direction);
    }
        /*フィードバックのための下ごしらえ*/
    ideal_rho = 2 * (L_velocity - R_velocity) / (ROBOT_WIDTH * (L_velocity + R_velocity));
    L_omega = L_velocity / TIER_RADIUS;
    R_omega = R_velocity / TIER_RADIUS;

    /*モータ回転方向の設定*/
    noInterrupts();
    enc[0].rotate_forward = L_omega >= 0;
    enc[1].rotate_forward = R_omega >= 0;

    angle_data.target = ideal_rho;
    speed_data[0].target = L_omega;
    speed_data[1].target = R_omega;
    interrupts();
}

void motor_output(int L_output ,int R_output){

    /*絶対値を取る*/
    L_output = abs(L_output);
    R_output = abs(R_output);

    /*最大値の制限*/
    L_output = min(L_output, POWER_MAX);
    R_output = min(R_output, POWER_MAX);

    /*出力*/
    if(L_output == 0){
        /*ブレーキ*/
        digitalWrite(Motor_L_A_PIN, HIGH);
        digitalWrite(Motor_L_B_PIN, HIGH);
    }else{
        digitalWrite(Motor_L_A_PIN, enc[0].rotate_forward);
        digitalWrite(Motor_L_B_PIN, !enc[0].rotate_forward);
    }
    if(R_output == 0){
        /*ブレーキ*/
        digitalWrite(Motor_R_A_PIN, HIGH);
        digitalWrite(Motor_R_B_PIN, HIGH);
    }else{
        digitalWrite(Motor_R_A_PIN, enc[1].rotate_forward);
        digitalWrite(Motor_R_B_PIN, !enc[1].rotate_forward);
    }
    MotorL.pulse_perc(L_output);
    MotorR.pulse_perc(R_output);
}

void angle_pid(){

}

void speed_pid(int i){

}