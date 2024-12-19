#include <WiFi.h>
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
#define TIER_DIAMETER 37 //[mm]
#define ROBOT_WIDTH 104 //[mm]
#define ENC_SLIT 40
#define INTERRUPT_FREQ 200//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 100//[mm/s]

/* Wi-Fi 設定 */
const char* ssid = "cafe_03";
const char* password = "123456789";
byte IP[] = { 192, 48, 56, 1 };
int PORT = 80;
int status = WL_IDLE_STATUS;
WiFiServer server(PORT);

FspTimer time_interrupt;

/*value*/
typedef struct enc_str{
    int count = 0;
    bool moved = false;
    bool rotate_forward = true;
} ENCODER;

typedef struct rbt_str{
    float headding = 0.f;
    float x = 0.f;
    float y = 0.f;
    float v_L = 0.f;
    float v_R = 0.f;
    float v = 0.f;
} ROBOT_STATE;

volatile ENCODER enc[2];
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

    /*通信設定*/
    /*シリアル通信*/
    Serial.begin(9600);
    /*Wi-Fi通信*/
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

void loop(){

}

void enc_counter_L(){
    enc[0].moved = true;
}

void enc_counter_R(){
    enc[1].moved = true;
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