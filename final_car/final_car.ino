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
PwmOut MotorL(Motor_L_PWM_PIN);
PwmOut MotorR(Motor_R_PWM_PIN);
PwmOut Servo(Servo_PWM_PIN);

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
    WiFi.config(IPAddress(IP));
    WiFi.beginAP(ssid, password);
    while (WiFi.status() != WL_AP_LISTENING) {
        delay(500);
        Serial.println("Starting AP...");
    }
    Serial.println("AP started");
    Serial.println(WiFi.softAPIP());
    server.begin();

    /*エンコーダ割り込み設定*/
    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), enc_counter_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), enc_counter_R, CHANGE);

    /*タイマ割り込み設定*/
    /*使用可能なTimerの取得*/
    uint8_t type;
    int8_t ch = FspTimer::get_available_timer(type);
    if(ch < 0){
        Serial.println("Can't get availabletimer...");
        return;
    }
    time_interrupt.begin(TIMER_MODE_PERIODIC, type, ch, INTERRUPT_FREQ, 50.0f,timer_callback, nullptr);
    time_interrupt.setup_overflow_irq();
    time_interrupt.open();
    time_interrupt.start();

    /*PWMスタート*/
    MotorL.begin(5000, 0.f);
    MotorR.begin(5000, 0.f);
    Servo.begin(50, 0.f);
    Servo.suspend();
}

void loop(){
    WiFiClient client = server.available();


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

void my_servo(int angle){
    Servo.resume();
    Servo.pulseWidth_us(map(angle,-90, 90,500, 2400));
}