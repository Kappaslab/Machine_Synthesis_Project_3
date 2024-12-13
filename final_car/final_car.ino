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
#define TIER_DIAMETER 35 //[mm]
#define ROBOT_WIDTH 104 //[mm]
#define ENC_SLIT 40
#define INTERRUPT_FREQ 20//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 100//[mm/s]


FspTimer time_interrupt;

/* Wi-Fi 設定 */
const char* ssid = "cafe_03";
const char* password = "123456789";
WiFiServer server(80);

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
    WiFi.config(IPAddress(192, 48, 56, 1));
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