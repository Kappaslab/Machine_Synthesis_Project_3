#include <WiFi.h>
#include "FspTimer.h"

/* Wi-Fi 設定 */
const char* ssid = "cafe_03";
const char* password = "123456789";
WiFiServer server(80);

/* ピン設定 */
#define ENC_L_PIN 2
#define ENC_R_PIN 3
#define Motor_L_A_PIN 7
#define Motor_L_B_PIN 8
#define Motor_L_PWM_PIN 9
#define Motor_R_A_PIN 11
#define Motor_R_B_PIN 12
#define Motor_R_PWM_PIN 10

#define VELOCITY_MAX 100

volatile bool enc_flag_L = false;
volatile bool enc_flag_R = false;

void setup() {
    // ピンモードの設定
    pinMode(ENC_L_PIN, INPUT_PULLUP);
    pinMode(ENC_R_PIN, INPUT_PULLUP);
    pinMode(Motor_L_A_PIN, OUTPUT);
    pinMode(Motor_L_B_PIN, OUTPUT);
    pinMode(Motor_L_PWM_PIN, OUTPUT);
    pinMode(Motor_R_A_PIN, OUTPUT);
    pinMode(Motor_R_B_PIN, OUTPUT);
    pinMode(Motor_R_PWM_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), enc_counter_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), enc_counter_R, CHANGE);

    // Wi-Fi接続
    Serial.begin(115200);
    WiFi.config(IPAddress(192, 48, 56, 2));
    WiFi.beginAP(ssid, password);

    while (WiFi.status() != WL_AP_LISTENING) {
        delay(500);
        Serial.println("Starting AP...");
    }
    Serial.println("AP started");
    Serial.println(WiFi.softAPIP());

    server.begin();
}

void loop() {
    WiFiClient client = server.available();

    if (client) {
        String command = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') {
                    parseCommand(command);
                    command = "";
                } else {
                    command += c;
                }
            }
        }
        client.stop();
    }
}

void parseCommand(String command) {
    float velocity = 0;
    float direction = 0;

    if (command.startsWith("MOVE")) {
        sscanf(command.c_str(), "MOVE %f %f", &velocity, &direction);
        move(Motor_L_A_PIN, Motor_L_B_PIN, Motor_L_PWM_PIN, Motor_R_A_PIN, Motor_R_B_PIN, Motor_R_PWM_PIN, velocity, direction);
    } else if (command == "STOP") {
        move(Motor_L_A_PIN, Motor_L_B_PIN, Motor_L_PWM_PIN, Motor_R_A_PIN, Motor_R_B_PIN, Motor_R_PWM_PIN, 0, 0);
    }
}

void enc_counter_L() {
    enc_flag_L = true;
}

void enc_counter_R() {
    enc_flag_R = true;
}

void move(int L_a_pin, int L_b_pin, int L_pwm_pin, int R_a_pin, int R_b_pin, int R_pwm_pin, float velocity, float direction) {
    float L_velocity = velocity;
    float R_velocity = velocity;

    if (direction >= 0) {
        R_velocity *= (1 - 2 * direction);
    } else {
        L_velocity *= (1 + 2 * direction);
    }

    int L_output = map(constrain(abs(L_velocity), 0, VELOCITY_MAX), 0, VELOCITY_MAX, 0, 255);
    int R_output = map(constrain(abs(R_velocity), 0, VELOCITY_MAX), 0, VELOCITY_MAX, 0, 255);

    digitalWrite(L_a_pin, L_velocity >= 0);
    digitalWrite(L_b_pin, L_velocity < 0);
    digitalWrite(R_a_pin, R_velocity >= 0);
    digitalWrite(R_b_pin, R_velocity < 0);

    analogWrite(L_pwm_pin, L_output);
    analogWrite(R_pwm_pin, R_output);
}
