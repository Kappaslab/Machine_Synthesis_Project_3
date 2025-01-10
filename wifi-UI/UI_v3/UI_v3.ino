#include <Servo.h>

/* ピンアサイン */
#define Motor_L_A_PIN 7
#define Motor_L_B_PIN 8
#define Motor_L_PWM_PIN 9
#define Motor_R_A_PIN 11
#define Motor_R_B_PIN 12
#define Motor_R_PWM_PIN 10
#define Servo_PIN 6 // サーボモーターのピン
#define LED 13
#define Thermistor_PIN A0 // サーミスタのアナログピン

/* 定数 */
#define MOTOR_SPEED 255 // 最大速度

Servo servoMotor;
bool isGrasping = false; // つかむ状態を保持

void setup() {
    pinMode(Motor_L_A_PIN, OUTPUT);
    pinMode(Motor_L_B_PIN, OUTPUT);
    pinMode(Motor_L_PWM_PIN, OUTPUT);
    pinMode(Motor_R_A_PIN, OUTPUT);
    pinMode(Motor_R_B_PIN, OUTPUT);
    pinMode(Motor_R_PWM_PIN, OUTPUT);

    Serial.begin(9600); // シリアル通信の初期化

    servoMotor.attach(Servo_PIN); // サーボモーターのピンを設定
    servoMotor.write(0); // 初期状態は0度
}

void loop() {
    // サーミスタの値を読み取って送信
    int thermistorValue = analogRead(Thermistor_PIN);
    Serial.println("THERMISTOR " + String(thermistorValue));

    // 通常のコマンド処理
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println("Received command: " + command); // デバッグ用

        if (command == "MOVE FORWARD") {
            moveForward();
        } else if (command == "MOVE BACKWARD") {
            moveBackward();
        } else if (command == "MOVE LEFT") {
            moveLeft();
        } else if (command == "MOVE RIGHT") {
            moveRight();
        } else if (command == "STOP") {
            stopMotors();
        } else if (command == "GRASP ON") {
            graspOn();
        } else if (command == "GRASP OFF") {
            graspOff();
        }
    }
    delay(100); // 過剰な送信を防ぐための短い待機
}


void moveForward() {
    analogWrite(Motor_L_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_L_A_PIN, HIGH);
    digitalWrite(Motor_L_B_PIN, LOW);

    analogWrite(Motor_R_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_R_A_PIN, HIGH);
    digitalWrite(Motor_R_B_PIN, LOW);
    Serial.println("Moving forward");
}

void moveBackward() {
    analogWrite(Motor_L_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_L_A_PIN, LOW);
    digitalWrite(Motor_L_B_PIN, HIGH);

    analogWrite(Motor_R_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_R_A_PIN, LOW);
    digitalWrite(Motor_R_B_PIN, HIGH);
    Serial.println("Moving backward");
}

void moveLeft() {
    analogWrite(Motor_R_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_R_A_PIN, HIGH);
    digitalWrite(Motor_R_B_PIN, LOW);

    analogWrite(Motor_L_PWM_PIN, 0);
    digitalWrite(Motor_L_A_PIN, LOW);
    digitalWrite(Motor_L_B_PIN, LOW);
    Serial.println("Turning left");
}

void moveRight() {
    analogWrite(Motor_L_PWM_PIN, MOTOR_SPEED);
    digitalWrite(Motor_L_A_PIN, HIGH);
    digitalWrite(Motor_L_B_PIN, LOW);

    analogWrite(Motor_R_PWM_PIN, 0);
    digitalWrite(Motor_R_A_PIN, LOW);
    digitalWrite(Motor_R_B_PIN, LOW);
    Serial.println("Turning right");
}

void stopMotors() {
    analogWrite(Motor_L_PWM_PIN, 0);
    digitalWrite(Motor_L_A_PIN, LOW);
    digitalWrite(Motor_L_B_PIN, LOW);

    analogWrite(Motor_R_PWM_PIN, 0);
    digitalWrite(Motor_R_A_PIN, LOW);
    digitalWrite(Motor_R_B_PIN, LOW);
    Serial.println("Stopping motors");
}

void graspOn() {
    isGrasping = true;
    servoMotor.write(-30); // サーボモーターを90度に回転
    Serial.println("Grasping: ON");
}

void graspOff() {
    isGrasping = false;
    servoMotor.write(90); // サーボモーターを0度に戻す
    Serial.println("Grasping: OFF");
}

