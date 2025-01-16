//#include <WiFi.h>
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

#define Thermistor_PIN A0

#define TEST_PIN 13

/*定数*/
#define PI 3.141593
#define TIER_RADIUS 18.5 //[mm]
#define ROBOT_WIDTH 104 //[mm]
#define ENC_SLIT 40
#define INTERRUPT_FREQ 500//[Hz]
#define DIRECTION_MAX 1
#define VELOCITY_MAX 55//[mm/s]
#define POWER_MAX 50
#define WMA_NUM 5

#define MOTOR_L_KP 5
#define MOTOR_L_KI 3
#define MOTOR_L_KD 0
#define MOTOR_L_MAX 50

#define MOTOR_R_KP 5
#define MOTOR_R_KI 3
#define MOTOR_R_KD 0
#define MOTOR_R_MAX 50

#define ANGLE_KP 5
#define ANGLE_KI 3
#define ANGLE_KD 0
#define ANGLE_MAX 15

// /* Wi-Fi 設定 */
// const char* ssid = "cafe_03";
// const char* password = "123456789";
// byte IP[] = { 192, 48, 56, 1 };
// int PORT = 80;
// int status = WL_IDLE_STATUS;

// WiFiServer server(PORT);
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
    float omega[WMA_NUM + 1] = {0.f};
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

enum{
    Section1,//1つ目の目標に移動して把持
    Section2,//1つ目の目標をゴールまで運ぶ
    Section3,//スタート位置まで戻る
    Section4,//2つ目の目標に移動して把持
    Section5,//2つ目の目標をゴールまで運ぶ
    Section6,//スタート位置まで戻る
    Section7,//3つ目の目標に移動して把持
    Section8,//4つ目の目標をゴールまで運ぶ
    Section9,//スタート位置まで戻る
    Section10,//4つ目の目標に移動して把持
    Section11,//4つ目の目標をゴールまで運ぶ
    Section12,//スタート位置まで戻る
};

volatile ENCODER enc[2];
volatile ROBOT_STATE robot;
volatile PID_data speed_data[2], angle_data;
char message[30];
char c = '.';

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

    /*PIDゲイン設定*/
    speed_data[0].kp = MOTOR_L_KP;
    speed_data[0].ki = MOTOR_L_KI;
    speed_data[0].kd = MOTOR_L_KD;
    speed_data[0].max = MOTOR_L_MAX;

    speed_data[1].kp = MOTOR_R_KP;
    speed_data[1].ki = MOTOR_R_KI;
    speed_data[1].kd = MOTOR_R_KD;
    speed_data[1].max = MOTOR_R_MAX;

    /*シリアル通信*/
    Serial.begin(9600);

    /*Wi-Fi通信*/
    //Wifi_setup();

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
    grab(false);
}

void loop(){
    static int i = 0;
    static int section = 0;
    static bool grab_state = "false";
    static String command;

    int thermistorValue = analogRead(Thermistor_PIN);
    //Serial.println("THERMISTOR " + String(thermistorValue));

    //grab_state = grab(!grab_state);

    //servo_test();

    //  通常のコマンド処理
    // if (Serial.available()) {
    //     command = Serial.readStringUntil('\n');
    //     command.trim();
    //     Serial.println("Received command: " + command); // デバッグ用
    // }

    if (command == "MOVE FORWARD") {
        move_data(40, 0);
    } else if (command == "MOVE BACKWARD") {
        move_data(-40, 0);
    } else if (command == "MOVE LEFT") {
        move_data(40, -1);
    } else if (command == "MOVE RIGHT") {
        move_data(40, 1);
    } else if (command == "STOP") {
        move_data(0,0);
        //movedata();
    } else if (command == "GRASP ON") {
        grab_state = grab(true);
    } else if (command == "GRASP OFF") {
        grab_state = grab(false);
    } else if (command == "AUTONOMUS ON"){
        if(millis() > 5000) sectiono = local_logic(section, &grab_state);
    }

    if(millis() > 5000) sectiono = local_logic(section, &grab_state);;
    //if(millis() > 5000) move_data(55, 0);

    // // アクセスポイントに他のデバイスがつながるのを待つ
    // if (WiFi.status() == WL_AP_CONNECTED) {
    //     //接続されているクライアントを確認
    //     WiFiClient client = server.available();
    //     if (!client) {
    //         Serial.println("NO CLIENT");

    //         return;
    //     }
    //     if (!client.connected()) {
    //         client.stop();  //接続が切れてたらクライアントを終了
    //         return;
    //     }
    //     if (client.available() <= 0) {
    //         return;  // データが来なかったらなにもしない
    //     }

    //     c = client.read();
    //     // 以下はコマンドの解釈．'U'のときのみ，A0ピンの値をPCに送信する．
    //     // それ以外のときは，コマンドの文字をそのままLEDに表示する．
    //     switch (c) {
    //         case 'U':  //センサ値を符号なし2バイトで送信
    //             client.println("50"); 
    //             break;
    //         default:
    //             sprintf(message, "%c  ", c);
    //             Serial.println("message");  //受け取った文字をLEDに表示
    //             break;
    //     }
    // }else{
    //     //Serial.println("No device");
        Serial.print(section);
    //     Serial.print(",");
    //     my_servo(90);
    //     // servo_test();
        // Serial.print(speed_data[0].target);
        // Serial.print(",");
        // Serial.print(speed_data[1].target);
        // Serial.print(",");
        // Serial.print(enc[0].WMA_omega);
        // Serial.print(",");
        // Serial.print(enc[1].WMA_omega);
        // Serial.print(",");
        // Serial.print(speed_data[0].output);
        // Serial.print(",");
        // Serial.println(speed_data[1].output);
        Serial.print(robot.headding);
        Serial.print(",");
        Serial.print(robot.x);
        Serial.print(",");
        Serial.print(robot.y);
        Serial.print(",");
        Serial.print(robot.v);
        Serial.print(",");
        Serial.print(enc[0].WMA_omega);
        Serial.print(",");
        Serial.print(enc[1].WMA_omega);
        Serial.print(",");
        Serial.print(speed_data[0].output);
        Serial.print(",");
        Serial.println(speed_data[1].output);
    //     // Serial.print(enc[0].omega[0]);
    //     // Serial.print(",");
    //     // Serial.print(enc[0].omega[1]);
    //     // Serial.print(",");
    //     // Serial.print(enc[0].omega[2]);
    //     // Serial.print(",");
    //     // Serial.print(enc[0].omega[3]);
    //     // Serial.print(",");
    //     // Serial.print(enc[0].omega[4]);
    //     // Serial.print(",");
    //     // Serial.print(enc[0].WMA_omega);
    //     // Serial.print(",");
    //     // Serial.print(speed_data[0].output);
    //     // Serial.print(",");
    //     // Serial.println(speed_data[1].output);
    //     //if(millis() > 5000) move_data(55, 0);
    //     section = v
    //     grab_state = grab(!grab_state);
    // }
    

}

void enc_counter_L(){
    enc[0].moved = true;
}

void enc_counter_R(){
    enc[1].moved = true;
}

void timer_callback(timer_callback_args_t *arg){
    //digitalWrite(TEST_PIN, HIGH);
    static int count = 0;
    static int prev_enc_counter[2];
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

        /*角速度が小さい時対策*/
        enc_zero(0, prev_enc_counter[0]);
        enc_zero(1, prev_enc_counter[1]);
        prev_enc_counter[0] = enc[0].count;
        prev_enc_counter[1] = enc[1].count;

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
            speed_data[1].target *= (1 - angle_data.output);
        }else{
            speed_data[0].target *= (1 - angle_data.output);
        }
        speed_pid(0);
        speed_pid(1);

        /*モータ出力*/
        motor_output(speed_data[0].output, speed_data[1].output);
    }
    count--;
    //digitalWrite(TEST_PIN, LOW);
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
        for(i = WMA_NUM; i > 0; i--){
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
        calc_wma(enc_num);
        /*フラグの初期化*/
        enc[enc_num].moved = false;
    }
}

/*回転数が小さいときは0で埋める*/
void enc_zero(int enc_num, int prev_count){
    int i;

    if(enc[enc_num].count == prev_count){
        for(i = WMA_NUM; i > 0; i--){
            enc[enc_num].omega[i] = enc[enc_num].omega[i - 1];
        }
        enc[enc_num].omega[0] = 0;
        calc_wma(enc_num);
    }
    digitalWrite(TEST_PIN, !digitalRead(TEST_PIN));
}

// /*Wifi通信のセットアップ*/
// void Wifi_setup(){
//     WiFi.config(IPAddress(IP));
//     WiFi.beginAP(ssid, password);
//     while (WiFi.status() != WL_AP_LISTENING) {
//         delay(500);
//         Serial.println("Starting AP...");
//     }
//     Serial.println("AP started");
//     Serial.println(WiFi.softAPIP());
//     server.begin();
// }

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

/*速さと向きから各モータの速度を生成*/
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

    noInterrupts();
    /*モータ回転方向の設定*/
    enc[0].rotate_forward = L_omega >= 0;
    enc[1].rotate_forward = R_omega >= 0;
    /*PIDの目標値に設定*/
    angle_data.target = ideal_rho;
    speed_data[0].target = L_omega;
    speed_data[1].target = R_omega;
    interrupts();
}

/*モータの回転方向の指定とPWM周波数の変更*/
void motor_output(float L_output ,float R_output){

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

/*直進補正*/
void angle_pid(){
    //angle_data.target_diff = angle_data.target - robot.rho;
    angle_data.output = 0.1;
}

/*速度制御*/
void speed_pid(int i){
    speed_data[i].integral = speed_data[i].target - enc[i].WMA_omega;
    speed_data[i].target_diff = speed_data[i].integral / 0.1;
    speed_data[i].differential = speed_data[i].target_diff / 0.1;
    speed_data[i].output += speed_data[i].kp * speed_data[i].target_diff + speed_data[i].ki * speed_data[i].integral + speed_data[i].ki * speed_data[i].differential;
    //speed_data[i].output = (speed_data[i].target + PI) * 100 / (2 * PI) -50;//おためし
    if(speed_data[i].output > speed_data[i].max) speed_data[i].output = speed_data[i].max;
    if(speed_data[i].output < -speed_data[i].max) speed_data[i].output = -speed_data[i].max;

}

/*加重平均速度の算出*/
void calc_wma(int enc_num){
    /*加重移動平均*/
    enc[enc_num].WMA_total += enc[enc_num].omega[0] - enc[enc_num].omega[WMA_NUM];
    enc[enc_num].WMA_numerator += WMA_NUM * enc[enc_num].omega[0] - enc[enc_num].WMA_total;
    enc[enc_num].WMA_omega = 2 * enc[enc_num].WMA_numerator / (WMA_NUM * (WMA_NUM + 1));
}

void servo_test(){
    static int i = 0;

    if(i == 0){
        my_servo(-30);
        delay(5000);
    }
    for(i = -30; i < 90; i += 10){
        my_servo(i);
        Serial.println(i);
        delay(200);
    }
    for(i = 90; i > -30; i -= 10){
        my_servo(i);
        Serial.println(i);
        delay(200);
    }
}

int local_logic(int section, bool *grab_state){
    float target_position[2];

    switch(section){
        case Section1:
            target_position[0] = 100;
            target_position[1] = 300;
            break;
        case Section2:
            target_position[0] = 100;
            target_position[1] = 800;
            break;
        case Section4:
            target_position[0] = -100;
            target_position[1] = 300;
            break;
        case Section5:
            target_position[0] = -100;
            target_position[1] = 800;
            break;
        case Section7:
            target_position[0] = 200;
            target_position[1] = 500;
            break;
        case Section8:
            target_position[0] = 300;
            target_position[1] = 800;
            break;
        case Section10:
            target_position[0] = -200;
            target_position[1] = 500;
            break;
        case Section11:
            target_position[0] = -300;
            target_position[1] = 500;
            break;
        case Section3:
        case Section6:
        case Section9:
        case Section12:
            target_position[0] = 0.f;
            target_position[1] = 0.f;
            *grab_state = true;
            break;
    }
    if(local_move(target_position[0],target_position[1]) == 1){
        if(grab(!*grab_state) == !*grab_state){
            *grab_state = !*grab_state;
            section++;
            if(section > 12) section = 0;
        }
    }
    return section;
}

int local_move(float target_x, float target_y){
    float x_diff, y_diff;
    float direction, distance;
    float angle,speed;

    noInterrupts();
    direction = atan2((target_x - x_diff), (target_y - y_diff));
    distance = sqrt(pow(target_x - x_diff, 2) + pow(target_y - y_diff, 2));
    interrupts();

    /*後ろへはバック*/
    if(abs(direction) > 1.57){
        direction = 2 * PI - direction;
        angle = -1;
        speed = -1;
    }else{
        angle = 1;
        speed = 1;
    }

    /*5cm以内に近づいたらOK*/
    if(abs(distance) < 50) return 1;

    if(direction > -0.085 && direction < 0.085){
        angle *= direction * 2;
        speed *= 50;
    }else{
        /*回転角*/
        if(direction < 0){
            angle = -1;
        }else{
            angle = 1;
        }
        /*スピード*/
        if(direction > -0.25 && direction < 0.25){
            speed = 50;
        }else{
            speed = abs(direction) * 200;
        }
    }

    move_data(angle, speed);
    return 0;
}

bool grab(bool grab_state){
    static unsigned long prev_time = 0;
    if(millis() - prev_time < 1000) return !grab_state;
    if(grab_state){
        my_servo(-30);
    }else{
        my_servo(90);
    }
    prev_time = millis();
    return grab_state;
}