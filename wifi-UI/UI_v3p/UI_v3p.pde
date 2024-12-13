import processing.net.*;

Client client;
String ip = "192.48.56.1"; // ArduinoのIPアドレス
int port = 80; // Arduinoのポート番号

// UI要素
float centerX, centerY; // 円形の中心位置
float pointerX, pointerY; // ポインタの位置
float radius; // ジョイスティックの半径
boolean isGrasping = false; // 把持機構の状態
boolean isMoving = false; // ロボットが移動中かどうか

void setup() {
  size(400, 400);
  client = new Client(this, ip, port); // クライアントを初期化
  centerX = width / 2;
  centerY = height / 2;
  radius = width / 4;
  pointerX = centerX;
  pointerY = centerY;
}

void draw() {
  background(255);

  // ジョイスティックの円を描画
  stroke(0);
  noFill();
  ellipse(centerX, centerY, radius * 2, radius * 2);

  // 中心円を描画
  fill(200);
  ellipse(centerX, centerY, 50, 50);

  // マウスが押されているとき、ポインタをマウス位置に追従させる
  if (isMoving) {
    float dx = mouseX - centerX;
    float dy = mouseY - centerY;
    float distance = dist(centerX, centerY, mouseX, mouseY);

    if (distance <= radius) {
      pointerX = mouseX;
      pointerY = mouseY;
    } else {
      PVector direction = new PVector(dx, dy).normalize().mult(radius);
      pointerX = centerX + direction.x;
      pointerY = centerY + direction.y;
    }
    sendDirection();
  } else {
    pointerX = centerX;
    pointerY = centerY;
  }

  // ポインタを描画
  fill(150, 0, 0);
  ellipse(pointerX, pointerY, 20, 20);

  // グリップボタンの表示
  fill(isGrasping ? color(0, 150, 0) : color(150, 0, 0));
  rect(width - 80, height - 50, 60, 30);
  fill(255);
  textAlign(CENTER, CENTER);
  text("Grasp", width - 50, height - 35);
}

// クリック時に移動開始、ボタンの切り替え
void mousePressed() {
  // グリップボタンをクリックした場合
  if (mouseX > width - 80 && mouseX < width - 20 && mouseY > height - 50 && mouseY < height - 20) {
    isGrasping = !isGrasping; // 把持状態をトグル
    sendGraspCommand();
  } else {
    isMoving = true; // 移動開始
  }
}

void mouseReleased() {
  isMoving = false; // 移動停止
  sendStopCommand(); // Arduinoに停止信号を送信
}

// ポインタの方向と速度に基づきArduinoにデータ送信
void sendDirection() {
  if (client.active()) {
    float dx = pointerX - centerX;
    float dy = pointerY - centerY;
    float distance = dist(centerX, centerY, pointerX, pointerY);
    float speed = map(constrain(distance, 0, radius), 0, radius, 0, 100); // 速度を計算
    float angle = atan2(dy, dx) * 180 / PI; // 中心からの角度を取得

    // データ形式例: "MOVE angle speed\n"
    String command = "MOVE " + nf(angle, 0, 2) + " " + nf(speed, 0, 2) + "\n";
    client.write(command);
  }
}

// グリップ状態をArduinoに送信
void sendGraspCommand() {
  if (client.active()) {
    String command = isGrasping ? "GRASP_ON\n" : "GRASP_OFF\n";
    client.write(command);
  }
}

// 停止コマンドをArduinoに送信
void sendStopCommand() {
  if (client.active()) {
    client.write("STOP\n");
  }
}
