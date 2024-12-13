import processing.net.*;

// クライアント
Client client;
String ip = "192.48.56.1";
int port = 80;

// UI要素
PVector center;
float radius;
PVector pointer;
boolean grasp = false;

// 表示データ
float currentVelocity = 0;
float currentDirection = 0;
PVector currentPosition = new PVector(0, 0);

void setup() {
  size(600, 400); // ウィンドウを拡張
  center = new PVector(width / 4, height / 2);
  radius = width / 6;
  pointer = new PVector(center.x, center.y);

  client = new Client(this, ip, port);
}

void draw() {
  background(240);

  // 左側: コントロールUI
  drawControlUI();

  // 右側: データ表示エリア
  drawDataDisplay();

  // データを受信
  receiveData();
}

void drawControlUI() {
  // ジョイスティック領域
  stroke(0);
  noFill();
  ellipse(center.x, center.y, radius * 2, radius * 2);

  // ポインタ
  fill(100, 150, 255);
  noStroke();
  ellipse(pointer.x, pointer.y, 20, 20);

  // グリップボタン
  fill(grasp ? color(0, 200, 0) : color(200, 0, 0));
  rect(width / 4 - 50, height - 50, 80, 30, 5);
  fill(255);
  textAlign(CENTER, CENTER);
  text("GRASP", width / 4 - 10, height - 35);

  // 移動コマンドの送信
  if (client.active() && mousePressed) {
    float dx = pointer.x - center.x;
    float dy = pointer.y - center.y;
    float distance = dist(pointer.x, pointer.y, center.x, center.y);
    float velocity = map(constrain(distance, 0, radius), 0, radius, 0, 100);
    float direction = atan2(dy, dx) / PI; // [-1, 1] に正規化

    String command = String.format("MOVE %.2f %.2f\n", velocity, direction);
    client.write(command);
  }
}

void drawDataDisplay() {
  // 速度、角度、座標を表示
  fill(0);
  textAlign(LEFT, CENTER);
  textSize(16);

  text("Velocity: " + nf(currentVelocity, 0, 2) + " cm/s", width / 2 + 20, 50);
  text("Direction: " + nf(currentDirection, 0, 2) + "°", width / 2 + 20, 100);
  text("Position: (" + nf(currentPosition.x, 0, 2) + ", " + nf(currentPosition.y, 0, 2) + ")", width / 2 + 20, 150);

  // グラフ表示
  stroke(0);
  noFill();
  rect(width / 2 + 20, 200, 150, 100);
  float graphX = width / 2 + 30;
  float graphY = 250;
  float barWidth = 30;

  fill(100, 200, 255);
  rect(graphX, graphY - map(currentVelocity, 0, 100, 0, 100), barWidth, map(currentVelocity, 0, 100, 0, 100));
  fill(200, 100, 100);
  rect(graphX + barWidth + 10, graphY - map(abs(currentDirection), 0, 180, 0, 100), barWidth, map(abs(currentDirection), 0, 180, 0, 100));
}

void receiveData() {
  if (client.active() && client.available() > 0) {
    String data = client.readStringUntil('\n');
    if (data != null) {
      parseData(data.trim());
    }
  }
}

void parseData(String data) {
  if (data.startsWith("VELOCITY")) {
    currentVelocity = float(data.substring(9).trim());
  } else if (data.startsWith("DIRECTION")) {
    currentDirection = float(data.substring(10).trim());
  } else if (data.startsWith("POSITION")) {
    String[] parts = split(data.substring(9).trim(), ' ');
    if (parts.length == 2) {
      currentPosition.set(float(parts[0]), float(parts[1]));
    }
  }
}

void mouseDragged() {
  if (dist(mouseX, mouseY, center.x, center.y) <= radius) {
    pointer.set(mouseX, mouseY);
  } else {
    pointer.set(PVector.sub(new PVector(mouseX, mouseY), center).normalize().mult(radius).add(center));
  }
}

void mouseReleased() {
  if (client.active()) {
    client.write("STOP\n");
  }
  pointer.set(center.x, center.y);
}

void mousePressed() {
  if (mouseX >= width / 4 - 50 && mouseX <= width / 4 + 30 && mouseY >= height - 50 && mouseY <= height - 20) {
    grasp = !grasp;
    String command = grasp ? "GRASP_ON\n" : "GRASP_OFF\n";
    if (client.active()) {
      client.write(command);
    }
  }
}
