import processing.serial.*;

Serial myPort;

float centerX, centerY, radius;
float pointerX, pointerY;
boolean isMoving = false;
boolean isGrasping = false;
int thermistorValue = 0; // サーミスタの値を保存
boolean isAutonomous = false;

void setup() {
  size(400, 400);

  centerX = width / 2;
  centerY = height / 2;
  radius = 100;
  pointerX = centerX;
  pointerY = centerY;
  
  String portName = "/dev/tty.usbmodemF0F5BD528D302";
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n'); // 改行まで受信を待つ
}

void draw() {
  background(255);

  fill(isAutonomous ? color(0, 150, 0) : color(150, 0, 0));
  rect(20, 20, 100, 50);
  fill(255);
  textAlign(CENTER, CENTER);
  text("Autonomous", 70, 45);

  // 円と方向UI
  stroke(0);
  noFill();
  ellipse(centerX, centerY, radius * 2, radius * 2);

  fill(200);
  ellipse(centerX, centerY, 50, 50);

  fill(150, 0, 0);
  ellipse(pointerX, pointerY, 20, 20);

  // つかむボタン
  fill(isGrasping ? color(0, 150, 0) : color(150, 0, 0));
  rect(width - 80, height - 50, 60, 30);
  fill(255);
  textAlign(CENTER, CENTER);
  text("Grasp", width - 50, height - 35);

  // サーミスタ値の表示
  fill(0);
  textSize(16);
  text("Thermistor: " + thermistorValue, 20, height - 20);
}

void mouseDragged() {
  float distance = dist(mouseX, mouseY, centerX, centerY);

  if (distance <= radius) {
    pointerX = mouseX;
    pointerY = mouseY;
  } else {
    PVector direction = new PVector(mouseX - centerX, mouseY - centerY).normalize().mult(radius);
    pointerX = centerX + direction.x;
    pointerY = centerY + direction.y;
  }

  isMoving = true;
  sendDirection();
}

void mouseReleased() {
  float distance = dist(pointerX, pointerY, centerX, centerY);
  if (distance < 25) { // 中心の黒い円にポインタが入った場合のみSTOPを送信
    sendStop();
    isMoving = false;
  }
}

void mousePressed() {
  if (mouseX > width - 80 && mouseX < width - 20 && mouseY > height - 50 && mouseY < height - 20) {
    isGrasping = !isGrasping;
    sendGraspToggle();
  }
  if (mouseX >= 20 && mouseX <= 120 && mouseY >= 20 && mouseY <= 70) {
    println("Autonomous button clicked"); // デバッグ用ログ
    isAutonomous = !isAutonomous;
    sendCommand(isAutonomous ? "AUTONOMOUS ON" : "AUTONOMOUS OFF");
  }
}

void serialEvent(Serial p) {
  String inData = p.readStringUntil('\n'); // 改行までのデータを読み取る
  inData = inData.trim(); // 不要な空白を削除

  if (inData.startsWith("THERMISTOR")) {
    String[] parts = split(inData, ' ');
    if (parts.length > 1) {
      thermistorValue = int(parts[1]); // サーミスタの値を取得
    }
  }
}

void sendDirection() {
  float dx = pointerX - centerX;
  float dy = pointerY - centerY;

  if (dy < -radius / 2) {
    sendCommand("MOVE FORWARD");
  } else if (dy > radius / 2) {
    sendCommand("MOVE BACKWARD");
  } else if (dx > radius / 2) {
    sendCommand("MOVE RIGHT");
  } else if (dx < -radius / 2) {
    sendCommand("MOVE LEFT");
  }
}

void sendStop() {
  sendCommand("STOP");
}

void sendGraspToggle() {
  sendCommand(isGrasping ? "GRASP ON" : "GRASP OFF");
}

void sendCommand(String command) {
  println("Sending: " + command);
  myPort.write(command + "\n");
}
