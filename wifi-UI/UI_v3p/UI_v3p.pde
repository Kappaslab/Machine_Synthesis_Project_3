import processing.net.*;

Client client;
String IP = "192.48.56.1";
int PORT = 80;

PVector center;
float radius;
PVector pointer;
boolean grasp = false;
boolean connected = false;

int t_prev; //タイマ用

void setup()
{ 
  size(500,500);
  background(0);
  textSize(40);
  textAlign(LEFT,TOP);
  
  println("connecting to "+IP+":"+PORT+"...");  
  client = new Client(this, IP, PORT); //サーバーIP:PORTに接続するクライアントを作成
  println("Client Started.");

  t_prev = millis();
}

void draw() {
  background(240);

  // Draw joystick boundary
  stroke(0);
  noFill();
  ellipse(center.x, center.y, radius * 2, radius * 2);

  // Draw pointer
  fill(100, 150, 255);
  noStroke();
  ellipse(pointer.x, pointer.y, 20, 20);

  // Draw grasp button
  fill(grasp ? color(0, 200, 0) : color(200, 0, 0));
  rect(width - 100, height - 50, 80, 30, 5);
  fill(255);
  textAlign(CENTER, CENTER);
  text("GRASP", width - 60, height - 35);

  // Display connection status
  fill(0);
  textAlign(LEFT, TOP);
  text(connected ? "Connected" : "Disconnected", 10, 10);

  // Send commands to Arduino
  if (connected && mousePressed) {
    float dx = pointer.x - center.x;
    float dy = pointer.y - center.y;
    float distance = dist(pointer.x, pointer.y, center.x, center.y);
    float velocity = map(constrain(distance, 0, radius), 0, radius, 0, 100);
    float direction = atan2(dy, dx) / PI; // Normalize direction to [-1, 1]

    String command = String.format("MOVE %.2f %.2f\n", velocity, direction);
    client.write(command);
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
  if (connected) {
    client.write("STOP\n");
  }
  pointer.set(center.x, center.y);
}

void mousePressed() {
  if (mouseX >= width - 100 && mouseX <= width - 20 && mouseY >= height - 50 && mouseY <= height - 20) {
    grasp = !grasp;
    String command = grasp ? "GRASP_ON\n" : "GRASP_OFF\n";
    if (connected) {
      client.write(command);
    }
  }
}
