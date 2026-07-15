import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;

// Data variables
String angle = "";
String distance1 = "";
String distance2 = "";
String tableEdge = "";
String data = "";
float pixsDistance1;
float pixsDistance2;
float pixsTableEdge;
int iAngle, iDistance1, iDistance2, iTableEdge;
int index1 = 0;
int index2 = 0;
int index3 = 0;
int index4 = 0;
PFont orcFont;

// Table geometry
final float TABLE_LONG_SIDE = 118.0;
final float TABLE_SHORT_SIDE = 60.0;

void setup() {
  size(1400, 800);
  smooth();
  
  // IMPORTANT: Change to your Arduino port
  myPort = new Serial(this, "/dev/", 9600);
  myPort.bufferUntil('.');
}

void draw() {
  // Dark background
  fill(0, 4); 
  noStroke();
  rect(0, 0, width, height - height * 0.065); 
  
  // Draw all components
  drawTable();
  drawRadar(); 
  drawTableEdgeLine();
  drawLine();
  drawObject();
  drawText();
}

void serialEvent(Serial myPort) {
  data = myPort.readStringUntil('.');
  if (data == null) return;
  data = data.substring(0, data.length() - 1);
  
  // Parse: angle,distance1,distance2,tableEdge.
  index1 = data.indexOf(",");
  if (index1 == -1) return;
  
  angle = data.substring(0, index1);
  
  index2 = data.indexOf(",", index1 + 1);
  if (index2 == -1) return;
  
  distance1 = data.substring(index1 + 1, index2);
  
  index3 = data.indexOf(",", index2 + 1);
  if (index3 == -1) return;
  
  distance2 = data.substring(index2 + 1, index3);
  tableEdge = data.substring(index3 + 1, data.length());
  
  try {
    iAngle = int(angle);
    iDistance1 = int(distance1);
    iDistance2 = int(distance2);
    iTableEdge = int(tableEdge);
  } catch (Exception e) {
    return;
  }
}

// Helper to convert servo angle to Processing angle
float servoToProcessingAngle(int servoAngle) {
  return radians(180 - servoAngle);
}

// Helper to get table edge distance at given angle
float getTableEdgeThreshold(int angle) {
  float threshold;
  
  if (angle >= 30 && angle <= 90) {
    float rad = radians(angle);
    threshold = TABLE_LONG_SIDE * sin(rad);
  }
  else if (angle > 90 && angle <= 180) {
    threshold = TABLE_SHORT_SIDE;
  }
  else if (angle > 180 && angle <= 240) {
    int mirrorAngle = 180 - (angle - 180);
    float rad = radians(mirrorAngle);
    threshold = TABLE_LONG_SIDE * sin(rad);
  }
  else {
    threshold = TABLE_SHORT_SIDE;
  }
  
  return threshold;
}

void drawTable() {
  pushMatrix();
  translate(width / 2, height - height * 0.15);
  
  // Draw the table outline (corner mount)
  stroke(139, 69, 19);  // Brown color
  strokeWeight(4);
  fill(101, 67, 33, 100);  // Semi-transparent brown
  
  float scale = 3.5;  // Scale factor for visualization
  
  // Table extends right (118cm) and up (60cm) from sensor corner
  // Convert to screen coordinates
  beginShape();
  vertex(0, 0);  // Sensor position (corner)
  
  // Trace table edge using the detection formula
  for (int angle = 30; angle <= 240; angle += 2) {
    float threshold = getTableEdgeThreshold(angle);
    float procAngle = servoToProcessingAngle(angle);
    float x = threshold * scale * cos(procAngle);
    float y = threshold * scale * sin(procAngle);
    vertex(x, y);
  }
  
  vertex(0, 0);  // Back to corner
  endShape();
  
  // Draw corner indicator
  fill(139, 69, 19);
  noStroke();
  ellipse(0, 0, 12, 12);
  
  // Label
  fill(139, 69, 19);
  textSize(16);
  textAlign(CENTER);
  text("TABLE", 150, -20);
  text("CORNER", 150, 0);
  textAlign(LEFT);
  
  popMatrix();
}

void drawRadar() {
  pushMatrix();
  translate(width / 2, height - height * 0.15);
  noFill();
  strokeWeight(1);
  stroke(98, 245, 31, 100);  // Dimmer grid
  
  float startAngle = servoToProcessingAngle(240);
  float endAngle = servoToProcessingAngle(30);
  
  float maxRadius = height * 0.75;
  
  // Range circles (every 30cm)
  for (int r = 30; r <= 120; r += 30) {
    float radius = r * 3.5;
    arc(0, 0, radius * 2, radius * 2, startAngle, endAngle);
  }
  
  // Radial lines every 30 degrees
  strokeWeight(1);
  for (int angle = 30; angle <= 240; angle += 30) {
    float procAngle = servoToProcessingAngle(angle);
    line(0, 0, maxRadius * 0.45 * cos(procAngle), maxRadius * 0.45 * sin(procAngle));
  }
  
  popMatrix();
}

void drawTableEdgeLine() {
  pushMatrix();
  translate(width / 2, height - height * 0.15);
  
  // Draw the dynamic table edge threshold line
  stroke(255, 255, 0);  // Yellow for table edge
  strokeWeight(3);
  
  float procAngle = servoToProcessingAngle(iAngle);
  float scale = 3.5;
  float edgeDist = iTableEdge * scale;
  
  line(0, 0, edgeDist * cos(procAngle), edgeDist * sin(procAngle));
  
  popMatrix();
}

void drawObject() {
  pushMatrix();
  translate(width / 2, height - height * 0.15);
  strokeWeight(9);
  
  float procAngle = servoToProcessingAngle(iAngle);
  float scale = 3.5;
  
  pixsDistance1 = iDistance1 * scale;
  pixsDistance2 = iDistance2 * scale;
  pixsTableEdge = iTableEdge * scale;
  
  // Draw Sensor 1 (Red) - only if within table boundary
  if (iDistance1 < iTableEdge && iDistance1 != 999) {
    stroke(255, 10, 10);
    line(pixsDistance1 * cos(procAngle), 
         pixsDistance1 * sin(procAngle), 
         pixsTableEdge * cos(procAngle), 
         pixsTableEdge * sin(procAngle));
  }
  
  // Draw Sensor 2 (Orange) - only if within table boundary
  if (iDistance2 < iTableEdge && iDistance2 != 999) {
    stroke(255, 165, 0);
    line(pixsDistance2 * cos(procAngle), 
         pixsDistance2 * sin(procAngle), 
         pixsTableEdge * cos(procAngle), 
         pixsTableEdge * sin(procAngle));
  }
  
  popMatrix();
}

void drawLine() {
  pushMatrix();
  strokeWeight(7);
  stroke(30, 250, 60, 200);
  translate(width / 2, height - height * 0.15);
  
  float procAngle = servoToProcessingAngle(iAngle);
  float maxRadius = height * 0.75;
  
  line(0, 0, 
       (maxRadius * 0.4) * cos(procAngle), 
       (maxRadius * 0.4) * sin(procAngle));
  
  popMatrix();
}

void drawText() {
  pushMatrix();
  
  // Black background bar
  fill(0, 0, 0);
  noStroke();
  rect(0, height - height * 0.08, width, height);
  
  // Range markers
  fill(98, 245, 31);
  textSize(18);
  text("30cm", width - 420, height - height * 0.04);
  text("60cm", width - 350, height - height * 0.04);
  text("90cm", width - 280, height - height * 0.04);
  text("120cm", width - 210, height - height * 0.04);
  
  // Title
  textSize(35);
  fill(139, 69, 19);  // Brown for table
  text("TABLE-EDGE", 30, height - height * 0.045);
  fill(98, 245, 31);
  text("RADAR", 240, height - height * 0.045);
  
  // Angle display
  textSize(32);
  text("Angle: " + iAngle + "°", width / 2 - 100, height - height * 0.045);
  
  // Table edge distance
  fill(255, 255, 0);  // Yellow
  textSize(26);
  text("Edge: " + iTableEdge + "cm", width / 2 + 120, height - height * 0.045);
  
  // Sensor distances
  textSize(28);
  fill(255, 10, 10);
  text("S1:", width - 180, height - height * 0.045);
  if (iDistance1 < iTableEdge && iDistance1 != 999) {
    text(iDistance1 + "cm", width - 140, height - height * 0.045);
  } else {
    text("--", width - 140, height - height * 0.045);
  }
  
  fill(255, 165, 0);
  text("S2:", width - 70, height - height * 0.045);
  if (iDistance2 < iTableEdge && iDistance2 != 999) {
    text(iDistance2 + "cm", width - 30, height - height * 0.045);
  } else {
    text("--", width - 30, height - height * 0.045);
  }
  
  // Draw angle labels
  translate(width / 2, height - height * 0.15);
  textSize(20);
  fill(98, 245, 60);
  
  float labelRadius = height * 0.75 * 0.48;
  
  for (int angle = 30; angle <= 240; angle += 30) {
    float procAngle = servoToProcessingAngle(angle);
    pushMatrix();
    translate(labelRadius * cos(procAngle), labelRadius * sin(procAngle));
    
    if (angle < 180) {
      rotate(procAngle - HALF_PI);
    } else {
      rotate(procAngle + HALF_PI);
    }
    
    textAlign(CENTER);
    text(angle + "°", 0, 0);
    textAlign(LEFT);
    popMatrix();
  }
  
  popMatrix();
}
