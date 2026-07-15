import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;

// Data variables
String angle = "";
String distance1 = "";
String distance2 = "";
String data = "";
float pixsDistance1;
float pixsDistance2;
int iAngle, iDistance1, iDistance2;
int index1 = 0;
int index2 = 0;
int index3 = 0;
PFont orcFont;

void setup() {
  size(1200, 700); // Adjust to your screen resolution
  smooth();
  
  // IMPORTANT: Change "COM7" to your actual Arduino port (e.g., "COM3", "/dev/ttyUSB0", etc.)
  myPort = new Serial(this, "/dev/cu.usbmodel2101", 9600);
  myPort.bufferUntil('.'); // Read until '.' character
}

void draw() {
  fill(98, 245, 31);
  
  // Simulate motion blur and slow fade
  noStroke();
  fill(0, 4); 
  rect(0, 0, width, height - height * 0.065); 
  
  fill(98, 245, 31); // Green color
  
  // Draw radar components
  drawRadar(); 
  drawLine();
  drawObject();
  drawText();
}

void serialEvent(Serial myPort) {
  // Read data from Serial Port up to '.'
  data = myPort.readStringUntil('.');
  data = data.substring(0, data.length() - 1);
  
  // Parse data format: angle,distance1,distance2.
  index1 = data.indexOf(","); // First comma
  if (index1 == -1) return; // Invalid data
  
  angle = data.substring(0, index1);
  
  index2 = data.indexOf(",", index1 + 1); // Second comma
  if (index2 == -1) return; // Invalid data
  
  distance1 = data.substring(index1 + 1, index2);
  distance2 = data.substring(index2 + 1, data.length());
  
  // Convert strings to integers
  try {
    iAngle = int(angle);
    iDistance1 = int(distance1);
    iDistance2 = int(distance2);
  } catch (Exception e) {
    // Handle parsing errors silently
    return;
  }
}

void drawRadar() {
  pushMatrix();
  translate(width / 2, height - height * 0.074);
  noFill();
  strokeWeight(2);
  stroke(98, 245, 31);
  
  // Draw arc lines (range circles) - extended to cover 240 degrees
  arc(0, 0, (width - width * 0.0625), (width - width * 0.0625), PI, TWO_PI + radians(60));
  arc(0, 0, (width - width * 0.27), (width - width * 0.27), PI, TWO_PI + radians(60));
  arc(0, 0, (width - width * 0.479), (width - width * 0.479), PI, TWO_PI + radians(60));
  arc(0, 0, (width - width * 0.687), (width - width * 0.687), PI, TWO_PI + radians(60));
  
  // Draw angle lines (radial lines) - extended range
  line(-width / 2, 0, width / 2, 0);
  line(0, 0, (-width / 2) * cos(radians(30)), (-width / 2) * sin(radians(30)));
  line(0, 0, (-width / 2) * cos(radians(60)), (-width / 2) * sin(radians(60)));
  line(0, 0, (-width / 2) * cos(radians(90)), (-width / 2) * sin(radians(90)));
  line(0, 0, (-width / 2) * cos(radians(120)), (-width / 2) * sin(radians(120)));
  line(0, 0, (-width / 2) * cos(radians(150)), (-width / 2) * sin(radians(150)));
  line(0, 0, (-width / 2) * cos(radians(180)), (-width / 2) * sin(radians(180)));
  line(0, 0, (-width / 2) * cos(radians(210)), (-width / 2) * sin(radians(210)));
  line(0, 0, (-width / 2) * cos(radians(240)), (-width / 2) * sin(radians(240)));
  line((-width / 2) * cos(radians(30)), 0, width / 2, 0);
  
  popMatrix();
}

void drawObject() {
  pushMatrix();
  translate(width / 2, height - height * 0.074);
  strokeWeight(9);
  
  // Convert distance from cm to pixels
  pixsDistance1 = iDistance1 * ((height - height * 0.1666) * 0.025);
  pixsDistance2 = iDistance2 * ((height - height * 0.1666) * 0.025);
  
  // Draw Sensor 1 object (Red) - limiting range to 60cm
  if (iDistance1 < 60 && iDistance1 != 999) {
    stroke(255, 10, 10); // Red color for sensor 1
    line(pixsDistance1 * cos(radians(iAngle)), 
         -pixsDistance1 * sin(radians(iAngle)), 
         (width - width * 0.505) * cos(radians(iAngle)), 
         -(width - width * 0.505) * sin(radians(iAngle)));
  }
  
  // Draw Sensor 2 object (Orange) - limiting range to 60cm
  if (iDistance2 < 60 && iDistance2 != 999) {
    stroke(255, 165, 0); // Orange color for sensor 2
    line(pixsDistance2 * cos(radians(iAngle)), 
         -pixsDistance2 * sin(radians(iAngle)), 
         (width - width * 0.505) * cos(radians(iAngle)), 
         -(width - width * 0.505) * sin(radians(iAngle)));
  }
  
  popMatrix();
}

void drawLine() {
  pushMatrix();
  strokeWeight(9);
  stroke(30, 250, 60);
  translate(width / 2, height - height * 0.074);
  
  // Draw the scanning line according to the angle
  line(0, 0, 
       (height - height * 0.12) * cos(radians(iAngle)), 
       -(height - height * 0.12) * sin(radians(iAngle)));
  
  popMatrix();
}

void drawText() {
  pushMatrix();
  
  // Determine status messages for both sensors
  String status1 = (iDistance1 > 60 || iDistance1 == 999) ? "Out of Range" : "In Range";
  String status2 = (iDistance2 > 60 || iDistance2 == 999) ? "Out of Range" : "In Range";
  
  // Draw black rectangle for text background
  fill(0, 0, 0);
  noStroke();
  rect(0, height - height * 0.0648, width, height);
  
  // Draw range markers
  fill(98, 245, 31);
  textSize(25);
  text("10cm", width - width * 0.3854, height - height * 0.0833);
  text("20cm", width - width * 0.281, height - height * 0.0833);
  text("30cm", width - width * 0.177, height - height * 0.0833);
  text("60cm", width - width * 0.0729, height - height * 0.0833);
  
  // Draw main information
  textSize(40);
  text("Dual Radar", width - width * 0.875, height - height * 0.0277);
  text("Angle: " + iAngle + "°", width - width * 0.48, height - height * 0.0277);
  
  // Display both sensor distances with color coding
  textSize(30);
  fill(255, 10, 10); // Red for sensor 1
  text("S1: ", width - width * 0.26, height - height * 0.0277);
  if (iDistance1 < 60 && iDistance1 != 999) {
    text(iDistance1 + " cm", width - width * 0.225, height - height * 0.0277);
  } else {
    text("---", width - width * 0.225, height - height * 0.0277);
  }
  
  fill(255, 165, 0); // Orange for sensor 2
  text("S2: ", width - width * 0.14, height - height * 0.0277);
  if (iDistance2 < 60 && iDistance2 != 999) {
    text(iDistance2 + " cm", width - width * 0.105, height - height * 0.0277);
  } else {
    text("---", width - width * 0.105, height - height * 0.0277);
  }
  
  // Draw angle labels
  textSize(25);
  fill(98, 245, 60);
  
  // 30 degrees
  translate((width - width * 0.4994) + width / 2 * cos(radians(30)), 
            (height - height * 0.0907) - width / 2 * sin(radians(30)));
  rotate(-radians(-60));
  text("30", 0, 0);
  resetMatrix();
  
  // 60 degrees
  translate((width - width * 0.503) + width / 2 * cos(radians(60)), 
            (height - height * 0.0888) - width / 2 * sin(radians(60)));
  rotate(-radians(-30));
  text("60", 0, 0);
  resetMatrix();
  
  // 90 degrees
  translate((width - width * 0.507) + width / 2 * cos(radians(90)), 
            (height - height * 0.0833) - width / 2 * sin(radians(90)));
  rotate(radians(0));
  text("90", 0, 0);
  resetMatrix();
  
  // 120 degrees
  translate(width - width * 0.513 + width / 2 * cos(radians(120)), 
            (height - height * 0.07129) - width / 2 * sin(radians(120)));
  rotate(radians(-30));
  text("120", 0, 0);
  resetMatrix();
  
  // 150 degrees
  translate((width - width * 0.5104) + width / 2 * cos(radians(150)), 
            (height - height * 0.0574) - width / 2 * sin(radians(150)));
  rotate(radians(-60));
  text("150", 0, 0);
  resetMatrix();
  
  // 180 degrees
  translate((width - width * 0.5) + width / 2 * cos(radians(180)), 
            (height - height * 0.05) - width / 2 * sin(radians(180)));
  rotate(radians(-90));
  text("180", 0, 0);
  resetMatrix();
  
  // 210 degrees
  translate((width - width * 0.49) + width / 2 * cos(radians(210)), 
            (height - height * 0.0574) - width / 2 * sin(radians(210)));
  rotate(radians(-120));
  text("210", 0, 0);
  resetMatrix();
  
  // 240 degrees
  translate((width - width * 0.48) + width / 2 * cos(radians(240)), 
            (height - height * 0.07129) - width / 2 * sin(radians(240)));
  rotate(radians(-150));
  text("240", 0, 0);
  resetMatrix();
  
  popMatrix();
}
