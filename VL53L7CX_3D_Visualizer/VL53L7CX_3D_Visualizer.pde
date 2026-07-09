/*
 * VL53L7CX 3D Distance Mapper - Processing Visualization
 * 
 * 3D surface plot visualization of distance data
 * 
 * Controls:
 * - Mouse drag to rotate view
 * - Mouse wheel to zoom
 * - Press 'r' to reset view
 * - Press 's' to save screenshot
 * - Press 'w' to toggle wireframe
 * - Press 'c' to change color scheme
 */

import processing.serial.*;

Serial myPort;
String serialData = "";
boolean readingJSON = false;

// Configuration
String SERIAL_PORT = "COM6";  // CHANGE THIS!
int BAUD_RATE = 115200;
int GRID_SIZE = 8;

// Data storage
float[][] heightMap = new float[GRID_SIZE][GRID_SIZE];
int[][] statusGrid = new int[GRID_SIZE][GRID_SIZE];

// 3D view settings
float rotationX = -0.5;
float rotationY = 0.5;
float zoom = 300;
float scaleZ = 0.5;  // Height scale factor
boolean wireframe = false;
int colorScheme = 0;  // 0=Rainbow, 1=Thermal, 2=Grayscale

// Camera control
float prevMouseX, prevMouseY;

// Display settings
int maxHeight = 4000;
int minHeight = 0;

PFont font;

void setup() {
  size(800, 600, P3D);
  
  surface.setLocation(0, 0);  // Exact top-left corner
  
  // List available serial ports
  println("Available serial ports:");
  printArray(Serial.list());
  println("\nTrying to connect to: " + SERIAL_PORT);
  
  // Connect to serial port
  try {
    myPort = new Serial(this, SERIAL_PORT, BAUD_RATE);
    myPort.bufferUntil('\n');
    println("Connected successfully!");
  } catch (Exception e) {
    println("ERROR: Could not connect to " + SERIAL_PORT);
    println("Please check the port name and try again.");
    exit();
  }
  
  font = createFont("Arial", 14);
  textFont(font);
  
  // Initialize grid
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      heightMap[i][j] = 0;
      statusGrid[i][j] = 0;
    }
  }
  
  frameRate(30);
}

void draw() {
  background(20);
  
  // Setup 3D view
  lights();
  
  // Position camera
  translate(width/2, height/2, 0);
  scale(zoom / 100.0);
  rotateX(rotationX);
  rotateY(rotationY);
  
  // Draw 3D surface
  draw3DSurface();
  
  // Draw grid base
  drawGridBase();
  
  // Draw axes
  drawAxes();
  
  // Reset to 2D for UI
  camera();
  hint(DISABLE_DEPTH_TEST);
  drawUI();
  hint(ENABLE_DEPTH_TEST);
}

void draw3DSurface() {
  pushMatrix();
  
  // Center the grid
  translate(-GRID_SIZE * 25 / 2, -GRID_SIZE * 25 / 2, 0);
  
  int cellSize = 25;
  
  if (wireframe) {
    noFill();
    stroke(255);
    strokeWeight(1);
  } else {
    strokeWeight(0.5);
    stroke(50);
  }
  
  // Draw surface
  for (int y = 0; y < GRID_SIZE - 1; y++) {
    beginShape(TRIANGLE_STRIP);
    for (int x = 0; x < GRID_SIZE; x++) {
      // First vertex
      float h1 = -heightMap[y][x] * scaleZ;
      if (!wireframe) {
        fill(getColorForHeight(heightMap[y][x]));
      }
      vertex(x * cellSize, y * cellSize, h1);
      
      // Second vertex
      float h2 = -heightMap[y + 1][x] * scaleZ;
      if (!wireframe) {
        fill(getColorForHeight(heightMap[y + 1][x]));
      }
      vertex(x * cellSize, (y + 1) * cellSize, h2);
    }
    endShape();
  }
  
  popMatrix();
}

void drawGridBase() {
  pushMatrix();
  translate(-GRID_SIZE * 25 / 2, -GRID_SIZE * 25 / 2, 0);
  
  stroke(100);
  strokeWeight(1);
  noFill();
  
  // Draw grid lines
  for (int i = 0; i <= GRID_SIZE; i++) {
    line(i * 25, 0, 0, i * 25, GRID_SIZE * 25, 0);
    line(0, i * 25, 0, GRID_SIZE * 25, i * 25, 0);
  }
  
  popMatrix();
}

void drawAxes() {
  strokeWeight(2);
  
  // X axis - Red
  stroke(255, 0, 0);
  line(0, 0, 0, 50, 0, 0);
  
  // Y axis - Green
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 50, 0);
  
  // Z axis - Blue
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, 50);
}

void drawUI() {
  fill(255);
  textAlign(LEFT);
  
  // Title
  textSize(20);
  text("VL53L7CX 3D Distance Mapper", 20, 30);
  
  // Instructions
  textSize(12);
  int y = 60;
  text("Controls:", 20, y);
  text("• Drag mouse to rotate", 20, y + 20);
  text("• Mouse wheel to zoom", 20, y + 40);
  text("• Press 'w' for wireframe", 20, y + 60);
  text("• Press 'c' for color scheme", 20, y + 80);
  text("• Press '+/-' for height scale", 20, y + 100);
  text("• Press 'r' to reset view", 20, y + 120);
  text("• Press 's' to screenshot", 20, y + 140);
  
  // Status
  y = height - 80;
  text("Zoom: " + nf(zoom, 0, 1), 20, y);
  text("Height Scale: " + nf(scaleZ, 0, 2), 20, y + 20);
  text("Wireframe: " + wireframe, 20, y + 40);
  
  String[] schemes = {"Rainbow", "Thermal", "Grayscale"};
  text("Color: " + schemes[colorScheme], 20, y + 60);
  
  // Distance range
  textAlign(RIGHT);
  text("Range: " + minHeight + " - " + maxHeight + " mm", width - 20, y);
}

color getColorForHeight(float height) {
  float normalized = map(height, minHeight, maxHeight, 0, 1);
  normalized = constrain(normalized, 0, 1);
  
  color c;
  
  switch(colorScheme) {
    case 0: // Rainbow
      if (normalized < 0.25) {
        c = lerpColor(color(0, 0, 255), color(0, 255, 255), normalized * 4);
      } else if (normalized < 0.5) {
        c = lerpColor(color(0, 255, 255), color(0, 255, 0), (normalized - 0.25) * 4);
      } else if (normalized < 0.75) {
        c = lerpColor(color(0, 255, 0), color(255, 255, 0), (normalized - 0.5) * 4);
      } else {
        c = lerpColor(color(255, 255, 0), color(255, 0, 0), (normalized - 0.75) * 4);
      }
      break;
      
    case 1: // Thermal
      if (normalized < 0.33) {
        c = lerpColor(color(0, 0, 128), color(255, 0, 0), normalized * 3);
      } else if (normalized < 0.66) {
        c = lerpColor(color(255, 0, 0), color(255, 255, 0), (normalized - 0.33) * 3);
      } else {
        c = lerpColor(color(255, 255, 0), color(255, 255, 255), (normalized - 0.66) * 3);
      }
      break;
      
    case 2: // Grayscale
      int gray = (int)(normalized * 255);
      c = color(gray);
      break;
      
    default:
      c = color(255);
  }
  
  return c;
}

void serialEvent(Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  
  if (inString != null) {
    inString = trim(inString);
    
    if (inString.equals("{")) {
      serialData = inString + "\n";
      readingJSON = true;
    } else if (readingJSON) {
      serialData += inString + "\n";
      
      if (inString.equals("}")) {
        readingJSON = false;
        parseJSON(serialData);
        serialData = "";
      }
    }
  }
}

void parseJSON(String jsonString) {
  try {
    JSONObject json = parseJSONObject(jsonString);
    
    if (json != null && json.hasKey("grid")) {
      JSONArray grid = json.getJSONArray("grid");
      
      minHeight = 4000;
      maxHeight = 0;
      
      for (int y = 0; y < GRID_SIZE && y < grid.size(); y++) {
        JSONArray row = grid.getJSONArray(y);
        for (int x = 0; x < GRID_SIZE && x < row.size(); x++) {
          if (row.isNull(x)) {
            heightMap[y][x] = 0;
            statusGrid[y][x] = 0;
          } else {
            float height = row.getFloat(x);
            heightMap[y][x] = height;
            statusGrid[y][x] = 5;
            
            if (height > 0) {
              minHeight = min((int)minHeight, (int)height);
              maxHeight = max((int)maxHeight, (int)height);
            }
          }
        }
      }
    }
  } catch (Exception e) {
    println("Error parsing JSON: " + e.getMessage());
  }
}

void mouseDragged() {
  rotationY += (mouseX - prevMouseX) * 0.01;
  rotationX += (mouseY - prevMouseY) * 0.01;
  
  // Limit X rotation
  rotationX = constrain(rotationX, -PI/2, PI/2);
}

void mousePressed() {
  prevMouseX = mouseX;
  prevMouseY = mouseY;
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  zoom -= e * 20;
  zoom = constrain(zoom, 50, 800);
}

void keyPressed() {
  // Reset view
  if (key == 'r' || key == 'R') {
    rotationX = -0.5;
    rotationY = 0.5;
    zoom = 300;
    println("View reset");
  }
  
  // Toggle wireframe
  if (key == 'w' || key == 'W') {
    wireframe = !wireframe;
    println("Wireframe: " + wireframe);
  }
  
  // Change color scheme
  if (key == 'c' || key == 'C') {
    colorScheme = (colorScheme + 1) % 3;
    println("Color scheme: " + colorScheme);
  }
  
  // Adjust height scale
  if (key == '+' || key == '=') {
    scaleZ += 0.1;
    println("Height scale: " + scaleZ);
  }
  if (key == '-' || key == '_') {
    scaleZ = max(0.1, scaleZ - 0.1);
    println("Height scale: " + scaleZ);
  }
  
  // Save screenshot
  if (key == 's' || key == 'S') {
    String filename = "vl53l7cx_3d_" + year() + month() + day() + "_" + hour() + minute() + second() + ".png";
    saveFrame(filename);
    println("Screenshot saved: " + filename);
  }
}
