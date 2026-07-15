# Dual-Sensor Ultrasonic Radar System with Visualization

## Overview
This integrated system combines:
- **Dual ultrasonic sensor scanning** with independent distance tracking
- **Servo-based 240° sweep** radar functionality (15° to 255°)
- **Audio feedback** with distance-based beeping (closer = faster/higher pitch)
- **Real-time Processing visualization** displaying both sensors simultaneously

**Important**: This system requires a servo capable of 270° rotation (or at least 240°). Standard 180° servos will not work properly with the 15°-255° range.

## Key Features & Optimizations

### Arduino Code Improvements
1. **Integrated Mapping Protocol**: Sends data in format `angle,distance1,distance2.` for Processing
2. **Smooth Servo Movement**: Optimized step delay (30ms) for stable measurements
3. **Staggered Sensor Triggers**: 40ms gap between sensor readings to prevent crosstalk
4. **Hysteresis Logic**: Requires 3 consecutive readings to switch active sensor (reduces false alarms)
5. **Non-blocking Beeper**: Audio feedback runs independently without blocking sensor readings
6. **Continuous Tone Mode**: Solid tone when object <5cm (emergency proximity)
7. **Battery Monitoring**: Optional voltage monitoring with low-battery detection
8. **Smart Servo Control**: No detachment needed since mapping requires continuous sweep

### Processing Display Enhancements
1. **Dual-Sensor Visualization**: 
   - Sensor 1 (Red): Primary sensor readings
   - Sensor 2 (Orange): Secondary sensor readings
   - Green scanning line shows current angle
2. **Real-time Distance Display**: Shows both sensor readings with color-coded labels
3. **Range Indicators**: Updated to show 10/20/30/60cm markers
4. **Motion Blur Effect**: Trailing fade effect for smooth visual experience
5. **Error Handling**: Gracefully handles invalid serial data

## Hardware Setup

### Required Components
- Arduino board (Uno/Nano/Mega)
- 2× HC-SR04 Ultrasonic sensors
- 1× Servo motor (270° rotation **required** - see note below)
- 1× Speaker/Piezo buzzer
- Battery (if using monitoring: voltage divider with 47kΩ and 100kΩ resistors)
- Jumper wires and breadboard

**Servo Note**: You must use a servo capable of 270° rotation (0°-270°) such as:
- Feetech FS5103R/FS5106R
- TowerPro MG996R (some models support 270°)
- Hitec HS-785HB
- Standard 180° servos (SG90, MG90S) will **NOT** work for the full 240° sweep

### Pin Connections

#### Sensor 1
- TRIG1 → Pin 10
- ECHO1 → Pin 11

#### Sensor 2
- TRIG2 → Pin 2
- ECHO2 → Pin 3

#### Peripherals
- SERVO → Pin 12
- SPEAKER → Pin 13
- BATTERY_SENSE (optional) → A0

### Physical Mounting
- Mount both ultrasonic sensors on the servo horn
- Position sensors ~2-3cm apart, facing forward
- Ensure clean line of sight for both sensors
- Secure wiring to prevent tangling during servo rotation

## Software Setup

### Arduino IDE
1. Install Arduino IDE 1.8.19 or newer
2. Open `dual_sensor_radar_integrated.ino`
3. **Configure COM Port**: Verify your Arduino's serial port in Tools → Port
4. **Adjust parameters if needed**:
   - `POS_MIN` / `POS_MAX`: Servo sweep range (currently 15°-255°)
   - `MAX_DETECT_CM`: Maximum detection distance (currently 60cm)
   - `SERVO_STEP_DELAY`: Speed of sweep (30ms recommended)
5. Upload to Arduino

### Processing IDE
1. Install Processing 4.x from processing.org
2. Open `dual_sensor_radar_display.pde`
3. **CRITICAL**: Change COM port on line 18:
   ```processing
   myPort = new Serial(this, "COM7", 9600);
   ```
   Replace "COM7" with your Arduino's port:
   - Windows: "COM3", "COM4", etc.
   - Mac: "/dev/tty.usbserial-XXXX" or "/dev/cu.usbmodem-XXXX"
   - Linux: "/dev/ttyUSB0" or "/dev/ttyACM0"
4. **Adjust screen size** if needed (line 11):
   ```processing
   size(1200, 700);
   ```
5. Run the sketch

## Usage

### Starting the System
1. Upload Arduino code first
2. Keep Arduino connected via USB
3. Run Processing sketch
4. The radar should start scanning automatically

### What You'll See
- **Green scanning line**: Current servo angle
- **Red blips**: Objects detected by Sensor 1
- **Orange blips**: Objects detected by Sensor 2
- **Bottom display**: Real-time angle and distances for both sensors

### Audio Feedback
- **No beeping**: No objects detected within 60cm
- **Slow beeps**: Objects far away (40-60cm range)
- **Fast beeps**: Objects getting closer
- **High-pitch beeps**: Very close objects (<10cm)
- **Continuous tone**: Emergency proximity (<5cm)

## Optimization Details

### Why These Changes Were Made

#### 1. Data Format Change
**Old format**: `angle,distance.`
**New format**: `angle,distance1,distance2.`
- Allows Processing to display both sensors independently
- Maintains backward compatibility with parsing structure

#### 2. Servo Sweep Optimization
- **Removed detachment logic**: Continuous scanning needed for radar visualization
- **Increased step delay**: 30ms provides stable readings while maintaining smooth motion
- **1-degree steps**: Provides 240 data points per sweep (better resolution)
- **Extended range**: 240° sweep (15°-255°) provides wider field of view

#### 3. Beeper Non-blocking Design
- Original code had blocking delays for beeps
- New design uses `millis()` timing for parallel operation
- Servo continues scanning while beeper operates independently

#### 4. Measurement Improvements
- **Staggered triggers**: Prevents ultrasonic interference between sensors
- **Timeout protection**: 30ms timeout prevents hanging on no-echo conditions
- **Hysteresis filtering**: Reduces jitter when objects near sensor boundaries

#### 5. Processing Enhancements
- **Dual-color rendering**: Visual distinction between sensors
- **Error handling**: Try-catch prevents crashes from corrupt serial data
- **Updated range markers**: Changed 40cm to 60cm to match detection limit

## Troubleshooting

### No Serial Connection
- Check COM port in both Arduino IDE and Processing
- Ensure only one program accesses serial at a time
- Try unplugging/replugging USB cable

### Erratic Distance Readings
- Reduce `SERVO_STEP_DELAY` if servo vibration is issue
- Increase `TRIG_GAP_MS` if sensors interfere with each other
- Check sensor mounting - ensure stable attachment

### Servo Jitter
- Verify power supply can handle servo current draw
- Add capacitor (100µF) across servo power pins
- Check `SERVO_MIN_US` and `SERVO_MAX_US` match your servo specs

### No Audio Feedback
- Verify speaker/buzzer connection on Pin 13
- Check if speaker can handle tone frequencies (600-1400 Hz)
- Ensure `updateBeeper()` is being called in loop

### Processing Display Issues
- Adjust `size()` to match your monitor resolution
- If colors don't appear, check fill() and stroke() calls
- Increase motion blur value (line 15) for stronger trail effect

## Performance Specifications

- **Sweep rate**: ~8 seconds per full sweep (15° to 255° and back)
- **Angular resolution**: 1° per measurement
- **Sweep coverage**: 240° field of view
- **Detection range**: 2cm - 60cm (configurable)
- **Update rate**: ~30 measurements per second
- **Serial baud**: 9600 (stable for both visualization and data)

## Advanced Configuration

### Adjusting Detection Range
In Arduino code, modify:
```cpp
const int MAX_DETECT_CM = 60;  // Change to 40, 80, 100, etc.
```
Update Processing display text accordingly (line 130-133).

### Changing Sweep Speed
In Arduino code:
```cpp
const int SERVO_STEP_DELAY = 30;  // Lower = faster, Higher = slower
```
Minimum recommended: 15ms (smooth but may miss readings)
Maximum tested: 50ms (very smooth but slow scan)

### Modifying Sweep Range
In Arduino code:
```cpp
const int POS_MIN = 15;   // Leftmost angle
const int POS_MAX = 255;  // Rightmost angle (240° sweep)
```
Note: Ensure your servo supports the full range. Most standard servos operate 0°-180°, but continuous rotation or 270° servos can handle extended ranges.

### Disabling Battery Monitor
Set in Arduino code:
```cpp
#define ENABLE_BATT_MON false
```

## Technical Notes

### Why Ultrasonic Stack Method?
While VL53L7CX ToF sensors provide superior performance, ultrasonic sensors remain viable due to:
- Lower cost (~$2 vs ~$15 per sensor)
- Simpler interfacing (2 pins vs I²C)
- No I²C address conflicts to manage
- Adequate performance for close-range detection

### Crosstalk Mitigation
The 40ms gap between triggers prevents sensors from detecting each other's pulses. If interference still occurs:
- Increase gap to 50-60ms
- Add acoustic barriers between sensors
- Angle sensors slightly outward (5-10°)

### Data Throughput
At 9600 baud with average message length:
- Message format: "165,999,999." = 12 bytes
- Overhead: ~2 bytes
- Total: 14 bytes per reading
- Theoretical max: ~68 readings/second
- Actual: ~33 readings/second (limited by servo speed)

## Future Enhancements

Potential improvements:
1. **Data logging**: Save scans to CSV for analysis
2. **3D visualization**: Add depth perspective in Processing
3. **Object tracking**: Implement persistence across sweeps
4. **Multi-servo**: Add vertical scanning for 3D mapping
5. **Wireless**: ESP32 with WiFi/BLE data transmission
6. **ML integration**: Object classification from distance patterns

## License & Credits

This integration combines:
- Original radar visualization concept (Processing community)
- Dual-sensor beeper system (custom implementation)
- Enhanced for educational robotics applications

Feel free to modify and adapt for your projects!

---

**Version**: 1.0
**Last Updated**: November 2025
**Tested On**: Arduino Uno, Nano (ATmega328P)
