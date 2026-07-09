"""
Low Latency VL53L7CX Mapper - OPTIMIZED VERSION
Matches compact Arduino data format for minimal delay

Usage:
    python low_latency_mapper.py --port /dev/cu.usbserial-XXXX --mode heatmap
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import time
import argparse
import serial.tools.list_ports


class LowLatencyMapper:
    """Optimized mapper with minimal processing overhead"""
    
    def __init__(self, port, baudrate=921600, grid_size=4):
        self.port = port
        self.baudrate = baudrate
        self.grid_size = grid_size
        self.serial_conn = None
        
        # Use smaller data structures for speed
        self.distance_map = np.zeros((grid_size, grid_size), dtype=np.int16)
        self.valid_mask = np.zeros((grid_size, grid_size), dtype=bool)
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        self.last_frame_time = time.time()
        self.latency_ms = 0
        
    def connect(self):
        """Connect with optimized settings"""
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,  # Very short timeout
                write_timeout=0.05
            )
            # Disable buffering for lowest latency
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            time.sleep(0.5)
            print(f"✓ Connected: {self.port} @ {self.baudrate} baud")
            print("✓ Low latency mode active")
            
            # Read and display Arduino messages
            start = time.time()
            while time.time() - start < 2:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line and not line.startswith('F:'):
                        print(f"  Arduino: {line}")
            
            return True
            
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def read_frame_fast(self):
        """
        Fast frame parser for compact format
        Format: F:frameNum:timestamp:Z:row:col:dist:Z:row:col:dist:...E
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            # Quick check if data available
            if not self.serial_conn.in_waiting:
                return False
            
            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            
            # Must start with frame marker
            if not line.startswith('F:'):
                return False
            
            # Parse frame header
            try:
                parts = line.split(':')
                if len(parts) < 4:
                    return False
                
                frame_num = int(parts[1])
                timestamp = int(parts[2])
                
            except (ValueError, IndexError):
                return False
            
            # Clear previous data
            self.distance_map.fill(0)
            self.valid_mask.fill(False)
            
            # Parse zone data quickly
            # Format after header: Z:row:col:dist:Z:row:col:dist:...E
            i = 3
            zones_parsed = 0
            
            while i < len(parts) - 1:
                if parts[i] == 'Z':
                    try:
                        if i + 3 < len(parts):
                            row = int(parts[i + 1])
                            col = int(parts[i + 2])
                            dist = int(parts[i + 3])
                            
                            # Validate and store
                            if (0 <= row < self.grid_size and 
                                0 <= col < self.grid_size and
                                10 <= dist <= 1180):
                                
                                self.distance_map[row, col] = dist
                                self.valid_mask[row, col] = True
                                zones_parsed += 1
                            
                            i += 4
                        else:
                            break
                    except (ValueError, IndexError):
                        i += 1
                elif parts[i] == 'E':
                    break
                else:
                    i += 1
            
            # Update statistics
            self.frame_count += 1
            current_time = time.time()
            
            # Calculate latency
            self.latency_ms = (current_time * 1000) - timestamp
            
            # Calculate FPS
            if self.frame_count > 1:
                self.fps = 1.0 / (current_time - self.last_frame_time)
            self.last_frame_time = current_time
            
            return zones_parsed > 0
            
        except Exception as e:
            print(f"Parse error: {e}")
            return False
    
    def get_point_cloud_fast(self):
        """Fast point cloud generation"""
        points = []
        
        # Simplified FOV calculation
        angle_step = 45.0 / self.grid_size  # 45° FOV
        start_angle = -22.5  # Center at 0
        
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                if self.valid_mask[row, col]:
                    dist = self.distance_map[row, col]
                    
                    # Simple spherical to cartesian
                    angle_h = np.radians(start_angle + (col + 0.5) * angle_step)
                    angle_v = np.radians(start_angle + (row + 0.5) * angle_step)
                    
                    x = dist * np.sin(angle_h)
                    y = dist * np.sin(angle_v)
                    z = dist * np.cos(angle_h) * np.cos(angle_v)
                    
                    points.append([x, y, z])
        
        return np.array(points) if points else np.array([]).reshape(0, 3)
    
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("\n✓ Disconnected")


class FastVisualizer:
    """Ultra-fast visualizer with minimal overhead"""
    
    def __init__(self, mapper, mode='heatmap'):
        self.mapper = mapper
        self.mode = mode
        
        # Set matplotlib to interactive mode for speed
        plt.ion()
        
        if mode == 'heatmap':
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.im = None
            self.texts = []
            
        elif mode == '3d':
            self.fig = plt.figure(figsize=(10, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.scatter = None
            
        elif mode == 'both':
            self.fig = plt.figure(figsize=(14, 6))
            self.ax1 = self.fig.add_subplot(121, projection='3d')
            self.ax2 = self.fig.add_subplot(122)
            self.im = None
            self.scatter = None
        
        self.last_update = time.time()
        self.update_count = 0
        
        self.fig.canvas.mpl_connect('close_event', self.on_close)
    
    def on_close(self, event):
        self.mapper.disconnect()
    
    def update_heatmap_fast(self):
        """Optimized heatmap update"""
        # Clear previous
        self.ax.clear()
        
        # Masked data for transparency
        masked_data = np.ma.masked_where(
            ~self.mapper.valid_mask,
            self.mapper.distance_map
        )
        
        # Plot with inverted colormap (red=close, blue=far)
        self.im = self.ax.imshow(
            masked_data,
            cmap='RdYlBu',
            aspect='auto',
            interpolation='nearest',
            vmin=10,
            vmax=1180
        )
        
        # Add text annotations (only for valid zones)
        for i in range(self.mapper.grid_size):
            for j in range(self.mapper.grid_size):
                if self.mapper.valid_mask[i, j]:
                    dist = self.mapper.distance_map[i, j]
                    color = 'white' if dist > 600 else 'black'
                    self.ax.text(j, i, str(dist), ha='center', va='center',
                               color=color, fontsize=10, fontweight='bold')
                else:
                    self.ax.text(j, i, '✕', ha='center', va='center',
                               color='gray', fontsize=12, alpha=0.3)
        
        # Title with stats
        valid_count = np.sum(self.mapper.valid_mask)
        total = self.mapper.grid_size ** 2
        title = (f'VL53L7CX Low Latency | Frame: {self.mapper.frame_count} | '
                f'FPS: {self.mapper.fps:.1f} | Valid: {valid_count}/{total} | '
                f'Latency: {self.mapper.latency_ms:.0f}ms')
        self.ax.set_title(title, fontsize=10)
        self.ax.set_xlabel('Column')
        self.ax.set_ylabel('Row')
    
    def update_3d_fast(self):
        """Optimized 3D update"""
        self.ax.clear()
        
        points = self.mapper.get_point_cloud_fast()
        
        if len(points) > 0:
            colors = points[:, 2]  # Color by distance
            self.ax.scatter(
                points[:, 0], points[:, 1], points[:, 2],
                c=colors, cmap='RdYlBu', s=60, alpha=0.8,
                vmin=10, vmax=1180
            )
        
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_xlim([-600, 600])
        self.ax.set_ylim([-600, 600])
        self.ax.set_zlim([10, 1180])
        self.ax.view_init(elev=20, azim=45)
        
        valid_count = np.sum(self.mapper.valid_mask)
        total = self.mapper.grid_size ** 2
        title = (f'VL53L7CX 3D | FPS: {self.mapper.fps:.1f} | '
                f'Valid: {valid_count}/{total} | Latency: {self.mapper.latency_ms:.0f}ms')
        self.ax.set_title(title, fontsize=10)
    
    def update_both_fast(self):
        """Optimized both views"""
        # 3D
        self.ax1.clear()
        points = self.mapper.get_point_cloud_fast()
        if len(points) > 0:
            colors = points[:, 2]
            self.ax1.scatter(points[:, 0], points[:, 1], points[:, 2],
                           c=colors, cmap='RdYlBu', s=50, alpha=0.8,
                           vmin=10, vmax=1180)
        self.ax1.set_xlim([-600, 600])
        self.ax1.set_ylim([-600, 600])
        self.ax1.set_zlim([10, 1180])
        self.ax1.view_init(elev=20, azim=45)
        
        # Heatmap
        self.ax2.clear()
        masked_data = np.ma.masked_where(
            ~self.mapper.valid_mask,
            self.mapper.distance_map
        )
        self.ax2.imshow(masked_data, cmap='RdYlBu', aspect='auto',
                       interpolation='nearest', vmin=10, vmax=1180)
        
        for i in range(self.mapper.grid_size):
            for j in range(self.mapper.grid_size):
                if self.mapper.valid_mask[i, j]:
                    dist = self.mapper.distance_map[i, j]
                    color = 'white' if dist > 600 else 'black'
                    self.ax2.text(j, i, str(dist), ha='center', va='center',
                                color=color, fontsize=8, fontweight='bold')
        
        valid_count = np.sum(self.mapper.valid_mask)
        total = self.mapper.grid_size ** 2
        self.fig.suptitle(
            f'VL53L7CX | FPS: {self.mapper.fps:.1f} | Valid: {valid_count}/{total} | '
            f'Latency: {self.mapper.latency_ms:.0f}ms',
            fontsize=10
        )
    
    def update(self, frame_num):
        """Main update function"""
        # Read new data
        if not self.mapper.read_frame_fast():
            return
        
        # Update visualization based on mode
        if self.mode == 'heatmap':
            self.update_heatmap_fast()
        elif self.mode == '3d':
            self.update_3d_fast()
        elif self.mode == 'both':
            self.update_both_fast()
        
        self.update_count += 1
    
    def start(self, interval=10):
        """Start with minimal interval for max speed"""
        print(f"\n✓ Starting visualization (mode: {self.mode})")
        print("  Close window to exit\n")
        
        anim = FuncAnimation(
            self.fig,
            self.update,
            interval=interval,  # 10ms = up to 100 FPS
            cache_frame_data=False,
            blit=False
        )
        
        plt.tight_layout()
        plt.show(block=True)


def list_serial_ports():
    """List available ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []
    
    print("\nAvailable serial ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    
    return [port.device for port in ports]


def main():
    parser = argparse.ArgumentParser(description='VL53L7CX Low Latency Mapper')
    parser.add_argument('--port', type=str, help='Serial port')
    parser.add_argument('--baudrate', type=int, default=921600,
                       help='Baud rate (default: 921600)')
    parser.add_argument('--mode', type=str, default='heatmap',
                       choices=['3d', 'heatmap', 'both'],
                       help='Visualization mode')
    parser.add_argument('--grid', type=int, default=4, choices=[4, 8],
                       help='Grid size (default: 4)')
    parser.add_argument('--list-ports', action='store_true',
                       help='List ports and exit')
    
    args = parser.parse_args()
    
    if args.list_ports:
        list_serial_ports()
        return
    
    # Select port
    if not args.port:
        ports = list_serial_ports()
        if not ports:
            return
        try:
            selection = int(input("\nSelect port number: "))
            args.port = ports[selection]
        except (ValueError, IndexError):
            print("Invalid selection")
            return
    
    print("\n" + "="*60)
    print("VL53L7CX LOW LATENCY MAPPER")
    print("="*60)
    print(f"Port:      {args.port}")
    print(f"Baud:      {args.baudrate}")
    print(f"Mode:      {args.mode}")
    print(f"Grid:      {args.grid}x{args.grid}")
    print(f"Target:    <50ms latency, >20 FPS")
    print("="*60 + "\n")
    
    # Create mapper
    mapper = LowLatencyMapper(args.port, args.baudrate, args.grid)
    
    if not mapper.connect():
        return
    
    try:
        # Create and start visualizer
        viz = FastVisualizer(mapper, args.mode)
        viz.start(interval=10)  # 10ms interval
        
    except KeyboardInterrupt:
        print("\n✓ Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        mapper.disconnect()


if __name__ == '__main__':
    main()