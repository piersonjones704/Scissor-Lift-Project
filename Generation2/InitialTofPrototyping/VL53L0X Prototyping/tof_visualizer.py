# ToF Servo Scanner Visualizer — robust, per-angle overwrite (0..270°)
# Supports 2- or 4-column serial lines: angle,dist[,x,y]
# Draws a rectangular safety zone overlay that matches Arduino RECT_*.
# Window stays up even if serial is missing.

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pkg_resources")

import math, time, pygame, serial
from serial.tools import list_ports

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/cu.usbserial-10"   # "AUTO" or explicit, e.g. "COM5" or "/dev/tty.usbmodemXXXX"
BAUD = 115200
MAX_RANGE_MM = 1200
WIDTH, HEIGHT = 900, 900
MARGIN = 60
DOT_RADIUS = 6
FPS = 60
ANGLE_MIN, ANGLE_MAX = 0, 270

# Rectangular safety zone (match Arduino)
RECT_X_MIN = -200
RECT_X_MAX = 800
RECT_Y_MIN = -100
RECT_Y_MAX = 600
# ----------------------------------------

def clamp(x, lo, hi): return max(lo, min(hi, x))

def hsv_to_rgb(h, s, v):
    h = h % 360; c = v*s; x = c*(1-abs(((h/60)%2)-1)); m = v-c
    if   0 <= h < 60:   rp,gp,bp = c,x,0
    elif 60 <= h <120:  rp,gp,bp = x,c,0
    elif 120<=h<180:    rp,gp,bp = 0,c,x
    elif 180<=h<240:    rp,gp,bp = 0,x,c
    elif 240<=h<300:    rp,gp,bp = x,0,c
    else:               rp,gp,bp = c,0,x
    return (int((rp+m)*255), int((gp+m)*255), int((bp+m)*255))

def proximity_color(dist_mm):
    hue = (clamp(dist_mm, 0, MAX_RANGE_MM) / MAX_RANGE_MM) * 240.0   # blue->red
    return hsv_to_rgb(hue, 0.95, 1.0)

def polar_to_xy(r_px, theta_deg, cx, cy):
    rad = math.radians(theta_deg)
    return int(cx + r_px*math.cos(rad)), int(cy - r_px*math.sin(rad))

def autodetect_port():
    cands = []
    for p in list_ports.comports():
        name = p.device
        desc = (p.description or "").lower()
        if any(tok in name.lower() for tok in ["usbmodem","usbserial","cu.usb","com"]) or "arduino" in desc:
            cands.append(name)
    return cands[0] if cands else None

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(0.3)
        return ser
    except Exception as e:
        print(f"Could not open serial port {port}: {e}")
        return None

def main():
    print("DEBUG: entering main()")

    # ----- serial setup -----
    port = SERIAL_PORT
    if port == "AUTO":
        port = autodetect_port()
        print(f"🔎 Auto-detected serial port: {port}" if port else "Auto-detect failed. Set SERIAL_PORT manually.")
    ser = open_serial(port, BAUD) if port else None
    connected = ser is not None

    # ----- pygame setup -----
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ToF Servo Scanner (Per-Angle Overwrite + Safety Zone)")
    print("DEBUG: window created")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 22)
    center = (WIDTH//2, HEIGHT//2)
    max_r_px = min(WIDTH, HEIGHT)//2 - MARGIN
    scale = max_r_px / MAX_RANGE_MM  # mm -> px scale

    # bins: one reading per degree (overwrite each sweep)
    bins = {deg: None for deg in range(ANGLE_MIN, ANGLE_MAX+1)}

    running = True
    while running:
        # events
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_r:
                    for k in bins: bins[k] = None
                elif ev.key == pygame.K_c:
                    if ser:
                        try: ser.close()
                        except: pass
                    ser = open_serial(port, BAUD) if port else None
                    connected = ser is not None

        # serial read (non-blocking)
        if connected:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    parts = line.split(",")
                    if len(parts) >= 2:
                        ang = float(parts[0])
                        dist = float(parts[1])
                        if dist >= 0:
                            deg_bin = int(round(ang))
                            deg_bin = int(clamp(deg_bin, ANGLE_MIN, ANGLE_MAX))
                            bins[deg_bin] = (clamp(dist, 0, MAX_RANGE_MM), proximity_color(dist))
            except Exception as e:
                print(f"⚠️ serial hiccup: {e}")
                connected = False

        # draw frame
        screen.fill((0, 0, 0))

        # range rings
        pygame.draw.circle(screen, (40,40,40), center, max_r_px, 1)
        for frac in [0.25, 0.5, 0.75, 1.0]:
            r = int(max_r_px * frac)
            pygame.draw.circle(screen, (25,25,25), center, r, 1)
            label = font.render(f"{int(MAX_RANGE_MM*frac)} mm", True, (120,120,120))
            screen.blit(label, (center[0] + r + 6, center[1] - 10))

        # angle spokes
        for a in [0, 45, 90, 135, 180, 225, 270]:
            x,y = polar_to_xy(max_r_px, a, *center)
            pygame.draw.line(screen, (35,35,35), center, (x,y), 1)

        # safety rectangle
        def mm_to_px(x_mm, y_mm):
            return int(center[0] + x_mm * scale), int(center[1] - y_mm * scale)
        x1, y1 = mm_to_px(RECT_X_MIN, RECT_Y_MIN)
        x2, y2 = mm_to_px(RECT_X_MAX, RECT_Y_MAX)
        rect_w, rect_h = (x2 - x1), (y1 - y2)
        pygame.draw.rect(screen, (80,80,80), (x1, y2, rect_w, rect_h), 1)

        # dots
        for deg, payload in bins.items():
            if payload is None: continue
            dist_mm, color = payload
            r_px = int((dist_mm / MAX_RANGE_MM) * max_r_px)
            x, y = polar_to_xy(r_px, deg, *center)
            pygame.draw.circle(screen, color, (x, y), DOT_RADIUS)

        # HUD
        status = f"Port: {port or '—'} | {'CONNECTED' if connected else 'NOT CONNECTED'}  |  Keys: [R]=reset  [C]=reconnect"
        hud = font.render(status, True, (180,180,180))
        screen.blit(hud, (20, 20))
        legend = font.render("Blue = far, Red = near | Gray box = safety zone", True, (160,160,160))
        screen.blit(legend, (20, 45))

        pygame.display.flip()
        clock.tick(FPS)

    if ser:
        try: ser.close()
        except: pass
    pygame.quit()
    print("DEBUG: exited cleanly")

if __name__ == "__main__":
    main()
