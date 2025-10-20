# server_tcp.py
# Tiny TCP control server for PiCar-X
# Protocol: newline-delimited JSON (one JSON object per line)

import json, socket, threading, time
from picarx import Picarx

HOST = "0.0.0.0"
PORT = 5555
DEFAULT_SPEED = 40        # 0..100
TURN_ANGLE = 30           # +/- degrees for left/right

px = Picarx()

def do_move(req):
    """Execute movement command and return a JSON-able result."""
    cmd = req.get("cmd")
    speed = int(req.get("speed", DEFAULT_SPEED))
    angle = int(req.get("angle", 0))

    # Clamp inputs a little to be safe
    speed = max(0, min(speed, 100))
    angle = max(-45, min(angle, 45))

    if cmd == "forward":
        px.set_dir_servo_angle(angle)
        px.forward(speed)
        return {"ok": True}

    if cmd == "backward":
        px.set_dir_servo_angle(angle)
        px.backward(speed)
        return {"ok": True}

    if cmd == "left":
        px.set_dir_servo_angle(-TURN_ANGLE)
        px.forward(speed)
        return {"ok": True}

    if cmd == "right":
        px.set_dir_servo_angle(TURN_ANGLE)
        px.forward(speed)
        return {"ok": True}

    if cmd == "stop":
        px.stop()
        return {"ok": True}

    if cmd == "sensors":
        dist = None
        try:
            dist = round(px.ultrasonic.read(), 2)
        except Exception as e:
            # ultrasonic may occasionally fail; report gracefully
            return {"ok": False, "error": f"ultrasonic: {e}"}
        # Add a couple handy stats if available
        info = {"distance_cm": dist}
        try:
            gm_val_list = px.get_grayscale_data()
            gm_state = px.get_cliff_status(gm_val_list)

            if gm_state is False:
                state = "safe"
            else:
                state = "danger" 

            info["cliff"] = state
        except Exception as e:
            
            return {"ok": False, "error": f"grayscale: {e}"}
            
        return {"ok": True, **info}

    return {"ok": False, "error": "unknown_cmd"}

def handle(conn, addr):
    conn.settimeout(60)
    buf = b""
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    req = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    conn.sendall(b'{"ok":false,"error":"bad_json"}\n')
                    continue
                try:
                    resp = do_move(req)
                except Exception as e:
                    # Never let the thread crash; stop motors on unexpected error
                    px.stop()
                    resp = {"ok": False, "error": str(e)}
                conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
    except socket.timeout:
        # idle connection; just close
        pass
    finally:
        # Safety stop when a client disconnects
        try: px.stop()
        except Exception: pass
        conn.close()

def main():
    print(f"TCP control server on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        while True:
            conn, addr = s.accept()
            print("client:", addr)
            threading.Thread(target=handle, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        try: px.stop()
        except Exception: pass
