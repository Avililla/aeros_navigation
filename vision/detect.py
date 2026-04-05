import cv2
import numpy as np
import socket
import struct
import sys
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from ultralytics import YOLO

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 5601


def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Bridge cerrado")
        data += chunk
    return data


def main():
    model = YOLO("yolov8n.pt")

    print("[AEROS Vision] Conectando al bridge (tcp:5601)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((BRIDGE_HOST, BRIDGE_PORT))
    except ConnectionRefusedError:
        print("[AEROS Vision] Error: bridge no está corriendo.")
        print("  Lanza primero: /usr/bin/python3 vision/gz_camera_bridge.py")
        sys.exit(1)

    print("[AEROS Vision] Conectado. Pulsa 'q' para salir.")

    try:
        while True:
            header = recv_exact(sock, 12)
            w, h, fmt = struct.unpack("<III", header)

            length_data = recv_exact(sock, 4)
            length = struct.unpack("<I", length_data)[0]

            frame_data = recv_exact(sock, length)

            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((h, w, 3))
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            results = model(frame_bgr, verbose=False)
            annotated = results[0].plot()

            cv2.imshow("AEROS Vision - YOLOv8", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print("[AEROS Vision] Cerrado.")


if __name__ == "__main__":
    main()
