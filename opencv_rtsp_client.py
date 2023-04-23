import sys
import cv2

def stream(addr, port):
    vcap = cv2.VideoCapture(f"rtsp://{addr}:{port}/mjpeg/1")
    while(1):
        ret, frame = vcap.read()
        cv2.imshow('VIDEO', frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python ./opencv_rtsp_client <address> <rtsp_port>")
        sys.exit(1)
    stream(sys.argv[1], sys.argv[2])
