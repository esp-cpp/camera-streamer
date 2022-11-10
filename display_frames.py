import argparse
import cv2
import socket
from turbojpeg import TurboJPEG
import time

def main(host="0.0.0.0", port=8888):
    header = b'\xaa\xbb\xcc\xdd'
    jpeg = TurboJPEG()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        print("server lisening on {}:{}".format(host,port))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            total_bytes = 0
            total_frames = 0
            start = None
            while True:
                # try to get all chunks of the message
                img_data = bytearray()
                max_recv_size = 40*1024
                remaining = max_recv_size
                got_header = False
                while True:
                    recv_len = remaining if remaining < max_recv_size else max_recv_size
                    # print("receiving", recv_len, "bytes")
                    data = conn.recv(recv_len)
                    if not data:
                        print("breaking loop because (not data) == true")
                        break
                    if not start:
                        start = time.time()
                    if data[0:4] == header:
                        length = (int(data[4]) << 24) + (int(data[5]) << 16) + (int(data[6]) << 8) + int(data[7])
                        remaining = length - len(data) + 8
                        img_data.extend(data[8:])
                        got_header = True
                        # print("Got header, image length:", length)
                        # print("remaining:", remaining)
                    elif got_header and remaining > 0:
                        if remaining >= len(data):
                            remaining = remaining - len(data)
                            img_data.extend(data)
                        else:
                            print("got more data than we needed:", len(data), remaining)
                            needed_length = len(data) - remaining
                            remaining = 0
                            img_data.extend(data[:needed_length])
                    if got_header and remaining <= 0:
                        break;
                end = time.time()
                elapsed = end - start
                total_bytes += len(img_data)
                total_frames += 1
                print("got image data, len:", len(img_data))
                print("metrics: {:.0f} KB/s, {:.1f} FPS".format((total_bytes / 1024) / elapsed, total_frames / elapsed))
                try:
                    img = jpeg.decode(img_data)
                    cv2.imshow('Remote Camera', img)
                    cv2.waitKey(1)
                except Exception as e:
                    # print("Could not open jpeg image:")
                    # print(e)
                    pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="display_frames",
                                     description="Start TCP server to receive JPEG encoded images and display them")
    parser.add_argument('-H', '--host', type=str, default="0.0.0.0",
                        help="IP address to bind to")
    parser.add_argument('-p', '--port', type=int, default=8888,
                        help="Port number to bind to")
    args = parser.parse_args()
    main(args.host, args.port)
