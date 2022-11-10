import argparse
import cv2
import socket
from turbojpeg import TurboJPEG

def main(host="192.168.1.23", port=8888):
    header = b'\xaa\xbb\xcc\xdd'
    jpeg = TurboJPEG()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        print("server lisening on {}:{}".format(host,port))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                # try to get all chunks of the message
                img_data = bytearray()
                max_recv_size = 10*1024
                remaining = max_recv_size
                got_header = False
                while True:
                    recv_len = remaining if remaining < max_recv_size else max_recv_size
                    data = conn.recv(recv_len)
                    if data[0:4] == header:
                        length = (int(data[4]) << 24) + (int(data[5]) << 16) + (int(data[6]) << 8) + int(data[7])
                        remaining = length - len(data) + 8
                        img_data.extend(data[8:])
                        got_header = True
                        print("Got header, image length:", length)
                        print("remaining:", remaining)
                    elif got_header and remaining > 0:
                        if remaining > len(data):
                            remaining = remaining - len(data)
                            img_data.extend(data)
                        else:
                            needed_length = len(data) - remaining
                            remaining = 0
                            img_data.extend(data[:needed_length])
                    if got_header and remaining <= 0:
                        break;
                print("got image data, len:", len(img_data))
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
    parser.add_argument('-H', '--host', type=str, default="192.168.1.23",
                        help="IP address to bind to")
    parser.add_argument('-p', '--port', type=int, default=8888,
                        help="Port number to bind to")
    args = parser.parse_args()
    main(args.host, args.port)
