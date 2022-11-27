import argparse
import cv2
import math
import socket
from turbojpeg import TurboJPEG
import time
import threading
import re

quitting = False
got_connection = False

def publish_mc_group():
    global quitting
    global got_connection
    MCAST_GRP = '239.1.1.1'
    MCAST_PORT = 5000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    while not quitting and not got_connection:
        try:
            sock.sendto(b'Hello World!', (MCAST_GRP, MCAST_PORT))
            time.sleep(1)
        except:
            break
    print("Multicast thread exiting")

def main(host="0.0.0.0", port=8888):
    global quitting
    global got_connection
    header = b'\xaa\xbb\xcc\xdd'
    jpeg = TurboJPEG()
    t = threading.Thread(target=publish_mc_group, args=[])
    t.start()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        print("server lisening on {}:{}".format(host,port))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            got_connection = True
            total_bytes = 0
            total_frames = 0
            start = None
            while True:
                # try to get all chunks of the message
                # print("Trying to get new image")
                img_data = bytearray()
                max_recv_size = 1*1024
                remaining = max_recv_size
                got_header = False
                while True:
                    recv_len = remaining if remaining < max_recv_size else max_recv_size
                    # print("receiving", recv_len, "bytes")
                    try:
                        data = conn.recv(recv_len)
                    except KeyboardInterrupt:
                        print("quitting...")
                        quitting = True
                        t.join()
                        return
                    if not start:
                        start = time.time()
                    # find the header in the bytearray
                    if not got_header:
                        try:
                            index = data.index(header)
                            if index >= 0:
                                got_header = True
                                length = \
                                    (int(data[index+4]) << 24) + \
                                    (int(data[index+5]) << 16) + \
                                    (int(data[index+6]) << 8) + \
                                    int(data[index+7])
                                data_start = index + 8
                                data_bytes = len(data) - data_start
                                num_img_bytes = min(length, data_bytes)
                                remaining = length - num_img_bytes
                                data_end = data_start + num_img_bytes + 1
                                img_data.extend(data[data_start:data_end])
                                # print("Got index:", index)
                                # print("Got data_start:", data_start)
                                # print("Got data_end:", data_end)
                                # print("Got header, image length:", length)
                                # print("remaining:", remaining)
                        except:
                            pass
                    elif got_header and remaining > 0:
                        num_img_bytes = min(remaining, len(data))
                        img_data.extend(data[:num_img_bytes])
                        remaining = remaining - num_img_bytes

                    if got_header and remaining == 0:
                        break;

                end = time.time()
                elapsed = end - start
                total_bytes += len(img_data)
                total_frames += 1
                # print("got image data, len:", len(img_data))
                print("metrics: {:.0f} KB/s, {:.1f} FPS".format((total_bytes / 1024) / elapsed, total_frames / elapsed))
                try:
                    img = jpeg.decode(img_data)
                    cv2.imshow('Remote Camera', img)
                    cv2.waitKey(1)
                except:
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
