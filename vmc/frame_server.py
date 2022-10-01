import socket
import time
from threading import Thread

import numpy as np

from vmc import stream


TOKEN = "uqqH_Uq0G0TxN9BO0pd5TN5a9qFGoz-FH6OrBTYo4yxEX-J9351rGTI3YCwnwoUVbGBIXFvNVRECoCJGQcbDiQ"

BUFF_SIZE = 65536
socket.setdefaulttimeout(2)
host_ip = '0.0.0.0'
port = 9999
socket_address = (host_ip, port)
print('Frame API listening at:', socket_address)

_, csi_frame = stream.encode_frame(np.zeros((1, 1, 3), np.uint8))
# zed_right_frame = np.zeros((1, 1, 3), np.uint8)
# zed_left_frame = np.zeros((1, 1, 3), np.uint8)
# zed_depth_frame = np.zeros((1, 1, 3), np.uint8)
# thermal_frame = np.zeros((8, 8, 3), np.uint8)

thread: Thread = None


# @app.route('/', methods=['HEAD'])
# def status():
#     return '', 200
#
#
# @app.route('/csi', methods=['GET'])
# def csi():
#     return csi_frame, 200


# @app.route('/zed/right', methods=['GET'])
# def zed_right():
#     return json.dumps(zed_right_frame.tolist())
#
#
# @app.route('/zed/left', methods=['GET'])
# def zed_left():
#     return json.dumps(zed_left_frame.tolist())
#
#
# @app.route('/zed/depth', methods=['GET'])
# def zed_depth():
#     return json.dumps(zed_depth_frame.tolist())
#
#
# @app.route('/thermal', methods=['GET'])
# def thermal():
#     return json.dumps(thermal_frame.tolist())


def update_csi(frame: np.ndarray):
    global csi_frame
    csi_frame = frame


# def update_zed_right(frame: np.ndarray):
#     global zed_right_frame
#     zed_right_frame = frame
#
#
# def update_zed_left(frame: np.ndarray):
#     global zed_left_frame
#     zed_left_frame = frame
#
#
# def update_zed_depth(frame: np.ndarray):
#     global zed_depth_frame
#     zed_depth_frame = frame
#
#
# def update_thermal(frame: np.ndarray):
#     global thermal_frame
#     thermal_frame = frame

# async def accept_client(ws):
#     try:
#         # token = await asyncio.wait_for(ws.recv(), 500)
#         # print(token)
#         # print(TOKEN)
#         # if token == TOKEN:
#         while True:
#             await ws.send(csi_frame)
#             #time.sleep(1/30)
#         # else:
#         #     print("auth failed")
#         #     await ws.close()
#         #     await ws.wait_closed()
#     except websockets.ConnectionClosedError as e:
#         print(e)
#     except websockets.ConnectionClosedOK:
#         print("closed")
#         return


def loop():
    while True:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
        server_socket.bind(socket_address)

        while True:
            try:
                msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
                break
            except TimeoutError:
                pass
        while True:
            try:
                server_socket.sendto(csi_frame, client_addr)
            except TimeoutError:
                print("break")
                break
            time.sleep(1/60)
        try:
            server_socket.shutdown(socket.SHUT_RDWR)
        except TimeoutError:
            pass
        server_socket.close()
    # async with websockets.serve(accept_client, "localhost", 8080):
    #     await asyncio.Future()


def start():
    global thread
    # thread = Thread(target = lambda: serve(app, host = "192.168.1.180", port = 8080, ipv6 = False))
    thread = Thread(target = lambda: loop())
    thread.start()


def watchdog_loop():
    while True:
        if thread is None or not thread.is_alive():
            start()
        time.sleep(5)

# class FrameSegment(object):
#     MAX_DGRAM = 2 ** 16
#     MAX_IMAGE_DGRAM = MAX_DGRAM - 64  # minus 64 bytes in case UDP frame overflown
#
#     def __init__(self, sock, port, addr = "127.0.0.1"):
#         self.s = sock
#         self.port = port
#         self.addr = addr
#
#     def udp_frame(self, img):
#         compress_img = cv2.imencode('.jpg', img)[1]
#         dat = compress_img.tostring()
#         size = len(dat)
#         num_of_segments = math.ceil(size / self.MAX_IMAGE_DGRAM)
#         array_pos_start = 0
#
#         while num_of_segments:
#             array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
#             self.s.sendto(struct.pack("B", num_of_segments) + dat[array_pos_start:array_pos_end], (self.addr, self.port))
#             array_pos_start = array_pos_end
#             num_of_segments -= 1


def run():
    Thread(target = watchdog_loop).start()


# def test():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     fs = FrameSegment(s, 8080)
#     while True:
#         fs.udp_frame(csi_frame)
#
#
# Thread(target = test).start()
