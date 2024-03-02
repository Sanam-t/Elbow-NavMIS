import socket
import struct
import numpy as np
import time
import pickle

class udp_telemetry():
    def __init__(self):
        self.OUTPUT_IP='192.168.1.3'
        self.OUTPUT_PORT=5000
        self.INPUT_IP='127.0.0.1'
        self.INPUT_PORT=5000
        self.out_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True
        self.transmit_counter = 0

    def transmit_data(self,data):
        byte_object = pickle.dumps(data)
        self.out_socket.sendto(byte_object, (self.OUTPUT_IP, self.OUTPUT_PORT))

    def start_rx(self,callback):
        self.in_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.in_sock.bind((self.INPUT_IP, self.INPUT_PORT))

        while self.RX_RUNNING:
            data = self.in_sock.recvmsg(4096)
            local_stamp,seq_num = struct.unpack('QQ', data[0][0:16])
            num_of_frames=struct.unpack('i',data[0][16:20])[0]
            markers_per_frame = struct.unpack(f'{num_of_frames}i',data[0][20:20+4*num_of_frames])
            total_number_of_markers = sum(markers_per_frame)
            msg_format=f'i{len(markers_per_frame)}i{2*sum(markers_per_frame)}d'
            unpacked=struct.unpack(msg_format,data[0][16:])
            all_markers = np.array(unpacked[(-2*sum(markers_per_frame)):]).reshape(-1,2)
            markers_in_frames=np.split(all_markers, np.cumsum(markers_per_frame).tolist())[:-1]
            stamp = time.time()
            callback(stamp,local_stamp ,seq_num, self.INPUT_PORT, markers_in_frames)

    def terminate(self):
        self.RX_RUNNING = False
        in_sock.close()
        out_sock.close()
