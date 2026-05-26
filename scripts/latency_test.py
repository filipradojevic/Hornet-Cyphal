import serial
import struct

ser = serial.Serial("COM25", 115200)

FMT = "<IIIIIIIIIH"
SIZE = struct.calcsize(FMT)

while True:

    if ser.read(1) != b'\xAA':
        continue

    if ser.read(1) != b'\x55':
        continue

    data = ser.read(SIZE)

    recv_can, processed, sync, latency, latency_recv, avgReceive, avg, maxv, minv, cnt = struct.unpack(FMT, data)

    print(f"avgReceive = {avgReceive} us | avg={avg} us | max={maxv} us | min={minv} us | count={cnt}")