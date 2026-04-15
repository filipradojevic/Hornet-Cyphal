import socket
import time
from pymavlink.dialects.v20.common import MAVLink

UDP_TARGET_IP = "192.168.1.150"   # Mikrokontroler
UDP_TARGET_PORT = 1024

# Socket koji će koristiti lažni izvorni IP i port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.1.151", 1024))   # <<< ključno!

mav = MAVLink(None)
mav.srcSystem = 0
mav.srcComponent = 76

print("Unos PWM u mikrosekundima...\n")

while True:
    param2 = float(input("Unesi vrednost za param2 (custom mode): "))

    msg = mav.command_long_encode(
        target_system=1,
        target_component=1,
        command=176,  # MAV_CMD_DO_SET_MODE
        confirmation=0,
        param1=1,    # base mode
        param2=param2,  # custom mode
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0
    )

    pkt = msg.pack(mav)
    sock.sendto(pkt, (UDP_TARGET_IP, UDP_TARGET_PORT))
    print("Poslato:", param2)



