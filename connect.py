#Gerekli kütüphanelerin indirilmesi
from time import sleep
from pymavlink import mavutil

#udpin:192.168.2.1:14550
# UDP iletişimin kurulması
master = mavutil.mavlink_connection("udpin:192.168.2.1:14550")

master.wait_heartbeat()
print("Bağlantı kuruldu!")
#mavpackettype = master.recv_match().to_dict()["mavpackettype"]
#Aracın durum ile ilgili veri alınması
"""
def sensor_data():
    try:
        packet = master.recv_match().to_dict()
        if packet['mavpackettype']=='ATTITUDE':
            return packet
    except:
        pass
"""
        #YAZILIM ÖRNEĞİ: ARAÇ BAĞLANTISI 
while True:
    try:
        packet = master.recv_match().to_dict()
        if packet['mavpackettype']=='ATTITUDE': #or packet['mavpackettype']=='AHRS2':
            print(packet['roll']+'\n'+packet['pitch']+'\n'+packet['yaw'])
            print(packet)
    except:
        pass
    sleep(0.1)
#5303083674  1430