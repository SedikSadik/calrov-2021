import time
from pymavlink import mavutil

#Mavlink Bağlantısı
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()

#Kamera Servo Kontrol Fonksiyonu
def look_at(tilt, roll=0, pan=0):
    """
    Gimbal istenen pozisyona taşınır.
    Args:
        tilt (float): santiderece cinsinden tilt (0 nötr)
        roll (float, isteğe bağlı): santiderece cinsinden roll (0 nötr)
        pan  (float, isteğe bağlı): santiderece cinsinden pan (0 nötr)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)


# Kamerayı kontrol etmek için herhangi bir komut verilebilir
while True:
    for angle in range(-50, 50):
        look_at(angle*100)
        time.sleep(0.1)
    for angle in range(-50, 50):
        look_at(-angle*100)
        time.sleep(0.1)
# YAZILIM ÖRNEĞİ: KAMERA SERVO KONTROL

