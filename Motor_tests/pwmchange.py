from time import sleep, time
import threading
from pymavlink import mavutil
#master = mavutil.mavlink_connection('192.168.2.1:14450')
def veri_al():
    while True:
        print('DATA recieved')
        sleep(0.1)
'''
def request_message_interval(message_id: int, frequency_hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # MAVLINK mesaj tanimi
        1e6 / frequency_hz, # Istenen frekans
        0, # Hedef, (0=arac)
        0, 0, 0, 0)

def pwm_gonder(channel_id, pwm=1500):
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                
        master.target_component,            
        *rc_channel_values)
'''
def changing_pwm(startpwm, stoppwm,step=1):
    if startpwm<stoppwm:
        pwm_start = startpwm
        while pwm_start < stoppwm:
            #pwm_gonder(3, pwm_start)
            pwm_start += step
            sleep(0.02)
            print(f'Pwm: {pwm_start}')
    elif startpwm>stoppwm:
        pwm_start = startpwm
        while pwm_start < stoppwm:
            #pwm_gonder(3, pwm_start)
            pwm_start -= step
            sleep(0.02)
            print(f'Pwm: {pwm_start}')
def Master_pwm(increment, channel_id=3, step=1):
    global operationtime
    #currentpwm = startpwm
    sleep(5)
    teststarttime = time()
    sleep(0.1)
    tm = increment
    while True:
        if time() > teststarttime      and  time()<teststarttime+tm:
            #pwm_gonder(channel_id,1900)
            print('PWM1')
            sleep(0.1)
        if time() > teststarttime+tm    and  time()<teststarttime+2*tm:
            #pwm_gonder(channel_id,1800)
            print('PWM2')
            sleep(0.1)
        if time() > teststarttime+2*tm   and  time()<teststarttime+3*tm:
            #pwm_gonder(channel_id,1700)
            print('PWM3')
            sleep(0.1)
        if time() > teststarttime+3*tm   and  time()<teststarttime+4*tm:
            #pwm_gonder(channel_id,1600)
            print('PWM4')
            sleep(0.1)
        if time() > teststarttime+4*tm   and  time()<teststarttime+5*tm:
            #pwm_gonder(channel_id,1500)
            print('PWM5')
            sleep(0.1)
        if time() > teststarttime+5*tm   and  time()<teststarttime+6*tm:
            #pwm_gonder(channel_id,1400)
            print('PWM6')
            sleep(0.1)
        if time() > teststarttime+6*tm   and  time()<teststarttime+7*tm:
            #pwm_gonder(channel_id,1300)
            print('PWM7')
            sleep(0.1)
        if time() > teststarttime+7*tm   and  time()<teststarttime+8*tm:
            #pwm_gonder(channel_id,1200)
            print('PWM8')
            sleep(0.1)
        if time() > teststarttime+40   and  time()<teststarttime+45:
            #pwm_gonder(channel_id,1100)
            print('PWM9')
            sleep(0.1)
        else:
            pass

pwm_thread = threading.Thread(target=Master_pwm, args=[5,3,1])
data_thread = threading.Thread(target=veri_al)
data_thread.start()
pwm_thread.start()
pwm_thread.join()
data_thread.join()
print("Done")