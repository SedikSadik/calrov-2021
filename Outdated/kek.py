from datetime import datetime as dt #zaman kütüphanesini import et
def on_saniye(): # on_saniye diye bir fonksiyon oluştur
    zaman = dt.now().second #anlık zamanı al
    while(True): # while döngüsüne gir
        if(zaman+10 != dt.now().second): # eğer zaman+10 şu anki zamandan farklıysa
            print("Running")
            #pwm_gonder(5,hiz(1)) # robotu full güçte ileriye hareket ettir.
        else: # eğer zaman+10 şu anki zamana eşit ise
            break # döngüyü durdur
on_saniye()
print("Done!")