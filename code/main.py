from machine import TouchPad, Pin,PWM,I2C,
from MX1508 import *
from tcs34725 import *
from time import sleep, sleep_ms
from neopixel import NeoPixel
from BLEUART import *
from micropython import const
import uasyncio as asio

# PINS
_NeoPin=4
_Servo180=23
_Servo360=12

i2c_bus = I2C(0, sda=Pin(21), scl=Pin(22))  #----------
tcs = TCS34725(i2c_bus)  #----------
tcs.gain(4)#gain must be 1, 4, 16 or 60  ----------
tcs.integration_time(80)  #----------


debug=1
NUM_OF_LED = 1
np = NeoPixel(Pin(_NeoPin), NUM_OF_LED)
color=['Cyan','Black','Yellow','Navy','Orange','Green','Red']

#1,2 - правая пара
motor = MX1508(26, 27)
motor2 = MX1508(32, 33)
motor3 = MX1508(16, 17)  
motor4 = MX1508(18, 19) 


sp=1023
an=0
on=0
col_id=0
comand=''
pwm = PWM(Pin(_Servo180,Pin.OUT))   #подключение сервы
pwm.freq(50)
pwm.duty(0)

way=0
pwm360 = PWM(Pin(_Servo360,Pin.OUT))
pwm360.freq(50)
pwm360.duty(0)

led = Pin(2, Pin.OUT)   #подключение лампочки
led_state = 0  

def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def servo(pin, angle):
    pin.duty(map(angle, 0, 180, 20, 120))   #cерва на 180
def on_rx():
    global comand,on
    on=1
    comand=uart.read().decode().strip()
    #comand=comand.rstip('\x00')
    print(comand)   #считывание команд с Bluefruit
def Servo360(pin):  #серва 360
    global way
    if way == 1:
        pin.duty(60) 
    elif way == 0:
        pin.duty(77) 
    elif way == -1:
        pin.duty(94)

ble = bluetooth.BLE()
uart = BLEUART(ble,"meow")  #блютус
uart.irq(handler=on_rx)    

async def color_det():
    global col_id
    await asio.sleep_ms(0)
    rgb=tcs.read(1)
    r,g,b=rgb[0],rgb[1],rgb[2]
    h,s,v=rgb_to_hsv(r,g,b)
    if (h>340)or(h<10):
        col_id=6
        np[0] = (255,0,0)
    if 10<h<60:
        col_id=4
        np[0] = (255,128,0)
    if 60<h<120:
        col_id=2
        np[0] = (255,255,51)
    if 120<h<180:
        col_id=5
        np[0] = (0,255,0)
    if 180<h<240:
        if v>130:
            col_id=0
            np[0] = (51,255,255)
        if 30<v<40:
            col_id=3
            np[0] = (0,0,153)
        if v<30:
            col_id=1
            np[0] = (0,0,0)
    np.write()
    if debug:
        print('Color is {}. R:{} G:{} B:{} H:{:.0f} S:{:.0f} V:{:.0f}'.format(color[col_id],r,g,b,h,s,v))
                
async def send_color(int_ms):     
    while(1):
        try:
            while True:
                uart.write(color[col_id]+"\n")
                await asio.sleep_ms(int_ms)
        except KeyboardInterrupt:
            pass
    
async def do_it(int_ms):
    global an,on,led,led_state, way
    while 1:
        await asio.sleep_ms(int_ms)
        #print(comand)
        if comand=='TEST_LED':
            led.value(not led_state)  # Toggle the LED state (on/off)
            led_state = 1 - led_state  # Update the LED state
        
        # Movement
        if comand=='!B516':
            motor.forward(sp)
            motor2.forward(sp)
            motor3.reverse(sp)
            motor4.reverse(sp)
        elif comand=='!B507':
            motor.stop()
            motor2.stop()
            motor3.stop()
            motor4.stop()
        if comand=='!B615':
            motor.reverse(sp)
            motor2.reverse(sp)
            motor3.forward(sp)
            motor4.forward(sp)
        elif comand=='!B606':
            motor.stop()
            motor2.stop()
            motor3.stop()
            motor4.stop()
        if comand=='!B813':
            motor3.reverse(sp)
            motor4.reverse(sp)
        elif comand=='!B804':
            motor3.stop()
            motor4.stop()
        if comand=='!B714':
            motor.forward(sp)
            motor2.forward(sp)
        elif comand=='!B705':
            motor.stop()
            motor2.stop()
        
        if comand=='!B11:' and on:
            an+=30
            on=0
            if an>180:
                an=180
        if comand=='!B219' and on:
            an-=30
            on=0
            if an<0:
                an=0
        
        if comand=='!B318':
            way=1
            #on=0
        elif comand=='!B417':
            way=-1
            #on=0
        else:
            way=0
        
        servo(pwm, an)
        Servo360(pwm360)
        await color_det()  #----------

# define loop
loop = asio.get_event_loop()

#create looped tasks
loop.create_task(do_it(5))
loop.create_task(send_color(100))  #----------
# loop run forever
loop.run_forever()

#uart.close()
