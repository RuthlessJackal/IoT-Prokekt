from uthingsboard.client import TBDeviceMqttClient
from time import sleep, ticks_ms
from sys import exit
from mpu6050 import MPU6050
from machine import Pin, PWM, I2C, UART
from gpio_lcd import GpioLcd
import gc
import secrets
from gps_simple import GPS_SIMPLE
from neopixel import NeoPixel
from adc_sub import ADC_substitute
from lmt87 import LMT87

buzzer = PWM(Pin(14,Pin.IN), duty = 0)
led1 = Pin(26,Pin.OUT)
lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25), d4_pin=Pin(33), d5_pin=Pin(32), d6_pin=Pin(21), d7_pin=Pin(22), num_lines=4, num_columns=20)
threshhold = 3000
handlerstart = ticks_ms()
knap1 = Pin(4, Pin.IN)
knap2 = Pin(0, Pin.IN, Pin.PULL_UP)
no_of_pixels = 12
np1 = NeoPixel(Pin(12, Pin.OUT), no_of_pixels)
np2 = NeoPixel(Pin(14, Pin.OUT), no_of_pixels)
on_color = (155,150,0)
off_color = (0,0,0)
lås = Pin(25, Pin.OUT)
låsledning = Pin(15, Pin.IN)
neopixel1ondelay = ticks_ms()
neopixel1offdelay = ticks_ms()
neopixel2ondelay = ticks_ms()
neopixel2offdelay = ticks_ms()
threshhold2 = 50
threshhold3 = 100
temp = LMT87(35)
displayupdate = ticks_ms()
threshhold4 = 250
telemetridelay = ticks_ms()
threshhold5 = 3000
alarmarming = ticks_ms()
alarmthreshold = 180000
locationdelay = ticks_ms()
locationthreshold = 200
alarmready = False
threshold1sec = 1000
alarmdelay1 = ticks_ms()
alarmdelay2 = ticks_ms()
alarmdelay3 = ticks_ms()

#########################################################################
# CONFIGURATION
gps_port = 2                                 # ESP32 UART port, Educaboard ESP32 default UART port
gps_speed = 9600                             # UART speed, defauls u-blox speed
#########################################################################
# OBJECTS
uart = UART(gps_port, gps_speed)             # UART object creation
gps = GPS_SIMPLE(uart)                      # GPS object creation
#########################################################################

#Initialisering af I2C objekt
i2c = I2C(0)
#Initialisering af mpu6050 objekt
imu = MPU6050(i2c)

# linær funktion til  batteri status
x1=1670
#3v
y1=0
x2=2440
#4,2
y2=100

a = (y2-y1)/(x2-x1)
b = y2 - a*x2

def formel_batt(x):
    y= a*x+b # y = 0,159x - 255,962 det er formlen vha. aflæsning af ADC-værdi til at finde en lineær funktion
    return int(y) #laver vores batteristatus om til integer(hel tal)

adc = ADC_substitute(34)


def blink1():
    
        for i in range(10):
            if ticks_ms() - neopixel1ondelay > threshhold2:
                for led in range(no_of_pixels):
                    np1[led] = on_color
                    np1.write()
                neopixel1ondelay = ticks_ms()
                
            if ticks_ms() - neopixel1offdelay > threshhold3:
                for led in range(no_of_pixels):
                    np1[led] = off_color
                    np1.write()
                print("on1")
                neopixel1offdelay = ticks_ms()

def blink2():
    for i in range(10):
        if ticks_ms() - neopixel2ondelay > threshhold2:
            for led in range(no_of_pixels):
                np2[led] = on_color
                np2.write()
            neopixel2ondelay = ticks_ms()
        if ticks_ms() - neopixel2offdelay > threshhold3:
            for led in range(no_of_pixels):
                np2[led] = off_color
                np2.write()
            print("on2")
            neopixel2offdelay = ticks_ms()

def sluk_alle(np_ring):        
    for i in range(no_of_pixels):
        np_ring[i] = off_color
    np_ring.write()       

sluk_alle(np1)
sluk_alle(np2)

def alarm():
    buzzer.duty(512)
    buzzer.freq(440) #a4
    if ticks_ms() - alarmdelay1 > threshold1sec: #her fungerer sleep funktionen som hvor lang tid den givne tone skal spilles
        buzzer.duty(0)
        led3 .on()
        alarmdelay1 = ticks_ms()
        if ticks_ms() - alarmdelay2 > threshhold3:
            buzzer.duty(512)
            buzzer.freq(1319)#e6
            alarmdelay2 = ticks_ms()
            if ticks_ms() - alarmdelay3 > threshold1sec:
                buzzer.duty(0)
                led3.off()
                alarmdelay3 = ticks_ms()

# Funktion til at tjekke om MPU er stoppet eller standser
def is_stopped(accel_data, threshold=0.05):
    ax = abs(accel_data['acceleration x']) / 16384  # Normaliseret til g
    ay = abs(accel_data['acceleration y']) / 16384
    az = abs(accel_data['acceleration z']) / 16384
    # Hvis ændringerne er meget små, antages det at den er stoppet
    return ax



# handler er funktionen man kalder tilbage til fra thingsboard
def handler(req_id, method, params):
    """handler callback to recieve RPC from server """
     # handler signature is callback(req_id, method, params)
    print(f'Response {req_id}: {method}, params {params}')
    print(params, "params type:", type(params))
    try:
        # check if the method is "toggle_led1" (needs to be configured on thingsboard dashboard)
        if method == "toggle_lås":
            # check if the value is "lås on"
            if params == True:
                print("lås on")
                lås.on()
            else:
                print("lås off")
                lås.off()
        if method == "toggle_led2":
            # check if the value is is "led1 on"
            if params == True:
                print("led2 on")
                led2.off()
            else:
                print("led2 off")
                led2.on()
        if method == "arm alarm":
            # check if the value is is "led1 on"
            if params == True:
                print("alarm armed")
                alarmready = True

            else:
                print("alarm not armed")
                
        if method == "toggle_alarm":
            if params == True:
                print("buzzer on")
                alarm()
            else:
                print("buzzer off")
    except TypeError as e:
        print(e)

# see more about ThingsBoard RPC at the documentation:
# https://thingsboard.io/docs/user-guide/rpc/
        
# See examples for more authentication options
# https://github.com/thingsboard/thingsboard-micropython-client-sdk/
client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token = secrets.ACCESS_TOKEN)


# Connecting to ThingsBoard
client.connect()
print("connected to thingsboard, starting to send and receive data")
while True:
    
    lat1 = gps.get_latitude()
    lon1 = gps.get_longitude()
    
    if ticks_ms() - locationdelay > locationthreshold:
        lat2 = gps.get_latitude()
        lon2 = gps.get_longitude()
        locationdelay = ticks_ms()
    
    if ticks_ms() - alarmarming > alarmthreshold and lat2 == lat1 and lon2 == lon1 and alarmready == False:
        alarmready = True
        alarmarming = ticks_ms()
    
    
    knap1val1 = knap1.value()
    sleep(0.01)
    knap1val2 = knap1.value()
    
    if knap1val1 == 0 and knap1val2 == 1:
        blink1()
    
    knap2val1 = knap2.value()
    sleep(0.01)
    knap2val2 = knap2.value()
    
    if knap2val1 == 0 and knap2val2 == 1:
        blink2()
    
    lat = gps.get_latitude()
    lon = gps.get_longitude()
    course = gps.get_course()
    
    if låsledning == 0:
        alarm()
    # printer hele dictionary som returneres fra get_values metoden
    values = imu.get_values()
    #print(values)
    #print(is_stopped(values))
    kmh = (is_stopped(values) * 9.81) *3.6
    #print(kmh)

    if lat1 != lat2 and lon1 != lon2 and alarmready == True:
        alarm()
    temperature = temp.get_temperature()
    #####
    adc_val = adc.read_adc()
    v = adc.read_voltage()
    batt_percentage = formel_batt(adc_val)
    ####
    if ticks_ms() - displayupdate > threshhold4:
        lcd.clear()
        lcd.move_to(0,0)
        lcd.putstr(f"batteri %: {batt_percentage}")
        lcd.move_to(0,1)
        lcd.putstr(f"km/t: {round(kmh)}")
        lcd.move_to(0,2)
        lcd.putstr(f"lat: {round(lat)}")
        lcd.move_to(11,2)
        lcd.putstr(f"lon: {round(lon)}")
        lcd.move_to(0,3)
        lcd.putstr(f"retning: {round(course)}")
        lcd.move_to(10,1)
        lcd.putstr(f"temp: {round(temperature)}")
        displayupdate = ticks_ms()
    
    if is_stopped(values) < 0.5:
        led1.on()
    else:
        led1.off()
    
    try:

        if gc.mem_free() < 2000:
            gc.collect()
        
        telemetry = {
        "KmH": kmh,
        "lat": lat,
        "lon": lon,
        "retning": course,
        "temperature": temperature,
        "batteri procent": batt_percentage
        }
        # uncomment for sending telemetry from device to server       
        if ticks_ms() - telemetridelay > threshhold5:
        
            client.send_telemetry(telemetry)
            telemetridelay = ticks_ms()
        
        if ticks_ms() - handlerstart > threshhold: #non blocking delay
            #callback to get server RPC requests
            client.set_server_side_rpc_request_handler(handler) 
            
            # Checking for incoming subscriptions or RPC call requests (non-blocking)
            client.check_msg()
            print("tb")
            handlerstart = ticks_ms()
        #sleep(3)
    except KeyboardInterrupt:
        print("Disconnected!")
        # Disconnecting from ThingsBoard
        client.disconnect()
        exit()




