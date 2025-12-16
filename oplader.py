import requests
from time import sleep
from machine import Pin

relæ = Pin(14,Pin.OUT)
led = Pin(26, Pin.OUT)

while True:
    response = requests.get(
        url = 'https://api.energidataservice.dk/dataset/CO2Emis?limit=5')
    result = response.json()
    co2 = result['records'][1]['CO2Emission']
    
    if co2 <= 50:
        relæ.on()
        led.off()

    elif co2 > 50:
        relæ.off()
        led.ob()

    sleep(5)
