
from RPLCD.i2c import CharLCD
from time import sleep

lcd = CharLCD('PCF8574', 0x27, cols=16, rows=2, charmap='A00')

def mostrar_ip_puerto(ip, puerto):
    lcd.clear()
    lcd.write_string(f'Roldac Port:{puerto}')
    lcd.crlf()
    lcd.write_string(f'IP:{ip}')
    sleep(2)
    