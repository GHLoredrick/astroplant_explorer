#!/usr/bin/python
# https://raspberrytips.nl/i2c-lcd-scherm/

import smbus
import time
import datetime
import RPi.GPIO as GPIO


I2C_ADDR  = 0x38 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

LCD_BACKLIGHT  = 0x08  # On 0X08 / Off 0x00

ENABLE = 0b00000100 # Enable bit

E_PULSE = 0.0005
E_DELAY = 0.0005

#APOS constants
MAX_SCREENS = 2 #total of scipted screens
APOS_VERSION = "APOS v1.0.2v"


#GPIO ports of actuators
LED_RED = 20
LED_WHITE = 19
LED_GREEN = 21

bus = smbus.SMBus(1) # Rev 2 Pi uses 1
prev_input = 0 #button mem
current_menu_screen = 0#current menu item

def lcd_init():
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
  
def button_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(5, GPIO.IN)
    

def lcd_byte(bits, mode):

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def home_screen():
    now = datetime.datetime.now()
    lcd_string("Astroplant",LCD_LINE_1)
    lcd_string( str(now.hour) + ":" + str(now.minute) + ":" + str(now.second) ,LCD_LINE_2)

def serial_screen():
    lcd_string("Board serialnr.",LCD_LINE_1)
    lcd_string("APEK-002",LCD_LINE_2)

def version_screen():
    lcd_string("OS version",LCD_LINE_1)
    lcd_string(APOS_VERSION,LCD_LINE_2)

def reset_screens():
    lcd_string("", LCD_LINE_1)
    lcd_string("", LCD_LINE_2)

def button_listener():
  input = GPIO.input(5)
  global current_menu_screen
  
  if (input) :
      print ("Button1 pressed")
      current_menu_screen += 1
      
  if current_menu_screen > MAX_SCREENS :
      current_menu_screen = 0

def led_init():

    GPIO.setwarnings(False)
    GPIO.setup(LED_RED, GPIO.OUT)
    GPIO.setup(LED_GREEN, GPIO.OUT)
    GPIO.setup(LED_WHITE, GPIO.OUT)

def led_test():
    reset_screens()
    time.sleep(1)
    lcd_string("TEST", LCD_LINE_1)
    lcd_string("ALL LEDS ON", LCD_LINE_2)
    time.sleep(1)
    print("LED on")
    GPIO.output(LED_RED, GPIO.HIGH)
    GPIO.output(LED_GREEN, GPIO.HIGH)
    GPIO.output(LED_WHITE, GPIO.HIGH)
    time.sleep(1)
    lcd_string("ALL LEDS OFF", LCD_LINE_2)
    print("LED off")
    GPIO.output(LED_GREEN, GPIO.LOW)
    GPIO.output(LED_RED, GPIO.LOW)
    GPIO.output(LED_WHITE, GPIO.LOW)
    time.sleep(1)
    lcd_string("TEST ENDED", LCD_LINE_1)
    lcd_string("GOTO HOME", LCD_LINE_2)
    time.sleep(1)

def main():

  lcd_init()
  button_init()
  led_init()
  print ("init succes,start boottest...")
  led_test()

  while True:
    #start with normalmode astroplant+time
   if current_menu_screen == 0 :  
        home_screen()
   
   if current_menu_screen == 1 :
       version_screen()
    
   if current_menu_screen == 2 :
       serial_screen()
   
   button_listener()
   
    
if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

