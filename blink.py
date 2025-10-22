from machine import Pin
from machine import I2C
from machine import lightsleep
from machine import freq
from time import sleep
import gc

# -––––––––––––––––––––––––––––––––––––––––––––––– Constants
I2C_SDA_PIN_INDEX = 0
I2C_SDA_PIN = Pin(I2C_SDA_PIN_INDEX) # Pin labeled 1 on Pico is PIN_0
I2C_SCL_PIN_INDEX = 1
I2C_SCL_PIN = Pin(I2C_SCL_PIN_INDEX) # Pin labeled 2 on Pico is PIN_1
I2C_SCL_PIN = Pin(I2C_SCL_PIN_INDEX) # Pin labeled 2 on Pico is PIN_1

LED_PIN = Pin(25, Pin.OUT)

ACTIVATE_BTN_PIN_INDEX = 4
ACTIVATE_BTN_PIN = Pin(ACTIVATE_BTN_PIN_INDEX, Pin.IN, Pin.PULL_DOWN) # Pin labeled 4 on Pico is PIN_3

NORMAL_MC_FREQ = 133_000_000
DEEP_SLEEP_MC_FREQ = 12_000_000

MEASUREMENTS_PER_SECOND = 100 # Measurements taken per second

LUNA_SLAVE_ADDRESS = 0x10 # Slave address of I2C frequency
LUNA_SEND_DATA = bytearray({ 0x5A, 0x05, 0x00, 0x01, 0x60 }) # Default I2C Send data

# -––––––––––––––––––––––––––––––––––––––––––––––– I2C
i2c_handler = None

# Startup I2C
def init_i2c():
    global i2c_handler
    i2c_handler = I2C(0, sda=I2C_SDA_PIN, scl=I2C_SCL_PIN, freq=400000)
    print(f"I2C Activated")
    
# Stop I2C
def deinit_i2c():
    global i2c_handler
    i2c_handler = None
    gc.collect() # collect garbage
    print(f"I2C Deactivated")
    
# -––––––––––––––––––––––––––––––––––––––––––––––– I2C

# -––––––––––––––––––––––––––––––––––––––––––––––– Luna Sensor
def power_sensor(turn_on: bool):
    if turn_on:
        print("Luna Sensor Activated")
    else:
        print("Luna Sensor Deactivated")
# -––––––––––––––––––––––––––––––––––––––––––––––– Luna Sensor

# -––––––––––––––––––––––––––––––––––––––––––––––– Activate Button
def _noop_irq(pin):
    # breaks the deep sleep of MC (called by button)
    pass

ACTIVATE_BTN_PIN.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=_noop_irq)

def activate_btn_pressed():
    # global ACTIVATE_BTN_PIN
    return ACTIVATE_BTN_PIN.value() == 1
# -––––––––––––––––––––––––––––––––––––––––––––––– Activate Button

# -––––––––––––––––––––––––––––––––––––––––––––––– Power ON
def run_on_cycle():
    global i2c_handler
    global power_is_on
    global ACTIVATE_BTN_PIN
    global LED_PIN
    
    init_i2c()
    power_sensor(True)
    power_is_on = True
    LED_PIN.on()
    print("\n--------------\nButton Released")
    
    try:
        while activate_btn_pressed():
            if not i2c_handler:
                break
            
            i2c_handler.writeto(LUNA_SLAVE_ADDRESS, LUNA_SEND_DATA)
            data = i2c_handler.readfrom(LUNA_SLAVE_ADDRESS, 9)
            
            # combine numbers for hex (0x_ _)
            distance = data[2] + data[3] * 256
            # Dist goes from 20cm to 900 cm
            # Under 20cm is unreliable
            # Dist 0 spans happen randomly
            #
            sig_strength = data[4] + data[5] * 256
            # Depends on material surface
            #
            temperature = (data[6] + data[7] * 256) / 8.0 - 256 # convert to F
            # 47 is avg and safe
            #
            print(f"\rDistance: {distance}, Signal: {sig_strength}, Temp: {temperature}", end="")
            
            sleep(1 / float(MEASUREMENTS_PER_SECOND)) # sleep 1sec
    finally:
        deinit_i2c()
        power_sensor(False)
        power_is_on = False
        LED_PIN.off()
        print("Button Released\n--------------\n")
    
# -––––––––––––––––––––––––––––––––––––––––––––––– Power ON

# -––––––––––––––––––––––––––––––––––––––––––––––– Power OFF
def idle_sleep_forever():
    try:
        freq(DEEP_SLEEP_MC_FREQ)
    except Exception:
        pass
    
    # put device into sleep mode
    lightsleep()  # interrupted by IRQ from button
    
    # sleep was interrupted -> button pressed
    try:
        freq(NORMAL_MC_FREQ)
    except Exception:
        pass
# -––––––––––––––––––––––––––––––––––––––––––––––– Power OFF

# -––––––––––––––––––––––––––––––––––––––––––––––– PROGRAM
power_is_on = False
def main():
    global power_is_on
    global LED_PIN
    
    # Start idled
    power_sensor(False)
    deinit_i2c()
    LED_PIN.off()
    try:
        while True:
            # deep idle sleep
            # TODO - Include this sleep function
            # idle_sleep_forever()
            
            if activate_btn_pressed(): # button pressed -> activate
                # stalls code until press stopped
                run_on_cycle()
                
                # no longer pressing

    except KeyboardInterrupt:
        # Program ended forcefully
        power_sensor(False)
        deinit_i2c()
        LED_PIN.off()

main()
# -––––––––––––––––––––––––––––––––––––––––––––––– PROGRAM