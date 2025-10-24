from machine import Pin
from machine import I2C
from machine import lightsleep
from machine import freq
from time import sleep
import time
import gc

# -––––––––––––––––––––––––––––––––––––––––––––––– Constants
I2C_SDA_PIN_INDEX = 0
I2C_SDA_PIN = Pin(I2C_SDA_PIN_INDEX) # Pin labeled 1 on Pico is PIN_0
I2C_SCL_PIN_INDEX = 1
I2C_SCL_PIN = Pin(I2C_SCL_PIN_INDEX) # Pin labeled 2 on Pico is PIN_1
I2C_SCL_PIN = Pin(I2C_SCL_PIN_INDEX) # Pin labeled 2 on Pico is PIN_1

MOTOR_CTRL_PIN_INDEX = 3
MOTOR_CTRL_PIN = Pin(MOTOR_CTRL_PIN_INDEX, Pin.OUT) # Pin labeled 2 on Pico is PIN_1
MOTOR_CLOSEST_DELAY = 50
MOTOR_FARTHEST_DELAY = 1500

LED_PIN = Pin(25, Pin.OUT)

ACTIVATE_BTN_PIN_INDEX = 4
ACTIVATE_BTN_PIN = Pin(ACTIVATE_BTN_PIN_INDEX, Pin.IN, Pin.PULL_DOWN) # Pin labeled 4 on Pico is PIN_3

NORMAL_MC_FREQ = 125_000_000
DEEP_SLEEP_MC_FREQ = 48_000_000

MEASUREMENTS_PER_SECOND = 100 # Measurements taken per second

LUNA_SLAVE_ADDRESS = 0x10 # Slave address of I2C frequency
LUNA_SEND_DATA = bytearray([ 0x5A, 0x05, 0x00, 0x01, 0x60 ]) # Default I2C Send data

class SYSTEM_STATE:
    NONE = -1
    SLEEPING = 0
    ACTIVE = 1
    
system_state = SYSTEM_STATE.NONE

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
    if not i2c_handler: return
    
    if turn_on:
        print("Luna Sensor Activated")
    else:
        print("Luna Sensor Deactivated")

    enable_hex = 0x01 if turn_on else 0x00
    freq = 0x01 if turn_on else 0x00
    cs = 0x91 if turn_on else 0x8F
    byte_cmd = bytearray([0x5A, 0x35, enable_hex, freq, 0x00, 0x00, 0x00, cs])
    
    i2c_handler.writeto(LUNA_SLAVE_ADDRESS, byte_cmd)
# -––––––––––––––––––––––––––––––––––––––––––––––– Luna Sensor

# -––––––––––––––––––––––––––––––––––––––––––––––– Activate Button
def _noop_irq(pin):
    # breaks the lightsleep of MC (called by button)
    pass

ACTIVATE_BTN_PIN.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=_noop_irq)

# Checks for valid activation trigger
def wait_for_activate():
    return ACTIVATE_BTN_PIN.value() == 1

# Checks for valid deactivation trigger
def wait_for_deactivate():
    return ACTIVATE_BTN_PIN.value() == 0
# -––––––––––––––––––––––––––––––––––––––––––––––– Activate Button

# -––––––––––––––––––––––––––––––––––––––––––––––– Power ON
motor_toggle_time = 0
motor_control = 1
def run_on_cycle():
    global i2c_handler
    global system_state
    global motor_toggle_time
    global motor_control
    global ACTIVATE_BTN_PIN
    global LED_PIN
    
    init_i2c()
    power_sensor(True)
    system_state = SYSTEM_STATE.ACTIVE
    LED_PIN.on()
    print("\n--------------\nButton Pressed")
    
    try:
        while True:
            if not i2c_handler: break
            
            # button released
            result = wait_for_deactivate()
            if result:
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
            print(f"Distance: {distance}, Signal: {sig_strength}, Temp: {temperature}", end="\n")
            
            motor_control = max(0, distance / 700.0)
            
            sleep(1 / float(MEASUREMENTS_PER_SECOND)) # sleep 1sec
            
            # Motor
            if time.ticks_ms() > motor_toggle_time:
                MOTOR_CTRL_PIN.toggle()
                
                motor_toggle_time = time.ticks_ms() + MOTOR_CLOSEST_DELAY + (MOTOR_FARTHEST_DELAY - MOTOR_CLOSEST_DELAY) * motor_control
    finally:
        print("")
        power_sensor(False)
        deinit_i2c()
        system_state = SYSTEM_STATE.NONE
        LED_PIN.off()
        MOTOR_CTRL_PIN.off()
        print("Button Released\n--------------\n")
    
# -––––––––––––––––––––––––––––––––––––––––––––––– Power ON

# -––––––––––––––––––––––––––––––––––––––––––––––– Power OFF
def idle_sleep_forever():
    global system_state
    
    if system_state == SYSTEM_STATE.SLEEPING: return
    system_state = SYSTEM_STATE.SLEEPING
    
    freq(DEEP_SLEEP_MC_FREQ)
    
    # put device into sleep mode
    print("\rChecking for sleep wakeup...", end="\n")
    lightsleep(60_000)  # interrupted by IRQ from button
    
    # sleep was interrupted -> button pressed
    freq(NORMAL_MC_FREQ)
    
    system_state = SYSTEM_STATE.NONE
# -––––––––––––––––––––––––––––––––––––––––––––––– Power OFF

# -––––––––––––––––––––––––––––––––––––––––––––––– PROGRAM
def main():
    global system_state
    global LED_PIN
    
    # Start idled
    init_i2c()
    power_sensor(False)
    deinit_i2c()
    LED_PIN.off()
    try:
        while True:
            # deep idle sleep
            idle_sleep_forever()
            
            valid_press = wait_for_activate()
            if valid_press:
                # press started
                
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