import uos
from machine import UART, Pin
import utime
import ustruct

"""
ESPRESSIF AT Command Set
https://docs.espressif.com/projects/esp-at/en/latest/AT_Command_Set/
"""

print()
print("Machine: \t" + uos.uname()[4])
#print("MicroPython: \t" + uos.uname()[3])

#indicate program started visually
led_onboard = machine.Pin(25, machine.Pin.OUT)
led_onboard.value(0)     # onboard LED OFF/ON for 0.5/1.0 sec
utime.sleep(0.5)
led_onboard.value(1)
utime.sleep(1.0)
led_onboard.value(0)

# I2C address
US_ADDR = 0x62
# Registers
REG_MODE = 0x01
REG_IDMODEL = 0x02
REG_SING_CPT = 0x03
REG_CAL = 0x04
REG_SMOOTH = 0x05

# Create I2C object
i2c = machine.I2C(0, scl=machine.Pin(5), sda=machine.Pin(4))
###############################################################################
# Functions
def reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """   
    # Construct message
    msg = bytearray()
    msg.append(data)  
    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)
    
def reg_read(i2c, addr, nbytes=16):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Check to make sure caller is asking for 16 bytes
    if nbytes != 16:
        return bytearray()
    
    # Request data from I2C address
    data = i2c.readfrom(addr,nbytes)    
    return data

#write mode
reg_write(i2c, US_ADDR, REG_MODE, 1)
#write ID model
reg_write(i2c, US_ADDR, REG_IDMODEL, True)
#write smoothing
reg_write(i2c, US_ADDR, REG_SMOOTH, True)
#read data from US I2C device
data = reg_read(i2c, US_ADDR)
#unpack
# data format is conf (float - 4 bytes), conf_id (float - 4 bytes),
# bounding box (4 x int), ID (1 byte)
conf = ustruct.unpack_from("<f",data,0)[0]
conf_id = ustruct.unpack_from("<f",data,4)[0]
x1 = ustruct.unpack_from("<b",data,8)[0]
y1 = ustruct.unpack_from("<b",data,9)[0]
x2 = ustruct.unpack_from("<b",data,10)[0]
y2 = ustruct.unpack_from("<b",data,11)[0]
id = ustruct.unpack_from("<b",data,12)[0]
confidence = conf * 100.0
print(f'Confidence: {confidence:.2f}')
print(f'ID: {id}')
print(f'BBox: ({x1},{y1}),({x2},{y2})')

uart0 = UART(0, rx=Pin(17), tx=Pin(16), baudrate=115200)
# NOTE that we explicitly set the Tx and Rx pins for use with the UART
# If we do not do this, they WILL default to Pin 0 and Pin 1
# Also note that Rx and Tx are swapped, meaning Pico Tx goes to ESP01 Rx 
# and vice versa.
#print(uart0)

def sendCMD_waitResp(cmd, uart=uart0, timeout=2000):
    print("CMD: " + cmd)
    uart.write(cmd)
    waitResp(uart, timeout)
    print()
    
def waitResp(uart=uart0, timeout=2000):
    prvMills = utime.ticks_ms()
    resp = b""
    while (utime.ticks_ms()-prvMills)<timeout:
        if uart.any():
            resp = b"".join([resp, uart.read(1)])
    print("resp:")
    try:
        print(resp.decode())
    except UnicodeError:
        print(resp)
    
#sendCMD_waitResp('AT\r\n')          #Test AT startup
#sendCMD_waitResp('AT+GMR\r\n')      #Check version information
##sendCMD_waitResp('AT+RESTORE\r\n')  #Restore Factory Default Settings
#sendCMD_waitResp('AT+CWMODE?\r\n')  #Query the Wi-Fi mode
#sendCMD_waitResp('AT+CWMODE=1\r\n') #Set the Wi-Fi mode = Station mode
#sendCMD_waitResp('AT+CWMODE?\r\n')  #Query the Wi-Fi mode again
##sendCMD_waitResp('AT+CWLAP\r\n', timeout=10000) #List available APs
#sendCMD_waitResp('AT+CWJAP="op","Granby18!"\r\n', timeout=5000) #Connect to AP
#utime.sleep(1)
#sendCMD_waitResp('AT+CIFSR\r\n')    #Obtain the Local IP Address
