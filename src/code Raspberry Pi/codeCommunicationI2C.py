import smbus as smbus
import time

ConvertStringToByte(src):
    


data = 10 
addr = 0x08
bus = smbus.SMBus(1)


while True:
    block = bus.read_i2c_block_data(addr,0,20)
    n = "".join(map(chr,block))
    n = n.split(";")
    print(n)
    
    bus.write_byte_data(addr,0,data)
    
    time.sleep(1)
    
    