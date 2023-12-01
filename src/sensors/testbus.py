from smbus2 import SMBus



with SMBus(1) as bus:
    bus.write_byte(0x00, 6)

    read_1 = bus.read_byte_data(0x40, 0)
    print(read_1)
    write_1 = bus.write_byte_data(0x40, 0, 49)
    print(write_1)
    read_2 = bus.read_byte_data(0x40, 0)
    print(read_2)

    bus.write_byte_data(0x40, 0x06, 0)
    bus.write_byte_data(0x40, 0x07, 0)
    bus.write_byte_data(0x40, 0x08, 0)
    bus.write_byte_data(0x40, 0x09, 8)


    read_2 = bus.read_i2c_block_data(0x40, 0x00, 4)
    print(read_2)
    

