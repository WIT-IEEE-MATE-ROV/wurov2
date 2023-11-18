"""

Python code for contolling the PCA9685

"""

import time
from smbus import SMBus
from typing import Tuple
# regster num
PCA_REG_PRE_SCALE = 0xFE
PCA_REG_MODE_1 = 0x00

# map for mode 1
PCA_M1_RESTART = 1 << 7
PCA_M1_EXTCLK = 1 << 6
PCA_M1_AUTO_INC = 1 << 5
PCA_M1_SLEEP = 1 << 4
PCA_CTRL_REG_OFFSET = 0x06


class PCA9685():
    
    def __init__(self, i2c_address:int, frequency_hz:float, measured_frequency = None):
        self.i2c_address = i2c_address
        self.__i2c_bus = SMBus(1)
        self.osc_frequency_Mhz = 25 

        if measured_frequency is not None:
            self.__measured_frequency = measured_frequency
        else:
            self.__measured_frequency = frequency_hz
        
        self.__frequency_hz = frequency_hz


    def __del__(self):
        self.close()


    def  get_bus(self):
        return self.__i2c_bus


    def close(self):
        self.__i2c_bus.close()


    def set_measured_frequency(self, measured_frequency:float):
        self.__measured_frequency = measured_frequency


    def restart(self):
        mode_1 = self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_MODE_1)
        mode_1 |= PCA_M1_RESTART
        self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)

    
    def software_reset(self):
        self.__i2c_bus.write_byte(0x00, 6)  


    def get_mode_1(self):
        return self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_MODE_1)
        
    
    def get_prescale(self):
        return self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_PRE_SCALE)


    def setup(self):
        self.restart()
        time.sleep(0.01)  

        mode_1 = PCA_M1_AUTO_INC | 1
        self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)
        self.set_prescale(100)


    def set_sleep(self, sleep_on:bool):
        mode_1 = self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_MODE_1)
        sleep_state = mode_1 & PCA_M1_SLEEP
        print("Sleep State: ", sleep_state)
        if sleep_on and not sleep_state:
            mode_1 |= PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)
        elif not sleep_on and sleep_state:
            mode_1 &= ~PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)


    def use_extclk(self):
        mode_1 = self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_MODE_1)
        extclk_state = mode_1 & PCA_M1_EXTCLK
        if not extclk_state:
            sleep_state = mode_1 & PCA_M1_SLEEP
            if not sleep_state:
                mode_1 |= PCA_M1_SLEEP
                self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)
            mode_1 |= PCA_M1_EXTCLK
            self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)
            if not sleep_state:
                mode_1 &= ~PCA_M1_SLEEP
                self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)

        
    def set_prescale(self, pwm_freq_hz: float):
        mode_1 = self.__i2c_bus.read_byte_data(self.i2c_address, PCA_REG_MODE_1)
        sleep_state = mode_1 & PCA_M1_SLEEP
        if not sleep_state:
            mode_1 |= PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)

        prescale_val = round((self.osc_frequency_Mhz * 1000000.0) / (4096.0 * pwm_freq_hz)) -1 
        print("prescale_val: ", prescale_val)   
        self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_PRE_SCALE, prescale_val)
        
        if not sleep_state:
            mode_1 &= ~PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.i2c_address, PCA_REG_MODE_1, mode_1)


    def set_counts(self, control_num:int, on_counts:int, off_counts:int):
        ctrl_reg = (control_num * 4) + 6
        ctrl_data = [0]*4

        ctrl_data[0] = on_counts & 0xFF
        ctrl_data[1] = (on_counts >> 8) & 0x0F 
        ctrl_data[2] = off_counts & 0xFF
        ctrl_data[3] = (off_counts >> 8) & 0x0F 

        self.__i2c_bus.write_i2c_block_data(self.i2c_address, ctrl_reg, ctrl_data)

    def get_counts(self, control_num) -> Tuple[int, int]:
        ctrl_reg = (control_num * 4) + 6
        ctrl_data = self.__i2c_bus.read_i2c_block_data(self.i2c_address, ctrl_reg, 4)
        on_counts = ctrl_data[0] + ((ctrl_data[1] & 0xFF) << 8 )
        off_counts = ctrl_data[2] + ((ctrl_data[3] & 0xFF) << 8 )

        return (on_counts, off_counts)


    def set_duty_cycle(self, control_num:int, duty_cycle:float):
        pwm_period_us = (1.0 / self.__measured_frequency) * 1_000_000.0
        us_on = (((duty_cycle + 1.0) / 2.0) * 800.0) +1100.0
        us_on_ratio = us_on / pwm_period_us

        off_counts = round(us_on_ratio * 4096.0)
        on_counts = 0

        self.set_counts(control_num, on_counts, off_counts)


    def set_duty_cycles(self, start_ctrl:int, end_ctrl:int, duty_cycles):
        end_ctrl += 1
        ctrl_len = end_ctrl - start_ctrl
        pwm_period_us = (1.0 / self.__measured_frequency) * 1_000_000.0
        ctrl_data = [0] * (ctrl_len * 4)
        print(ctrl_data)

        for i in range(ctrl_len):
            print(i)
            us_on = (((duty_cycles[i] + 1.0) / 2.0) * 800.0) +1100.0
            us_on_ratio = us_on / pwm_period_us
            off_counts = round(us_on_ratio * 4096.0)
            on_counts = 0
            ctrl_reg = i * 4

            ctrl_data[ctrl_reg] = on_counts & 0xFF
            ctrl_data[ctrl_reg + 1] = (on_counts >> 8) & 0x0F 
            ctrl_data[ctrl_reg + 2] = off_counts & 0xFF
            ctrl_data[ctrl_reg + 3] = (off_counts >> 8) & 0x0F 

        
        print(ctrl_data)



        start_reg = start_ctrl * 4 + 6
        end_reg = end_ctrl * 4 + 6

        reg_length = end_reg - start_reg

        self.__i2c_bus.write_i2c_block_data(self.i2c_address, start_reg, ctrl_data)



    



p = PCA9685(0x40, 100, 103.7)
p.software_reset()
p.setup()
p.set_sleep(False)

p.set_duty_cycles(2, 7, range(6))

# p.set_duty_cycle(0, 0)



print(p.get_counts(0))

print(p.get_bus().read_i2c_block_data(0x40, 0x00, 4))
p.close()
