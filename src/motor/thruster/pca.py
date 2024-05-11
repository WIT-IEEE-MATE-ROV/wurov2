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


class PCA9685:
    '''Python Class to controll a PCA9685 over an I2C bus.
        
    	Allows for more fine tune control of each PWM signal coming from one of 16 channels.
   
        Referenced this documentation -> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
    	
        This is a good reference if you need to see what each register for the PCA dose.

    '''


    def __init__(self, i2c_address: int, frequency_hz: float, measured_frequency_hz=None, osc_frequency_mhz=None,
                 bus_num=1):
        '''
        Parameters
        ----------

        i2c_address : int
            The unique address of the PCA9585 (default is 0x40)

        frequency_hz : int
            output frequenct of the PWM signal 

        measured_frequency : float
            measured frequency of PCAs signals to adjust for accuracy
        
        '''

        self.__i2c_address = i2c_address
        self.__i2c_bus = SMBus(bus_num)

        if osc_frequency_mhz is not None:
            self.__osc_frequency = osc_frequency_mhz
        else:
            self.__osc_frequency = 25.

        if measured_frequency_hz is not None:
            self.__measured_frequency = measured_frequency_hz
        else:
            self.__measured_frequency = frequency_hz

        self.__frequency_hz = frequency_hz


    def __del__(self):
        self.close()


    @property
    def i2c_bus(self):
        return self.__i2c_bus


    @property
    def i2c_address(self):
        return self.__i2c_address


    @property
    def measured_frequency_hz(self):
        return self.__measured_frequency


    @measured_frequency_hz.setter
    def measured_frequency_hz(self, hz: float):
        self.__measured_frequency = hz


    def close(self):
        try:
            if self.__i2c_bus is not None:
                self.__i2c_bus.close()
        except AttributeError:
            print('No I2C bus object was created so it wasn\'t closed')


    def restart(self):
        '''
        Restarts all the previously active PWM channels
            
        sets the RESTART bit (MODE1 bit 7) to logic 1 at end of PWM refresh cycle
        The contents of each PWM register are held valid when the clock is off.
        
        '''

        mode_1 = self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_MODE_1)
        mode_1 |= PCA_M1_RESTART
        self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)


    def software_reset(self):
        '''
        Allows for the PCA to be reset to the inital 'power up state' without turning the PCA on and off
        '''

        self.__i2c_bus.write_byte(0x00, 6)


    def get_mode_1(self):
        '''
        Returns access to Mode1 register
        '''

        return self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_MODE_1)


    def get_prescale(self):
        '''
        Returns the frequency at which the PCA outputs modulate 

        The max PWM frequency is 1526Hz
        The min PWM frequency is 24Hz

        '''

        return self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_PRE_SCALE)


    def setup(self):
        self.restart()
        time.sleep(0.01)

        mode_1 = PCA_M1_AUTO_INC | 1
        self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)
        self.set_pwm_frequency(self.__frequency_hz)


    def set_sleep(self, sleep_on: bool):
        '''
        Puts the PCA into low power mode by turning off its oscillator
        NOTE: outputs cannot be turned on or off

        Parameters
        ----------

            sleep_on : bool
                Sets the sleep state on or 
            
                True -> on
                False -> off

        '''
        mode_1 = self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_MODE_1)
        sleep_state = mode_1 & PCA_M1_SLEEP
        print("Sleep State: ", sleep_state)
        if sleep_on and not sleep_state:
            mode_1 |= PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)
        elif not sleep_on and sleep_state:
            mode_1 &= ~PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)


    def use_extclk(self):
        '''
        Allows for the use of an external clock on the PCA.   
        This function will put the PCA to sleep to allow for external clocks to be used.

    	Inorder to rest the PCA to the default clock the PCA needs to be software_reset().
    	If this doesn't work try power cycling the PCA.

        '''

        mode_1 = self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_MODE_1)
        extclk_state = mode_1 & PCA_M1_EXTCLK
        if not extclk_state:
            sleep_state = mode_1 & PCA_M1_SLEEP
            if not sleep_state:
                mode_1 |= PCA_M1_SLEEP
                self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)
            mode_1 |= PCA_M1_EXTCLK
            self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)
            if not sleep_state:
                mode_1 &= ~PCA_M1_SLEEP
                self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)


    def set_pwm_frequency(self, pwm_freq_hz: float):
        '''Sets the frequency of the output PWM signals

            The max PWM frequency is 1526Hz
            The min PWM frequency is 24Hz

        '''
        # Clamp pwm frequency to max and min values
        pwm_freq_hz = max(min(pwm_freq_hz, 1526), 26)

        mode_1 = self.__i2c_bus.read_byte_data(self.__i2c_address, PCA_REG_MODE_1)
        sleep_state = mode_1 & PCA_M1_SLEEP
        if not sleep_state:
            mode_1 |= PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)

        prescale_val = round((self.__osc_frequency * 1000000.0) / (4096.0 * pwm_freq_hz)) - 1
        print("prescale_val: ", prescale_val)
        self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_PRE_SCALE, prescale_val)

        if not sleep_state:
            mode_1 &= ~PCA_M1_SLEEP
            self.__i2c_bus.write_byte_data(self.__i2c_address, PCA_REG_MODE_1, mode_1)

        self.__frequency_hz = pwm_freq_hz


    def set_counts(self, control_num: int, on_counts: int, off_counts: int):
        ctrl_reg = (control_num * 4) + 6
        ctrl_data = [0] * 4

        ctrl_data[0] = on_counts & 0xFF
        ctrl_data[1] = (on_counts >> 8) & 0x0F
        ctrl_data[2] = off_counts & 0xFF
        ctrl_data[3] = (off_counts >> 8) & 0x0F

        self.__i2c_bus.write_i2c_block_data(self.__i2c_address, ctrl_reg, ctrl_data)


    def get_counts(self, control_num) -> Tuple[int, int]:
        ctrl_reg = (control_num * 4) + 6
        ctrl_data = self.__i2c_bus.read_i2c_block_data(self.__i2c_address, ctrl_reg, 4)
        on_counts = ctrl_data[0] + ((ctrl_data[1] & 0xFF) << 8)
        off_counts = ctrl_data[2] + ((ctrl_data[3] & 0xFF) << 8)

        return on_counts, off_counts


    def set_duty_cycle(self, control_num: int, duty_cycle: float):
        pwm_period_us = (1.0 / self.__measured_frequency) * 1_000_000.0
        us_on = (((duty_cycle + 1.0) / 2.0) * 800.0) + 1100.0
        us_on_ratio = us_on / pwm_period_us

        off_counts = round(us_on_ratio * 4096.0)
        on_counts = 0

        self.set_counts(control_num, on_counts, off_counts)

    # TODO: Support `us` being a single number
    def set_us(self, control_num: int, us: list):
        end_ctrl = len(us) + control_num
        if end_ctrl > 15:
            raise IndexError(f"There only 16 channels on the PCA. Attempted to access channel {end_ctrl}")

        ctrl_len = end_ctrl - control_num
        pwm_period_us = (1.0 / self.__measured_frequency) * 1_000_000.0
        ctrl_data = [0] * (ctrl_len * 4)

        for i in range(ctrl_len):
            us_on = us[i]
            us_on_ratio = us_on / pwm_period_us
            off_counts = int(round(us_on_ratio * 4096.0))
            on_counts = 0
            ctrl_reg = i * 4

            ctrl_data[ctrl_reg] = on_counts & 0xFF
            ctrl_data[ctrl_reg + 1] = (on_counts >> 8) & 0x0F
            ctrl_data[ctrl_reg + 2] = off_counts & 0xFF
            ctrl_data[ctrl_reg + 3] = (off_counts >> 8) & 0x0F

        # print(ctrl_data)

        start_reg = control_num * 4 + 6
        end_reg = end_ctrl * 4 + 6

        reg_length = end_reg - start_reg
        # print('reg_length: ', reg_length)

        if end_reg > 69:
            raise OverflowError("[ERROR] PCA9685 only has 16 control slots")

        self.__i2c_bus.write_i2c_block_data(self.__i2c_address, start_reg, ctrl_data[0:32])
        if reg_length > 32:
            ctrl_data_2 = ctrl_data[32:64]
            ctrl_data_2_start = start_reg + 32
            self.__i2c_bus.write_i2c_block_data(self.__i2c_address, ctrl_data_2_start, ctrl_data_2)


    def set_duty_cycles(self, start_ctrl: int, duty_cycles):
        end_ctrl = len(duty_cycles) + start_ctrl
        if end_ctrl > 15:
            raise IndexError(f"There only 16 channels on the PCA. Attempted to access channel {end_ctrl}")

        ctrl_len = end_ctrl - start_ctrl
        pwm_period_us = (1.0 / self.__measured_frequency) * 1_000_000.0
        ctrl_data = [0] * (ctrl_len * 4)

        for i in range(ctrl_len):
            us_on = (((duty_cycles[i] + 1.0) / 2.0) * 800.0) + 1100.0
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
        # print('reg_length: ', reg_length)

        if end_reg > 69:
            raise OverflowError("[ERROR] PCA9685 only has 16 control slots")

        self.__i2c_bus.write_i2c_block_data(self.__i2c_address, start_reg, ctrl_data[0:32])
        if reg_length > 32:
            ctrl_data_2 = ctrl_data[32:64]
            ctrl_data_2_start = start_reg + 32
            self.__i2c_bus.write_i2c_block_data(self.__i2c_address, ctrl_data_2_start, ctrl_data_2)


if __name__ == '__main__':
    # GRIPPER TEST
    # p = PCA9685(0x41, 100)
    # p.software_reset()
    # p.setup()
    # p.set_sleep(False)
    # try:
    #     while True:
    #         p.set_us(10, [1800])
    #         time.sleep(3)
    #         p.set_us(10, [1200])
    #         time.sleep(2)
    #         # p.set_us(10, [1500])
    #         # time.sleep(2)
    # finally:
    #     p.close()

    # exit()

    # # LIGHT TEST
    # p = PCA9685(0x41, 100)
    # p.software_reset()
    # p.setup()
    # p.set_sleep(False)
    # try:
    #     while True:
    #         p.set_us(8, [1900])
    #         time.sleep(0.5)
    #         p.set_us(8, [1100])
    #         time.sleep(0.5)
    # finally:
    #     p.close()

    # exit()

    # THRUSTER TEST
    p1 = PCA9685(0x40, 100, 105.6)
    p1.setup()
    # p1.set_sleep(False)
    # exit()


     # Horizontal thruster PCA slots
    __FLH_ID = 0
    __FRH_ID = 5
    __BLH_ID = 1
    __BRH_ID = 6
    # Vertical thruster PCA slots__FLH_ID
    __FLV_ID = 3
    __FRV_ID = 4
    __BLV_ID = 2
    __BRV_ID = 7


    try:
        while True:
            # p1.set_us(8, [1900])
            # time.sleep(0.5)
            # p1.set_us(8, [1100])
            # time.sleep(0.5)
            # on_counts, off_counts = p1.get_counts(0)
            # print(f'Before on_counts = {on_counts}, off_counts = {off_counts}')
            # p.set_duty_cycles(0, [0])
            p1.set_us(__BLV_ID , [1500])
            on_counts, off_counts = p1.get_counts(0)
            print(f' After on_counts = {on_counts}, off_counts = {off_counts}')

            time.sleep(1)

            on_counts, off_counts = p1.get_counts(0)
            print(f'Before on_counts = {on_counts}, off_counts = {off_counts}')
            # p.set_duty_cycles(0, [0])
            p1.set_us(__BRV_ID , [1700])
            on_counts, off_counts = p1.get_counts(0)
            print(f' After on_counts = {on_counts}, off_counts = {off_counts}')

            time.sleep(1)

    finally:
        # p.set_us(4, [1500])
        p.close()
        p1.close()

    # print(p.i2c_bus.read_i2c_block_data(0x40, 0x00, 4))
    # Horizontal thruster PCA slots
    # __FLH_ID = 0
    # __FRH_ID = 5
    # __BLH_ID = 1
    # __BRH_ID = 6
    # Vertical thruster PCA slots__FLH_ID
    # __FLV_ID = 3
    # __FRV_ID = 4
    # __BLV_ID = 2
    # __BRV_ID = 7
