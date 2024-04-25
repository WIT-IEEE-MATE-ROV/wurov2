from pca import PCA9685


class RoboArm:
    def __init__(self, pca: PCA9685, pca_slot: int):
        self.__pca = pca
        self.__pca_slot = pca_slot
        self.__previous_state = 0


    def open(self):
        if 1 != self.__previous_state:
            self.__pca.set_us(self.__pca_slot, [1800])
            self.__previous_state = 1


    def close(self):
        if 2 != self.__previous_state:
            self.__pca.set_us(self.__pca_slot, [1300])
            self.__previous_state = 2


    def neutral(self):
        if 3 != self.__previous_state:
            self.__pca.set_us(self.__pca_slot, [1500])
            self.__previous_state = 3
