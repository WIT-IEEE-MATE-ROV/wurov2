from pca import PCA9685


class Headlamp:
    def __init__(self, pca: PCA9685, pca_slot: int):
        self.__pca = pca
        self.__pca_slot = pca_slot
        self.__previous_us = 0

    def set_brightness(self, b: float):
        # Clamp brightness arg to [0, 1]
        b = max(min(b, 1), 0)
        # Map brightness arg to us
        b = int((b * 800.0) + 1100.0)

        # Set brightness to b only if it's a new value
        if not b == self.__previous_us:
            self.__pca.set_us(self.__pca_slot, [b])
            self.__previous_us = b
