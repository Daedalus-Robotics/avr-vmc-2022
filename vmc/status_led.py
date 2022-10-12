# noinspection PyPackageRequirements
import board
import neopixel_spi


class StatusStrip:
    def __init__(self, led_count: int) -> None:
        self.led_count = led_count

        self.spi = board.SPI()
        self.pixels = neopixel_spi.NeoPixel_SPI(self.spi, led_count)

        self.status_leds = {}

    def get_status_led(self, led_num: int):
        if not led_num in self.status_leds:
            self.status_leds[led_num] = StatusLed(self, led_num)
        return self.status_leds.get(led_num, None)


class StatusLed:
    def __init__(self, status_strip: StatusStrip, led_num: int) -> None:
        self.strip = status_strip
        self.led_num = led_num

        self.color = (0, 0, 0)

    def set_color(self, color: tuple[int, int, int]) -> None:
        self.color = color

        self.strip.pixels[self.led_num] = color
        self.strip.pixels.show()

    def set_color_rgb(self, r: int, g: int, b: int) -> None:
        self.set_color((r, g, b))
