# Import the ADS1x15 module.
import Adafruit_ADS1x15
# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
# Read the specified ADC channel using the previously set gain value.
values = (adc.read_adc(2, gain=GAIN))*23.48/32768
print(values) 
