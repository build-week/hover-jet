//%deps(bmp388)


#include "third_party/bmp388/Adafruit_BMP3XX.hh"
#include <iostream>
#include <unistd.h> // usleep
#include "third_party/bmp388/bmp3_defs.h"

int main() {
	Adafruit_BMP3XX bmp = Adafruit_BMP3XX("/dev/i2c-1");
	if (!bmp.begin()) {
		std::cout << "Failed to init BMP388 sensor" << std::endl;
		return 1;
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

	while (1) {
		if (!bmp.performReading()) {
			std::cout << "Failed to perform reading" << std::endl;
		}
		std::cout << "Temp: " << bmp.temperature << " Pressure: " << bmp.pressure/100.0 << " Approx. Altitude: " << bmp.readAltitude(bmp.SEALEVELPRESSURE_HPA) << " m" << std::endl;
		usleep(1000*1000);
	}

}
