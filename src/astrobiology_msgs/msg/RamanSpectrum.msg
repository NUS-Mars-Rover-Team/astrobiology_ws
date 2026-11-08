# One scan from the B&W Tek BTC100 Raman spectrometer (raman_spectrometer/raman_node).
#
# All arrays have the same length N = detector pixels (2048). Index i is detector pixel i, so
# plotting counts against the array index gives the raw pixel axis used for calibration.

std_msgs/Header header

# Raman shift in cm^-1 for each pixel: 1e7 / laser_nm - 1e7 / wavelength.
# Negative below the laser line; the 550 nm longpass filter blocks everything under ~600 cm^-1.
float32[] raman_shift

# Wavelength in nm for each pixel (cubic calibration polynomial)
float32[] wavelengths

# Detector counts, minus the stored dark scan when dark_subtracted is true
float32[] counts
bool dark_subtracted

float32 laser_nm            # excitation wavelength used for raman_shift
uint32 integration_ms
uint32 averages
uint32 raw_max              # largest count before dark subtraction; >= 65000 means saturated
uint32 saturated_pixels     # pixels at or above 65000
