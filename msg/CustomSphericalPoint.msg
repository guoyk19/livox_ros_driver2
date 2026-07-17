# Livox spherical point format.

uint32 offset_time      # Offset from the frame timebase, unit:ns
float32 depth           # Radial distance, unit:m
float32 theta           # Polar angle from the positive Z axis, unit:rad
float32 phi             # Azimuth angle in the XY plane, unit:rad
uint8 reflectivity      # Reflectivity, 0~255
uint8 tag               # Livox point tag
uint8 line              # Laser channel number
