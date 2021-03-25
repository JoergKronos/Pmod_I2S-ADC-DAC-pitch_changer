# Pmod_I2S-ADC-DAC-pitch
Pitch Changer

//Version 1.0
//  For Arduino IDE 1.8.13
//  ESP-32 / i2s protocol / DMA background transfer
//  Read audio from i2s ADC (DIGILENT Pmod I2S2) into buffer
//  Highpass filter 300Hz /remove rumble
//  Pitch shift
//  Write audio using i2s to DAC (DIGILENT Pmod I2S2)
//Version 2.2
//  Rotary encoder to change the pitch live. Hardware Interrupt and run at 2nd Core.
//Version 3.1
//  Add a small OLED display. 128x32 pixel. I2C interface. Shows Pitch and peak level with memory
