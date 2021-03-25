// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modifications to run the code at Arduino IDE
// Some thoughts of setting ring buffer length and overlapping
//    Based of [#24] ESP32 - I2S Audio Loopback with MCLK and
//    and [#7] Pitch Shifting - Audio DSP On STM32 (24 Bit / 96 kHz)
//    by YetAnotherElectronicsChannel
// Joerg Rettkowski 22.03.2021

#include "driver/i2s.h"

//error type
esp_err_t err;

//buffers
#define queue  16   // 2+(R+L)  Samples
int rxbuf[queue], txbuf[queue];
float l_in[queue/2], r_in[queue/2];
float l_out[queue/2], r_out[queue/2];

//Pitch
float Shift = 0.7f;


 
// Ring Buffer
// Original values are:
// #define BufSize 2100
// #define Overlap 210
//
// The length of the ring buffer is related to the lowest frequency.
// For human voice this is 300Hz. Therefore the highpass.
// Best will be more than two periods (2x)
// Because of 2 read pointers are in use, lenght need to be doubled (2x)
// Minimum buffer length needed is 2x 2x (44100Hz/300Hz) = 588
// The shift also has an influence to the buffer lenght.
// But best quality is buffer length = 10 * Overlap
#define BufSize 2100

// The overlapping / crossfade length is also related to the lowest frequency
// but also by the pitch shift. We know already the "lenght" of a 300Hz sample:
// 44100Hz/300Hz = 147 samples,  at pitch shift of 1
// The lower the shift the more samples in the overlap are necessary.
// The number of samples need to be divided by the pitch
// Pitch 2  : 147 samples/2   = 74
// Pitch 1  : 147 samples/1   = 147
// Pitch 0.7: 147 samples/0.7 = 210
#define Overlap 210



int Buf[BufSize];         // the ring buffer

int WtrP;                 // write pointer
float Rd_P;               // read pointer
float CrossFade;
float a0, a1, a2, b1, b2, hp_in_z1, hp_in_z2, hp_out_z1, hp_out_z2;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
//Highpass, 44kHz, 300Hz, Q=SQRT(2)/2=0.7071, 6dB gain

int Do_HighPass (int inSample) {
  a0 = 0.9702281038091352f;
  a1 = -1.9404562076182703f;
  a2 = 0.9702281038091352f;
  b1 = -1.9395696618226337f;
  b2 = 0.9413427534139069f;

  float inSampleF = (float)inSample;
  float outSampleF =
      a0 * inSampleF
      + a1 * hp_in_z1
      + a2 * hp_in_z2
      - b1 * hp_out_z1
      - b2 * hp_out_z2;
      
  hp_in_z2 = hp_in_z1;
  hp_in_z1 = inSampleF;
  hp_out_z2 = hp_out_z1;
  hp_out_z1 = outSampleF;

  return (int) outSampleF;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int Do_PitchShift(int Sample) {
  int sum;
  //sum up and do high-pass
  sum=Do_HighPass(Sample);

  //write to ringbuffer
  Buf[WtrP] = sum;

  //read fractional readpointer and generate 0° and 180° read-pointer in integer
  int RdPtr_Int = roundf(Rd_P);
  int RdPtr_Int2 = 0;
  if (RdPtr_Int >= BufSize/2) RdPtr_Int2 = RdPtr_Int - (BufSize/2);
  else RdPtr_Int2 = RdPtr_Int + (BufSize/2);

  //read the two samples...
  float Rd0 = (float) Buf[RdPtr_Int];
  float Rd1 = (float) Buf[RdPtr_Int2];

  //Check if first readpointer starts overlap with write pointer?
  // if yes -> do cross-fade to second read-pointer
  if (Overlap >= (WtrP-RdPtr_Int) && (WtrP-RdPtr_Int) >= 0 && Shift!=1.0f) {
    int rel = WtrP-RdPtr_Int;
    CrossFade = ((float)rel)/(float)Overlap;

  }
  else if (WtrP-RdPtr_Int == 0) {
    CrossFade = 0.0f;
  }


  //Check if second readpointer starts overlap with write pointer?
  // if yes -> do cross-fade to first read-pointer
  if (Overlap >= (WtrP-RdPtr_Int2) && (WtrP-RdPtr_Int2) >= 0 && Shift!=1.0f) {
      int rel = WtrP-RdPtr_Int2;
      CrossFade = 1.0f - ((float)rel)/(float)Overlap;
    }
  else if (WtrP-RdPtr_Int2 == 0) {
    CrossFade = 1.0f;
  }


  //do cross-fading and sum up
  sum = (Rd0*CrossFade + Rd1*(1.0f-CrossFade));

  //increment fractional read-pointer and write-pointer
  Rd_P += Shift;
  WtrP++;
  if (WtrP == BufSize) WtrP = 0;
  if (roundf(Rd_P) >= BufSize) Rd_P = 0.0f;

  return sum;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void setup() {

Serial.begin(115200);
delay (500);

//ring buffer start 0°
  WtrP = 0;
  Rd_P = 0.0f;
  CrossFade = 1.0f;

//------------------------------------------------------------
//enable MCLK on GPIO0
  REG_WRITE(PIN_CTRL, 0xFF0); 
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

//------------------------------------------------------------
  
  i2s_config_t i2s_config = {

        .mode                 = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX  | I2S_MODE_RX), // i2s_mode_t             work mode
        .sample_rate          = 44100,                                                    // int                    sample rate
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,                                // i2s_bits_per_sample_t  bits per sample
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,                               // i2s_channel_fmt_t      channel format
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,                                  // i2s_comm_format_t      communication format
        .intr_alloc_flags     = 0,                                                        // int                    Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info
        .dma_buf_count        = 128,                                                      // int                    DMA Buffer Count
        .dma_buf_len          = queue*2,                                                  // int                    DMA Buffer Length
        .use_apll             = true,                                                     // bool                   using APLL as main clock, enable it to get accurate clock
        .tx_desc_auto_clear   = true,                                                     // bool                   auto clear tx descriptor if there is underflow condition (helps in avoiding noise in case of data unavailability)
        .fixed_mclk           = true                                                      // int                    using fixed MCLK output. If use_apll = true and fixed_mclk > 0, then the clock output for is fixed and equal to the fixed_mclk value
                                             
    };

    err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
       Serial.print("Failed installing driver: ");
       Serial.print(err);
       while (true);
     }
//------------------------------------------------------------

    i2s_pin_config_t pin_config = {
        .bck_io_num   = 26,
        .ws_io_num    = 25,
        .data_out_num = 22,
        .data_in_num  = 19                                                       
    };

   err = i2s_set_pin(I2S_NUM_0, &pin_config);
   if (err != ESP_OK) {
      Serial.print("Failed setting pin: ");
      Serial.print(err);
      while (true);
   }

   Serial.println("I2S driver installed.");
  
 

 }
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {
size_t readsize = 0; 
  
    //read [queue] samples ([queue/2] stereo samples)
    err = i2s_read(I2S_NUM_0,&rxbuf[0],queue*4, &readsize, 1000); //???
    
    if (err == ESP_OK && readsize==queue*4) {
        
          //extract stereo samples to mono buffers
          int y=0;
          for (int i=0; i<queue; i=i+2) {
            l_in[y] = (float) rxbuf[i];
            r_in[y] = (float) rxbuf[i+1];
            y++;
          }
          
          
          //do something with your left + right channel samples here in the buffers l_in/r_in and ouput result to l_out and r_out (e.g. build mono sum and apply -6dB gain (*0.5)
          for (int i=0; i<(queue/2); i++) {
            l_out[i] = 0.5f * (l_in[i] + r_in[i]);    // Create Mono to left
            r_out[i] = Do_PitchShift((int)l_out[i]);  // The magic happens here
            l_out[i] = r_out[i];                      // Copy shifted Mono to left
          }
          
          
          //merge two l and r buffers into a mixed buffer and write back to HW
          y=0;
          for (int i=0;i<(queue/2);i++) {
            txbuf[y] = (int) l_out[i];
            txbuf[y+1] = (int) r_out[i];
            y=y+2;
          }

          //write [queue] samples ([queue/2] stereo samples)
          i2s_write(I2S_NUM_0, &txbuf[0],queue*4, &readsize, 1000);


      }   // end of cycle

}         // end of void loop()
