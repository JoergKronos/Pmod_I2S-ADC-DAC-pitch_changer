// Modifications to run the code at Arduino IDE
// Joerg Rettkowski 20.03.2021
//
//Version 1.0 20.March 2021
//  For Arduino IDE 1.8.13
//  ESP-32 / i2s protocol / DMA background transfer
//  Read audio from i2s ADC (DIGILENT Pmod I2S2) into buffer
//  Highpass filter 300Hz /remove rumble
//  Pitch shift
//  Write audio using i2s to DAC (DIGILENT Pmod I2S2)
//  Mono mode only
//Version 2.2 22.March 2021
//  Rotary encoder to change the pitch live. Hardware Interrupt and run at 2nd Core.
//Version 3.1 24.March 2021
//  Add a small OLED display. 128x32 pixel. I2C interface. Shows Pitch and peak level with memory
//Version 4.1 01.Apr.2021
//  Add Microphone, output amplifier. Add a notch filter to eliminate feedbacks
//Planned Updates:
//  Add a 6th order Butterworth filter on the output  to smooth the high frequency noise from the signal. 
//  Read mix of 2 buffer pointed by read pointer
//  To make it sound like Kylo Ren, some more magic is needed. https://www.ecgprod.com/how-to-sound-like-kylo-ren/
// ************************************************************



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


#include "driver/i2s.h"


//error type
esp_err_t err;

//buffers
#define queue  16   // 2+(R+L)  Samples
int rxbuf[queue], txbuf[queue];
float l_in[queue/2], r_in[queue/2];
float l_out[queue/2], r_out[queue/2];


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
int Peak[4];              // Stores the first value of the DMA transfer to output. 4 rounds are one cycle
int PC=0;                 //Peak Counter

int WtrP;                 // write pointer
float Rd_P;               // read pointer
float Shift;              // shift for pitch
float CrossFade;
float a0h, a1h, a2h, b1h, b2h, hp_in_z1h, hp_in_z2h, hp_out_z1h, hp_out_z2h;
float a0n, a1n, a2n, b1n, b2n, hp_in_z1n, hp_in_z2n, hp_out_z1n, hp_out_z2n;
float a0, a1, a2, b1, b2;
int NotchF,NotchFold;     //Notch filter frequency

//---Job for CORE 0 ---
TaskHandle_t Core0;   // This is a independet loop
                      // Arduino runs be defaultat core 1

#include <esp_task_wdt.h>

// Pitch
byte dirE = 0;     //no direction is giving
byte ACKE = 0;     //Arbitrary

//Feedback
byte dirF = 0;     //no direction is giving
byte ACKF = 0;     //Arbitrary

//Display
#include "SSD1306Wire.h"



//**************************************************************************************
//******  Interrupt
//**************************************************************************************

  void IRAM_ATTR ISRccwE() {  //Code hold in RAM. Change seen at encoder pin EA 
    if(ACKE==2){             //Acknowledge the other was first 
      dirE=2;
      ACKE=0;
    }  
    if(ACKE==0)
      if(dirE==0)
        ACKE=1;
  }

//**************************************************************************************  
  void IRAM_ATTR ISRcwE() { //Code hold in RAM. Change seen at encoder pin EB
    if(ACKE==1){             //Acknowledge the other was first 
      dirE=1;
      ACKE=0;
    }  
    if(ACKE==0)
      if(dirE==0)
        ACKE=2;
  }
//**************************************************************************************

  void IRAM_ATTR ISRccwF() {  //Code hold in RAM. Change seen at encoder pin EA 
    if(ACKF==2){             //Acknowledge the other was first 
      dirF=2;
      ACKF=0;
    }  
    if(ACKF==0)
      if(dirF==0)
        ACKF=1;
  }

//**************************************************************************************  
  void IRAM_ATTR ISRcwF() { //Code hold in RAM. Change seen at encoder pin EB
    if(ACKF==1){             //Acknowledge the other was first 
      dirF=1;
      ACKF=0;
    }  
    if(ACKF==0)
      if(dirF==0)
        ACKF=2;
  }

//**************************************************************************************
//******  CORE 0
//**************************************************************************************

void core0code( void * pvParameters ){  

// Pitch live change. -----------------------------
// variable pitch using Rotary Encoder with a switch
// KY-040 Rotary Encoder – EV3DM
// GND  Ground
// +    Vcc
// SW   LOW if switch is pressed !! This switch has not pull up, so I add a 10K to Vcc
// DT   Encoder Pin A  CCW this is first changed
// CLK  Encoder Pin B   CW this is first changed

//Notch
#define EA  15          // Encoder Pin A
#define EB  4           // Encoder Pin B
#define ES  14          // Encoder Switch

//Feedback Notch filering
#define FA  13          // Encoder Pin A
#define FB  27          // Encoder Pin B
#define FS  5           // Encoder Switch


#define PitchH  2.00f    // Maximum pitch
#define PitchL  0.50f    // Minimum pitch
#define PitchS  0.02f   // Pitch Shift per Step

  
// Pitch Encoder Module
  pinMode(EA, INPUT);     // Encoder Pin A
  pinMode(EB, INPUT);     // Encoder Pin B
  pinMode(ES, INPUT);     // Encoder Switch

// Feedback Encoder Module
  pinMode(FA, INPUT);     // Encoder Pin A
  pinMode(FB, INPUT);     // Encoder Pin B
  pinMode(FS, INPUT);     // Encoder Switch


  attachInterrupt(EA, ISRccwE, CHANGE);
  attachInterrupt(EB, ISRcwE,  CHANGE);
  attachInterrupt(FA, ISRccwF, CHANGE);
  attachInterrupt(FB, ISRcwF,  CHANGE);

  Shift     = 1.0f; //Default Pitch
  NotchF    = 1000; //Default Notch
  NotchFold = 1;
  

//Display --------------------------------------------
// 0.91 Inch OLED LCD Display Module I2C interface 
// Install the "ESP8266 and ESP32 OLED driver for ssd1306 displays" Library
// I2C address is 0x3C (or 0x3D)

// GND: Ground
// VCC: Vcc
// SCL: I2C Clock Line
// SDA: I2C Data Line

// Using the second I2C interface
#define I2C_SDA 33 
#define I2C_SCL 32

// ADDRESS, SDA, SCL, OLEDDISPLAY_GEOMETRY, I2C Select, I2C Frequency 100kHz
  SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL, GEOMETRY_128_32, I2C_TWO, 100000);;  

  display.init();
  display.clear();                            // clear the display
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(20, 13, "ESP32-Pitch");
  display.display();                          // write the buffer to the display

  delay(2000);

  int valu0;     //Peak
  int valu1;     //Peak
  int valu2;     //Peak
  int valu3;     //Peak
  int n=0;       //counter
  int valu[16];  //Bucket Chain Storage

//Define coefficients for Notch
  float norm,V,K,Q,Gain;

    
// LOOP ---------------------------------
  while(true){
    vTaskDelay(10);  

//Feedback ---------------------------
/has the feedback frequency been changed?
  if (dirF==1){
    NotchF += 100;  //increase frequency
    dirF = 0;
  }  

  if (dirF==2){
    NotchF -= 100;  //decrease Frequenz
    dirF = 0;
  }  

  if(NotchF!=NotchFold){
    NotchFold = NotchF;
//Yes, is has been changed.
//Recalculate the values of the filter. Check here how IIR filters do work:
// https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/

    Gain = 6.0f;  //6dB/Cotave
    Q    = 3.0f;  //Iteration
    V    = pow(10, (Gain/20));
    K    = tan(PI * NotchF / 44100);

    norm = 1 / (1 + K / Q + K * K);
    a0   = (1 + K * K) * norm;
    a1   = 2 * (K * K - 1) * norm;
    a2   = a0;
    b1   = a1;
    b2   = (1 - K / Q + K * K) * norm;
  }


//Pitch ---------------------------
  if (dirE==1){
    Shift += PitchS;  //increase Pitch Shift
    dirE = 0;
  }  

  if (dirE==2){
    Shift -= PitchS;  //decrease Pitch Shift
    dirE = 0;
  }  


//Feedback Shift button pressed  
    if (digitalRead(FS)==LOW)  
      NotchF=1500; 
        
//Reset Shift button pressed  
    if (digitalRead(ES)==LOW)  
      Shift=1.0f; 
      
//Indicate Shift is 1  
    if ((Shift<=1.009f)&&(Shift>=0.999f)) Shift=1.00f;
    

//Check for Pitch Shift boundarys 
  if ((Shift-PitchH)>= 0.009f) Shift = PitchH;
  if ((Shift-PitchL)<= 0.009f) Shift = PitchL;

//Check for Pitch Shift boundarys 
  if (NotchF<=1) NotchF=1;

//Display Pitch ----------------------------------------
  valu0=abs(Peak[0])/2400000;    // Store peak value from DMA Burst 0
  valu1=abs(Peak[1])/2400000;    // Store peak value from DMA Burst 1
  valu2=abs(Peak[2])/2400000;    // Store peak value from DMA Burst 2
  valu3=abs(Peak[3])/2400000;    // Store peak value from DMA Burst 3

  if (valu0<valu1) valu0=valu1;  // search for lagest peak
  if (valu0<valu2) valu0=valu2;
  if (valu0<valu3) valu0=valu3;
  if (valu0>=128)  valu0=127;    // Check and fix maximum value

                                 //Bucket Chain Storage is used for the 500ms delay of the mximum peak view
  valu[15]= valu0;               //Current value to top bucket
  valu[0]  = 0;                  //Clean last bucket
  for (n=1;n<=15;n++) {          //copy bucket down and search for the peak
    if (valu[0]<valu[n]) valu[0]=valu[n];
    valu[n-1]=valu[n];
  }
  
  
  display.clear();                                        // clear the display
  display.drawString(0, 0, "Pitch : " + String(Shift) );  //show the pitch
  display.drawString(0, 13, "Notch: " + String(NotchF));  //show the Notch
  display.fillRect(0, 29, valu0  , 4);                    // (x, y, width, height)
  display.drawVerticalLine(valu[0], 29, 4);               // (x, y, width)

  display.display();                                      // write the buffer to the display

//**************************************************************************************
 
  
  } //While loop
}   //CORE0
  

//**************************************************************************************
//******  CORE 1
//**************************************************************************************
void setup() {

Serial.begin(115200);
delay (500);


//ring buffer start 0°
  WtrP = 0;
  Rd_P = 0.0f;
  Shift = 0.70f;
  CrossFade = 1.0f;

//------------------------------------------------------------
//enable MCLK on GPIO0
  REG_WRITE(PIN_CTRL, 0xFF0); 
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);

//------------------------------------------------------------
  
  i2s_config_t i2s_config = {

        .mode                 = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX  | I2S_MODE_RX), //Work mode
        .sample_rate          = 44100,                                                    //Ssample rate
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,                                //bits per sample
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,                               //channel format
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,                                  //communication format
        .intr_alloc_flags     = 0,                                                        //Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info
        .dma_buf_count        = 8,                                                        //DMA Buffer Count
        .dma_buf_len          = queue*2,                                                  //DMA Buffer Length
        .use_apll             = true,                                                     //using APLL as main clock, enable it to get accurate clock
        .tx_desc_auto_clear   = true,                                                     //auto clear tx descriptor if there is underflow condition (helps in avoiding noise in case of data unavailability)
        .fixed_mclk           = true                                                      //using fixed MCLK output. If use_apll = true and fixed_mclk > 0, then the clock output for is fixed and equal to the fixed_mclk value
                                             
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


//----------------------------------------------------------------
//create a task that will be executed in the CORE0() function, with priority 1 and executed on core 0
//Core 1 is default for arduino
  xTaskCreatePinnedToCore(
                    core0code,   // Task function.
                    "CORE0",     // name of task. 
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    5,           // priority of the task 
                    &Core0,      // Task handle to keep track of created task 
                    0);          // pin task to core 0 
    delay(500); 


 }


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Notch filter, 44kHz, Q=3, 
// IIR Filter calulation 
// https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
// Notch to reduce feedback

int Do_Notch (int inSample) {
  a0n = a0;
  a1n = a1;
  a2n = a2;
  b1n = b1;
  b2n = b2;

  float inSampleF = (float)inSample;
  float outSampleF =
      a0n * inSampleF
      + a1n * hp_in_z1n
      + a2n * hp_in_z2n
      - b1n * hp_out_z1n
      - b2n * hp_out_z2n;
      
  hp_in_z2n = hp_in_z1n;
  hp_in_z1n = inSampleF;
  hp_out_z2n = hp_out_z1n;
  hp_out_z1n = outSampleF;

  return (int) outSampleF;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Highpass, 44kHz, 300Hz, Q=SQRT(2)/2=0.7071, 6dB gain

int Do_HighPass (int inSample) {
  a0h = 0.9087554064944908f;
  a1h = -1.7990948352036205f;
  a2h = 0.9087554064944908f;
  b1h = -1.7990948352036205f;
  b2h = 0.8175108129889816f;

  float inSampleF = (float)inSample;
  float outSampleF =
      a0h * inSampleF
      + a1h * hp_in_z1h
      + a2h * hp_in_z2h
      - b1h * hp_out_z1h
      - b2h * hp_out_z2h;
      
  hp_in_z2h = hp_in_z1h;
  hp_in_z1h = inSampleF;
  hp_out_z2h = hp_out_z1h;
  hp_out_z1h = outSampleF;

  return (int) outSampleF;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int Do_PitchShift(int Sample) {
  int sum;
  float share;
    
//sum up and do high-pass and notch
  sum=Do_HighPass(Sample);
  sum=Do_Notch(sum);
  
  //write to ringbuffer
  Buf[WtrP] = sum;
 
//read fractional readpointer and generate 0° and 180° read-pointer in integer
  int RdPtr_Int = roundf(Rd_P);

//frac share of RdPtr_Int1a. // -1>= share <1
  share =  (float) Rd_P - RdPtr_Int;

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


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void loop() {
size_t readsize = 0; 

  
  
    //read [queue] samples ([queue/2] stereo samples)
    err = i2s_read(I2S_NUM_0,&rxbuf[0],queue*4, &readsize, 1000); 
    
    if (err == ESP_OK && readsize==queue*4) {

          //extract stereo samples to mono buffers
          int y=0;
          for (int i=0; i<queue; i=i+2) {
              //sum up and do high-pass and notch
  

            l_in[y] = (float) rxbuf[i];
            r_in[y] = (float) rxbuf[i+1];
            y++;
          }
          
          
          //do something with your left + right channel samples here in the buffers l_in/r_in and ouput result to l_out and r_out (e.g. build mono sum and apply -6dB gain (*0.5)
          for (int i=0; i<(queue/2); i++) {
            l_out[i] = 0.5f * (l_in[i] + r_in[i]);
            r_out[i] = Do_PitchShift((int)l_out[i]);
            l_out[i] = r_out[i];
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
          
          // Stor the last of the 4 DMA queues
          Peak[PC] = txbuf[(queue/2)-1];
          if (PC==3) PC=0;
          else PC++;

      }   // end of cycle

}         // end of void loop()
