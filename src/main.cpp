#include <Arduino.h>
#include <Wire.h>
#include <Stream.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Fusion.h>

// The serial port used to read the AHRS data
#define INPUT_AHRS_SERIAL Serial8
#define INPUT_AHRS_BAUDRATE 2000000

// Bluetooth low energy
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
// A message consists of a start-of-header byte, a start-of-text byte, and the actual payload.
// The payload contains an identifier byte (e.g. go left or go forward), and a quantifier 
// float (e.g. desired velocity) represented by 4 bytes.
// So in total, a message has a byte size of 7.
uint8_t ble_data_buffer[5];
bool new_ble_reading_available = false;
byte ble_header[] = {1, 2}; //{'!', 'B'};

uint32_t timestamp;
float speed_input = 0.0;
float steering_input = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Configure the input serial port for our AHRS device
  INPUT_AHRS_SERIAL.begin(INPUT_AHRS_BAUDRATE);  

  // Configure the bluetooth chip
  if(ble.begin(VERBOSE_MODE)){
    ble.echo(false);
    ble.verbose(false);
    ble.setMode(BLUEFRUIT_MODE_DATA);
  }
  else{
    Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }

  timestamp = micros();  
}

FusionQuaternion sensor_orientation;
bool new_ahrs_reading_available = false;
byte ahrs_header[] = {1, 2};

bool readFromSerial(Stream& input_device, int message_length, uint8_t* buffer, const byte* header) {
  if(input_device.available() < message_length+2)
    return false;  
  bool header_started = false;
  bool message_started = false;  
  while (input_device.available()) {

    // If the header was sent AND the message was started, we can start reading
    // out the message values
    if(header_started && message_started){            
      while(input_device.available() < message_length)
        yield();            

      for(int i=0; i<message_length; i++){        
        buffer[i] = input_device.read();
      }
      return true;                  
    }

    else{
      // get the new byte:
      uint8_t inbyte = (uint8_t)input_device.read();      

      // If the header is already started, we now expect the start-of-message.
      // If that is not the case, the header was actually not started yet.
      if(header_started && !message_started){
        if(inbyte==header[1]){
          message_started = true;
          continue;
        }
        else
          header_started = false;
      }

      // Check if the header is being sent
      if(!header_started){
        // check if the byte is a start-of-header byte
        if(inbyte==header[0]){
          header_started = true;
          continue;
        }
      }
    }
  }

  return false;
}

void serialEvent8() {    
  if(readFromSerial(INPUT_AHRS_SERIAL, 16, (uint8_t*) sensor_orientation.array, ahrs_header)){
    new_ahrs_reading_available = true;
  }
}

void bleEvent(){
  if(readFromSerial(ble, 5, ble_data_buffer, ble_header)){
    new_ble_reading_available = true;
  }
}

int ctr = 0;
void loop() {      

    digitalWrite(13, HIGH);    
    
    if(new_ahrs_reading_available){
      uint32_t curr_time = micros();
      uint32_t time_diff = curr_time - timestamp;      
      float samplePeriod = time_diff/1000000.0;
      timestamp = curr_time;      
      new_ahrs_reading_available = false;      

      if(ctr > 400){                
        // Serial.println(samplePeriod, 10);
        Serial.print("Orientation: ");
        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(sensor_orientation);
        Serial.print(eulerAngles.angle.yaw);
        Serial.print(" ");
        Serial.print(eulerAngles.angle.pitch);
        Serial.print(" ");
        Serial.println(eulerAngles.angle.roll); 
        ctr = 0;
      }

      ctr += 1;
    }

    
    // Check if there is a bluetooth command ready in the buffer
    bleEvent();
    if(new_ble_reading_available){
      new_ble_reading_available=false;                  
      float val = *((float*)&(ble_data_buffer[1]));
      if(ble_data_buffer[0] == 0)
        speed_input = val;
      else
        steering_input = val;
      Serial.print("Speed: ");
      Serial.print(speed_input);
      Serial.print(" / Steering: ");
      Serial.println(steering_input);
                     
      char outstr[30];
      sprintf(outstr, "\001Speed: %.2f\nSteering: %.2f\n", speed_input, steering_input);
      ble.print(outstr);      

    }          
        

}