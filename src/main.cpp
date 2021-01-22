// #define ENABLE_BLE 

#include <Arduino.h>
#include <limits.h>
#include <Wire.h>
#include <Stream.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#ifdef ENABLE_BLE
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#endif 
#include <Fusion.h>
#include <PID_v1.h>
#include "led_colors.h"

#define LED_PIN1        24
#define LED_PIN2        25
#define LED_NUMPIXELS 13
#define PITCH_ANGLE_LED_MINMAX (10.0)


#define LCD_LOOP_FREQ (1/60.0) // Update LCD once per minute
#define VISAL_LOOP_FREQ (25.0) // Update LEDs at 25Hz

#define ANALOG_BATTERY_VOLTAGE_PIN (41)

#define FILTER_UPDATE_RATE_HZ (200.0)

#define PITCH_ANGLE_DEADBAND (0.0) // Don't do anything if the pitch error is smaller than this

// The serial port used to read the AHRS data
#define INPUT_AHRS_SERIAL Serial5
#define INPUT_AHRS_BAUDRATE 115200

// The serial port used to communicate with the motor controller
#define MOTOR_CONTROLLER_SERIAL Serial2
#define MOTOR_CONTROLLER_SERIAL_BAUDRATE 57600

#ifdef ENABLE_BLE
// Bluetooth low energy
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);
// A message consists of a start-of-header byte, a start-of-text byte, and the actual payload.
// The payload contains an identifier byte (e.g. go left or go forward), and a quantifier 
// float (e.g. desired velocity) represented by 4 bytes.
// So in total, a message has a byte size of 7.
uint8_t ble_data_buffer[5];
bool new_ble_reading_available = false;
byte ble_header[] = {1, 2}; //{'!', 'B'};
#endif

// double Kp = 70.0;  // 100 //40
// double Ki = 1200.0;  // 1500 //600
// double Kd = 1.3; // 1.5 //0.3

// LATEST
double aggressive_Kp = 90.0;  // 100 //40
double aggressive_Ki = 600.0;  // 1500 //600
double aggressive_Kd = 0.8; // 1.5 //0.3

double conservative_Kp = 120.0;  // 100 //40
double conservative_Ki = 600.0;  // 1500 //600
double conservative_Kd = 1.0; // 1.5 //0.3


// double Kp = 120.0;  // 100 //40
// double Ki = 300.0;  // 1500 //600
// double Kd = 1.8; // 1.5 //0.3

double pid_input = 0.0;
double pid_output = 0.0;
double prev_pid_output = 0.0;
double pid_setpoint = 0.0;

PID balancing_pid = PID(&pid_input, &pid_output, &pid_setpoint, aggressive_Kp, aggressive_Ki, aggressive_Kd, P_ON_M, DIRECT);

bool initialized = false;
uint32_t timestamp_of_initialization = 0;

double speed_based_setpoint = 0;
bool new_motor_controller_reading_available = false;
byte motor_controller_header[] = {1, 2};

uint32_t timestamp;

float last_roll_angle = 0.0;
uint32_t last_roll_angle_updated_timestamp = 0;

#ifdef ENABLE_BLE
float speed_input = 0.0;
float steering_input = 0.0;
#endif

LiquidCrystal_I2C lcd(0x20,16,2);
char lcd_buff[16];
uint32_t timestamp_at_boot;
uint32_t seconds_since_boot;
uint32_t last_lcd_update_timestamp = UINT_MAX;
int last_lcd_elem_updated = 0;

Adafruit_NeoPixel pixels1(LED_NUMPIXELS, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(LED_NUMPIXELS, LED_PIN2, NEO_GRB + NEO_KHZ800);

void initialize_lcd(){  
    lcd.init();                      // initialize the lcd   
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Battery:       V");      
    lcd.setCursor(0,1);
    lcd.print("Runtime:     min");
}

float get_battery_voltage(){
    int analogval = analogRead(ANALOG_BATTERY_VOLTAGE_PIN);            
    return 5.3669178897*((analogval * 3.285) / 1023.0);
}

int update_lcd(int total_runtime, int last_lcd_elem_updated){
  if(last_lcd_elem_updated==1){
      lcd.setCursor(9,0);                
      lcd.print(get_battery_voltage());
      return 0;
  }
  
  lcd.setCursor(9,1);      
  lcd.print(round((total_runtime/60.0)));
  return 1;
  
}

void setup() {

  digitalWrite(13, HIGH);    // Light up the internal led to show it's game on
  delay(20);

  Serial.begin(115200);

  // Wire.setClock(400000); // 400KHz
  initialize_lcd();        
  timestamp_at_boot = millis();  
  
  last_lcd_elem_updated = update_lcd(0, 0);
  last_lcd_elem_updated = update_lcd(0, 1);
  
  balancing_pid.SetMode(AUTOMATIC); 
  balancing_pid.SetOutputLimits(-2047, 2047);
  balancing_pid.SetSampleTime(1000.0/FILTER_UPDATE_RATE_HZ);
  
  // Configure the input serial port for our AHRS device
  INPUT_AHRS_SERIAL.begin(INPUT_AHRS_BAUDRATE);  

  MOTOR_CONTROLLER_SERIAL.begin(MOTOR_CONTROLLER_SERIAL_BAUDRATE);  

  pixels1.begin();
  pixels1.setBrightness(50);
  pixels2.begin();
  pixels2.setBrightness(50);

#ifdef ENABLE_BLE
  // Configure the bluetooth chip
  if(ble.begin(VERBOSE_MODE)){
    ble.echo(false);
    ble.verbose(false);
    ble.setMode(BLUEFRUIT_MODE_DATA);
  }
  else{
    Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }
#endif

  timestamp = micros();  
  delay(20);
}

FusionQuaternion sensor_orientation;
bool new_ahrs_reading_available = false;
byte ahrs_header[] = {1, 2};

uint16_t crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

const int serial_timeout_ms = 5;
bool readFromSerial(Stream& input_device, int message_length, uint8_t* buffer, const byte* header, int crc_size=0) {
  if(input_device.available() < message_length+2+crc_size)
    return false;  
  bool header_started = false;
  bool message_started = false;  
  while (input_device.available()) {

    // If the header was sent AND the message was started, we can start reading
    // out the message values
    if(header_started && message_started){   
      uint32_t start_yield_ts = millis();         
      while(input_device.available() < message_length+crc_size){
        if(millis()-start_yield_ts > serial_timeout_ms){
          Serial.println("SERIAL TIMEOUT!");         
          while (input_device.available())
            input_device.read(); 
          return false;
        }
        yield();            
      }

      for(int i=0; i<message_length; i++){        
        buffer[i] = input_device.read();            
      }

      if(crc_size > 0){
        uint16_t crc;
        for(int i=0; i<crc_size; i++){
          ((uint8_t*)&crc)[i] = input_device.read();
        }

        uint16_t expected_crc = 0xFFFF;        
        for (uint16_t i = 0; i < message_length; i++) {
          expected_crc = crc16_update(expected_crc, buffer[i]);
        }

        if(expected_crc != crc){
          Serial.println("BAD PACKET!");
          while (input_device.available())
            input_device.read(); 
          return false;
        }                
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

void serialEvent2() {      
  if(readFromSerial(MOTOR_CONTROLLER_SERIAL, sizeof(speed_based_setpoint), (uint8_t*) &speed_based_setpoint, motor_controller_header, 2)){
    new_motor_controller_reading_available = true;        
  }  
}

void serialEvent5() {    
  if(readFromSerial(INPUT_AHRS_SERIAL, 16, (uint8_t*) sensor_orientation.array, ahrs_header)){
    new_ahrs_reading_available = true;
  }  
}
void show_pitch_led(Adafruit_NeoPixel &pixels, int led_ix){
    pixels.clear();

    if(led_ix>0)
      pixels.setPixelColor(led_ix-1, led_colors_half_brightness[led_ix-1]);
    pixels.setPixelColor(led_ix, led_colors[led_ix]);
    if(led_ix<LED_NUMPIXELS-1)
      pixels.setPixelColor(led_ix+1, led_colors_half_brightness[led_ix+1]);
    
    pixels.show();  
}

void visualize_pitch(float led_pitch){  
      if(led_pitch<-PITCH_ANGLE_LED_MINMAX)
        led_pitch = -PITCH_ANGLE_LED_MINMAX;
      else if(led_pitch>PITCH_ANGLE_LED_MINMAX)
        led_pitch = PITCH_ANGLE_LED_MINMAX;
      int led_ix = round(map(led_pitch, -PITCH_ANGLE_LED_MINMAX, PITCH_ANGLE_LED_MINMAX, 0, LED_NUMPIXELS-1));
      show_pitch_led(pixels1, led_ix);
      show_pitch_led(pixels2, led_ix);
}

#ifdef ENABLE_BLE
void bleEvent(){
  if(readFromSerial(ble, 5, ble_data_buffer, ble_header)){
    new_ble_reading_available = true;
  }
}
#endif

int ctr = 0;

void loop() {          
  
    if(initialized && new_motor_controller_reading_available && millis()-timestamp_of_initialization>2000){
      new_motor_controller_reading_available = false;
      pid_setpoint = speed_based_setpoint;
      if(abs(pid_setpoint) <= 0.3)
        pid_setpoint = 0.0;
      // Serial.print("PID setpoint: ");
      // Serial.println(pid_setpoint);
    }
    

    uint32_t curr_time = micros();
    if(new_ahrs_reading_available){      
      uint32_t time_diff = curr_time - timestamp;      
      if(time_diff >= (1000000 / FILTER_UPDATE_RATE_HZ)){        
        float samplePeriod = time_diff/1000000.0;

        // if (time_diff < (1000000 / FILTER_UPDATE_RATE_HZ))
        //       return;
            

        timestamp = curr_time;      
        new_ahrs_reading_available = false;      

        FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(sensor_orientation);
        // if(ctr > 10){                
        //   Serial.println(samplePeriod, 10);
        //   Serial.print("Orientation: ");        
        //   Serial.print(eulerAngles.angle.yaw);
        //   Serial.print(" ");
        //   Serial.print(eulerAngles.angle.pitch);
        //   Serial.print(" ");
        //   Serial.println(eulerAngles.angle.roll); 
        //   ctr = 0;        
        // }       
        // if(samplePeriod < 0.002499 || samplePeriod > 0.002502)
        //   Serial.println(samplePeriod, 10);
        if(samplePeriod != 0.005)
          Serial.println(samplePeriod, 10);

        last_roll_angle = eulerAngles.angle.roll;               

        // Serial.println(samplePeriod, 10);
        // Serial.println(eulerAngles.angle.roll);

        double abs_err = abs(pid_setpoint - eulerAngles.angle.roll);
        if(initialized){        
          // We only balance if the robot is +-30 degrees from horizontal.
          // Otherwise things can get dangerous (i.e. stop the wheels when falling)
          if(abs(eulerAngles.angle.roll) < 30){

            pid_input = (double)(eulerAngles.angle.roll);          
            if(abs_err < 3)
              balancing_pid.SetTunings(conservative_Kp, conservative_Ki, conservative_Kd, P_ON_M);
            else
              balancing_pid.SetTunings(aggressive_Kp, aggressive_Ki, aggressive_Kd, P_ON_M);

            
            // pid_setpoint = 0.0;
            // if(abs_err > PITCH_ANGLE_DEADBAND){          
              prev_pid_output = pid_output;
              balancing_pid.Compute();
              // if(abs(pid_output - prev_pid_output))
              // Serial.print(pid_input);
              // Serial.print(" / ");
              // Serial.print(pid_output);
              // Serial.print(" / ");
              // Serial.print(prev_pid_output);     
              // Serial.print(" / ");
              // Serial.println(balancing_pid.GetKp());                  
            // }
          }
          else{          
            pid_setpoint = 0.0;
            pid_input = 0.0;
            balancing_pid.Compute();
            pid_output = 0.0;
            initialized = false;          
            timestamp_of_initialization = 0;
          }
          // Once the robot is held in upwards position for half a second, we start balancing
          // if((curr_time/1000 - timestamp_of_initialization > 500)){

            uint16_t crc = 0xFFFF;
            uint8_t* buf = (uint8_t*)&pid_output;
            for (uint16_t i = 0; i < sizeof(pid_output); i++) {
              crc = crc16_update(crc, buf[i]);
            }
            
            if(!initialized || abs_err > PITCH_ANGLE_DEADBAND){         
              MOTOR_CONTROLLER_SERIAL.print("\001\002"); // Start-of-header and start-of-text bytes
              MOTOR_CONTROLLER_SERIAL.write((uint8_t*) &(pid_output), sizeof(pid_output));
              MOTOR_CONTROLLER_SERIAL.write((uint8_t*) &crc, 2);
            }
          // }
        }
        else{
          // We only start balancing if the robot is held in upwards position.
          // Otherwise things can get dangerous
          float ang = abs(eulerAngles.angle.roll);
          if(ang >= -3 && ang <= 3){
            initialized = true;
            timestamp_of_initialization = millis();
          }
        }
                      
        
        // ctr += 1;
      }
    }

    else if(curr_time-last_roll_angle_updated_timestamp >= (1000000 / VISAL_LOOP_FREQ)){
        last_roll_angle_updated_timestamp = timestamp;
        visualize_pitch(last_roll_angle);        
    }

    else if (last_lcd_update_timestamp == UINT_MAX || curr_time-last_lcd_update_timestamp >= (1000000 / LCD_LOOP_FREQ)){
        // if (curr_time-last_lcd_update_timestamp >= (1000000 / 400)){          
            seconds_since_boot = (timestamp/1000 - timestamp_at_boot)/1000;                
            last_lcd_elem_updated = update_lcd(seconds_since_boot, last_lcd_elem_updated);
            last_lcd_update_timestamp = timestamp;
        }

    #ifdef ENABLE_BLE
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
    #endif
        

}