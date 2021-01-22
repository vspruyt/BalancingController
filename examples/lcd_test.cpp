//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LCD_LOOP_FREQ (1)
#define ANALOG_BATTERY_VOLTAGE_PIN (41)

LiquidCrystal_I2C lcd(0x20,16,2);
char lcd_buff[16];
uint32_t timestamp_at_boot;
uint32_t seconds_since_boot;

uint32_t timestamp;

void initialize_lcd(){
    lcd.init();                      // initialize the lcd   
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Battery:       V");      
    lcd.setCursor(0,1);
    lcd.print("Runtime:   :  ");
}

void update_lcd(float battery_voltage, float total_runtime){
    lcd.setCursor(9,0);    
    sprintf(lcd_buff, "%0.2f", battery_voltage);
    lcd.print(lcd_buff); 
    lcd.setCursor(9,1);
    int minutes = total_runtime/60.0;        
    sprintf(lcd_buff, "%02d", minutes);
    lcd.print(lcd_buff);
    int seconds = total_runtime-minutes*60;
    sprintf(lcd_buff, "%02d", seconds);
    lcd.setCursor(12,1);
    lcd.print(lcd_buff);
}

float get_battery_voltage(){
    int analogval = analogRead(ANALOG_BATTERY_VOLTAGE_PIN);            
    return 5.3669178897*((analogval * 3.285) / 1023.0);
}

void setup()
{
    initialize_lcd();    

    timestamp = micros();
    timestamp_at_boot = millis();  
}


void loop()
{
    
    uint32_t curr_time = micros();
    uint32_t time_diff = curr_time - timestamp;
    if (time_diff < (1000000 / LCD_LOOP_FREQ)) return;    
    timestamp = curr_time;

    seconds_since_boot = (timestamp/1000 - timestamp_at_boot)/1000;    
    float battery_voltage = get_battery_voltage();
    update_lcd(battery_voltage, seconds_since_boot);
}