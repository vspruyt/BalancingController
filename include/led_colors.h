#include <Adafruit_NeoPixel.h>

uint32_t led_color_black = Adafruit_NeoPixel::Color(0, 0, 0);
uint32_t led_colors[] = {
  Adafruit_NeoPixel::Color(251, 0, 10),
  Adafruit_NeoPixel::Color(253, 52, 14), 
  Adafruit_NeoPixel::Color(253, 89, 15),
  Adafruit_NeoPixel::Color(242, 238, 11),  
  Adafruit_NeoPixel::Color(169, 206, 9),
  Adafruit_NeoPixel::Color(94, 172, 8),
  Adafruit_NeoPixel::Color(32, 137, 1),  
  Adafruit_NeoPixel::Color(94, 172, 8),  
  Adafruit_NeoPixel::Color(169, 206, 9),  
  Adafruit_NeoPixel::Color(242, 238, 11),    
  Adafruit_NeoPixel::Color(253, 89, 15),
  Adafruit_NeoPixel::Color(253, 52, 14),  
  Adafruit_NeoPixel::Color(251, 0, 10)
};

uint32_t led_colors_half_brightness[] = {
  Adafruit_NeoPixel::Color(251*0.1, 0*0.1, 10*0.1),
  Adafruit_NeoPixel::Color(253*0.1, 52*0.1, 14*0.1), 
  Adafruit_NeoPixel::Color(253*0.1, 89*0.1, 15*0.1),
  Adafruit_NeoPixel::Color(242*0.1, 238*0.1, 11*0.1),  
  Adafruit_NeoPixel::Color(169*0.1, 206*0.1, 9*0.1),
  Adafruit_NeoPixel::Color(94*0.1, 172*0.1, 8*0.1),
  Adafruit_NeoPixel::Color(32*0.1, 137*0.1, 1*0.1),  
  Adafruit_NeoPixel::Color(94*0.1, 172*0.1, 8*0.1),  
  Adafruit_NeoPixel::Color(169*0.1, 206*0.1, 9*0.1),  
  Adafruit_NeoPixel::Color(242*0.1, 238*0.1, 11*0.1),    
  Adafruit_NeoPixel::Color(253*0.1, 89*0.1, 15*0.1),
  Adafruit_NeoPixel::Color(253*0.1, 52*0.1, 14*0.1),  
  Adafruit_NeoPixel::Color(251*0.1, 0*0.1, 10*0.1)
};