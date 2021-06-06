#include <movingAvg.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN 6
#define LED_COUNT 8

//################# Adjustable Settings ####################
//Arduino Testing
int testMode = 1;  //Set testMode=1 to test LED strip without using the signal from the coil pack, set testMode=0 when using RPM signal from coil pack
int testPR = 1;
int testRPM = 0;
//Adjustable LED colors [colors follow RGB color code, 0-255] - Try to keep the sum of the 3 numbers for each color around 255 for 
//even brightness between the three colors (ex yellow code is 255,255,0 - divide each number by 2 to get 125+125+0=250)
int color1[] = {0,0,255};   //Color 1, Blue
int color2[] = {125,125,0}; //Color 2, Yellow
int color3[] = {255,0,0};   //Color 3, Red
int flash_speed = 10; //Can be increased or decreased to adjust flash speed (flashes when shift RPM is exceeded)
//LED Brightness Control
int brightness_response_time = 500;  //LED brightness response time (lower number responds more quickly to ambient light, higher number is more stable during high variation of ambient light brightness)
int min_bright = 25;  //min LED brightness (any value from 1 to max_bright)
int max_bright = 200; //max LED brightness (any value from min_bright to 255)
float max_PR = 550.0;  //Photoresistor voltage at max daylight (direct sunlight)
float min_PR = 10.0; //Photoresistor voltage at night
//RPM control, adjustable
int on_rpm = 3000;  //RPM at which the LED's begin to turn on
int shift_rpm = 6000; //Desired shift point, initiates flashing LEDS. Must be higher than on_rpm
//RPM Calibration | y = m*x + b
int m = 22.315;  //Slope of trendline
int b = -66.2;  //Y-intercept of trendline

//############### Setup, DO NOT ADJUST #####################
int x = 5;   //RPM simulator counter / Test Mode
int RPM_test_mode = 1500; //current RPM simulator / Test Mode
int RPM_pin = A4; //RPM analog input pin
int RPM_volt;  //current RPM Voltage
int RPM_volt_ave; // average RPM voltage
int RPM_ave; // Actual RPM Ave
int max_rpm_calibration = 0;
int PR_pin = A6;  //Photoresistor analog input pin
int brightness;   //LED brightness ratio
int PR_volt;      //photoresistor voltage
int PR_ave;       //Average photoresistor voltage
bool flash = 1;   //flash boolean
int flash_count = 0;
int flash_counter = 1;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
movingAvg photoCell(brightness_response_time); //averages photoresistor voltage over last 500 readings
movingAvg rpmMovingAve(15); // averages rpm voltage over last 30 readings (to smooth out the signal)


//############### Void Setup - runs once when powered on #####################
void setup() {
  Serial.begin(115200);
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  pinMode(PR_pin,INPUT);  //establishes photoresistor pin
  pinMode(RPM_pin, INPUT);  // establishes RPM pin
  photoCell.begin();   //begins photoresistor moving average
  rpmMovingAve.begin();   //begins rpm moving average
}


//############### Void Loop - runs until power off #####################
void loop() {

//---------- RPM ----------
  RPM_volt = analogRead(RPM_pin);  //Reads voltage from LM2917N
  RPM_volt_ave = rpmMovingAve.reading(RPM_volt);  //Averages reading to smooth out noise
  RPM_ave = int(m * RPM_volt_ave + b);  //Converts the ave voltage to the current RPM value (based off of voltage-RPM correlation)
  if (RPM_volt_ave > max_rpm_calibration){  //Logs the max RPM voltage for calibration
    max_rpm_calibration = RPM_volt_ave;
  }
  if (testRPM == 1){
    String a1 = "   Current RPM volt = ";
    String b1 = a1 + RPM_volt_ave;
    String c1 = "Max RPM volt = ";
    String d1 = c1 + max_rpm_calibration;
    Serial.print(d1); //RPM max readout
    Serial.println(b1); //RPM current readout
  }
  //RPM Simulator / Test Mode
  if (testMode == 1){
    RPM_test_mode = RPM_test_mode + x;
    if (RPM_test_mode > 6500){
      x = -x;
    }
    if (RPM_test_mode < 1300){
      x = -x;
    }
    RPM_ave = RPM_test_mode;
  }

//---------- Photoresistor ----------
  PR_volt = analogRead(PR_pin);   //Reads photoresistor voltage
  PR_ave = photoCell.reading(PR_volt);  //averages photoresistor voltage
  if (testPR == 1){
    String str1 = "Photoresistor value: ";
    String PR_ave_str = str1 + PR_ave;
    Serial.println(PR_ave_str);  //average photoresistor value readout
  }

//---------- LED Brightness Control ----------
  //linear interpolation to scale LED brightness based on ambient lighting:
  brightness = int(min_bright + (PR_ave - min_PR)*((max_bright - min_bright)/(max_PR - min_PR)));
  if (brightness > max_bright){ //Caps brightness at max
    brightness = max_bright;
  }
  if (brightness < min_bright){ //Caps brightness at min
    brightness = min_bright;
  }

  //creates brightness ratio, from 0.0 to 1.0:
  float y = brightness/255.00;
  //Adjusts all three colors to correct brightness level:
  int color1R = color1[0]*y;
  int color1G = color1[1]*y;
  int color1B = color1[2]*y;
  int color2R = color2[0]*y;
  int color2G = color2[1]*y;
  int color2B = color2[2]*y;
  int color3R = color3[0]*y;
  int color3G = color3[1]*y;
  int color3B = color3[2]*y;

  //Converts colors to single 32-bit type for easier use
  uint32_t color1 = strip.Color(color1R, color1G, color1B);
  uint32_t color2 = strip.Color(color2R, color2G, color2B);
  uint32_t color3 = strip.Color(color3R, color3G, color3B);
  uint32_t LED_off = strip.Color(0,0,0);


//---------- LED Sequencing ----------
  //scales individual LED activation based on choosen on_rpm and shift_rpm
  int led_divider = (shift_rpm - on_rpm)/4;
  
  //LED activation based on current RPM. LEDS turn on from outside to middle.
  if (RPM_ave < on_rpm){  //LED's off below on_rpm
    strip.clear();
  }
  if (on_rpm < RPM_ave){  //Two LED's on (color1) above on_rpm
    strip.fill(color1,0,1);  // First LED On
    strip.fill(color1,7,1);  // Last LED ON
    strip.fill(LED_off,1,6); // LEDs in middle off
  }
  if ((on_rpm + led_divider) < RPM_ave){  //Four LED's on (color1)
    strip.fill(color1,0,2);
    strip.fill(color1,6,2);
    strip.fill(LED_off,2,4);
  }
  if ((on_rpm + led_divider*2) < RPM_ave){  //Six LED's on (color1)
    strip.fill(color1,0,3);
    strip.fill(color1,5,3);
    strip.fill(LED_off,3,2);
  }
  if ((on_rpm + led_divider*3) < RPM_ave){  //Six color1 and Two color2 LED's on
    strip.fill(color1,0,3);
    strip.fill(color1,5,3);
    strip.fill(color2,3,2);
  }
  if ((shift_rpm - RPM_ave) < 250){   //Eight color2 LED's on just prior to shift_rpm
    strip.fill(color2,0,8);
  }
  if ((shift_rpm < RPM_ave) && (flash == 1)){   //Color3 flash on, all eight LED's
    strip.fill(color3,0,8);
  }
  if ((shift_rpm < RPM_ave) && (flash == 0)){   //Color3 flash off
    strip.fill(LED_off,0,8);
  }
  
  flash_count = flash_count + flash_counter;
  if (flash_count > flash_speed){
    flash_counter = -1;
    flash = 0;
  }
  if (flash_count < 1){
    flash_counter = 1;
    flash = 1;
  }
  
  strip.show(); //Update LED strip with current configuration
  delay(1); //1ms delay
}
  
