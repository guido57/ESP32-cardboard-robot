#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int x;
    int y;
    char cmd;
} struct_message;

#define out0 16
#define out1 17
#define out2 12
#define out3 32


// Create a struct_message called myData
struct_message myData;

// Which pins on the Arduino are connected to the green and white LEDs?
#define WHITE_LED_PIN  0
#define GREEN_LED_PIN  2

// Which pin on the ESP32 is connected to the NeoPixels?
#define RING_LED_PIN  4

// How many NeoPixels LEDs are attached to the Arduino?
#define LED_COUNT 12

int           pixelInterval = 50;       // Pixel Interval (ms)
uint16_t      pixelCurrent = 0;         // Pattern Current Pixel Number
uint16_t      pixelNumber = LED_COUNT;  // Total Number of Pixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, RING_LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

long last_data_rcv = -1;
int speedL = 0;
int speedR = 0;
int deltaspeed = 100;
char cmd = 0x00;

void getcmd_old(int vx, int vy){
  
  int speed = sqrt((vx-2048)*(vx-2048)+(vy-2048)*(vy-2048))/8;
  if(speed>255) speed=255;
  if(speed > 50)
    speed = 150 + (speed - 50) * 105 / (255-50);

  if(vy<1800){
      if(vx < 1800){
        // BWD Left
        cmd = 0x04;
        speedL = speed;
        speedR = speed-deltaspeed;
      }else if(vx < 2200){
        // BWD 
        cmd = 0x05;
        speedL = speed;
        speedR = speed;
      }else{
        // BWD Right
        // vx >= 2200
        cmd = 0x01;
        speedL = speed-deltaspeed;
        speedR = speed;
      }  
      ledcWrite(0, speedL);   
      ledcWrite(1, 0);   
      ledcWrite(2, speedR);   
      ledcWrite(3, 0);   

  }else if(vy<2200){
      if(vx < 1800){
        // Left
        cmd = 0x06;
        speedL = speed;
        speedR = speed;
        ledcWrite(0, 0);   
        ledcWrite(1, speedL);   
        ledcWrite(2, speedR);   
        ledcWrite(3, 0);   
      }else if(vx < 2200){
        cmd = 0x00; // no movement
        speedL = 0;
        speedR = 0;
        ledcWrite(0, 0);   
        ledcWrite(1, 0);   
        ledcWrite(2, 0);   
        ledcWrite(3, 0);   
      }else{
        // vx >= 2200
        // Right
        cmd = 0x09;
        speedL = speed;
        speedR = speed;
        ledcWrite(0, speedL);   
        ledcWrite(1, 0);   
        ledcWrite(2, 0);   
        ledcWrite(3, speedR);   
      }  
      
  }else{
      // vy >= 2200
      if(vx < 1800){
        // FWD Left
        cmd = 0x02;
        speedL = speed ;
        speedR = speed - deltaspeed;
      }else if(vx < 2200){
        cmd = 0x0A; // FWD
        speedL = speed;
        speedR = speed;
      }else{
        // vx >= 2200
        // FWD Right
        cmd = 0x08; 
        speedL = speed - deltaspeed;
        speedR = speed;
      }  
      ledcWrite(0, 0);   
      ledcWrite(1, speedL);   
      ledcWrite(2, 0);   
      ledcWrite(3, speedR);   
  }
}

void getcmd_old2(int vx, int vy){
  float x = vx - 2048.0;
  float y = vy - 2048.0; 
  
  int speed = sqrt((vx-2048)*(vx-2048)+(vy-2048)*(vy-2048))/8;
  if(speed>255) speed=255;
  if(speed > 50)
    speed = 150 + (speed - 50) * 105 / (255-50);
  else{
    cmd = 0x00;
    ledcWrite(0, 0);   
    ledcWrite(1, 0);   
    ledcWrite(2, 0);   
    ledcWrite(3, 0);   
    return;
  }    

  float k = 0.4142; // tan(pi/8)=0.4142
  
  if(x >= - k*y && x < k*y){
    cmd = 0x0A; // FWD
    speedL = speed;
    speedR = speed;
    ledcWrite(0, 0);   
    ledcWrite(1, speedL);   
    ledcWrite(2, 0);   
    ledcWrite(3, speedR);   
    return;
  }
  if(y >= x*k && y < x / k){
    cmd = 0x08; // FWD Right
    speedL = speed - deltaspeed;
    speedR = speed;
    ledcWrite(0, 0);   
    ledcWrite(1, speedL);   
    ledcWrite(2, 0);   
    ledcWrite(3, speedR);   
    return;
  }
  if(y >= - x*k  && y < -x / k){
    cmd = 0x02; // FWD left
    speedL = speed ;
    speedR = speed - deltaspeed;
    ledcWrite(0, 0);   
    ledcWrite(1, speedL);   
    ledcWrite(2, 0);   
    ledcWrite(3, speedR);   
    return;
  }

  if(y >= x*k  && y < -x * k){
    // Left
    cmd = 0x06;
    speedL = speed;
    speedR = speed;
    ledcWrite(0, 0);   
    ledcWrite(1, speedL);   
    ledcWrite(2, speedR);   
    ledcWrite(3, 0);   
    return;
  }  

  if(y >= -x*k  && y < x * k){
    // Right
    cmd = 0x09;
    speedL = speed;
    speedR = speed;
    ledcWrite(0, speedL);   
    ledcWrite(1, 0);   
    ledcWrite(2, 0);   
    ledcWrite(3, speedR);   
    return;
  }
  if(y >= -x/k  && y < -x * k){
    // BWD Right 
    cmd = 0x01;
    speedL = speed-deltaspeed;
    speedR = speed;
    ledcWrite(0, speedL);   
    ledcWrite(1, 0);   
    ledcWrite(2, speedR);   
    ledcWrite(3, 0);   
    return;
  }
  if(y >= x/k  && y < x * k){
    // BWD Left 
    cmd = 0x04;
    speedL = speed;
    speedR = speed-deltaspeed;
    ledcWrite(0, speedL);   
    ledcWrite(1, 0);   
    ledcWrite(2, speedR);   
    ledcWrite(3, 0);   
    return;
  }
  if(x >= y*k  && x < - y * k){
    // BWD
    cmd = 0x05;
    speedL = speed;
    speedR = speed;
    ledcWrite(0, speedL);   
    ledcWrite(1, 0);   
    ledcWrite(2, speedR);   
    ledcWrite(3, 0);   
    return;
  }
}

// -255 <= speedX <= 255
// -255 <= speedY <= 255
void motors(int speedL, int speedR){

  if(speedL >=0){
    ledcWrite(2, 0);   
    ledcWrite(3, speedL);   
  }else{
    ledcWrite(2, -speedL);   
    ledcWrite(3, 0);   
  }  
    
  if(speedR >=0){
    ledcWrite(0, 0);   
    ledcWrite(1, speedR);   
  }else{
    ledcWrite(0, -speedR);   
    ledcWrite(1, -0);   
  }  
}

// Set cmd speed speedR and speedL according to joystick x and y
void getcmd(int vx, int vy){
  float x = vx - 2048.0;
  float y = vy - 2048.0; 
  
  double fi = atan2(x,y); 
  if(fi<0) fi+=2*PI;
  Serial.printf("fi=%f\r\n",fi);

  int speed = sqrt((vx-2048)*(vx-2048)+(vy-2048)*(vy-2048))/8;
  if(speed>255) speed=255;
  if(speed > 50)
    speed = 150 + (speed - 50) * 105 / (255-50);
  else{
    cmd = 0x00;
    motors(0,0);
    return;
  }    
  
  speedR = speedL = 0;
  if(fi >= 0 && fi < PI/2){
    speedR = 255*cos(2*fi);
    speedL = 255;
  }else if(fi >= PI/2 && fi < PI){
    speedR = -255;
    speedL = 255*cos(2*fi-PI);
  }else if(fi >= PI && fi < 3*PI/2){
    speedR = 255*sin(2*fi-5*PI/2);
    speedL = -255;
  }else if(fi >= 3*PI/2 && fi < 2*PI){
    speedR = 255;
    speedL =255*sin(2*fi-7*PI/2);
  } 
  motors((speed*speedL)/256, (speed*speedR)/256);

  if(fi >= 0 && fi < PI/8 || fi >= 7*PI/8 && fi < 2*PI){
    cmd = 0x0A; // FWD
  }else if(fi >= PI/8 < 3*PI/8){
    cmd = 0x08; // FWD Right
  }else if(fi >= 3*PI/8 < 5*PI/8){
    cmd = 0x09; // Right
  }else if(fi >= 5*PI/8 < 7*PI/8){
    cmd = 0x01; // BWD Right
  }else if(fi >= 7*PI/8 < 9*PI/8){
    cmd = 0x05; // BWD 
  }else if(fi >= 9*PI/8 < 11*PI/8){
    cmd = 0x04; // BWD Left
  }else if(fi >= 11*PI/8 < 13*PI/8){
    cmd = 0x06; // Left
  }else if(fi >= 13*PI/8 < 15*PI/8){
    cmd = 0x02; // FWD Left
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  getcmd(myData.x, myData.y);
  Serial.printf("Cmd: %x speedL=%d speedR=%d vx=%d vy=%d\r\n", cmd, speedL, speedR, myData.x, myData.y);
  
  last_data_rcv = millis();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  Serial.println(WiFi.macAddress());
  
  // init the 4 outputs for the motors
  pinMode(out0, OUTPUT);
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  digitalWrite(out0, LOW);
  digitalWrite(out1, LOW);
  digitalWrite(out2, LOW);
  digitalWrite(out3, LOW);

  // init the 2 outputs for the LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(WHITE_LED_PIN, LOW);

  // Init the 4 pwm channels for the motors
  ledcSetup(0, 30000, 8);
  ledcSetup(1, 30000, 8);
  ledcSetup(2, 30000, 8);
  ledcSetup(3, 30000, 8);

  ledcAttachPin(out0, 0);
  ledcAttachPin(out1, 1);
  ledcAttachPin(out2, 2);
  ledcAttachPin(out3, 3);
  
  // Init 2 pwm channels for the LEDs
  ledcSetup(4, 30000, 8);
  ledcSetup(5, 30000, 8);
  ledcAttachPin(GREEN_LED_PIN, 4);
  ledcAttachPin(WHITE_LED_PIN, 5);
  
  // Init the 12 LEDs ring
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

void theaterChaseNB(uint32_t color, int wait) {
  
  static int b = 0;
  static long last_millis = -1;
  
  if(millis()>last_millis + wait){
    //for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
    if(b<3){
        strip.clear();         //   Set all pixels in RAM to 0 (off)
        // 'c' counts up from 'b' to end of strip in steps of 3...
        for(int c=b; c<strip.numPixels(); c += 3) {
          strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
        }
        strip.show(); // Update strip with new contents
        last_millis = millis();
        //delay(wait);  // Pause for a moment
        b++;
    }else{
      b = 0;
    }
  }
}

void theaterChaseNB_CCW(uint32_t color, int wait) {
  
  static int b = 2;
  static long last_millis = -1;
  
  if(millis()>last_millis + wait){
    //for(int b=0; b<3; b++) { //  'b' counts from 2 to 0...
    if(b>=0){
        strip.clear();         //   Set all pixels in RAM to 0 (off)
        // 'c' counts up from end of strip to b in steps of 3...
        for(int c=b; c<strip.numPixels(); c += 3) {
          strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
        }
        strip.show(); // Update strip with new contents
        last_millis = millis();
        //delay(wait);  // Pause for a moment
        b--;
    }else{
      b = 2;
    }
  }
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbowNB(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  
  static long firstPixelHue = 0;
  static long last_millis = -1;
  
  if(millis()>last_millis + wait){
  
    //for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    if(firstPixelHue < 5*65536) {
      // strip.rainbow() can take a single argument (first pixel hue) or
      // optionally a few extras: number of rainbow repetitions (default 1),
      // saturation and value (brightness) (both 0-255, similar to the
      // ColorHSV() function, default 255), and a true/false flag for whether
      // to apply gamma correction to provide 'truer' colors (default true).
      strip.rainbow(firstPixelHue);
      // Above line is equivalent to:
      // strip.rainbow(firstPixelHue, 1, 255, 255, true);
      strip.show(); // Update strip with new contents
      //delay(wait);  // Pause for a moment
      firstPixelHue += 256;
    }else{
      firstPixelHue = 0;
    }
    last_millis = millis();
  }
}


// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbowNB(int wait) {
  static int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  static long last_millis = -1;
  static int b = 0;

  if(millis() > last_millis + wait){
    //for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
    if(b<3){
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      //delay(wait);                 // Pause for a moment
      last_millis = millis();
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
      b++;
    }else
      b = 0;

  }
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipeNB(uint32_t color, int wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   //  Update delay time
  strip.setPixelColor(pixelCurrent, color); //  Set pixel's color (in RAM)
  strip.show();                             //  Update strip to match
  pixelCurrent++;                           //  Advance current pixel
  if(pixelCurrent >= pixelNumber)           //  Loop the pattern from the first LED
    pixelCurrent = 0;
}

// Change max brightness and min flickering period for left and right LEDs
void flashleds(int ledl_min_period_ms, int ledl_max_brightness, int ledr_min_period_ms, int ledr_max_brightness){
    
  static long ledl_last_ms = 0;
  static long ledr_last_ms = 0;
  static int ledl_status = 1;
  static int ledr_status = 1;
  static uint32_t ledl_period_ms = 1;
  static uint32_t ledr_period_ms = 1;

  // manage the left eye LED    
  if(millis() > ledl_last_ms + ledl_period_ms){
  
    // get a brightness between 0 and max_brightness
    uint32_t ledl_brightness =  ( (esp_random() >> 24 ) * ledl_max_brightness ) / 255;
  
    //Serial.printf("ledl_brightness=%d ledl_period=%d\r\n", ledl_brightness, ledl_period_ms);

    if(ledl_status == 0){
      ledcWrite(5, ledl_brightness);
      ledl_status = 1;
    }else{
      ledcWrite(5, 0);
      ledl_status = 0;
    }    

    // get a period between 300 msecs and min_period_ms
    ledl_period_ms = ledl_min_period_ms + ( (esp_random() >> 24) * (300 - ledl_min_period_ms) ) / 255;
    ledl_last_ms = millis();  
  }
  
  
  // manage the right eye LED    
  if(millis() > ledr_last_ms + ledr_period_ms){

    // get a brightness between 0 and max_brightness
    uint32_t ledr_brightness =  ( (esp_random() >> 24 ) * ledr_max_brightness ) / 255;
  
    //Serial.printf("ledr_brightness=%d ledr_period=%d\r\n", ledr_brightness, ledr_period_ms);

    if(ledr_status == 0){
      ledcWrite(4, ledr_brightness);
      ledr_status = 1;
    }else{
      ledcWrite(4, 0);
      ledr_status = 0;
    }    

    // get a period between 1000 msecs and min_period_ms
    ledr_period_ms = ledr_min_period_ms + ( (esp_random() >> 24) * (300 - ledr_min_period_ms) ) / 255;
  
    ledr_last_ms = millis();  
  }

}

void loop() {

  if(millis() > last_data_rcv + 1000){
     // not receving anymore
     cmd = 0x00;              // stop the motors   
     strip.setBrightness(5); // set brightness low 

      // Eyes LEDs
      flashleds(10,5,10,5);
      
  }else{
     // ... and set BRIGHTNESS medium (max = 255)
      strip.setBrightness(50); 
      // Flowing rainbow cycle along the whole ring
      //rainbowNB(1);             
      
      // Eyes LEDs
      flashleds(10,250,10,250);
  }
     
  //Serial.printf("cmd=%x\r\n",cmd);
  switch(cmd){
    case 0x00: 
      // ... stop the motors ...
      ledcWrite(0, 0);   
      ledcWrite(1, 0);   
      ledcWrite(2, 0);   
      ledcWrite(3, 0);   
      // Flowing rainbow cycle along the whole ring
      rainbowNB(5);             
      break;
    case 0x0A:                    // FWD
      theaterChaseNB(strip.Color(127,   127,   127), 50); // White, half brightness      
      break;
    case 0x02:                    // FWD LEFT
      theaterChaseNB(strip.Color(127,   0,   0), 50); // Red, half brightness      
      break;
    case 0x08:                    // FWD RIGHT
      theaterChaseNB_CCW(strip.Color(0,  127,   0), 50); // Green, half brightness      
      break;
    case 0x06:                    // LEFT
      theaterChaseNB(strip.Color(0,  0,   127), 50); // Green, half brightness      
      break;
    case 0x09:                    // RIGHT
      theaterChaseNB_CCW(strip.Color(0,  127,   127), 50); // Green Blue, half brightness      
      break;
    case 0x05:                    // BWD
      theaterChaseRainbowNB(50); 
      break;
    case 0x04:                    // LEFT BWD
      theaterChaseNB(strip.Color(127,   0,  127), 50); // Red Blue, half brightness      
      break;
    case 0x01:                    // RIGHT BWD
      theaterChaseNB_CCW(strip.Color(128,   0,   32), 50); // Red Green Blue, half brightness      
      break;
  }
  if(myData.cmd &0x10){
      rainbowNB(1);
  }

}