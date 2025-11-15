//Include libraires
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//Define for TFT screen
#define TFT_CS        7
#define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         6
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 10  // Clock out
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
uint16_t Var1 = 0;
uint8_t accellpedal = 0;
uint8_t brakepedal = 0;

double rpm = 0;
double rpm_max = 9000;
int engine_sts = 0;
bool was_black = true;

struct can_frame buttonMsg;
struct can_frame PdlMsg;
struct can_frame PotMsg;
struct can_frame veh_sw_information;
unsigned long veh_sw_information_time = 0;
int veh_sw_information_mm = 0;
int veh_sw_information_mm_sts = 0;


MCP2515 mcp2515(4);


// Digital button struct
struct d_btn_str {
  const int button_pin;
  int current_reading;
  uint16_t button_state;
  int last_button_state;
  unsigned long lastDebounceTime;
  unsigned long debounceTime;
  String button_name;
};

//Potentiometer reading
struct anl_pot_str {
  const int button_pin;
  float min_range;
  float max_range;
  int percentage;
};

// analog button struct
struct anl_btn_str {
  const int button_pin;
  int position_reading;
  float partitor_number;
  int old_position_reading[4];
};

//Timer set
unsigned long Timer_10_Millis = 0;
unsigned long Timer_100_Millis = 0;
unsigned long Timer_1000_Millis = 0;
unsigned long Timer_2000_Millis = 0;

//Msg timer
unsigned long lastTimeMsg1 = 0;
unsigned long lastTimeMsg2 = 0;
unsigned long lastTimeMsg3 = 0;

//Digital read button setup
d_btn_str dbtn_1 = {8, LOW, HIGH, HIGH, 0, 1000,  "1"}; //paddle 1
d_btn_str dbtn_2 = {9, LOW, HIGH, HIGH, 0, 1000, "2"}; //paddle 2
d_btn_str dbtn_3 = {1, LOW, HIGH, HIGH, 0, 1000, "3"};
d_btn_str dbtn_4 = {2, LOW, HIGH, HIGH, 0, 1000, "4"};
d_btn_str dbtn_5 = {3, LOW, HIGH, HIGH, 0, 1000, "5"};
d_btn_str dbtn_6 = {5, LOW, HIGH, HIGH, 0, 1000, "6"};
d_btn_str dbtn_7 = {A5, LOW, HIGH, HIGH, 0, 1000, "7"};
d_btn_str dbtn_8 = {A4, LOW, HIGH, HIGH, 0, 1000, "8"};
//Analog read button setup
anl_btn_str abtn_1 = {A3, 0, 5.0, {0, 0, 0, 0}};
anl_btn_str abtn_2 = {A2, 0, 5.0, {0, 0, 0, 0}};

//MultiState selector setup
anl_btn_str mltsts_1 = {21, 0, 6.0, {0, 0, 0, 0}};
anl_btn_str mltsts_2 = {14, 0, 6.0, {0, 0, 0, 0}};

//Analog potentiometer setup
anl_pot_str a_pot_1 = {A6, 0, 44, 0};
anl_pot_str a_pot_2 = {A1, 932, 1023, 0};

void setup() {

  //Screen Init
  tftscreeninit();
//  welcomescreen(&a_pot_1, &a_pot_2);
  statictextwrite();
  writegeartoeScreen('N', 55, 40);
  // Init can TX frame
  init_canframe_fcn();

  pinMode(dbtn_1.button_pin, INPUT);
  pinMode(dbtn_2.button_pin, INPUT);
  pinMode(dbtn_3.button_pin, INPUT);
  pinMode(dbtn_4.button_pin, INPUT);
  pinMode(dbtn_5.button_pin, INPUT);
  pinMode(dbtn_6.button_pin, INPUT);
  pinMode(A1, INPUT_PULLUP);
  //  pinMode(A5, INPUT);
  //  pinMode(A4, INPUT);

  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();


}

void loop() {
  // Timer 2 sec
  if (millis() - Timer_2000_Millis >= 100) {
    Timer_2000_Millis = millis();
    writepedalgauge(a_pot_2.percentage, a_pot_1.percentage);
  }

  // Timer 100 msec
  if (millis() - Timer_100_Millis >= 10) {
    Timer_100_Millis = millis();

    PdlMsg.data[0] = dbtn_1.button_state;
    PdlMsg.data[1] = dbtn_2.button_state;
    PdlMsg.data[2] = (analogRead(mltsts_1.button_pin) / 1023.0) * 5.0;
    PdlMsg.data[3] = (analogRead(mltsts_2.button_pin) / 1023.0) * 5.0;
    PdlMsg.data[4] = ((analogRead(A4) / 1023.0) * 5.0 >= 4);
    PdlMsg.data[5] = dbtn_5.button_state;
    PdlMsg.data[6] = dbtn_6.button_state;
    PdlMsg.data[7] = ((analogRead(A5) / 1023.0) * 5.0 >= 4);
    //Pot can message
    PotMsg.data[0] = a_pot_1.percentage;
    PotMsg.data[1] = a_pot_2.percentage;
    //Button
    buttonMsg.data[0] = (abtn_1.position_reading == 0);
    buttonMsg.data[1] = (abtn_1.position_reading == 1);
    buttonMsg.data[2] = (abtn_1.position_reading == 2);
    buttonMsg.data[3] = (abtn_1.position_reading == 3);
    buttonMsg.data[4] = (abtn_2.position_reading == 0);
    buttonMsg.data[5] = (abtn_2.position_reading == 1);
    buttonMsg.data[6] = (abtn_2.position_reading == 2);
    buttonMsg.data[7] = (abtn_2.position_reading == 3);


//    Serial.print("position reading -->  ");
//    Serial.println(abtn_2.position_reading);
  }

  // Timer 10 msec
  if (millis() - Timer_10_Millis >= 100) {
    Timer_10_Millis = millis();

    analogprocessing_fcn(&abtn_1);
    analogprocessing_fcn(&abtn_2);


    digital_debounce(&dbtn_1);
    digital_debounce(&dbtn_2);
    digital_debounce(&dbtn_3);
    digital_debounce(&dbtn_4);
    digital_debounce(&dbtn_5);
    digital_debounce(&dbtn_6);
    digital_debounce(&dbtn_7);
    Anl_pot_processing(&a_pot_1);
    Anl_pot_processing(&a_pot_2);
    // read multistate switch

  }

  //CAN WRITE
  if (millis() - lastTimeMsg1 >= 200) {
    lastTimeMsg1 = millis();
    mcp2515.sendMessage(&PotMsg);
  }

  if (millis() - lastTimeMsg2 >= 200 && millis() - lastTimeMsg1 > 66) {
    lastTimeMsg2 = millis();
    mcp2515.sendMessage(&PdlMsg);
  }

  if (millis() - lastTimeMsg3 >= 200 && millis() - lastTimeMsg1 > 130) {
    lastTimeMsg3 = millis();
    mcp2515.sendMessage(&buttonMsg);
  }

  //CAN READ
  if (mcp2515.readMessage(&veh_sw_information) == MCP2515::ERROR_OK)
  {
    if (veh_sw_information.can_id == 0x0F7) {
      // RPM
      writeRPMtoScreen(min(9500,veh_sw_information.data[1]*50), 65, 10, 8500);
      // Speed
      writetmptoScreen(min(500,veh_sw_information.data[2]*2), 45, 85, 300, ST7735_WHITE);
      // Drv and Pwt
      writemodetoScreen(veh_sw_information.data[3] & 0b00001111, (veh_sw_information.data[3] & 0b11110000)>>4);
      // Gear egear
      writetmptoScreen(min(8,((veh_sw_information.data[4] & 0b11110000)>>4)), 95, 90, 10, ST7735_RED);
      // Gear ice
      writetmptoScreen(min(8,veh_sw_information.data[4] & 0b00001111), 95, 55, 10, ST7735_RED);
      // PRNDM
      writegeartoeScreen(min(7,(veh_sw_information.data[5] & 0b11110000)>>4), 55, 40);
      // Temp Oil
      writetmptoScreen(min(255,veh_sw_information.data[6]), 10, 90, 85, ST7735_BLUE);
      // Temp Coolant
      writetmptoScreen(min(255,veh_sw_information.data[7]), 10, 55, 95, ST7735_BLUE);

    }
  }


}


void digital_debounce(struct d_btn_str* r)
{
  if (digitalRead(r->button_pin) == HIGH && r->last_button_state == HIGH) {
    r->button_state = HIGH;
  }
  else {
    r->button_state = LOW;
  }
  r->last_button_state = digitalRead(r->button_pin);
}

//Process analog reading from button
void analogprocessing_fcn(struct anl_btn_str* r)
{
  float button_current_reading = analogRead(r->button_pin);
  if (button_current_reading <= 10) r->position_reading = 0;
  else if (button_current_reading >= 650 && button_current_reading <= 670) r->position_reading = 1;
  else if (button_current_reading >= 870 && button_current_reading <= 890) r->position_reading = 2;
  else if (button_current_reading >= 980 && button_current_reading <= 1000) r->position_reading = 3;
  else  r->position_reading = 10;
}


void Anl_pot_processing(struct anl_pot_str *r) {
  r->percentage = map(analogRead(r->button_pin), r->min_range, r->max_range, 0, 100);


  if (r->percentage >= 98) {
    r->percentage = 100;
  }
  else if (r->percentage <= 3) {
    r->percentage = 0;
  }
}



void init_canframe_fcn() {

  // Init buttonMsg
  buttonMsg.can_id  = 0x016;
  buttonMsg.can_dlc = 8;
  buttonMsg.data[0] = 0x00;
  buttonMsg.data[1] = 0x00;
  buttonMsg.data[2] = 0x00;
  buttonMsg.data[3] = 0x00;
  buttonMsg.data[4] = 0x00;
  buttonMsg.data[5] = 0x00;
  buttonMsg.data[6] = 0x00;
  buttonMsg.data[7] = 0x00;
  // Init PdlMsg
  PdlMsg.can_id  = 0x036;
  PdlMsg.can_dlc = 8;
  PdlMsg.data[0] = 0x00;
  PdlMsg.data[1] = 0x00;
  PdlMsg.data[2] = 0x00;
  PdlMsg.data[3] = 0x00;
  PdlMsg.data[4] = 0x00;
  PdlMsg.data[5] = 0x00;
  // Init PotMsg
  PotMsg.can_id = 0x026;
  PotMsg.can_dlc = 2;
  PotMsg.data[0] = 0x00;
  PotMsg.data[1] = 0x00;
}


void tftscreeninit() {
  tft.initR(INITR_BLACKTAB);  // Initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);  // Fill screen with black
  tft.setRotation(2);  // Set orientation of the display. Values are from 0 to 3. If not declared, orientation would be 0,
  // which is portrait mode.
  tft.setTextWrap(false);  // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
  // To override this behavior (so text will run off the right side of the display - useful for
  // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
  // with setTextWrap(true).
}

void statictextwrite() {
  //RPM
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(10, 13);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("RPM ENG");  // Print a text or value
  // Top Line
  tft.drawLine(0, 30, 128, 30, ST7735_WHITE);  // Draw line (x0,y0,x1,y1,color)
  tft.drawLine(0, 110, 128, 110, ST7735_WHITE);  // Draw line (x0,y0,x1,y1,color)
  tft.drawLine(0, 135, 128, 135, ST7735_WHITE);  // Draw line (x0,y0,x1,y1,color)

  //_TWater
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(10, 40);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("TCool");  // Print a text or value

  //_TOil
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(10, 75);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("TOil");  // Print a text or value

  //_ICEgear
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(80, 40);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("ICEGear");  // Print a text or value

  //EGear
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(85, 75);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("EGear");  // Print a text or value

  //kph
  tft.setTextSize(1);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
  tft.setCursor(48, 70);  // Set position (x,y)
  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
  tft.println("SPEED");  // Print a text or value



  //
  tft.fillRect(64, 135, 64, 25, ST7735_GREEN);
  tft.fillRect(0, 135, 64, 25, ST7735_RED);
}

void writeRPMtoScreen(uint16_t rpm, int16_t  x0, int16_t y0, int16_t const RPM_Lim) {
  char string[10];  // Create a character array of 10 characters
  // Convert int to a string:
  dtostrf(rpm, 3, 0, string);  // (<rpm>,<amount of digits we are going to use>,<amount of decimal digits>,<string name>)
  tft.setCursor(x0, y0);  // Set position (x,y)
  if (rpm >= RPM_Lim) {
    tft.setTextColor(ST7735_RED, ST7735_BLACK);  // Set color of text. First is the color of text and after is color of background
  }
  else
  {
    tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);  // Set color of text. First is the color of text and after is color of background

  }
  tft.setTextSize(2);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.println(rpm);  // Print a text or value

  if (rpm < 10) // If rpm1 is less than 100...
  {
    // Fill the other digit with background color:
    tft.fillRect(76, y0, 12, 18, ST7735_BLACK);  // Draw filled rectangle (x,y,width,height,color)
  }
  if (rpm < 100) // If rpm1 is less than 100...
  {
    // Fill the other digit with background color:
    tft.fillRect(88, y0, 12, 18, ST7735_BLACK);  // Draw filled rectangle (x,y,width,height,color)
  }
  if (rpm < 1000) // If rpm1 is less than 100...
  {
    // Fill the other digit with background color:
    tft.fillRect(100, y0, 12, 18, ST7735_BLACK);  // Draw filled rectangle (x,y,width,height,color)
  }

}


void writetmptoScreen(uint16_t tmp, int16_t  x0, int16_t y0, int16_t const tmpLim, uint16_t color) {
  char string[3];  // Create a character array of 10 characters
  // Convert int to a string:
  dtostrf(tmp, 3, 0, string);  // (<rpm>,<amount of digits we are going to use>,<amount of decimal digits>,<string name>)
  tft.setCursor(x0, y0);  // Set position (x,y)
  if (tmp >= tmpLim) {
    tft.setTextColor(ST7735_RED, ST7735_BLACK);  // Set color of text. First is the color of text and after is color of background
  }
  else
  {
    tft.setTextColor(color, ST7735_BLACK);  // Set color of text. First is the color of text and after is color of background

  }
  tft.setTextSize(2);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
  tft.println(tmp);  // Print a text or value

  if (tmp < 10) // If rpm1 is less than 100...
  {
    // Fill the other digit with background color:
    tft.fillRect(x0 + 11, y0, 12, 18, ST7735_BLACK); // Draw filled rectangle (x,y,width,height,color)
  }
  if (tmp < 100) // If rpm1 is less than 100...
  {
    // Fill the other digit with background color:
    tft.fillRect(x0 + 23, y0, 12, 18, ST7735_BLACK); // Draw filled rectangle (x,y,width,height,color)
  }

}

void writegeartoeScreen(uint8_t gear, int16_t  x0, int16_t y0) {
  switch(gear)
  {
  case 0:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('N');  // Print a text or value
    break;
  case 1:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('P');  // Print a text or value
    break;
  case 2:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('R');  // Print a text or value
    break;
  case 4:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('D');  // Print a text or value
    break;
  case 5:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('M');  // Print a text or value
  break;
  case 6:  
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_BLACK);
    tft.println(' ');  // Print a text or value
    tft.setCursor(x0, y0);  // Set position (x,y)
    tft.setTextSize(3);  // Set text size. Goes from 0 (the smallest) to 20 (very big)
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.println('M');  // Print a text or value
  break;
  }

}

void writemodetoScreen(uint8_t drivemode, uint8_t pwtmode) {
  //DriveMode
  tft.setTextSize(2);  // Set text size. We are using custom font so you should always set text size as 0
  // Write to the display the text "World":
//  tft.setCursor(5, 116);  // Set position (x,y)
//  tft.setTextColor(ST7735_WHITE);  // Set color of text. We are using custom font so there is no background color supported
   switch (drivemode){
    case 0:
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.println("     ");
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.println("COMFR");  // Print a text or value
      break;
    case 2:
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.print("     ");
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print("SPORT");
      break;
    case 1:
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.print("     ");
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print("ECHAR");
      break;
    case 4:
      tft.setCursor(5, 116);  
      tft.setTextColor(ST7735_BLACK);
      tft.print("     ");
      tft.setCursor(5, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print("TRACK");
      break;
  }
   switch (pwtmode){
    case 0:
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.print("   ");
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.println("HYB");  // Print a text or value
      break;
    case 1:
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.print("   ");
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print("EV ");
      break;
    case 2:
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_BLACK);
      tft.print("   ");
      tft.setCursor(80, 116);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print("ICE");
      break;
  }
}

void writepedalgauge(uint8_t accelpedal, uint8_t brakepedal) {

  //map(value, fromLow, fromHigh, toLow, toHigh)
  long mapped_accel = map(accelpedal, 0, 100, 0, 64);
  long mapped_brk = map(brakepedal, 100, 0, 0, 64);

  tft.fillRect(64 + mapped_accel, 135, 64 - mapped_accel, 25, ST7735_BLACK);
  tft.fillRect(64, 135, mapped_accel, 25, ST7735_GREEN);

  tft.fillRect(mapped_brk, 135, 64 - mapped_brk, 25, ST7735_RED);
  tft.fillRect(0, 135, mapped_brk, 25, ST7735_BLACK);


}



void welcomescreen(struct anl_pot_str *pot1, struct anl_pot_str *pot2) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 30);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("LB3D_SW");

  testdrawtext("This is not inteded for racing purpose.\nDesigned and Engineered in Rubiera, Italy", ST77XX_WHITE);
  unsigned long previous_time = millis();
  while (millis() - previous_time <= 5000) {
    //do nothing
  }
  previous_time = millis();
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("CAN 125 BPS \nSW version V1.0", ST77XX_WHITE);
  while (millis() - previous_time <= 2500) {
    //do nothing
  }
  previous_time = millis();
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("LOADING...", ST77XX_WHITE);
  //loading screen
  uint16_t w = 0;
  while (w <= 128) {
    while (millis() - previous_time <= 150) {
      //do nothing
    }
    tft.fillRect(0, 135, w , 25, ST7735_GREEN);
    w += 5;
    previous_time = millis();
  }
  tft.fillScreen(ST77XX_BLACK);
  //Potentiometer calibration
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 30);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("PADDLE CALIBRATION - MIN");
  testdrawtext("Dont press neither of the paddle pot", ST77XX_WHITE);
  previous_time = millis();
  while (millis() - previous_time <= 5000) {
    //do nothing, wait
  }
  pot1->min_range = analogRead(pot1->button_pin) + 10;
  pot2->min_range = analogRead(pot2->button_pin) + 10;
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 30);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.println("PADDLE CALIBRATION - MAX");
  testdrawtext("Press both the paddles to the max", ST77XX_WHITE);
  previous_time = millis();
  while (millis() - previous_time <= 5000) {
    //do nothing, wait
  }
  pot1->max_range = analogRead(pot1->button_pin) - 10;
  pot2->max_range = analogRead(pot2->button_pin) - 10;
  tft.fillScreen(ST77XX_BLACK);
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 70);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.setTextSize(1);
  tft.print(text);
}
