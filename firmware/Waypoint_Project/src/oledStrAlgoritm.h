#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void drawLineAngle(byte _x, byte _y, byte _h, float _deg, bool _e){
  _deg += 2;
  if(_e){
    u8g2.setDrawColor(0);
    u8g2.drawDisc(_x, _y, _h, U8G2_DRAW_ALL);
    u8g2.setDrawColor(1);
  }
  _h++;
  u8g2.drawDisc(_x, _y, 5, U8G2_DRAW_ALL);
  u8g2.setDrawColor(0);
  u8g2.drawDisc(_x, _y, 4, U8G2_DRAW_ALL);
  u8g2.setDrawColor(1);
  u8g2.drawDisc(_x, _y, 1, U8G2_DRAW_ALL);
  for(int i = _deg - 9; i <= _deg + 9; i += 3){
    u8g2.drawLine(_x, _y, _x + _h * sin(i * PI / 180), _y - _h * cos(i * PI / 180));
  }
}

void updateDisplay(byte _horz, byte _vert, String _str){ 
    u8g2.setDrawColor(0);
    u8g2.drawBox(_horz, _vert + 1, 5 * _str.length(), 8);
    u8g2.setDrawColor(1);

    u8g2.drawStr(_horz, _vert, _str.c_str());
}

void displaySendBuffer(){
    u8g2.sendBuffer();
}

void oledDisplayBegin(){
  u8g2.begin();
  u8g2.clearBuffer();
  
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  
  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.drawStr(0, 0,  "GPS Status:");
  u8g2.drawStr(0, 8,  "Now Long  : ");
  u8g2.drawStr(0, 16, "Now Lat   :");
  u8g2.drawStr(0, 24, "Next Long :");
  u8g2.drawStr(0, 32, "Next Lat  :");
  u8g2.drawStr(0, 40, "Off Head  :");
  u8g2.drawStr(0, 48, "Land Acc  :");
  u8g2.drawStr(0, 56, "S:");
  u8g2.drawStr(60, 56, "B:");
    
  u8g2.drawStr(56, 0,  "--");
  u8g2.drawStr(56, 8,  "--");
  u8g2.drawStr(56, 16, "--");
  u8g2.drawStr(56, 24, "--");
  u8g2.drawStr(56, 32, "--");
  u8g2.drawStr(56, 40, "--");
  u8g2.drawStr(56, 48, "--");
  u8g2.drawStr(10, 56, "--");
  u8g2.drawStr(70, 56, "--");

  u8g2.setFontDirection(1);
  u8g2.drawStr(128, 0, "S");
  u8g2.setFontDirection(0);  

  u8g2.drawLine(114, 30, 114, 35);
  u8g2.drawLine(115, 30, 115, 36);
  u8g2.drawLine(116, 30, 116, 35);
  u8g2.drawLine(113, 34, 117, 34);
  u8g2.drawLine(112, 33, 118, 33);

  u8g2.drawDisc(115, 51, 12, U8G2_DRAW_ALL);

  drawLineAngle(115, 51, 10, 0, true);
  u8g2.sendBuffer();
}