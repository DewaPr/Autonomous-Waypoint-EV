#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

byte pathID = 0;

struct can_frame canMsg;
MCP2515 mcp2515(PA4);

byte carDataStatus = 1;

bool recordPathFlag = 0;

void canBegin(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void canUpdate(byte _id, int _data){
  canMsg.can_id  = _id;
  canMsg.can_dlc = 2;
  canMsg.data[0] = (_data >> 8) & 0xff;
  canMsg.data[1] = _data & 0xff;

  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK){
    // Serial.println("Message Sent!");
  }
  else{
    // Serial.println("Message Not Sent!");
  }
}

void canCheck(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    Serial.print(canMsg.data[0],HEX);
    Serial.print(" ");
    
    if(canMsg.can_id == 0x00){
        if(canMsg.data[0] < 0x05){
          recordPathFlag = 0;

          if(carDataStatus == 1){
            carDataStatus = canMsg.data[0];
          }
          else{
            if(canMsg.data[0] == 0x01) carDataStatus = canMsg.data[0];
          }
        }
        else if(canMsg.data[0] >= 0x10 && carDataStatus == 1) pathID = canMsg.data[0] - 0x10;
        
        if(canMsg.data[0] == 0x05) recordPathFlag = 1;
        else if(canMsg.data[0] == 0x08) recordPathFlag = 0;
    }
    Serial.println();
  }
}

void setCarStatus(int _a){
  carDataStatus = _a;
}

byte getCarStatus(){
    return carDataStatus;
}

byte getPathID(){
    return pathID;
}

bool recordPath(){
  return recordPathFlag;
}