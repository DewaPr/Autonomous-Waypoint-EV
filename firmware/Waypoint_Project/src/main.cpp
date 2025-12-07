#include <SPI.h>
#include <wireAlgorithm.h>
#include <sdAlgorithm.h>
#include <compassAlgorithm.h>
#include <oledStrAlgoritm.h>
#include <canAlgorithm.h>
#include <rs485Algorithm.h>
#include <function.h>
#include <EEPROM.h>

unsigned long quarterSecondUpdate = 0;
unsigned long logUpdate = 0;
unsigned long steerUpdateTime = 0;
signed long prevLong = 0;
signed long prevLat = 0;
signed long nowLong = 0;
signed long nowLat = 0;

byte carStatus = 255;
byte lastPathID;
bool lastRec;

bool transCmdSendToggle = false;

bool lastRecordPathFlag = 1;

void setup()
{
  Serial.setRx(PA3);
  Serial.setTx(PA2);
  Serial.begin(115200);
  Serial.println("Serial alive!");

  rs485.begin(115200);

  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  
  Serial.println("CAN Begin!");
  canBegin();

  Serial.println("SD Begin!");
  sdCardBegin();

  Serial.println("Compass Begin!");
  compassBegin();
  setDeclinationAngle((0.0 + (45.0 / 60.0)) / (180 / PI));

  Serial.println("Oled Begin!");
  oledDisplayBegin();

  Serial.println("Here3 Begin!");
  setLedBrightness(1);
  
  // String _linePathData = sdReadFileByLine("path1.txt", 0);
  // Serial.print("First Line Path data: ");
  // Serial.println(_linePathData);

  // delay(5000);

  Serial.println("Loop Started!");
  
  pinMode(PA0, INPUT_PULLUP);
  // sdWriteFile("calHard.txt", "40|103|-28", false);
  // sdWriteFile("path1.txt", "", false);

}

void loop(void)
{
  String oledStr = "";
  String str = "";
  float heading = 0;
  float idealHead = 0;

  // if(!digitalRead(PA0)) calibrateCompass();
  
  rs485Check();
  canCheck();
  updateHeading();

  if((carStatus != getCarStatus() && getCarStatus() <= 3) || lastPathID != getPathID() || lastRec != recordPath()){
    lastRec = recordPath();
    carStatus = getCarStatus();
    lastPathID = getPathID();
    transCmdSendToggle = true;
  }
  else if(transCmdSendToggle){
    transCmdSendToggle = false;
    
    u8g2.setFontDirection(1);
    if(carStatus == 0){
      u8g2.setDrawColor(0);
      u8g2.drawBox(120, 0, 8, 40);
      u8g2.setDrawColor(1);
      String _strTmp = "M" + String(lastPathID);
      if(lastRec) _strTmp += " Rec!";
      u8g2.drawStr(128, 0, _strTmp.c_str());
      canUpdate(0x01, 0);
      canUpdate(0x02, 0);
    }
    else if(carStatus == 1){
      u8g2.setDrawColor(0);
      u8g2.drawBox(120, 0, 8, 40);
      u8g2.setDrawColor(1);
      String _strTmp = "S" + String(lastPathID);
      u8g2.drawStr(128, 0, _strTmp.c_str());
      canUpdate(0x01, 0);
      canUpdate(0x02, 0);
      resetPathCount();
    }
    else if(carStatus == 3){
      u8g2.setDrawColor(0);
      u8g2.drawBox(120, 0, 8, 40);
      u8g2.setDrawColor(1);
      String _strTmp = "T" + String(lastPathID);
      u8g2.drawStr(128, 0, _strTmp.c_str());
    }
    u8g2.setFontDirection(0);
  }
  

  if (wireUpdate())
  {
    // str = String(getAccData(0)) + "\t" + String(getAccData(1)) + "\t" + String(getAccData(2));
    // Serial.println(str);

    // Serial.println(getHorAcc());
    
    updateDisplay(56, 0, getFixData());

    bool validLongLat = true;
    for (int i = 0; i < 2; i++)
    {
      oledStr = "";
      if (!getKalmanLatLong(i))
      {
        oledStr = "--         ";
        validLongLat = false;
      }
      else
      {
        oledStr = longToFloatStr(getKalmanLatLong(i));
        if (fabs(getKalmanLatLong(i)*1e-7) < 100)
          oledStr += " ";
        if (fabs(getKalmanLatLong(i)*1e-7) < 10)
          oledStr += " ";
      }
      if (!i)
        updateDisplay(56, 8, oledStr);
      else
        updateDisplay(56, 16, oledStr);
    }

    if (validLongLat)
    {
      oledStr = "         ";
      updateDisplay(56, 48, oledStr);
      oledStr = String(getHorAccuracy(), 2) + " m";
 
      // u8g2.setDrawColor(0);
      // u8g2.drawBox(120, 0, 8, 40);
      // u8g2.setDrawColor(1);
      // String _strTmp;
      // if(recordPath()){
      //   _strTmp = "M" + String(getPathID()) + "Rec!";
      // }
      // else{
      //   _strTmp = "M" + String(getPathID());
      // }
      // u8g2.drawStr(128, 0, _strTmp.c_str());
      
      if(carStatus == 0){
        if(lastRecordPathFlag != recordPath() && recordPath()){
          String _pathStr = "path" + String(getPathID()) + ".csv";
          bool _dataWriten = sdWriteFile(_pathStr, "", false);

          Serial.println("Record Data at Path: " + String(getPathID()) + ", " + _dataWriten);

          prevLat = getKalmanLatLong(1);
          prevLong = getKalmanLatLong(0);
          /////////////////////////////////////////////////////////////////////////////made change here today
          String _sdCardGnssDumpData = longToFloatStr(getKalmanLatLong(0)) + "," 
                                    + longToFloatStr(getKalmanLatLong(1)) + "\n"
                                    ;
                                    
          sdWriteFile(_pathStr, _sdCardGnssDumpData, true);
        }
        
        lastRecordPathFlag = recordPath();

        if(calculateEarthDistance(prevLat, prevLong, getKalmanLatLong(1), getKalmanLatLong(0)) > 3 && recordPath()){
          prevLat = getKalmanLatLong(1);
          prevLong = getKalmanLatLong(0);

          String _sdCardGnssDumpData = longToFloatStr(getKalmanLatLong(0)) + "," 
                                    + longToFloatStr(getKalmanLatLong(1)) + "\n"
                                    ;
          // Serial.print(_sdCardGnssDumpData);
          
          String _pathStr = "path" + String(getPathID()) + ".csv";
          sdWriteFile(_pathStr, _sdCardGnssDumpData, true);
        }
      }      
    }
    else
    {
      oledStr = "--      ";      
    }
    updateDisplay(56, 48, oledStr);

    idealHead = getIdealHead();
    
    if (idealHead < 0)
      idealHead += 2 * PI;
    if (idealHead > 2 * PI)
      idealHead -= 2 * PI;
    idealHead *= 180.0 / PI;

    heading =  getHeading();
    
    if (heading < 0)
        heading += 2 * PI;
    if (heading > 2 * PI)
        heading -= 2 * PI;
    heading *= 180 / PI;
    
    if(carStatus != 2) idealHead = 0;


    // int headOff = heading - idealHead;
    int headOff = getMinDeg(getIdealHead(), getHeading());
    // Serial.print(getHeading());
    // Serial.print(" ");
    // Serial.print(getIdealHead());
    // Serial.print(" ");
    // Serial.println(headOff);
    // if(headOff > 180) headOff -= 360;
    
    // if(steerUpdateTime < millis()){
    //   steerUpdateTime = millis() + 100;
    //   setSteering(headOff);
    // }

    // if(headOff){
    oledStr = String(headOff) + " deg";
    if (abs(headOff) < 100)
      oledStr += " ";
    if (abs(headOff) < 10)
      oledStr += " ";
    // }
    // else{
    //   oledStr = "--      ";
    // }
    updateDisplay(56, 40, oledStr);

    float spdTmp = getKalmanSpeed(1);
    oledStr = "";
    if (!spdTmp)
      oledStr = "--      ";
    else
    {
      oledStr = String(spdTmp, 1) + " km/h";
      if (fabs(spdTmp) < 100)
        oledStr += " ";
      if (fabs(spdTmp) < 10)
        oledStr += " ";
    }
    updateDisplay(10, 56, oledStr);
  }

  drawLineAngle(115, 51, 10, idealHead - heading, true);
  if(carStatus == 1){
    canUpdate(0x01, (1 << 9));
    canUpdate(0x02, (1 << 13));    
  }
  else if(carStatus == 2) calculateMovement();  
  if(carStatus != 2) updateNextLatLong(0, 0);

  if (quarterSecondUpdate < millis())
  {
    quarterSecondUpdate = millis() + 250;
    oledStr = "";

    float battVoltage = readBattery();
    oledStr = String(battVoltage, 1) + "v";
    if (battVoltage < 10)
      oledStr += " ";    
    
    updateDisplay(70, 56, oledStr);
  }
  displaySendBuffer();

  String _serialTxt = "";
  while (Serial.available())
  {
    _serialTxt += (char)Serial.read();
    _serialTxt.trim();
    // Serial.println(_serialTxt);
  }
  if(_serialTxt[0] == 's'){
    _serialTxt.remove(0,1);
    int _steeringL = _serialTxt.toInt();

    bool _steeringDir;
    if(_steeringL < 0) _steeringDir = 1;
    else _steeringDir = 0;

    int _canData = (1 << 9) | (_steeringDir << 8) | abs(_steeringL);
    canUpdate(0x01, _canData);
    
    Serial.println("Steering: " + String(_steeringL) + "deg, Dir:" + String(_steeringDir));
  }
  else if(_serialTxt[0] == 'd'){
    _serialTxt.remove(0,1);
    int _drive = _serialTxt.toInt();

    bool _driveOrBrake;
    if(_drive < 0) _driveOrBrake = 1;
    else _driveOrBrake = 0;
    _drive = abs(_drive);

    int _canData = (1 << 13) | (_driveOrBrake << 12) | _drive;
    canUpdate(0x02, _canData);
    
    Serial.println("Drive: " + String(_drive) + "Dir:" + String(_driveOrBrake));
  }
  else if (_serialTxt == "calCps")
    calibrateCompass();
  
  else if (_serialTxt == "resetEep"){
    Serial.println("Reseting EEPROM");
    EEPROM[5] = 0;
    Serial.println("Id Back To: " + String(EEPROM[5]));
  }

  // delay(1000);
}