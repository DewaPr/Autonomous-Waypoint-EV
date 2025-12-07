#include <Arduino.h>
#include <EEPROM.h>

float _voltage[3] = {0};
float idealHeadCal = 0;

signed long nextLat = 0;
signed long nextLng = 0;
signed long pathLat = 0;
signed long pathLng = 0;

signed long prevLatAve[5] = {0};
signed long prevLongAve[5] = {0};

int pathCount = 0;

int maxNextData;
int maxErrorData;

bool pathDone;
bool pathError;

bool readFirstPathData;

int IDRecordActualData = 0;
bool updateRecordActualData = false;

int getSteeringPID(float _data, float _setpoint, uint8_t _kp, uint8_t _kd);
void recordActualData(byte _pathId);
void setSteering(int);
void setDrive(int);
int getMinDeg(float, float);

String reverseStr(String _str){
  String _reverseTemp = "";
  for(int i = _str.length() - 1; i >= 0; i--){
    _reverseTemp += String(_str[i]);
  }
  return _reverseTemp;
}

String longToFloatStr(long _a){
  String _returnStr;
  String _tmpStr;
  _tmpStr = String(_a);  
  _tmpStr = reverseStr(_tmpStr);
  _tmpStr.remove(0,7);
  _tmpStr = reverseStr(_tmpStr);
  _returnStr = _tmpStr + ".";

  _tmpStr = String(_a);  
  _tmpStr = reverseStr(_tmpStr);
  _tmpStr.remove(7);
  _tmpStr = reverseStr(_tmpStr);
  _returnStr += _tmpStr;

  return _returnStr;
}

float readBattery()
{
    pinMode(PA1, INPUT);

    const int R1 = 10000;
    const int R2 = 1810;

    float _mvVoltage = 0;

    for (int i = 0; i < sizeof(_voltage) / sizeof(_voltage[1]) - 1; i++)
        _voltage[i] = _voltage[i + 1];
    _voltage[sizeof(_voltage) / sizeof(_voltage[1]) - 1] = (analogRead(PA1) * 3.3 / 1024) * (R1 + R2) / R2;
    for (int i = 0; i < sizeof(_voltage) / sizeof(_voltage[1]); i++)
        _mvVoltage += _voltage[i];
    _mvVoltage /= sizeof(_voltage) / sizeof(_voltage[1]);

    return _mvVoltage;
}

void updateNextLatLong(signed long _Nlat, signed long _Nlong)
{
    // Serial.println(String(_Nlat) + "," + String(_Nlong));
    if(abs(_Nlong) > 1e-3) updateDisplay(56, 24, longToFloatStr(_Nlong));
    else updateDisplay(56, 24, "--         ");
    if(abs(_Nlat) > 1e-3)updateDisplay(56, 32, longToFloatStr(_Nlat));
    else updateDisplay(56, 32, "--        ");
}

float calculateEarthDistance(signed long _lat1, signed long _lng1, signed long _lat2, signed long _lng2){
    const int _R = 6371e3; //m
    const float _latR1 = _lat1 * 1e-7 * PI/180;
    const float _latR2 = _lat2 * 1e-7 * PI/180;
    const float _deltaLat = (_lat2 - _lat1) * 1e-7 * PI/180;
    const float _deltaLng = (_lng2 - _lng1) * 1e-7 * PI/180;

    const float _a = sin(_deltaLat/2) * sin(_deltaLat/2) +
                cos(_latR1) * cos(_latR2) *
                sin(_deltaLng/2) * sin(_deltaLng/2);
    const float _c = 2 * asin(sqrt(_a));

    return (float)(_R * _c);
}

void resetPathCount(){
  updateRecordActualData = false;
  maxNextData = 2;
  maxErrorData = 100;
  pathCount = 0;
  nextLat = 0;
  nextLng = 0;
  pathDone = false;
  pathError = false;
  readFirstPathData = true;
  for(int i = 0; i < 5; i++){
    prevLatAve[i] = 0;
    prevLongAve[i] = 0;
  }
}

void readNextLatLngPath(){
  String _pathStr = "path" + String(getPathID()) + ".csv";
  String _pathLineStr = sdReadFileByLine(_pathStr, pathCount);
  String _bufftmp = _pathLineStr;
  Serial.println(_pathLineStr);

  String _strTemp = _pathLineStr;
  if(!pathError){
    if(_strTemp.indexOf(',') > 0){
      _bufftmp.remove(_pathLineStr.indexOf(','));
      _bufftmp.replace(".", "");
      pathLng = _bufftmp.toInt();
      _bufftmp = _pathLineStr;
      _bufftmp.remove(0, _pathLineStr.indexOf(',') + 1);
      _bufftmp.replace(".", "");
      pathLat = _bufftmp.toInt();
      Serial.println(String(pathLng) + "," + String(pathLat));
    }
    else{
      pathDone = true;
      pathLng = 0;
      pathLat = 0;
    }
  }

  String _oledDisplayPathCount = "A" + String(getPathID());
  if(pathDone) _oledDisplayPathCount += "(done)";
  else if(pathError){
    _oledDisplayPathCount += "(err)";
  }
  else _oledDisplayPathCount += "(" + String(pathCount) + ")";

  u8g2.setDrawColor(0);
  u8g2.drawBox(120, 0, 8, 35);
  u8g2.setDrawColor(1);
  u8g2.setFontDirection(1);
  u8g2.drawStr(128, 0, _oledDisplayPathCount.c_str());
  u8g2.setFontDirection(0);

  pathCount++;
}

void calculateMovement()
{
  if(!pathDone && !pathError){
    if(readFirstPathData){
      readFirstPathData = false;
      readNextLatLngPath();
      nextLat = pathLat;
      nextLng = pathLng;
    }
    // recordActualData(getPathID());
    float _disHvs = calculateEarthDistance(getKalmanLatLong(1), getKalmanLatLong(0), nextLat, nextLng);
    // Serial.println(_disHvs);
    if((_disHvs < maxNextData)/* || nextLat == 0 || nextLng == 0*/){
      readNextLatLngPath();
      nextLat = pathLat;
      nextLng = pathLng;
      updateNextLatLong(nextLat, nextLng);
      // for(int i = 4; i > 0; i--){
      //   prevLatAve[i] = prevLatAve[i - 1];
      // }
      // prevLatAve[0] = nextLat;
      // prevLongAve[0] = nextLng;
    }
    else if(_disHvs > maxErrorData){
      Serial.println("Point Too Far: " + String(_disHvs) + "m");
      pathError = true;
      readNextLatLngPath();
    }
    
    if(getKalmanLatLong(0) && getKalmanLatLong(1) && pathLat && pathLat){
      float _atanHead = atan2((nextLng - getKalmanLatLong(0)), (nextLat - getKalmanLatLong(1)));
      if(abs(get485deg()) > abs(_atanHead)){
        idealHeadCal = get485deg();
        maxNextData = 8; /////////////////////////////////////////////////////////////////////////// change val here
      }
      else{
        idealHeadCal = _atanHead;
        maxNextData = 2;
      }
      // if(abs(_atanHead) > 90){
      //   pathError = true;
      // }
    }
    else idealHeadCal = 0;

    // for(int i = 0; i < 4; i++){
    //   if(prevLatAve[i + 1] > 0.01 || prevLongAve[i + 1] > 0.01){
    //     idealHeadCal += atan2((prevLatAve[i + 1] - prevLongAve[i + 1]), (prevLatAve[i] - prevLongAve[i]));
    //     idealHeadCal /= 2;
    //   }
    // }

    int finalMinRad = getMinDeg(idealHeadCal, getHeading());  
    // Serial.println(finalMinRad);
    int _drive = 1000;

    setSteering(finalMinRad);
    setDrive(_drive);

  }
  else {
    canUpdate(0x01, (1 << 9));
    canUpdate(0x02, (1 << 13));
  }
}

int getMinDeg(float _deg1, float _deg2){
  float _disLeft = _deg1 - _deg2;    
  float _disRight;
  if(_deg1 < _deg2) _disRight = _deg1 + (2*PI - _deg2);
  else _disRight = (_deg2 + (2*PI - _deg1)) * -1;

  float _minDeg;
  if(fabs(_disLeft) > fabs(_disRight)){
    _minDeg = _disRight;
  }
  else {
    _minDeg = _disLeft;
  }
  
  if (_minDeg < 0)
      _minDeg += 2 * PI;
  if (_minDeg > 2 * PI)
      _minDeg -= 2 * PI;
  _minDeg *= 180 / PI;

  if(_minDeg > 180) _minDeg -= 360;

  return (int)_minDeg;
}

void setSteering(int _deg){
  int _steeringL;
  // _steeringL = getSteeringPID(finalMinRad, 0, 50, 100);
  _steeringL = _deg;

  bool _steeringDir;
  if(_steeringL < 0) _steeringDir = 1;
  else _steeringDir = 0;

  int _canData = (1 << 9) | (_steeringDir << 8) | abs(_steeringL);
  canUpdate(0x01, _canData);
  // Serial.println("Steering: " + String(getHeadingVar - idealHeadCal) + "deg, Dir:" + String(_steeringDir));
}

void setDrive(int _val){
  bool _driveOrBrake;
  if(_val < 0) _driveOrBrake = 1;
  else _driveOrBrake = 0;

  int _canData = (1 << 13) | (_driveOrBrake << 12) | abs(_val);
  canUpdate(0x02, _canData);
}

signed long prevLatAct = 0;
signed long prevLongAct = 0;

void recordActualData(byte _pathId){
  if(!updateRecordActualData){
    updateRecordActualData = true;
    IDRecordActualData = EEPROM[5];
    EEPROM[5] = IDRecordActualData + 1;
    
    String _dirStr = "actpath" + String(_pathId);
    String _pathStr = _dirStr + "/take" + String(IDRecordActualData) + ".csv";
    
    sdMakeDir(_dirStr);
    bool _dataWriten = sdWriteFile(_pathStr, "", false);
    
    Serial.println("Record Act at : " + _pathStr + ", " + _dataWriten);
    
    prevLatAct = getKalmanLatLong(1);
    prevLongAct = getKalmanLatLong(0);
    /////////////////////////////////////////////////////////////////////////////made change here today
    String _sdCardGnssDumpData = longToFloatStr(getKalmanLatLong(0)) + "," 
                                + longToFloatStr(getKalmanLatLong(1)) + "\n"
                                ;
                                
    sdWriteFile(_pathStr, _sdCardGnssDumpData, true);
  }

  if(calculateEarthDistance(prevLatAct, prevLongAct, getKalmanLatLong(1), getKalmanLatLong(0)) > 1.5){
    
    String _sdCardGnssDumpData = longToFloatStr(getKalmanLatLong(0)) + "," 
                                + longToFloatStr(getKalmanLatLong(1)) + "\n"
                                ;
    // Serial.print(_sdCardGnssDumpData);
        
    String _dirStr = "actpath" + String(_pathId);
    String _pathStr = _dirStr + "/take" + String(IDRecordActualData) + ".csv";
    sdWriteFile(_pathStr, _sdCardGnssDumpData, true);
  }
}

unsigned long previousTime = 0;
float lastError = 0.000;

int getSteeringPID(float _setpoint, float _data, uint8_t _kp, uint8_t _kd){
  unsigned long currentTime = 0;
	float elapsedTime = 0.000;
	float error = 0.000;
	float output = 0.000;
	float setpoint = _setpoint;
	float cumError = 0.000;
	float rateError = 0.000;

	currentTime = HAL_GetTick();
	elapsedTime = currentTime - previousTime;

	error = setpoint - _data;
	if(!elapsedTime) elapsedTime = 10e-10;

	rateError = (error - lastError) / elapsedTime;

	output = _kp * error + _kd * rateError;

	lastError = error;
	previousTime = currentTime;

	return constrain(output, -150, 150);
}

float getIdealHead(){
  return idealHeadCal;
}

signed long getNextLatLng(byte _a){
  if (_a)
      return nextLat;
  else
      return nextLng;
}