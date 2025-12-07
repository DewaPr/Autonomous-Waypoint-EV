#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define ADDRESS 0x1E
#define M1 PB12
#define M2 PB13

float x, y, z;
int16_t middleX;
int16_t middleY;
int16_t middleZ;

float declinationAngle = 0;

int mvaCpsX[8] = {0};
int mvaCpsY[8] = {0};
int mvaCpsZ[8] = {0};

float deviation = 0;

void compassBegin()
{
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x02);
    Wire.write(0x00);
    Wire.endTransmission();

    String _hardIronData = sdReadFile("calHard.txt");
    Serial.print("Hard iron data: ");
    Serial.println(_hardIronData);

    String _strTemp = "";
    _strTemp = _hardIronData;
    _strTemp.remove(_strTemp.indexOf('|'));
    middleX = _strTemp.toInt();
    _strTemp = _hardIronData;
    _strTemp.remove(0, _strTemp.indexOf('|') + 1);
    _strTemp.remove(_strTemp.indexOf('|'));
    middleY = _strTemp.toInt();
    _strTemp = _hardIronData;
    _strTemp.remove(0, _strTemp.lastIndexOf('|') + 1);
    middleZ = _strTemp.toInt();

    Serial.print("middleX: ");
    Serial.print(middleX);
    Serial.print(",  middleY: ");
    Serial.print(middleY);
    Serial.print(",  middleZ: ");
    Serial.println(middleZ);
}

void setDeclinationAngle(float _dec)
{
    declinationAngle = _dec;
}

void getRawData()
{
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(ADDRESS, 6);
    int16_t _x, _y, _z;
    while (Wire.available())
    {
        _x = Wire.read() << 8 | Wire.read();
        _y = Wire.read() << 8 | Wire.read();
        _z = Wire.read() << 8 | Wire.read();
        // if (abs(x) > 800 || abs(y) > 800 || abs(z) > 800)
        //     Wire.requestFrom(ADDRESS, 6);
    }

    for (int i = 0; i < sizeof(mvaCpsX) / sizeof(mvaCpsX[1]) - 1; i++)
        mvaCpsX[i] = mvaCpsX[i + 1];
    mvaCpsX[sizeof(mvaCpsX) / sizeof(mvaCpsX[1]) - 1] = (int)_x;
    x = 0;
    for (int i = 0; i < sizeof(mvaCpsX) / sizeof(mvaCpsX[1]); i++)
        x += mvaCpsX[i];
    x /= sizeof(mvaCpsX) / sizeof(mvaCpsX[1]);

    for (int i = 0; i < sizeof(mvaCpsY) / sizeof(mvaCpsY[1]) - 1; i++)
        mvaCpsY[i] = mvaCpsY[i + 1];
    mvaCpsY[sizeof(mvaCpsY) / sizeof(mvaCpsY[1]) - 1] = (int)_y;
    y = 0;
    for (int i = 0; i < sizeof(mvaCpsY) / sizeof(mvaCpsY[1]); i++)
        y += mvaCpsY[i];
    y /= sizeof(mvaCpsY) / sizeof(mvaCpsY[1]);

    for (int i = 0; i < sizeof(mvaCpsZ) / sizeof(mvaCpsZ[1]) - 1; i++)
        mvaCpsZ[i] = mvaCpsZ[i + 1];
    mvaCpsZ[sizeof(mvaCpsZ) / sizeof(mvaCpsZ[1]) - 1] = (int)_z;
    z = 0;
    for (int i = 0; i < sizeof(mvaCpsZ) / sizeof(mvaCpsZ[1]); i++)
        z += mvaCpsZ[i];
    z /= sizeof(mvaCpsZ) / sizeof(mvaCpsZ[1]);
}

void calibrateCompass()
{
    unsigned long calibrationTime = millis() + 30000;
    bool calibrated = false;

    int16_t devXP = 0;
    int16_t devYP = 0;
    int16_t devZP = 0;
    int16_t devXN = 0;
    int16_t devYN = 0;
    int16_t devZN = 0;

    while (!calibrated)
    {
        getRawData();
        if (devXP < x)
            devXP = x;
        if (devXN > x)
            devXN = x;
        if (devYP < y)
            devYP = y;
        if (devYN > y)
            devYN = y;
        if (devZP < z)
            devZP = z;
        if (devZN > z)
            devZN = z;
        Serial.print("  devXP: ");
        Serial.print(devXP);
        Serial.print("  devXN: ");
        Serial.print(devXN);
        Serial.print("  devYP: ");
        Serial.print(devYP);
        Serial.print("  devYN: ");
        Serial.print(devYN);
        Serial.print("  devZP: ");
        Serial.print(devZP);
        Serial.print("  devZN: ");
        Serial.println(devZN);

        if (calibrationTime < millis())
            calibrated = true;
    }

    middleX = (devXP + devXN) / 2;
    middleY = (devYP + devYN) / 2;
    middleZ = (devZP + devZN) / 2;

    String _calibrationData;
    _calibrationData = String(middleX) + "|" + String(middleY) + "|" + String(middleZ);
    Serial.println(_calibrationData);
    sdWriteFile("calHard.txt", _calibrationData, false);
    // for (int i = 0; i < 3; i++)
    // {
    //     for (int j = 0; j < 2; j++)
    //     {
    //         if (i == 0)
    //             EEPROM[j + (i * 2) + 5] = (byte)(middleX >> (8 * j));
    //         if (i == 1)
    //             EEPROM[j + (i * 2) + 5] = (byte)(middleY >> (8 * j));
    //         if (i == 2)
    //             EEPROM[j + (i * 2) + 5] = (byte)(middleZ >> (8 * j));
    //     }
    // }
    // Serial.print("  devXP: ");
    // Serial.print(devXP);
    // Serial.print("  devXN: ");
    // Serial.print(devXN);
    // Serial.print("  devZP: ");
    // Serial.print(devZP);
    // Serial.print("  devZN: ");
    // Serial.print(devZN);
    Serial.print("  middleX: ");
    Serial.print(middleX);
    Serial.print("  middleY: ");
    Serial.print(middleY);
    Serial.print("  middleZ: ");
    Serial.println(middleZ);
}

float headCps = 0;

void updateHeading()
{
    getRawData();
    x -= (float)middleX;
    y -= (float)middleY;
    z -= (float)middleZ;
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.print(z);
    // Serial.println();

    // float xtc = x * getAccData(2) * -1 + y * getAccData(0) * -1;
    // float ztc = z * getAccData(2) * -1 + y * getAccData(1) * -1;

    headCps = atan2(x, z);
    headCps += declinationAngle;

    // mvaHead[arrHeadSize - 1] = headCps;

    // for(int i = 1; i < arrHeadSize - 1; i++) mvaHead[i - 1] = mvaHead[i];

    // for(int i = 0; i < arrHeadSize - 1; i++){
    //     headCps += mvaHead[i];
    //     headCps /= 2;
    // }

    // Serial.print("  headCps: ");
    // Serial.print(headCps);
    
    headCps *= -1;
}

float getHeading(){
    return headCps;
}