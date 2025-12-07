#include <Arduino.h>
#include <Wire.h>

float accX;
float accY;
float accZ;

float mvaAccX[3] = {0};
float mvaAccY[3] = {0};
float mvaAccZ[3] = {0};

// float mvaSpeed[5] = {0};

String fixData = "unknown";

signed long lngL = 0;
signed long latL = 0;
signed long spdL = 0;
signed long hAcc = 0;
signed long sensorData;

signed long  kalmanFilterData[3];
signed long  Xt[3];
signed long  Xt_update[3];
signed long  Xt_prev[3];
signed long  Pt[3];
signed long  Pt_update[3];
signed long  Pt_prev[3];
float Kt[3];
float R[3];
float Q[3];

bool firstData = true;

bool wireUpdate()
{
    bool dataValid = 0;
    Wire.beginTransmission(0x48);
    Wire.write(0x08);
    Wire.endTransmission();
    Wire.requestFrom(0x48, 26);

    int count = 0;
    byte wireData[26];
    bool dataComing = false;
    while (Wire.available())
    {
        wireData[count] = (byte)Wire.read();
        // Serial.print(wireData[count], HEX);
        // Serial.print("  ");
        dataComing = true;
        count++;
    }
    // Serial.println();

    byte chkA;
    byte chkB;
    for (int i = 0; i < 24; i++)
    {
        byte _buff;
        _buff = (chkA + wireData[i]) & 0xFF;
        chkA = _buff;
        _buff = (chkB + chkA) & 0xFF;
        chkB = _buff;
    }
    // Serial.print(chkA, HEX);
    // Serial.print(" ");
    // Serial.print(chkB, HEX);
    // Serial.print(" ");
    // Serial.println();
    if (dataComing && wireData[24] == chkA && wireData[25] == chkB)
    {
        dataValid = 1;
        ///////////////////////////////////////////////gyr Raw Data
        // _gyrX = wireData[0] | wireData[1] << 8;
        // _gyrY = wireData[2] | wireData[3] << 8;
        // _gyrZ = wireData[4] | wireData[5] << 8;

        ///////////////////////////////////////////////yaw Data
        // yaw = wireData[0] | wireData[1] << 8 | wireData[2] << 16 | wireData[3] << 24;

        ///////////////////////////////////////////////acc Raw Data
        int16_t _accX, _accY, _accZ;
        _accX = wireData[0] | wireData[1] << 8;
        _accY = wireData[2] | wireData[3] << 8;
        _accZ = wireData[4] | wireData[5] << 8;

        for (int i = 0; i < sizeof(mvaAccX) / sizeof(mvaAccX[1]) - 1; i++)
            mvaAccX[i] = mvaAccX[i + 1];
        mvaAccX[sizeof(mvaAccX) / sizeof(mvaAccX[1]) - 1] = (int)_accX;
        accX = 0;
        for (int i = 0; i < sizeof(mvaAccX) / sizeof(mvaAccX[1]); i++)
            accX += mvaAccX[i] * 1e-3;
        accX /= sizeof(mvaAccX) / sizeof(mvaAccX[1]);

        for (int i = 0; i < sizeof(mvaAccY) / sizeof(mvaAccY[1]) - 1; i++)
            mvaAccY[i] = mvaAccY[i + 1];
        mvaAccY[sizeof(mvaAccY) / sizeof(mvaAccY[1]) - 1] = (int)_accY;
        accY = 0;
        for (int i = 0; i < sizeof(mvaAccY) / sizeof(mvaAccY[1]); i++)
            accY += mvaAccY[i] * 1e-3;
        accY /= sizeof(mvaAccY) / sizeof(mvaAccY[1]);

        for (int i = 0; i < sizeof(mvaAccZ) / sizeof(mvaAccZ[1]) - 1; i++)
            mvaAccZ[i] = mvaAccZ[i + 1];
        mvaAccZ[sizeof(mvaAccZ) / sizeof(mvaAccZ[1]) - 1] = (int)_accZ;
        accZ = 0;
        for (int i = 0; i < sizeof(mvaAccZ) / sizeof(mvaAccZ[1]); i++)
            accZ += mvaAccZ[i] * 1e-3;
        accZ /= sizeof(mvaAccZ) / sizeof(mvaAccZ[1]);

        ////////////////////////////////////////////////////////////gnss
        bool _gnssFix = false;
        if ((wireData[7] & 0b00000111) == 0x02)
        {
            _gnssFix = true;
            fixData = "2D fix";
        }
        else if ((wireData[7] & 0b00000111) == 0x03)
        {
            _gnssFix = true;
            fixData = "3D fix";
        }
        else
            fixData = "no fix";

        if (wireData[7] & 0b00001000)
            fixData += " DGNSS";
        else
            fixData += "      ";

        if (_gnssFix)
        {
            lngL = wireData[8] | wireData[9] << 8 | wireData[10] << 16 | wireData[11] << 24;
            latL = wireData[12] | wireData[13] << 8 | wireData[14] << 16 | wireData[15] << 24;
            spdL = wireData[16] | wireData[17] << 8 | wireData[18] << 16 | wireData[19] << 24;
            hAcc = wireData[20] | wireData[21] << 8 | wireData[22] << 16 | wireData[23] << 24;

            if(firstData){
                firstData = false;
                R[0] = 10; R[1] = 10; R[2] = 10;
                Q[0] = 5; Q[1] = 5; Q[2] = 5;

                Pt_prev[0] = latL;
                Pt_prev[1] = lngL;
                Pt_prev[2] = spdL;
            }

            for(int i = 0; i < 3; i++){
                if(i == 0) sensorData = latL;
                if(i == 1) sensorData = lngL;
                if(i == 2) sensorData = spdL;

                Xt_update[i] = Xt_prev[i];
                Pt_update[i] = Pt_prev[i] + Q[i];
                Kt[i] = Pt_update[i] / (float)(Pt_update[i] + R[i]);
                Xt[i] = Xt_update[i] + (long)(Kt[i] * (float)(sensorData - Xt_update[i]));
                Pt[i] = (float)(1 - Kt[i]) * Pt_update[i];
                
                Xt_prev[i] = Xt[i];
                Pt_prev[i] = Pt[i];
                
                kalmanFilterData[i] = Xt[i];
            }
        }
        else
        {
            for(int i = 0; i < 3; i++) kalmanFilterData[i] = 0;
            lngL = 0;
            latL = 0;
            spdL = 0;
            hAcc = 0;
        }
    }
    return dataValid;
}

String getFixData()
{
    return fixData;
}

float getSpeed(bool _a)
{
    if (_a)
        return spdL * 1e-3 * (1e-3 * 3600); // km/h
    else
        return spdL * 1e-3; // m/s
}

float getHorAccuracy(){
    return hAcc * 1e-3;
}

float getKalmanSpeed(bool _a)
{
    if (_a)
        return kalmanFilterData[2] * 1e-3 * (1e-3 * 3600); // km/h
    else
        return kalmanFilterData[2] * 1e-3; // m/s
}

signed long getNowLatLong(byte _a)
{
    if (_a)
        return latL;
    else
        return lngL;
}

signed long getKalmanLatLong(byte _a)
{
    if(fixData != "no ") return kalmanFilterData[!_a];
    else return 0;
}

void setLedBrightness(byte _brgt)
{
    if (_brgt < 10 && _brgt)
        _brgt = 10;
    Wire.beginTransmission(0x48);
    Wire.write(0x20 + ((int)_brgt + 1) * 25 / 255);
    Wire.endTransmission();
}

// float getYawData()
// {
//     return yaw * 1e-7;
// }

// float getGyrData(int _sel) {
//   if (_sel == 0) return (float)_gyrX * 1e-2;
//   else if (_sel == 1) return (float)_gyrY * 1e-2;
//   else if (_sel == 2) return (float)_gyrZ * 1e-2;
//   else return 0;
// }

float getAccData(int _sel)
{
    if (_sel == 0)
        return accX;
    else if (_sel == 1)
        return accY;
    else if (_sel == 2)
        return accZ;
    else
        return 0;
}