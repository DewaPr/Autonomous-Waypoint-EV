#include <Arduino.h>

HardwareSerial rs485(PA10, PA9);

byte rs485Data = 0;

bool rs485Stp = 0;
bool rs485Dir = 0;
int rs485deg = 0;

unsigned long rsDegResetTime = 0;

void rs485Check(){
    while(rs485.available()){
        rsDegResetTime = millis() + 2000;
        rs485Data = rs485.read();
        rs485Stp = (bool)(rs485Data & 0b10000000);
        rs485Dir = (bool)(rs485Data & 0b01000000);
        rs485deg = rs485Data & 0b00111111;

        if(rs485Dir) rs485deg *= -1;
        // Serial.println(String(rs485Stp) + " " + String(rs485Dir) + " " + String(rs485deg));
        // Serial.write(rs485Data);
    }
    if(rsDegResetTime < millis()){
        rs485deg = 0;
    }
}

bool get485Stp(){
    return rs485Stp;
}

int get485deg(){
    return rs485deg;
}
