#include <Arduino.h>
#include <SD.h>

File myFile;

void sdCardBegin(){
    if (!SD.begin(PB0)) {
        Serial.println("initialization failed!");
        while (1);
    }
}

String sdReadFile(String _path){
    String _sdCardData = "";
    myFile = SD.open(_path);
    if(myFile){
        while(myFile.available()){
            _sdCardData += (char)myFile.read();
        }
        myFile.close();
    }
    return _sdCardData;
}

String sdReadFileByLine(String _path, int _line){
    String _sdCardData = "";
    int _lineCount = 0;
    myFile = SD.open(_path);
    if(myFile){
        while(myFile.available()){
            char _c = (char)myFile.read();
            if(_lineCount <= _line){
                _sdCardData += _c;
                if(_c == '\n'){
                    if(_lineCount < _line) _sdCardData = "";
                    _lineCount++;
                }
            }
        }        
        myFile.close();
    }
    return _sdCardData;
}

bool sdWriteFile(String _path, String _data, bool _append){
    bool _dataWrite = false;
    if(!_append) SD.remove(_path);
    myFile = SD.open(_path, FILE_WRITE);
    if(myFile){
        _dataWrite = true;
        myFile.print(_data);
        myFile.close();
    }
    return _dataWrite;
}

bool sdMakeDir(String _path){
    return (bool)SD.mkdir(_path);
}