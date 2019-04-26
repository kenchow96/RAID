#ifndef fileIO_h
#define fileIO_h

#include <SD.h>
#include <SPI.h>

bool writeStringToSD(String inputString, String fileName = GPSLOG_FILENAME) {
	char instr[128];
	fileName.toCharArray(instr,fileName.length() + 1);
	File dataFile = SD.open(instr, FILE_WRITE);

  	// if the file is available, write to it:
  	if (dataFile) {
    	dataFile.println(inputString);
    	dataFile.close();
    	// print to the serial port too:
    	//Serial.println(inputString);
  	}  
  	// if the file isn't open, pop up an error:
  	else {
    	//Serial.println("error opening datalog.txt");
    	return false;
  	} 

  	return true;
}

bool deleteFile(String fileName) {
	char instr[128];
	fileName.toCharArray(instr,fileName.length() + 1);
	SD.remove(instr);
	if (SD.exists(instr)) {
		return false;
	}
	return true;
}

#include <vector>
std::vector<String> fileList;

void populateFileList() {
	File root = SD.open("/");

	while(true){
		File entry =  root.openNextFile();
		if (!entry) {
	       // no more files
	       //Serial.println("**nomorefiles**");
	       break;
	    }
	    if (!entry.isDirectory()) {
	    	fileList.push_back(entry.name());
	    }
	}
	/*
	for (std::vector<String>::const_iterator i = fileList.begin(); i != fileList.end(); ++i)
    Serial.println(*i);

	Serial.println("OK");
	*/
}

#endif