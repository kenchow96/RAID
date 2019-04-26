#include <init.h>

#define IGNORE_GPS true

void setup() {

  initSerials();
  Serial.println("Initialising Sensors");
  initSensors();
  Serial.println("Initialising ISRs");
  initISRs();
  Serial.println("Initialising Control Panel");
  initControlPanel();
  Serial.println("Initialising SD");
  initSD();
  Serial.println("Initialising GPS");
  initGPS();
  Serial.println("Setup Complete");

  //lcd.setCursor(0, 0);
  //lcd.print("RAID: ");
  lcdPrintAt("RAID: ", 0, 0);
  //lcd.setCursor(6, 0);
  //lcd.print(String(gov.modeNumberToString((unsigned int)globalStates[0])));
  lcdPrintAt(String(gov.modeNumberToString((unsigned int)globalStates[0])), 6, 0);

}

double turnPercentage, moveSpeed;

int loopCount = 0;
int debugPage = 0;
int learnPage = 0;
int missionPage = 0;

bool btConnect = true;

String newGPSString = "";
String lastGPSString = "00:00:00,0,0";

File dataFile;

String timeData = "";
double targetLat = 0;
double targetLong = 0;
bool currentWaypointComplete = true;

void loop() {

  loopCount++;

  elapsedMillis loopTime;

  //Serial.print("point 1 "); Serial.println(loopTime);
  pollSensors();
  //printCurrentSensorReadings();
  teensyToWorld();

  byte buttonStatus = getButtonStatus();

  if (buttonStatus) {
    //Serial.println(buttonStatus);
    if (buttonStatus == 1) gov.setMode(globalStates[0] - 1);
    else if (buttonStatus == 2 && (globalStates[0] + 1 != ESTOP_MODE)) gov.setMode(globalStates[0] + 1);
  }


  //move(300);

  //globalStates[0] = 4;

  runLED();

  turnPercentage = 0; moveSpeed = 0;

  switch ((int)globalStates[0]) {
    case DEBUG_MODE:
      {
        //Serial.println(getGPSString());

        //TOFMUX.select(LCD_PORT);

        if (buttonStatus == 3) {
          debugPage--;
        }
        else if (buttonStatus == 4) {
          debugPage++;
        }

        debugPage = (debugPage + 10) % 10;

        String debugString = "";

        switch (debugPage) {
          case 0:
            {
              debugString += "F TOF1 R: ";
              debugString += currentSensorReadings.frontTOF1.reading;
              debugString += "     ";
              break;
            }
          case 1:
            {
              debugString += "F TOF2 R: ";
              debugString += currentSensorReadings.frontTOF2.reading;
              debugString += "     ";
              break;
            }
          case 2:
            {
              debugString += "F TOF3 R: ";
              debugString += currentSensorReadings.frontTOF3.reading;
              debugString += "     ";
              break;
            }
          case 3:
            {
              debugString += "F TOF4 R: ";
              debugString += currentSensorReadings.frontTOF4.reading;
              debugString += "     ";
              break;
            }
          case 4:
            {
              debugString += "B TOF  R: ";
              debugString += currentSensorReadings.backTOF.reading;
              debugString += "     ";
              break;
            }
          case 5:
            {
              debugString += "G HDOP: ";
              debugString += gps.hdop.hdop();
              debugString += "        ";
              break;
            }
          case 6:
            {
              debugString += "G Lat: ";
              debugString += String(gps.location.lat(), 6);
              debugString += " ";
              break;
            }
          case 7:
            {
              debugString += "G Lng: ";
              debugString += String(gps.location.lng(), 6);
              debugString += " ";
              break;
            }
          case 8:
            {
              debugString += "G Time: ";
              debugString += getGPSTime();
              debugString += " ";
              break;
            }
          case 9:
            {
              debugString += "Heading: ";
              debugString += currentSensorReadings.compass.reading;
              debugString += "     ";
              break;
            }
          default:
            {
              break;
            }
        }

        //lcd.setCursor(0, 1);
        //lcd.print(debugString);
        lcdPrintAt(debugString, 0, 1);

        //Serial.println(debugString);

        unsigned long breatheSpeed = 0.1024 * millis(); // 12 breaths per minute
        byte intensity = (breatheSpeed % 512 > 255) ? 255 - breatheSpeed % 256 : breatheSpeed % 256;
        frontLED.setIntensity(intensity); backLED.setIntensity(intensity);

        break;
      }
    case STANDBY_MODE:
      {
        //check control panel and sensor health

        unsigned int breatheSpeed = 0.1667 * millis(); // 20 breaths per minute
        byte intensity = (breatheSpeed % 512 > 255) ? 255 - breatheSpeed % 256 : breatheSpeed % 256;
        frontLED.setIntensity(intensity); backLED.setIntensity(intensity);

        break;
      }
    case BT_MODE:
      {
        if (millis() - latestBTMessage.lastAlive > 1000) {
          latestBTMessage.targetSpeed = {0, 0};
          latestBTMessage.remoteDirection = {5, 0};
          latestBTMessage.command = {3, 0};

          frontLED.setColor(RED); backLED.setColor(RED);
          frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
          //Serial.println("dead");

          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print("DISCONNECTED... ");
          lcdPrintAt("DISCONNECTED... ", 0, 1);

          btConnect = true;

          break;
        }
        else {
          if (btConnect) {
            //TOFMUX.select(LCD_PORT);
            //lcd.setCursor(0, 1);
            //lcd.print("CONNECTED       ");
            lcdPrintAt("CONNECTED       ", 0, 1);
            btConnect = false;
          }
          frontLED.setColor(BLUE); backLED.setColor(BLUE);
          frontLED.setBlinkPeriod(0); backLED.setBlinkPeriod(0);
          frontLED.setIntensity(255); backLED.setIntensity(255);
        }
        switch (latestBTMessage.remoteDirection.data) {
          case 1:
            //speed up
            latestBTMessage.targetSpeed.data += 100;
            latestBTMessage.remoteDirection.data = 0;
            turnPercentage = 0;
            break;
          case 2:
            //speed down
            latestBTMessage.targetSpeed.data -= 100;
            latestBTMessage.remoteDirection.data = 0;
            turnPercentage = 0;
            break;
          case 3:
            //left
            turnPercentage = -50;
            break;
          case 4:
            //right
            turnPercentage = 50;
            break;
          case 5:
            //stop
            latestBTMessage.targetSpeed.data = 0;
            latestBTMessage.remoteDirection.data = 0;
            turnPercentage = 0;
            break;
          default:
            break;
        }
        moveSpeed = latestBTMessage.targetSpeed.data;
        //Serial.println(moveSpeed);
        break;
      }
    //Learn Mode
    case LEARN_MODE:
      {
        if (!modeReady) {
          if (buttonStatus == 3) {
            learnPage--;
          }
          else if (buttonStatus == 4) {
            learnPage++;
          }

          byte fileCount = fileList.size();

          learnPage = (learnPage + fileCount) % fileCount;

          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print(fileList[learnPage] + "                ");
          lcdPrintAt(fileList[learnPage] + "                ", 0, 1);

          if (buttonStatus == 5) modeReady = true;

          break;
        }

        if (gps.hdop.hdop() > 5) {
          //Serial.println(gps.hdop.hdop());
          moveSpeed = 0;
          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print("No GPS Lock...  ");
          if (!IGNORE_GPS) {
            lcdPrintAt("No GPS Lock...  ", 0, 1);
            break;
          }
        }

        newGPSString = getGPSString();
        if (newGPSString.length() > 9) {
          if (newGPSString.substring(9) != lastGPSString.substring(9)) {
            writeStringToSD(newGPSString, fileList[learnPage]);
            lastGPSString = newGPSString;
          }
        }

        lcdPrintAt(String(frontCamReading.targx) + " " + String(frontCamReading.targw) + "               ", 0, 1);

        if (frontCamReading.targx + 0.5 * frontCamReading.targw >= 800) {
          moveSpeed = max(0, 600 - (int)(millis() - frontCamReading.timestamp));
          turnPercentage = -100;
        }
        else if (frontCamReading.targx - 0.5 * frontCamReading.targw <= -800) {
          moveSpeed = max(0, 600 - (int)(millis() - frontCamReading.timestamp));
          turnPercentage = 100;
        }
        else {
          moveSpeed = constrain((WALK_SPEED * 1.5) - (frontCamReading.targw * 6), 0, (WALK_SPEED * 1.5) - constrain((int)(8 * (millis() - frontCamReading.timestamp)), 0, (WALK_SPEED * 1.5)));
          turnPercentage = (moveSpeed < 0) ? frontCamReading.targx / 30.0 : - frontCamReading.targx / 30.0;
        }

        break;
      }
    //Mission Mode
    case MISSION_MODE:
      {
        if (!modeReady) {
          if (buttonStatus == 3) {
            missionPage--;
          }
          else if (buttonStatus == 4) {
            missionPage++;
          }

          byte fileCount = fileList.size();

          missionPage = (missionPage + fileCount) % fileCount;

          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print(fileList[missionPage] + "                ");
          lcdPrintAt(fileList[missionPage] + "                ", 0, 1);

          if (buttonStatus == 5) {
            char instr[128];
            fileList[missionPage].toCharArray(instr, fileList[missionPage].length() + 1);
            dataFile = SD.open(instr);
            modeReady = true;
          }

          break;
        }

        if (gps.hdop.hdop() > 5) {
          //Serial.println(gps.hdop.hdop());
          moveSpeed = 0;
          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print("No GPS Lock...  ");
          lcdPrintAt("No GPS Lock...  ", 0, 1);
          break;
        }

        if (currentWaypointComplete) {
          //Serial.println("here");
          String dataPoint = "";
          if (dataFile) {
            if (dataFile.available()) {
              while (true) {
                char c = dataFile.read();
                //Serial.println(c);
                if (c != '\n') {
                  dataPoint += c;
                }
                else {
                  break;
                }
              }
            }
            else {
              dataFile.close();
            }
          }
          else {
            //TOFMUX.select(LCD_PORT);
            //lcd.setCursor(0, 1);
            //lcd.print("FILE ERROR      ");
            lcdPrintAt("FILE ERROR      ", 0, 1);
          }

          byte state = 0;
          timeData = "";
          targetLat = 0;
          targetLong = 0;

          char dataPointStr[128];
          dataPoint.toCharArray(dataPointStr, dataPoint.length() + 1);
          char * tok = strtok(dataPointStr, ",");

          while (tok)
          {
            switch (state)
            {
              case 0: timeData = tok; state = 1; break;
              case 1: targetLat = String(tok).toFloat(); state = 2; break;
              case 2: targetLong = String(tok).toFloat(); state = 3; break;
              default: break; // Skip any additional tokens
            }
            tok = strtok(NULL, ",");
          }

          //TOFMUX.select(LCD_PORT);
          //lcd.setCursor(0, 1);
          //lcd.print(targetLat, 6);
          //lcd.setCursor(8, 1);
          //lcd.print(targetLong, 6);
          Serial.print(targetLat, 6); Serial.print(" "); Serial.println(targetLong, 6);
          Serial.println("HEREEEEE");
        }

        unsigned long mToWaypoint = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetLat, targetLong);
        double angleToWaypoint = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetLat, targetLong);

        int headingError = (int)(360 + currentSensorReadings.compass.reading - angleToWaypoint) % 360;
        int correctedHeadingError = (headingError > 180) ? 360 - headingError : -headingError;

        //TOFMUX.select(LCD_PORT);
        //lcd.setCursor(0, 1);
        //lcd.print(String(correctedHeadingError) + " " + String(mToWaypoint) + "               ");
        lcdPrintAt(String(correctedHeadingError) + " " + String(mToWaypoint) + "               ", 0, 1);

        int maxHeadingError = max(0, 12 - mToWaypoint);

        if (correctedHeadingError < -maxHeadingError || correctedHeadingError > maxHeadingError) {
          turnPercentage = (correctedHeadingError > 0) ? 100 : -100;
          moveSpeed = 1000;
        }
        else {
          turnPercentage = correctedHeadingError * 4;
          moveSpeed = 1000;//min(mToWaypoint * 500, 5000);
        }
        //Serial.print(toNextTarget.angle); Serial.print(" "); Serial.print(naza.getHeading()); Serial.print(" "); Serial.println(toNextTarget.distance);
        //Serial.println(correctedHeadingError);

        currentWaypointComplete = (mToWaypoint < max(gps.hdop.hdop() * 2, GPS_WAYPOINT_MIN_ERROR_M)) ? true : false;

        break;
      }
    //Software eStop Mode
    case ESTOP_MODE:
      {
        //TOFMUX.select(LCD_PORT);
        //lcd.setCursor(0, 1);
        //lcd.print("hold down key...");
        lcdPrintAt("hold down key...", 0, 1);

        if (!digitalRead(DOWN_BUTTON_PIN)) break;

        //elapsedMillis holdTime;
        unsigned long startHold = millis();

        while (digitalRead(DOWN_BUTTON_PIN) && millis() - startHold <= 5000);

        //if (holdTime >= 5000) {
          gov.setMode(0, true);
          //lcd.setCursor(0, 1);
          //lcd.print("                ");
          lcdPrintAt("                ", 0, 1);
        //}

        break;
      }
    default:
      {
        break;
      }
  }

  //moveSpeed=0;

  //Serial.println(loopTime);
  move2(constrain(moveSpeed, -1000, 1000), constrain(turnPercentage, -100, 100));
  //move2(0,0);

}
