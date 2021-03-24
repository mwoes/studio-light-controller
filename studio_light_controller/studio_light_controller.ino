#include <ArduinoJson.h>
#include <Ethernet3.h>
#include <Wire.h>
#include <PCA9685.h>
#include <aREST.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#define SS 10    //W5500 CS
#define RST 7    //W5500 RST
#define CS 4     //SD CS pin

#define MW 34 //Master Warm OE
#define MC 35 //Master Cool OE

// SETUP NETWORK / SERVER
byte mac[] = {0xD2, 0x8A, 0xA7, 0x3C, 0x0C, 0x67};
byte ip[] = {10, 0, 0, 174};
EthernetServer server(80);

// CREATE AREST INSTANCE
aREST rest = aREST();

// INSTANTIATE PCA DEVICES
PCA9685 warm(B000000);
PCA9685 cool(B000001);

// LIGHT STATE GLOBALS
unsigned int temp = 0;
int warmT = 0;
int prevW = 0;
int coolT = 0;
int prevC = 0;

// CREATE JSON STATE DOCS
StaticJsonDocument <512> warmLaneState;
StaticJsonDocument <512> coolLaneState;
StaticJsonDocument <1024> lightState;
String tmpW = lightState["warm"].as<String>();
String tmpC = lightState["cool"].as<String>();
String allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

// REPORT INTERVAL SETUP
unsigned long previousMillis = 0;
const long reportInterval = 500;

// SETUP ENCODERS AND BUTTONS
enum PinAssignments {
encoderPinA_T = 19,
encoderPinB_T = 18,
encoderPinA_B = 3,
encoderPinB_B = 2,
systemOffLED = 5,
systemOffBTN = 8,
systemOnLED = 11,
systemOnBTN = 13
};
volatile unsigned int encoderPos_T = 0;
unsigned int lastReportedPos_T = 0;
static boolean rotating_T = false;
boolean A_set_T = false;
boolean B_set_T = false;
volatile unsigned int encoderPos_B = 0;
unsigned int lastReportedPos_B = 0;
static boolean rotating_B = false;
boolean A_set_B = false;
boolean B_set_B = false;

void setup() {
  pinMode(SS, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(CS, OUTPUT);
  digitalWrite(SS, LOW);
  digitalWrite(CS, HIGH);
  digitalWrite(RST,HIGH);

  pinMode(MW, OUTPUT);
  pinMode(MC, OUTPUT);
  digitalWrite(MW, LOW);
  digitalWrite(MC, LOW);

  pinMode(encoderPinA_T, INPUT);
  pinMode(encoderPinB_T, INPUT);
  digitalWrite(encoderPinA_T, HIGH);
  digitalWrite(encoderPinB_T, HIGH);
  pinMode(encoderPinA_B, INPUT);
  pinMode(encoderPinB_B, INPUT);
  digitalWrite(encoderPinA_B, HIGH);
  digitalWrite(encoderPinB_B, HIGH);

  pinMode(systemOffBTN, INPUT);
  pinMode(systemOnBTN, INPUT);
  pinMode(systemOffLED, OUTPUT);
  pinMode(systemOnLED, OUTPUT);
  digitalWrite(systemOffBTN, HIGH);
  digitalWrite(systemOnBTN, LOW);
  analogWrite(systemOffLED, 0);
  analogWrite(systemOnLED, 255);

  // encoder pin on interrupt 5 (pin 19)
  attachInterrupt(5, doEncoderA_T, CHANGE);
  // encoder pin on interrupt 4 (pin 18)
  attachInterrupt(4, doEncoderB_T, CHANGE);
  // encoder pin on interrupt 0 (pin 3)
  attachInterrupt(0, doEncoderA_B, CHANGE);
  // encoder pin on interrupt 1 (pin 2)
  attachInterrupt(1, doEncoderB_B, CHANGE);

  Serial.begin(115200);
  Wire.begin();

  warm.resetDevices();
  warm.init(
    PCA9685_OutputDriverMode_TotemPole,
    PCA9685_OutputEnabledMode_Normal,
    PCA9685_OutputDisabledMode_Low,
    PCA9685_ChannelUpdateMode_AfterAck,
    PCA9685_PhaseBalancer_None
  );
  warm.setPWMFrequency(100);
  cool.init(
    PCA9685_OutputDriverMode_TotemPole,
    PCA9685_OutputEnabledMode_Normal,
    PCA9685_OutputDisabledMode_Low,
    PCA9685_ChannelUpdateMode_AfterAck,
    PCA9685_PhaseBalancer_None
  );
  cool.setPWMFrequency(100);
  
  rest.function("warmSet", warmSet);
  rest.function("coolSet", coolSet);
  rest.function("allOff", allOff);
  rest.function("allOn", allOn);
  rest.function("ch", channel);
  rest.function("bankSet", bankSet);
  rest.function("scene", sceneSet);
  rest.variable("lights", &allLightState);

//  // Give name and ID to device (ID should be 6 characters long)
  rest.set_id("000001");
  rest.set_name("studio-light-controller");

  Ethernet.setHostname("studioLightControl");
  
  Ethernet.begin(mac, ip);

  for(int i=0; i<14; i++) {
    warmLaneState[i] = 0;
    warm.setChannelPWM(i, 255);
  };

  for(int i=0; i<14; i++) {
    coolLaneState[i] = 0;
    cool.setChannelPWM(i, 255);
  };

  lightState["warm"] = warmLaneState;
  lightState["cool"] = coolLaneState;
  lightState["temp"] = temp;

  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  // Start watchdog
  wdt_enable(WDTO_4S);

}

void loop() {
  handleEncoders();
  rest.handle(Serial);
  wdt_reset();
  warm.getChannelPWM(0);
  cool.getChannelPWM(0);
  readBtns();
  checkStatus();
  emitStatus();
}

int allOff() {
  for(int j=255; j>=0; j--) {
    for(int i=0; i<14; i++) {
      int checkInt = warmLaneState[i];
      if(checkInt >= j) {
        int y = invert(j);
        warm.setChannelPWM(i, y << 4);
        warmLaneState[i] = j;
      };
    };
    for(int i=0; i<14; i++) {
      int checkInt = coolLaneState[i];
      if(checkInt >= j) {
        int y = invert(j);
        cool.setChannelPWM(i, y << 4);
        coolLaneState[i] = j;
      };
    };
  };

  digitalWrite(MW, LOW);
  digitalWrite(MC, LOW);

  lightState["warm"] = warmLaneState;
  lightState["cool"] = coolLaneState;

  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  return 1;
}

int allOn() {
  digitalWrite(MW, HIGH);
  digitalWrite(MC, HIGH);
  for(int j=0; j<190; j++) {
    for(int i=0; i<14; i++) {
      int checkInt = warmLaneState[i];
      if(checkInt <= j) {
        int y = invert(j);
        warm.setChannelPWM(i, y << 4);
        warmLaneState[i] = j;
      };
    };
    for(int i=0; i<14; i++) {
      int checkInt = coolLaneState[i];
      if(checkInt <= j) {
        int y = invert(j);
        cool.setChannelPWM(i, y << 4);
        coolLaneState[i] = j;
      };
    };
  };

  lightState["warm"] = warmLaneState;
  lightState["cool"] = coolLaneState;

  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  return 1;
}

int bankSet(String command) {
  String ch = getValue(command,':',0);
  String val = getValue(command,':',1);
  int valInt = val.toInt();

  if(ch == "w") {
    if(valInt > 0) {
      digitalWrite(MW, HIGH);
    }
    for(int i=0; i<14; i++) {
      int y = invert(valInt);
      warm.setChannelPWM(i, y << 4);
      warmLaneState[i] = valInt;
    }
    lightState["warm"] = warmLaneState;
  } else if(ch == "c") {
    if(valInt > 0) {
      digitalWrite(MC, HIGH);
    }
    for(int j=0; j<14; j++) {
      int x = invert(valInt);
      cool.setChannelPWM(j, x << 4);
      coolLaneState[j] = valInt;
    }
    lightState["cool"] = coolLaneState;
  } else {
    return 0;
  }
  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";
  return 1;
}

int warmSet(String command) {
  for(int i=0; i<14; i++) {
    String val = getValue(command,':',i);
    int valInt = val.toInt();
    if(valInt > 0) {
      digitalWrite(MW, HIGH);
    }
    int y = invert(valInt);
    warm.setChannelPWM(i, y << 4);
    warmLaneState[i] = valInt;
  }
  
  lightState["warm"] = warmLaneState;
  tmpW = lightState["warm"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  return 1;
}

int coolSet(String command) {
  for(int i=0; i<14; i++) {
    String val = getValue(command,':',i);
    int valInt = val.toInt();
    if(valInt > 0) {
      digitalWrite(MC, HIGH);
    }
    int y = invert(valInt);
    cool.setChannelPWM(i, y << 4);
    coolLaneState[i] = valInt;
  }

  lightState["cool"] = coolLaneState;
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  return 1;
}

int channel(String command) {
  // OPTIONS ARE w OR c for color
  String color = getValue(command,':',0);
  String chan = getValue(command,':',1);
  String value = getValue(command,':',2);
  int valInt = value.toInt();
  int channel = chan.toInt();

  if(color == "w") {
    if(valInt > 0) {
      digitalWrite(MW, HIGH);
    }
    int y = invert(valInt);
    warm.setChannelPWM(channel, y << 4);
    warmLaneState[channel] = valInt;
    lightState["warm"] = warmLaneState;
  } else if(color == "c") {
    if(valInt > 0) {
      digitalWrite(MC, HIGH);
    }
    int y = invert(valInt);
    cool.setChannelPWM(channel, y << 4);
    coolLaneState[channel] = valInt;
    lightState["cool"] = coolLaneState;
  }
  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";
  return 1;
}

// Parse incoming string
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void doEncoderA_T() {
  if ( rotating_T ) delay (3);
  if ( digitalRead(encoderPinA_T) != A_set_T ) {
    A_set_T = !A_set_T;
    if ( A_set_T && !B_set_T )
      encoderPos_T += 1;
    rotating_T = false;
  }
}

void doEncoderB_T() {
  if ( rotating_T ) delay (3);
  if ( digitalRead(encoderPinB_T) != B_set_T ) {
    B_set_T = !B_set_T;
    if ( B_set_T && !A_set_T )
      encoderPos_T -= 1;
    rotating_T = false;
  }
}

void doEncoderA_B() {
  if ( rotating_B ) delay (3);
  if ( digitalRead(encoderPinA_B) != A_set_B ) {
    A_set_B = !A_set_B;
    if ( A_set_B && !B_set_B )
      encoderPos_B += 1;
    rotating_B = false;
  }
}

void doEncoderB_B() {
  if ( rotating_B ) delay (3);
  if ( digitalRead(encoderPinB_B) != B_set_B ) {
    B_set_B = !B_set_B;
    if ( B_set_B && !A_set_B )
      encoderPos_B -= 1;
    rotating_B = false;
  }
}

int sceneSet(String command) {
  // OPTIONS ARE w for write OR s for set
  String action = getValue(command,':',0);
  String scene = getValue(command,':',1);
  
  int sNum = scene.toInt();
  int addrW = sNum * 14;
  int addrC = (sNum * 14) + 512;

  int valInt = 0;

  if(sNum <= 10 && (action == "w" || action == "s")) {
    if(action == "w") {
      for(int i=0; i<14; i++) {
        addrW = addrW + i;
        EEPROM.update(addrW, warmLaneState[i]);
      };
      for(int j=0; j<14; j++) {
        addrC = addrC + j;
        EEPROM.update(addrC, coolLaneState[j]);
      }
    } else if(action == "s") {
      int test = 0;
      for(int i=0; i<14; i++) {
        addrW = addrW + i;
        int val = EEPROM.read(addrW);
        test += val;
        int y = invert(val);
        warm.setChannelPWM(i, y << 4);
        warmLaneState[i] = val;
        valInt += val;
      };
      if(test > 0) {
        digitalWrite(MW, HIGH);
      }
      lightState["warm"] = warmLaneState;
      test = 0;
      for(int j=0; j<14; j++) {
        addrC = addrC + j;
        int val = EEPROM.read(addrC);
        test += val;
        int y = invert(val);
        cool.setChannelPWM(j, y << 4);
        coolLaneState[j] = val;
        valInt += val;
      };
      if(test > 0) {
        digitalWrite(MC, HIGH);
      }
      lightState["cool"] = coolLaneState;
    }
    tmpW = lightState["warm"].as<String>();
    tmpC = lightState["cool"].as<String>();
    allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";
    return 1;
  } else {
    return 0;
  }
}

// ch = cool or warm -- 0=warm 1=cool
void updatePCA(uint8_t value, bool adjust, int ch) {
  int offCheck = 0;
  if(ch == 0 || ch == 1) {
    int laneInt = 0;
    value = value * 10;
    if(adjust) { 
      offCheck = offCheck + value;
    };
    for(int i=0; i<14; i++) {
      if(ch == 0) {
        laneInt = warmLaneState[i];
      } else if(ch == 1) {
        laneInt = coolLaneState[i];
      }
      if(adjust) {
        if(laneInt < 255) {
          laneInt = laneInt + value;
          if(laneInt > 255) {
            laneInt = 255;
          };
        };
      } else if(!adjust) {
        if(laneInt > 0 && laneInt != 0) {
          laneInt = laneInt - value;
          if(laneInt < 0 || laneInt > 255) {
            laneInt = 0;
          }
        }
      }
      if(ch == 0) {
        int checkInt = warmLaneState[i];
        if(laneInt > 0) {
          digitalWrite(MW, HIGH);
        }
        if(laneInt != checkInt) {
          int y = invert(laneInt);
          warm.setChannelPWM(i, y << 4);
          warmLaneState[i] = laneInt;
        }
      } else if(ch == 1) {
        int checkInt = coolLaneState[i];
        if(laneInt > 0) {
          digitalWrite(MC, HIGH);
        }
        if(laneInt != checkInt) {
          int y = invert(laneInt);
          cool.setChannelPWM(i, y << 4);
          coolLaneState[i] = laneInt;
        };
      };
    };
    
    if(ch == 0) {
      lightState["warm"] = warmLaneState;
      tmpW = lightState["warm"].as<String>();
    } else if(ch == 1) {
      lightState["cool"] = coolLaneState;
      tmpC = lightState["cool"].as<String>();
    }
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";
  }
}

void handleEncoders() {
  if (lastReportedPos_T != encoderPos_T) {
    if(lastReportedPos_T < encoderPos_T && (encoderPos_T - lastReportedPos_T) < 100) {
      int value = (encoderPos_T - lastReportedPos_T);
      updatePCA(value, false, 0);
      updatePCA(value, true, 1);
    } else if(lastReportedPos_T > encoderPos_T && (lastReportedPos_T - encoderPos_T) < 100) {
      int value = (lastReportedPos_T - encoderPos_T);
      updatePCA(value, true, 0);
      updatePCA(value, false, 1);
    }
    lastReportedPos_T = encoderPos_T;
  }

  if (lastReportedPos_B != encoderPos_B) {
    if(lastReportedPos_B < encoderPos_B && (encoderPos_B - lastReportedPos_B) < 100) {
      int value = (encoderPos_B - lastReportedPos_B);
      updatePCA(value, false, 0);
      updatePCA(value, false, 1);
    } else if(lastReportedPos_B > encoderPos_B && (lastReportedPos_B - encoderPos_B) < 100) {
      int value = (lastReportedPos_B - encoderPos_B);
      if(warmT > 0) {
        updatePCA(value, true, 0);
      } else if(coolT > 0) { 
        updatePCA(value, true, 1);
      } else if(coolT == 0 && warmT == 0) {
        updatePCA(value, true, 0);
        updatePCA(value, true, 1);
      };
    }
    lastReportedPos_B = encoderPos_B;
  }
}

void readBtns() {
  if(!digitalRead(systemOffBTN)) {
    allOff();
  };
  if(digitalRead(systemOnBTN)) {
    allOn();
  }
 if(warmT > 0 || coolT > 0) {
   analogWrite(systemOffLED, 255);
   analogWrite(systemOnLED, 0);
 } else if(warmT == 0 && coolT == 0) {
   analogWrite(systemOffLED, 0);
   analogWrite(systemOnLED, 255);
 }
}

int invert(int value) {
  value = map(value, 0, 255, 255, 0);
  return value;
}

void checkStatus() {
  int checkerW = 0;
  int checkerC = 0;
  float wAvg = 0;
  float cAvg = 0;
  for(int i=0; i<14; i++ ) {
    int cw = warmLaneState[i];
    checkerW += cw;
  }
  for(int i=0; i<14; i++) {
    int cc = coolLaneState[i];
    checkerC += cc;
  }
  // UPDATE GLOBAL VALUES
  warmT = checkerW;
  coolT = checkerC;

  // RUN AVERAGES AND CALCULATE TEMP
  wAvg = checkerW / 14;
  cAvg = checkerC / 14;
  if(wAvg > cAvg) {
    temp = (cAvg / wAvg) * 100;
    temp = map(temp, 0, 100, 2700, 4600);
  } else if(cAvg > wAvg) {
      temp = (wAvg / cAvg) * 100;
      temp = map(temp, 0, 100, 6500, 4600);
  } else if(cAvg == 0 && wAvg == 0) {
      temp = 0;
  } else if(cAvg == wAvg) {
      temp = 4600;
  };

  lightState["temp"] = temp;
  tmpW = lightState["warm"].as<String>();
  tmpC = lightState["cool"].as<String>();
  allLightState = "[" + tmpW + "," + tmpC + "," + temp + "]";

  if(checkerW == 0) {
    digitalWrite(MW, LOW); 
  };
  if(checkerC == 0) {
    digitalWrite(MC, LOW);
  };
}

void emitStatus() {
  bool change = false;
  if(millis() - previousMillis > reportInterval) {
    previousMillis = millis();
    if(warmT != prevW || coolT != prevC) {
      prevW = warmT;
      prevC = coolT;
      change = true;
    };
    if(change) {
      serializeJson(lightState, Serial);
    }
  };
};
