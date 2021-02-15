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
EthernetServer server(80);

// CREATE AREST INSTANCE
aREST rest = aREST();

// INSTANTIATE PCA DEVICES
PCA9685 warm(B000000);
PCA9685 cool(B000001);

// CREATE JSON STATE DOCS
StaticJsonDocument <1024> warmLaneState;
String warmLightState = warmLaneState.as<String>();

StaticJsonDocument <1024> coolLaneState;
String coolLightState = coolLaneState.as<String>();

StaticJsonDocument <32> network;
String IP = network.as<String>();

StaticJsonDocument <32> sysStatus;
String systemStatus = sysStatus.as<String>();

// SETUP ENCODERS AND BUTTONS
enum PinAssignments {
encoderPinA_W = 19,
encoderPinB_W = 18,
encoderPinA_C = 2,
encoderPinB_C = 3,
systemOffLED = 5,
systemOffBTN = 8,
systemOnLED = 11,
systemOnBTN = 13
};
volatile unsigned int encoderPos_W = 0;
unsigned int lastReportedPos_W = 0;
static boolean rotating_W = false;
boolean A_set_W = false;
boolean B_set_W = false;
volatile unsigned int encoderPos_C = 0;
unsigned int lastReportedPos_C = 0;
static boolean rotating_C = false;
boolean A_set_C = false;
boolean B_set_C = false;

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

  pinMode(encoderPinA_W, INPUT);
  pinMode(encoderPinB_W, INPUT);
  digitalWrite(encoderPinA_W, HIGH);
  digitalWrite(encoderPinB_W, HIGH);
  pinMode(encoderPinA_C, INPUT);
  pinMode(encoderPinB_C, INPUT);
  digitalWrite(encoderPinA_C, HIGH);
  digitalWrite(encoderPinB_C, HIGH);

  pinMode(systemOffBTN, INPUT);
  pinMode(systemOnBTN, INPUT);
  pinMode(systemOffLED, OUTPUT);
  pinMode(systemOnLED, OUTPUT);
  digitalWrite(systemOffBTN, HIGH);
  digitalWrite(systemOnBTN, LOW);
  analogWrite(systemOffLED, 0);
  analogWrite(systemOnLED, 255);

  // encoder pin on interrupt 5 (pin 19)
  attachInterrupt(5, doEncoderA_W, CHANGE);
  // encoder pin on interrupt 4 (pin 18)
  attachInterrupt(4, doEncoderB_W, CHANGE);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderA_C, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(0, doEncoderB_C, CHANGE);

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
  rest.variable("warmBank", &warmLightState);
  rest.variable("coolBank", &coolLightState);
  rest.variable("ip", &IP);
  rest.variable("status", &systemStatus);

//  // Give name and ID to device (ID should be 6 characters long)
  rest.set_id("000001");
  rest.set_name("studio-light-controller");

  Ethernet.setHostname("studioLightControl");
  
  if (Ethernet.begin(mac) == 0) {
    if(Ethernet.link() == 0) {
      network["ip"] = "NO LINK";
      IP = network["ip"].as<String>();
      serializeJson(network, Serial);
    } else {
      network["ip"] = "NO DHCP";
      IP = network["ip"].as<String>();
      serializeJson(network, Serial);
    };
  } else {
    // success, display your local IP address:
    network["ip"] = DisplayAddress(Ethernet.localIP());
    IP = network["ip"].as<String>();
    serializeJson(network, Serial);
  };

  for(int i=0; i<14; i++) {
    warmLaneState[i] = 0;
    warm.setChannelPWM(i, 255);
  };

  for(int i=0; i<14; i++) {
    coolLaneState[i] = 0;
    cool.setChannelPWM(i, 255);
  };

  warmLightState = warmLaneState.as<String>();
  coolLightState = coolLaneState.as<String>();

  // Start watchdog
  wdt_enable(WDTO_4S);

  sysStatus["status"] = "lightsOff";
  serializeJson(sysStatus, Serial);
  systemStatus = sysStatus["status"].as<String>();
}

void loop() {
  handleEncoders();
  rest.handle(Serial);
  wdt_reset();
  warm.getChannelPWM(0);
  cool.getChannelPWM(0);
  checkNetwork();
  readBtns();
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

  warmLightState = warmLaneState.as<String>();
  coolLightState = coolLaneState.as<String>();

  sysStatus["status"] = "lightsOff";
  systemStatus = sysStatus["status"].as<String>();

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

  sysStatus["status"] = "lightsOn";
  systemStatus = sysStatus["status"].as<String>();

  warmLightState = warmLaneState.as<String>();
  coolLightState = coolLaneState.as<String>();

  return 1;
}

int bankSet(String command) {
  int offCheck = 0;

  String ch = getValue(command,':',0);
  String val = getValue(command,':',1);
  int valInt = val.toInt();

  if(ch == "w") {
    if(valInt > 0) {
      digitalWrite(MW, HIGH);
    } else {
      digitalWrite(MW, LOW);
    }
    for(int i=0; i<14; i++) {
      int coolCheck = coolLaneState[i];
      offCheck = offCheck + valInt + coolCheck;
      int y = invert(valInt);
      warm.setChannelPWM(i, y << 4);
      warmLaneState[i] = valInt;
    }
    warmLightState = warmLaneState.as<String>();
  } else if(ch == "c") {
    if(valInt > 0) {
      digitalWrite(MC, HIGH);
    } else {
      digitalWrite(MC, LOW);
    }
    for(int j=0; j<14; j++) {
      int warmCheck = warmLaneState[j];
      offCheck = offCheck + valInt + warmCheck;
      int x = invert(valInt);
      cool.setChannelPWM(j, x << 4);
      coolLaneState[j] = valInt;
    }
    coolLightState = coolLaneState.as<String>();
  } else {
    return 0;
  }

  if(offCheck > 0) {
    sysStatus["status"] = "lightsOn";
  } else {
    sysStatus["status"] = "lightsOff";
  }
  
  systemStatus = sysStatus["status"].as<String>();

  return 1;
}

int warmSet(String command) {
  int offCheck = 0;

  for(int i=0; i<14; i++) {
    String val = getValue(command,':',i);
    int valInt = val.toInt();
    int coolCheck = coolLaneState[i];
    offCheck = offCheck + valInt + coolCheck;
    int y = invert(valInt);
    warm.setChannelPWM(i, y << 4);
    warmLaneState[i] = valInt;
  }

  if(offCheck > 0) {
    digitalWrite(MW, HIGH);
    sysStatus["status"] = "lightsOn";
  } else {
    digitalWrite(MW, LOW);
    sysStatus["status"] = "lightsOff";
  }
  
  systemStatus = sysStatus["status"].as<String>();
  warmLightState = warmLaneState.as<String>();

  return 1;
}

int coolSet(String command) {
  int offCheck = 0;

  for(int i=0; i<14; i++) {
    String val = getValue(command,':',i);
    int valInt = val.toInt();
    int warmCheck = warmLaneState[i];
    offCheck = offCheck + valInt + warmCheck;
    int y = invert(valInt);
    cool.setChannelPWM(i, y << 4);
    coolLaneState[i] = valInt;
  }

  if(offCheck > 0) {
    digitalWrite(MC, HIGH);
    sysStatus["status"] = "lightsOn";
  } else {
    digitalWrite(MC, LOW);
    sysStatus["status"] = "lightsOff";
  }

  systemStatus = sysStatus["status"].as<String>();
  coolLightState = coolLaneState.as<String>();

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
    warmLightState = warmLaneState.as<String>();
  } else if(color == "c") {
    if(valInt > 0) {
      digitalWrite(MC, HIGH);
    }
    int y = invert(valInt);
    cool.setChannelPWM(channel, y << 4);
    coolLaneState[channel] = valInt;
    coolLightState = coolLaneState.as<String>();
  }

  if(valInt > 0) {
    sysStatus["status"] = "lightsOn";
  } else {
    digitalWrite(MW, LOW);
    digitalWrite(MC, LOW);
    sysStatus["status"] = "lightsOff";
  }

  systemStatus = sysStatus["status"].as<String>();

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

void doEncoderA_W() {
  if ( rotating_W ) delay (3);
  if ( digitalRead(encoderPinA_W) != A_set_W ) {
    A_set_W = !A_set_W;
    if ( A_set_W && !B_set_W )
      encoderPos_W += 1;
    rotating_W = false;
  }
}

void doEncoderB_W() {
  if ( rotating_W ) delay (3);
  if ( digitalRead(encoderPinB_W) != B_set_W ) {
    B_set_W = !B_set_W;
    if ( B_set_W && !A_set_W )
      encoderPos_W -= 1;
    rotating_W = false;
  }
}

void doEncoderA_C() {
  if ( rotating_C ) delay (3);
  if ( digitalRead(encoderPinA_C) != A_set_C ) {
    A_set_C = !A_set_C;
    if ( A_set_C && !B_set_C )
      encoderPos_C += 1;
    rotating_C = false;
  }
}

void doEncoderB_C() {
  if ( rotating_C ) delay (3);
  if ( digitalRead(encoderPinB_C) != B_set_C ) {
    B_set_C = !B_set_C;
    if ( B_set_C && !A_set_C )
      encoderPos_C -= 1;
    rotating_C = false;
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
      } else {
        digitalWrite(MW, LOW);
      }
      warmLightState = warmLaneState.as<String>();
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
      } else {
        digitalWrite(MC, LOW);
      }
      coolLightState = coolLaneState.as<String>();
    }
  
    if(valInt > 0) {
      sysStatus["status"] = "lightsOn";
      systemStatus = sysStatus["status"].as<String>();
    };
  
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
        int coolCheck = coolLaneState[i];
        offCheck = offCheck + laneInt + coolCheck;
        if(checkInt > 0 && laneInt > 0) {
          digitalWrite(MW, HIGH);
        }
        if(laneInt != checkInt) {
          int y = invert(laneInt);
          warm.setChannelPWM(i, y << 4);
          warmLaneState[i] = laneInt;
        }
      } else if(ch == 1) {
        int checkInt = coolLaneState[i];
        int warmCheck = warmLaneState[i];
        offCheck = offCheck + laneInt + warmCheck;
        if(checkInt > 0 && laneInt > 0) {
          digitalWrite(MC, HIGH);
        }
        if(laneInt != checkInt) {
          int y = invert(laneInt);
          cool.setChannelPWM(i, y << 4);
          coolLaneState[i] = laneInt;
        };
      };
    };

    if(offCheck > 0) {
      sysStatus["status"] = "lightsOn";
    } else {
      digitalWrite(MW, LOW);
      digitalWrite(MC, LOW);
      sysStatus["status"] = "lightsOff";
    }

    systemStatus = sysStatus["status"].as<String>();
    
    if(ch == 0) {
      warmLightState = warmLaneState.as<String>();
    } else if(ch == 1) {
      coolLightState = coolLaneState.as<String>();
    }
  }
}

String DisplayAddress(IPAddress address)
{
 return String(address[0]) + "." + 
        String(address[1]) + "." + 
        String(address[2]) + "." + 
        String(address[3]);
}

void handleEncoders() {
  if (lastReportedPos_W != encoderPos_W) {
    if(lastReportedPos_W < encoderPos_W && (encoderPos_W - lastReportedPos_W) < 100) {
      int value = (encoderPos_W - lastReportedPos_W);
      updatePCA(value, false, 0);
    } else if(lastReportedPos_W > encoderPos_W && (lastReportedPos_W - encoderPos_W) < 100) {
      int value = (lastReportedPos_W - encoderPos_W);
      updatePCA(value, true, 0);
    }
    lastReportedPos_W = encoderPos_W;
  }

  if (lastReportedPos_C != encoderPos_C) {
    if(lastReportedPos_C < encoderPos_C && (encoderPos_C - lastReportedPos_C) < 100) {
      int value = (encoderPos_C - lastReportedPos_C);
      updatePCA(value, false, 1);
    } else if(lastReportedPos_C > encoderPos_C && (lastReportedPos_C - encoderPos_C) < 100) {
      int value = (lastReportedPos_C - encoderPos_C);
      updatePCA(value, true, 1);
    }
    lastReportedPos_C = encoderPos_C;
  }
}

void checkNetwork() {
  if(Ethernet.link() == 0) {
    if(IP != "NO LINK" && IP != "NO DHCP") {
      network["ip"] = "NO LINK";
      IP = network["ip"].as<String>();
      serializeJson(network, Serial);
    }
  } else {
    EthernetClient client = server.available();
    rest.handle(client);
  }
  if(IP == "NO LINK") {
    if(Ethernet.link() == 1) {
      Ethernet.setHostname("studioLightControl");
      if (Ethernet.begin(mac) == 0) {
        network["ip"] = DisplayAddress(Ethernet.localIP());
        IP = network["ip"].as<String>();
        serializeJson(network, Serial);
      }
    }
  }
}

void readBtns() {
  if(!digitalRead(systemOffBTN)) {
    allOff();
  };
  if(digitalRead(systemOnBTN)) {
    allOn();
  }
  if(systemStatus == "lightsOn") {
    analogWrite(systemOffLED, 255);
    analogWrite(systemOnLED, 0);
  } else if(systemStatus == "lightsOff") {
    digitalWrite(MW, LOW);
    digitalWrite(MC, LOW);
    analogWrite(systemOffLED, 0);
    analogWrite(systemOnLED, 255);
  }
}

int invert(int value) {
  value = map(value, 0, 255, 255, 0);
  return value;
}
