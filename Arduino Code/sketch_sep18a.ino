#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ------------------- WiFi -------------------
#define WIFI_SSID "Aaquil"
#define WIFI_PASSWORD "Aafiya@07"

// ------------------- Firebase -------------------
String firebaseURL = "https://smart-library-noise-detection-default-rtdb.firebaseio.com/seats.json";

// ------------------- Ultrasonic Pins -------------------
#define TRIG_PIN1 5
#define ECHO_PIN1 18
#define TRIG_PIN2 19
#define ECHO_PIN2 21
#define TRIG_PIN3 22
#define ECHO_PIN3 23
#define TRIG_PIN4 27
#define ECHO_PIN4 14

// ------------------- Mic Pins -------------------
const int micPin1 = 32;
const int micPin2 = 33;
const int micPin3 = 34;
const int micPin4 = 35;

// ------------------- LED Pins -------------------
const int greenLED1 = 2;   const int redLED1   = 4;
const int greenLED2 = 12;  const int redLED2   = 13;
const int greenLED3 = 15;  const int redLED3   = 25;
const int greenLED4 = 26;  const int redLED4   = 16;

// ------------------- Variables -------------------
long distance1, distance2, distance3, distance4;
const int thresholdDistance = 10;   // cm cutoff for occupancy
const int sampleWindow = 50;        // ms for mic sampling
const int noiseDuration = 2000;     // ms persistent noise check
int baseline1, baseline2, baseline3, baseline4;
const int noiseThreshold = 200;     // above baseline
unsigned long noiseStart1=0, noiseStart2=0, noiseStart3=0, noiseStart4=0;
bool persistentNoise1=false, persistentNoise2=false, persistentNoise3=false, persistentNoise4=false;

// ------------------- Reservation Flags -------------------
bool reserved1 = false, reserved2 = false, reserved3 = false, reserved4 = false;
unsigned long reservedTime1 = 0, reservedTime2 = 0, reservedTime3 = 0, reservedTime4 = 0;
const unsigned long reservationDuration = 30 * 1000; // 30 seconds

// ------------------- LED Blinking -------------------
bool blinkState1 = false, blinkState2 = false, blinkState3 = false, blinkState4 = false;
unsigned long lastBlinkTime1 = 0, lastBlinkTime2 = 0, lastBlinkTime3 = 0, lastBlinkTime4 = 0;
const unsigned long blinkInterval = 500; // 500ms

void setup() {
  Serial.begin(115200);

  // Ultrasonic pins
  pinMode(TRIG_PIN1, OUTPUT); pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT); pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT); pinMode(ECHO_PIN3, INPUT);
  pinMode(TRIG_PIN4, OUTPUT); pinMode(ECHO_PIN4, INPUT);

  // LED pins
  pinMode(greenLED1, OUTPUT); pinMode(redLED1, OUTPUT);
  pinMode(greenLED2, OUTPUT); pinMode(redLED2, OUTPUT);
  pinMode(greenLED3, OUTPUT); pinMode(redLED3, OUTPUT);
  pinMode(greenLED4, OUTPUT); pinMode(redLED4, OUTPUT);

  // Calibrate baseline for each mic
  delay(2000);
  baseline1 = calibrateBaseline(micPin1);
  baseline2 = calibrateBaseline(micPin2);
  baseline3 = calibrateBaseline(micPin3);
  baseline4 = calibrateBaseline(micPin4);

  Serial.println("✅ Calibration Done");
  Serial.print("Mic Baselines -> 1: "); Serial.print(baseline1);
  Serial.print(", 2: "); Serial.print(baseline2);
  Serial.print(", 3: "); Serial.print(baseline3);
  Serial.print(", 4: "); Serial.println(baseline4);

  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.println("System Ready...");
}

void loop() {
  unsigned long currentTime = millis();

  // Fetch reservations from Firebase
  fetchReservationsFromFirebase();

  // Print reserved status for serial monitoring
  Serial.print("Firebase Reserved Status -> ");
  Serial.print("Seat1: "); Serial.print(reserved1?"Yes":"No"); Serial.print(", ");
  Serial.print("Seat2: "); Serial.print(reserved2?"Yes":"No"); Serial.print(", ");
  Serial.print("Seat3: "); Serial.print(reserved3?"Yes":"No"); Serial.print(", ");
  Serial.print("Seat4: "); Serial.println(reserved4?"Yes":"No");

  // Noise Detection
  persistentNoise1 = detectNoise(micPin1, baseline1, currentTime, noiseStart1, persistentNoise1);
  persistentNoise2 = detectNoise(micPin2, baseline2, currentTime, noiseStart2, persistentNoise2);
  persistentNoise3 = detectNoise(micPin3, baseline3, currentTime, noiseStart3, persistentNoise3);
  persistentNoise4 = detectNoise(micPin4, baseline4, currentTime, noiseStart4, persistentNoise4);

  // Ultrasonic distances
  distance1 = readDistance(TRIG_PIN1, ECHO_PIN1); delay(30);
  distance2 = readDistance(TRIG_PIN2, ECHO_PIN2); delay(30);
  distance3 = readDistance(TRIG_PIN3, ECHO_PIN3); delay(30);
  distance4 = readDistance(TRIG_PIN4, ECHO_PIN4); delay(30);

  // Process seats
  bool occupied1 = processSeat(1, distance1, persistentNoise1, greenLED1, redLED1, reserved1);
  bool occupied2 = processSeat(2, distance2, persistentNoise2, greenLED2, redLED2, reserved2);
  bool occupied3 = processSeat(3, distance3, persistentNoise3, greenLED3, redLED3, reserved3);
  bool occupied4 = processSeat(4, distance4, persistentNoise4, greenLED4, redLED4, reserved4);

  // Handle reservation timers & blinking
  handleReservationTimer(reserved1, reservedTime1, 1, currentTime, greenLED1, redLED1);
  handleReservationTimer(reserved2, reservedTime2, 2, currentTime, greenLED2, redLED2);
  handleReservationTimer(reserved3, reservedTime3, 3, currentTime, greenLED3, redLED3);
  handleReservationTimer(reserved4, reservedTime4, 4, currentTime, greenLED4, redLED4);

  // Push status to Firebase (optional)
  pushSeatStatusToFirebase(
    occupied1, persistentNoise1, reserved1,
    occupied2, persistentNoise2, reserved2,
    occupied3, persistentNoise3, reserved3,
    occupied4, persistentNoise4, reserved4
  );

  delay(200);
}

// ------------------- Functions -------------------
long readDistance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if(duration==0) return -1;
  return duration*0.034/2;
}

int readMicLevel(int micPin){
  unsigned long startMillis = millis();
  int signalMax = 0, signalMin = 4096;
  while(millis()-startMillis < sampleWindow){
    int sample = analogRead(micPin);
    if(sample>signalMax) signalMax=sample;
    if(sample<signalMin) signalMin=sample;
  }
  return signalMax - signalMin;
}

int calibrateBaseline(int micPin){
  int total=0;
  for(int i=0;i<50;i++){
    total += readMicLevel(micPin);
    delay(20);
  }
  return total/50;
}

bool detectNoise(int micPin, int baseline, unsigned long currentTime, unsigned long &noiseStart, bool &persistentFlag){
  int peakToPeak = readMicLevel(micPin);
  if(peakToPeak > baseline + noiseThreshold){
    if(noiseStart==0) noiseStart = currentTime;
    if(currentTime - noiseStart >= noiseDuration) persistentFlag = true;
  } else {
    noiseStart = 0;
    persistentFlag = false;
  }
  return persistentFlag;
}

bool processSeat(int seat, long distance, bool noisy, int greenLED, int redLED, bool reserved){
  bool occupied = (distance!=-1 && distance < thresholdDistance);
  bool recommend = (!occupied && !noisy && !reserved);

  // Reserved handled separately; noisy seats solid red
  if(reserved){
    // blinking handled elsewhere
  } else if(noisy){
    digitalWrite(redLED,HIGH);
    digitalWrite(greenLED,LOW);
  } else {
    digitalWrite(redLED,LOW);
    digitalWrite(greenLED,recommend?HIGH:LOW);
  }

  Serial.print("Seat "); Serial.print(seat);
  if(distance==-1) Serial.print(": No Reading");
  else Serial.print(": "+String(distance)+" cm");
  Serial.print(" | Occupied: "); Serial.print(occupied?"Yes":"No");
  Serial.print(" | Noise: "); Serial.print(noisy?"Persistent":"Quiet");
  Serial.print(" | Reserved: "); Serial.println(reserved?"Yes":"No");

  return occupied;
}

void handleReservationTimer(bool &reserved, unsigned long &reservedTime, int seatNumber, unsigned long currentTime, int greenLED, int redLED){
  bool *blinkState; unsigned long *lastBlinkTime;
  switch(seatNumber){
    case 1: blinkState=&blinkState1; lastBlinkTime=&lastBlinkTime1; break;
    case 2: blinkState=&blinkState2; lastBlinkTime=&lastBlinkTime2; break;
    case 3: blinkState=&blinkState3; lastBlinkTime=&lastBlinkTime3; break;
    case 4: blinkState=&blinkState4; lastBlinkTime=&lastBlinkTime4; break;
    default: return;
  }

  if(reserved){
    if(reservedTime==0) reservedTime=currentTime;
    if(currentTime - reservedTime >= reservationDuration){
      reserved=false;
      reservedTime=0;
      digitalWrite(redLED,LOW);
      digitalWrite(greenLED,LOW);
    } else {
      if(currentTime - *lastBlinkTime >= blinkInterval){
        *blinkState = !*blinkState;
        digitalWrite(redLED, *blinkState?HIGH:LOW);
        digitalWrite(greenLED,LOW);
        *lastBlinkTime=currentTime;
      }
    }
  } else {
    reservedTime=0;
    digitalWrite(redLED,LOW);
  }
}

void fetchReservationsFromFirebase(){
  if(WiFi.status()!=WL_CONNECTED) return;
  HTTPClient http;
  http.begin(firebaseURL);
  int httpResponse = http.GET();
  if(httpResponse==200){
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc,payload);
    if(!error){
      reserved1 = doc["seat1"]["reserved"] | false;
      reserved2 = doc["seat2"]["reserved"] | false;
      reserved3 = doc["seat3"]["reserved"] | false;
      reserved4 = doc["seat4"]["reserved"] | false;
    }
  }
  http.end();
}

void pushSeatStatusToFirebase(bool o1, bool n1, bool r1,
                              bool o2, bool n2, bool r2,
                              bool o3, bool n3, bool r3,
                              bool o4, bool n4, bool r4){
  if(WiFi.status()!=WL_CONNECTED) return;
  HTTPClient http;
  http.begin(firebaseURL);
  http.addHeader("Content-Type","application/json");
  String jsonData="{";
  jsonData += "\"seat1\":{\"occupied\":" + String(o1?"true":"false") + ",\"noise\":" + String(n1?"true":"false") + ",\"reserved\":" + String(r1?"true":"false") + "},";
  jsonData += "\"seat2\":{\"occupied\":" + String(o2?"true":"false") + ",\"noise\":" + String(n2?"true":"false") + ",\"reserved\":" + String(r2?"true":"false") + "},";
  jsonData += "\"seat3\":{\"occupied\":" + String(o3?"true":"false") + ",\"noise\":" + String(n3?"true":"false") + ",\"reserved\":" + String(r3?"true":"false") + "},";
  jsonData += "\"seat4\":{\"occupied\":" + String(o4?"true":"false") + ",\"noise\":" + String(n4?"true":"false") + ",\"reserved\":" + String(r4?"true":"false") + "}";
  jsonData += "}";
  http.PUT(jsonData);
  http.end();
}

