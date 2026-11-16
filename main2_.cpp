// LEPSZA WERSJA CYKLU CZYSZCZENIA 

#include <Arduino.h>
#include <ESP32Servo.h>

// =====================================
// PINY
// =====================================
constexpr int PIN_BUTTON = 0; // BOOT

constexpr int PIN_HORIZONTAL_SERVO = 21;
constexpr int PIN_VERTICAL_SERVO = 19;
constexpr int PIN_BRUSH_POSITION = 18;
constexpr int PIN_BRUSH_ROTATION = 22;

// =====================================
// PARAMETRY CZYSZCZENIA
// =====================================

// Ruch pionowy kielicha
constexpr int VERTICAL_HOME = 0;
constexpr int VERTICAL_TARGET = 180;
constexpr unsigned long VERTICAL_MOVE_MS = 2000;

// Wysuwanie szczotki
constexpr int BRUSH_HOME = 0;
constexpr int BRUSH_TARGET = 90;
constexpr unsigned long BRUSH_MOVE_MS = 1000;
constexpr int BRUSH_SPEED_US = 1700;
constexpr int SERVO_STOP_US = 1500;

constexpr int HORIZONTAL_FORWARD_US = 1800;
constexpr int HORIZONTAL_BACKWARD_US = 1200;
constexpr int HORIZONTAL_STOP_US = 1500;
// Czas wspolnego obracania

constexpr unsigned long CLEANING_TIME_MS = 24000;
constexpr int HORIZONTAL_REPEATS = 2;

// Czas jednego ruchu
const unsigned long STEP_TIME_MS = CLEANING_TIME_MS / (2 * HORIZONTAL_REPEATS);


// Dodatkowy margines czasowy
constexpr unsigned long CLEANING_MARGIN_MS = 2000;

// Przycisk
constexpr unsigned long DEBOUNCE_MS = 50;


// =====================================
// OBIEKTY SERW
// =====================================
Servo horizontalServo;
Servo verticalServo;
Servo brushPositionServo;
Servo brushRotationServo;

// =====================================
// ZMIENNE
// =====================================
bool testRunning = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// =====================================
// ZATRZYMANIE NAPEDOW OBROTOWYCH
// =====================================
void stopRotation() {

  horizontalServo.writeMicroseconds(
      HORIZONTAL_STOP_US
  );

  brushRotationServo.writeMicroseconds(
      SERVO_STOP_US
  );
}

// =====================================
// FUNKCJA CZYSZCZENIA
// =====================================
void cleanRainGauge(bool signal) {

  if (!signal) {
    return;
  }

  Serial.println("START CZYSZCZENIA");

  // 1. Obrot pionowy kielicha
  Serial.println("Obrot pionowy");

  verticalServo.write(VERTICAL_TARGET);
  delay(VERTICAL_MOVE_MS);

  // 2. Wysuniecie szczotki
  Serial.println("Wysuwanie szczotki");

  brushPositionServo.write(BRUSH_TARGET);
  delay(BRUSH_MOVE_MS);

  // 3. Jednoczesne uruchomienie
  Serial.println("Start obrotow");

  brushRotationServo.writeMicroseconds(
    BRUSH_SPEED_US
  );

 for (int i = 0; i < HORIZONTAL_REPEATS; i++) {

  // Obrot w pierwsza strone
  horizontalServo.writeMicroseconds(HORIZONTAL_FORWARD_US);
  delay(STEP_TIME_MS);

  // Obrot w przeciwna strone
  horizontalServo.writeMicroseconds(HORIZONTAL_BACKWARD_US);
  delay(STEP_TIME_MS);
  }
  // Oba napedy pracuja jednoczesnie
  //delay(CLEANING_TIME_MS + CLEANING_MARGIN_MS);

  // 4. Zatrzymanie obu napedow
  Serial.println("Stop obrotow");

  horizontalServo.writeMicroseconds(
    SERVO_STOP_US
  );

  brushRotationServo.writeMicroseconds(
    SERVO_STOP_US
  );
  delay(CLEANING_MARGIN_MS);
  // 5. Schowanie szczotki
  Serial.println("Chowanie szczotki");

  brushPositionServo.write(BRUSH_HOME);
  delay(BRUSH_MOVE_MS);

  // 6. Powrot pionowy kielicha
  Serial.println("Powrot pionowy");

  verticalServo.write(VERTICAL_HOME);
  delay(VERTICAL_MOVE_MS);

  stopRotation(); 

  Serial.println("KONIEC CZYSZCZENIA");

  testRunning = false;
}

// =====================================
// SETUP
// =====================================
void setup() {

  Serial.begin(115200);

  // Przycisk BOOT
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  horizontalServo.setPeriodHertz(50);
  verticalServo.setPeriodHertz(50);
  brushPositionServo.setPeriodHertz(50);
  brushRotationServo.setPeriodHertz(50);

  horizontalServo.attach(
    PIN_HORIZONTAL_SERVO, 500, 2400
  );

  verticalServo.attach(
    PIN_VERTICAL_SERVO, 500, 2400
  );

  brushPositionServo.attach(
    PIN_BRUSH_POSITION, 500, 2400
  );

  brushRotationServo.attach(
    PIN_BRUSH_ROTATION, 500, 2400
  );

  // Stan poczatkowy
  stopRotation();

  verticalServo.write(VERTICAL_HOME);
  brushPositionServo.write(BRUSH_HOME);

  lastButtonState = digitalRead(PIN_BUTTON);

  Serial.println("SYSTEM GOTOWY");
  Serial.println("BOOT - uruchom czyszczenie");
  Serial.println("EN - reset ESP32");
}

// =====================================
// LOOP
// =====================================
void loop() {

  bool buttonState = digitalRead(PIN_BUTTON);

  // Wykrycie nacisniecia BOOT
  if (lastButtonState == HIGH &&
      buttonState == LOW &&
      millis() - lastDebounceTime > DEBOUNCE_MS) {

    lastDebounceTime = millis();

    cleanRainGauge(true);
  }

  lastButtonState = buttonState;
}