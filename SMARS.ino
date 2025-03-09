#include <Wire.h>
#include <NewPing.h>
#include <ezButton.h>
#include <MPU6050_light.h>
#include <Adafruit_MotorShield.h>
#include <DIYables_IRcontroller.h>

// Global parameters
enum Direction { Stop,
                 Forward,
                 Backward,
                 Left,
                 Right };

enum State {
  SLEEP,
  MOVE,
  TRAP,
  AVOIDANCE_ROTATE,
  AVOIDANCE_ROLLBACK,
};

struct Color {
  uint8_t r, g, b;

  constexpr Color(uint8_t red, uint8_t green, uint8_t blue)
    : r(red), g(green), b(blue) {}

  static const Color Red;
  static const Color Green;
  static const Color Blue;
  static const Color Orange;
  static const Color Yellow;
};

// Define static members outside the struct
const Color Color::Red = Color(255, 0, 0);
const Color Color::Green = Color(0, 255, 0);
const Color Color::Blue = Color(0, 0, 255);
const Color Color::Orange = Color(255, 165, 0);
const Color Color::Yellow = Color(255, 255, 0);

// Hardcoded pins
const int PIN_BUTTON = 7;

const int PIN_SONAR_ECHO = 3;
const int PIN_SONAR_TRIGGER = 2;

const int PIN_IR_RECEIVER = 4;

const int PIN_AVOIDANCE_LEFT = 8;
const int PIN_AVOIDANCE_RIGHT = 12;

const int PIN_LED_RED = 9;
const int PIN_LED_BLUE = 10;
const int PIN_LED_GREEN = 11;

const int PID_LIGHT_SENSOR = A0;
const int PIN_LIGHT_LEFT = 5;
const int PIN_LIGHT_RIGHT = 6;

// === Sensors and others
// Accel+Gyro
MPU6050 mpu(Wire);

//ultrasonic setup: trigger, echo, distance in cm
NewPing sonar(PIN_SONAR_TRIGGER, PIN_SONAR_ECHO, 500);

// IR Receiver
DIYables_IRcontroller_21 irReceiver(PIN_IR_RECEIVER, 200);

// Create the motor shield object with the default I2C address
// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M2
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor &motorL = *AFMS.getMotor(3);
Adafruit_DCMotor &motorR = *AFMS.getMotor(4);

// Button
ezButton button(PIN_BUTTON);

// Variables
bool automatic = true;
State state = SLEEP;

int speed = 155;

struct {
  int distance;
  int illuminance = 1024;  // disabled by default to start in the dark
  float temperature;
  bool mpuEnabled = true;
  bool leftObstacle, rightObstacle;
  float angleX, angleY, angleZ;
  unsigned long lastUpdate;
} sensors, previous_sensors;

unsigned long lastPrintTime = 0;

void setup() {
  // set up Serial library at 9600 bps
  Serial.begin(9600);

  // TODO: We can use only one pin with irReceiver
  pinMode(PIN_SONAR_ECHO, INPUT);
  pinMode(PIN_SONAR_TRIGGER, OUTPUT);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  pinMode(PIN_LIGHT_LEFT, OUTPUT);
  pinMode(PIN_LIGHT_RIGHT, OUTPUT);

  pinMode(PIN_BUTTON, INPUT);

  pinMode(PIN_AVOIDANCE_LEFT, INPUT);
  pinMode(PIN_AVOIDANCE_RIGHT, INPUT);

  irReceiver.begin();

  button.setDebounceTime(100);  // 100 millis

  sensors.lastUpdate = millis();

  // Initialize MPU.
  while (!(sensors.mpuEnabled = !mpu.begin()) && sensors.lastUpdate + 2000 < millis()) {
    delay(10);
  }

  if (sensors.mpuEnabled) {
    Serial.println("MPU detected. Calibrating...");
    mpu.calcOffsets();
    Serial.println("Done");
  } else {
    Serial.println("MPU is not enabled");
  }

  if (AFMS.begin()) {
    changeSpeed(speed);

    changeColor(Color::Blue);
    changeDirection(Stop);
  } else {
    Serial.println("Motorshield is not enabled");
  }

  previous_sensors = sensors;
}

void loop() {
  // copy current state to track changes, if required
  readSensors();

  if (sensors.lastUpdate - lastPrintTime >= 500) {
    printSensors();
    lastPrintTime = sensors.lastUpdate;
  }

  processKeyPad();
  processNightLight();

  if (state != SLEEP) {
    autonomic();
  }

  delay(50);
}

void readSensors() {
  // process the button state
  button.loop();

  // Copy previous state to track changes, if required
  previous_sensors = sensors;

  int r = sonar.ping_cm();

  sensors.distance = r == 0 ? 999 : r;

  sensors.leftObstacle = digitalRead(PIN_AVOIDANCE_LEFT);
  sensors.rightObstacle = digitalRead(PIN_AVOIDANCE_RIGHT);

  if (sensors.mpuEnabled) {
    mpu.update();

    sensors.angleX = mpu.getAngleX();
    sensors.angleY = mpu.getAngleY();
    sensors.angleZ = mpu.getAngleZ();
  }

  //   sensors_event_t a, g, temp;
  //   mpu.getEvent(&a, &g, &temp);

  //   sensors.gyro = a.gyro;
  //   sensors.acceleration = a.acceleration;
  //   sensors.temperature = temp.temperature;
  // }

  sensors.illuminance = analogRead(PID_LIGHT_SENSOR);

  sensors.lastUpdate = millis();
}

void printSensors() {
  Serial.print("Dist: ");
  Serial.print(sensors.distance);
  Serial.print(" Temp: ");
  Serial.print(sensors.angleX);
  Serial.print(" Light: ");
  Serial.print(sensors.illuminance);
  Serial.print(" Btn: ");
  Serial.print(button.getState());
  Serial.print(" L-Obs: ");
  Serial.print(sensors.leftObstacle);
  Serial.print(" R-Obs: ");
  Serial.print(sensors.rightObstacle);
  Serial.print(" Gyro: ");
  Serial.print(sensors.angleX);
  Serial.print(", ");
  Serial.print(sensors.angleY);
  Serial.print(", ");
  Serial.println(sensors.angleZ);
}

void processKeyPad() {
  Key21 key = irReceiver.getKey();
  if (key == Key21::KEY_PLAY_PAUSE || button.isReleased()) {
    changeDirection(Stop);


    if (changeState(SLEEP)) {
      changeColor(Color::Blue);
    } else {
      stateMove();
    }
  } else if (key == Key21::KEY_VOL_PLUS) {
    changeSpeed(min(speed + 10, 255));
  } else if (key == Key21::KEY_VOL_MINUS) {
    changeSpeed(max(speed - 10, 75));
  } else if (key == Key21::KEY_EQ) {
    changeSpeed(155);
  }
}

void processNightLight() {
  if (sensors.illuminance < 300) {
    if (previous_sensors.illuminance >= 300) {
      changeNightLight(HIGH);
    }
  } else if (sensors.illuminance >= 400) {
    if (previous_sensors.illuminance < 400) {
      changeNightLight(LOW);
    }
  }
}

void autonomic() {
  // Base case: interrupt currect state if we have enough space
  if (sensors.distance > 40) {
    stateMove();
  } else if (state == MOVE) {
    stateMove();
  } else if (state == AVOIDANCE_ROLLBACK) {
    stateAvoidanceRollback();
  } else if (state == AVOIDANCE_ROTATE) {
    stateAvoidanceRotate();
  } else if (state == TRAP) {
    stateTrap();
  }
}

void stateMove() {
  if (sensors.distance < 10) {
    stateAvoidance();
  } else if (changeState(MOVE)) {
    changeColor(Color::Green);
    changeDirection(Forward);
  }
}

void stateAvoidance() {
  stateAvoidanceRollback();
}

void stateAvoidanceRollback() {
  if (!backIsFree()) {
    stateTrap();
  } else if (sensors.distance >= 15) {
    stateAvoidanceRotate();
  } else if (changeState(AVOIDANCE_ROLLBACK)) {
    changeColor(Color::Yellow);
    changeDirection(Backward);
  }
}

void stateAvoidanceRotate() {
  if (changeState(AVOIDANCE_ROTATE)) {
    changeColor(Color::Orange);
    Direction rotate = random(0, 2) ? Left : Right;
    changeDirection(rotate);
  }
}

void stateTrap() {
  if (backIsFree()) {
    stateAvoidance();
  } else if (changeState(TRAP)) {
    changeColor(Color::Red);
    changeDirection(Stop);
  }
}

bool backIsFree() {
  return sensors.leftObstacle && sensors.rightObstacle;
}

bool changeState(State newState) {
  if (newState != state) {
    state = newState;
    return true;
  }
  return false;
}

void changeSpeed(int newSpeed) {
  speed = newSpeed;
  motorL.setSpeed(speed);
  motorR.setSpeed(speed);
}

void changeNightLight(int mode) {
  digitalWrite(PIN_LIGHT_LEFT, mode);
  digitalWrite(PIN_LIGHT_RIGHT, mode);
}

void changeDirection(Direction direction) {
  motorL.run((direction == Left) ? BACKWARD : (direction == Right) ? FORWARD : RELEASE);
  motorR.run((direction == Left) ? FORWARD : (direction == Right) ? BACKWARD : RELEASE);
}

void changeColor(Color color) {
  analogWrite(PIN_LED_RED, color.r);
  analogWrite(PIN_LED_BLUE, color.b);
  analogWrite(PIN_LED_GREEN, color.g);
}