#include <Wire.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <LiquidCrystal_I2C.h>
#pragma GCC diagnostic pop

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

const static int CurrentSensorPin = A0;

const static int ENPin = 4;
const static int HS1Pin = 11;
const static int LS1Pin = 12;
const static int HS2Pin = 3;
const static int LS2Pin = 2;

const static int ExtendDir = 0;
const static int RetractDir = 1;

const static int ExtendPin = 38;
const static int TestPin = 40;
const static int StopPin = 42;

/* This is the maximum zero level to detect when we have stopped.
 */
const static int MaxCurrentError = 5;

/* When using the simple move commands we want to make sure the actuator isn't loaded
 * significantly. The rated no load current is 2.6A. We use 4A.
 */
const static int MaxNoLoadCurrent = 65;

/* Current at max load is 13A (213 steps). */
const static int MaxCurrent = 213;

/* Amps per step in ADC reading. 80mV/A in MLX91210 CAS-101 */
const static float AmpsPerStep = 0.061f;

/* Returns current in steps, positive or negative depending on direction.
 */
int read_current() {
  return analogRead(CurrentSensorPin) - 512;
}

int current_to_kg(float current) {
  return (int) (35.09f * current - 22.456f); // from calibration
}

int current_to_newton(float current) {
  return (int) ((35.09f * current - 22.456f) * 9.81f); // from calibration
}

bool read_extend_button() {
  return !digitalRead(ExtendPin);
}

bool read_test_button() {
  return !digitalRead(TestPin);
}

bool read_stop_button() {
  return !digitalRead(StopPin);
}

void turn_off_all_gates() {
  digitalWrite(HS1Pin, LOW);
  digitalWrite(HS2Pin, LOW);
  digitalWrite(LS1Pin, LOW);
  digitalWrite(LS2Pin, LOW);
  delay(10);
}

// dir = 0 is HS1, LS2.
// dir = 1 is HS2, LS1.
int dir = 0;
int ls_pin = LS2Pin;
void set_dir(int new_dir) {
  turn_off_all_gates();

  if (new_dir == 0) {
    dir = 0;
    ls_pin = LS2Pin;
    digitalWrite(HS1Pin, HIGH);
  } else {
    dir = 1;
    ls_pin = LS1Pin;
    digitalWrite(HS2Pin, HIGH);
  }
}

void drive_motor(bool val) {
  digitalWrite(ls_pin, val ? HIGH : LOW);
}

char lcd_buffer[4][20];
byte lcd_x = 0;
byte lcd_y = 0;

void init_lcd_buffer() {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 20; ++j) {
      lcd_buffer[i][j] = ' ';
    }
  }
}

void lcd_print(const char *s) {
  lcd.print(s);
  size_t len = strlen(s);
  for (size_t i = 0; i < len; ++i) {
    lcd_buffer[lcd_y][lcd_x + i] = s[i];
  }
  lcd_x += len;
}

void lcd_print(char c) {
  lcd_buffer[lcd_y][lcd_x] = c;
  lcd.write(c);
  ++lcd_x;
}

void lcd_newline() {
  // If we are on last line, we have to scroll everything.
  if (lcd_y == 3) {
    lcd.clear();

    // Copy every line up.
    for (int i = 0; i < 3; ++i) {
      lcd.setCursor(0, i);
      for (int c = 0; c < 20; ++c) {
        lcd.write(lcd_buffer[i+1][c]);
        lcd_buffer[i][c] = lcd_buffer[i+1][c];
      }
    }

    // Clear the last line.
    for (int i = 0; i < 20; ++i) {
      lcd_buffer[3][i] = ' ';
    }

    lcd.setCursor(0, 3);
    lcd_y = 3;
    lcd_x = 0;
  } else {
    ++lcd_y;
    lcd_x = 0;
    lcd.setCursor(lcd_x, lcd_y);
  }
}

void lcd_return_to_start_of_line() {
  lcd_x = 0;
  lcd.setCursor(lcd_x, lcd_y);
}

void move_unloaded(int dir) {
  int stop_count = 0;
  int overload_count = 0;
  float filtered_current = 0.0f;
  
  set_dir(dir);
  drive_motor(true);
  while (true) {
    for (int i = 0; i < 1000; ++i) {
      // Total = 50us
      drive_motor(true);
      delayMicroseconds(25);
      drive_motor(false);
      delayMicroseconds(25);
    }
    int current = read_current();
    int abs_current = abs(current); /* abs is implemented as a macro on arduino... silly! */
    filtered_current = filtered_current * 0.8f + float(abs_current) * 0.2f;
    Serial.println(filtered_current * AmpsPerStep);
    
    if (filtered_current < MaxCurrentError) {
      //++stop_count;
      if (stop_count >= 5) {
        break; /* we have reached the end (current fell to 0) */
      }
    } else {
      stop_count = 0;
    }

    if (filtered_current > MaxNoLoadCurrent) {
      //++overload_count; /* we never clear this */
      if (overload_count > 3) {
        Serial.println("Overloaded! Stopping. extend/retract should only be used unloaded.");
        break;
      }
    }
  }
  drive_motor(false);
}

int drive(int current_setting, int dir, unsigned long end_time_ms) {
  unsigned int averaging_window[64] = { 0 };
  byte next_index = 0;
  unsigned int sum = 0;

  current_setting = 20;

  if (dir == ExtendDir) {
    current_setting *= -1;
  }
  
  while (millis() < end_time_ms) {
    int current = read_current();
    if ((current > current_setting && dir == ExtendDir) || (current < current_setting && dir == RetractDir)) {
      drive_motor(true);
    } else {
      drive_motor(false);
    }

    unsigned int abs_current = abs(current);

    sum -= averaging_window[next_index];
    sum += abs_current;
    averaging_window[next_index] = abs_current;
    next_index = (next_index + 1) % 64;
  }
  drive_motor(false);

  return sum / 64;
}

void calibrate(int dir, int current_setting) {
  set_dir(dir);
  int final_current = drive(current_setting, dir, millis() + 15000);
  Serial.print(final_current * AmpsPerStep * 1000);
  Serial.println(" mA");
}

// Fast int to str that only works from 0 to 999. s must be big enough to hold 4 digits.
void lcd_print_int(int x) {
  if (x < 0) {
    lcd_print('0');
  } else if (x < 10) {
    lcd_print(x + '0');
  } else if (x < 100) {
    lcd_print(x / 10 + '0');
    lcd_print(x % 10 + '0');
  } else if (x < 1000) {
    lcd_print(x / 100 + '0');
    lcd_print((x % 100) / 10 + '0');
    lcd_print((x % 10) + '0');
  } else {
    lcd_print("ERR");
    lcd_print_int(x - 1000);
  }
}

bool test(int dir) {  
  set_dir(dir);
  // unsigned long start_time = millis();
  int current_setting = 40;
  int end_count = 0;
  int max_current = 0;

  int current_settings[256];
  int avg_currents[256];
  unsigned int iteration = 0;
  bool stopped = false;

  lcd_newline();

  while (iteration < 256 && !(stopped = read_stop_button())) {
    int avg_current = drive(current_setting, dir, millis() + 200);

    if (avg_current > max_current) {
      max_current = avg_current;
    }

// Too slow and will interrupt motor driving.
#if 0
    char buf[64];
    sprintf(buf, "%d %d", current_setting, avg_current);
    Serial.println(buf);

    //lcd_return_to_start_of_line();
    //lcd_print_int(static_cast<int>(current_to_kg(avg_current * AmpsPerStep)));
#endif   

    current_settings[iteration] = current_setting;
    avg_currents[iteration] = avg_current;
    ++iteration;

    if (avg_current < MaxCurrentError) {
      ++end_count;

      if (end_count > 3) {
        // we have reached end stop
        Serial.println("End stop reached");
        break;
      }
    } else {
      end_count = 0;
    }
    
    if (current_setting > avg_current) {
      // wait for current to catch up
    } else {
      current_setting += 1; // 0.05A per 200 millisecond

      if (current_setting > MaxCurrent) {
        Serial.println("Max current reached");
        lcd_print("Max current reached.");
        break;
      }
    }
  }

  drive_motor(false);

  char buf[64];
  for (unsigned int i = 0; i < iteration; ++i) {
    sprintf(buf, "Setting: %d mA\tActual: %d mA %c (%d kg)", 
            (int) (current_settings[i] * AmpsPerStep * 1000), 
            (int) (avg_currents[i] * AmpsPerStep * 1000),
            avg_currents[i] == max_current ? '<' : ' ',
            current_to_kg(avg_currents[i] * AmpsPerStep));
    //sprintf(buf, "%d,", StepToKg[avg_currents[i]]);
    Serial.println(buf);
  }

  sprintf(buf, "%d", (int) current_to_kg(max_current * AmpsPerStep));

  lcd_print(buf);
  lcd_print(" kg. ");

  sprintf(buf, "Max: %d mA (%d kg)", (int) (max_current * AmpsPerStep * 1000), (int) current_to_kg(max_current * AmpsPerStep));
  Serial.println(buf);
  return stopped;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  lcd.begin(20,4);
  init_lcd_buffer();
  lcd_print("Ready");

  pinMode(CurrentSensorPin, INPUT_PULLUP);

  digitalWrite(ENPin, LOW);
  pinMode(ENPin, OUTPUT);

  pinMode(HS1Pin, OUTPUT);
  pinMode(LS1Pin, OUTPUT);
  pinMode(HS2Pin, OUTPUT);
  pinMode(LS2Pin, OUTPUT);

  // Make sure we don't get shoot-through when we enable the gate driver!
  turn_off_all_gates();

  digitalWrite(ENPin, HIGH);  

  pinMode(ExtendPin, INPUT_PULLUP);
  pinMode(TestPin, INPUT_PULLUP);
  pinMode(StopPin, INPUT_PULLUP);

  // set prescaler to 16 (77kHz sampling rate, 13 microseconds)
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);

  Serial.println("Reset");
}

void loop() {  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  bool extend_button = read_extend_button();
  bool test_button = read_test_button();
  
  if (cmd.length() != 0 || extend_button || test_button) {
    Serial.println("===============================================================");
    Serial.print(">> ");
    Serial.println(cmd);
    if (cmd.compareTo("help") == 0) {
      Serial.println("help:         print this help text");
      Serial.println("read_current: get the current reading in amps");
      Serial.println("extend:       extend the actuator to limit under no/minimal load");
      Serial.println("retract:      retract the actuator to limit under no/minimal load");
    } else if (cmd.compareTo("read_current") == 0) {
      Serial.println(read_current() * AmpsPerStep);
    } else if (cmd.compareTo("extend") == 0 || extend_button) {
      Serial.println("Extending...");
      lcd_newline();
      lcd_print("Extending... ");
      move_unloaded(ExtendDir);
      lcd_print("Done.");
    } else if (cmd.compareTo("retract") == 0) {
      Serial.println("Retracting...");
      move_unloaded(RetractDir);
    } else if (cmd.compareTo("test") == 0 || test_button) {
      Serial.println("Testing retract...");
      lcd_newline();
      lcd_print("Testing... ");
      bool stopped = test(RetractDir);
      if (!stopped) {
        lcd_print("Done.");
      } else {
        lcd_print("Stopped.");
      }
    } else if (cmd.substring(0, 9).compareTo("calibrate") == 0) {
      int current_setting = cmd.substring(10).toInt();
      if (current_setting <= 0 || current_setting > MaxCurrent) {
        Serial.println("Usage: calibrate [setting (<= MaxCurrent)]");
      }
      Serial.print("Calibrating at ");
      Serial.println(current_setting);
      move_unloaded(RetractDir);
      calibrate(RetractDir, current_setting);
    }
    Serial.println("Done");
  }
}

