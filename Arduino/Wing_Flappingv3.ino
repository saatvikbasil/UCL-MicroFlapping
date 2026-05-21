// ---- Sweep config ----
const float START_FREQ    = 2.0;
const float END_FREQ      = 50.0;
const float STEP_FREQ     = 2.0;
const unsigned long STOP_MS    = 1000;   // pause between frequencies (motors off)
const unsigned long DWELL_MS   = 3000;   // run time at each frequency
const unsigned long START_DELAY_MS = 3000;  // initial wait before sweep starts

// Motor A
const int AIN1 = 2;
const int AIN2 = 4;
const int PWMA = 9;

4const int BIN1 = 8;
const int BIN2 = 7;
const int PWMB = 10;

// TB6612 standby
const int STBY = 6;

const uint8_t sineLUT[128] PROGMEM = {
      0,   6,  12,  18,  25,  31,  37,  43,
     49,  55,  62,  68,  74,  80,  86,  92,
     97, 103, 109, 115, 120, 126, 131, 136,
    142, 147, 152, 157, 162, 167, 171, 176,
    180, 185, 189, 193, 197, 200, 204, 208,
    211, 214, 217, 220, 223, 226, 228, 231,
    233, 235, 237, 238, 240, 241, 243, 244,
    245, 246, 247, 247, 248, 248, 249, 249,
    249, 249, 248, 248, 247, 247, 246, 245,
    244, 243, 241, 240, 238, 237, 235, 233,
    231, 228, 226, 223, 220, 217, 214, 211,
    208, 204, 200, 197, 193, 189, 185, 180,
    176, 171, 167, 162, 157, 152, 147, 142,
    136, 131, 126, 120, 115, 109, 103,  97,
     92,  86,  80,  74,  68,  62,  55,  49,
     43,  37,  31,  25,  18,  12,   6,   0
};

volatile uint8_t step = 0;

ISR(TIMER2_COMPA_vect) {
    step++;

    uint8_t pwm = pgm_read_byte(&sineLUT[step < 128 ? step : step - 128]);

    if (step < 128) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
    else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
    analogWrite(PWMA, pwm);

    if (step < 128) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
    else            { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
    analogWrite(PWMB, pwm);
}

void setFrequency(float freq) {
    uint8_t prescaler;
    uint8_t ocr;

    uint16_t ocr64   = (uint16_t)(16000000.0 / (128.0 * 64.0   * freq) - 1);
    uint16_t ocr1024 = (uint16_t)(16000000.0 / (128.0 * 1024.0 * freq) - 1);

    if (ocr64 <= 255) {
        prescaler = 0b100;
        ocr       = (uint8_t)ocr64;
    } else {
        prescaler = 0b111;
        ocr       = (uint8_t)ocr1024;
    }

    noInterrupts();
    TCCR2B = prescaler;
    OCR2A  = ocr;
    TCNT2  = 0;
    interrupts();
}

void motorsStop() {
    // Disable the timer interrupt so the ISR stops driving the motors
    noInterrupts();
    TIMSK2 = 0;
    interrupts();

    // Brake both motors
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

void motorsStart(float freq) {
    // Reset step so waveform begins cleanly from zero crossing
    noInterrupts();
    step = 0;
    interrupts();

    setFrequency(freq);

    // Re-enable the timer interrupt
    noInterrupts();
    TIMSK2 = (1 << OCIE2A);
    interrupts();
}

float currentFreq = START_FREQ;
bool  sweepDone   = false;

void setup() {
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    Serial.begin(9600);

    noInterrupts();
    TCCR2A = (1 << WGM21);
    TCNT2  = 0;
    interrupts();

    Serial.print("Waiting ");
    Serial.print(START_DELAY_MS / 1000);
    Serial.println(" seconds before sweep...");
    delay(START_DELAY_MS);

    Serial.println("Sweep starting.");
}

void loop() {
    if (sweepDone) return;

    // --- Run at current frequency ---
    motorsStart(currentFreq);

    Serial.print("FREQ ");
    Serial.print(currentFreq, 1);
    Serial.println(" Hz - START");

    delay(DWELL_MS);

    Serial.print("FREQ ");
    Serial.print(currentFreq, 1);
    Serial.println(" Hz - END");

    // --- Full stop ---
    motorsStop();

    currentFreq += STEP_FREQ;

    if (currentFreq > END_FREQ + 0.01) {
        sweepDone = true;
        digitalWrite(STBY, LOW);
        Serial.println("Sweep complete.");
    } else {
        // Pause with motors off before next frequency
        Serial.println("STOPPED");
        delay(STOP_MS);
    }
}