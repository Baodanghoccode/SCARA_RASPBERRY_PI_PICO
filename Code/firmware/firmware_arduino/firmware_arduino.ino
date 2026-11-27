// Firmware Arduino Uno + CNC Shield V3 (Khe X: STEP = 2, DIR = 5, EN = 8)
// Chế độ vi bước 1/16 (Cắm đủ 3 jumper MS1, MS2, MS3 dưới khe X)
// Timer1 phần cứng: Nhấc bút 3000 xung / 3.0s (1000 us/xung) | Hạ bút 3060 xung / 3.0s (980 us/xung)

const int STEP_PIN = 2;
const int DIR_PIN  = 5;
const int EN_PIN   = 8;

const unsigned long Z_MOVE_DURATION_MS = 1400UL; // Thời gian cố định 3.000 giây cho mỗi lần nhấc/hạ

volatile long cur_pos = 0;
volatile long target_pos = 0;
volatile int  jog_dir = 0;
volatile int  last_dir = 0;
volatile bool is_moving = false;
volatile bool move_done_flag = false;
volatile bool reset_zero_on_done = false; // Khóa lại cur_pos = 0 sau khi hạ đủ 3060 xung

volatile unsigned long current_interval_us = 1000;
volatile unsigned long target_interval_us = 1000;
const unsigned long START_INTERVAL_US = 2000; // Khởi động mềm 500 Hz cho JOG vi bước 1/16

unsigned long last_jog_ms = 0;
unsigned long last_report_ms = 0;

// =====================================================================
// CẤU HÌNH NGẮT TIMER1 PHẦN CỨNG (CTC MODE, PRESCALER = 8 -> 0.5 us/tick)
// =====================================================================
void startTimer1(unsigned long us) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  if (us < 100UL) us = 100UL;       // Tối đa 10,000 Hz cho vi bước 1/16
  if (us > 32000UL) us = 32000UL;
  OCR1A = (uint16_t)(us * 2UL - 1UL);
  TCCR1B |= (1 << WGM12);  // Chế độ CTC
  TCCR1B |= (1 << CS11);   // Prescaler = 8 (16MHz / 8 = 2MHz -> 0.5us/tick)
  TIMSK1 |= (1 << OCIE1A); // Bật ngắt OCR1A
  interrupts();
}

void stopTimer1() {
  noInterrupts();
  TIMSK1 &= ~(1 << OCIE1A);
  TCCR1B = 0;
  interrupts();
}

void doStepISR(int dir) {
  if (dir != last_dir) {
    digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
    delayMicroseconds(10);
    last_dir = dir;
  }
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
  cur_pos += dir;
}

// =====================================================================
// TRÌNH PHỤC VỤ NGẮT TIMER1
// =====================================================================
ISR(TIMER1_COMPA_vect) {
  // 1. Chế độ JOG bằng tay
  if (jog_dir != 0) {
    doStepISR(jog_dir);
    if (current_interval_us > target_interval_us) {
      unsigned long drop = max(10UL, current_interval_us / 40);
      current_interval_us = (current_interval_us - drop > target_interval_us)
                            ? (current_interval_us - drop)
                            : target_interval_us;
      OCR1A = (uint16_t)(current_interval_us * 2UL - 1UL);
    }
    return;
  }

  // 2. Chế độ MOVA (3000 xung / 3.0s khi nhấc, 3060 xung / 3.0s khi hạ)
  if (is_moving) {
    if (cur_pos < target_pos) {
      doStepISR(1);
    } else if (cur_pos > target_pos) {
      doStepISR(-1);
    }

    if (cur_pos == target_pos) {
      is_moving = false;
      TIMSK1 &= ~(1 << OCIE1A);
      TCCR1B = 0;
      if (reset_zero_on_done) {
        cur_pos = 0; // Khóa lại mốc 0 tại mặt giấy sau khi hạ đủ 3060 xung
        target_pos = 0;
        reset_zero_on_done = false;
      }
      move_done_flag = true;
    }
    return;
  }

  TIMSK1 &= ~(1 << OCIE1A);
  TCCR1B = 0;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH); // HIGH = tắt driver
}

long getSafePos() {
  noInterrupts();
  long p = cur_pos;
  interrupts();
  return p;
}

void reportPos() {
  Serial.print("#ZVAL:");
  Serial.println(getSafePos());
}

void startTimedMove(long new_target, bool zero_after) {
  long current = getSafePos();
  long delta = abs(new_target - current);

  stopTimer1();
  noInterrupts();
  jog_dir = 0;
  target_pos = new_target;
  reset_zero_on_done = zero_after;
  move_done_flag = false;
  interrupts();

  if (delta == 0) {
    if (zero_after) {
      noInterrupts();
      cur_pos = 0;
      target_pos = 0;
      reset_zero_on_done = false;
      interrupts();
    }
    is_moving = false;
    reportPos();
    Serial.println("#EVT:Z_DONE");
  } else {
    // 3000 xung -> 3,000,000 / 3000 = 1000 us/xung (1000 Hz trong đúng 3.0s)
    // 3060 xung -> 3,000,000 / 3060 = 980 us/xung  (1020 Hz trong đúng 3.0s)
    unsigned long interval_us = (Z_MOVE_DURATION_MS * 1000UL) / (unsigned long)delta;
    if (interval_us < 100UL) interval_us = 100UL;
    if (interval_us > 32000UL) interval_us = 32000UL;

    noInterrupts();
    current_interval_us = interval_us;
    target_interval_us = interval_us;
    is_moving = true;
    interrupts();

    reportPos();
    startTimer1(interval_us);
  }
}

void parseCmd(String cmd) {
  cmd.trim();
  int idx = cmd.indexOf('#');
  if (idx < 0) return;
  cmd = cmd.substring(idx + 1);

  if (cmd == "PING") {
    Serial.println("#OK:Z_PING");
    reportPos();
  } 
  else if (cmd == "STAT") {
    reportPos();
  }
  else if (cmd.startsWith("EN:")) {
    int en = cmd.substring(3).toInt();
    if (en == 0) {
      stopTimer1();
      is_moving = false;
      jog_dir = 0;
    }
    digitalWrite(EN_PIN, en == 1 ? LOW : HIGH);
    Serial.println("#OK:Z_EN");
    reportPos();
  } 
  else if (cmd == "ZERO") {
    stopTimer1();
    noInterrupts();
    cur_pos = 0;
    target_pos = 0;
    is_moving = false;
    jog_dir = 0;
    reset_zero_on_done = false;
    move_done_flag = false;
    interrupts();
    Serial.println("#OK:Z_ZERO");
    reportPos();
  } 
  else if (cmd.startsWith("MOVA:")) {
    // Cú pháp: #MOVA:<target>,<speed>[,<zero_after>]
    int c1 = cmd.indexOf(',');
    int c2 = (c1 > 0) ? cmd.indexOf(',', c1 + 1) : -1;
    long new_target = (c1 > 0) ? cmd.substring(5, c1).toInt() : cmd.substring(5).toInt();
    bool zero_after = (c2 > 0) ? (cmd.substring(c2 + 1).toInt() == 1) : false;
    startTimedMove(new_target, zero_after);
  } 
  else if (cmd.startsWith("JOG:")) {
    int comma = cmd.indexOf(',');
    int new_dir = (comma > 0) ? cmd.substring(4, comma).toInt() : cmd.substring(4).toInt();
    long sps = (comma > 0) ? max(100L, cmd.substring(comma + 1).toInt()) : 1000L;
    unsigned long t_us = 1000000UL / (unsigned long)sps;
    last_jog_ms = millis();

    if (new_dir == 0) {
      stopTimer1();
      noInterrupts();
      jog_dir = 0;
      is_moving = false;
      interrupts();
      reportPos();
    } else {
      unsigned long start_us = max(START_INTERVAL_US, t_us);
      stopTimer1();
      noInterrupts();
      is_moving = false;
      reset_zero_on_done = false;
      target_interval_us = t_us;
      if (jog_dir != new_dir) {
        current_interval_us = start_us;
      }
      jog_dir = new_dir;
      interrupts();
      startTimer1(current_interval_us);
    }
  }
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseCmd(line);
  }

  unsigned long now_ms = millis();

  if (jog_dir != 0 && (now_ms - last_jog_ms > 15000UL)) {
    stopTimer1();
    noInterrupts();
    jog_dir = 0;
    interrupts();
    reportPos();
  }

  if ((jog_dir != 0 || is_moving) && (now_ms - last_report_ms >= 60UL)) {
    last_report_ms = now_ms;
    reportPos();
  }

  if (move_done_flag) {
    noInterrupts();
    move_done_flag = false;
    interrupts();
    reportPos();
    Serial.println("#EVT:Z_DONE");
  }
}