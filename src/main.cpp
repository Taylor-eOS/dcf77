#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define PIN_OUT 4
#define PIN_PON 27
#define SDA_PIN 5
#define SCL_PIN 6
#define I2C_ADDR 0x3C
#define USABLE_WIDTH 72
#define USABLE_HEIGHT 40
#define BUF_SZ 2048
#define DEBOUNCE_US 20000
#define FALSE_MIN_US 60000
#define FALSE_MAX_US 140000
#define TRUE_MIN_US 160000
#define TRUE_MAX_US 260000
#define TICK59_MIN_US 1200000
#define REPORT_INTERVAL 2000

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL_PIN, SDA_PIN, U8X8_PIN_NONE);
const uint8_t offsetX = 28;
const uint8_t offsetY = 24;
const uint8_t brightness = 255;

volatile uint32_t times[BUF_SZ];
volatile uint8_t vals[BUF_SZ];
volatile int head = 0;
volatile int count = 0;
volatile uint32_t last_edge_us = 0;

int quality_score = 0;
char line1[32];
char line2[32];
char line3[32];

void IRAM_ATTR irq_handler() {
    uint32_t now = micros();
    if (last_edge_us && (now - last_edge_us) < DEBOUNCE_US) {
        return;
    }
    last_edge_us = now;
    times[head] = now;
    vals[head] = digitalRead(PIN_OUT);
    head = (head + 1) % BUF_SZ;
    if (count < BUF_SZ) {
        count++;
    }
}

void snapshot(uint32_t* arr_t, uint8_t* arr_v, int* c) {
    noInterrupts();
    *c = count;
    int h = head;
    if (*c > BUF_SZ) {
        *c = BUF_SZ;
    }
    int start = (h - *c + BUF_SZ) % BUF_SZ;
    for (int i = 0; i < *c; i++) {
        int p = (start + i) % BUF_SZ;
        arr_t[i] = times[p];
        arr_v[i] = vals[p];
    }
    interrupts();
}

int decode_bcd(const uint8_t* arr, const int* weights, int len, bool parity) {
    int s = 0;
    int pc = 0;
    for (int i = 0; i < len; i++) {
        if (arr[i]) {
            s += weights[i];
            pc++;
        }
    }
    if (parity) {
        if ((pc % 2) == 0 && arr[len]) {
            return -1;
        }
    }
    return s;
}

bool decode_frame(const uint8_t* bits, int* min_out, int* hr_out, int* day_out, int* weekday_out, int* mon_out, int* year_out) {
    if (bits[0] != 0 || bits[20] != 1) {
        return false;
    }
    int weights_min[8] = {1, 2, 4, 8, 10, 20, 40, 80};
    int mins = decode_bcd(&bits[21], weights_min, 7, true);
    if (mins < 0) return false;
    int weights_hr[7] = {1, 2, 4, 8, 10, 20, 40};
    int hrs = decode_bcd(&bits[29], weights_hr, 6, true);
    if (hrs < 0) return false;
    int day = 0;
    int weights_day[6] = {1, 2, 4, 8, 10, 20};
    for (int i = 0; i < 6; i++) {
        if (bits[36 + i]) {
            day += weights_day[i];
        }
    }
    int weekday = 0;
    int weights_weekday[3] = {1, 2, 4};
    for (int i = 0; i < 3; i++) {
        if (bits[42 + i]) {
            weekday += weights_weekday[i];
        }
    }
    int month = 0;
    int weights_month[5] = {1, 2, 4, 8, 10};
    for (int i = 0; i < 5; i++) {
        if (bits[45 + i]) {
            month += weights_month[i];
        }
    }
    int year = 0;
    int weights_year[8] = {1, 2, 4, 8, 10, 20, 40, 80};
    for (int i = 0; i < 8; i++) {
        if (bits[50 + i]) {
            year += weights_year[i];
        }
    }
    *min_out = mins;
    *hr_out = hrs;
    *day_out = day;
    *weekday_out = weekday;
    *mon_out = month;
    *year_out = year;
    return true;
}

void analyze_and_diagnose() {
    uint32_t* tlist = new uint32_t[BUF_SZ];
    uint8_t* vlist = new uint8_t[BUF_SZ];
    int n;
    snapshot(tlist, vlist, &n);
    if (n < 4) {
        quality_score = 0;
        snprintf(line1, sizeof(line1), "0");
        snprintf(line2, sizeof(line2), "NO SIG");
        snprintf(line3, sizeof(line3), "");
        delete[] tlist;
        delete[] vlist;
        return;
    }
    uint32_t* low_durs = new uint32_t[n];
    uint32_t* low_ts = new uint32_t[n];
    uint32_t* rising_times = new uint32_t[n];
    int low_count = 0;
    int rising_count = 0;
    uint32_t low_start = 0;
    bool low_active = false;
    for (int i = 0; i < n - 1; i++) {
        uint8_t a = vlist[i];
        uint8_t b = vlist[i + 1];
        uint32_t ta = tlist[i];
        uint32_t tb = tlist[i + 1];
        if (a == 1 && b == 0) {
            low_start = tb;
            low_active = true;
        } else if (a == 0 && b == 1 && low_active) {
            uint32_t d = tb - low_start;
            if (d > 0) {
                low_durs[low_count] = d;
                low_ts[low_count] = low_start;
                low_count++;
            }
            low_active = false;
            rising_times[rising_count++] = tb;
        }
    }
    if (low_count == 0) {
        quality_score = 0;
        snprintf(line1, sizeof(line1), "0");
        snprintf(line2, sizeof(line2), "NO PULSE");
        snprintf(line3, sizeof(line3), "");
        delete[] tlist;
        delete[] vlist;
        delete[] low_durs;
        delete[] low_ts;
        delete[] rising_times;
        return;
    }
    int pulse_count = low_count;
    int short_count = 0;
    int long_count = 0;
    for (int i = 0; i < pulse_count; i++) {
        if (low_durs[i] >= FALSE_MIN_US && low_durs[i] <= FALSE_MAX_US) {
            short_count++;
        } else if (low_durs[i] >= TRUE_MIN_US && low_durs[i] <= TRUE_MAX_US) {
            long_count++;
        }
    }
    int other = pulse_count - short_count - long_count;
    int sec_like = 0;
    int tick59_like = 0;
    for (int i = 0; i < rising_count - 1; i++) {
        uint32_t gap = rising_times[i + 1] - rising_times[i];
        if (gap >= 900000 && gap <= 1100000) {
            sec_like++;
        }
        if (gap >= TICK59_MIN_US) {
            tick59_like++;
        }
    }
    float shortlong_ratio = (float)(short_count + long_count) / pulse_count;
    float other_ratio = (float)other / pulse_count;
    int frame_count = 0;
    int frame_mins[10], frame_hrs[10], frame_days[10];
    for (int i = 0; i < low_count - 1 && frame_count < 10; i++) {
        uint32_t gap = low_ts[i + 1] - low_ts[i];
        if (gap >= TICK59_MIN_US) {
            int start = i + 1;
            if (start + 59 <= low_count) {
                uint8_t bits[59];
                bool valid = true;
                for (int j = 0; j < 59; j++) {
                    uint32_t d = low_durs[start + j];
                    if (d >= FALSE_MIN_US && d <= FALSE_MAX_US) {
                        bits[j] = 0;
                    } else if (d >= TRUE_MIN_US && d <= TRUE_MAX_US) {
                        bits[j] = 1;
                    } else {
                        valid = false;
                        break;
                    }
                }
                if (valid) {
                    int min_val, hr_val, day_val, weekday_val, mon_val, year_val;
                    if (decode_frame(bits, &min_val, &hr_val, &day_val, &weekday_val, &mon_val, &year_val)) {
                        frame_mins[frame_count] = min_val;
                        frame_hrs[frame_count] = hr_val;
                        frame_days[frame_count] = day_val;
                        frame_count++;
                    }
                }
            }
        }
    }
    quality_score = 0;
    if (sec_like >= 10) quality_score += 40;
    else if (sec_like >= 5) quality_score += 20;
    else if (sec_like >= 2) quality_score += 10;
    if (tick59_like >= 1) quality_score += 20;
    if (shortlong_ratio >= 0.7) quality_score += 20;
    else if (shortlong_ratio >= 0.5) quality_score += 10;
    if (other_ratio <= 0.2) quality_score += 10;
    else if (other_ratio <= 0.4) quality_score += 5;
    if (frame_count > 0) quality_score += 10;
    snprintf(line1, sizeof(line1), "%d", quality_score);
    if (frame_count > 0) {
        snprintf(line2, sizeof(line2), "%02d:%02d", frame_hrs[0], frame_mins[0]);
        snprintf(line3, sizeof(line3), "%02d/%02d", frame_days[0], 0);
    } else if (quality_score >= 50) {
        snprintf(line2, sizeof(line2), "GOOD");
        snprintf(line3, sizeof(line3), "NO FRAME");
    } else if (quality_score >= 30) {
        snprintf(line2, sizeof(line2), "WEAK");
        snprintf(line3, sizeof(line3), "%d/%d", short_count, long_count);
    } else {
        snprintf(line2, sizeof(line2), "NOISE");
        snprintf(line3, sizeof(line3), "");
    }
    delete[] tlist;
    delete[] vlist;
    delete[] low_durs;
    delete[] low_ts;
    delete[] rising_times;
}

void drawDisplay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_logisoso32_tn);
    uint8_t textWidth = u8g2.getStrWidth(line1);
    uint8_t fontHeight = u8g2.getMaxCharHeight();
    uint8_t x = offsetX + (USABLE_WIDTH - textWidth) / 2;
    uint8_t y = offsetY + fontHeight;
    u8g2.drawStr(x, y, line1);
    u8g2.setFont(u8g2_font_8x13_tf);
    u8g2.drawStr(offsetX + 2, offsetY + 11, line2);
    u8g2.drawStr(offsetX + 2, offsetY + 24, line3);
    u8g2.sendBuffer();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    u8g2.setI2CAddress(I2C_ADDR * 2);
    u8g2.begin();
    u8g2.setBusClock(400000UL);
    u8g2.setContrast(brightness);
    pinMode(PIN_PON, OUTPUT);
    digitalWrite(PIN_PON, LOW);
    pinMode(PIN_OUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_OUT), irq_handler, CHANGE);
    snprintf(line1, sizeof(line1), "0");
    snprintf(line2, sizeof(line2), "INIT");
    snprintf(line3, sizeof(line3), "");
    drawDisplay();
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    if (now - lastUpdate >= REPORT_INTERVAL) {
        lastUpdate = now;
        analyze_and_diagnose();
        drawDisplay();
    }
}
