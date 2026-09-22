// Transparent USB <-> UART bridge for poking the lidar, with an in-band escape
// for control: FE ED C0 <cmd> <u32 arg LE>.
#include <Arduino.h>

#define TX_PIN 20  // uart1 TX; not broken out on the RP2040 Zero
#define RX_PIN 5
static SerialUART &U = Serial2;
static uint32_t baud = 115200;
static uint32_t cfg = SERIAL_8N1;

static const uint8_t ESC[3] = {0xFE, 0xED, 0xC0};
static uint8_t esc[8];
static uint8_t escN = 0;

static void uartStart() {
  U.end();
  U.setRX(RX_PIN);
  U.setTX(TX_PIN);
  U.setFIFOSize(4096);
  U.begin(baud, cfg);
}

static void reply(const char *fmt, ...) {
  char b[96];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  Serial.print("\n#BR ");
  Serial.println(b);
}

static void command(uint8_t c, uint32_t a) {
  switch (c) {
    case 'B': baud = a; uartStart(); reply("baud %lu", (unsigned long)baud); break;
    case 'F':  // frame format: 0=8N1 1=8E1 2=8O1 3=8N2
      cfg = a == 1 ? SERIAL_8E1 : a == 2 ? SERIAL_8O1 : a == 3 ? SERIAL_8N2 : SERIAL_8N1;
      uartStart(); reply("fmt %lu", (unsigned long)a); break;
    case 'K': {  // break: hold TX low a ms
      U.end();
      pinMode(TX_PIN, OUTPUT); digitalWrite(TX_PIN, LOW); delay(a);
      digitalWrite(TX_PIN, HIGH); delay(2);
      uartStart(); reply("break %lu", (unsigned long)a); break;
    }
    case 'P': {  // pin (a&0xff) level: (a>>8)&3: 0 low 1 high 2 input 3 input_pullup
      uint8_t p = a & 0xFF, m = (a >> 8) & 3;
      if (m == 0) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
      else if (m == 1) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
      else pinMode(p, m == 2 ? INPUT_PULLDOWN : INPUT_PULLUP);
      reply("pin %u mode %u read %d", p, m, digitalRead(p)); break;
    }
    case 'W': {  // pwm: pin a&0xff, duty (a>>8)&0xffff of 65535, freq fixed via 'Q'
      uint8_t p = a & 0xFF; uint32_t d = (a >> 8) & 0xFFFF;
      analogWriteRange(65535); analogWrite(p, d);
      reply("pwm pin %u duty %lu", p, (unsigned long)d); break;
    }
    case 'Q': analogWriteFreq(a); reply("pwmfreq %lu", (unsigned long)a); break;
    case 'R': {  // read pin
      reply("pin %lu = %d", (unsigned long)a, digitalRead(a)); break;
    }
    case 'M': {  // activity on pin (a>>16, default RX) for (a&0xffff) ms: edges, min pulse, high time %
      uint8_t pin = (a >> 16) ? (a >> 16) : RX_PIN; uint32_t ms = a & 0xFFFF;
      if (pin == RX_PIN || pin == TX_PIN) U.end();
      pinMode(pin, INPUT);
      uint32_t t0 = micros(), last = t0, minp = 0xFFFFFFFF, maxp = 0, edges = 0, hi = 0, lastS = t0;
      int lv = gpio_get(pin);
      while (micros() - t0 < ms * 1000) {
        int v = gpio_get(pin); uint32_t n = micros();
        if (lv) hi += n - lastS; lastS = n;
        if (v != lv) { if (edges) { minp = min(minp, n - last); maxp = max(maxp, n - last); } last = n; lv = v; edges++; }
      }
      uartStart();
      reply("pin %u edges %lu minpulse %lu us maxpulse %lu us high %lu%% level %d", pin, (unsigned long)edges,
            (unsigned long)minp, (unsigned long)maxp, (unsigned long)(hi / (ms * 10)), lv); break;
    }
    case 'L': {  // log edges on pin (a>>16) for (a&0xffff) ms: binary dump "#EL" n, then u32 us + u8 level each
      static uint32_t ts[6000]; static uint8_t lv[6000];
      uint8_t pin = a >> 16; uint32_t ms = a & 0xFFFF, n = 0;
      pinMode(pin, INPUT);
      uint32_t t0 = micros(); int l = gpio_get(pin);
      ts[n] = 0; lv[n++] = l;
      while (micros() - t0 < ms * 1000 && n < 6000) {
        int v = gpio_get(pin);
        if (v != l) { ts[n] = micros() - t0; lv[n++] = v; l = v; }
      }
      Serial.printf("\n#EL %lu\n", (unsigned long)n);
      for (uint32_t i = 0; i < n; i++) { Serial.write((uint8_t *)&ts[i], 4); Serial.write(lv[i]); }
      break;
    }
    case '?': reply("bridge baud %lu", (unsigned long)baud); break;
    default: reply("unknown %c", c);
  }
}

void setup() {
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  Serial.begin(115200);
  uartStart();
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (escN < 3) {
      if (b == ESC[escN]) { esc[escN++] = b; continue; }
      if (escN) { U.write(esc, escN); escN = 0; }
      if (b == ESC[0]) { esc[escN++] = b; continue; }
      U.write(b);
    } else {
      esc[escN++] = b;
      if (escN == 8) {
        uint32_t a = esc[4] | (esc[5] << 8) | (esc[6] << 16) | ((uint32_t)esc[7] << 24);
        escN = 0;
        command(esc[3], a);
      }
    }
  }
  uint8_t buf[256];
  int n = 0;
  while (U.available() && n < (int)sizeof buf) buf[n++] = U.read();
  if (n) Serial.write(buf, n);
}
