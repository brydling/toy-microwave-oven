/*
*/

#include <arduino-timer.h>
#include <Encoder.h>
#include <TM1637Display.h>

#define BLINK_MS 500ul

#define LED_BAR_STEP 2

#define TIMER_SMALLEST_STEP 2

#define BEEP_NUM 3
#define BEEP_PERIOD_MS 1000ul
#define BEEP_LENGTH_MS 160ul
#define BEEP_FREQ 2000ul

#define LIGHT_BRIGHTNESS   255 // 0 (off) - 255 (bright)
#define DISPLAY_BRIGHTNESS   7 // 0 (dim) -   7 (bright)
#define DISPLAY_BRIGHTNESS_STANDBY 2

const uint8_t RELAY_PIN = 18;
const uint8_t LIGHT_PIN = 13;
const uint8_t SPEAKER_PIN = 12;

const uint8_t LED_BAR_PINS[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

const uint8_t ENC_1_PINS[2] = { 1, 0 };
const uint8_t ENC_1_BTN_PIN = 19;

const uint8_t DOOR_SWITCH_PIN = 20;

const uint8_t DISP_CLK_PIN = 14;
const uint8_t DISP_DIO_PIN = 15;

struct timer_type {
  uint8_t min;
  uint8_t sec;
};

struct clock_type {
  uint8_t hr;
  uint8_t min;
  uint8_t sec;
};

auto timer = timer_create_default();

timer_type timerVal = { 0, 0 };
clock_type clock = { 14, 14, 5 };
timer_type display = { 0, 0 };
enum state_type { STANDBY, STOPPED, RUNNING, BEEPING };
state_type state = STANDBY;

typedef uint16_t tick_type;
tick_type ticks = 0;

Encoder enc(ENC_1_PINS[0], ENC_1_PINS[1]);

#define TICK_FREQ 20ul
#define FREQ_TO_MS(F) (1000ul/F)
#define FREQ_TO_TICKS(F) ((tick_type) F*TICK_FREQ)
#define MS_TO_TICKS(MS) ((tick_type) MS*TICK_FREQ/1000ul)

#define STANDBY_TIMEOUT MS_TO_TICKS(60000)


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);

  // initialize digital pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(ENC_1_BTN_PIN, INPUT_PULLUP);
  pinMode(DOOR_SWITCH_PIN, INPUT_PULLUP);

  led_bar_init();

  timer.every(FREQ_TO_MS(TICK_FREQ), step);
  timer.every(1000, update_clock);
}

// the loop function runs over and over again forever
void loop() {
  timer.tick();
}

bool step(void* arg) {
  static bool first = true;

  state_type prev_state = state;
  state_type next_state = state;

  ticks++;

  bool rerun = false;

  do {
    rerun = false;

    switch(state) {
      case STANDBY:
        next_state = state_standby(prev_state);
        break;
      case STOPPED:
        next_state = state_stopped(prev_state);
        /*if(first) {
          timerVal.sec = 0;
          timerVal.min = 0;
          //next_state = RUNNING;
          first = false;
        }*/
        break;

      case RUNNING:
        next_state = state_running(prev_state);
        break;

      case BEEPING:
        next_state = state_beeping(prev_state);
        break;
    }

    prev_state = state;

    if(next_state != state) {
      rerun = true;
      state = next_state;
    }

  } while(rerun);

  led_bar_update(state == RUNNING);
  display_update(state == RUNNING);

  analogWrite(LIGHT_PIN, (digitalRead(DOOR_SWITCH_PIN) || state == RUNNING) ? LIGHT_BRIGHTNESS : 0);

  return true;
}

state_type state_standby(state_type prev_state) {
  state_type next_state = state;

  if(state != prev_state) {
    timerVal.min = 0;
    timerVal.sec = 0;
  }

  if(!digitalRead(ENC_1_BTN_PIN) || abs(enc.read()) >= 4) {
    next_state = STOPPED;
  }

  display.min = clock.hr;
  display.sec = clock.min;

  return next_state;
}

state_type state_stopped(state_type prev_state) {
  static uint32_t stopped_timer = 0;

  int32_t knob = -read_encoder()*TIMER_SMALLEST_STEP;
  state_type next_state = state;

  if(state != prev_state) {
    digitalWrite(RELAY_PIN, 0);
    stopped_timer = 0;
  }

  if(!digitalRead(ENC_1_BTN_PIN) && (timerVal.min > 0 || timerVal.sec > 0)) {
    next_state = RUNNING;
  } else {
    if(knob != 0) {
      stopped_timer = 0;
      if(knob > 0) {
        if(timerVal.sec + knob > 59) {
          timerVal.min++;
          timerVal.sec = knob - (60 - timerVal.sec);
        } else {
          timerVal.sec += knob;
        }
      } else if(knob < 0) {
        if(((int8_t) timerVal.sec) + knob < 0) {
          if(timerVal.min == 0) {
            timerVal.sec = 0;
          } else {
            timerVal.min--;
            timerVal.sec = 60 - (-knob - timerVal.sec);
          }
        } else {
          timerVal.sec += knob;
        }
      }
    } else {
      stopped_timer++;

      if(stopped_timer == STANDBY_TIMEOUT) {
        next_state = STANDBY;
      }
    }
  }

  display.min = timerVal.min;
  display.sec = timerVal.sec;

  return next_state;
}

state_type state_running(state_type prev_state) {
  static tick_type next_countdown;

  state_type next_state = state;

  if(digitalRead(DOOR_SWITCH_PIN)) {
    next_state = STOPPED;
  } else {
    if(state != prev_state) {
      next_countdown = ticks + FREQ_TO_TICKS(1);
      digitalWrite(RELAY_PIN, 1);
    }

    if(ticks == next_countdown) {
      next_countdown += FREQ_TO_TICKS(1);

      if(timerVal.sec > 0) {
        timerVal.sec--;
      } else if(timerVal.min > 0) {
        timerVal.min--;
        timerVal.sec = 59;
      } else {
        // if this happens, something is wrong
      }

      if(timerVal.min == 0 && timerVal.sec == 0) {
        next_state = BEEPING;
      }
    }
  }

  display.min = timerVal.min;
  display.sec = timerVal.sec;

  return next_state;
}

state_type state_beeping(state_type prev_state) {
  static uint8_t beeps;
  static tick_type next_beep;

  state_type next_state = state;

  if(state != prev_state) {
    // start beeping
    beeps = BEEP_NUM;
    next_beep = ticks;

    // other states
    digitalWrite(RELAY_PIN, 0);
  }

  if(ticks == next_beep) {
    next_beep = ticks + MS_TO_TICKS(BEEP_PERIOD_MS);
    tone(SPEAKER_PIN, BEEP_FREQ, BEEP_LENGTH_MS);
    beeps--;
  }

  if(beeps == 0) {
    next_state = STOPPED;
  }

  return next_state;
}

void led_bar_init() {
  for(int i = 0; i < sizeof(LED_BAR_PINS); i++) {
    pinMode(LED_BAR_PINS[i], OUTPUT);
    digitalWrite(LED_BAR_PINS[i], 1);
  }
}

void led_bar_update(bool flash) {
  const uint8_t STEP = LED_BAR_STEP;

  static tick_type next_toggle;
  static bool toggle = false;
  static bool flash_last = false;
  static uint8_t num_lit_last = sizeof(LED_BAR_PINS);

  uint16_t num_lit = (timerVal.min*60 + timerVal.sec + (STEP-1))/STEP;

  bool init_new_flashing = false;

  if(num_lit > sizeof(LED_BAR_PINS)) {
    num_lit = sizeof(LED_BAR_PINS);
  }


  if(flash) {
    num_lit--;

    if(!flash_last || (num_lit < num_lit_last)) {
      init_new_flashing = true;
    }

    num_lit_last = num_lit;

    if(init_new_flashing) {
      toggle = false; // start with it black
      next_toggle = ticks;
    }
  }

  int i;
  for(i = 0; i < num_lit && i < sizeof(LED_BAR_PINS); i++) {
    digitalWrite(LED_BAR_PINS[i], 0);
  }

  if(flash) {
    if(ticks == next_toggle) {
      next_toggle = ticks + MS_TO_TICKS(BLINK_MS);
      toggle = !toggle;
      digitalWrite(LED_BAR_PINS[num_lit], toggle);
    }
    i++;
  }

  for(; i < sizeof(LED_BAR_PINS); i++) {
    digitalWrite(LED_BAR_PINS[i], 1);
  }

  flash_last = flash;
}

int8_t read_encoder() {
  static int32_t counter = 0;

  int8_t ret;

  counter += enc.readAndReset();

  ret = counter/4;
  counter %= 4;

  return ret;
}

void display_update(bool flash) {
  static TM1637Display disp(DISP_CLK_PIN, DISP_DIO_PIN);
  static tick_type next_toggle;
  static bool toggle = false;
  static bool flash_last = false;

  if(flash) {
    if(!flash_last) {
      toggle = false; // start with it black
      next_toggle = ticks;
    }

    if(ticks == next_toggle) {
      next_toggle = ticks + MS_TO_TICKS(BLINK_MS);
      toggle = !toggle;
    }
  }

  if(state == STANDBY) {
    disp.setBrightness(DISPLAY_BRIGHTNESS_STANDBY);
  } else {
    disp.setBrightness(DISPLAY_BRIGHTNESS);
  }
  
  disp.showNumberDecEx(((int)display.min)*100+display.sec, (flash&&toggle) ? 0x00 : 0x40, true);

  flash_last = flash;
}

void update_clock() {
  if(++clock.sec == 60) {
    clock.sec = 0;
    if(++clock.min == 60) {
      clock.min = 0;
      if(++clock.hr == 24) {
        clock.hr = 0;
      }
    }
  }
}