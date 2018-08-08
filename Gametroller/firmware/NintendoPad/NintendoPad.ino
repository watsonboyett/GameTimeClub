
#include "Arduino.h"

#define P1_DATA_PIN    (20)
#define P1_STROBE_PIN  (19)
#define P1_CLOCK_PIN   (18)

#define P2_DATA_PIN    (23)
#define P2_STROBE_PIN  (22)
#define P2_CLOCK_PIN   (21)

#define HB_PIN      (13)

#define USE_SERIAL    (0)
#define USE_KEYBOARD  (1)

#define NES_A       (1U << 0)
#define NES_B       (1U << 1)
#define NES_SELECT  (1U << 2)
#define NES_START   (1U << 3)
#define NES_UP      (1U << 4)
#define NES_DOWN    (1U << 5)
#define NES_LEFT    (1U << 6)
#define NES_RIGHT   (1U << 7)

void strobe(void)
{
  digitalWrite(P1_STROBE_PIN, HIGH);
  digitalWrite(P2_STROBE_PIN, HIGH);

  delayMicroseconds(12);

  digitalWrite(P1_STROBE_PIN, LOW);
  digitalWrite(P2_STROBE_PIN, LOW);
}

uint8_t shiftin(void)
{
  bool p1_ret = digitalRead(P1_DATA_PIN);
  bool  p2_ret = digitalRead(P2_DATA_PIN);

  digitalWrite(P1_CLOCK_PIN, LOW);
  digitalWrite(P2_CLOCK_PIN, LOW);

  delayMicroseconds(6);

  digitalWrite(P1_CLOCK_PIN, HIGH);
  digitalWrite(P2_CLOCK_PIN, HIGH);

  delayMicroseconds(6);

  return (p2_ret << 1) | (p1_ret << 0);
}

uint16_t buttons(void)
{
  uint8_t p1_ret = 0;
  uint8_t p2_ret = 0;
  uint8_t i;

  digitalWrite(P1_CLOCK_PIN, HIGH);
  digitalWrite(P2_CLOCK_PIN, HIGH);

  strobe();

  for (i = 0; i < 8; i++) {
    uint8_t data = shiftin();
    p1_ret |= ((data & 0b01) >> 0) << i;
    p2_ret |= ((data & 0b10) >> 1) << i;
  }

  return ~((p2_ret << 8) | (p1_ret << 0));
}

void setup(void)
{
#if USE_SERIAL
  Serial.begin(115200);
  Serial.println("Started...");
#endif

  pinMode(HB_PIN,  OUTPUT);

  pinMode(P1_DATA_PIN, INPUT_PULLUP);
  pinMode(P1_STROBE_PIN, OUTPUT);
  pinMode(P1_CLOCK_PIN,  OUTPUT);

  pinMode(P2_DATA_PIN, INPUT_PULLUP);
  pinMode(P2_STROBE_PIN, OUTPUT);
  pinMode(P2_CLOCK_PIN,  OUTPUT);
}

uint32_t startTime;
uint32_t elapsedTime;

uint32_t loop_count = 0;
bool hb_state = LOW;
void loop(void)
{
  startTime = micros();

  if (loop_count % 32 == 0)
  {
    hb_state = !hb_state;
    digitalWrite(HB_PIN, hb_state);
  }

  uint16_t btns = buttons();
  //Serial.println(btns, BIN);

  if ((btns & NES_A) == NES_A)
  {
    //Serial.println("A");
    Keyboard.press(KEY_E);
  }
  else
  {
    Keyboard.release(KEY_E);
  }

  if ((btns & NES_B) == NES_B)
  {
    //Serial.println("B");
    Keyboard.press(KEY_Q);
  }
  else
  {
    Keyboard.release(KEY_Q);
  }

  if ((btns & NES_SELECT) == NES_SELECT)
  {
    //Serial.println("SELECT");
  }

  if ((btns & NES_START) == NES_START)
  {
    //Serial.println("START");
  }

  if ((btns & NES_UP) == NES_UP)
  {
    //Serial.println("UP");
    Keyboard.press(KEY_W);
  }
  else
  {
    Keyboard.release(KEY_W);

  }

  if ((btns & NES_DOWN) == NES_DOWN)
  {
    //Serial.println("DOWN");
    Keyboard.press(KEY_S);
  }
  else
  {
    Keyboard.release(KEY_S);
  }

  if ((btns & NES_LEFT) == NES_LEFT)
  {
    //Serial.println("LEFT");
    Keyboard.press(KEY_A);
  }
  else
  {
    Keyboard.release(KEY_A);
  }

  if ((btns & NES_RIGHT) == NES_RIGHT)
  {
    //Serial.println("RIGHT");
    Keyboard.press(KEY_D);
  }
  else
  {
    Keyboard.release(KEY_D);
  }

  loop_count++;
  elapsedTime = micros() - startTime;
  uint16_t suspend = constrain(16667U - elapsedTime, 0U, 16667U);
  // NOTE: microsecond delay is only accurate for a few thousand counts
  // so we split the delay into four smaller chunks
  suspend = suspend >> 2;
  delayMicroseconds(suspend);
  delayMicroseconds(suspend);
  delayMicroseconds(suspend);
  delayMicroseconds(suspend);
}


