
#include <Arduino.h>
#include <Keyboard.h>

#define P2_DATA_PIN    (7)
#define P2_STROBE_PIN  (6)
#define P2_CLOCK_PIN   (5)

#define P1_DATA_PIN    (2)
#define P1_STROBE_PIN  (8)
#define P1_CLOCK_PIN   (9)

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


#define NES_A_KEY ('x')
#define NES_B_KEY ('z')
#define NES_SELECT_KEY (KEY_RIGHT_SHIFT)
#define NES_START_KEY (KEY_RETURN)
#define NES_UP_KEY (KEY_UP_ARROW)
#define NES_DOWN_KEY (KEY_DOWN_ARROW)
#define NES_LEFT_KEY (KEY_LEFT_ARROW)
#define NES_RIGHT_KEY (KEY_RIGHT_ARROW)


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

  if ((btns & NES_A) == NES_A)
  {
    Keyboard.press(NES_A_KEY);
  }
  else
  {
    Keyboard.release(NES_A_KEY);
  }

  if ((btns & NES_B) == NES_B)
  {
    Keyboard.press(NES_B_KEY);
  }
  else
  {
    Keyboard.release(NES_B_KEY);
  }

  if ((btns & NES_SELECT) == NES_SELECT)
  {
    Keyboard.press(NES_SELECT_KEY);
  }
  else
  {
    Keyboard.release(NES_SELECT_KEY);
  }

  if ((btns & NES_START) == NES_START)
  {
    Keyboard.press(NES_START_KEY);
  }
  else
  {
    Keyboard.release(NES_START_KEY);
  }

  if ((btns & NES_UP) == NES_UP)
  {
    Keyboard.press(NES_UP_KEY);
  }
  else
  {
    Keyboard.release(NES_UP_KEY);

  }

  if ((btns & NES_DOWN) == NES_DOWN)
  {
    Keyboard.press(NES_DOWN_KEY);
  }
  else
  {
    Keyboard.release(NES_DOWN_KEY);
  }

  if ((btns & NES_LEFT) == NES_LEFT)
  {
    Keyboard.press(NES_LEFT_KEY);
  }
  else
  {
    Keyboard.release(NES_LEFT_KEY);
  }

  if ((btns & NES_RIGHT) == NES_RIGHT)
  {
    Keyboard.press(NES_RIGHT_KEY);
  }
  else
  {
    Keyboard.release(NES_RIGHT_KEY);
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


