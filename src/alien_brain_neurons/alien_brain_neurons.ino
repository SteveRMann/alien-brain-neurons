/*
  Alien Brain — flickering "neuron" effect for a WS28xx LED strip
  -----------------------------------------------------------------
  Library: FastLED
  Board:   any (Uno/Nano/ESP8266/ESP32/etc.) — set LED_PIN for your wiring

  Behavior (repeats forever), matching the requested spec:
    1. Wait a random time (WAIT_MIN_MS..WAIT_MAX_MS)
    2. Pick a random color
    3. Pick a random start pixel (START_MIN..START_MAX)
    4. Pick a random run length (LENGTH_MIN..LENGTH_MAX). If LOOP is false,
       this is clamped so start + length never runs past the end of the
       strip. If LOOP is true, it is NOT clamped — instead the run wraps
       around and continues from pixel 0 (e.g. start=18, NUM_LEDS=20 gives
       the sequence 18, 19, 0, 1, 2, ...).
    5. "Marquee" the run on, lighting one new pixel every MARQUEE_STEP_MIN_MS
       to MARQUEE_STEP_MAX_MS ms (a fresh random cadence is picked before
       each pixel, so the marquee speed jitters instead of ticking evenly)
    6. Each pixel starts dimming to black the INSTANT it lights, taking
       FADE_DURATION_MS to reach zero — independently of every other
       pixel in the run. Because the fade is much shorter than the time
       between new pixels lighting (by default), only a couple of LEDs
       are ever lit at once: a short "comet" chasing down the strip
       rather than the whole run building up and fading as one block.
    6b. The LAST pixel in the run is a "hero" pixel: instead of fading
        immediately, it holds at LAST_LED_BRIGHTNESS_MULTIPLIER x
        BRIGHTNESS for LAST_LED_BOOST_STEPS x MARQUEE_STEP_MAX_MS (a bright
        little "pop" at the end of the run, like a synapse firing),
        THEN fades to black over FADE_DURATION_MS like any other pixel.
    7. Once every pixel in the run has been spawned and finished fading,
       go back to step 1.

  The whole thing is a non-blocking millis()-based state machine (no
  delay() calls), so it's easy to drop into a larger sketch that also
  needs to do other things in loop().

  A note on the brightness boost: WS28xx LEDs have no per-pixel brightness
  channel — the only way to make one LED read as genuinely brighter than
  its already-maxed-out color value is to briefly raise FastLED's global
  strip brightness while that pixel is the dominant thing lit. Because the
  comet effect above keeps at most a couple of LEDs on at once, this reads
  correctly as "that one LED flashed brighter" rather than brightening the
  whole strip — any other still-fading pixel at that instant is already so
  dim the boost on it is imperceptible.

  Wiring notes:
    - Data line typically wants a ~300-500 ohm resistor near the strip.
    - Add a large capacitor (e.g. 1000uF) across strip + / - power leads.
    - For >~30 LEDs at full white, power the strip from a separate 5V
      supply (not the board's onboard regulator), sharing ground with it.
*/

#include <FastLED.h>

// ------------------------------ USER TUNABLES ------------------------------

#define LED_PIN            D2          // data pin driving the strip
#define LED_TYPE           WS2812B     // e.g. WS2811, WS2812, WS2812B, SK6812...
#define COLOR_ORDER        GRB         // swap if colors look wrong (e.g. RGB)
#define BRIGHTNESS         255          // global brightness ceiling, 0-255

#define WAIT_MIN_MS        500          // step 1: min gap between flashes
#define WAIT_MAX_MS        1500        // step 1: max gap between flashes

#define NUM_LEDS           20          // total LEDs on the string
#define START_MIN          0          // step 3: min random start pixel
#define START_MAX          12         // step 3: max random start pixel

#define LENGTH_MIN         10          // step 4: min random run length (10)
#define LENGTH_MAX         15          // step 4: max random run length (30)
                                        //   if LOOP is false: auto-clamped so
                                        //   start+length never exceeds NUM_LEDS.
                                        //   if LOOP is true: not clamped —
                                        //   the run wraps around to pixel 0
                                        //   instead (see LOOP below).

#define LOOP                true       // true: a run that would run past the
                                        //   last LED wraps around and keeps
                                        //   going from pixel 0, instead of
                                        //   being cut short at the strip end.
                                        //   e.g. start=18, NUM_LEDS=20 ->
                                        //   18, 19, 0, 1, 2, ...

#define MARQUEE_STEP_MIN_MS 5           // Speed: fastest ms between new pixels
#define MARQUEE_STEP_MAX_MS 50          // Speed: slowest ms between new pixels
                                         //   (a fresh random value in this
                                         //    range is picked before every
                                         //    single pixel lights)

#define FADE_DURATION_MS   25          // ms for EACH pixel to fade to black,
                                        //   starting the moment it lights
#define FADE_STEP_MS       5           // fade update resolution (smaller = smoother)

#define LAST_LED_BRIGHTNESS_MULTIPLIER 3.0   // hero flash: how many times
                                              //   brighter than BRIGHTNESS
#define LAST_LED_BOOST_STEPS           4     // hero flash hold time, in units
                                              //   of MARQUEE_STEP_MAX_MS (kept
                                              //   fixed so the hold is stable
                                              //   even though the step speed
                                              //   itself jitters)
#define LAST_LED_BOOST_DURATION_MS     (LAST_LED_BOOST_STEPS * MARQUEE_STEP_MAX_MS)

// -----------------------------------------------------------------------------

// How many pixels can be simultaneously mid-fade at once. Sized off the
// FASTEST possible marquee step (MARQUEE_STEP_MIN_MS), since that's the
// worst case for overlap — a couple of extra slots give headroom for timing
// jitter. Scales automatically if you retune FADE_DURATION_MS /
// MARQUEE_STEP_MIN_MS; on a very RAM-limited board, pushing FADE_DURATION_MS
// far above MARQUEE_STEP_MIN_MS will grow this table, so watch your RAM.
#define MAX_ACTIVE_PIXELS  (((FADE_DURATION_MS + MARQUEE_STEP_MIN_MS - 1) / MARQUEE_STEP_MIN_MS) + 2)

CRGB leds[NUM_LEDS];
uint8_t boostedBrightness = BRIGHTNESS;   // computed in setup(), clamped to 255

enum State { STATE_WAITING, STATE_SPAWNING, STATE_DRAINING };
State state = STATE_WAITING;

unsigned long stateStartTime = 0;   // when STATE_WAITING was entered
unsigned long currentWaitMs  = 0;   // randomized wait chosen for step 1

CRGB currentColor;
int  currentStart   = 0;
int  currentLength  = 0;
int  marqueeIndex   = 0;
unsigned long lastSpawnTime = 0;
unsigned long currentMarqueeStepMs = MARQUEE_STEP_MIN_MS;  // re-rolled before every pixel

unsigned long lastFadeUpdateTime = 0;
unsigned long heroBoostEndTime   = 0;   // millis() when the current hero
                                         // flash should stop being boosted

// One in-flight, independently-fading pixel.
struct ActivePixel {
  bool active;
  bool isHero;                // true for the last pixel of a run
  int index;
  CRGB color;
  unsigned long startTime;    // millis() when this pixel was lit
};
ActivePixel activePixels[MAX_ACTIVE_PIXELS];

// ------------------------------- HELPERS -----------------------------------

CRGB randomNeuronColor() {
  // Random hue, mostly-to-fully saturated, full value -> punchy alien colors.
  return CHSV(random8(), random(180, 256), 255);
}

int findFreeSlot() {
  for (int i = 0; i < MAX_ACTIVE_PIXELS; i++) {
    if (!activePixels[i].active) return i;
  }
  return -1;  // shouldn't happen given how MAX_ACTIVE_PIXELS is sized
}

int countActivePixels() {
  int n = 0;
  for (int i = 0; i < MAX_ACTIVE_PIXELS; i++) {
    if (activePixels[i].active) n++;
  }
  return n;
}

// Lights one pixel at full brightness right now, and registers it so
// updateFades() takes over on the very next fade tick. isHero marks the
// last pixel of a run, which gets the extra brightness hold first.
void spawnPixel(int ledIndex, unsigned long now, bool isHero) {
  int slot = findFreeSlot();
  if (slot < 0) return;  // all slots busy (shouldn't happen); drop silently

  activePixels[slot].active = true;
  activePixels[slot].isHero = isHero;
  activePixels[slot].index = ledIndex;
  activePixels[slot].color = currentColor;
  activePixels[slot].startTime = now;

  if (isHero) {
    heroBoostEndTime = now + LAST_LED_BOOST_DURATION_MS;
  }

  leds[ledIndex] = currentColor;
  FastLED.show();
}

void beginNewFlash(unsigned long now) {
  currentColor = randomNeuronColor();
  currentStart = random(START_MIN, START_MAX + 1);

  int length = random(LENGTH_MIN, LENGTH_MAX + 1);
  if (!LOOP && currentStart + length > NUM_LEDS) {
    length = NUM_LEDS - currentStart;   // never exceed the strip
  }
  // When LOOP is true, length is left as-is; updateSpawning() wraps the
  // pixel index with modulo NUM_LEDS instead of clamping it here.
  currentLength = length;

  marqueeIndex = 0;
  state = STATE_SPAWNING;
  // MARQUEE_STEP_MAX_MS is always >= whatever currentMarqueeStepMs holds,
  // so this guarantees the first pixel of the run spawns immediately.
  lastSpawnTime = now - MARQUEE_STEP_MAX_MS;
}

// -------------------------------- STATES ------------------------------------

void updateWaiting(unsigned long now) {
  if (now - stateStartTime >= currentWaitMs) {
    beginNewFlash(now);
  }
}

void updateSpawning(unsigned long now) {
  if (now - lastSpawnTime < currentMarqueeStepMs) return;
  lastSpawnTime = now;

  bool isLast = (marqueeIndex == currentLength - 1);
  int ledIndex = (currentStart + marqueeIndex) % NUM_LEDS;  // wraps when LOOP runs past the end
  spawnPixel(ledIndex, now, isLast);
  marqueeIndex++;

  // Roll a fresh random cadence for the gap before the *next* pixel.
  currentMarqueeStepMs = random(MARQUEE_STEP_MIN_MS, MARQUEE_STEP_MAX_MS + 1);

  if (isLast) {
    state = STATE_DRAINING;
  }
}

void updateDraining(unsigned long now) {
  if (countActivePixels() == 0) {
    state = STATE_WAITING;
    stateStartTime = now;
    currentWaitMs = random(WAIT_MIN_MS, WAIT_MAX_MS + 1);
  }
}

// Keeps FastLED's global brightness at the boosted level for exactly as
// long as a hero pixel is holding, and back to normal otherwise. Global
// brightness is the only lever WS28xx LEDs give us for making one pixel
// read as brighter than its maxed-out color value (see file header note).
void updateGlobalBrightness(unsigned long now) {
  static bool boosting = false;
  bool shouldBoost = (heroBoostEndTime != 0) && (now < heroBoostEndTime);
  if (shouldBoost != boosting) {
    FastLED.setBrightness(shouldBoost ? boostedBrightness : BRIGHTNESS);
    boosting = shouldBoost;
  }
}

// Runs regardless of state: fades every currently-lit pixel independently,
// each on its own clock starting from when it was spawned. Hero pixels hold
// at full color for LAST_LED_BOOST_DURATION_MS before their fade timer even
// starts. This is what keeps only a handful of LEDs on at any instant.
void updateFades(unsigned long now) {
  if (now - lastFadeUpdateTime < FADE_STEP_MS) return;
  lastFadeUpdateTime = now;

  bool touchedAny = false;
  for (int i = 0; i < MAX_ACTIVE_PIXELS; i++) {
    if (!activePixels[i].active) continue;
    touchedAny = true;

    unsigned long elapsed = now - activePixels[i].startTime;

    if (activePixels[i].isHero && elapsed < LAST_LED_BOOST_DURATION_MS) {
      leds[activePixels[i].index] = activePixels[i].color;  // held, full color
      continue;
    }

    unsigned long fadeElapsed =
        activePixels[i].isHero ? (elapsed - LAST_LED_BOOST_DURATION_MS) : elapsed;

    if (fadeElapsed >= FADE_DURATION_MS) {
      leds[activePixels[i].index] = CRGB::Black;
      activePixels[i].active = false;
    } else {
      uint8_t scale = 255 - (uint8_t)((fadeElapsed * 255UL) / FADE_DURATION_MS);
      CRGB c = activePixels[i].color;
      c.nscale8(scale);
      leds[activePixels[i].index] = c;
    }
  }

  if (touchedAny) FastLED.show();
}

// --------------------------------- MAIN --------------------------------------

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);

  long raw = (long)(BRIGHTNESS * LAST_LED_BRIGHTNESS_MULTIPLIER + 0.5);
  if (raw > 255) raw = 255;
  if (raw < 0) raw = 0;
  boostedBrightness = (uint8_t)raw;

  randomSeed(analogRead(A0));   // seed RNG from a floating analog pin

  for (int i = 0; i < MAX_ACTIVE_PIXELS; i++) {
    activePixels[i].active = false;
  }

  stateStartTime = millis();
  currentWaitMs = random(WAIT_MIN_MS, WAIT_MAX_MS + 1);
}

void loop() {
  unsigned long now = millis();

  updateGlobalBrightness(now);

  switch (state) {
    case STATE_WAITING:  updateWaiting(now);  break;
    case STATE_SPAWNING: updateSpawning(now); break;
    case STATE_DRAINING: updateDraining(now); break;
  }

  updateFades(now);
}
