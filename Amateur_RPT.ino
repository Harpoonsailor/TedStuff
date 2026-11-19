
// ============================================================
// ARDUINO UNO GMRS REPEATER CONTROLLER
// RUS / PTT / MORSE IDENTIFIER
// ============================================================
//
// RUS INPUT:
//   D9 LOW  = RUS ACTIVE
//   D9 HIGH = NO RUS
//
// RUS ACTIVATION QUALIFICATION:
//   LOW continuously for 75 ms
//
// RUS LOSS QUALIFICATION:
//   HIGH continuously for MORE THAN 300 ms
//
// PTT:
//   D8 = PTT output
//
// STATUS LEDs:
//   D4 = POWER LED
//   D5 = PTT LED
//   D6 = ID LED
//   D7 = RUS / COR LED
//
// MAXIMUM CONTINUOUS RUS PTT:
//   3 minutes
//
// SHORT RUS DROPOUTS:
//   300 ms or less do not reset transmit timer
//
// LOCKOUT:
//   20 seconds
//   Followed by 3-second PTT burst
//
// CW IDENTIFICATION:
//   Identifier: _____   Default for distribution = N0NE. Enter yours on Line 106
//   Frequency: approximately 490.2 Hz
//   Output: D11 (Timer2 OC2A)
//
// ID SCHEDULING:
//   Startup ID: 4 seconds
//   Normal ID interval: ~9.4 minutes
//   No-RUS ID interval: 39 minutes after initial 19-minute quiet (no RUS) period.
//   PTT lead: 400 ms
//
// RUS IDENTIFICATION WINDOW:
//   19 minutes since last actual qualified RUS
//
// ============================================================

#include <string.h>

// ============================================================
// PIN ASSIGNMENTS
// ============================================================

const byte RUS_PIN = 9;

const byte POWER_LED = 4;
const byte PTT_LED   = 5;
const byte ID_LED    = 6;
const byte RUS_LED   = 7;

const byte PTT_PIN   = 8;
const byte MORSE_PIN = 11;

// ============================================================
// TIMING
// ============================================================

const unsigned long MAX_RUS_PTT = 178200UL;

const unsigned long PTT_HOLD = 2000UL;
const unsigned long PTT_LOCKOUT = 20000UL;

const unsigned long RUS_ACTIVE_QUALIFY = 75UL;
const unsigned long RUS_LOSS_QUALIFY = 300UL;

const unsigned long ID_RUS_TIMEOUT = 1140000UL;

const unsigned long TIMEOUT_LED_INTERVAL = 200UL;

const unsigned long LOCKOUT_PTT_TIME = 3000UL;

const unsigned long ID_INTERVAL = 564000UL;

const unsigned long ID_NO_RUS_INTERVAL = 2340000UL;

const unsigned long ID_PTT_LEAD = 400UL;
const unsigned long STARTUP_ID_DELAY = 4000UL;

const unsigned long ID_FLASH_INTERVAL = 167UL;

const byte CW_TIMER_TOP = 255; // 0-255, where a lower value is higher audio freq, vice versa

const unsigned long DOT_TIME    = 60UL;
const unsigned long DASH_TIME   = 180UL;
const unsigned long ELEMENT_GAP = 65UL;
const unsigned long LETTER_GAP  = 190UL;

// ============================================================
// IDENTIFIER
// ============================================================

const char IDENTIFIER[] = "N0NE/R";

// ============================================================
// RUS STATE
// ============================================================

bool rus = false;
bool previousRus = false;

bool rusActivePending = false;
unsigned long rusActiveCandidateStart = 0;

bool rusLossPending = false;
unsigned long rusLossCandidateStart = 0;

bool rusPttActive = false;
bool pttTimedOut = false;
bool pttLockedOut = false;

unsigned long rusStartTime = 0;
unsigned long rusLossTime = 0;
unsigned long pttLockoutStart = 0;

unsigned long lastActualRusTime = 0;
bool rusIdWindowActive = false;

// ============================================================
// TIMEOUT LED ROTATION STATE
// ============================================================

byte timeoutLedStep = 0;
unsigned long timeoutLedTime = 0;

// ============================================================
// END-OF-LOCKOUT PTT/CW BURST
// ============================================================

bool lockoutPttPulseActive = false;
unsigned long lockoutPttPulseStart = 0;

// ============================================================
// IDENTIFICATION STATE
// ============================================================

bool idPending = false;
bool idActive = false;

unsigned long nextIdDue = 0;

bool idPttLeadActive = false;
unsigned long idPttLeadStart = 0;

unsigned long idFlashTime = 0;
bool idFlashState = false;

bool startupIdPending = true;
unsigned long startupTime = 0;

// ============================================================
// MORSE STATE
// ============================================================

byte morseChar = 0;
byte morseElement = 0;

bool morseToneActive = false;
unsigned long morseStateTime = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void setupCwTimer();
void setCwCarrier(bool enabled);

void updateRusInput();

void startIdentifier();
void startMorseElement();
void updateMorse();

void updateIdLed();
void updateTimeoutLeds();
void updatePttOutput();

// ============================================================
// MORSE LOOKUP
// ============================================================

const char* getMorse(char c)
{
  switch (c)
  {
    case 'A': return ".-";
    case 'B': return "-...";
    case 'C': return "-.-.";
    case 'D': return "-..";
    case 'E': return ".";
    case 'F': return "..-.";
    case 'G': return "--.";
    case 'H': return "....";
    case 'I': return "..";
    case 'J': return ".---";
    case 'K': return "-.-";
    case 'L': return ".-..";
    case 'M': return "--";
    case 'N': return "-.";
    case 'O': return "---";
    case 'P': return ".--.";
    case 'Q': return "--.-";
    case 'R': return ".-.";
    case 'S': return "...";
    case 'T': return "-";
    case 'U': return "..-";
    case 'V': return "...-";
    case 'W': return ".--";
    case 'X': return "-..-";
    case 'Y': return "-.--";
    case 'Z': return "--..";

    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";
    case '/': return "-..-.";

    default: return "";
  }
}

// ============================================================
// SET UP TIMER2 HARDWARE CW GENERATOR
// ============================================================

void setupCwTimer()
{
  TCCR2A = 0;
  TCCR2B = 0;

  TCNT2 = 0;

  OCR2A = CW_TIMER_TOP;

  TCCR2A |= _BV(COM2A0);

  TCCR2A |= _BV(WGM20);
  TCCR2B |= _BV(WGM22);

  TCCR2B |= _BV(CS21);
  TCCR2B |= _BV(CS20);

  pinMode(MORSE_PIN, OUTPUT);

  TCCR2A &= ~_BV(COM2A1);
  TCCR2A &= ~_BV(COM2A0);

  digitalWrite(MORSE_PIN, LOW);
}

// ============================================================
// ENABLE / DISABLE CW CARRIER
// ============================================================

void setCwCarrier(bool enabled)
{
  if (enabled)
  {
    TCNT2 = 0;

    TCCR2A &= ~_BV(COM2A1);
    TCCR2A |= _BV(COM2A0);
  }
  else
  {
    TCCR2A &= ~_BV(COM2A1);
    TCCR2A &= ~_BV(COM2A0);

    digitalWrite(MORSE_PIN, LOW);
  }
}

// ============================================================
// UPDATE / QUALIFY RUS INPUT
// ============================================================

void updateRusInput()
{
  unsigned long now = millis();

  bool rawRus = (digitalRead(RUS_PIN) == LOW);

  // RUS ACTIVE:
  // Must remain active for 75 ms.

  if (rawRus)
  {
    rusLossPending = false;
    rusLossCandidateStart = 0;

    if (rus)
      return;

    if (!rusActivePending)
    {
      rusActivePending = true;
      rusActiveCandidateStart = now;
      return;
    }

    if (now - rusActiveCandidateStart >= RUS_ACTIVE_QUALIFY)
    {
      rus = true;

      rusActivePending = false;
      rusActiveCandidateStart = 0;
    }

    return;
  }

  // RUS INACTIVE:
  // Must remain inactive MORE THAN 300 ms.

  rusActivePending = false;
  rusActiveCandidateStart = 0;

  if (!rus)
    return;

  if (!rusLossPending)
  {
    rusLossPending = true;
    rusLossCandidateStart = now;
    return;
  }

  if (now - rusLossCandidateStart > RUS_LOSS_QUALIFY)
  {
    rus = false;

    rusLossPending = false;
    rusLossCandidateStart = 0;

    // Genuine RUS loss permits a new
    // transmit-time cycle.

    pttTimedOut = false;
  }
}

// ============================================================
// UPDATE ID LED
// ============================================================

void updateIdLed()
{
  unsigned long now = millis();

  if (idActive)
  {
    digitalWrite(ID_LED, HIGH);
    return;
  }

  if (idPending)
  {
    if (now - idFlashTime >= ID_FLASH_INTERVAL)
    {
      idFlashTime = now;
      idFlashState = !idFlashState;
    }

    digitalWrite(ID_LED, idFlashState ? HIGH : LOW);
    return;
  }

  idFlashState = false;
  digitalWrite(ID_LED, LOW);
}

// ============================================================
// UPDATE TIMEOUT LED ROTATION
// ============================================================

void updateTimeoutLeds()
{
  unsigned long now = millis();

  if (!pttLockedOut)
    return;

  if (now - timeoutLedTime >= TIMEOUT_LED_INTERVAL)
  {
    timeoutLedTime = now;

    timeoutLedStep++;

    if (timeoutLedStep >= 3)
      timeoutLedStep = 0;
  }

  digitalWrite(PTT_LED, LOW);
  digitalWrite(RUS_LED, LOW);
  digitalWrite(POWER_LED, LOW);

  if (timeoutLedStep == 0)
  {
    digitalWrite(PTT_LED, HIGH);
  }
  else if (timeoutLedStep == 1)
  {
    digitalWrite(RUS_LED, HIGH);
  }
  else
  {
    digitalWrite(POWER_LED, HIGH);
  }
}

// ============================================================
// START IDENTIFICATION
// ============================================================

void startIdentifier()
{
  if (idActive)
    return;

  if (!rusIdWindowActive &&
      rus &&
      !lockoutPttPulseActive)
    return;

  idPending = false;

  idFlashState = false;
  idFlashTime = millis();

  idActive = true;

  morseChar = 0;
  morseElement = 0;
  morseToneActive = false;

  digitalWrite(PTT_PIN, HIGH);

  idPttLeadActive = true;

  idPttLeadStart = millis();
  morseStateTime = idPttLeadStart;
}

// ============================================================
// START MORSE ELEMENT
// ============================================================

void startMorseElement()
{
  setCwCarrier(true);

  morseToneActive = true;
  morseStateTime = millis();
}

// ============================================================
// UPDATE MORSE
// ============================================================

void updateMorse()
{
  if (!idActive)
    return;

  unsigned long now = millis();

  if (idPttLeadActive)
  {
    if (now - idPttLeadStart < ID_PTT_LEAD)
      return;

    idPttLeadActive = false;

    startMorseElement();
    return;
  }

  const char* code = getMorse(IDENTIFIER[morseChar]);

  char element = code[morseElement];

  unsigned long duration;

  if (element == '.')
    duration = DOT_TIME;
  else
    duration = DASH_TIME;

  if (morseToneActive)
  {
    if (now - morseStateTime >= duration)
    {
      setCwCarrier(false);

      morseToneActive = false;
      morseStateTime = now;
    }

    return;
  }

  unsigned long gap;

  if (morseElement < strlen(code) - 1)
    gap = ELEMENT_GAP;
  else
    gap = LETTER_GAP;

  if (now - morseStateTime < gap)
    return;

  if (morseElement < strlen(code) - 1)
  {
    morseElement++;

    startMorseElement();
    return;
  }

  morseChar++;

  if (morseChar >= strlen(IDENTIFIER))
  {
    setCwCarrier(false);

    idActive = false;
    idPttLeadActive = false;

    nextIdDue = now +
      (rusIdWindowActive ? ID_INTERVAL : ID_NO_RUS_INTERVAL);

    return;
  }

  morseElement = 0;

  startMorseElement();
}

// ============================================================
// UPDATE PTT OUTPUT
// ============================================================

void updatePttOutput()
{
  if (lockoutPttPulseActive)
  {
    digitalWrite(PTT_PIN, HIGH);
    return;
  }

  if (idActive)
  {
    digitalWrite(PTT_PIN, HIGH);
    return;
  }

  if (rusPttActive && !pttTimedOut)
  {
    digitalWrite(PTT_PIN, HIGH);
    return;
  }

  digitalWrite(PTT_PIN, LOW);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  pinMode(RUS_PIN, INPUT);

  pinMode(PTT_PIN, OUTPUT);
  pinMode(MORSE_PIN, OUTPUT);

  pinMode(POWER_LED, OUTPUT);
  pinMode(PTT_LED, OUTPUT);
  pinMode(ID_LED, OUTPUT);
  pinMode(RUS_LED, OUTPUT);

  digitalWrite(PTT_PIN, LOW);
  digitalWrite(MORSE_PIN, LOW);

  digitalWrite(POWER_LED, HIGH);
  digitalWrite(PTT_LED, LOW);
  digitalWrite(ID_LED, LOW);
  digitalWrite(RUS_LED, LOW);

  setupCwTimer();

  startupTime = millis();

  // Schedule first normal ID after startup ID interval.
  // Prevents duplicate identification at power-up.

  nextIdDue = startupTime + ID_INTERVAL;
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  unsigned long now = millis();

  updateRusInput();

  if (!pttLockedOut)
  {
    digitalWrite(RUS_LED, rus ? HIGH : LOW);
  }

  // NEW QUALIFIED RUS

  if (rus && !previousRus)
  {
    lastActualRusTime = now;
    rusIdWindowActive = true;

    nextIdDue = now + ID_INTERVAL;

    if (!pttLockedOut && !lockoutPttPulseActive)
    {
      rusStartTime = now;

      rusPttActive = true;
      pttTimedOut = false;

      rusLossTime = 0;
    }
  }

  // ACTUAL RUS ACTIVE

  if (rus)
  {
    lastActualRusTime = now;

    if (rusPttActive &&
        !pttTimedOut &&
        now - rusStartTime >= MAX_RUS_PTT)
    {
      rusPttActive = false;
      pttTimedOut = true;

      pttLockedOut = true;
      pttLockoutStart = now;

      timeoutLedTime = now;
      timeoutLedStep = 0;

      digitalWrite(PTT_PIN, LOW);
    }
  }

  // QUALIFIED RUS LOSS

  if (!rus && previousRus)
  {
    rusLossTime = now;

    if (idPending &&
        !idActive &&
        rusIdWindowActive)
    {
      startIdentifier();
    }
  }

  // NORMAL PTT HOLD AFTER RUS LOSS

  if (!rus && rusLossTime != 0 && rusPttActive)
  {
    if (now - rusLossTime >= PTT_HOLD)
    {
      rusPttActive = false;
      rusLossTime = 0;
    }
  }

  // STARTUP ID

  if (startupIdPending &&
      !idActive &&
      now - startupTime >= STARTUP_ID_DELAY)
  {
    startupIdPending = false;

    lastActualRusTime = now;
    rusIdWindowActive = true;

    startIdentifier();
  }

  // NORMAL / ALTERNATE ID SCHEDULING

  if (!idActive &&
      !idPending &&
      (long)(now - nextIdDue) >= 0 &&
      (rusIdWindowActive || !rus))
  {
    idPending = true;

    idFlashTime = now;
    idFlashState = true;

    if (!rus)
    {
      startIdentifier();
    }
  }

  // UPDATE MORSE

  updateMorse();

  // UPDATE ID LED

  updateIdLed();

  // 28.2-MINUTE NO-RUS IDENTIFICATION TIMEOUT

  if (rusIdWindowActive &&
      !rus &&
      lastActualRusTime != 0 &&
      now - lastActualRusTime >= ID_RUS_TIMEOUT)
  {
    rusIdWindowActive = false;

    idPending = false;
    idFlashState = false;

    nextIdDue = now + ID_NO_RUS_INTERVAL;

    digitalWrite(ID_LED, LOW);
  }

  // 20-SECOND PTT LOCKOUT

  if (pttLockedOut)
  {
    if (now - pttLockoutStart >= PTT_LOCKOUT)
    {
      pttLockedOut = false;

      lockoutPttPulseActive = true;
      lockoutPttPulseStart = now;

      if (rusIdWindowActive && !idActive)
      {
        startIdentifier();
      }
    }
    else
    {
      updateTimeoutLeds();
    }
  }

  // END-OF-LOCKOUT PTT BURST

  if (lockoutPttPulseActive)
  {
    digitalWrite(PTT_PIN, HIGH);

    if (now - lockoutPttPulseStart >= LOCKOUT_PTT_TIME)
    {
      lockoutPttPulseActive = false;

      if (!idActive)
      {
        digitalWrite(PTT_PIN, LOW);
      }

      digitalWrite(POWER_LED, HIGH);
      digitalWrite(RUS_LED, rus ? HIGH : LOW);
    }
  }

  // UPDATE PTT

  updatePttOutput();

  // NORMAL LED OPERATION

  if (!pttLockedOut && !lockoutPttPulseActive)
  {
    digitalWrite(POWER_LED, HIGH);

    digitalWrite(PTT_LED,
                 (rusPttActive || idActive)
                 ? HIGH
                 : LOW);

    digitalWrite(RUS_LED, rus ? HIGH : LOW);
  }

  // SAVE QUALIFIED RUS STATE

  previousRus = rus;
}
