// ============================================================
// ARDUINO UNO AMATEUR RADIO REPEATER CONTROLLER
// RUS / PTT / MORSE IDENTIFIER By K3RTA
// ============================================================

// Change the repeater ID on Line 57

#include <string.h>

// PIN ASSIGNMENTS

const byte RUS_PIN = 9;
const byte POWER_LED = 4;
const byte PTT_LED = 5;
const byte ID_LED = 6;
const byte RUS_LED = 7;
const byte PTT_PIN = 8;
const byte MORSE_PIN = 11;

// TIMING

const unsigned long MAX_RUS_PTT = 89500UL;   // Rptr Timeout, Change as desired. 
const unsigned long WARNING_TIME = 86000UL;  // Timeout warning, make this value > 3000ms less than MAX RUS PTT
const unsigned long PTT_HOLD = 3500UL;        // Squench Tail
const unsigned long PTT_LOCKOUT = 15000UL;   // Period that the repeater is off-air if MAX_RUS_PTT is exceeded


const unsigned long RUS_ACTIVE_QUALIFY = 10UL;    // Minimum RX signal duration needed for recognition 
const unsigned long RUS_LOSS_QUALIFY = 400UL;     // "Courtesy" time required for signal loss to reset TOT timer (re:picket-fencing)

const unsigned long ID_INTERVAL = 480000UL;          // Normal ID interval as long as activiity happens at least every ID_RUS_TIMEOUT period
const unsigned long ID_RUS_TIMEOUT = 972000UL;       // ID frrequency chanbges from ID_INTERVAL to the ID_NO_RUS_INTERVAL when quiet
const unsigned long ID_NO_RUS_INTERVAL = 2520000UL;  // Standby ID interval after dormancy, for ID_RUS_TIMEOUT length of time
const unsigned long STARTUP_ID_DELAY = 4000UL;       // First ID after repeater power-on

const unsigned long TIMEOUT_LED_INTERVAL = 200UL;     // Flash rate after Timeout
const unsigned long ID_PTT_LEAD = 800UL;              // Time between PTT active and an ID transmission, when it's time to ID
const unsigned long ID_FLASH_INTERVAL = 40UL;         // Flash interval during ID

const unsigned long LOCKOUT_FOLLOWUP_DELAY = 200UL;   // Delay of error message upon Timeout restoration PTT
const unsigned long LOCKOUT_CW_LEAD = 1400UL;         // Delay of TOT recovery ID after the error message

// TIMER2 FREQUENCY SETTINGS

const byte CW_NORMAL_TOP = 255;    // Tone of the station ID   1-255
const byte CW_WARNING_TOP = 124;   // Tone of Timeout warning    1-255

// MORSE TIMING

const unsigned long DOT_TIME = 60L;       //  Dot length
const unsigned long DASH_TIME = 180UL;    // = Dot x 3
const unsigned long ELEMENT_GAP = 60UL;   // = Dot
const unsigned long LETTER_GAP = 180UL;   // = Dot x 3

// IDENTIFIERS

const char IDENTIFIER[] = "N/0/N/E";           //  ** Your station ID  ** 
const char WARNING_IDENTIFIER[] = "IIIIIII";   // Timeout Warning message
const char LOCKOUT_IDENTIFIER[] = "TIME";      // Timeout restoration message

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
bool warningSent = false;
bool dormantIdMode = false;

// DEFERRED DORMANT ID

bool dormantIdDeferred = false;

// ============================================================
// TIMEOUT LED STATE
// ============================================================

byte timeoutLedStep = 0;
unsigned long timeoutLedTime = 0;

// ============================================================
// LOCKOUT PTT / CW STATE
// ============================================================

bool lockoutPttPulseActive = false;
unsigned long lockoutPttPulseStart = 0;

bool lockoutFollowupPending = false;
bool lockoutFollowupActive = false;
unsigned long lockoutFollowupStart = 0;

bool lockoutCwPending = false;
unsigned long lockoutCwStart = 0;
bool lockoutCwComplete = false;

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

bool lockoutMorseMode = false;
bool warningMorseMode = false;

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
// TIMER2 CW GENERATOR
// ============================================================

void setupCwTimer()
{
  TCCR2A = 0;
  TCCR2B = 0;

  TCNT2 = 0;
  OCR2A = CW_NORMAL_TOP;

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
// CW CARRIER CONTROL
// ============================================================

void setCwCarrier(bool enabled)
{
  if (enabled)
  {
    TCNT2 = 0;

    if (warningMorseMode)
      OCR2A = CW_WARNING_TOP;
    else
      OCR2A = CW_NORMAL_TOP;

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
// RUS INPUT QUALIFICATION
// ============================================================

void updateRusInput()
{
  unsigned long now = millis();
  bool rawRus = (digitalRead(RUS_PIN) == LOW);

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
    pttTimedOut = false;
  }
}

// ============================================================
// ID LED
// ============================================================

void updateIdLed()
{
  unsigned long now = millis();

  if (idActive || lockoutCwPending)
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
// TIMEOUT LED ROTATION
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
  digitalWrite(POWER_LED, LOW);
  digitalWrite(RUS_LED, LOW);

  if (timeoutLedStep == 0)
    digitalWrite(PTT_LED, HIGH);
  else if (timeoutLedStep == 1)
    digitalWrite(RUS_LED, HIGH);
  else
    digitalWrite(POWER_LED, HIGH);
}

// ============================================================
// START NORMAL IDENTIFICATION
// ============================================================

void startIdentifier()
{
  if (idActive)
    return;

  idPending = false;
  idFlashState = false;
  idFlashTime = millis();

  idActive = true;
  lockoutMorseMode = false;
  warningMorseMode = false;

  morseChar = 0;
  morseElement = 0;
  morseToneActive = false;

  digitalWrite(PTT_PIN, HIGH);

  idPttLeadActive = true;
  idPttLeadStart = millis();
  morseStateTime = idPttLeadStart;
}

// ============================================================
// START WARNING: SEVEN LETTER I CHARACTERS
// ============================================================

void startWarningIdentifier()
{
  if (idActive)
    return;

  idActive = true;
  warningMorseMode = true;
  lockoutMorseMode = false;

  morseChar = 0;
  morseElement = 0;
  morseToneActive = false;

  idPttLeadActive = false;

  digitalWrite(PTT_PIN, HIGH);
}

// ============================================================
// START LOCKOUT IDENTIFICATION: TIME
// ============================================================

void startLockoutIdentifier()
{
  if (idActive)
    return;

  idActive = true;

  lockoutMorseMode = true;
  warningMorseMode = false;

  morseChar = 0;
  morseElement = 0;

  morseToneActive = false;
  idPttLeadActive = false;

  lockoutCwPending = false;

  setCwCarrier(false);
  digitalWrite(PTT_PIN, HIGH);

  startMorseElement();
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

  const char* message;

  if (warningMorseMode)
    message = WARNING_IDENTIFIER;
  else if (lockoutMorseMode)
    message = LOCKOUT_IDENTIFIER;
  else
    message = IDENTIFIER;

  const char* code = getMorse(message[morseChar]);
  char element = code[morseElement];

  unsigned long duration =
    (element == '.') ? DOT_TIME : DASH_TIME;

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

  unsigned long gap =
    (morseElement < strlen(code) - 1)
    ? ELEMENT_GAP : LETTER_GAP;

  if (now - morseStateTime < gap)
    return;

  if (morseElement < strlen(code) - 1)
  {
    morseElement++;
    startMorseElement();
    return;
  }

  morseChar++;

  if (morseChar >= strlen(message))
  {
    setCwCarrier(false);

    idActive = false;
    idPttLeadActive = false;

    if (warningMorseMode)
    {
      warningMorseMode = false;
    }
    else if (lockoutMorseMode)
    {
      lockoutMorseMode = false;
      lockoutCwComplete = true;
    }
    else
    {
      nextIdDue = now +
        (dormantIdMode ? ID_NO_RUS_INTERVAL : ID_INTERVAL);
    }

    return;
  }

  morseElement = 0;
  startMorseElement();
}

// ============================================================
// PTT OUTPUT
// ============================================================

void updatePttOutput()
{
  if (lockoutPttPulseActive || idActive)
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
    digitalWrite(RUS_LED, rus ? HIGH : LOW);

  // ==========================================================
  // NEW QUALIFIED RUS
  // ==========================================================

  if (rus && !previousRus)
  {
    lastActualRusTime = now;
    rusIdWindowActive = true;

    if (dormantIdMode)
    {
      dormantIdMode = false;
      nextIdDue = now + ID_INTERVAL;

      // Defer dormant ID until RUS becomes inactive.

      dormantIdDeferred = true;

      idPending = false;
      idFlashState = false;
    }

    if (!pttLockedOut && !lockoutPttPulseActive)
    {
      rusStartTime = now;
      rusPttActive = true;
      pttTimedOut = false;
      warningSent = false;
      rusLossTime = 0;
    }
  }

  // ==========================================================
  // ACTUAL RUS ACTIVE
  // ==========================================================

  if (rus)
  {
    lastActualRusTime = now;

    // WARNING

    if (rusPttActive &&
        !pttTimedOut &&
        !warningSent &&
        now - rusStartTime >= WARNING_TIME)
    {
      warningSent = true;
      startWarningIdentifier();
    }

    // MAXIMUM PTT TIME

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

  // ==========================================================
  // QUALIFIED RUS LOSS
  // ==========================================================

  if (!rus && previousRus)
  {
    rusLossTime = now;
  }

  // ==========================================================
  // NORMAL PTT HOLD AFTER RUS LOSS
  // ==========================================================

  if (!rus && rusLossTime != 0 && rusPttActive)
  {
    if (now - rusLossTime >= PTT_HOLD)
    {
      rusPttActive = false;
      rusLossTime = 0;
    }
  }

  // ==========================================================
  // DEFERRED DORMANT ID AFTER USER TRANSMISSION
  // ==========================================================

  // ID can start immediately after qualified RUS loss.
  // It does NOT have to wait for the 3.5-second PTT tail.

  if (dormantIdDeferred &&
      !rus &&
      !idActive &&
      !pttLockedOut &&
      !lockoutPttPulseActive)
  {
    dormantIdDeferred = false;
    startIdentifier();
  }

  // ==========================================================
  // STARTUP ID
  // ==========================================================

  // Startup ID waits for RUS to become inactive.
  // It may transmit during the PTT courtesy tail.

  if (startupIdPending &&
      now - startupTime >= STARTUP_ID_DELAY &&
      !rus &&
      !idActive &&
      !pttLockedOut &&
      !lockoutPttPulseActive)
  {
    startupIdPending = false;
    lastActualRusTime = now;
    rusIdWindowActive = true;
    startIdentifier();
  }

  // ==========================================================
  // NORMAL / DORMANT ID SCHEDULING
  // ==========================================================

  // Never start a normal ID while RUS is active.
  // No need to wait for the PTT courtesy tail.

  if (!lockoutPttPulseActive &&
      !idActive &&
      (long)(now - nextIdDue) >= 0 &&
      (rusIdWindowActive || !rus))
  {
    if (!idPending)
    {
      idPending = true;
      idFlashTime = now;
      idFlashState = true;
    }

    // Start as soon as RUS is qualified inactive.

    if (!rus)
      startIdentifier();
  }

  // ==========================================================
  // 28.2-MINUTE NO-RUS TIMEOUT
  // ==========================================================

  if (rusIdWindowActive &&
      !rus &&
      lastActualRusTime != 0 &&
      now - lastActualRusTime >= ID_RUS_TIMEOUT)
  {
    rusIdWindowActive = false;
    dormantIdMode = true;

    idPending = false;
    idFlashState = false;

    nextIdDue = now + ID_NO_RUS_INTERVAL;
    digitalWrite(ID_LED, LOW);
  }

  // ==========================================================
  // PTT LOCKOUT
  // ==========================================================

  if (pttLockedOut)
  {
    if (now - pttLockoutStart >= PTT_LOCKOUT)
    {
      pttLockedOut = false;

      lockoutPttPulseActive = true;
      lockoutPttPulseStart = now;

      lockoutCwPending = true;
      lockoutCwStart = now;
      lockoutCwComplete = false;

      lockoutFollowupPending = false;
      lockoutFollowupActive = false;

      digitalWrite(PTT_PIN, HIGH);
    }
    else
    {
      updateTimeoutLeds();
    }
  }

  // ==========================================================
  // WAIT BEFORE TRANSMITTING TIME
  // ==========================================================

  if (lockoutCwPending &&
      !idActive &&
      now - lockoutCwStart >= LOCKOUT_CW_LEAD)
  {
    startLockoutIdentifier();
  }

  // ==========================================================
  // LOCKOUT RECOVERY FOLLOW-UP SEQUENCE
  // ==========================================================

  if (lockoutPttPulseActive)
  {
    // Keep transmitter keyed throughout recovery.

    digitalWrite(PTT_PIN, HIGH);

    // TIME transmission has finished.

    if (lockoutCwComplete)
    {
      lockoutCwComplete = false;

      lockoutFollowupPending = true;
      lockoutFollowupStart = now;
    }

    // Wait 200 ms after TIME before station ID.

    if (lockoutFollowupPending &&
        now - lockoutFollowupStart >=
        LOCKOUT_FOLLOWUP_DELAY)
    {
      lockoutFollowupPending = false;
      lockoutFollowupActive = true;

      startIdentifier();
    }

    // Release PTT only after ID has finished.

    if (lockoutFollowupActive && !idActive)
    {
      lockoutFollowupActive = false;
      lockoutPttPulseActive = false;

      digitalWrite(PTT_PIN, LOW);

      digitalWrite(POWER_LED, HIGH);
      digitalWrite(RUS_LED, rus ? HIGH : LOW);
    }
  }

  // ==========================================================
  // UPDATE MORSE
  // ==========================================================

  updateMorse();

  // ==========================================================
  // UPDATE ID LED
  // ==========================================================

  updateIdLed();

  // ==========================================================
  // UPDATE PTT
  // ==========================================================

  updatePttOutput();

  // ==========================================================
  // NORMAL LED OPERATION
  // ==========================================================

  if (!pttLockedOut && !lockoutPttPulseActive)
  {
    digitalWrite(POWER_LED, HIGH);

    digitalWrite(PTT_LED,
                 (rusPttActive || idActive)
                 ? HIGH
                 : LOW);

    digitalWrite(RUS_LED, rus ? HIGH : LOW);
  }

  // ==========================================================
  // SAVE QUALIFIED RUS STATE
  // ==========================================================

  previousRus = rus;
}
