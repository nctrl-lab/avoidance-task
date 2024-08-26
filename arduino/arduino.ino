// ======================================
// === teensy code for avoidance task ===
// === by Dohoung Kim                 ===
// === since 23 May, 2024             ===
// === last edited: 26 Aug, 2024      ===
// ======================================


// ===============================
// === 1. Defines and includes ===
// ===============================
#define USE_AUDIO

#ifdef USE_AUDIO
    // Audio shield for teensy 4.0: https://www.pjrc.com/audio-shield-for-teensy-4-0/
    #include <Audio.h>
    #include <Wire.h>
    #include <SPI.h>
    #include <SD.h>
    #include <SerialFlash.h>

    AudioSynthWaveformSine   sine1;
    AudioMixer4              mixer1;
    AudioOutputI2S           i2s1;
    AudioConnection          patchCord1(sine1, 0, mixer1, 0);
    AudioConnection          patchCord2(mixer1, 0, i2s1, 0);
    AudioConnection          patchCord3(mixer1, 0, i2s1, 1);

    AudioControlSGTL5000 audioShield;
#endif


// =========================
// === 2. Pin definition ===
// =========================

// Pin: Usage
//   0: Sync pulse for NI
//   1: Encoder A
//   2: Encoder B
//   3: Sound for NI 
//   4: Air
//   5: Water
//  14: Laser pin

// Audio shield related (do not use)
//   6: MEMCS
//   7: Audio DIN
//   8: Audio DOUT
//  10: SDCS
//  11: MOSI
//  12: MISO
//  13: SCK
//  15: Vol
//  18: Audio SDA
//  19: Audio SCL
//  20: Audio LRCLK
//  21: Audio BCLK
//  23: Audio MCLK

#define SYNC            0
#define ENCA            1
#define ENCB            2
#define SOUND           3
#define AIR             4
#define WATER           5
#define LASER           14

#define syncOn()        digitalWriteFast(SYNC, HIGH);
#define syncOff()       digitalWriteFast(SYNC, LOW);
#define soundOn()       {mixer1.gain(0, 1.0);digitalWriteFast(SOUND, HIGH);}
#define soundOff()      {mixer1.gain(0, 0.0);digitalWriteFast(SOUND, LOW);}
#define punishmentOn()  digitalWriteFast(AIR, HIGH);
#define punishmentOff() digitalWriteFast(AIR, LOW);
#define rewardOn()      digitalWriteFast(WATER, HIGH);
#define rewardOff()     digitalWriteFast(WATER, LOW);
#define laserOn()       digitalWriteFast(LASER, HIGH);
#define laserOff()      digitalWriteFast(LASER, LOW);
#define allOff()        {digitalWriteFast(SYNC, LOW);digitalWriteFast(AIR,LOW);digitalWriteFast(WATER,LOW);digitalWriteFast(SOUND, LOW);digitalWriteFast(LASER,LOW);mixer1.gain(0, 0.0);}


// =========================
// === 3. Task variables ===
// =========================

// *** Please carefully review the variables and modify them if necessary. ***


unsigned long now;

// sync pulse
bool syncState = false;
unsigned long syncTime;
const unsigned long SYNC_INTERVAL = 1000000; // 1 second

// trial state-related
#define TRIALSTART 0 // cue play
#define TRIALEND 1 // failed moving
#define ITI 2
#define STANDBY 9
int state = STANDBY;
int nTrial = 200;

// intertrial interval
// min interval: 20 seconds
// mean interval: 40 seconds
// max interval: MIN + 3 * (MEAN - MIN) = 80 seconds
unsigned long itiTime, itiDuration;
const unsigned long ITI_MIN = 20000000 ; // 20 seconds
const unsigned long ITI_MEAN = 40000000; // 40 seconds

// position-related
// report animal's position at 100 Hz
unsigned long positionTime;
const unsigned long POSITION_INTERVAL = 10000;

// 20 cm diameter, 512 ppr
// 512 pulse/round / (3.141592 * 20 cm/round) * 10 cm = 81.487 pulses/10 cm
int32_t previousTarget = 0;
const int32_t N_PULSE_PER_10CM = 82;
int32_t targetZoneLength = 4 * N_PULSE_PER_10CM ; // 40 cm

// speed-related
// 10 cm/s = 82 pulses/s
int32_t speed = 0;
int iSpeed = 0;
int32_t speedThreshold = (int32_t)(N_PULSE_PER_10CM / 2); // 41 pulses/s
int32_t ys[10];
int iY = 0;
bool isMoving = false;

// cue duration
unsigned long cueTime;
unsigned long cueDuration = 5000000; // 5 seconds

// reward
#define REWARD_DISABLED 0
#define REWARD_ENABLED  1
#define REWARD_ON       2
uint16_t rewardState = REWARD_DISABLED;

unsigned long rewardTime, rewardInterval;
unsigned long rewardDuration = 80000; // This decides the reward amount
const unsigned long REWARD_INTERVAL_MIN = 5000000; // 5 seconds
const unsigned long REWARD_INTERVAL_MEAN = 15000000; // 15 seconds

// punishment
#define PUNISH_STANDBY 0
#define PUNISH_ON      1
#define PUNISH_OFF     2
uint16_t punishmentState = PUNISH_STANDBY;

unsigned long punishmentTime, punishmentStart;
const unsigned long PUNISHMENT_PULSE = 200000;
const unsigned long PUNISHMENT_INTERVAL = 300000;
const unsigned long PUNISHMENT_DURATION = 5000000; // 5 seconds

// laser
#define LASER_DONE 0     // Idle state, waiting for trigger
#define LASER_ON 1       // Laser is currently on
#define LASER_OFF 2      // Laser is off, waiting for next pulse
int laserState = LASER_DONE;  // Current state

const unsigned long LASER_DURATION = 5000;            // Duration of each laser pulse (5 ms)
const unsigned long LASER_INTERVAL_MIN = 500000;      // Minimum inter-pulse interval (0.5 s)
const unsigned long LASER_INTERVAL_MEAN = 1000000;    // Mean inter-pulse interval (1 s)
unsigned long laserInterval;                          // Current inter-pulse interval (set dynamically)
unsigned long laserTime;

int N_PULSE = 300;       // Total number of pulses in a sequence
int iPulse = 0;          // Current pulse count

// packet related
char cCOM;
uint8_t sendVR, sendSync, sendTrial, sendLaser;

#define TRIAL_PACKET_SIZE 6
struct trialinfo
{
    uint32_t time;
    uint16_t iTrial;
} triallog;

#define VR_PACKET_SIZE 8
struct vrinfo
{
    uint32_t time;
    int32_t y;
} vrlog;

bool debug = false;


// =======================================
// === 4.1 Task                        ===
// =======================================
void checkTask()
{
    ////////////////////////////////////
    // ****** Task logic start ****** //
    ////////////////////////////////////

    if (state == ITI) {
        // iti duration has passed
        if (now - itiTime >= itiDuration) {
            if (triallog.iTrial >= nTrial)
                finishTrial();
            else
                startTrial();
        }
    }

    else if (state == TRIALSTART) {
        // stayed
        if (now - cueTime >= cueDuration)
        {
            endTrial();
        }

        // moved
        if (vrlog.y - previousTarget >= targetZoneLength) {
            itiTrial(1); // successful trial
        }
    }

    else if (state == TRIALEND) {
        // didn't move at all!!
        if (now - cueTime >= cueDuration + PUNISHMENT_DURATION) {
            itiTrial(0);
        }
        
        // (finally) moved
        if (vrlog.y - previousTarget >= targetZoneLength) {
            itiTrial(0); // failed trial
        }
    }

    //////////////////////////////////
    // ****** Task logic end ****** //
    //////////////////////////////////
}


// =================================
// === 4.2 Task-related function ===
// =================================
void startTrial()
{
    if (debug)
        Serial.println("startTrial");

    state = TRIALSTART;
    soundOn();
    stopReward();
    cueTime = now;
    triallog.time = now;
    ++triallog.iTrial;
    previousTarget = vrlog.y;
    sendTrial = 1;
}

void endTrial()
{
    if (debug)
        Serial.println("endTrial");

    state = TRIALEND;
    soundOff();
    punishment(1);
    triallog.time = now;
    sendTrial = 2;

}

void itiTrial(int success)
{
    if (debug)
        Serial.println("itiTrial");

    state = ITI;
    soundOff();
    punishment(0);
    if (triallog.iTrial == 0)
        itiDuration = 5000000; // 5 second
    else
        itiDuration = getInterval(ITI_MIN, ITI_MEAN);

    Serial.print("itiDuration: ");
    Serial.println(itiDuration);

    triallog.time = now;
    itiTime = now;
    if (success == 1)
        sendTrial = 3;
    else
        sendTrial = 4;
}

void finishTrial()
{
    if (debug)
        Serial.println("finishTrial");

    state = STANDBY;
    allOff();
    packetCOM(99);
}

void resetTrial()
{
    if (debug)
        Serial.println("resetTrial");

    allOff();
    syncState = false;
    syncTime = now;

    itiTime = now;
    itiDuration = ITI_MIN;

    positionTime = now;
    previousTarget = 0;

    speed = 0;
    iSpeed = 0;
    iY = 0;
    isMoving = true;

    cueTime = now;

    rewardState = REWARD_DISABLED;

    punishmentState = PUNISH_STANDBY;
    punishmentTime = now;
    punishmentStart = now;

    laserState = LASER_DONE;
    iPulse = N_PULSE; // make sure it doesn't trigger

    triallog.time = now;
    triallog.iTrial = 0;

    vrlog.time = now;
    vrlog.y = 0;

    sendVR = 0;
    sendSync = 0;
    sendTrial = 0;
    sendLaser = 0;

}

void initTrial()
{
    if (debug)
        Serial.println("initTrial");

    nTrial = 200;
}

// =====================================
// === 4.3 Position-related function ===
// =====================================
void checkTreadmill()
{
    if (now - positionTime >= POSITION_INTERVAL)
    {
        vrlog.time = now;
        positionTime = now;

        if (!debug)
            sendVR = 1;

        // check speed per 0.1 second and average speed per second
        if (iSpeed == 0) {
            checkSpeed();
            sendVR = 1;
        }
        iSpeed++;
        if (iSpeed == 10) {
            iSpeed = 0;
        }
    }
}

void checkSpeed() {
    // idx:   0  1  2  3  4  5  6  7  8  9
    // pos: -10 -9 -8 -7 -6 -5 -4 -3 -2 -1
    speed = vrlog.y - ys[iY];
    ys[iY] = vrlog.y;
    iY++;
    if (iY == 10) {
        iY = 0;
    }

    if (state == ITI) {
        if (speed >= speedThreshold) {
            if (!isMoving) {
                isMoving = true;
                stopReward();
                if (debug) {
                    Serial.print("Animals is moving, speed: ");
                    Serial.println(speed);
                }
            }
        }
        else {
            if (isMoving) {
                isMoving = false;
                nextReward();
                if (debug) {
                    Serial.println("Animals stopped moving. Start rewarding...");
                }
            }
        }
    }
}

void checkPosition() // run by interrupt
{
    // reading encoder
    if (digitalReadFast(ENCA) != digitalReadFast(ENCB))
        vrlog.y++;
    else
        vrlog.y--;
}


// ===================================
// === 4.4 Reward-related function ===
// ===================================
void nextReward() {
    rewardOff();
    rewardState = REWARD_ENABLED;
    rewardTime = now;
    rewardInterval = getInterval(REWARD_INTERVAL_MIN, REWARD_INTERVAL_MEAN);

    if (debug) {
        Serial.print("Reward interval: ");
        Serial.println(rewardInterval);
    }
}

void startReward() {
    rewardState = REWARD_ON;
    rewardTime = now;
    rewardOn();

    if (debug) {
        Serial.println("Rewarding");
    }
}

void stopReward() {
    rewardOff();
    isMoving = true;
    rewardState = REWARD_DISABLED;

    if (debug) {
        Serial.println("Reward disabled");
    }
}


// =======================================
// === 4.5 Punishment-related function ===
// =======================================
void punishment(int on)
{
    if (on == 1) {
        if (debug)
            Serial.println("punishment(1)");
        punishmentOn();
        punishmentState = PUNISH_ON;
        punishmentTime = now;
        punishmentStart = now;
    }
    else {
        if (debug)
            Serial.println("punishment(0)");
        punishmentOff();
        punishmentState = PUNISH_STANDBY;
    }
}


// ================================
// === 5. BCS-COM communication ===
// ================================
void checkCOM()
{
    if (Serial.available() > 0) // 175 ns
    {
        cCOM = Serial.read();

        // command list
        // s: start trial
        // n: set trial number
        // p: give punishment

        // e: end trial
        // f: force to end trial

        if (state == STANDBY)
        {
            if (cCOM == 's') // start trial
            {
                randomSeed(micros());
                resetTrial();
                itiTrial(0);
            }
            else if (cCOM == 'n') // set trial number
            {
                delay(10);
                nTrial = Serial.parseInt();

                if (nTrial <= 0)
                    nTrial = 200;

                Serial.print("==== nTrial: ");
                Serial.print(nTrial);
                Serial.println(" ====");
            }
            else if (cCOM == 'p') // give punishment
            {
                Serial.println("Testing punishment");
                punishment(1);
            }
            else if (cCOM == 'P') // stop punishment
            {
                Serial.println("Stopping punishment");
                punishment(0);
            }
            else if (cCOM == 'r') // reward
            {
                Serial.println("Testing reward");
                startReward();
            }
            else if (cCOM == 'w') // setting reward duration
            {
                rewardDuration = Serial.parseInt() * 1000;
                if (rewardDuration <= 0)
                    rewardDuration = 80000;
                Serial.print("Reward duration: ");
                Serial.print(rewardDuration);
                Serial.println(" microseconds");
            }
            else if (cCOM == 'l') {
                laserState = LASER_ON;
                laserOn();
                laserTime = now;
                iPulse = 0;
                sendLaser = 2;
            }
            else if (cCOM == 'L' || cCOM == 'f') {
                laserState = LASER_DONE;
                laserOff();
                laserTime = now;
                sendLaser = 10;
            }
            else if (cCOM == 'd') // debug mode
            {
                debug = true;
                Serial.println("Debug mode on");
            }
            else if (cCOM == 'D')
            {
                debug = false;
                Serial.println("Debug mode off");
            }
            else if (cCOM == 'h' || cCOM == '?') {
                Serial.println("\n\n=============== Help =============");
                Serial.println("Task related:");
                Serial.println("  s: Start trial");
                Serial.println("  n: Set trial number");
                Serial.println("  e: Finish trial");
                Serial.println("  f: Force stop");
                Serial.println("\nComponent related:");
                Serial.println("  p: Give punishment (15 seconds)");
                Serial.println("  P: Stop punishment");
                Serial.println("\n  r: Give reward");
                Serial.println("  w: Set reward duration in ms (e.g. w58");
                Serial.println("\n  l: Turn laser on");
                Serial.println("  L: Turn laser off");
                Serial.println("\n  d: Enable debug mode");
                Serial.println("  D: Disable debug mode");
                Serial.println("=====================================");
            }
        }
        else {
            if (cCOM == 'e') // finish trial, soft finish...
            {
                nTrial = triallog.iTrial;
            }
            else if (cCOM == 'f') // force end (also laser off)
            {
                finishTrial();
            }
        }
    }
}

void sendCOM()
{
    if (sendVR)
    {
        packetCOM(30);
        sendVR = 0;
    }
    if (sendSync) // 41: sync on, 40: sync off; 520 us (7 byte)
    {
        packetCOM(39+sendSync);
        sendSync = 0;
    }
    if (sendTrial) // 61: start trial, 62: end trial, 63: good trial, 64: bad trial (9 byte)
    {
        packetCOM(60+sendTrial);
        sendTrial = 0;
    }
    if (sendLaser) { // 61: laser on, 62: laser off, 69: laser finished
        packetCOM(69+sendLaser);
        sendLaser = 0;
    }
}

//// packet design (3-32 byte)
// 1. starting packet (1 byte; 0xFF)
// 2. command (1 byte; 99: trial finished)
// 3. data (4 byte for time, 29 byte for vr info)
// 4. packet end (1 byte; line feed = '\n')
void packetCOM(uint8_t command)
{
    if (debug) {        
        if (command < 40)
        {
            // Serial.print(command);
            // Serial.print(" vrlog.time:");
            // Serial.print(vrlog.time);
            // Serial.print(",y:");
            // Serial.print(vrlog.y);
            // Serial.print(",speed: ");
            // Serial.println(speed);
        }
        else if (command < 50)
        {
            //Serial.print(command);
            //Serial.print(" syncTime:");
            //Serial.println(syncTime);
        }
        else if (command < 70)
        {
            Serial.print(command);
            Serial.print(" triallog.time:");
            Serial.print(triallog.time);
            Serial.print(",iTrial:");
            Serial.println(triallog.iTrial);
        }
        else if (command < 80) {
            Serial.print(command);
            Serial.print(" laserTime:");
            Serial.print(laserTime);
            Serial.print(",iPulse:");
            Serial.println(iPulse);
        }
    }
    else {
        Serial.write(255);     // start string
        Serial.write(command); // command

        if (command < 40) // 30-39: vr or treadmill message
            Serial.write((uint8_t *)&vrlog, VR_PACKET_SIZE);
        else if (command < 50) // 40-49: sync time
            Serial.write((uint8_t *)&syncTime, 4);
        //else if (command < 60) // 50-59: lick message
        //    Serial.write((uint8_t *)&lickTime, 4);
        else if (command < 70) // 60-69: trial summary
            // 61: trial start
            // 62: trial end (failed -> punishment start)
            // 63: iti (successful trial)
            // 64: iti (failed trial)
            Serial.write((uint8_t *)&triallog, TRIAL_PACKET_SIZE);
        else if (command < 80) // 70-79: laser message
            Serial.write((uint8_t *)&laserTime, 4);
        Serial.write(13);
        Serial.write(10); // end character (line feed)
    }
}


// =================
// === 6. Timers ===
// =================
// checkSync: check sync time and generate pulse
// sync pulse will be 0.5 Hz
void checkSync()
{
    if (now - syncTime >= SYNC_INTERVAL) // This is immune to overflow.
    {
        if (syncState) {
            syncOff();

            syncState = 0;
            sendSync = 1;
        }
        else {
            syncOn();

            syncState = 1;
            sendSync = 2;
        }
        syncTime = now;
    }
}

void checkTimer() {
    // punishment
    if (punishmentState == PUNISH_ON) {
    	if (now - punishmentStart >= PUNISHMENT_DURATION) {
    	      punishmentState = PUNISH_STANDBY;
            punishmentOff();
            if (debug)
                Serial.println("Punishment finished");
    	}
        else if (now - punishmentTime >= PUNISHMENT_PULSE) {
            punishmentState = PUNISH_OFF;
            punishmentTime = now;
            punishmentOff();
        }
    }
    else if (punishmentState == PUNISH_OFF) {
        if (now - punishmentTime >= PUNISHMENT_INTERVAL) {
            punishmentState = PUNISH_ON;
            punishmentTime = now;
            punishmentOn();
        }
    }

    // reward
    if (rewardState == REWARD_ENABLED) {
        if (now - rewardTime >= rewardInterval) {
            startReward();
        }
    }
    else if (rewardState == REWARD_ON) {
        if (now - rewardTime >= rewardDuration) {
            if (state == STANDBY) {
                stopReward();
            }
            else {
                nextReward();
            }
        }
    }
}

void checkLaser() {
    if (laserState == LASER_ON) {
        if (now - laserTime >= LASER_DURATION) {
            laserState = LASER_OFF;
            laserTime = now;
            laserOff();
            iPulse++;
            if (iPulse >= N_PULSE) {
                sendLaser = 10;
                laserState = LASER_DONE;
            } else {
                sendLaser = 1;
                laserInterval = getInterval(LASER_INTERVAL_MIN, LASER_INTERVAL_MEAN);
            }
        }
    }
    else if (laserState == LASER_OFF) {
        if (now - laserTime >= laserInterval) {
            laserState = LASER_ON;
            laserTime = now;
            laserOn();
            sendLaser = 2;
        }
    }
}

unsigned long getInterval(unsigned long interval_min, unsigned long interval_mean) {
    return -logf(((float)random(500, 10001)/10000)) * (interval_mean - interval_min) + interval_min;
}


// =====================
// === 7. Initiation ===
// =====================

void setup()
{
    Serial.begin(2000000);

    pinMode(SYNC, OUTPUT);
    pinMode(ENCA, INPUT_PULLUP);
    pinMode(ENCB, INPUT_PULLUP);
    pinMode(SOUND, OUTPUT);
    pinMode(AIR, OUTPUT);
    pinMode(WATER, OUTPUT);
    pinMode(LASER, OUTPUT);

    attachInterrupt(ENCA, checkPosition, RISING);

    #ifdef USE_AUDIO
        AudioMemory(6);
        audioShield.enable();
        audioShield.volume(1.0);
        sine1.amplitude(1.0);
        sine1.frequency(5000);
    #endif

    initTrial();
    resetTrial();
    Serial.println("Ready!");
}


// ====================
// === 8. Main loop ===
// ====================

void loop()
{
    now = micros();

    if (state < STANDBY)
    {
        checkTask();
        checkTreadmill();
        checkSync();
    }
    else {
        checkLaser();
    }
    checkTimer();
    checkCOM();
    sendCOM();
}