/**************************************************************
 * Keyboard Practice Logging
 * Either as a live feed to Blynk
 * Or via the Maker Channel to IFTTT and then on to Numerous
 * 
 * 
 * Blynk is a platform with iOS and Android apps to control
 * Arduino, Raspberry Pi and the likes over the Internet.
 * You can easily build graphic interfaces for all your
 * projects by simply dragging and dropping widgets.
 *
 *   Downloads, docs, tutorials: http://www.blynk.cc
 *   Blynk community:            http://community.blynk.cc
 *   Social networks:            http://www.fb.com/blynkapp
 *                               http://twitter.com/blynk_app
 *
 * Blynk library is licensed under MIT license
 *
 **************************************************************
 * This code uses Adafruit CC3000 breakout
 * For which you need Adafruit_CC3000_Library library:
 *   https://github.com/adafruit/Adafruit_CC3000_Library
 *
 * Note: CC3000 Firmware version 1.14 or later is preferred.
 * getHostByName must be moved from to be always compiled rather
 * than only if CC3000_TINY_DRIVER is NOT defined.
 *
 * 1. Update pin definitions according to your setup.
 * 2. Change WiFi ssid, pass, and Blynk auth token
 * 3. Run :)
 *
 * Virtual Channels
 * V0 MIDI connected (LED)
 * V1 Playing (Notes/minute)
 * V2 Duration played (seconds)
 * V3 Session Completed (LED)
 **************************************************************/
//#define _BLYNK
#define _IFTTT
#define _JSON

//#define DEBUG_BY_DEFAULT

#define CC3000_TINY_DRIVER
//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#ifdef _CODERDOJO
#define WLAN_SSID       "CoderDojo"
#define WLAN_PASS       "coderdojo"
#define WLAN_SECURITY   WLAN_SEC_WPA2
#else
#define WLAN_SSID       "RobotmadUP"
#define WLAN_PASS       "mou3se43"
#define WLAN_SECURITY   WLAN_SEC_WPA2
#endif
#define WLAN_RETRIES    (5)

// These are the interrupt and control pins for СС3000
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  11
#define ADAFRUIT_CC3000_CS    12

// Other pins for LEDs - which are ACTIVE Low
#define WIFI_LED              18        // Wifi is connected
#define MIDI_LED               5        // MIDI is connected (flashes with notes if no seperate NOTE_LED)
//#define NOTE_LED             6        // Playing Detected (flashes with notes)
#define SESSION_LED            7        // Minimum practice completed (flashes on suspicious play detected)
#define SENT_LED               9        // Record sent to Internet

#define DEBUG_PIN             19        // DEBUG Mode Input
#define DEBUG_LED             13        // DEBUG LED (active High)

#define MIN_NOTES                    (2)  //Minimum number of notes for an update to be worthwhile
#define MIDI_CONNECTION_TIMEOUT (1000UL)  //mS max time with no bytes from USB
#define UPDATE_PERIOD           (1000UL)  //mS min period between Updates
#define PLAYING_TIMEOUT         (5000UL)  //mS maximum gap between notes before we consider that practice has paused
#define MIN_SESSION_DURATION       (600)  //S minimum practice duration 10minutes
#define DEBUG_SESSION_DURATION      (20)  //S minimum practice duration 1minute - for debugging purposes 
#define SESSION_TIMEOUT      (1800000UL)  //mS maximum gap after last note before we consider that practice session is over
#define SENT_LED_TIMEOUT       (60000UL)  //mS SENT LED on period

#if (UPDATE_PERIOD >= PLAYING_TIMEOUT)
#error 'UPDATE_PERIOD must be less than PLAYING_TIMEOUT'
#endif

#include <SPI.h>
#include <Adafruit_CC3000.h>
#ifdef _BLYNK
#include <BlynkSimpleCC3000.h>
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "767739d722264b71a1f4f5c9260351c5";
#endif

#ifdef _IFTTT
#include <ArduinoJson.h>

StaticJsonBuffer<200> jsonBuffer;

// 
// IFTTT Maker parameters: 
//     Key -- Obtained when setting up/connecting the Maker channel in IFTTT 
//   Event -- Arbitrary name for the event; used in the IFTTT recipe. 
// 
// What page to grab!
#define WEBSITE      "maker.ifttt.com"
// Christopher's Maker Channel
#define WEBPAGE1     "/trigger/KP10M/with/key/RFb7kSPvPoz7xvE9puBO-"
// Lincoln's Maker Channel
#define WEBPAGE2     "/trigger/KP10M/with/key/bpTkr8FwmbvXh3J1okO6cm"

uint32_t ip;

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed                                    

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
#endif // _IFTTT
 

// Playing Characteristics Monitoring to filter out automated play/styles
#define SUSPICIOUS_RATIO  (5) //i.e. one note representing more than 20% of those played
// Once suspicious play has been detected this can be reset by disconnecting MIDI.

// MIDI protocol decode state machine states
#define DECODE_KEY      1
#define DECODE_VELOCITY 2

#define MAX_KEY_VALUE (127)

int RunningStatus = 0x90; // Default to Note On Events (Keyboard makes heavy use of RunningStatus and so we may have missed the NoteOn) 
int DecodeState = DECODE_KEY;
int Key = 0;
//int Velocity = 0;
int DataBytesExpected = 0;
short int NotesPlayed[MAX_KEY_VALUE+1];

// Control Parameters
unsigned int u16MinSessionSeconds;

// Tracking of MIDI connection status
boolean bMIDIConnected = false;
unsigned long MIDIconnectedTime = 0;

// Tracking of WIFI connection status
boolean bWIFIConnected;
unsigned long WIFIconnectedTime;

// Tracking of Sent LED status
unsigned long SENTTime = 0;

// Tracking of NOTE LED status
boolean bNOTELEDState;
  
// Tracking of Playing
boolean bPlaying = false;
unsigned int notes = 0;
unsigned int TotalNotes = 0;
unsigned int LastAnalysisNotes = 0;
unsigned long TotalDuration = 0;
unsigned long startPlayTime;
unsigned long lastPlayTime = 0;
unsigned long sessionStartTime = 0;
boolean bSession = false;
boolean bSessionComplete = false;
boolean bSuspiciousPlay = false;

unsigned long lastSendTime;
boolean bDebug = false;

void setup()
{
  // Initialise UART for serial debug messages asap
  Serial.begin(115200);
      
  // Debug LED  
#ifdef DEBUG_LED
  pinMode(DEBUG_LED, OUTPUT);
#endif

  pinMode(DEBUG_PIN, INPUT_PULLUP);
#ifdef DEBUG_BY_DEFAULT  
  if (digitalRead(DEBUG_PIN))
#else
  // Debug Control Input Pin (pull low to enable)
  if (!digitalRead(DEBUG_PIN))
#endif  
  {
    //delay(1000);    
    //Serial.println(F("\nD"));
    bDebug = true;
    u16MinSessionSeconds = DEBUG_SESSION_DURATION;
#ifdef DEBUG_LED 
    digitalWrite(DEBUG_LED, HIGH);
#endif
  }
  else
  {
    u16MinSessionSeconds = MIN_SESSION_DURATION;  
#ifdef DEBUG_LED
    digitalWrite(DEBUG_LED, LOW);
#endif    
  }

  // Connections Established LED  
  pinMode(MIDI_LED, OUTPUT);
  digitalWrite(MIDI_LED, HIGH);
  
  // Notes being detected LED
#ifdef NOTE_LED  
  pinMode(NOTE_LED, OUTPUT);
  digitalWrite(NOTE_LED, HIGH);
#endif

  // Session Completed LED
  pinMode(SESSION_LED, OUTPUT);
  digitalWrite(SESSION_LED, HIGH);
  
  // WiFi Connected LED
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, HIGH);

  // Record Sent LED
  pinMode(SENT_LED, OUTPUT);
  digitalWrite(SENT_LED, HIGH);

  // LED test pattern so that user can check all is well...
  LEDTest();
  //LEDTest();
    
  Serial1.begin(31250); // MIDI Baud Rate
    
#ifdef _BLYNK
  setupBlynk();
#endif
#ifdef _IFTTT
  setupIFTTT();
#endif
  //Serial.println(F("\nR"));
}

#define NUM_LEDS      (4)   // number of  LEDs in test sequence
#define LED_ON_PERIOD (200) // mS
int pins[NUM_LEDS] = {MIDI_LED, SESSION_LED, SENT_LED, WIFI_LED};

void LEDTest(void)
{
  for (int j=0; j<5; j++)
  {
    for (int i=0; i<NUM_LEDS; i++)
    {
      digitalWrite(pins[i], LOW); // LED on
      delay(bDebug?(2000):LED_ON_PERIOD); // Slow flashes if debug mode
      digitalWrite(pins[i], HIGH);  // LED off
    }
  }
}

void loop()
{
  unsigned long timeNow = millis();
  MIDIScan(timeNow);
  processTimeouts(timeNow);
#ifdef _IFTTT
  if (bWIFIConnected)
  {
    if (!cc3000.checkConnected())
    {
      eventWIFIDropped(timeNow);  
    }
  }
#endif
#ifdef _BLYLNK  
  Blynk.run();
#endif  
}

// detected that WIFI is connected
void eventWIFIConnected(unsigned long tNow)
{      
  Serial.println(F("\nW"));
  digitalWrite(WIFI_LED, LOW);  
  bWIFIConnected = true;
  WIFIconnectedTime = tNow;
}

// Lost WIFI connection
void eventWIFIDropped(unsigned long tNow)
{
  //Serial.println(F("\n!W"));
  digitalWrite(WIFI_LED, HIGH);
  bWIFIConnected = false;
  WIFIconnectedTime = 0;
}

// detected that MIDI is connected
void eventMIDIConnected(unsigned long tNow)
{      
  //Serial.println(F("\nM"));
  digitalWrite(MIDI_LED, LOW);
#ifndef NOTE_LED
  bNOTELEDState = true; // MIDI LED doubles up as NOTE LED - so it is now on  
#endif
  bMIDIConnected = true;
  MIDIconnectedTime = tNow;
#ifdef _BLYNK
  Blynk.virtualWrite(V0, HIGH);
  lastSendTime = tNow;
#endif 
}

// Lost MIDI connection
void eventMIDIDropped(unsigned long tNow)
{
  //Serial.println(F("\n!M"));
  digitalWrite(MIDI_LED, HIGH);
#ifndef NOTE_LED
  bNOTELEDState = false; // MIDI LED doubles up as NOTE LED - so it is now off  
#endif
  bMIDIConnected = false;
  MIDIconnectedTime = 0;
#ifdef _BLYNK
  Blynk.virtualWrite(V0, LOW);
  lastSendTime = tNow;
#endif
}

// Playing has started
void eventPLAYStarted(unsigned long tNow)
{
  //Serial.println(F("\nP"));
#ifdef NOTE_LED
  digitalWrite(NOTE_LED, LOW);
  bNOTELEDState = true;
#endif
  bPlaying = true;
  startPlayTime = tNow;
  lastPlayTime = tNow; 
  if (!bSession)
  {
    // Session Started
    eventSESSIONStarted(tNow);
  }
#ifdef _BLYNK  
  Blynk.virtualWrite(V1, 1);          
#endif  

}

// No notes for a period
void eventPLAYStopped(unsigned long tNow)
{
  //Serial.print(F("\nPd ("));
  //Serial.print((lastPlayTime - startPlayTime)/1000);
  //Serial.println(F("s)"));
#ifdef NOTE_LED  
  digitalWrite(NOTE_LED, HIGH);
  bNOTELEDState = false; // NOTE LED - so it is now off
#else
  // With no dedicated NOTE_LED we pulse the MIDI LED to show that notes are being detected
  if (bMIDIConnected)
  {
    digitalWrite(MIDI_LED, LOW);  // Ensure that MIDI LED is left on when notes stop (as long as MIDI is connected) 
    bNOTELEDState = true;
  }
#endif  
  if (bSuspiciousPlay)
  {
    digitalWrite(SESSION_LED, HIGH);  // Ensure Session LED is Off is there has been suspicious play
  }
  bPlaying = false;
  // lastPlayTime = 0; Need to remember lastPlayTime to check for session being resumed.
#ifdef _BLYNK  
  Blynk.virtualWrite(V1, 0);          
  lastSendTime = tNow;
#endif  
}


// Session started
void eventSESSIONStarted(unsigned long tNow)
{
  //Serial.println(F("\nS"));
  digitalWrite(SENT_LED, HIGH); // Clear Sent LED  
  SENTTime = 0;
  bSession = true;
  bSessionComplete = false;
  sessionStartTime = tNow;
  // Clear record of how many times each note has been played...
  for (short int key = 0; key < MAX_KEY_VALUE; key++)
  {
    NotesPlayed[key]=0;
  }
}

// Minimum session duration reached
void eventSESSIONComplete(void)
{
  //Serial.println(F("\nDone"));
  digitalWrite(SESSION_LED, LOW);
  bSessionComplete = true;
#ifdef _BLYNK  
  Blynk.virtualWrite(V3, HIGH);          
#endif  
}

boolean eventSESSIONSend(unsigned int TotalDuration, unsigned int TotalNotes, unsigned int SessionDuration)
{
//Serial.print(F("\nSummary: "));
//Serial.print(TotalNotes);
//Serial.print(F(" notes in "));
//Serial.print(TotalDuration);
//Serial.print(F("s; over "));
//Serial.print(SessionDuration);
//Serial.println(F("s."));
  
#ifdef _IFTTT
  IFTTTEvent(F(WEBPAGE2),TotalDuration, TotalNotes, SessionDuration);  
  return (IFTTTEvent(F(WEBPAGE1),TotalDuration, TotalNotes, SessionDuration));
#endif  
}


// Timeout since session (or session has been reported) - new playing would be a new session
void eventSESSIONReset(void)
{
  //Serial.println(F("\nRst"));
  digitalWrite(SESSION_LED, HIGH);
  bSession = false;
  bSessionComplete = false;
  bSuspiciousPlay = false;
  TotalDuration = 0;
  TotalNotes = 0;
#ifdef _BLYNK  
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V3, LOW);          
#endif  
}

// Suspicious Play
void eventSuspiciousPlay(void)
{
  //eventSESSIONReset();
  // Make the Session LED flash with the Notes
  bSuspiciousPlay = true;  
}

#define MAX_SYSTEM_EXCLUSIVE_LEN  (30000)

void MIDIDecode(int cmd)
{
  if ((cmd & 0xF0) == 0xF0)    
  {
    // System Common and Real Time 
    switch (cmd & 0x0F)
    {
    // System Common  
    case 0: // System Exclusive
      //Serial.println(F("se")); 
      // variable number of data bytes
      DataBytesExpected = MAX_SYSTEM_EXCLUSIVE_LEN;
      break;
    case 1: // Quarter Frame
      //Serial.println(F("qf"));  
      // 1 byte of data
      //DataBytesExpected = 1;
      //break;
      // fall through
    case 3: // Song Select
      //Serial.println(F("ss"));  
      // 1 byte of data
      DataBytesExpected = 1;
      break;
    case 2: // Song Position pointer
      //Serial.println(F("spp"));      
      // 2 bytes of data
      DataBytesExpected = 2;
      break;      

    case 7: // System Exclusive End
      //Serial.println(F("see")); 
      DataBytesExpected = 0;
    default:          
    //case 4: // undefined
    //case 5: // undefined
    //case 6: // Tune Request
    // System Real Time (can be interleaved with a System Exclusive)
    //case 8: // Timing Clock
      // These events seen in Yamaha Keyboard output...
    //case 9: // undefined
    //case 10:  // Start
    //case 11:  // Continue
    //case 12:  // Stop
    //case 13:  // undefined
    //case 14:  // Active Sensing
    //case 15:  // System Reset
      // These events seen in Yamaha Keyboard output...
      //Serial.print(F("{"));
      //Serial.print(cmd, DEC);
      //Serial.println(F("}"));    
      break;
    }
  }
  else if (0 != (cmd & 0x80))
  {
    // Top Bit Set
    RunningStatus = cmd & 0xF0;
    switch (RunningStatus)
    {
    case 0x90:  // Note On    
      //Serial.println(F("Note On"));
      //DecodeState = DECODE_KEY;
      //break;
      // fall through       
    case 0x80:  // Note Off
      //Serial.println(F("Note Off"));
      DecodeState = DECODE_KEY;
      DataBytesExpected = 0;
      break;
    default:
      //Serial.print(F("["));
      //Serial.print(cmd, DEC);
      //Serial.println(F("]"));  
      break;
    // Messages with two following bytes  
    //case 0xA0:  // Polyphoneic Aftertouch
    //case 0xB0:  // Control Change
    //case 0xE0:  // Pitchbend
      //DataBytesExpected = 2; 
      //DecodeState = DECODE_KEY;
      //RunningStatus = cmd & 0xF0;
      //break;
    // Messages with one following byte
    //case 0xC0:  // Mode Change
    //case 0xD0:  // Program Change
      //DataBytesExpected = 1;  
      //DecodeState = DECODE_VELOCITY;
      //RunningStatus = cmd & 0xF0;
      //break;
    }
  }           
  else
  {
    // Top bit NOT set
    if (0 < DataBytesExpected)
    {
      // We have received a message which is followed by data bytes which we need to skip over
      //Serial.println(F("-"));
      DataBytesExpected--;
    }
    else
    {
      switch (RunningStatus)
      {
      //case 0xA0:  // Polyphoneic Aftertouch
      //case 0xB0:  // Control Change
      //case 0xE0:  // Pitchbend
        //DataBytesExpected = 2; 
        //DecodeState = DECODE_KEY;
        //RunningStatus = cmd & 0xF0;
        //break;
      // Messages with one following byte
      //case 0xC0:  // Mode Change
      //case 0xD0:  // Program Change
        //DataBytesExpected = 1;  
        //DecodeState = DECODE_VELOCITY;
        //RunningStatus = cmd & 0xF0;
        // ignore data bytes for the commands
        //break;
        
      case 0x80:  // Note Off
      case 0x90:  // Note On
        switch (DecodeState)
        {
        case DECODE_KEY:
          DecodeState = DECODE_VELOCITY;
          Key = cmd;
          break;
        case DECODE_VELOCITY:
          DecodeState = DECODE_KEY;
          //Velocity = cmd;
          if ((0x90 == RunningStatus) && (cmd > 0))
          {
            // Actually received a complete Note On
            NotesPlayed[Key]++; // Increase count of specific note
            notes++;
            //Serial.print("N");
            toggleNOTE_LED();
          }
          break;
        default:
          break;                
        }
        break;
      
      default:
        // Ignore data byte from any other command
        //Serial.println(F("D?"));
        break;
      }
    }  
  }
}

void toggleNOTE_LED(void)
{
  bNOTELEDState = !bNOTELEDState;
  
#ifdef NOTE_LED    
  digitalWrite(NOTE_LED, bNOTELEDState); 
#else
  // Toggle MIDI_LED to show that playing had been detected (as there is no dedicated NOTE_LED)
  digitalWrite(MIDI_LED, bNOTELEDState);
#endif
  if (bSuspiciousPlay)
  {
    // If we have detected suspicious play then flash the session LED as well
    digitalWrite(SESSION_LED, bNOTELEDState);  
  }
}

// Check for Automated play - style
boolean playAnalysis(void)
{
  short int DiscreteNotes = 0;
  short int MaxTimesPlayed = 0;

  for (short int key = 0; key <= MAX_KEY_VALUE; key++)
  {
    // Scan all played notes to compile some statistics
    if (NotesPlayed[key])
    {
      if (MaxTimesPlayed < NotesPlayed[key]) MaxTimesPlayed = NotesPlayed[key];
      // Other ideas - second most played note?
      // 
      DiscreteNotes++;
    }
  }
  
  // Decision Logic
  //Serial.print(F("Tot:")); Serial.println(TotalNotes);
  //Serial.print(F("Max:")); Serial.println(MaxTimesPlayed);
  //Serial.print(F("Dis:")); Serial.println(DiscreteNotes);
  
  if (MaxTimesPlayed > (TotalNotes / SUSPICIOUS_RATIO))
  {
    //Serial.println(F("Suspicious!"));
    return(true);
  }
  return(false); 
}


void playingUpdate(unsigned long timeNow)
{
  TotalNotes += notes;
  
  if (lastPlayTime)
  {
    //Serial.print(F("\nN:")); Serial.println(notes);
    
    TotalDuration += (timeNow - lastPlayTime);
    if (!bSessionComplete)
    {
      if (500 < (TotalNotes - LastAnalysisNotes))
      {
        LastAnalysisNotes = TotalNotes;
        if (playAnalysis())
        {
          eventSuspiciousPlay();
        }
      }
      if (!bSuspiciousPlay)
      {
        // Check for Practice Session Completion
        if (u16MinSessionSeconds < (TotalDuration/1000))
        {
          eventSESSIONComplete();
        }
      }
    } 
  }
  lastPlayTime = timeNow;
#ifdef _BLYNK      
  Blynk.virtualWrite(V1, (60000UL/UPDATE_PERIOD) * notes);
  Blynk.virtualWrite(V2, TotalDuration/1000);
#endif
  lastSendTime = timeNow;
  notes = 0;
}
      
void MIDIScan(unsigned long timeNow)
{
  // Debugging assistance
#ifdef _NEVER   
  if (bDebug)
  {
    if (Serial.available())
    {
      int testCmd = Serial.read();
      switch (testCmd)
      {  
//    case 'D': // Debug
//      Serial.println(F("Debug Mode"));
//      bDebug = true;
//      u16MinSessionSeconds = DEBUG_SESSION_DURATION;
//      break;
      
//    case 'M': // Fake MIDI Connection
//      eventMIDIConnected(timeNow);
//      break;
  
//    case 'N': // Fake Note
//      notes++;
//      toggleNOTE_LED();
//      break;

      case 'P': // Fake Playing
        eventPLAYStarted(timeNow);
        break;
      
//    case 'S': // Fake Session Started
//      eventSESSIONStarted(timeNow);
//      break;
        
      case 'C': // Fake Session Complete
        eventSESSIONComplete();
        break;
          
      default:
//      Serial.println(F("Unrecognised Cmd"));
        break;
      }
    }
  }
#endif
  
  while (Serial1.available())
  {
    if (!bMIDIConnected)
    {
      eventMIDIConnected(timeNow);
    }
    MIDIconnectedTime = timeNow;     
    MIDIDecode(Serial1.read());
  }
    
  if (MIN_NOTES < notes)
  {
    if (!bPlaying)
    {
      eventPLAYStarted(timeNow);
    }
    else if ((timeNow - lastSendTime) >= UPDATE_PERIOD)
    {
      // at least XXXmS since last data sent
      playingUpdate(timeNow);
    }
  }
}

// Timeouts
void processTimeouts(unsigned long timeNow)
{
  // Check the SENT LED Timeout BEFORE running the session checks as they can
  // lead to it being started and we don't want to turn it off in the same pass.
  if (SENTTime)
  {
    if (SENT_LED_TIMEOUT < (timeNow - SENTTime))
    {
      //Serial.println(F("!St"));
      digitalWrite(SENT_LED, HIGH);
      SENTTime = 0;
    }
  }
  
  if (bPlaying)
  {
    if (PLAYING_TIMEOUT < (timeNow - lastPlayTime))
    {
      // No notes for a while - keyboard is not being played
      if (notes) playingUpdate(timeNow); // Include residual time and notes in record of session
      eventPLAYStopped(timeNow);
    } 
  }
  else if (bSession)
  {
    // Time period since last note or MIDI disconnected for timeout period
    if ((SESSION_TIMEOUT < (timeNow - lastPlayTime)) || (!bMIDIConnected && (bSuspiciousPlay || bSessionComplete)))
    {
      // No practice for a while - session has ended
      if (bSessionComplete)
      {
        // There was enough practice (total playing duration) for this to count
        if (eventSESSIONSend(TotalDuration/1000, TotalNotes, (lastPlayTime - sessionStartTime)/1000))
        {
          eventSESSIONReset();
        }
        else
        {
          // Will naturally retry sending as we have left the session open
        }
      }
      else
      {
        eventSESSIONReset();
      }
    }
  }
  
  if (bMIDIConnected)
  {
    if (MIDI_CONNECTION_TIMEOUT < (timeNow - MIDIconnectedTime))
    {
      eventMIDIDropped(timeNow);
    }
  }
}

//////////////////////////////////

#ifdef _BLYNK
void setupBlynk(void)
{
  Blynk.begin(auth, WLAN_SSID, WLAN_PASS, WLAN_SECURITY);
  while (Blynk.connect() == false) 
  {
    // Wait until connected
  }

  // Notify immediately on startup
  Serial.println(F("Blynk Connected"));
  Blynk.notify("Device started");
  Blynk.virtualWrite(V0, LOW);  // MIDI not connected (yet)
  Blynk.virtualWrite(V1, 0);    // No notes being played (yet)
  Blynk.virtualWrite(V2, 0);    // No duration played (yet)
  Blynk.virtualWrite(V3, LOW);  // Session not complete (yet)
  lastSendTime = millis();
  eventWIFIConnected(millis());
}
#endif

//////////////////////////////////

#ifdef _IFTTT
void setupIFTTT(void)
{
  /* Initialise the module */
  //Serial.println(F("\nInit"));
  if (!cc3000.begin())
  {
    //Serial.println(F("H/W?"));
    while(1);
  }
   
  //Serial.print(F("\nC:")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY, WLAN_RETRIES)) 
  {
    //Serial.println(F("Fail"));
    while(1);
  }
  //Serial.println(F("d"));
  
  /* Wait for DHCP to complete */
  //Serial.println(F("DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(1000);
  }  

  ip = 0;
  // Try looking up the website's IP address
  //Serial.print(WEBSITE); Serial.print(F("->"));
  while (ip == 0) 
  {
    if (!cc3000.getHostByName(WEBSITE, &ip)) 
    {
      //Serial.println(F("!ip"));
    }
    if (ip == 0x7f000001)
    {
      //Serial.println(F("wa"));
      ip = 0; // Work around for DNS returning local host which is WRONG
              // force it to try again
    }
    delay(500);
  }
  digitalWrite(WIFI_LED, LOW);
  cc3000.printIPdotsRev(ip);
  eventWIFIConnected(millis());
}

// We look for a simple string in the reply from the server to indicate success
// The parse is simple so it is important that the string starts with a unique character
#define CONFIRMATION_STRING     "Congratulations!"; // Length 15 characters
#define CONFIRMATION_LENGTH     (15)

int connectionFailures = 0;

// Send Practice Complete Event
boolean IFTTTEvent(const __FlashStringHelper* pWebPage, unsigned int u16A, unsigned int u16B, unsigned int u16C)
{
  if (bWIFIConnected)
  {
    // We think we have a connection - check again now (including DHCP)
    if (!cc3000.checkConnected() || !cc3000.checkDHCP())
    {
      eventWIFIDropped(millis());  
    }
  }
  if (!bWIFIConnected)
  {
    // If we are not connected then try to restore connection...
    setupIFTTT();
  }

  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  //Serial.println(F("S"));  
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);

  if (www.connected()) 
  {
    connectionFailures = 0;
    
    //Serial.println(F("\nSend"));
    www.fastrprint(F("POST "));  www.fastrprint(pWebPage);   www.fastrprintln(F(" HTTP/1.1"));
    www.fastrprint(F("Host: ")); www.fastrprintln(F(WEBSITE));
    
#ifdef _JSON
    char szLen[3];    
    JsonObject& root = jsonBuffer.createObject();    
    root["value1"] = u16A;
    root["value2"] = u16B;
    root["value3"] = u16C;

    int len = root.measureLength();
    
    www.fastrprintln(F("Content-Type: application/json"));
    www.fastrprint(F("Content-Length: "));
    szLen[0] = '0' + (len/10); 
    szLen[1] = '0' + (len%10);
    szLen[2] = NULL;
    www.fastrprintln(szLen);
    www.fastrprintln("");
    root.printTo(www);    
#endif    
    www.fastrprintln("");
    www.println();
    //Serial.println(F("Parse"));
        
    if (IFTTTCheckReply(&www))
    {
      //Serial.println(F("OK"));
      digitalWrite(SENT_LED, LOW); 
      SENTTime = millis();     
    }          
    
    //Serial.println(F("C"));
    www.close();
    
    return(true);
  } 
  else 
  {
    //Serial.println(F("F"));
    if (2 < ++connectionFailures)
    {
      // Force reconnection
      //Serial.println(F("D"));
      cc3000.disconnect();   
      //Serial.println(F("R"));
      cc3000.reboot();   
      bWIFIConnected = false;
    }
    www.stop();    
  }
  return (false);
}

bool IFTTTCheckReply(Adafruit_CC3000_Client *www)
{
    /* Read data until either the connection is closed, or the idle timeout is reached. */ 
    unsigned long lastRead = millis();
    const char PROGMEM expectedReply[] = CONFIRMATION_STRING;
    int index = 0;
    boolean bSuccess = false;
   
    while (www->connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) 
    {
      while (www->available()) 
      {
        char c = www->read();
        //Serial.print(c);
        lastRead = millis();
        if (!bSuccess)
        {
          // Check for a key word in the response which confirms that the send was OK
          if (c == expectedReply[index])
          {
            //Serial.print('=');
            if (CONFIRMATION_LENGTH == ++index)
            {
              // We have received the expected response
              bSuccess = true;    
            }
          }
          else if (index)
          {
            //Serial.print('^');
            index = 0;        // Character missmatch - start checking again from start of string 
          }
        }
      }
    }
    return(bSuccess);
}

#endif  // _IFTTT
