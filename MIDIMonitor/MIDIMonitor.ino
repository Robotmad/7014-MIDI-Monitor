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

#define CC3000_TINY_DRIVER
//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#ifdef _CODERDOJO
#define WLAN_SSID       "CoderDojo"
#define WLAN_PASS       "coderdojo"
#define WLAN_SECURITY   WLAN_SEC_WPA2
#else
<<<<<<< .merge_file_a10124
//#define WLAN_SSID       "BTHub5-HP6W"
//#define WLAN_PASS       "b49e7bec56"
=======
>>>>>>> .merge_file_a10028
#define WLAN_SSID       "Robotmad"
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
#define MIDI_LED               5        // MIDI is connected
//#define NOTE_LED             6        // Playing Detected
#define SESSION_LED            7        // Minimum practice completed
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
#define WEBPAGE     "/trigger/keyboard_played_for_10_minutes/with/key/RFb7kSPvPoz7xvE9puBO-"
uint32_t ip;

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed                                    

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
#endif // _IFTTT
 

// MIDI protocol decode state machine states
#define DECODE_KEY      1
#define DECODE_VELOCITY 2

int RunningStatus = 0x90; // Default to Note On Events
int DecodeState = DECODE_KEY;
int Key = 0;
int Velocity = 0;

// Control Parameters
unsigned int u16MinSessionSeconds;

// Tracking of MIDI connection status
boolean bMIDIConnected;
unsigned long MIDIconnectedTime;

// Tracking of WIFI connection status
boolean bWIFIConnected;
unsigned long WIFIconnectedTime;

// Tracking of Sent LED status
unsigned long SENTTime;

// Tracking of NOTE LED status
boolean bNOTELEDState;
  
// Tracking of Playing
boolean bPlaying;
unsigned int notes;
unsigned long TotalNotes;
unsigned long TotalDuration;
unsigned long startPlayTime;
unsigned long lastPlayTime;
unsigned long sessionStartTime;
boolean bSession;
boolean bSessionComplete;

unsigned long lastSendTime;
boolean bDebug = false;

void setup()
{
  notes = 0;
  TotalNotes = 0;
  TotalDuration = 0;
  lastPlayTime = 0;
  sessionStartTime = 0;
  MIDIconnectedTime = 0;
  SENTTime = 0;
  bMIDIConnected = false;
  bPlaying = false;
  bSession = false;
  bSessionComplete = false;

  // Debug LED  
#ifdef DEBUG_LED
  pinMode(DEBUG_LED, OUTPUT);
#endif
  // Debug Control Input Pin (pull low to enable)
  pinMode(DEBUG_PIN, INPUT_PULLUP);
  if (!digitalRead(DEBUG_PIN))
  {
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
  pinMode(MIDI_LED, OUTPUT);  // LED
  digitalWrite(MIDI_LED, HIGH);
  
  // Notes being detected LED
#ifdef NOTE_LED  
  pinMode(NOTE_LED, OUTPUT);  // LED
  digitalWrite(NOTE_LED, HIGH);
#endif
  // Session Completed LED
  pinMode(SESSION_LED, OUTPUT);  // LED
  digitalWrite(SESSION_LED, HIGH);
  
  // WiFi Connected (and Hence READY) LED
  pinMode(WIFI_LED, OUTPUT);  // LED
  digitalWrite(WIFI_LED, HIGH);

  // Record Sent LED
  pinMode(SENT_LED, OUTPUT);  // LED
  digitalWrite(SENT_LED, HIGH);
  
  Serial.begin(115200);
  Serial1.begin(31250); // MIDI Baud Rate
    
#ifdef _BLYNK
  setupBlynk();
#endif
#ifdef _IFTTT
  setupIFTTT();
#endif
  Serial.println(F("\nReady..."));
  digitalWrite(WIFI_LED, LOW);
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
  Serial.println(F("\nWIFI connected"));
  digitalWrite(WIFI_LED, LOW);  
  bWIFIConnected = true;
  WIFIconnectedTime = tNow;
}

// Lost WIFI connection
void eventWIFIDropped(unsigned long tNow)
{
  Serial.println(F("\nWIFI dropped"));
  digitalWrite(WIFI_LED, HIGH);
  bWIFIConnected = false;
  WIFIconnectedTime = 0;
}

// detected that MIDI is connected
void eventMIDIConnected(unsigned long tNow)
{      
  Serial.println(F("\nMIDI connected"));
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
  Serial.println(F("\nMIDI dropped"));
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
  Serial.println(F("\nPlaying"));
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
  Serial.print(F("\nPlayed ("));
  Serial.print((lastPlayTime - startPlayTime)/1000);
  Serial.println(F("s)"));
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
  Serial.println(F("\nSession Started"));
  digitalWrite(SENT_LED, HIGH); // Clear Sent LED  
  SENTTime = 0;
  bSession = true;
  bSessionComplete = false;
  sessionStartTime = tNow;
}

// Minimum session duration reached
void eventSESSIONComplete(void)
{
  Serial.println(F("\nMinimum Session Achieved"));
  digitalWrite(SESSION_LED, LOW);
  bSessionComplete = true;
#ifdef _BLYNK  
  Blynk.virtualWrite(V3, HIGH);          
#endif  
}

boolean eventSESSIONSend(unsigned int TotalDuration, unsigned int TotalNotes, unsigned int SessionDuration)
{
  Serial.print(F("\nSession Summary: Played "));
  Serial.print(TotalNotes);
  Serial.print(F(" notes in "));
  Serial.print(TotalDuration);
  Serial.print(F("s; over "));
  Serial.print(SessionDuration);
  Serial.println(F("s."));
#ifdef _IFTTT
  return (IFTTTEvent(TotalDuration, TotalNotes, SessionDuration));
#endif  
}


// Timeout since session (or session has been reported) - new playing would be a new session
void eventSESSIONReset(void)
{
  Serial.println(F("\nSession Reset"));
  digitalWrite(SESSION_LED, HIGH);
  bSession = false;
  bSessionComplete = false;
  TotalDuration = 0;
  TotalNotes = 0;
#ifdef _BLYNK  
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V3, LOW);          
#endif  
}

void MIDIDecode(int cmd)
{    
  switch (cmd & 0xF0)
  {
  case (0xF0): // System Common and Real Time 
    switch (cmd & 0x0F)
    {
    case 0: // System Exclusive
    case 1: // Quarter Frame
    case 2: // Song Position pointer
    case 3: // Song Select
    case 4: // undefined
    case 5: // undefined
    case 6: // Tune Request
    case 7: // System Excllusive End
    // System Real Time
    case 8: // Timing Clock
    case 9: // undefined
    case 10:  // Start
    case 11:  // Continue
    case 12:  // Stop
    case 13:  // undefined
    case 14:  // Active Sensing
    case 15:  // System Reset
      break;
    }
    break;
       
  case 0x90:  // Note On
    //Serial.println(F("Note On"));
    RunningStatus = 0x90;
    DecodeState = DECODE_KEY;
    break;
       
  case 0x80:  // Note Off
    //Serial.println(F("Note Off"));
    RunningStatus = 0x80;
    DecodeState = DECODE_KEY;
    break;
   // Messages with two following bytes  
  case 0xA0:  // Polyphoneic Aftertouch
  case 0xB0:  // Control Change
  case 0xE0:  // Pitchbend
    DecodeState = DECODE_KEY;
    RunningStatus = cmd & 0xF0;
    break;
   // Messages with one following byte
  case 0xC0:  // Mode Change
  case 0xD0:  // Program Change
    DecodeState = DECODE_VELOCITY;
    RunningStatus = cmd & 0xF0;
    break;
                 
  default:
    switch (RunningStatus)
    {
    case 0x80:  // Note Off
    case 0x90:  // Note On
    case 0xA0:  // Polyphonic Aftertouch
      switch (DecodeState)
      {
      case DECODE_KEY:
        DecodeState = DECODE_VELOCITY;
        Key = cmd;
        break;
      case DECODE_VELOCITY:
        DecodeState = DECODE_KEY;
        Velocity = cmd;
        if ((RunningStatus == 0x90) && (Velocity > 0))
        {
          // Actually received a complete Note On
          notes++;
          Serial.print("N");
          toggleNOTE_LED();
        }
        break;                
      }
      break;
          
    default:
      //Serial.print(F("["));
      //Serial.print(cmd, DEC);
      //Serial.println(F("]"));
      break;
    }
    break;
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
}

void playingUpdate(unsigned long timeNow)
{
  TotalNotes += notes;
  
  if (lastPlayTime)
  {
    Serial.print(F("\nNotes:"));
    Serial.println(notes);
    
    TotalDuration += (timeNow - lastPlayTime);
    if (!bSessionComplete)
    {
      // Check for Practice Session Completion
      if (u16MinSessionSeconds < (TotalDuration/1000))
      {
        eventSESSIONComplete();
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
  while (Serial1.available())
  {
    if (!bMIDIConnected)
    {
      eventMIDIConnected(timeNow);
    }
    MIDIconnectedTime = timeNow;
       
    int cmd = Serial1.read();

    if (bDebug && (cmd == 'N'))
    {
      // Pretend we received a valid note
      notes++;
      Serial.print("N");
      toggleNOTE_LED();      
    }
    else
    {
      MIDIDecode(cmd);
    }
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

void processTimeouts(unsigned long timeNow)
{
  // Timeouts
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
    if ((SESSION_TIMEOUT < (timeNow - lastPlayTime)) || (!bMIDIConnected && bSessionComplete))
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

  if (SENTTime)
  {
    if (SENT_LED_TIMEOUT < (timeNow - SENTTime))
    {
      Serial.println(F("Sent LED Off"));
      digitalWrite(SENT_LED, HIGH);
      SENTTime = 0;
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
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
   
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY, WLAN_RETRIES)) 
  {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(1000);
  }  

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) 
  {
    if (!cc3000.getHostByName(WEBSITE, &ip)) 
    {
      Serial.println(F("Couldn't resolve!"));
    }
    if (ip == 0x7f000001)
    {
      Serial.println(F("Localhost workaround"));
      ip = 0; // Work around for DNS returning local host which is WRONG
              // force it to try again
    }
    delay(500);
  }
  cc3000.printIPdotsRev(ip);
  eventWIFIConnected(millis());
}

// We look for a simple string in the reply from the server to indicate success
// The parse is simple so it is important that the string starts with a unique character
#define CONFIRMATION_STRING     "Congratulations!"; // Length 15 characters
#define CONFIRMATION_LENGTH     (15)

int connectionFailures = 0;

// Send Practice Complete Event
boolean IFTTTEvent(unsigned int u16A, unsigned int u16B, unsigned int u16C)
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
  Serial.println(F("Connecting to server"));  
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);

  if (www.connected()) 
  {
    connectionFailures = 0;
    
    Serial.println(F("\nSend Event to IFTTT"));
    www.fastrprint(F("POST "));  www.fastrprint(F(WEBPAGE));   www.fastrprintln(F(" HTTP/1.1"));
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
    Serial.println(F("Parse server response:"));
        
    /* Read data until either the connection is closed, or the idle timeout is reached. */ 
    unsigned long lastRead = millis();
    const char PROGMEM expectedReply[] = CONFIRMATION_STRING;
    int index = 0;
    boolean bSuccess = false;
   
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) 
    {
      while (www.available()) 
      {
        char c = www.read();
        Serial.print(c);
        lastRead = millis();
        if (!bSuccess)
        {
          // Check for a key word in the response which confirms that the send was OK
          if (c == expectedReply[index])
          {
            Serial.print('=');
            if (CONFIRMATION_LENGTH == ++index)
            {
              // We have received the expected response
              bSuccess = true;    
            }
          }
          else if (index)
          {
            Serial.print('^');
            index = 0;        // Character missmatch - start checking again from start of string 
          }
        }
      }
    }
    
    Serial.println();
    
    if (bSuccess)
    {
      Serial.println(F("Success"));
      digitalWrite(SENT_LED, LOW); 
      SENTTime = lastRead;     
    }          
    
    Serial.println(F("Closing"));
    www.close();
    
    return(true);
  } 
  else 
  {
    Serial.println(F("Connection failed"));
    if (2 < ++connectionFailures)
    {
      // Force reconnection
      Serial.println(F("Disconnecting..."));
      cc3000.disconnect();   
      Serial.println(F("Rebooting..."));
      cc3000.reboot();   
      bWIFIConnected = false;
    }
    www.stop();    
  }
  return (false);
}

#endif  // _IFTTT
