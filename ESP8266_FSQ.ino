//
// Simple FSQ beacon for ESP8266, with the Etherkit Si5351A Breakout 
// Board, by Jason Milldrum NT7S.
//
// Adapted for ESP8266 by Marco Campinoti IU5HKU 28/03/2019
// This release uses the timer1 hardware interrupt for the required precise timing
// For the moment i've powered off the WiFi, maybe in a future release i'll use
// it for something like an interactive webpage to change frequency ad text 
// of the beacon.
// 
// Original code based on Feld Hell beacon for Arduino by Mark 
// Vandewettering K6HX, adapted for the Si5351A by Robert 
// Liesenfeld AK6L <ak6l@ak6l.org>.  Timer setup
// code by Thomas Knutsen LA3PNA.
// 
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include <si5351.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ticker.h>

#define TONE_SPACING            879ULL        // ~8.7890625 Hz
#define BAUD_2                  7812          // CTC value for 2 baud
#define BAUD_3                  5208          // CTC value for 3 baud
#define BAUD_4_5                3472          // CTC value for 4.5 baud
#define BAUD_6                  2604          // CTC value for 6 baud

#define LED_PIN                 D4

// Global variables
Si5351 si5351;
//unsigned long freq = 7105350;   // Base freq is 1350 Hz higher than dial freq in USB
unsigned long long freq = 144400000;   // Base freq is 1350 Hz higher than dial freq in USB
#define CORRECTION   58000      // Change this for your ref osc -12000 

uint8_t cur_tone = 0;
static uint8_t crc8_table[256];
char callsign[10] = "IU5HKU";
char tx_buffer[40];
uint8_t callsign_crc;

// Global variables used in ISRs
volatile bool proceed = false;

// Define the structure of a varicode table
typedef struct fsq_varicode
{
    uint8_t ch;
    uint8_t var[2];
} Varicode;
 
// The FSQ varicode table, based on the FSQ Varicode V3.0
// document provided by Murray Greenman, ZL1BPU

const Varicode code_table[] PROGMEM =
{
  {' ', {00, 00}}, // space
  {'!', {11, 30}},
  {'"', {12, 30}},
  {'#', {13, 30}},
  {'$', {14, 30}},
  {'%', {15, 30}},
  {'&', {16, 30}},
  {'\'', {17, 30}},
  {'(', {18, 30}},
  {')', {19, 30}},
  {'*', {20, 30}},
  {'+', {21, 30}},
  {',', {27, 29}},
  {'-', {22, 30}},
  {'.', {27, 00}},
  {'/', {23, 30}},
  {'0', {10, 30}},
  {'1', {01, 30}},
  {'2', {02, 30}},
  {'3', {03, 30}},
  {'4', {04, 30}},
  {'5', {05, 30}},
  {'6', {06, 30}},
  {'7', {07, 30}},
  {'8', {8, 30}},
  {'9', {9, 30}},
  {':', {24, 30}},
  {';', {25, 30}},
  {'<', {26, 30}},
  {'=', {00, 31}},
  {'>', {27, 30}},
  {'?', {28, 29}},
  {'@', {00, 29}},
  {'A', {01, 29}},
  {'B', {02, 29}},
  {'C', {03, 29}},
  {'D', {04, 29}},
  {'E', {05, 29}},
  {'F', {06, 29}},
  {'G', {07, 29}},
  {'H', {8, 29}},
  {'I', {9, 29}},
  {'J', {10, 29}},
  {'K', {11, 29}},
  {'L', {12, 29}},
  {'M', {13, 29}},
  {'N', {14, 29}},
  {'O', {15, 29}},
  {'P', {16, 29}},
  {'Q', {17, 29}},
  {'R', {18, 29}},
  {'S', {19, 29}},
  {'T', {20, 29}},
  {'U', {21, 29}},
  {'V', {22, 29}},
  {'W', {23, 29}},
  {'X', {24, 29}},
  {'Y', {25, 29}},
  {'Z', {26, 29}},
  {'[', {01, 31}},
  {'\\', {02, 31}},
  {']', {03, 31}},
  {'^', {04, 31}},
  {'_', {05, 31}},
  {'`', {9, 31}},
  {'a', {01, 00}},
  {'b', {02, 00}},
  {'c', {03, 00}},
  {'d', {04, 00}},
  {'e', {05, 00}},
  {'f', {06, 00}},
  {'g', {07, 00}},
  {'h', {8, 00}},
  {'i', {9, 00}},
  {'j', {10, 00}},
  {'k', {11, 00}},
  {'l', {12, 00}},
  {'m', {13, 00}},
  {'n', {14, 00}},
  {'o', {15, 00}},
  {'p', {16, 00}},
  {'q', {17, 00}},
  {'r', {18, 00}},
  {'s', {19, 00}},
  {'t', {20, 00}},
  {'u', {21, 00}},
  {'v', {22, 00}},
  {'w', {23, 00}},
  {'x', {24, 00}},
  {'y', {25, 00}},
  {'z', {26, 00}},
  {'{', {06, 31}},
  {'|', {07, 31}},
  {'}', {8, 31}},
  {'~', {00, 30}},
  {127, {28, 31}}, // DEL
  {13,  {28, 00}}, // CR
  {10,  {28, 00}}, // LF
  {0,   {28, 30}}, // IDLE
  {241, {10, 31}}, // plus/minus
  {246, {11, 31}}, // division sign
  {248, {12, 31}}, // degrees sign
  {158, {13, 31}}, // multiply sign
  {156, {14, 31}}, // pound sterling sign
  {8,   {27, 31}}  // BS
};
 
// Define an upper bound on the number of glyphs.  Defining it this
// way allows adding characters without having to update a hard-coded
// upper bound.
#define NGLYPHS         (sizeof(code_table)/sizeof(code_table[0]))

// Timer interrupt vector.  This toggles the variable we use to gate
// each column of output to ensure accurate timing.  Called whenever
// Timer1 hits the count set below in setup().
void ICACHE_RAM_ATTR onTimerISR(){
    proceed = true;
}

// This is the heart of the beacon.  Given a character, it finds the
// appropriate glyph and sets output from the Si5351A to key the
// FSQ signal.
void encode_char(int ch)
{
    uint8_t i, fch, vcode1, vcode2;

    for(i = 0; i < NGLYPHS; i++)
    {
        // Check each element of the varicode table to see if we've found the
        // character we're trying to send.
        fch = pgm_read_byte(&code_table[i].ch);

        if(fch == ch)
        {
            // Found the character, now fetch the varicode chars
            vcode1 = pgm_read_byte(&(code_table[i].var[0]));
            vcode2 = pgm_read_byte(&(code_table[i].var[1]));
            
            // Transmit the appropriate tone per a varicode char
            if(vcode2 == 0)
            {
              // If the 2nd varicode char is a 0 in the table,
              // we are transmitting a lowercase character, and thus
              // only transmit one tone for this character.

              // Generate tone
              encode_tone(vcode1);
              while(!proceed)   // Wait for the timer interrupt to fire
                ;               // before continuing.
              noInterrupts();   // Turn off interrupts; just good practice...
              proceed = false;  // reset the flag for the next character...
              interrupts();     // and re-enable interrupts.
            }
            else
            {
              // If the 2nd varicode char is anything other than 0 in
              // the table, then we need to transmit both

              // Generate 1st tone
              encode_tone(vcode1);
              while(!proceed)   // Wait for the timer interrupt to fire
                ;               // before continuing.
              noInterrupts();   // Turn off interrupts; just good practice...
              proceed = false;  // reset the flag for the next character...
              interrupts();     // and re-enable interrupts.

              // Generate 2nd tone
              encode_tone(vcode2);
              while(!proceed)   // Wait for the timer interrupt to fire
                ;               // before continuing.
              noInterrupts();   // Turn off interrupts; just good practice...
              proceed = false;  // reset the flag for the next character...
              interrupts();     // and re-enable interrupts.
            }           
            break; // We've found and transmitted the char,
               // so exit the for loop
        }
    }
}


void encode_tone(uint8_t tone)
{
  cur_tone = ((cur_tone + tone + 1) % 33);
  //Serial.println(cur_tone);
  si5351.set_freq((freq * 100ULL) + (cur_tone * TONE_SPACING), SI5351_CLK0);
}
 
// Loop through the string, transmitting one character at a time.
void encode(char *str)
{
    // Reset the tone to 0 and turn on the output
    cur_tone = 0;
    si5351.output_enable(SI5351_CLK0, 1);
    digitalWrite(LED_PIN, LOW);
    //Serial.println("=======");

    // Transmit BOT
    noInterrupts();
    proceed = false;
    interrupts();
    encode_char(' '); // Send a space for the dummy character
    while(!proceed);
    noInterrupts();
    proceed = false;
    interrupts();

    // Send another space
    encode_char(' ');
    while(!proceed);
    noInterrupts();
    proceed = false;
    interrupts();

    // Now send LF
    encode_char(10);
    while(!proceed);
    noInterrupts();
    proceed = false;
    interrupts();
    
    // Now do the rest of the message
    while (*str != '\0')
    {
        encode_char(*str++);
    }
        
    // Turn off the output
    si5351.output_enable(SI5351_CLK0, 0);
    digitalWrite(LED_PIN, HIGH);
}

static void init_crc8(void)
{
  int i,j;
  uint8_t crc;

  for(i = 0; i < 256; i++)
  {
    crc = i;
    for(j = 0; j < 8; j++)
    {
      crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
    }
    crc8_table[i] = crc & 0xFF;
  }
}

uint8_t crc8(char * text)
{
  uint8_t crc='\0';
  uint8_t ch;
  int i;
  for(i = 0; i < strlen(text); i++)
  {
    ch = text[i];
    crc = crc8_table[(crc) ^ ch];
    crc &= 0xFF;
  }

  return crc;
}
 
void setup()
{
  // Use the ESP8266's on-board LED as a keying indicator.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  //Serial.begin(57600);

  // Shutdown the WiFi, don't needed in this release
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
      
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION);

  // Set CLK0 output
  si5351.set_freq(freq * 100ULL, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  
  // carrier output test
  si5351.output_enable(SI5351_CLK0, 1); // Enable the clock
  delay(5000);
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially
    
  // Initialize the CRC table
  init_crc8();

  // Generate the CRC for the callsign
  callsign_crc = crc8(callsign);

  // We are building a directed message here, but you can do whatever.
  // So apparently, FSQ very specifically needs "  \b  " in
  // directed mode to indicate EOT. A single backspace won't do it.
  sprintf(tx_buffer, "%s:%02x%s", callsign, callsign_crc, "beacon  \b  ");
    
  // Set up Timer1 for interrupts at 6 Hz.
  noInterrupts();          // Turn off interrupts.

  //Init interrupt
  timer1_attachInterrupt(onTimerISR);
  // 6Hz = 166666.6666667 us
  // timer1_write(833333);  //(maximum ticks 8388607)
  // 2Hz = 500000 us
  timer1_write(2500000);   //(maximum ticks 8388607)
  
  interrupts();            // Re-enable interrupts.
  
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  
  // timer dividers
  //  TIM_DIV1 = 0    //80MHz (80 ticks/us - 104857.588 us max)
  //  TIM_DIV16 = 1   //5MHz (5 ticks/us - 1677721.4 us max)
  //  TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)

  delay(1000);
}
 
void loop()
{
    // Beacon a call sign and a locator.
    encode(tx_buffer);
    // Wait 0.5 minutes between beacons.
    delay(30000);
}
