// === COMPILATION "FLAGS" ==========================================================================================================================

#define SKIP_DEVICE
#define DEBUG


// === INLCUDE STATEMENTS ===========================================================================================================================

#include "WiFi.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/uart.h"
#include "esp32/rom/uart.h"

#ifndef SKIP_DEVICE
#include <Adafruit_NeoPixel.h>
#endif


// === DEFINE STATEMENTS ============================================================================================================================

#define BAUD_RATE 115200
#define SYS_DELAY 100

#define LEFT  1
#define RIGHT 2

#define PB_1_PIN 36
#define PB_2_PIN 39
#define PB_3_PIN 32

#define SW_1_PIN 25
#define SW_2_PIN 26

#define JOYSTICK_X_PIN  33
#define JOYSTICK_Y_PIN  32
#define JOYSTICK_SW_PIN 22

#define RGB1_R_PIN 25
#define RGB1_G_PIN 26
#define RGB1_B_PIN 27
#define RGB2_R_PIN 14
#define RGB2_G_PIN 12
#define RGB2_B_PIN 13

#define RGB1_R_CHANNEL 0
#define RGB1_G_CHANNEL 1
#define RGB1_B_CHANNEL 2
#define RGB2_R_CHANNEL 3
#define RGB2_G_CHANNEL 4
#define RGB2_B_CHANNEL 5

#define LED_STRIP_PIN  22
#define LED_NUM_PIXELS 8

#define PWM_FREQUENCY 5000  // 5000Hz frequency
#define PWM_RESOLUTION   8  // 8 bits of resolution (0-255)

#define WIFI_CHANNEL 0      // Wifi channel used for ESP-NOW
#define DEFAULT_ID   1      // Default ID for unpaired sweeper devices
#define MAX_PEERS    3      // Maximum number of allowed peer devices

// Pre-defined Colors
#define RED       0xFF0000
#define YELLOW    0xFFBB00
#define GREEN     0x00FF00
#define TURQUIOSE 0x00FFFF
#define BLUE      0x0000FF
#define PURPLE    0xFF00FF
#define WHITE     0xFFFFFF

#define LOWER_THRESHOLD 1200
#define UPPER_THRESHOLD 2800
#define CLEAR_THRESHOLD 900

// #define LEDC_LS_TIMER          LEDC_TIMER_0
// #define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
// #define LEDC_LS_CH2_GPIO       (19)
// #define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2

// #define LEDC_TEST_CH_NUM       (6)
// #define LEDC_TEST_DUTY         (4000)
// #define LEDC_TEST_FADE_TIME    (3000)


// === STRUCTS & ENUMS ============================================================================

typedef enum state_enum {
  IDLE,
  CLEAR,
  SWEEP_LIGHT,
  SWEEP_HARD
} state_t;

typedef enum message_type_enum {
  PAIRING,
  DATA
} message_t;

typedef struct esp_now_message_packet_struct {
  uint8_t type;
  uint8_t id;
  uint8_t leftState;
  uint8_t rightState;
} data_packet_t;

typedef struct esp_now_paring_packet_struct {
  uint8_t type;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
} pairing_packet_t;


// === GLOBAL VARIABLES ===========================================================================

volatile bool enable = true;
volatile bool paired = false;
float ledBrightness = 0.10;

#ifdef SKIP_DEVICE

bool pbLeft =  false;  // value of 'left'  push button (PB) [digital]
bool pbRight = false;  // value of 'right' push button (PB) [digital]

int jsX =  0;          // value of 'x' joystick (JS) position [analog]
int jsY =  0;          // value of 'y' joystick (JS) position [analog]
int jsSW = 0;          // value of joystick (JS) 'switch'     [analog]

int rgb1Pins[] =     {RGB1_R_PIN,     RGB1_G_PIN,     RGB1_B_PIN    };  // RGB LED 1 PWM pins
int rgb2Pins[] =     {RGB2_R_PIN,     RGB2_G_PIN,     RGB2_B_PIN    };  // RGB LED 2 PWM pins
int rgb1Channels[] = {RGB1_R_CHANNEL, RGB1_G_CHANNEL, RGB1_B_CHANNEL};  // RGB LED 1 PWM channels
int rgb2Channels[] = {RGB2_R_CHANNEL, RGB2_G_CHANNEL, RGB2_B_CHANNEL};  // RGB LED 2 PWM channels

int* leftLED = rgb1Channels;   // Aliase for RGB LED left PWM channels
int* rightLED = rgb2Channels;  // Aliase for RGB LED right PWM channels

#else

bool pbPower =  false;  // value of 'power' push button (PB) [digital]
int swPosition = 0;     // value of 'position' switch (SW) [LEFT/RIGHT]

Adafruit_NeoPixel pixels(LED_NUM_PIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

#endif

data_packet_t    dataPacket;     // ESP-NOW packet for sent/recieved data messages
pairing_packet_t pairingPacket;  // ESP-NOW packet for device pairing messages
esp_now_peer_info_t peerInfo;    // Information about paired/connected peers

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t serverMAC[] =    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t boardMAC[] =     {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t peerMACs[MAX_PEERS][6] = {};
int numPeers = 0;


// === MAIN PROGRAM ===============================================================================

#define DEBOUNCE_DELAY 10
#define LED3_PIN 21

int led1State = 0;
int led2State = 0;

int button1State = 0;
int button2State = 0;
int button3State = 0;

int button1Last = 0;
int button2Last = 0;
int button3Last = 0;

long button1LastDebounce = 0;
long button2LastDebounce = 0;
long button3LastDebounce = 0;

bool inDebouncePeriod1 = false;
bool inDebouncePeriod2 = false;
bool inDebouncePeriod3 = false;

void setup() {

  // Configure Serial Communication
  Serial.begin(BAUD_RATE);

  Serial.println("Starting Program...");
  
  // Configure input GPIOs
  pinMode(PB_1_PIN, INPUT);
  pinMode(PB_2_PIN, INPUT);
  pinMode(JOYSTICK_SW_PIN, INPUT);

  pinMode(LED3_PIN, OUTPUT);

  // Configure PWM channels
  configurePWM(rgb1Pins, rgb1Channels, 3);  // configure PWM for RGB LED 1
  configurePWM(rgb2Pins, rgb2Channels, 3);  // configure PWM for RGB LED 2
}

void loop() {

  // Read Pins
  int button1Current = digitalRead(PB_1_PIN);
  int button2Current = digitalRead(PB_2_PIN);
  int button3Current = digitalRead(JOYSTICK_SW_PIN);

  // Debounce
  int newButton1State = debounce(button1State, button1Current, button1Last, &button1LastDebounce, &inDebouncePeriod1);
  int newButton2State = debounce(button2State, button2Current, button2Last, &button2LastDebounce, &inDebouncePeriod2);
  int newButton3State = debounce(button3State, button3Current, button3Last, &button3LastDebounce, &inDebouncePeriod3);

  // Edge detection
  led1State = edgeDetection(button1State, newButton1State, led1State);
  led2State = edgeDetection(button2State, newButton2State, led2State);

  if (newButton3State) {
    led1State = 0;
    led2State = 0;
  }

  // Set LEDs
  setPwmRGBA(leftLED,  led1State == 1 ? BLUE : WHITE, ledBrightness);
  setPwmRGBA(rightLED, led2State == 1 ? BLUE : WHITE, ledBrightness);

  if ( inDebouncePeriod1 | inDebouncePeriod2 | inDebouncePeriod3 ) {
    digitalWrite(LED3_PIN, HIGH);
  } else {
    digitalWrite(LED3_PIN, LOW);
  }

  // Update last states
  button1Last = button1Current;
  button2Last = button2Current;
  button3Last = button3Current;

  button1State = newButton1State;
  button2State = newButton2State;
  button3State = newButton3State;
}

int debounce(int state, int current, int last, long* lastDebounce, bool* inDebounce) {

  // Check for 'noise'
  if (current != last) {
    // Turn on LED 3
    *inDebounce = true;

    // Reset debounce timer
    *lastDebounce = millis();
  }

  // Debounce timeout
  if ((millis() - *lastDebounce) > DEBOUNCE_DELAY) {
    // Turn off LED 3
    *inDebounce = false;

    // Change button state
    if (current != state) {
      state = current;
    }
  }

  return state;
}

int edgeDetection(int previous, int current, int state) {
  if (current == 1 && previous == 0) {
    state = !state;
  }
  return state;
}


// === GPIO FUNCTIONS ===============================================================================================================================

void configurePWM(int* pin, int* channel, int count) {
  for (int i = 0; i < count; i++) {
    ledcSetup(channel[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin[i], channel[i]);
  }
}

// Set the Red (R) Blue (B) Green (G) and Alpha (A) values for a PWM controlled RGB LED
void setPwmRGBA(int* channels, int rgb, float alpha) {
  ledcWrite(channels[0], (int)(((rgb >> 16) & 0xFF) * alpha));
  ledcWrite(channels[1], (int)(((rgb >> 8 ) & 0xFF) * alpha));
  ledcWrite(channels[2], (int)(((rgb >> 0 ) & 0xFF) * alpha));
}
