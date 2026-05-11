/*
 * Chameleon Audio System Display Interface
 * Target: Adafruit Qualia ESP32-S3 with 4.58" 320x960 bar display
 *
 * Receives SPI messages from Raspberry Pi and displays:
 *   - Source selection (HDMI, RADIO, BT, USB, AUX)
 *   - Audio effects modes (MIDNIGHT, DIALOGUE, POP, ROCK, JAZZ)
 *   - Volume level with visual bar indicator
 *   - ID3 tag information (song/artist)
 *   - FFT audio visualization (pre=white, boost=green, cut=red)
 *
 * SPI message format (1049 bytes):
 *   [source 1B][volume 1B][effects 1B][adj_tag 23B][fft_pre 512B][fft_adjust 512B]
 *
 * Source encoding:  0=HDMI, 1=RADIO, 2=BT, 3=USB, 4=AUX
 * Volume encoding:  0-100 direct
 * Effects encoding: 0=none, 1=POP, 2=ROCK, 3=JAZZ, 4=Dialogue
 *                   11=Midnight+POP, 12=Midnight+ROCK,
 *                   13=Midnight+JAZZ, 14=Midnight+Dialogue
 * FFT adjust:       128=0, 127=-1, 129=+1 (offset encoding)
 *
 * SPI Slave pins (Qualia ESP32-S3 exposed SPI):
 *   SCK  = Arduino pin 5
 *   MISO = Arduino pin 6
 *   MOSI = Arduino pin 7
 *   CS   = Arduino pin 15 (has 10K pull-up)
 *
 * Wire to Raspberry Pi SPI0:
 *   Pi MOSI (GPIO10) -> ESP32 pin 7
 *   Pi MISO (GPIO9)  -> ESP32 pin 6
 *   Pi SCLK (GPIO11) -> ESP32 pin 5
 *   Pi CE0  (GPIO8)  -> ESP32 pin 15
 *   Pi GND           -> ESP32 GND
 *
 * Libraries required (Arduino Library Manager):
 *   - Arduino_GFX_Library by Moon On Our Nation
 *   - Adafruit BusIO
 */

#include <Arduino_GFX_Library.h>
#include <driver/spi_slave.h>

// Fonts
#include "ARIALNB12pt7b.h"
#include "ARIALNB28pt7b.h"
#include "ARIALNB16pt7b.h"

// ====================================================================
// DISPLAY SETUP
// ====================================================================
// Display uses an I2C expander (PCA9554) and RGB panel interface
// Configuration taken from Adafruit Qualia reference example

Arduino_XCA9554SWSPI *expander = new Arduino_XCA9554SWSPI(
    PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI,
    &Wire, 0x3F);  // I2C address 0x3F for the GPIO expander

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,  // Display control signals
    TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,    // Red bits 1-5
    TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,  // Green bits 0-5
    TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,    // Blue bits 1-5
    1, 50, 10, 31,   // hsync: polarity, front_porch, pulse_width, back_porch
    1, 14,  2, 17,    // vsync: polarity, front_porch, pulse_width, back_porch
    1, 16000000       // clock: polarity, pixel clock (16MHz)
);

// 4.58" 320x960 bar display, rotation=1 gives logical 960x320
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    320, 960, rgbpanel, 1 /* rotation 90deg */, true /* auto_flush */,
    expander, GFX_NOT_DEFINED,
    HD458002C40_init_operations, sizeof(HD458002C40_init_operations),
    80 /* col_offset1 */);

// ====================================================================
// COLOR DEFINITIONS (RGB565 format)
// ====================================================================
#define BLACK  0x0000   // All bits off
#define WHITE  0xFFFF   // All bits on
#define RED    0xF800   // Full red, no green/blue
#define GREEN  0x07E0   // Full green, no red/blue

// ====================================================================
// SPI SLAVE CONFIGURATION
// ====================================================================
// ESP32-S3 uses SPI2 for slave mode
// Pins are fixed for the Qualia board's exposed SPI connector
#define SPI_SLAVE_MOSI  7   // Master Out Slave In (from Pi MOSI)
#define SPI_SLAVE_MISO  6   // Master In Slave Out (to Pi MISO)
#define SPI_SLAVE_SCK   5   // Serial Clock (from Pi SCLK)
#define SPI_SLAVE_CS    15  // Chip Select (from Pi CE0)

// Total message size: 1 + 1 + 1 + 23 + 512 + 512 = 1049 bytes
#define SPI_MESSAGE_LENGTH 1049

// DMA buffers must be 32-bit aligned for ESP32
// Using 1052 to allow for 32-bit alignment padding
static uint8_t spi_rx_buf[1052] __attribute__((aligned(4)));  // Receive buffer
static uint8_t spi_tx_buf[1052] __attribute__((aligned(4)));  // Transmit buffer (unused, but required)

// ====================================================================
// APPLICATION STATE VARIABLES
// ====================================================================
// Current state (updated from SPI messages)
static uint8_t  current_source  = 3;           // Default to USB (3)
static uint8_t  current_volume  = 50;          // Default volume 50%
static uint8_t  current_effects = 0;          // No effects initially
static char     current_id3[24] = "Load File first";  // ID3 tag text
static uint8_t  fft_pre[512];                  // Pre-EQ FFT values (0-128)
static int16_t  fft_adjust[512];              // EQ adjustments (-128 to +127)

// Previous state (used to detect changes and avoid redrawing unchanged elements)
// Initialized with impossible values to force first draw
static uint8_t  prev_source  = 255;
static uint8_t  prev_volume  = 255;
static uint8_t  prev_effects = 255;
static char     prev_id3[24] = "";

// ====================================================================
// UI LAYOUT CONSTANTS
// ====================================================================
// Matches the CircuitPython layout for consistency across implementations

// --- Source Selection (left side, vertical list) ---
#define SOURCE_X        25       // X position for source labels
#define SOURCE_Y_START  158      // Y position of first source
#define SOURCE_SPACING  30       // Vertical spacing between sources

// --- ID3 Tag Display (center top) ---
#define ID3_X  150      // X position for ID3 text
#define ID3_Y  60       // Y position for ID3 text

// --- Effects Labels (right side, vertical list) ---
#define EFFECT_X        650      // X position for effect labels
#define EFFECT_Y_START  186      // Y position of first effect
#define EFFECT_SPACING  30       // Vertical spacing between effects

// --- Volume Bar (far right) ---
#define VOL_OUTER_X  850        // Outer rectangle X
#define VOL_OUTER_Y  8          // Outer rectangle Y
#define VOL_OUTER_W  100        // Outer rectangle width
#define VOL_OUTER_H  305        // Outer rectangle height
#define VOL_SEG_X    861        // Volume segment X
#define VOL_SEG_Y    45         // Volume segment Y start
#define VOL_SEG_W    79         // Volume segment width
#define VOL_SEG_H    21         // Volume segment height
#define VOL_FILL_X   862        // Fill area X
#define VOL_FILL_Y   46         // Fill area Y
#define VOL_FILL_W   77         // Fill area width
#define VOL_FILL_H   19         // Fill area height
#define VOL_SPACING  22         // Vertical spacing between segments

// --- FFT Visualization (center bottom) ---
#define FFT_X       123         // FFT X start position
#define FFT_Y_TOP   156         // Top of FFT display area
#define FFT_Y_BASE  284         // Baseline of FFT display
#define FFT_HEIGHT  (FFT_Y_BASE - FFT_Y_TOP)  // Total FFT height (128)
#define FFT_W       512         // FFT width (512 frequency bins)

// ====================================================================
// SPI SLAVE INITIALIZATION
// ====================================================================
void spi_slave_init_hw() {
    // Configure the SPI bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num     = SPI_SLAVE_MOSI;  // Data from master
    bus_cfg.miso_io_num     = SPI_SLAVE_MISO;  // Data to master
    bus_cfg.sclk_io_num     = SPI_SLAVE_SCK;   // Clock from master
    bus_cfg.quadwp_io_num   = -1;              // Not used (no WP pin)
    bus_cfg.quadhd_io_num   = -1;              // Not used (no HD pin)
    bus_cfg.max_transfer_sz = SPI_MESSAGE_LENGTH;  // Maximum transfer size

    // Configure the SPI slave interface
    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.spics_io_num  = SPI_SLAVE_CS;   // Chip select pin
    slave_cfg.flags         = 0;              // No special flags
    slave_cfg.queue_size    = 1;              // Transaction queue size
    slave_cfg.mode          = 0;              // Mode 0: CPOL=0, CPHA=0 (matches Pi default)
    slave_cfg.post_setup_cb = NULL;           // No post-setup callback
    slave_cfg.post_trans_cb = NULL;           // No post-transaction callback

    // Initialize SPI slave on SPI2_HOST
    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("SPI slave init failed: %d\n", ret);
    } else {
        Serial.println("SPI slave ready");
    }
}

// ====================================================================
// SPI FRAME RECEPTION
// ====================================================================
// Attempts to receive one SPI frame with a 50ms timeout
// Returns true if a complete frame was received
bool spi_receive_frame() {
    // Clear transmit buffer (not used, but required by API)
    memset(spi_tx_buf, 0, SPI_MESSAGE_LENGTH);

    // Set up the transaction
    spi_slave_transaction_t t = {};
    t.length    = SPI_MESSAGE_LENGTH * 8;  // Length in bits
    t.tx_buffer = spi_tx_buf;               // Transmit buffer
    t.rx_buffer = spi_rx_buf;               // Receive buffer

    // Perform the transaction with 50ms timeout
    // If no data arrives within 50ms, we still update the display with last known state
    esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return false;
    
    // Check if we received the full message
    if (t.trans_len < (uint32_t)(SPI_MESSAGE_LENGTH * 8)) return false;
    
    return true;
}

// ====================================================================
// FRAME PARSING
// ====================================================================
// Extracts data from received SPI buffer into application state variables
void parse_frame() {
    // Byte 0: Source (1 byte)
    current_source = spi_rx_buf[0];
    if (current_source > 4) current_source = 3;  // Clamp to valid range (0-4), default to USB

    // Byte 1: Volume (1 byte)
    current_volume = spi_rx_buf[1];
    if (current_volume > 100) current_volume = 100;  // Clamp to valid range (0-100)

    // Byte 2: Effects mode (1 byte)
    current_effects = spi_rx_buf[2];

    // Bytes 3-25: ID3 tag text (23 bytes, null-terminated string)
    memcpy(current_id3, &spi_rx_buf[3], 23);
    current_id3[23] = '\0';  // Ensure null-termination

    // Bytes 26-537: Pre-EQ FFT values (512 bytes)
    // Values are 0-128 representing FFT magnitude
    for (int i = 0; i < 512; i++) {
        fft_pre[i] = spi_rx_buf[26 + i];
        if (fft_pre[i] > 128) fft_pre[i] = 128;  // Clamp to valid range
    }

    // Bytes 538-1049: FFT adjustments (512 bytes)
    // Offset-encoded: 128 = 0, 127 = -1, 129 = +1, etc.
    for (int i = 0; i < 512; i++) {
        fft_adjust[i] = (int16_t)spi_rx_buf[538 + i] - 128;  // Convert from offset to signed value
    }
}

// ====================================================================
// SOURCE DISPLAY
// ====================================================================
// Source names in order of encoding (0=HDMI, 1=RADIO, 2=BT, 3=USB, 4=AUX)
const char* SOURCE_NAMES[] = {"HDMI", "RADIO", "BT", "USB", "AUX"};

// Draw all source labels, highlighting the current source in red
void draw_sources() {
    gfx->setFont(&ARIALNB16pt7b);
    for (int i = 0; i < 5; i++) {
        // Current source is highlighted in red, others in white
        uint16_t colour = (i == current_source) ? RED : WHITE;
        gfx->setTextColor(colour, BLACK);
        gfx->setCursor(SOURCE_X, SOURCE_Y_START + i * SOURCE_SPACING);
        gfx->print(SOURCE_NAMES[i]);
    }
    prev_source = current_source;
}

// Only redraw sources if the source has changed
void update_sources() {
    if (current_source == prev_source) return;
    draw_sources();
}

// ====================================================================
// ID3 TAG DISPLAY
// ====================================================================
// Draw the current ID3 tag (song title/artist) in large font
void draw_id3() {
    gfx->setFont(&ARIALNB28pt7b);
    // Erase old text by filling a rectangle over the previous text area
    gfx->fillRect(ID3_X, ID3_Y - 40, 650, 90, BLACK);
    // Draw new text
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(ID3_X, ID3_Y+20);
    gfx->print(current_id3);
    memcpy(prev_id3, current_id3, 24);
}

// Only redraw ID3 if the text has changed
void update_id3() {
    if (strcmp(current_id3, prev_id3) == 0) return;
    draw_id3();
}

// ====================================================================
// EFFECTS DISPLAY
// ====================================================================
// Structure to track effect label visibility and position
struct EffectLabel {
    const char* name;   // Effect name to display
    int x, y;           // Screen position
    bool visible;       // Whether currently visible
};

// Effect labels with their positions
// Note: POP, ROCK, JAZZ share the same Y position (only one shown at a time)
EffectLabel effect_labels[] = {
    {"MIDNIGHT", EFFECT_X, EFFECT_Y_START, false},
    {"DIALOGUE", EFFECT_X, EFFECT_Y_START + 1 * EFFECT_SPACING, false},
    {"POP",      EFFECT_X, EFFECT_Y_START + 2 * EFFECT_SPACING, false},
    {"ROCK",     EFFECT_X, EFFECT_Y_START + 2 * EFFECT_SPACING, false},
    {"JAZZ",     EFFECT_X, EFFECT_Y_START + 2 * EFFECT_SPACING, false},
};

// Show or hide an effect label
void set_effect(int idx, bool show) {
    if (effect_labels[idx].visible == show) return;  // No change needed
    gfx->setFont(&ARIALNB16pt7b);
    // White text on black background when showing, black text (erases) when hiding
    gfx->setTextColor(show ? WHITE : BLACK, BLACK);
    gfx->setCursor(effect_labels[idx].x, effect_labels[idx].y);
    gfx->print(effect_labels[idx].name);
    effect_labels[idx].visible = show;
}

// Decode effects encoding and update display
void update_effects() {
    if (current_effects == prev_effects) return;  // No change

    // Initialize all effect flags to false
    bool midnight = false, dialogue = false;
    bool pop = false, rock = false, jazz = false;

    // Decode the effects byte into individual flags
    // Values 1-4 are single effects, 10-14 combine Midnight with another effect
    switch (current_effects) {
        case 1:  pop      = true; break;      // POP only
        case 2:  rock     = true; break;      // ROCK only
        case 3:  jazz     = true; break;      // JAZZ only
        case 4:  dialogue = true; break;      // DIALOGUE only
        case 10: midnight = true; break;      // MIDNIGHT only
        case 11: midnight = true; pop      = true; break;  // MIDNIGHT + POP
        case 12: midnight = true; rock     = true; break;  // MIDNIGHT + ROCK
        case 13: midnight = true; jazz     = true; break;  // MIDNIGHT + JAZZ
        case 14: midnight = true; dialogue = true; break;  // MIDNIGHT + DIALOGUE
    }

    // Update each effect label visibility
    set_effect(0, midnight);   // Index 0 = MIDNIGHT
    set_effect(1, dialogue);   // Index 1 = DIALOGUE
    set_effect(2, pop);        // Index 2 = POP
    set_effect(3, rock);       // Index 3 = ROCK
    set_effect(4, jazz);       // Index 4 = JAZZ
    
    prev_effects = current_effects;
}

// ====================================================================
// VOLUME BAR DISPLAY
// ====================================================================
// Draw the static volume bar frame (outer rectangle and segment outlines)
void draw_volume_frame() {
    // Draw outer rectangle
    gfx->drawRect(VOL_OUTER_X, VOL_OUTER_Y, VOL_OUTER_W, VOL_OUTER_H, WHITE);
    // Fill inside with black
    gfx->fillRect(VOL_OUTER_X + 1, VOL_OUTER_Y + 1, VOL_OUTER_W - 2, VOL_OUTER_H - 2, BLACK);
    
    // Draw "Volume" label at top
    gfx->setFont(&ARIALNB12pt7b);
    gfx->setTextColor(WHITE, BLACK);
    gfx->setCursor(VOL_OUTER_X + 10, VOL_OUTER_Y + 15);
    gfx->print("Volume");
    
    // Draw 12 segment outlines (empty rectangles)
    for (int i = 0; i < 12; i++) {
        gfx->drawRect(VOL_SEG_X, VOL_SEG_Y + i * VOL_SPACING, VOL_SEG_W, VOL_SEG_H, WHITE);
    }
}

// Update the filled portion of volume bar based on current volume
void update_volume() {
    if (current_volume == prev_volume) return;  // No change
    
    // Calculate how many segments to fill (12 total)
    // Volume 0-100 maps to 0-12 segments
    // At volume 100, v_adj = 0, so all 12 segments filled
    // At volume 0, v_adj = 12, so no segments filled
    int v_adj = 12 - current_volume * 12 / 100;
    
    // Fill segments from bottom up (or unfill from top down)
    for (int i = 0; i < 12; i++) {
        // Segments below v_adj are empty (black), above are filled (white)
        uint16_t colour = (i < v_adj) ? BLACK : WHITE;
        gfx->fillRect(VOL_FILL_X, VOL_FILL_Y + i * VOL_SPACING, VOL_FILL_W, VOL_FILL_H, colour);
    }
    prev_volume = current_volume;
}

// ====================================================================
// FFT VISUALIZATION
// ====================================================================
// Draw the FFT visualization - redrawn every frame for animation
// Visualization shows:
//   - White: Base FFT magnitude (pre-EQ)
//   - Green: Boosted frequencies (post-EQ > pre-EQ)
//   - Red: Cut frequencies (pre-EQ > post-EQ)
//   - Black: No signal
void draw_fft() {
    for (int i = 0; i < FFT_W; i++) {
        int pre_val  = fft_pre[i];       // Pre-EQ value (0-128)
        int post_val = pre_val + fft_adjust[i];  // Post-EQ value (0-255)
        
        // Clamp post_val to valid range
        if (post_val < 0)   post_val = 0;
        if (post_val > 255) post_val = 255;
        
        int base = min(pre_val, post_val);  // Base level (minimum of pre and post)
        int x    = FFT_X + i;               // X position for this frequency bin

        // Work from top of FFT area downward, painting each zone
        
        // Zone 1: Black from top down to where signal starts
        // This erases the area above the highest signal
        int top_black = FFT_HEIGHT - max(pre_val, post_val);
        if (top_black > 0)
            gfx->drawFastVLine(x, FFT_Y_TOP, top_black, BLACK);

        // Zone 2: Green or red difference above base
        // Shows the EQ adjustment: green for boost, red for cut
        if (post_val > pre_val)
            // Post-EQ is higher than pre-EQ: show boost in green
            gfx->drawFastVLine(x, FFT_Y_TOP + top_black, post_val - pre_val, GREEN);
        else if (pre_val > post_val)
            // Pre-EQ is higher than post-EQ: show cut in red
            gfx->drawFastVLine(x, FFT_Y_TOP + top_black, pre_val - post_val, RED);

        // Zone 3: White base common to both
        // Shows the base FFT magnitude that exists in both pre and post
        if (base > 0)
            gfx->drawFastVLine(x, FFT_Y_BASE - base, base, WHITE);
    }
}

// ====================================================================
// SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Chameleon Display starting...");

    // Configure I2C clock for expander
    Wire.setClock(1000000);
    
    // Initialize display
    if (!gfx->begin()) {
        Serial.println("Display init failed!");
        while (1) delay(100);  // Halt on failure
    }
    Serial.println("Display OK");

    // Turn on backlight via expander
    expander->pinMode(PCA_TFT_BACKLIGHT, OUTPUT);
    expander->digitalWrite(PCA_TFT_BACKLIGHT, HIGH);

    // Clear screen to black
    gfx->fillScreen(BLACK);

    // Draw static elements (frame that doesn't change)
    draw_volume_frame();

    // Draw dynamic elements (prev values are initialized to force full redraw)
    draw_sources();
    draw_id3();
    update_volume();

    // Initialize SPI slave interface
    spi_slave_init_hw();

    Serial.println("Ready");
}

// ====================================================================
// MAIN LOOP
// ====================================================================
void loop() {
    // Try to receive an SPI frame (timeout after 50ms if none available)
    if (spi_receive_frame()) {
        // Parse the received frame into state variables
        parse_frame();
    }

    // Update all display elements
    update_sources();   // Update source selection highlight
    update_id3();       // Update song/artist info
    update_effects();   // Update effects labels
    update_volume();    // Update volume bar fill
    draw_fft();         // Redraw FFT visualization (always redrawn for animation)
}
