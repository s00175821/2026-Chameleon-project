# Audio System Display Interface for CircuitPython
# This program creates a visual interface for an audio system, displaying:
# - Source selection (HDMI, RADIO, BT, USB, AUX)
# - Audio effects modes (MIDNIGHT, DIALOGUE, POP)
# - Volume level with visual bar indicator
# - ID3 tag information (song/artist) with scrolling text
# - FFT audio visualization

import traceback
from displayio import release_displays
import time
import displayio
import busio
import board
import dotclockframebuffer
import wifi
import random
from framebufferio import FramebufferDisplay
from adafruit_bitmap_font import bitmap_font
from adafruit_display_text.label import Label
from adafruit_display_shapes.rect import Rect
import bitmaptools

# ------------------------------------------------------------------
# Initial Setup - Clear existing displays and disable WiFi to free resources
# ------------------------------------------------------------------
release_displays()  # Release any existing display objects
wifi.radio.enabled = False  # Disable WiFi radio to free up memory

# ------------------------------------------------------------------
# Constants - Display configuration and font loading
# Change these values to modify screen behavior
# ------------------------------------------------------------------
SCREEN_WIDTH = 960     # Display width in pixels
SCREEN_HEIGHT = 320    # Display height in pixels
TFT_FREQUENCY = 16000000  # TFT display clock frequency (16 MHz)
# Load bitmap fonts from files for UI text rendering
font1 = bitmap_font.load_font("/fonts/Robo-24.bdf")  # 24pt font for source/effect labels
font2 = bitmap_font.load_font("/fonts/Robo-40.bdf")  # 40pt font for ID3 tag display
font3 = bitmap_font.load_font("/fonts/Robo-20.bdf")  # 20pt font for volume label

# ------------------------------------------------------------------
# Display Initialization Sequence
# Binary commands sent to the TFT display during hardware initialization
# DO NOT MODIFY - These are low-level display configuration commands
# ------------------------------------------------------------------
INIT_SEQUENCE = bytes(
    (
        b"\xff\x05w\x01\x00\x00\x13"
        b"\xef\x01\x08"
        b"\xff\x05w\x01\x00\x00\x10"
        b"\xc0\x02w\x00"
        b"\xc1\x02\t\x08"
        b"\xc2\x02\x01\x02"
        b"\xc3\x01\x02"
        b"\xcc\x01\x10"
        b"\xb0\x10@\x14Y\x10\x12\x08\x03\t\x05\x1e\x05\x14\x10h3\x15"
        b"\xb1\x10@\x08S\t\x11\t\x02\x07\t\x1a\x04\x12\x12d))"
        b"\xff\x05w\x01\x00\x00\x11"
        b"\xb0\x01m"
        b"\xb1\x01\x1d"
        b"\xb2\x01\x87"
        b"\xb3\x01\x80"
        b"\xb5\x01I"
        b"\xb7\x01\x85"
        b"\xb8\x01 "
        b"\xc1\x01x"
        b"\xc2\x01x"
        b"\xd0\x01\x88"
        b"\xe0\x03\x00\x00\x02"
        b"\xe1\x0b\x02\x8c\x00\x00\x03\x8c\x00\x00\x0033"
        b"\xe2\r3333\xc9<\x00\x00\xca<\x00\x00\x00"
        b"\xe3\x04\x00\x0033"
        b"\xe4\x02DD"
        b"\xe5\x10\x05\xcd\x82\x82\x01\xc9\x82\x82\x07\xcf\x82\x82\x03\xcb\x82\x82"
        b"\xe6\x04\x00\x0033"
        b"\xe7\x02DD"
        b"\xe8\x10\x06\xce\x82\x82\x02\xca\x82\x82\x08\xd0\x82\x82\x04\xcc\x82\x82"
        b"\xeb\x07\x08\x01\xe4\xe4\x88\x00@"
        b"\xec\x03\x00\x00\x00"
        b"\xed\x10\xff\xf0\x07eO\xfc\xc2/\xf2,\xcf\xf4Vp\x0f\xff"
        b"\xef\x06\x10\r\x04\x08?\x1f"
        b"\xff\x05w\x01\x00\x00\x00"
        b"\x11\x80x"
        b"5\x01\x00"
        b":\x81fd"
        b")\x00"
    )
)


# ------------------------------------------------------------------

def screen_init():
    """
    Initialize the TFT display hardware and create display objects.
    
    Configures I2C communication, sends initialization sequence to the display,
    creates framebuffer with timing parameters, and sets up the display group.
    
    Returns:
        tuple: (display, bitmap, group) - The main display objects for rendering
    """
    # Send init sequence via I2C expander
    board.I2C().deinit()
    i2c = busio.I2C(board.SCL, board.SDA)
    tft_io_expander = dict(board.TFT_IO_EXPANDER)
    # Uncomment next line for rev B hardware:
    # tft_io_expander['i2c_address'] = 0x38
    dotclockframebuffer.ioexpander_send_init_sequence(
        i2c, INIT_SEQUENCE, **tft_io_expander
    )
    i2c.deinit()

    # Create framebuffer and display
    tft_pins = dict(board.TFT_PINS)
    tft_timings = {
        "frequency": TFT_FREQUENCY,
        "width": 320,
        "height": 960,
        "overscan_left": 80,
        "hsync_pulse_width": 10,
        "hsync_front_porch": 50,
        "hsync_back_porch": 30,
        "hsync_idle_low": False,
        "vsync_pulse_width": 2,
        "vsync_front_porch": 17,
        "vsync_back_porch": 15,
        "vsync_idle_low": False,
        "pclk_active_high": False,
        "pclk_idle_high": False,
        "de_idle_high": False,
    }
    fb = dotclockframebuffer.DotClockFramebuffer(**tft_pins, **tft_timings)
    display = FramebufferDisplay(fb, auto_refresh=False)

    # Bitmap covers the full screen
    bitmap = displayio.Bitmap(display.height, display.width , 65535)
    # Wire bitmap -> TileGrid -> Group -> Display
    pixel_shader = displayio.ColorConverter(
        input_colorspace=displayio.Colorspace.RGB565
    )
    tile_grid = displayio.TileGrid(bitmap, pixel_shader=pixel_shader)
    group = displayio.Group()
    group.append(tile_grid)
    display.root_group = group
    display.auto_refresh = True
    display.rotation = 90
    return display, bitmap, group


# ------------------------------------------------------------------
# UI Element Functions - Create and manage on-screen elements
# ------------------------------------------------------------------

def fill_screen(bitmap, colour):
    """Fill the entire screen bitmap with a single color.
    
    Args:
        bitmap: The display bitmap to fill
        colour: Color value in RGB565 format (e.g., 0x0000 = black, 0xFFFF = white)
    """
    for y in range(320):
        for x in range(960):
            bitmap[x, y] = colour

def draw_sources(group, selected="USB"):
    """Create source selection labels on the display.
    
    Args:
        group: The displayio Group to add labels to
        selected: The currently selected source (highlighted in red)
    
    Returns:
        list: List of Label objects for later updates
    """
    sources = ["HDMI", "RADIO", "BT", "USB", "AUX"]
    labels = []
    for i, src in enumerate(sources):
        colour = 0xFF0000 if src == selected else 0xFFFFFF
        my_label = Label(
            font1,
            anchor_point=(0, 0),
            anchored_position=(25, 158 + i * 30),
            text=src,
            color=colour,
        )
        group.append(my_label)
        labels.append(my_label)
    return labels

def draw_effects(group):
    """Create audio effect mode labels on the display.
    
    Args:
        group: The displayio Group to add labels to
    
    Returns:
        list: List of Label objects (MIDNIGHT, DIALOGUE, POP)
    """
    labels = []
    colour = 0xFFFFFF  # White color for effect labels
    my_label = Label(
        font1,
        anchor_point=(0, 0),
        anchored_position=(650, 186),
        text="MIDNIGHT",
        color=colour,
    )
    group.append(my_label)
    labels.append(my_label)
    my_label2 = Label(
        font1,
        anchor_point=(0, 0),
        anchored_position=(650, 216),
        text="DIALOGUE",
        color=colour,
    )
    group.append(my_label2)
    labels.append(my_label2)
    my_label3 = Label(
        font1,
        anchor_point=(0, 0),
        anchored_position=(650, 246),
        text="POP",
        color=colour,
    )
    group.append(my_label3)
    labels.append(my_label3)
    return labels

def draw_id3_tag(id_lab, id3_text, scroll_pos, delay_help, adj_tag):
    """Update the ID3 tag label with scrolling text if needed.
    
    Handles long ID3 tags by truncating to 20 characters and scrolling
    the text from left to right with controlled speed.
    
    Args:
        id_lab: The Label object to update
        id3_text: Full ID3 tag text (artist - song title)
        scroll_pos: Current scroll position
        delay_help: Counter for scroll timing control
        adj_tag: Currently displayed truncated text
    
    Returns:
        tuple: (scroll_pos, delay_help, adj_tag) - Updated state values
    """
    over_length = len(id3_text)-20
    if over_length > 0 and delay_help == 0:
        adj_tag = id3_text[0+scroll_pos:20+scroll_pos]
        delay_help = 5
        scroll_pos = (scroll_pos + 1) % (over_length+1)
        if scroll_pos == 0:
            delay_help = 12
    elif over_length <= 0:
        adj_tag = id3_text
    delay_help = max(delay_help - 1, 0)
    font2.load_glyphs(adj_tag.encode("utf-8"))
    id_lab.text = adj_tag
    return scroll_pos, delay_help, adj_tag

def draw_volume(group):
    """Create the volume bar UI element on the display.
    
    Draws a vertical volume bar with 12 segments and a "Volume" label.
    
    Args:
        group: The displayio Group to add the volume bar to
    
    Returns:
        list: List of Rect objects representing individual volume segments
    """
    volbar_rect = []  # List to store volume bar segment rectangles
    rect1 = Rect(850, 8, 100, 305, fill=0xFFFFFF)  # Outer white border
    rect2 = Rect(851, 9, 98, 303, fill=0x000000)
    group.append(rect1)
    group.append(rect2)
    group.append(Label(
        font3,
        anchor_point=(0.5,0.5),
        anchored_position=(900,25),
        text="Volume",
        color=0xFFFFFF,))
    for i in range (12):
        rect = Rect(861, 45 + i * 22, 79, 21, fill=0xFFFFFF)
        group.append(rect)
        rect_b = Rect(862, 46 + i * 22, 77, 19, fill=0xFFFFFF)
        group.append(rect_b)
        volbar_rect.append(rect_b)
    return volbar_rect



def draw_fft(bitmap):
    """Draw FFT audio visualization bars on the bitmap.
    
    Clears the FFT area and draws vertical bars representing audio frequency
    data. Uses random data for demonstration purposes.
    
    Args:
        bitmap: The display bitmap to draw on
    """
    fft_bin = [random.randint(10, 128) for _ in range(512)]  # Generate random FFT data (512 bins)
    bitmaptools.fill_region(bitmap, 123, 156, 635, 285, 0x0000)  # Clear FFT area to black
    for i, val in enumerate(fft_bin):
        if val > 0:
            bitmaptools.fill_region(bitmap, 123+i, 284-val, 124+i, 284, 0xFFFF)

def update_volume(vol_graph, volume):
    """Update the volume bar display based on current volume level.
    
    Args:
        vol_graph: List of volume segment Rect objects from draw_volume()
        volume: Current volume level (0-100)
    """
    v_adj=12-volume*12//100  # Calculate how many segments should be off (black)
    for i in range (0, v_adj):
        vol_graph[i].fill = 0x000000
    for i in range (v_adj, 12):
        vol_graph[i].fill = 0xFFFFFF

def update_source(source_labels, source):
    """Update source selection labels to highlight the active source.
    
    Args:
        source_labels: List of Label objects from draw_sources()
        source: Index of the currently selected source (0=HDMI, 1=RADIO, 2=BT, 3=USB, 4=AUX)
    """
    for i in range(5):
        if i == source:
            source_labels[i].color = 0xFF0000
        else:
            source_labels[i].color = 0xFFFFFF
    
# ------------------------------------------------------------------
# Main Application Entry Point
# ------------------------------------------------------------------

def main():
    """
    Main application loop for the audio system display interface.
    
    Initializes the display, creates all UI elements, then enters an infinite
    loop that:
    - Updates the ID3 tag text based on the current audio source
    - Animates volume level between 0-100%
    - Randomly cycles through audio sources
    - Updates the FFT visualization
    - Handles scrolling for long ID3 text
    - Refreshes the display at ~30 FPS
    """
    # ----------------------------------------------------------------
    # Local Variables - Track application state
    # ----------------------------------------------------------------
    id3_text = "Artist Name - Song Title - ID3 tag"  # Current ID3 tag text
    scroll_pos = 0  # Current scroll position for long ID3 text
    delay_help = 0  # Counter for scroll timing control
    adj_tag = ""  # Currently displayed truncated ID3 text
    volume = 100  # Current volume level (0-100)
    vol_direction = 0  # 0 = volume decreasing, 1 = volume increasing
    source = 3  # Current audio source index (3 = USB)

    # Initialize display and get display objects
    display, bitmap, group = screen_init()
    display.auto_refresh = False  # Disable auto-refresh for manual control
    counter = 0  # Frame counter for timing operations
    fill_screen(bitmap, 0x0000)  # Clear screen to black

    # Create all UI elements
    source_labels = draw_sources(group, "USB")  # Source selection labels (left side)
    ID3_label = Label(
        font2,
        anchor_point=(0, 0),
        anchored_position=(150, 60),
        text=adj_tag,
        color=0xFFFFFF,  # White text
        )
    group.append(ID3_label)  # Add ID3 tag label to display group
    effect_labels = draw_effects(group)  # Effect mode labels (right side)
    vol_graph = draw_volume(group)  # Volume bar with segments

    # ------------------------------------------------------------------
    # Main Loop - Runs indefinitely, updating display each frame
    # ------------------------------------------------------------------
    while True:
        display.refresh()  # Refresh the display to show latest changes
        current_id3 = id3_text  # Save previous ID3 text for comparison

        # Update ID3 text based on current audio source
        if source == 0:
            id3_text = "HDMI Mode - 5.1 output"
        elif source == 1:
            id3_text = "Classic Hits Radio"
        elif source == 2:
            id3_text = "Artist Name - Song Title - ID3 tag"
        elif source == 3:
            id3_text = "Artist Name - Song Title - ID3 tag"
        else:
            id3_text = "AUX mode"

        # Animate volume level: auto-cycle between 0% and 100%
        if vol_direction == 0:  # Volume decreasing
            if volume == 0:
                vol_direction = 1  # Switch to increasing at 0%
            volume = volume - 1
        if vol_direction == 1:  # Volume increasing
            if volume == 100:
                vol_direction = 0  # Switch to decreasing at 100%
            volume = volume + 1
        update_volume(vol_graph, volume)  # Update visual volume bar
        # scrolling_label.update()

        # Reset scrolling state when ID3 text changes
        if current_id3 != id3_text:
            adj_tag = ""
            scroll_pos = 0
            delay_help = 0

        # Update ID3 tag display with scrolling if needed
        scroll_pos, delay_help, adj_tag = draw_id3_tag(ID3_label, id3_text, scroll_pos, delay_help, adj_tag)

        # Every 20 frames (~0.66 seconds), randomly change audio source
        if counter % 20 == 19:
            source = random.randint(0, 4)  # Random source index 0-4
            update_source(source_labels, source)  # Update source label highlighting
        
        time.sleep(0.033)  # ~30 FPS (1000ms / 30 = ~33.3ms per frame)
        counter = counter + 1
        draw_fft(bitmap)  # Update FFT visualization each frame


# ------------------------------------------------------------------
# Application Entry Point with Error Handling
# ------------------------------------------------------------------

try:
    main()  # Start the main application
    
except Exception as e:
    # If any error occurs, log it to /error_log.txt for debugging
    with open("/error_log.txt", "w") as f:
        f.write(str(e) + "\n")  # Write error message
        traceback.print_exception(type(e), e, e.__traceback__, file=f)  # Write full traceback
