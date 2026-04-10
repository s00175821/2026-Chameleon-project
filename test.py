import soundfile as sf
import pygame
import random
import os
import tkinter as tk
import tkinter.filedialog as fd
import sys
import pyaudio
import numpy as np
import wave
import threading
from scipy import signal as sp_signal

os.environ['SDL_AUDIODRIVER'] = 'alsa'
os.environ['PYGAME_SDL_AUDIODRIVER'] = 'alsa'
os.environ['AUDIODEV'] = 'hw:0,0'

pygame.init() #initializing pygame library
screen = pygame.display.set_mode((1024, 600))  # screen initialization
# Defining fonts
font = pygame.font.SysFont('liberationsansnarrow', 24, bold=True) 
font2 = pygame.font.SysFont('takaogothic', 40)
font3 = pygame.font.SysFont('liberationsansnarrow', 18, bold=True)
font4 = pygame.font.SysFont('liberationsansnarrow', 20, bold=True)

# offsetting screen simulation
screen_top_corner = 32
scroll_pos = 0 # Help with scrolling id3 tag if it is to long
delay_help = 0 # Help with scroll speed
st_mid = "Off" # midnight mode off by default
st_dial = "Off" # dialogue enhancement off by default
st_dsp = "Off" # DSP effects off by default
st_volume = 50 # Default volume is 50%
adj_tag = "" # control of id3 tag display size
id3_text = "Load File first" # id3_tag holder
dsp_effect = "POP" # current DSP effect applied when DSP is on

# Global variables for audio playback
_pa_instance = None # single Pyaudio instance, created once
current_stream = None
current_wf = None # wave.Wave_read object
current_path = None # file path string
is_playing = False
_playback_lock = threading.Lock()

# global variables for fft
fft_pre  = [0] * 512   # magnitudes before processing
fft_post = [0] * 512   # magnitudes after processing
fft_buffer_pre  = np.zeros((1024, 2), dtype='float32')
fft_buffer_post = np.zeros((1024, 2), dtype='float32')
fft_buffer_lock = threading.Lock()
CHUNK= 2048 # larger buffer -> fewer undercuts on RPi

#Equalizer globals
BAND_CENTRES = [55, 77, 110, 156, 220, 311, 440, 622, 880,
                1200, 1800, 2500, 3500, 5000, 7000, 10000, 14000, 20000]
BAND_Q = 1.4  # Q factor for all bands - reasonable width for 18 bands

current_samplerate = 44100  # updated on file load
_eq_sos  = None  # computed SOS matrix, shape (18, 6)
_eq_zi   = None  # filter state, shape (18, 2, 2) - 18 bands, stereo, 2 state vars
_eq_gains_db = [0.0] * 18  # flat by default

EQ_FLAT = [0.0] * 18

EQ_DIALOGUE = [ -2.0, -2.0, -3.0, -2.0,  0.0,  0.0,  1.0,  2.0,  3.0,
                 4.0,  4.0,  3.0,  3.0,  2.0,  0.0, -1.0, -2.0, -3.0]

EQ_POP  = [  6.0,  5.0,  3.0,  0.0,  -2.0,  -4.0,  -4.0,  -6.0,  -3.0,
             1.0,  0.0,  0.0,  2.0,  1.0,  2.0,  4.0,  5.0, 6.0]

EQ_ROCK = [  4.0,  5.0,  5.0,  5.0,  4.0,  3.0, 1.0, 0.0,  -1.0,
             -2.0,  -2.0,  0.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0]

EQ_JAZZ = [  0.0,  1.0,  2.0,  2.0,  3.0,  1.0,  2.0,  0.0,  0.0,
             2.0,  1.0,  2.0,  4.0,  3.0,  3.0,  2.0,  1.0,  0.0]



class FlacWrapper:
    def __init__(self, data, samplerate):
        # if mono, make it 2D so channel logic is consistent
        if data.ndim == 1:
            data = data.reshape(-1, 1)
        self.data       = data
        self.samplerate = samplerate
        self.pos        = 0          # current read position in frames

    def getsampwidth(self):  return 4                    # int32 = 4 bytes
    def getnchannels(self):  return self.data.shape[1]
    def getframerate(self):  return self.samplerate
    
    def readframes(self, n):
        chunk = self.data[self.pos : self.pos + n]
        self.pos += n
        return chunk.tobytes()
    
    def rewind(self):        self.pos = 0
    def close(self):         pass                        # nothing to close

# Midnight mode state - persists between chunks for smooth attack/release
_mm_smooth_gain = 1.0

def apply_midnight_mode(chunk):
    global _mm_smooth_gain
    
    # Constants matching VHDL spec
    THRESH_HIGH = 10 ** (-18 / 20)   # -18 dBFS = 0.1259
    THRESH_LOW  = 10 ** (-45 / 20)   # -45 dBFS = 0.00562
    PEAK_CEIL   = 10 ** (-3  / 20)   # -3 dBFS  = 0.7079
    MAKEUP      = 10 ** (6   / 20)   # +6 dB    = 1.9953
    RATIO_HIGH  = 4.0
    RATIO_LOW   = 2.0
    
    # Attack/release coefficients matching VHDL (per chunk, not per sample)
    # VHDL uses per-sample at 96kHz; we approximate per-chunk at ~44100Hz
    SR = 44100
    ATTACK_TC  = np.exp(-1.0 / (0.010 * SR / len(chunk)))   # 10ms
    RELEASE_TC = np.exp(-1.0 / (0.200 * SR / len(chunk)))   # 200ms
    
    # Compute RMS level of this chunk (mono mix)
    mono = chunk.mean(axis=1) if chunk.ndim == 2 else chunk
    rms = np.sqrt(np.mean(mono ** 2))
    rms = max(rms, 1e-9)   # avoid log of zero
    
    # Determine target gain from compression curve
    if rms > THRESH_HIGH:
        # Downward compression 4:1 above -18dBFS
        excess = rms / THRESH_HIGH
        target_gain = THRESH_HIGH * (excess ** (1.0 / RATIO_HIGH)) / rms
    elif rms < THRESH_LOW:
        # Upward expansion 2:1 below -45dBFS
        deficit = rms / THRESH_LOW
        target_gain = THRESH_LOW * (deficit ** RATIO_LOW) / rms
    else:
        target_gain = 1.0
    
    target_gain *= MAKEUP
    
    # Smooth the gain with attack/release
    if target_gain < _mm_smooth_gain:
        _mm_smooth_gain = ATTACK_TC  * _mm_smooth_gain + (1 - ATTACK_TC)  * target_gain
    else:
        _mm_smooth_gain = RELEASE_TC * _mm_smooth_gain + (1 - RELEASE_TC) * target_gain
    
    # Apply gain
    out = chunk * _mm_smooth_gain
    
    # Hard limiter at -3dBFS
    out = np.clip(out, -PEAK_CEIL, PEAK_CEIL)
    
    return out.astype(np.float32)

def build_eq_sos(gains_db, samplerate):
    """
    Build a (18, 6) SOS matrix from 18 dB gain values.
    Each band is a peaking EQ biquad (bell filter).
    Returns sos matrix ready for sosfilt.
    """
    sections = []
    for i, (fc, gain_db) in enumerate(zip(BAND_CENTRES, gains_db)):
        if fc >= samplerate / 2:
            # Band above Nyquist - pass through (unity biquad)
            sections.append([1.0, 0.0, 0.0, 1.0, 0.0, 0.0])
            continue
        A  = 10 ** (gain_db / 40.0)   # amplitude, sqrt of power gain
        w0 = 2 * np.pi * fc / samplerate # normalised angular frequency
        alpha = np.sin(w0) / (2 * BAND_Q) # controls the width of the bell
        
        b0 =  1 + alpha * A
        b1 = -2 * np.cos(w0)
        b2 =  1 - alpha * A
        a0 =  1 + alpha / A
        a1 = -2 * np.cos(w0)
        a2 =  1 - alpha / A
        
        sections.append([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])
    
    return np.array(sections)

def reset_eq(gains_db, samplerate):
    """Recompute SOS and reset state. Call on file load or preset change."""
    global _eq_sos, _eq_zi, _eq_gains_db
    _eq_gains_db = gains_db
    _eq_sos  = build_eq_sos(gains_db, samplerate)
    # zi shape: (n_sections, 2, n_channels) - scipy convention
    _eq_zi   = np.zeros((len(BAND_CENTRES), 2, 2))
    
def apply_eq(chunk):
    """
    Apply 18-band EQ to a float32 stereo chunk.
    chunk shape: (n_frames, 2)
    Processes each channel independently, maintaining state via _eq_zi.
    """
    global _eq_zi
    if _eq_sos is None:
        return chunk
    
    out = chunk.copy()
    for ch in range(chunk.shape[1]):
        # Extract single channel, process through all 18 bands in series
        sig = chunk[:, ch].astype(np.float64)
        for band in range(len(BAND_CENTRES)):
            # if the adjustment value is too small, bypass it
            if abs(_eq_gains_db[band]) < 0.1:
                continue   # true bypass, no floating point touch
            # sosfilt from scipy applies filter and generates new zi_out
            # this zi_out will be used again on next chunk, thus preventing clicks at band joints
            zi_band = _eq_zi[band, :, ch].reshape(1, 2)  # shape (1,2) for single section
            sig, zi_out = sp_signal.sosfilt(_eq_sos[band:band+1], sig, zi=zi_band)
            _eq_zi[band, :, ch] = zi_out[0]
        out[:, ch] = sig.astype(np.float32)
    
    return out

def _get_pa():
    """Return the single shared PyAudio instance, creating it if needed."""
    global _pa_instance
    if _pa_instance is None:
        _pa_instance = pyaudio.PyAudio()
    return _pa_instance

def _stop_stream():
    """Stop and close the current stream without touching is_playing."""
    global current_stream
    if current_stream is not None:
        try:
            current_stream.stop_stream()
            current_stream.close()
        except Exception:
            pass
        current_stream = None

def _open_stream(wf):
    """Open a new PyAudio output stream matched to the wave file."""
    global current_stream
    _stop_stream()
    pa = _get_pa()
    current_stream = pa.open(
        format=pa.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        output=True,
        frames_per_buffer=CHUNK
    )
    
def _playback_thread():
    """Runs in a daemon thread; reads frames and writes to stream."""
    global is_playing, current_wf, current_stream
    while True:
        with _playback_lock:
            if not is_playing or current_wf is None or current_stream is None:
                break
            data = current_wf.readframes(CHUNK)
        if not data:
            # End of file
            with _playback_lock:
                is_playing = False
            break
        try:
            chunk_array = np.frombuffer(data, dtype='float32').copy().reshape(-1, current_wf.getnchannels())
            # taking sample for fft pre sampling
            with fft_buffer_lock:
                fft_buffer_pre[:] = chunk_array[:1024] if len(chunk_array) >= 1024 else np.pad(chunk_array, ((0, 1024-len(chunk_array)), (0,0)))
                
            chunk_array = np.frombuffer(data, dtype='float32').copy().reshape(-1, current_wf.getnchannels())

            if st_mid == "On":
                chunk_array = apply_midnight_mode(chunk_array)
                
            if st_dsp != "Off" or st_dial == "On":
                chunk_array = apply_eq(chunk_array)
                
            with fft_buffer_lock:
                fft_buffer_post[:] = chunk_array[:1024] if len(chunk_array) >= 1024 else np.pad(chunk_array, ((0, 1024-len(chunk_array)), (0,0)))

            chunk_array *= (st_volume / 100.0)
            data = chunk_array.astype(np.float32).tobytes()
            current_stream.write(data)
        except IOError as e:
            print(f"Audio playback error: {e}")
            with _playback_lock:
                is_playing = False
            break

def play_audio(file_path):
    """Load file_path and start playback from the beginning."""
    global current_wf, current_path, is_playing
    with _playback_lock:
        is_playing = False # signal any running thread to stop
        
    # Give the old thread a moment to exit cleanly
    import time; time.sleep(0.05)
    
    with _playback_lock:
        try:
            if current_wf is not None:
                current_wf.close()
            
            ext = os.path.splitext(file_path)[1].lower()
            
            if ext == '.wav':
                current_wf = wave.open(file_path, 'rb')
                current_path = file_path
                _open_stream(current_wf)
            elif ext == '.flac':
                data, samplerate = sf.read(file_path, dtype='float32')
                current_wf = FlacWrapper(data, samplerate)
                current_samplerate = current_wf.getframerate()
                reset_eq(_eq_gains_db, current_samplerate)
                current_path = file_path
                _open_stream(current_wf)
                
            is_playing = True        
        except Exception as e:
            print(f"Error loading audio file: {e}")
            return

    t = threading.Thread(target=_playback_thread, daemon = True)
    t.start()
    
def select_file(start_dir=os.path.expanduser("~")):
    current_dir = start_dir
    selected = None
    
    while selected is None:
        # Get directory contents - folders first, then wav/flac files only
        entries = [".."] + \
                  [e for e in sorted(os.listdir(current_dir)) if os.path.isdir(os.path.join(current_dir, e))] + \
                  [e for e in sorted(os.listdir(current_dir)) if e.lower().endswith(('.wav', '.flac'))]
        
        # Draw the browser
        screen.fill((30, 30, 30))
        screen.blit(font3.render(current_dir, True, (200, 200, 200)), (10, 10))
        
        for i, entry in enumerate(entries[:20]):  # show max 20 entries
            color = (100, 200, 100) if os.path.isdir(os.path.join(current_dir, entry)) else (255, 255, 255)
            screen.blit(font3.render(entry, True, color), (20, 40 + i * 26))
        
        pygame.display.flip()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                # which entry was clicked?
                idx = (my - 40) // 26
                if 0 <= idx < len(entries):
                    full_path = os.path.join(current_dir, entries[idx])
                    if entries[idx] == "..":
                        current_dir = os.path.dirname(current_dir)
                    elif os.path.isdir(full_path):
                        current_dir = full_path
                    else:
                        selected = full_path  # it's a file, we're done
    
    return selected

def load_file():
    print("Load File clicked")
    path = select_file()
    if not path:
        return
    global id3_text, adj_tag, scroll_pos, delay_help
    id3_text = os.path.splitext(os.path.basename(path))[0]
    adj_tag = ""
    scroll_pos = 0
    delay_help = 0
    print(f"Selected file: {os.path.basename(path)}")
    play_audio(path)

def toggle_midnight():
    global st_mid
    st_mid = "Off" if st_mid == "On" else "On"
    print("Midnight Mode:", st_mid)
            
def toggle_dialogue():
    global st_dial, st_dsp
    if st_dial == "Off":
        st_dial = "On"
        st_dsp = "Off"
    else:
        st_dial = "Off"
    gains = EQ_DIALOGUE if st_dial == "On" else EQ_FLAT
    reset_eq(gains, current_samplerate)
    print("Dialogue Enhancement:", st_dial)

def toggle_dsp():
    global st_dsp, dsp_effect, st_dial
    if st_dsp == "Off" and st_dial == "Off":
        st_dsp="On"; gains = EQ_POP if dsp_effect=="POP" else EQ_ROCK if dsp_effect == "ROCK" else EQ_JAZZ if dsp_effect == "JAZZ" else EQ_FLAT
    elif st_dsp == "On" and st_dial == "Off":
        if dsp_effect == "POP":
            dsp_effect = "ROCK"; gains = EQ_ROCK
        elif dsp_effect == "ROCK":
            dsp_effect = "JAZZ"; gains = EQ_JAZZ
        elif dsp_effect == "JAZZ":
            st_dsp="Off"; dsp_effect = "POP"; gains = EQ_FLAT
    elif st_dial == "On":
            st_dial = "Off"
            st_dsp = "On"
            gains = EQ_POP if dsp_effect=="POP" else EQ_ROCK if dsp_effect == "ROCK" else EQ_JAZZ if dsp_effect == "JAZZ" else EQ_FLAT
    reset_eq(gains, current_samplerate)
    print("DSP Effects:", st_dsp, dsp_effect)

def volume_up():
    global st_volume
    st_volume = min(st_volume + 4, 100)
    print("Volume Up:", st_volume)
    
def volume_down():
    global st_volume
    st_volume = max(st_volume - 4, 0)
    print("Volume Down:", st_volume)
    
def audio_play():
    """Resume from current position, or restart if stopped."""
    global is_playing, current_wf, current_path
    with _playback_lock:
        already = is_playing
    if already:
        return
    with _playback_lock:
        if current_wf is not None:
            try:
                _open_stream(current_wf)
                is_playing = True
            except Exception as e:
                print (f"Play error: {e}")
                return
        elif current_path is not None:
            pass # fall through to play_audio below
        else:
            return
    
    if not is_playing and current_path:
        play_audio(current_path)
        return
    t = threading.Thread(target=_playback_thread, daemon=True)
    t.start()

def audio_pause():
    """Pause: stop writing but keep file position."""
    global is_playing
    with _playback_lock:
        is_playing = False
    import time; time.sleep(0.05)
    _stop_stream()
    print("Paused")
    
def audio_stop():
    """Stop and rewind."""
    global is_playing, current_wf
    with _playback_lock:
        is_playing = False
    import time; time.sleep(0.05)
    _stop_stream()
    with _playback_lock:
        if current_wf is not None:
            current_wf.rewind()
        fft_buffer_pre[:] = 0  # clear the buffer so display goes quiet
        fft_buffer_post[:] = 0  # clear the buffer so display goes quiet
    print("Stopped")
        
button_actions = {
    "Load File": load_file,
    "Midnight Mode": toggle_midnight,
    "Dialogue enhancement": toggle_dialogue,
    "DSP effects": toggle_dsp,
    "Volume Up": volume_up,
    "Volume Down": volume_down,
    "Play" : audio_play,
    "Pause" : audio_pause,
    "Stop" : audio_stop
}

def compute_fft(raw_chunk, n=512):
    """
    Takes a chunk of raw audio samples (numpy array),
    returns 512 magnitude values scaled for display.
    Mimics what the FPGA returns over SPI - just 512 integers.
    """
    FIXED_PEAK = 500.0
    FINE_TUNE =90   
    if raw_chunk.ndim == 2:
        mono = raw_chunk.mean(axis=1)
    else:
        mono = raw_chunk.astype(float)

    window = np.hanning(len(mono))
    windowed = mono * window
    spectrum = np.fft.rfft(windowed, n=1024)[:512]
    magnitude = np.abs(spectrum)

    magnitude = 20 * np.log10(magnitude / FIXED_PEAK + 1e-9)
    # normalise this frame to 0-1 then scale to display
    magnitude = np.clip(magnitude, -FINE_TUNE, 0)
    magnitude = ((magnitude + FINE_TUNE) / FINE_TUNE * 128).astype(int)
    magnitude = np.clip(magnitude, 0, 128)   
    
    return magnitude.tolist()  # plain list of 512 ints, just like SPI would give you

# ── Draw functions ────────────────────────────────────────────────────────────

def draw_sources(selected="USB"):
    sources = ["HDMI", "RADIO", "BT", "USB", "AUX"]
    for i, src in enumerate(sources):
        color = (255, 0, 0) if src == selected else (255, 255, 255)
        text = font.render(src, True, color)
        screen.blit(text, (25+screen_top_corner, 158+screen_top_corner + i * 24))

def draw_id3_tag():
    global scroll_pos, delay_help, adj_tag
    over_length=len(id3_text)-23
    if over_length>0 and delay_help==0:
        adj_tag=id3_text[0+scroll_pos:23+scroll_pos]
        delay_help=50
        scroll_pos = (scroll_pos + 1) % (over_length+1)
        if scroll_pos == 0:
            delay_help=125
    elif over_length <= 0:
        adj_tag = id3_text
    delay_help = max(delay_help -1, 0)
    text_surface = font2.render(adj_tag, True, (255, 255, 255))
    screen.blit(text_surface, (150+ screen_top_corner, 60 + screen_top_corner))
    pygame.draw.rect(screen, (0,0,0), (screen_top_corner+610, screen_top_corner+55, 100, 50))

def draw_effects():
    if st_mid  == "On":  screen.blit(font.render("MIDNIGHT",  True, (255,255,255)), (682, 218))
    if st_dial == "On":  screen.blit(font.render("DIALOGUE",  True, (255,255,255)), (682, 248))
    if st_dsp  != "Off": screen.blit(font.render(dsp_effect,  True, (255,255,255)), (682, 278))

def draw_volume_bar(volume_level=50):
    pygame.draw.rect(screen, (255,255,255), (850+screen_top_corner, 8+screen_top_corner, 100,305))
    pygame.draw.rect(screen, (0,0,0), (851+screen_top_corner, 9+screen_top_corner, 98,303))
    text_surface = font4.render("Volume", True, (255, 255, 255))
    screen.blit(text_surface, (screen_top_corner+870, screen_top_corner+15))
    for i in range (12):
        pygame.draw.rect(screen, (255,255,255), (861 + screen_top_corner, 45 + screen_top_corner+i*22, 79, 21))
    v_adj=12-volume_level*12//100
    for i in range (0, v_adj):
        pygame.draw.rect(screen, (0,0,0), (862 + screen_top_corner, 46 + screen_top_corner+i*22, 77, 19))

def draw_fft(pre_fft, post_fft):
    for i in range(512):
        pre = pre_fft[i]
        post = post_fft[i]
        base = min(pre, post)
        x = screen_top_corner + 123 + i
        y = screen_top_corner + 284

        if base > 0:
            pygame.draw.line(screen, (255, 255, 255),
                             (x, y), (x, y - base), 1)
        if post > pre:
            pygame.draw.line(screen, (0, 255, 0),
                             (x, y - pre), (x, y - post), 1)
        elif pre > post:
            pygame.draw.line(screen, (255, 0, 0),
                             (x, y - post), (x, y - pre), 1)

def draw_buttons():
    buttons = ["Load File","Midnight Mode","Dialogue enhancement","DSP effects",
               "Volume Up","Volume Down","Play","Pause","Stop"]
    specs = [
        (screen_top_corner+100, screen_top_corner+390, 0),
        (screen_top_corner+300, screen_top_corner+340, 1),
        (screen_top_corner+300, screen_top_corner+390, 2),
        (screen_top_corner+300, screen_top_corner+440, 3),
        (screen_top_corner+500, screen_top_corner+360, 4),
        (screen_top_corner+500, screen_top_corner+410, 5),
        (screen_top_corner+700, screen_top_corner+340, 6),
        (screen_top_corner+700, screen_top_corner+390, 7),
        (screen_top_corner+700, screen_top_corner+440, 8),
    ]
    for (x, y, idx) in specs:
        pygame.draw.rect(screen, (200,200,200), (x, y, 180, 40))
        txt = font3.render(buttons[idx], True, (0,0,0))
        screen.blit(txt, (x + 90 - txt.get_width()//2, y + 11))

def positioncheck(x , y, a, b, c, d):
    return a < x < c and b < y < d

def main():
    pygame.display.set_caption("Audio Test UI")
    clock = pygame.time.Clock()
    
    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    if   positioncheck(mx, my, 132, 422, 312, 462): button_actions['Load File']()
                    elif positioncheck(mx, my, 332, 372, 512, 412): button_actions['Midnight Mode']()
                    elif positioncheck(mx, my, 332, 422, 512, 462): button_actions['Dialogue enhancement']()
                    elif positioncheck(mx, my, 332, 472, 512, 512): button_actions['DSP effects']()
                    elif positioncheck(mx, my, 532, 392, 712, 432): button_actions['Volume Up']()
                    elif positioncheck(mx, my, 532, 442, 712, 482): button_actions['Volume Down']()
                    elif positioncheck(mx, my, 732, 372, 912, 412): button_actions['Play']()
                    elif positioncheck(mx, my, 732, 422, 912, 462): button_actions['Pause']()
                    elif positioncheck(mx, my, 732, 472, 912, 512): button_actions['Stop']()

            screen.fill((100, 100, 100))
            pygame.draw.rect(screen, (0,0,0), (0+screen_top_corner,0+screen_top_corner,960,320))
            draw_sources()
            draw_id3_tag()
            draw_effects()
            draw_volume_bar(st_volume)
            with fft_buffer_lock:
                pre_copy = fft_buffer_pre.copy()
                post_copy = fft_buffer_post.copy()
            fft_pre = compute_fft(pre_copy)
            fft_post = compute_fft(post_copy)
            draw_fft(fft_pre, fft_post)
            draw_buttons()
            pygame.display.flip()
            clock.tick(30)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        audio_stop()
        if _pa_instance is not None:
            _pa_instance.terminate()
        pygame.quit()
        sys.exit()
        
if __name__ == "__main__":
    main()
