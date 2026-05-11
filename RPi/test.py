"""
Raspberry Pi Audio Player with DSP Effects
==========================================

A Pygame-based audio player application for Raspberry Pi that supports:
- WAV and FLAC file playback
- Real-time FFT visualization
- 18-band parametric equalizer
- Audio effects: Midnight Mode (dynamic range compression), Dialogue Enhancement, DSP presets
- Volume control
- File browser interface

Author: Grzegorz Chojnacki
Date: Jan-May 2026
"""

# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------
import soundfile as sf              # For reading FLAC files
import pygame                       # For GUI and display
import random                       # For potential randomization (currently unused)
import os                          # For file system operations
import tkinter as tk                # For file dialog (currently unused)
import tkinter.filedialog as fd     # For file dialog (currently unused)
import sys                         # For system exit
import pyaudio                     # For audio output
import numpy as np                  # For numerical operations (FFT, audio processing)
import wave                        # For reading WAV files
import threading                   # For playback thread
from scipy import signal as sp_signal  # For SOS filter implementation


# -----------------------------------------------------------------------------
# PYGAME / AUDIO DRIVER CONFIGURATION
# -----------------------------------------------------------------------------
# Configure environment variables for ALSA audio driver on Raspberry Pi
os.environ['SDL_AUDIODRIVER'] = 'alsa'
os.environ['PYGAME_SDL_AUDIODRIVER'] = 'alsa'
os.environ['AUDIODEV'] = 'hw:0,0'  # Hardware device 0,0

# Initialize Pygame for display and event handling
pygame.init()

# Set up display window: 1024 pixels wide, 600 pixels tall
screen = pygame.display.set_mode((1024, 600))

# Define fonts for different UI elements
font = pygame.font.SysFont('liberationsansnarrow', 24, bold=True)   # For buttons and labels
font2 = pygame.font.SysFont('takaogothic', 40)                   # For ID3 tag display
font3 = pygame.font.SysFont('liberationsansnarrow', 18, bold=True) # For smaller labels
font4 = pygame.font.SysFont('liberationsansnarrow', 20, bold=True) # For medium labels


# -----------------------------------------------------------------------------
# UI STATE VARIABLES
# -----------------------------------------------------------------------------
# Screen layout offset (margins)
screen_top_corner = 32

# Scrolling variables for long ID3 tags
scroll_pos = 0          # Current scroll position for ID3 tag text
delay_help = 0         # Delay counter for scroll timing

# Feature toggle states
st_mid = "Off"         # Midnight mode (dynamic range compression): Off by default
st_dial = "Off"        # Dialogue enhancement: Off by default
st_dsp = "Off"         # DSP effects: Off by default
st_volume = 50         # Default volume level (0-100%)

# ID3 tag display variables
adj_tag = ""           # Adjusted tag text for display
id3_text = "Load File first"  # Default text when no file loaded

# Current DSP effect preset
dsp_effect = "POP"     # Current DSP effect: POP, ROCK, or JAZZ


# -----------------------------------------------------------------------------
# AUDIO PLAYBACK GLOBALS
# -----------------------------------------------------------------------------
# PyAudio instance (singleton - only one instance for the entire application)
_pa_instance = None

# Current audio stream and file
current_stream = None      # PyAudio output stream
current_wf = None          # Wave file or FLAC wrapper object
current_path = None        # Path to currently loaded audio file
is_playing = False         # Playback state flag

# Thread lock to prevent race conditions between playback thread and main thread
_playback_lock = threading.Lock()


# -----------------------------------------------------------------------------
# FFT VISUALIZATION GLOBALS
# -----------------------------------------------------------------------------
# FFT magnitude arrays for pre and post processing visualization
fft_pre = [0] * 512       # Magnitudes before audio processing
fft_post = [0] * 512      # Magnitudes after audio processing

# Buffers for storing audio chunks for FFT analysis
fft_buffer_pre = np.zeros((1024, 2), dtype='float32')   # Before processing
fft_buffer_post = np.zeros((1024, 2), dtype='float32')  # After processing

# Lock to prevent race conditions when accessing FFT buffers
fft_buffer_lock = threading.Lock()

# Audio buffer chunk size - larger buffer reduces audio glitches on Raspberry Pi
CHUNK = 2048


# -----------------------------------------------------------------------------
# EQUALIZER CONFIGURATION
# -----------------------------------------------------------------------------
# Center frequencies for 18-band EQ (in Hz)
BAND_CENTRES = [55, 77, 110, 156, 220, 311, 440, 622, 880,
                1200, 1800, 2500, 3500, 5000, 7000, 10000, 14000, 20000]

# Q factor for all EQ bands - controls the width/bandwidth of each band
# Higher Q = narrower band, Lower Q = wider band
BAND_Q = 1.4

# Current sample rate (updated when loading a file)
current_samplerate = 44100

# Equalizer state variables
_eq_sos = None       # Second-order sections (SOS) matrix, shape (18, 6) - computed from gains
_eq_zi = None        # Filter state for each band, shape (18, 2, 2) - 18 bands, stereo, 2 state vars
_eq_gains_db = [0.0] * 18  # Current gain values in dB for each band (flat by default)

# Predefined EQ presets (gain values in dB for each of the 18 bands)
# Flat EQ - no boost or cut
EQ_FLAT = [0.0] * 18

# Dialogue enhancement - boosts midrange frequencies for clearer voice
EQ_DIALOGUE = [ -2.0, -2.0, -3.0, -2.0,  0.0,  0.0,  1.0,  2.0,  3.0,
                 4.0,  4.0,  3.0,  3.0,  2.0,  0.0, -1.0, -2.0, -3.0]

# POP music preset - bass and treble boost
EQ_POP = [  6.0,  5.0,  3.0,  0.0,  -2.0,  -4.0,  -4.0,  -6.0,  -3.0,
             1.0,  0.0,  0.0,  2.0,  1.0,  2.0,  4.0,  5.0, 6.0]

# ROCK music preset - boosted highs and mids
EQ_ROCK = [  4.0,  5.0,  5.0,  5.0,  4.0,  3.0, 1.0, 0.0,  -1.0,
             -2.0,  -2.0,  0.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0]

# JAZZ music preset - warm tone with mid and treble emphasis
EQ_JAZZ = [  0.0,  1.0,  2.0,  2.0,  3.0,  1.0,  2.0,  0.0,  0.0,
             2.0,  1.0,  2.0,  4.0,  3.0,  3.0,  2.0,  1.0,  0.0]


# =============================================================================
# FLAC WRAPPER CLASS
# =============================================================================
class FlacWrapper:
    """
    Wrapper class to make FLAC files (loaded via soundfile) compatible
    with the wave.Wave_read interface expected by the playback code.
    """
    
    def __init__(self, data, samplerate):
        """
        Initialize with audio data and sample rate.
        
        Args:
            data: numpy array of audio samples (float32)
            samplerate: Sample rate in Hz
        """
        # if mono, reshape to 2D so channel logic is consistent
        if data.ndim == 1:
            data = data.reshape(-1, 1)
        self.data = data
        self.samplerate = samplerate
        self.pos = 0  # current read position in frames

    def getsampwidth(self):
        """Return sample width in bytes (4 bytes for float32)."""
        return 4

    def getnchannels(self):
        """Return number of channels (1 for mono, 2 for stereo)."""
        return self.data.shape[1]

    def getframerate(self):
        """Return sample rate in Hz."""
        return self.samplerate

    def readframes(self, n):
        """
        Read n frames of audio data.
        
        Args:
            n: Number of frames to read
            
        Returns:
            Bytes object containing the audio data
        """
        chunk = self.data[self.pos : self.pos + n]
        self.pos += n
        return chunk.tobytes()

    def rewind(self):
        """Reset read position to beginning of file."""
        self.pos = 0

    def close(self):
        """Close the file (no-op for this wrapper)."""
        pass  # nothing to close


# =============================================================================
# AUDIO EFFECTS
# =============================================================================

# State variable for Midnight Mode smooth gain transition
_mm_smooth_gain = 1.0

def apply_midnight_mode(chunk):
    """
    Apply Midnight Mode dynamic range compression to audio chunk.
    
    This implements a dynamic range compressor/expander that:
    - Compresses loud signals above -18 dBFS (4:1 ratio)
    - Expands quiet signals below -45 dBFS (2:1 ratio)
    - Applies smooth attack/release to avoid artifacts
    - Includes a hard limiter at -3 dBFS
    - Adds +6 dB makeup gain
    
    Args:
        chunk: numpy array of audio samples (float32), shape (n_frames, n_channels)
        
    Returns:
        Processed audio chunk with dynamic range compression applied
    """
    global _mm_smooth_gain
    
    # Constants matching hardware VHDL specification
    THRESH_HIGH = 10 ** (-18 / 20)   # -18 dBFS = 0.1259 (compression threshold)
    THRESH_LOW = 10 ** (-45 / 20)    # -45 dBFS = 0.00562 (expansion threshold)
    PEAK_CEIL = 10 ** (-3 / 20)      # -3 dBFS = 0.7079 (hard limiter ceiling)
    MAKEUP = 10 ** (6 / 20)          # +6 dB = 1.9953 (makeup gain)
    RATIO_HIGH = 4.0                # Compression ratio above threshold
    RATIO_LOW = 2.0                 # Expansion ratio below threshold
    
    # Attack/release coefficients (per chunk, approximated from per-sample at 96kHz)
    # VHDL uses per-sample at 96kHz; we approximate per-chunk at ~44100Hz
    SR = 44100
    ATTACK_TC = np.exp(-1.0 / (0.010 * SR / len(chunk)))   # 10ms attack time
    RELEASE_TC = np.exp(-1.0 / (0.200 * SR / len(chunk)))  # 200ms release time
    
    # Compute RMS level of this chunk (mono mix if stereo)
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
    
    # Smooth the gain with attack/release envelope
    if target_gain < _mm_smooth_gain:
        # Attack phase (gain decreasing)
        _mm_smooth_gain = ATTACK_TC * _mm_smooth_gain + (1 - ATTACK_TC) * target_gain
    else:
        # Release phase (gain increasing)
        _mm_smooth_gain = RELEASE_TC * _mm_smooth_gain + (1 - RELEASE_TC) * target_gain
    
    # Apply gain to audio
    out = chunk * _mm_smooth_gain
    
    # Hard limiter at -3dBFS to prevent clipping
    out = np.clip(out, -PEAK_CEIL, PEAK_CEIL)
    
    return out.astype(np.float32)


def build_eq_sos(gains_db, samplerate):
    """
    Build a Second-Order Sections (SOS) matrix from 18 dB gain values.
    
    Each band is implemented as a peaking EQ biquad (bell filter).
    
    Args:
        gains_db: List of 18 gain values in dB
        samplerate: Sample rate in Hz
        
    Returns:
        numpy array of shape (18, 6) containing SOS coefficients ready for sosfilt
    """
    sections = []
    for i, (fc, gain_db) in enumerate(zip(BAND_CENTRES, gains_db)):
        # Skip bands above Nyquist frequency
        if fc >= samplerate / 2:
            # Pass through (unity biquad) - no effect
            sections.append([1.0, 0.0, 0.0, 1.0, 0.0, 0.0])
            continue
        
        # Convert dB gain to amplitude (sqrt of power gain)
        A = 10 ** (gain_db / 40.0)
        
        # Normalized angular frequency
        w0 = 2 * np.pi * fc / samplerate
        
        # Alpha controls the width of the bell (bandwidth)
        alpha = np.sin(w0) / (2 * BAND_Q)
        
        # Biquad coefficients for peaking EQ
        b0 = 1 + alpha * A
        b1 = -2 * np.cos(w0)
        b2 = 1 - alpha * A
        a0 = 1 + alpha / A
        a1 = -2 * np.cos(w0)
        a2 = 1 - alpha / A
        
        # Normalize by a0 and store as SOS: [b0, b1, b2, 1.0, a1, a2]
        # Note: a0 is divided out, so the first element of the denominator is 1.0
        sections.append([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])
    
    return np.array(sections)


def reset_eq(gains_db, samplerate):
    """
    Recompute SOS matrix and reset filter state.
    Call this on file load or when changing EQ preset.
    
    Args:
        gains_db: List of 18 gain values in dB
        samplerate: Sample rate in Hz
    """
    global _eq_sos, _eq_zi, _eq_gains_db
    _eq_gains_db = gains_db
    _eq_sos = build_eq_sos(gains_db, samplerate)
    # zi shape: (n_sections, 2, n_channels) - scipy convention for filter state
    _eq_zi = np.zeros((len(BAND_CENTRES), 2, 2))


def apply_eq(chunk):
    """
    Apply 18-band EQ to a float32 stereo audio chunk.
    
    Processes each channel independently, maintaining state via _eq_zi
    to prevent clicks at chunk boundaries.
    
    Args:
        chunk: numpy array of shape (n_frames, 2) containing stereo audio
        
    Returns:
        Processed audio chunk with EQ applied
    """
    global _eq_zi
    
    if _eq_sos is None:
        return chunk
    
    out = chunk.copy()
    
    for ch in range(chunk.shape[1]):
        # Extract single channel, process through all 18 bands in series
        sig = chunk[:, ch].astype(np.float64)
        
        for band in range(len(BAND_CENTRES)):
            # Optimization: bypass band if gain is very small
            if abs(_eq_gains_db[band]) < 0.1:
                continue  # true bypass, no floating point touch
            
            # sosfilt from scipy applies filter and generates new zi_out
            # This zi_out will be used on next chunk, preventing clicks at boundaries
            zi_band = _eq_zi[band, :, ch].reshape(1, 2)  # shape (1,2) for single section
            sig, zi_out = sp_signal.sosfilt(_eq_sos[band:band+1], sig, zi=zi_band)
            _eq_zi[band, :, ch] = zi_out[0]
        
        out[:, ch] = sig.astype(np.float32)
    
    return out


# =============================================================================
# PYAUDIO STREAM MANAGEMENT
# =============================================================================

def _get_pa():
    """
    Return the single shared PyAudio instance, creating it if needed.
    
    Returns:
        PyAudio instance
    """
    global _pa_instance
    if _pa_instance is None:
        _pa_instance = pyaudio.PyAudio()
    return _pa_instance


def _stop_stream():
    """
    Stop and close the current stream without touching is_playing flag.
    """
    global current_stream
    if current_stream is not None:
        try:
            current_stream.stop_stream()
            current_stream.close()
        except Exception:
            pass
        current_stream = None


def _open_stream(wf):
    """
    Open a new PyAudio output stream matched to the wave file.
    
    Args:
        wf: Wave file or FLAC wrapper object
    """
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


# =============================================================================
# PLAYBACK THREAD
# =============================================================================

def _playback_thread():
    """
    Runs in a daemon thread; reads frames from wave file and writes to stream.
    
    This function runs continuously while is_playing is True, reading audio
    chunks and applying effects before outputting to the audio stream.
    """
    global is_playing, current_wf, current_stream
    
    while True:
        with _playback_lock:
            if not is_playing or current_wf is None or current_stream is None:
                break
            data = current_wf.readframes(CHUNK)
        
        if not data:
            # End of file reached
            with _playback_lock:
                is_playing = False
            break
        
        try:
            # Convert bytes to numpy array and reshape
            chunk_array = np.frombuffer(data, dtype='float32').copy().reshape(-1, current_wf.getnchannels())
            
            # Store sample for FFT visualization (pre-processing)
            with fft_buffer_lock:
                if len(chunk_array) >= 1024:
                    fft_buffer_pre[:] = chunk_array[:1024]
                else:
                    # Pad if chunk is smaller than buffer
                    fft_buffer_pre[:] = np.pad(chunk_array, ((0, 1024-len(chunk_array)), (0,0)))
            
            # Re-read data (could reuse chunk_array but this is clearer)
            chunk_array = np.frombuffer(data, dtype='float32').copy().reshape(-1, current_wf.getnchannels())
            
            # Apply Midnight Mode if enabled
            if st_mid == "On":
                chunk_array = apply_midnight_mode(chunk_array)
            
            # Apply EQ if either DSP or Dialogue enhancement is enabled
            if st_dsp != "Off" or st_dial == "On":
                chunk_array = apply_eq(chunk_array)
            
            # Store sample for FFT visualization (post-processing)
            with fft_buffer_lock:
                if len(chunk_array) >= 1024:
                    fft_buffer_post[:] = chunk_array[:1024]
                else:
                    fft_buffer_post[:] = np.pad(chunk_array, ((0, 1024-len(chunk_array)), (0,0)))
            
            # Apply volume scaling
            chunk_array *= (st_volume / 100.0)
            
            # Convert back to bytes and write to stream
            data = chunk_array.astype(np.float32).tobytes()
            current_stream.write(data)
            
        except IOError as e:
            print(f"Audio playback error: {e}")
            with _playback_lock:
                is_playing = False
            break


def play_audio(file_path):
    """
    Load file_path and start playback from the beginning.
    
    Args:
        file_path: Path to audio file (WAV or FLAC)
    """
    global current_wf, current_path, is_playing
    
    # Signal any running playback thread to stop
    with _playback_lock:
        is_playing = False
    
    # Give the old thread a moment to exit cleanly
    import time; time.sleep(0.05)
    
    with _playback_lock:
        try:
            # Close current file if open
            if current_wf is not None:
                current_wf.close()
            
            # Determine file type from extension
            ext = os.path.splitext(file_path)[1].lower()
            
            if ext == '.wav':
                # Open WAV file
                current_wf = wave.open(file_path, 'rb')
                current_path = file_path
                _open_stream(current_wf)
                
            elif ext == '.flac':
                # Load FLAC file using soundfile
                data, samplerate = sf.read(file_path, dtype='float32')
                current_wf = FlacWrapper(data, samplerate)
                current_samplerate = current_wf.getframerate()
                
                # Initialize EQ with current gains and sample rate
                reset_eq(_eq_gains_db, current_samplerate)
                current_path = file_path
                _open_stream(current_wf)
            
            # Start playback
            is_playing = True
            
        except Exception as e:
            print(f"Error loading audio file: {e}")
            return
    
    # Start playback thread as daemon (will exit when main thread exits)
    t = threading.Thread(target=_playback_thread, daemon=True)
    t.start()


# =============================================================================
# FILE BROWSER
# =============================================================================

def select_file(start_dir=os.path.expanduser("~")):
    """
    Display a file browser and let user select a WAV or FLAC file.
    
    Args:
        start_dir: Starting directory for browsing
        
    Returns:
        Full path to selected audio file, or None if cancelled
    """
    current_dir = start_dir
    selected = None
    
    while selected is None:
        # Get directory contents - folders first, then WAV/FLAC files
        entries = [".."] + \
                  [e for e in sorted(os.listdir(current_dir)) if os.path.isdir(os.path.join(current_dir, e))] + \
                  [e for e in sorted(os.listdir(current_dir)) if e.lower().endswith(('.wav', '.flac'))]
        
        # Draw the browser
        screen.fill((30, 30, 30))
        screen.blit(font3.render(current_dir, True, (200, 200, 200)), (10, 10))
        
        # Display first 20 entries
        for i, entry in enumerate(entries[:20]):
            # Green for directories, white for files
            color = (100, 200, 100) if os.path.isdir(os.path.join(current_dir, entry)) else (255, 255, 255)
            screen.blit(font.render(entry, True, color), (20, 40 + i * 30))
        
        pygame.display.flip()
        
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                # Which entry was clicked?
                idx = (my - 40) // 30
                if 0 <= idx < len(entries):
                    full_path = os.path.join(current_dir, entries[idx])
                    if entries[idx] == "..":
                        # Go up one directory
                        current_dir = os.path.dirname(current_dir)
                    elif os.path.isdir(full_path):
                        # Enter directory
                        current_dir = full_path
                    else:
                        # It's a file, we're done
                        selected = full_path
    
    return selected


def load_file():
    """Load File button handler - show file browser and play selected file."""
    print("Load File clicked")
    path = select_file()
    if not path:
        return
    
    global id3_text, adj_tag, scroll_pos, delay_help
    
    # Set ID3 tag to filename (without extension)
    id3_text = os.path.splitext(os.path.basename(path))[0]
    adj_tag = ""
    scroll_pos = 0
    delay_help = 0
    
    print(f"Selected file: {os.path.basename(path)}")
    play_audio(path)


# =============================================================================
# BUTTON ACTION HANDLERS
# =============================================================================

def toggle_midnight():
    """Toggle Midnight Mode on/off."""
    global st_mid
    st_mid = "Off" if st_mid == "On" else "On"
    print("Midnight Mode:", st_mid)


def toggle_dialogue():
    """
    Toggle Dialogue Enhancement on/off.
    When enabled, disables DSP effects and applies dialogue EQ preset.
    """
    global st_dial, st_dsp
    if st_dial == "Off":
        st_dial = "On"
        st_dsp = "Off"  # Disable DSP when enabling dialogue
    else:
        st_dial = "Off"
    
    # Set EQ gains based on state
    gains = EQ_DIALOGUE if st_dial == "On" else EQ_FLAT
    reset_eq(gains, current_samplerate)
    print("Dialogue Enhancement:", st_dial)


def toggle_dsp():
    """
    Cycle through DSP effects: Off -> POP -> ROCK -> JAZZ -> Off.
    When enabled, disables Dialogue Enhancement.
    """
    global st_dsp, dsp_effect, st_dial
    
    if st_dsp == "Off" and st_dial == "Off":
        # Currently off, turn on with current effect
        st_dsp = "On"
        gains = EQ_POP if dsp_effect == "POP" else EQ_ROCK if dsp_effect == "ROCK" else EQ_JAZZ if dsp_effect == "JAZZ" else EQ_FLAT
        
    elif st_dsp == "On" and st_dial == "Off":
        # Currently on, cycle to next effect
        if dsp_effect == "POP":
            dsp_effect = "ROCK"
            gains = EQ_ROCK
        elif dsp_effect == "ROCK":
            dsp_effect = "JAZZ"
            gains = EQ_JAZZ
        elif dsp_effect == "JAZZ":
            st_dsp = "Off"
            dsp_effect = "POP"
            gains = EQ_FLAT
    
    elif st_dial == "On":
        # Dialogue is on, turn it off and turn DSP on
        st_dial = "Off"
        st_dsp = "On"
        gains = EQ_POP if dsp_effect == "POP" else EQ_ROCK if dsp_effect == "ROCK" else EQ_JAZZ if dsp_effect == "JAZZ" else EQ_FLAT
    
    reset_eq(gains, current_samplerate)
    print("DSP Effects:", st_dsp, dsp_effect)


def volume_up():
    """Increase volume by 4%, max 100%."""
    global st_volume
    st_volume = min(st_volume + 4, 100)
    print("Volume Up:", st_volume)


def volume_down():
    """Decrease volume by 4%, min 0%."""
    global st_volume
    st_volume = max(st_volume - 4, 0)
    print("Volume Down:", st_volume)


def audio_play():
    """
    Resume from current position, or restart if stopped.
    If no file is loaded, does nothing.
    """
    global is_playing, current_wf, current_path
    
    with _playback_lock:
        already = is_playing
    
    if already:
        return  # Already playing
    
    with _playback_lock:
        if current_wf is not None:
            try:
                _open_stream(current_wf)
                is_playing = True
            except Exception as e:
                print(f"Play error: {e}")
                return
        elif current_path is not None:
            pass  # fall through to play_audio below
        else:
            return  # No file loaded
    
    if not is_playing and current_path:
        play_audio(current_path)
        return
    
    t = threading.Thread(target=_playback_thread, daemon=True)
    t.start()


def audio_pause():
    """
    Pause: stop writing but keep file position.
    Allows playback to be resumed from current position.
    """
    global is_playing
    with _playback_lock:
        is_playing = False
    import time; time.sleep(0.05)
    _stop_stream()
    print("Paused")


def audio_stop():
    """
    Stop and rewind to beginning of file.
    Clears FFT buffers so visualization goes quiet.
    """
    global is_playing, current_wf
    with _playback_lock:
        is_playing = False
    import time; time.sleep(0.05)
    _stop_stream()
    with _playback_lock:
        if current_wf is not None:
            current_wf.rewind()
        # Clear FFT buffers so display goes quiet
        fft_buffer_pre[:] = 0
        fft_buffer_post[:] = 0
    print("Stopped")


# Dictionary mapping button names to action functions
button_actions = {
    "Load File": load_file,
    "Midnight Mode": toggle_midnight,
    "Dialogue enhancement": toggle_dialogue,
    "DSP effects": toggle_dsp,
    "Volume Up": volume_up,
    "Volume Down": volume_down,
    "Play": audio_play,
    "Pause": audio_pause,
    "Stop": audio_stop
}


# =============================================================================
# FFT COMPUTATION
# =============================================================================

def compute_fft(raw_chunk, n=512):
    """
    Compute FFT magnitude spectrum for visualization.
    
    Takes a chunk of raw audio samples and returns 512 magnitude values
    scaled for display. Mimics what the FPGA returns over SPI.
    
    Args:
        raw_chunk: numpy array of audio samples
        n: Number of FFT bins to return (default 512)
        
    Returns:
        List of 512 integer magnitude values (0-128) for visualization
    """
    FIXED_PEAK = 500.0      # Reference peak level
    FINE_TUNE = 90          # Scaling factor for display
    
    # Convert stereo to mono by averaging channels
    if raw_chunk.ndim == 2:
        mono = raw_chunk.mean(axis=1)
    else:
        mono = raw_chunk.astype(float)
    
    # Apply Hanning window to reduce spectral leakage
    window = np.hanning(len(mono))
    windowed = mono * window
    
    # Compute FFT (real FFT, return only positive frequencies)
    spectrum = np.fft.rfft(windowed, n=1024)[:512]
    magnitude = np.abs(spectrum)
    
    # Convert to dB scale
    magnitude = 20 * np.log10(magnitude / FIXED_PEAK + 1e-9)
    
    # Normalize and scale to display range (0-128)
    magnitude = np.clip(magnitude, -FINE_TUNE, 0)
    magnitude = ((magnitude + FINE_TUNE) / FINE_TUNE * 128).astype(int)
    magnitude = np.clip(magnitude, 0, 128)
    
    return magnitude.tolist()  # Return as plain list of 512 ints


# =============================================================================
# DRAWING FUNCTIONS
# =============================================================================

def draw_sources(selected="USB"):
    """
    Draw the input source indicators at the top of the screen.
    
    Args:
        selected: Currently selected source (highlighted in red)
    """
    sources = ["HDMI", "RADIO", "BT", "USB", "AUX"]
    for i, src in enumerate(sources):
        color = (255, 0, 0) if src == selected else (255, 255, 255)
        text = font.render(src, True, color)
        screen.blit(text, (25+screen_top_corner, 158+screen_top_corner + i * 24))


def draw_id3_tag():
    """
    Draw the ID3 tag (filename) in the center of the screen.
    Handles scrolling if the text is too long for the display.
    """
    global scroll_pos, delay_help, adj_tag
    
    over_length = len(id3_text) - 23
    
    if over_length > 0 and delay_help == 0:
        # Text is too long, scroll it
        adj_tag = id3_text[0+scroll_pos:23+scroll_pos]
        delay_help = 50
        scroll_pos = (scroll_pos + 1) % (over_length + 1)
        if scroll_pos == 0:
            delay_help = 125  # Longer pause at start
    elif over_length <= 0:
        # Text fits, display as-is
        adj_tag = id3_text
    
    # Decrement delay counter
    delay_help = max(delay_help - 1, 0)
    
    # Render and display the text
    text_surface = font2.render(adj_tag, True, (255, 255, 255))
    screen.blit(text_surface, (150 + screen_top_corner, 60 + screen_top_corner))
    
    # Draw black rectangle to clear any leftover text
    pygame.draw.rect(screen, (0, 0, 0), (screen_top_corner+610, screen_top_corner+55, 100, 50))


def draw_effects():
    """
    Draw indicator labels for enabled audio effects.
    Shows MIDNIGHT, DIALOGUE, or current DSP preset name.
    """
    if st_mid == "On":
        screen.blit(font.render("MIDNIGHT", True, (255, 255, 255)), (682, 218))
    if st_dial == "On":
        screen.blit(font.render("DIALOGUE", True, (255, 255, 255)), (682, 248))
    if st_dsp != "Off":
        screen.blit(font.render(dsp_effect, True, (255, 255, 255)), (682, 278))


def draw_volume_bar(volume_level=50):
    """
    Draw the volume bar indicator on the right side of the screen.
    
    Args:
        volume_level: Current volume (0-100)
    """
    # Draw outer rectangle (border)
    pygame.draw.rect(screen, (255, 255, 255), (850+screen_top_corner, 8+screen_top_corner, 100, 305))
    
    # Draw inner black rectangle
    pygame.draw.rect(screen, (0, 0, 0), (851+screen_top_corner, 9+screen_top_corner, 98, 303))
    
    # Draw "Volume" label
    text_surface = font4.render("Volume", True, (255, 255, 255))
    screen.blit(text_surface, (screen_top_corner+870, screen_top_corner+15))
    
    # Draw volume segments (12 segments representing 0-100%)
    for i in range(12):
        pygame.draw.rect(screen, (255, 255, 255), 
                         (861 + screen_top_corner, 45 + screen_top_corner + i*22, 79, 21))
    
    # Calculate which segments to fill (black = active, white = inactive)
    v_adj = 12 - volume_level * 12 // 100
    
    # Fill inactive segments
    for i in range(0, v_adj):
        pygame.draw.rect(screen, (0, 0, 0), 
                         (862 + screen_top_corner, 46 + screen_top_corner + i*22, 77, 19))


def draw_fft(pre_fft, post_fft):
    """
    Draw the FFT visualization bars.
    
    Shows pre-processing FFT in white, post-processing FFT in green (boost)
    or red (cut) to indicate effect of audio processing.
    
    Args:
        pre_fft: List of 512 magnitude values before processing
        post_fft: List of 512 magnitude values after processing
    """
    for i in range(512):
        pre = pre_fft[i]
        post = post_fft[i]
        base = min(pre, post)
        x = screen_top_corner + 123 + i
        y = screen_top_corner + 284
        
        # Draw base level (white) - common to both pre and post
        if base > 0:
            pygame.draw.line(screen, (255, 255, 255), (x, y), (x, y - base), 1)
        
        # Draw difference: green if post > pre (boost), red if pre > post (cut)
        if post > pre:
            pygame.draw.line(screen, (0, 255, 0), (x, y - pre), (x, y - post), 1)
        elif pre > post:
            pygame.draw.line(screen, (255, 0, 0), (x, y - post), (x, y - pre), 1)


def draw_buttons():
    """
    Draw all the control buttons at the bottom of the screen.
    """
    buttons = ["Load File", "Midnight Mode", "Dialogue enhancement", "DSP effects",
               "Volume Up", "Volume Down", "Play", "Pause", "Stop"]
    
    # Button specifications: (x, y, index)
    specs = [
        (screen_top_corner+100, screen_top_corner+390, 0),   # Load File
        (screen_top_corner+300, screen_top_corner+340, 1),   # Midnight Mode
        (screen_top_corner+300, screen_top_corner+390, 2),   # Dialogue enhancement
        (screen_top_corner+300, screen_top_corner+440, 3),   # DSP effects
        (screen_top_corner+500, screen_top_corner+360, 4),   # Volume Up
        (screen_top_corner+500, screen_top_corner+410, 5),   # Volume Down
        (screen_top_corner+700, screen_top_corner+340, 6),   # Play
        (screen_top_corner+700, screen_top_corner+390, 7),   # Pause
        (screen_top_corner+700, screen_top_corner+440, 8),   # Stop
    ]
    
    for (x, y, idx) in specs:
        # Draw button background
        pygame.draw.rect(screen, (200, 200, 200), (x, y, 180, 40))
        # Draw button text (centered)
        txt = font3.render(buttons[idx], True, (0, 0, 0))
        screen.blit(txt, (x + 90 - txt.get_width()//2, y + 11))


def positioncheck(x, y, a, b, c, d):
    """
    Check if point (x,y) is within rectangle defined by (a,b) to (c,d).
    
    Args:
        x, y: Point coordinates
        a, b: Top-left corner of rectangle
        c, d: Bottom-right corner of rectangle
        
    Returns:
        True if point is inside rectangle, False otherwise
    """
    return a < x < c and b < y < d


# =============================================================================
# MAIN FUNCTION
# =============================================================================

def main():
    """
    Main application loop.
    Handles events, updates display, and manages audio playback.
    """
    pygame.display.set_caption("Audio Test UI")
    clock = pygame.time.Clock()
    
    try:
        running = True
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    # Check which button was clicked using position check
                    if positioncheck(mx, my, 132, 422, 312, 462):
                        button_actions['Load File']()
                    elif positioncheck(mx, my, 332, 372, 512, 412):
                        button_actions['Midnight Mode']()
                    elif positioncheck(mx, my, 332, 422, 512, 462):
                        button_actions['Dialogue enhancement']()
                    elif positioncheck(mx, my, 332, 472, 512, 512):
                        button_actions['DSP effects']()
                    elif positioncheck(mx, my, 532, 392, 712, 432):
                        button_actions['Volume Up']()
                    elif positioncheck(mx, my, 532, 442, 712, 482):
                        button_actions['Volume Down']()
                    elif positioncheck(mx, my, 732, 372, 912, 412):
                        button_actions['Play']()
                    elif positioncheck(mx, my, 732, 422, 912, 462):
                        button_actions['Pause']()
                    elif positioncheck(mx, my, 732, 472, 912, 512):
                        button_actions['Stop']()
            
            # Draw everything
            screen.fill((100, 100, 100))  # Gray background
            
            # Draw main display area
            pygame.draw.rect(screen, (0, 0, 0), (0+screen_top_corner, 0+screen_top_corner, 960, 320))
            
            # Draw UI elements
            draw_sources()
            draw_id3_tag()
            draw_effects()
            draw_volume_bar(st_volume)
            
            # Get FFT buffers (thread-safe)
            with fft_buffer_lock:
                pre_copy = fft_buffer_pre.copy()
                post_copy = fft_buffer_post.copy()
            
            # Compute FFT for visualization
            fft_pre = compute_fft(pre_copy)
            fft_post = compute_fft(post_copy)
            
            # Draw FFT and buttons
            draw_fft(fft_pre, fft_post)
            draw_buttons()
            
            # Update display
            pygame.display.flip()
            
            # Limit to 30 FPS
            clock.tick(30)
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Cleanup on exit
        audio_stop()
        if _pa_instance is not None:
            _pa_instance.terminate()
        pygame.quit()
        sys.exit()


# Run the application if this script is executed directly
if __name__ == "__main__":
    main()
