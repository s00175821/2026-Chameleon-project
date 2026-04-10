----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 16.03.2026 13:09:08
-- Design Name: 
-- Module Name: crossfeed - rtl
-- Project Name: Chameleon
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- crossfeed effect for headphones - simulates speakers
-- by feeding some left channel to right and vice versa
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------

-- Bauer BS2B Crossfeed for headphone listening
--
-- Simulates speaker-to-ear acoustic crosstalk:
--   - Each channel receives a delayed, low-pass filtered,
--     attenuated copy of the opposite channel
--   - Creates a natural front-centre stereo image on headphones
--   - Reduces listening fatigue on long sessions
--
-- Parameters (fixed, tuned for BS2B standard):
--   Direct path attenuation : -1dB   (level = 0.891 in linear)
--   Crossfeed attenuation   : -6dB   (level = 0.501 in linear)
--   Crossfeed delay         : 0.3ms  = 29 samples at 96kHz
--   Low-pass cutoff         : 700Hz  (one-pole IIR, coefficient pre-computed)
--
-- Signal flow:
--   left_out  = direct_gain*left_in  + cross_gain*lpf(delay(right_in))
--   right_out = direct_gain*right_in + cross_gain*lpf(delay(left_in))
--
-- Enable signal: when low, passes audio through unmodified
-- Controlled by RPi headphone detect GPIO

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity crossfeed is
    Port (
        sys_clk             : in std_logic;
        rst                 : in std_logic;
        
        -- Enable: high = crossfeed active (headphones connected)
        --         low  = bypass (speakers, pass through unchanged)
        enable              : in std_logic;
        -- audio input
        left_in             : in std_logic_vector(23 downto 0);
        right_in            : in std_logic_vector(23 downto 0);
        valid_in            : in std_logic;
        -- audio output
        left_out             : out std_logic_vector(23 downto 0);
        right_out            : out std_logic_vector(23 downto 0);
        valid_out            : out std_logic
    );
end crossfeed;

architecture rtl of crossfeed is
    -- ----------------------------------------------------------------
    -- Fixed gain constants (Q2.29 format, same as biquad coefficients)
    -- Direct path: -1dB  = 0.8913 * 2^29 = 478513044
    -- Cross path:  -6dB  = 0.5012 * 2^29 = 269079701
    -- ----------------------------------------------------------------
    constant DIRECT_GAIN : signed (31 downto 0) := to_signed(478513044, 32);
    constant CROSS_GAIN : signed (31 downto 0) := to_signed(269079701, 32);
    
    -- ----------------------------------------------------------------
    -- Low-pass filter coefficient (one-pole IIR)
    -- H(z) = (1-a) / (1 - a*z^-1)
    -- Cutoff 700Hz at 96kHz:
    --   a = exp(-2*pi*700/96000) = exp(-0.04581) = 0.95522
    -- In Q2.29: 0.95521 * 2^29 = 512829169
    -- Feedforward: (1-a) = 0.04479
    -- In Q2.29: 0.04479 * 2^29 = 24041743
    -- ----------------------------------------------------------------
    constant LPF_A : signed (31 downto 0) := to_signed(512829169, 32);
    constant LPF_1MINUS_A : signed (31 downto 0) := to_signed(24041743, 32);
    
    -- ----------------------------------------------------------------
    -- Crossfeed delay line
    -- 0.3ms at 96kHz = 28.8 samples, round to 29
    -- ----------------------------------------------------------------
    constant DELAY_SAMPLES : integer := 29;
    
    type delay_line_t is 
        array (0 to DELAY_SAMPLES-1) of signed (23 downto 0);
    signal left_delay : delay_line_t := (others => (others => '0'));
    signal right_delay : delay_line_t := (others => (others => '0'));
     -- Delayed samples (output of delay line)
     signal left_delayed : signed (23 downto 0) := (others => '0');
     signal right_delayed : signed (23 downto 0) := (others => '0');
     -- ----------------------------------------------------------------
    -- Low-pass filter state and output
    -- Applied to the delayed cross-channel signal
    -- ----------------------------------------------------------------
    signal lpf_left_y1 : signed(23 downto 0) := (others => '0');
    signal lpf_right_y1 : signed(23 downto 0) := (others => '0');
    signal lpf_left_out : signed(23 downto 0) := (others => '0');
    signal lpf_right_out : signed(23 downto 0) := (others => '0');
    
    -- ----------------------------------------------------------------
    -- DSP48 inference attributes
    -- ----------------------------------------------------------------
    attribute use_dsp : string;
    
    signal l_direct_prod : signed(63 downto 0) := (others => '0');
    signal r_direct_prod : signed(63 downto 0) := (others => '0');
    signal l_cross_prod : signed(63 downto 0) := (others => '0');
    signal r_cross_prod : signed(63 downto 0) := (others => '0');
    signal lpf_l_a_prod : signed(63 downto 0) := (others => '0');
    signal lpf_r_a_prod : signed(63 downto 0) := (others => '0');
    signal lpf_l_ff_prod : signed(63 downto 0) := (others => '0');
    signal lpf_r_ff_prod : signed(63 downto 0) := (others => '0');
    
    attribute use_dsp of l_direct_prod : signal is "yes";
    attribute use_dsp of r_direct_prod : signal is "yes";
    attribute use_dsp of l_cross_prod  : signal is "yes";
    attribute use_dsp of r_cross_prod  : signal is "yes";
    attribute use_dsp of lpf_l_a_prod  : signal is "yes";
    attribute use_dsp of lpf_r_a_prod  : signal is "yes";
    attribute use_dsp of lpf_l_ff_prod : signal is "yes";
    attribute use_dsp of lpf_r_ff_prod : signal is "yes";
    
    -- ----------------------------------------------------------------
    -- Pipeline valid delay (accounts for multiply latency)
    -- ----------------------------------------------------------------
    signal valid_pipe : std_logic_vector(3 downto 0) := (others => '0');
    
    -- ----------------------------------------------------------------
    -- Saturation helper (identical to biquad_filter)
    -- ----------------------------------------------------------------
    function saturate(x : signed(63 downto 0))
        return signed is
        constant MAX_POS : signed(23 downto 0) := to_signed(8388607, 24); --  2^23 - 1
        constant MAX_NEG : signed(23 downto 0) := to_signed(-8388608, 24); --  -2^23
    begin
        -- Check if upper bits are all the same as sign bit
        -- If not, we have overflow 
        if x(63 downto 60) = "0000" or
           x(63 downto 60) = "1111" then
            --No overflow: take bits 52 downto 29 (Q23 result)
            return x(60 downto 37);
        elsif x(63) = '0' then  
            return MAX_POS; -- positive overflow
        else
            return MAX_NEG; -- negative overflow
        end if;
    end function;       
          
begin
    valid_out <= valid_pipe(3);
    process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            valid_pipe <= valid_pipe(2 downto 0) & valid_in;
            if rst = '1' then
                left_delay    <= (others => (others => '0'));
                right_delay   <= (others => (others => '0'));
                left_delayed  <= (others => '0');
                right_delayed <= (others => '0');
                lpf_left_y1   <= (others => '0');
                lpf_right_y1  <= (others => '0');
                lpf_left_out  <= (others => '0');
                lpf_right_out <= (others => '0');
                l_direct_prod <= (others => '0');
                r_direct_prod <= (others => '0');
                l_cross_prod  <= (others => '0');
                r_cross_prod  <= (others => '0');
                left_out      <= (others => '0');
                right_out     <= (others => '0');
            elsif valid_in = '1' then
                if enable = '0' then
                    -- ------------------------------------------------
                    -- Bypass: pass through unchanged
                    -- ------------------------------------------------
                    left_out <= left_in;
                    right_out <= right_in;
                else
                    -- ------------------------------------------------
                    -- Cycle 1: shift delay lines
                    -- Oldest sample falls off the end automatically
                    -- ------------------------------------------------
                    for i in DELAY_SAMPLES-1 downto 1 loop
                        left_delay(i) <= left_delay(i-1);
                        right_delay(i) <= right_delay(i-1);
                    end loop;
                    left_delay(0) <= signed(left_in);
                    right_delay(0) <= signed(right_in);
                    
                    -- Read delayed sample from end of line
                    left_delayed <= left_delay(DELAY_SAMPLES-1);
                    right_delayed <= right_delay(DELAY_SAMPLES-1);
                    
                    -- ------------------------------------------------
                    -- Cycle 2: one-pole low-pass on delayed signal
                    -- y[n] = (1-a)*x[n] + a*y[n-1]
                    -- DSP48 for both multiply operations
                    -- ------------------------------------------------
                    lpf_l_ff_prod <= LPF_1MINUS_A * resize(left_delayed, 32);
                    lpf_l_a_prod <= LPF_A * resize(lpf_left_y1, 32);
                    lpf_r_ff_prod <= LPF_1MINUS_A * resize(right_delayed, 32);
                    lpf_r_a_prod <= LPF_A * resize(lpf_right_y1, 32);
                    
                    lpf_left_out <= saturate(lpf_l_ff_prod + lpf_l_a_prod);
                    lpf_right_out <= saturate(lpf_r_ff_prod + lpf_r_a_prod);
                    
                    -- Update LPF state
                    lpf_left_y1 <= lpf_left_out;
                    lpf_right_y1 <= lpf_right_out;
                    
                    -- ------------------------------------------------
                    -- Cycle 3: apply gains and sum
                    -- left_out  = DIRECT*left_in + CROSS*lpf(right)
                    -- right_out = DIRECT*right_in + CROSS*lpf(left)
                    -- DSP48 for all four multiplications
                    -- ------------------------------------------------
                    l_direct_prod <= DIRECT_GAIN * resize(signed(left_in), 32);
                    r_direct_prod <= DIRECT_GAIN * resize(signed(right_in), 32);
                    l_cross_prod <= CROSS_GAIN * resize(lpf_right_out, 32);
                    r_cross_prod <= CROSS_GAIN * resize(lpf_left_out, 32);
                    
                    -- ------------------------------------------------
                    -- Cycle 4: accumulate and output
                    -- ------------------------------------------------
                    left_out <= std_logic_vector(saturate(l_direct_prod + l_cross_prod));
                    right_out <= std_logic_vector(saturate(r_direct_prod + r_cross_prod));
                end if;
            end if;
        end if;
    end process;
end rtl;
