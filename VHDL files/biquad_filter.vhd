----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 16.03.2026 13:09:08
-- Design Name: 
-- Module Name: biquad_filter - rtl
-- Project Name: Chameleon
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- Implements Biquad IIR filter with 5 coefficients.    
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- Single biquad (second-order IIR) filter section
-- Direct Form I implementation
--
-- Difference equation:
--   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
--                  - a1*y[n-1] - a2*y[n-2]
--
-- Coefficient format: Q2.29 signed 32-bit
--   Range: -2.0 to +1.999999998
--   Scale: float * 2^29
--
-- Audio format: Q23 signed 24-bit (matches rest of design)
--
-- Pipeline: 5 multiply-accumulate stages + output register
-- Latency: 6 sys_clk cycles per sample
-- Throughput: 1 stereo sample pair per valid pulse (96kHz)
-- At 96MHz: 1000 cycles available, 6 used. Very comfortable.

entity biquad_filter is
    Port (
        sys_clk             : in std_logic;
        rst                 : in std_logic;
        
        -- coefficient inputs (Q2.29, written once by SPI handler)
        coeff_b0            : in signed(31 downto 0);
        coeff_b1            : in signed(31 downto 0);
        coeff_b2            : in signed(31 downto 0);
        coeff_a1            : in signed(31 downto 0);
        coeff_a2            : in signed(31 downto 0);
        
        -- bypass: when high, passes input through unchanged
        bypass              : in std_logic;
        
        -- audio input
        left_in             : in signed(23 downto 0);
        right_in            : in signed(23 downto 0);
        valid_in            : in std_logic;
        
        -- audio output
        left_out            : out signed(23 downto 0);
        right_out           : out signed(23 downto 0);
        valid_out           : out std_logic
    );
end biquad_filter;

architecture rtl of biquad_filter is
    -- DSP48 inference attribute
    attribute use_dsp : string;
    
    -- ----------------------------------------------------------------
    -- Delay line state registers
    -- x1=x[n-1], x2=x[n-2], y1=y[n-1], y2=y[n-2]
    -- Separate state for left and right channels
    -- ----------------------------------------------------------------
    signal lx1, lx2         : signed(23 downto 0) := (others => '0');
    signal ly1, ly2         : signed(23 downto 0) := (others => '0');
    signal rx1, rx2         : signed(23 downto 0) := (others => '0');
    signal ry1, ry2         : signed(23 downto 0) := (others => '0');
    
    -- ----------------------------------------------------------------
    -- Multiply-accumulate pipeline
    -- Product format: Q23 * Q2.29 = Q25.52 (56-bit signed)
    -- After accumulation, scale back to Q23 by shifting right 29
    -- ----------------------------------------------------------------
    
    -- Left channel products (DSP48 inferred)
    signal l_b0x0           : signed(63 downto 0) := (others => '0');
    signal l_b1x1           : signed(63 downto 0) := (others => '0');
    signal l_b2x2           : signed(63 downto 0) := (others => '0');
    signal l_a1y1           : signed(63 downto 0) := (others => '0');
    signal l_a2y2           : signed(63 downto 0) := (others => '0');
    
    attribute use_dsp of l_b0x0 : signal is "yes";
    attribute use_dsp of l_b1x1 : signal is "yes";
    attribute use_dsp of l_b2x2 : signal is "yes";
    attribute use_dsp of l_a1y1 : signal is "yes";
    attribute use_dsp of l_a2y2 : signal is "yes";
    
    -- Right channel products (DSP48 inferred)
    signal r_b0x0           : signed(63 downto 0) := (others => '0');
    signal r_b1x1           : signed(63 downto 0) := (others => '0');
    signal r_b2x2           : signed(63 downto 0) := (others => '0');
    signal r_a1y1           : signed(63 downto 0) := (others => '0');
    signal r_a2y2           : signed(63 downto 0) := (others => '0');
    
    attribute use_dsp of r_b0x0 : signal is "yes";
    attribute use_dsp of r_b1x1 : signal is "yes";
    attribute use_dsp of r_b2x2 : signal is "yes";
    attribute use_dsp of r_a1y1 : signal is "yes";
    attribute use_dsp of r_a2y2 : signal is "yes";
    
     -- Accumulator (sum of all products)
     signal l_accum         : signed(63 downto 0) := (others => '0');
     signal r_accum         : signed(63 downto 0) := (others => '0');
     
     -- Output before saturation
     signal l_result        : signed(23 downto 0) := (others => '0');
     signal r_result        : signed(23 downto 0) := (others => '0');
     
      -- Pipeline valid delay: 6 stages to match multiply latency
      signal valid_pipe     : std_logic_vector(5 downto 0) := (others => '0');
      
      -- Input registered for pipeline alignment
      signal left_in_r      : signed(23 downto 0) := (others => '0');
      signal right_in_r     : signed(23 downto 0) := (others => '0');
      
       -- ----------------------------------------------------------------
    -- Saturation function: clamp 56-bit result to 24-bit range
    -- Prevents overflow wrapping which would sound terrible
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
    valid_out <= valid_pipe(5);
    
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            -- Shift valid flag through pipeline
            valid_pipe <= valid_pipe(4 downto 0) & valid_in;
            if rst = '1' then
                lx1 <= (others => '0'); lx2 <= (others => '0');
                ly1 <= (others => '0'); ly2 <= (others => '0');
                rx1 <= (others => '0'); rx2 <= (others => '0');
                ry1 <= (others => '0'); ry2 <= (others => '0');
                l_b0x0 <= (others => '0');
                l_b1x1 <= (others => '0'); 
                l_b2x2 <= (others => '0'); 
                l_a1y1 <= (others => '0'); 
                l_a2y2 <= (others => '0'); 
                r_b0x0 <= (others => '0');
                r_b1x1 <= (others => '0'); 
                r_b2x2 <= (others => '0'); 
                r_a1y1 <= (others => '0'); 
                r_a2y2 <= (others => '0');
                l_accum <= (others => '0');
                r_accum <= (others => '0');
                l_result <= (others => '0');
                r_result <= (others => '0');
                left_out <= (others => '0');
                right_out <= (others => '0');
            elsif valid_in = '1' then
                -- register inputs
                left_in_r <= left_in;
                right_in_r <= right_in;
                
                 -- ------------------------------------------------
                -- Cycle 1: all five multiplications in parallel
                -- DSP48 blocks handle these simultaneously
                -- ------------------------------------------------
                l_b0x0 <= coeff_b0 * resize(left_in, 32);
                l_b1x1 <= coeff_b1 * resize(lx1, 32);
                l_b2x2 <= coeff_b2 * resize(lx2, 32);
                l_a1y1 <= coeff_a1 * resize(ly1, 32);
                l_a2y2 <= coeff_a2 * resize(ly2, 32);
                
                r_b0x0 <= coeff_b0 * resize(right_in, 32);
                r_b1x1 <= coeff_b1 * resize(rx1, 32);
                r_b2x2 <= coeff_b2 * resize(rx2, 32);
                r_a1y1 <= coeff_a1 * resize(ry1, 32);
                r_a2y2 <= coeff_a2 * resize(ry2, 32);
                
                -- ------------------------------------------------
                -- Cycle 2: accumulate
                -- y[n] = b0x0 + b1x1 + b2x2 - a1y1 - a2y2
                -- Note: subtract a1 and a2 terms (IIR feedback)
                -- ------------------------------------------------
                l_accum <= l_b0x0+l_b1x1+l_b2x2-l_a1y1-l_a2y2;
                r_accum <= r_b0x0+r_b1x1+r_b2x2-r_a1y1-r_a2y2;
                
                -- ------------------------------------------------
                -- Cycle 3: scale back to Q23 and saturate
                -- Q25.52 >> 29 = Q23 (shift right by 29)
                -- ------------------------------------------------
                l_result <= saturate(l_accum);
                r_result <= saturate(r_accum);
                
                -- ------------------------------------------------
                -- Cycle 4: update delay line state registers
                -- Uses result from cycle 3
                -- ------------------------------------------------
                lx2 <= lx1;
                lx1 <= left_in_r;
                ly2 <= ly1;
                ly1 <= l_result;
                rx2 <= rx1;
                rx1 <= right_in_r;
                ry2 <= ry1;
                ry1 <= r_result;
                
                 -- ------------------------------------------------
                -- Cycle 5: output register
                -- Bypass inserts original input instead of result
                -- ------------------------------------------------
                if bypass = '1' then
                    left_out <= left_in_r;
                    right_out <= right_in_r;
                else
                    left_out <= l_result;
                    right_out <= r_result;
                end if;
            end if;
        end if;
    end process;                                                                            
                 
end rtl;
