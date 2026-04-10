----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 16.03.2026 13:09:08
-- Design Name: 
-- Module Name: i2s_transmitter - rtl
-- Project Name: Chameleon
-- Target Devices: 
-- Tool Versions: 
-- Description: Module that sends results to codec board
-- 
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

entity i2s_transmitter is
    Port (
        sys_clk             : in std_logic;
        rst                 : in std_logic;
        
        -- Clocking strobes from audio_clock_gen (sys_clk domain)
        bclk_falling        : in std_logic; -- shift out on falling BCLK edge
        lrclk_int           : in std_logic; -- current LRCLK state
        
        -- Audio input (Q23, from crossfeed output)
        left_in             : in std_logic_vector(23 downto 0);
        right_in            : in std_logic_vector(23 downto 0);
        valid_in            : in std_logic;
        
        -- I2S serial output to PCM3168A SDIN
        sdata               : out std_logic 
         );
end i2s_transmitter;

architecture rtl of i2s_transmitter is
    -- 32-bit shift registers (24-bit audio + 8 bit xero padding)
    signal left_shift       : std_logic_vector(31 downto 0) := (others => '0');
    signal right_shift      : std_logic_vector(31 downto 0) := (others => '0');
    
    -- Current active shift register (selects L or R based on LRCLK)
    signal active_shift     : std_logic_vector(31 downto 0) := (others => '0');
    
    -- Bit counter: counts 32 bits per channel
    signal bit_count        : integer range 0 to 31 := 0;
    
    -- LRCLK edge detection
    signal lrclk_prev       : std_logic :='0';
        
begin
    process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                left_shift <= (others=> '0');
                right_shift <= (others=> '0');
                active_shift <= (others=> '0');
                bit_count <= 0;
                lrclk_prev <= '0';
                sdata <= '0';
            else
                lrclk_prev <= lrclk_int;
                
                -- Latch new sample pair when valid
                -- Store both channels, output them when LRCLK says so
                if valid_in = '1' then
                    -- Pad 24-bit audio to 32-bit frame (8 zero LSBs)
                    left_shift <= left_in & x"00";
                    right_shift <= right_in & x"00";
                end if;
                
                -- LRCLK rising edge: switch to right channel
                -- LRCLK falling edge: switch to left channel
                -- I2S standard: left channel transimitted when LRCLK low
                -- right channel when LRCLK high
                if lrclk_int /= lrclk_prev then
                    bit_count <= 0;
                    if lrclk_int='1' then
                        -- switching to right channel
                        active_shift <= right_shift;
                    else
                        -- switching to left channel
                        active_shift <= left_shift;
                    end if;
                end if;
                
                -- shift out on BCLK falling edge (PCM3168A samples on rising)
                if bclk_falling = '1' then
                    --MSB first
                    sdata <= active_shift(31);
                    active_shift <= active_shift(30 downto 0) & '0';
                    if bit_count <31 then
                        bit_count <= bit_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;                                                                                                                                                            
end rtl;
