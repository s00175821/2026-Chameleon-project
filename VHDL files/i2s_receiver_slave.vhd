----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 13.03.2026 16:12:43
-- Design Name: 
-- Module Name: i2s_receiver_slave - rtl
-- Project Name: Chameleon 
-- Target Devices: 
-- Tool Versions: 
-- Description: Receives I2S audio from Pi or 
-- from codec board
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

entity i2s_receiver_slave is
    generic (
        WORD_WIDTH      : integer := 32  -- 32-bit I2S frames (carrying 24-bit audio)
    );
    Port ( 
           sys_clk      : in STD_LOGIC;
           rst          : in STD_LOGIC;
           
           -- Strobes from audio_clock_gen (sys_clk domain)
           bclk_rising  : in STD_LOGIC;
           lrclk_int    : in STD_LOGIC;
           
           -- serial data from RPi
           sdata        : in STD_LOGIC;
           
            -- Output: one stereo pair per 96kHz cycle
           left_sample  : out STD_LOGIC_VECTOR (23 downto 0);
           right_sample : out STD_LOGIC_VECTOR (23 downto 0);
           sample_valid : out STD_LOGIC;
           debug_pin    : out STD_LOGIC
     );
end i2s_receiver_slave;

architecture rtl of i2s_receiver_slave is
    signal shift_reg    : std_logic_vector(WORD_WIDTH-1 downto 0) := (others=>'0');
    signal bit_count   : integer range 0 to WORD_WIDTH-1 :=0;
    signal lrclk_prev   : std_logic :='0';
    signal left_buf      : std_logic_vector(WORD_WIDTH-1 downto 0) := (others=>'0');
begin
    process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            sample_valid <='0';
            
            if rst = '1' then
                shift_reg <= (others => '0');
                left_buf <= (others => '0');
                bit_count <= 0;
                lrclk_prev <= '0';
            elsif bclk_rising='1' then
                -- shift SDATA in, MSB first
                shift_reg <= shift_reg(WORD_WIDTH-2 downto 0) & sdata;
                lrclk_prev <= lrclk_int;
                
                -- Detect LRCLK edge = end of one channel word
                if lrclk_int /= lrclk_prev then
                    bit_count <= 0;
                    --debug_pin <= '1';  -- pulse on every LRCLK edge
                    
                    if lrclk_prev = '0' then
                        -- Rising LRCLK edge: left channel word just completed
                        -- Shift left by 1 to correct 1-bit offset
                        left_buf <= shift_reg(WORD_WIDTH-2 downto 0) & '0';
                    else
                        -- Falling LRCLK edge: right channel word just completed
                        -- Shift left by 1 to correct 1-bit offset, output stereo pair
                        left_sample  <= left_buf(WORD_WIDTH-1 downto WORD_WIDTH-24);
                        right_sample <= shift_reg(WORD_WIDTH-2 downto WORD_WIDTH-25);
                        sample_valid <= '1';
                    end if;
                else    
                    if bit_count < WORD_WIDTH-1 then
                        bit_count <= bit_count+1;
                    end if;
                end if;
            end if;
        end if;
    end process;
                
end rtl;