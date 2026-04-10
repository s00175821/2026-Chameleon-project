----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 13.03.2026 15:45:23
-- Design Name: 
-- Module Name: audio_clock_gen - rtl
-- Project Name: Chameleon
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- Uses systems 12MHz clock to generate sys_clk at 96MHz and I2S clocks
-- Dependencies: clk_wiz_0
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Revision 0.02 - LRCLK fix - deriving directly from clocks rather then counting BCLK
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

entity audio_clock_gen is
    Port ( 
           clk_12mhz    : in STD_LOGIC; -- Cmod A7 onboard 12MHz
           
           -- System clock for all FPGA logic
           sys_clk : out STD_LOGIC; --96 MHz
           pll_locked   : out STD_LOGIC;
           
           -- I2S master clocks (output to RPi and PCM3168A)
           bclk_out     : out STD_LOGIC; -- 6.144MHz
           -- MCLK for codec
           mclk_out     : out std_logic; -- 24.576MHz
           -- Internal strobes synchronous to sys_clk
           -- Use these inside FPGA - never use bclk_out as a clock internally
           bclk_rising  : out STD_LOGIC; -- 1 sys_clk pulse on each BCLK rising edge
           bclk_falling : out STD_LOGIC; -- 1 sys_clk pulse on each BCLK falling edge
           lrclk_out    : out STD_LOGIC; -- 96kHz word clock (to RPi + PCM3168A)
           lrclk_int     : out STD_LOGIC -- internal copy of LRCLK state
          );
end audio_clock_gen;

architecture rtl of audio_clock_gen is
    signal sys_clk_int  : std_logic :='0';
    signal bclk_int     : std_logic :='0'; -- from MMCM output 2
    
    signal bclk_sync_1  : std_logic :='0';
    signal bclk_sync_2  : std_logic :='0';
    signal bclk_prev    : std_logic :='0';
    
    signal lrclk_reg   : std_logic :='0';
    -- 96MHz / 6.144MHz = 15.625 sys_clk ticks per BCLK tick
    -- 32 BCLK ticks per LRCLK half = 32 * 15.625 * 2 = 1000 sys_clk ticks per LRCLK period
    -- So count to 500 sys_clk ticks per LRCLK half (96MHz / 96kHz / 2 = 500) 
    signal lr_div_count : integer range 0 to 499 := 0;
     
     component clk_wiz_0
        port(
            clk_in1     : in std_logic;
            clk_out1    : out std_logic; -- 24.576 MHz
            clk_out2    : out std_logic; -- 96 MHz
            clk_out3    : out std_logic; -- 6.144 MHz
            locked      : out std_logic;
            reset       : in std_logic
        );
     end component;
     
begin
    sys_clk <= sys_clk_int;
    lrclk_int <= lrclk_reg;
    lrclk_out <= lrclk_reg;
    bclk_out <= bclk_int;
    
    u_mmcm : clk_wiz_0
        port map (
            clk_in1 => clk_12mhz,
            clk_out1 => mclk_out,
            clk_out2 => sys_clk_int,
            clk_out3 => bclk_int,
            locked => pll_locked,
            reset => '0'
        );
    
    process(sys_clk_int)
    begin
        if rising_edge(sys_clk_int) then
            bclk_rising <= '0';
            bclk_falling <= '0';
            
            bclk_sync_1 <= bclk_int;
            bclk_sync_2 <= bclk_sync_1;
            bclk_prev <= bclk_sync_2;
            
            -- Edge detection
            if bclk_sync_1 = '0' and bclk_int = '1' then
                bclk_rising <='1';
            elsif bclk_sync_1 = '1' and bclk_int = '0' then
                bclk_falling <= '1';
            end if;
            
            -- LRCLK: 96MHz / 96kHz / 2 = 500 counts per half period
            if lr_div_count >= 499 and bclk_sync_1 = '1' and bclk_int = '0' then
                lr_div_count <= 0;
                lrclk_reg    <= not lrclk_reg;
            else 
                lr_div_count <= lr_div_count + 1;
            end if;
            
            
            
            
        end if;
    end process;
end rtl;
