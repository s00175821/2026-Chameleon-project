----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Module Name: spi_slave - rtl
-- Project Name: Chameleon
-- Revision 0.03 - Fixed MOSI sampling alignment.
--   sclk_rising/falling were previously registered inside the clocked
--   process, adding an extra cycle of latency vs mosi_sync_2.
--   They are now combinatorial signals derived from already-registered
--   sclk_sync_2 and sclk_prev, so mosi_sync_2 is perfectly aligned
--   when the shift register process samples it.
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity spi_slave is
    Port ( 
           sys_clk          : in STD_LOGIC;
           rst              : in STD_LOGIC;
           spi_sclk         : in STD_LOGIC;
           spi_cs_n         : in STD_LOGIC;
           spi_miso         : out STD_LOGIC;
           spi_mosi         : in STD_LOGIC;
           tx_data          : in STD_LOGIC_VECTOR (7 downto 0);
           tx_load          : in STD_LOGIC;
           tx_done          : out STD_LOGIC;
           rx_data          : out STD_LOGIC_VECTOR (7 downto 0);
           rx_valid         : out STD_LOGIC;
           cmd_filter_mode  : out STD_LOGIC_VECTOR (7 downto 0);
           cmd_filter_valid : out STD_LOGIC;
           cmd_coeff_index  : out STD_LOGIC_VECTOR (7 downto 0);
           cmd_coeff_value  : out STD_LOGIC_VECTOR (23 downto 0);
           cmd_coeff_valid  : out STD_LOGIC;
           cmd_midnight_en  : out std_logic;
           cmd_midnight_set : out std_logic;
           cmd_headphone_en : out std_logic;
           cmd_headphone_set: out std_logic;
           eq_coeff_band    : out integer range 0 to 17;
           eq_coeff_sel     : out integer range 0 to 4;
           eq_coeff_val     : out signed(31 downto 0);
           eq_coeff_we      : out std_logic;
           eq_bypass_band   : out integer range 0 to 17;
           eq_bypass_val    : out std_logic;
           eq_bypass_we     : out std_logic;
           eq_bypass_all    : out std_logic;
           eq_bypass_set    : out std_logic;
           cmd_fft_request  : out std_logic;
           busy             : out STD_LOGIC
           );
end spi_slave;

architecture rtl of spi_slave is

    signal sclk_sync_1  : std_logic := '0';
    signal sclk_sync_2  : std_logic := '0';
    signal sclk_prev    : std_logic := '0';
    signal cs_sync_1    : std_logic := '1';
    signal cs_sync_2    : std_logic := '1';
    signal cs_prev      : std_logic := '1';
    signal mosi_sync_1  : std_logic := '0';
    signal mosi_sync_2  : std_logic := '0';
    
    -- Combinatorial edge strobes - derived from registered signals so
    -- glitch-free, but fire one cycle earlier than if registered again.
    -- This aligns them with mosi_sync_2.
    signal sclk_rising  : std_logic;
    signal sclk_falling : std_logic;
    signal cs_rising    : std_logic;
    signal cs_falling   : std_logic;
    
    signal tx_shift     : std_logic_vector(7 downto 0) := (others => '0');
    signal rx_shift     : std_logic_vector(7 downto 0) := (others => '0');
    signal bit_count    : integer range 0 to 7 := 0;
    signal active       : std_logic := '0';
    
    signal rx_valid_int  : std_logic := '0';
    signal tx_done_int   : std_logic := '0';
    signal rx_data_int   : std_logic_vector(7 downto 0) := (others => '0');
    
    signal coeff_buf     : std_logic_vector(31 downto 0) := (others => '0');
    signal eq_band_reg   : integer range 0 to 9 := 0;
    signal eq_sel_reg    : integer range 0 to 4 := 0;
    
    type cmd_state_t is (
        WAIT_CMD, CMD_FILTER_DATA,
        CMD_COEFF_IDX, CMD_COEFF_B0, CMD_COEFF_B1, CMD_COEFF_B2,
        CMD_MIDNIGHT_DATA, CMD_HEADPHONE_DATA,
        CMD_EQ_BAND, CMD_EQ_COEFF_SEL,
        CMD_EQ_C0, CMD_EQ_C1, CMD_EQ_C2, CMD_EQ_C3,
        CMD_EQ_BYPASS_BAND, CMD_EQ_BYPASS_VAL, CMD_EQ_GLOBAL
    );
    signal cmd_state     : cmd_state_t := WAIT_CMD;
    signal coeff_idx_buf : std_logic_vector(7 downto 0) := (others => '0');
    
begin
    
    rx_valid <= rx_valid_int;
    tx_done  <= tx_done_int;
    rx_data  <= rx_data_int;

    -- ----------------------------------------------------------------
    -- Combinatorial edge detection.
    -- sclk_sync_2 and sclk_prev are registered, so these are glitch-free.
    -- By not registering them again, the shift register process sees
    -- sclk_rising in the same cycle that sclk_prev updates, which is
    -- exactly the cycle where mosi_sync_2 holds the correct data bit.
    -- ----------------------------------------------------------------
    sclk_rising  <= sclk_sync_2 and not sclk_prev;
    sclk_falling <= not sclk_sync_2 and sclk_prev;
    cs_falling   <= not cs_sync_2 and cs_prev;
    cs_rising    <= cs_sync_2 and not cs_prev;

    -- ----------------------------------------------------------------
    -- Synchroniser - registers all SPI inputs into sys_clk domain
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            sclk_sync_1 <= spi_sclk;
            sclk_sync_2 <= sclk_sync_1;
            sclk_prev   <= sclk_sync_2;
            cs_sync_1   <= spi_cs_n;
            cs_sync_2   <= cs_sync_1;
            cs_prev     <= cs_sync_2;
            mosi_sync_1 <= spi_mosi;
            mosi_sync_2 <= mosi_sync_1;
        end if;
    end process;
    
    -- ----------------------------------------------------------------
    -- SPI shift register - TX and RX simultaneously
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            tx_done_int  <= '0';
            rx_valid_int <= '0';
            
            if rst = '1' then
                tx_shift  <= (others => '0');
                rx_shift  <= (others => '0');
                bit_count <= 0;
                active    <= '0';
                busy      <= '0';
                spi_miso  <= '1';              
            else                
                if cs_falling = '1' then
                    active    <= '1';
                    busy      <= '1';
                    bit_count <= 0;
                    tx_shift  <= tx_data;
                    spi_miso  <= tx_data(7);
                end if;
                
                if active = '1' then
                    if sclk_rising = '1' then   
                        rx_shift <= rx_shift(6 downto 0) & mosi_sync_2;
                    end if;
                    
                    if sclk_falling = '1' then
                        if bit_count = 7 then
                            bit_count    <= 0;
                            rx_data_int  <= rx_shift;
                            rx_valid_int <= '1';
                            tx_done_int  <= '1';
                            tx_shift     <= tx_data;
                            spi_miso     <= tx_data(7);
                        else
                            bit_count <= bit_count + 1;
                            tx_shift  <= tx_shift(6 downto 0) & '0';
                            spi_miso  <= tx_shift(6);
                        end if;
                    end if;
                end if;
                
                if cs_rising = '1' then
                    active   <= '0';
                    busy     <= '0';
                    spi_miso <= '1';
                end if;
            end if;
        end if;                                                
    end process; 
    
    -- ----------------------------------------------------------------
    -- Command decoder
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            cmd_filter_valid  <= '0';
            cmd_coeff_valid   <= '0';
            cmd_midnight_set  <= '0';
            cmd_headphone_set <= '0';
            eq_coeff_we       <= '0';
            eq_bypass_we      <= '0';
            eq_bypass_set     <= '0';
            cmd_fft_request   <= '0';
            
            if rst = '1' then
                cmd_state        <= WAIT_CMD;
                cmd_filter_mode  <= (others => '0');
                cmd_coeff_index  <= (others => '0');
                cmd_coeff_value  <= (others => '0');
                cmd_midnight_en  <= '0';
                cmd_headphone_en <= '0';
                eq_coeff_band    <= 0;
                eq_coeff_sel     <= 0;
                eq_coeff_val     <= (others => '0');
                eq_bypass_band   <= 0;
                eq_bypass_val    <= '0';
                eq_bypass_all    <= '0';
                coeff_idx_buf    <= (others => '0');
                coeff_buf        <= (others => '0');
                eq_band_reg      <= 0;
                eq_sel_reg       <= 0;
                
            elsif rx_valid_int = '1' then
                case cmd_state is
                    when WAIT_CMD =>
                        case rx_data_int is
                            when x"01" => cmd_state <= CMD_FILTER_DATA;
                            when x"02" => cmd_state <= CMD_COEFF_IDX;
                            when x"03" =>
                                cmd_fft_request <= '1';
                                cmd_state <= WAIT_CMD;
                            when x"04" => cmd_state <= CMD_MIDNIGHT_DATA;
                            when x"05" => cmd_state <= CMD_HEADPHONE_DATA;
                            when x"10" => cmd_state <= CMD_EQ_BAND;
                            when x"11" => cmd_state <= CMD_EQ_BYPASS_BAND;
                            when x"12" => cmd_state <= CMD_EQ_GLOBAL;
                            when others => cmd_state <= WAIT_CMD;
                        end case;
                    when CMD_FILTER_DATA =>
                        cmd_filter_mode  <= rx_data_int;
                        cmd_filter_valid <= '1';
                        cmd_state        <= WAIT_CMD;
                    when CMD_COEFF_IDX =>
                        coeff_idx_buf <= rx_data_int;
                        cmd_state     <= CMD_COEFF_B0;
                    when CMD_COEFF_B0 =>
                        coeff_buf(23 downto 16) <= rx_data_int;
                        cmd_state <= CMD_COEFF_B1;
                    when CMD_COEFF_B1 =>
                        coeff_buf(15 downto 8) <= rx_data_int;
                        cmd_state <= CMD_COEFF_B2;
                    when CMD_COEFF_B2 =>
                        cmd_coeff_index <= coeff_idx_buf;
                        cmd_coeff_value <= coeff_buf(23 downto 8) & rx_data_int;
                        cmd_coeff_valid <= '1';
                        cmd_state       <= WAIT_CMD;
                    when CMD_MIDNIGHT_DATA =>
                        cmd_midnight_en  <= rx_data_int(0);
                        cmd_midnight_set <= '1';
                        cmd_state        <= WAIT_CMD;
                    when CMD_HEADPHONE_DATA =>
                        cmd_headphone_en  <= rx_data_int(0);
                        cmd_headphone_set <= '1';
                        cmd_state         <= WAIT_CMD;
                    when CMD_EQ_BAND =>
                        eq_band_reg <= to_integer(unsigned(rx_data_int(3 downto 0)));
                        cmd_state   <= CMD_EQ_COEFF_SEL;
                    when CMD_EQ_COEFF_SEL =>
                        eq_sel_reg <= to_integer(unsigned(rx_data_int(2 downto 0)));
                        cmd_state  <= CMD_EQ_C0;
                    when CMD_EQ_C0 =>
                        coeff_buf(31 downto 24) <= rx_data_int;
                        cmd_state <= CMD_EQ_C1;
                    when CMD_EQ_C1 =>
                        coeff_buf(23 downto 16) <= rx_data_int;
                        cmd_state <= CMD_EQ_C2;
                    when CMD_EQ_C2 =>
                        coeff_buf(15 downto 8) <= rx_data_int;
                        cmd_state <= CMD_EQ_C3;
                    when CMD_EQ_C3 =>
                        eq_coeff_band <= eq_band_reg;
                        eq_coeff_sel  <= eq_sel_reg;
                        eq_coeff_val  <= signed(coeff_buf(31 downto 8) & rx_data_int);
                        eq_coeff_we   <= '1';
                        cmd_state     <= WAIT_CMD;
                    when CMD_EQ_BYPASS_BAND =>
                        eq_bypass_band <= to_integer(unsigned(rx_data_int(3 downto 0)));
                        cmd_state      <= CMD_EQ_BYPASS_VAL;
                    when CMD_EQ_BYPASS_VAL =>
                        eq_bypass_val <= rx_data_int(0);
                        eq_bypass_we  <= '1';
                        cmd_state     <= WAIT_CMD;
                    when CMD_EQ_GLOBAL =>
                        eq_bypass_all <= rx_data_int(0);
                        eq_bypass_set <= '1';
                        cmd_state     <= WAIT_CMD;    
                end case;
            end if;
        end if;
    end process;                                                     
                                                                                                                                                                                                                                                                                                                                                           
end rtl;