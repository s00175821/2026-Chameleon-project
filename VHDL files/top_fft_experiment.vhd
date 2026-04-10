----------------------------------------------------------------------------------
-- Module Name: top_fft_experiment - Behavioral
-- Project Name: Chameleon
-- Description: Stage 2 - Verify FFT buffer output.
--
-- Pipeline: I2S receiver -> fft_buffer -> capture RAM -> SPI
--
-- fft_buffer produces 1024 Hann-windowed 32-bit samples per frame.
-- We capture these into a 1024-word RAM as they stream out.
-- Python reads the RAM over SPI.
--
-- SPI protocol:
--   0x01 = read: transfer 4096 bytes (1024 x int32 LE windowed samples)
--   0x02 = arm:  clear ready flag, wait for next complete frame
--   0x03 = poll: returns 0xFF if frame ready, 0x00 if not
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity top_fft_experiment is
    Port (
        clk_12mhz    : in  STD_LOGIC;
        mclk_out     : out std_logic;
        i2s_bclk     : out STD_LOGIC;
        i2s_lrclk    : out STD_LOGIC;
        i2s_sdata_in : in  STD_LOGIC;
        i2s_sdata_out: out std_logic;
        spi_sclk     : in  STD_LOGIC;
        spi_cs_n     : in  STD_LOGIC;
        spi_miso     : out STD_LOGIC;
        spi_mosi     : in  STD_LOGIC;
        led          : out STD_LOGIC_VECTOR(1 downto 0);
        rgb_r        : out STD_LOGIC;
        rgb_g        : out STD_LOGIC;
        rgb_b        : out STD_LOGIC;
        debug_pin    : out STD_LOGIC
    );
end top_fft_experiment;

architecture rtl of top_fft_experiment is

    signal sys_clk      : std_logic;
    signal pll_locked   : std_logic;
    signal rst          : std_logic;
    signal bclk_rising  : std_logic;
    signal bclk_falling : std_logic;
    signal lrclk_int    : std_logic;

    -- I2S receiver
    signal left_sample  : std_logic_vector(23 downto 0);
    signal right_sample : std_logic_vector(23 downto 0);
    signal sample_valid : std_logic;

    -- FFT buffer AXI-Stream
    signal m_tdata      : std_logic_vector(31 downto 0);
    signal m_tvalid     : std_logic;
    signal m_tlast      : std_logic;

    -- Capture RAM - force BRAM inference
    type ram_t is array (0 to 1023) of std_logic_vector(31 downto 0);
    signal cap_ram      : ram_t;
    attribute ram_style : string;
    attribute ram_style of cap_ram : signal is "block";
    signal cap_ptr      : integer range 0 to 1023 := 0;
    signal cap_done     : std_logic := '0';
    signal arm          : std_logic := '0';  -- set by SPI 0x02, cleared by capture process

    -- LED
    signal led_stretch  : integer range 0 to 9999999 := 0;

    -- SPI synchroniser
    signal sclk_r       : std_logic_vector(2 downto 0);
    signal cs_r         : std_logic_vector(2 downto 0);
    signal mosi_r       : std_logic_vector(1 downto 0);
    signal mosi_data    : std_logic;
    signal sclk_rising  : std_logic;
    signal sclk_falling : std_logic;
    signal cs_active    : std_logic;
    signal cs_start     : std_logic;
    signal cs_stop      : std_logic;

    -- SPI reading section
    signal bitcnt               : integer range 0 to 7 := 0;
    signal byte_received        : std_logic;
    signal byte_data_received   : std_logic_vector(7 downto 0) := (others => '0');

    -- SPI transmit section
    signal tx_shift             : std_logic_vector(7 downto 0) := (others => '0');

    -- SPI state machine
    type spi_state_t is (IDLE, RX_CMD, TX_STATUS, TX_DATA);
    signal spi_state    : spi_state_t := IDLE;
    signal word_idx     : integer range 0 to 1023 := 0;
    signal byte_in_word : integer range 0 to 3 := 0;
    signal cur_word     : std_logic_vector(31 downto 0) := (others => '0');

begin

    rst           <= not pll_locked;
    led(0)        <= pll_locked;
    led(1)        <= '1' when led_stretch > 0 else '0';
    rgb_r         <= '1';
    rgb_g         <= '1';
    rgb_b         <= '1';
    i2s_sdata_out <= '0';
    debug_pin     <= cap_done;

    -- ----------------------------------------------------------------
    -- Clock generation
    -- ----------------------------------------------------------------
    u_clkgen : entity work.audio_clock_gen
        port map (
            clk_12mhz    => clk_12mhz,   mclk_out     => mclk_out,
            sys_clk      => sys_clk,      pll_locked   => pll_locked,
            bclk_out     => i2s_bclk,     bclk_rising  => bclk_rising,
            bclk_falling => bclk_falling, lrclk_out    => i2s_lrclk,
            lrclk_int    => lrclk_int
        );

    -- ----------------------------------------------------------------
    -- I2S receiver
    -- ----------------------------------------------------------------
    u_i2s_rx : entity work.i2s_receiver_slave
        port map (
            sys_clk      => sys_clk,      rst          => rst,
            bclk_rising  => bclk_rising,  lrclk_int    => lrclk_int,
            sdata        => i2s_sdata_in,
            left_sample  => left_sample,  right_sample => right_sample,
            sample_valid => sample_valid, debug_pin    => open
        );

    -- ----------------------------------------------------------------
    -- FFT buffer (left channel only)
    -- ----------------------------------------------------------------
    u_fft_buf : entity work.fft_buffer
        port map (
            sys_clk      => sys_clk,      rst          => rst,
            sample_in    => left_sample,  sample_valid => sample_valid,
            m_tdata      => m_tdata,      m_tvalid     => m_tvalid,
            m_tlast      => m_tlast,      m_tready     => '1'
        );

    -- ----------------------------------------------------------------
    -- Capture RAM - store windowed samples as they stream out
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                cap_ptr    <= 0;
                cap_done   <= '0';
                led_stretch<= 0;
            else
                if m_tvalid = '1' then
                    cap_ram(cap_ptr) <= std_logic_vector(to_unsigned(cap_ptr * 4, 32));
                    if m_tlast = '1' then
                        cap_done    <= '1';
                        cap_ptr     <= 0;
                        led_stretch <= 9999999;
                    else
                        cap_ptr <= cap_ptr + 1;
                    end if;
                end if;
                -- Clear cap_done when SPI arms for next frame
                if arm = '1' then
                    cap_done <= '0';
                end if;
                if led_stretch > 0 then
                    led_stretch <= led_stretch - 1;
                end if;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- SPI section
    -- ----------------------------------------------------------------
    -----------------------------------------------------------
    -- SPI synchronisation section                           --
    -----------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            -- SCLK edge detection --
            sclk_r <= sclk_r(1 downto 0) & spi_sclk;
            if sclk_r(2 downto 1) = "01" then
                sclk_rising <= '1';
                sclk_falling <='0';
            elsif sclk_r(2 downto 1) = "10" then
                sclk_rising <= '0';
                sclk_falling <= '1';
            else
                sclk_rising <= '0';
                sclk_falling <= '0';
            end if;
            -- cs edge detection --
            cs_r(2 downto 0) <= cs_r(1 downto 0)&spi_cs_n;
            cs_active<=not cs_r(1); -- CS is active low
            if cs_r(2 downto 1) = "10" then -- message starts on falling edge
                cs_start <= '1';
                cs_stop <= '0';
            elsif cs_r(2 downto 1) = "01" then -- message stop at rising edge
                cs_start <= '0';
                cs_stop <= '1';
            else
                cs_start <= '0';
                cs_stop <= '0';
            end if;
            -- MOSI edge detection
            mosi_r <= mosi_r(0)&spi_mosi;
            mosi_data <= mosi_r(1);
        end if;
    end process;

    -----------------------------------------------------------
    -- SPI receive and state machine section                --
    -----------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                bitcnt <= 0;
                byte_received <= '0';
                byte_data_received <= "00000000";
                spi_state <= IDLE;
                tx_shift <= (others => '0');
                word_idx <= 0;
                byte_in_word <= 0;
                cur_word <= (others => '0');
                arm <= '0';
            else
                if (cs_active = '0') then
                    bitcnt <= 0;
                else
                    if (sclk_rising = '1') then
                        bitcnt <= bitcnt + 1;
                        byte_data_received <= byte_data_received(6 downto 0)&mosi_data;
                        if bitcnt = 7 then
                            byte_received<='1';
                        else
                            byte_received<='0';
                        end if;
                    end if;
                end if;

                -- Process received byte and update state
                if byte_received = '1' and cs_active = '1' then
                    case byte_data_received is
                        when x"01" =>
                            -- Read command: start transmitting from cap_ram
                            spi_state <= TX_DATA;
                            word_idx <= 0;
                            byte_in_word <= 0;
                            cur_word <= cap_ram(0);
                            tx_shift <= cap_ram(0)(7 downto 0);
                        when x"02" =>
                            -- Arm command: wait for next frame
                            arm <= '1';
                            spi_state <= IDLE;
                        when x"03" =>
                            -- Poll command: return frame ready status
                            if cap_done = '1' then
                                tx_shift <= x"FF";
                            else
                                tx_shift <= x"00";
                            end if;
                            spi_state <= TX_STATUS;
                        when others =>
                            spi_state <= IDLE;
                    end case;
                    byte_received <= '0';  -- Reset byte_received after processing
                end if;

                case spi_state is
                    when IDLE =>
                        arm <= '0';
                        tx_shift <= (others => '0');

                    when TX_STATUS =>
                        if sclk_rising = '1' then
                            spi_miso <= tx_shift(7);
                            if bitcnt = 7 then
                                spi_state <= IDLE;
                                bitcnt <= 0;
                            else
                                tx_shift <= tx_shift(6 downto 0) & '0';
                            end if;
                        end if;

                    when TX_DATA =>
                        if sclk_rising = '1' then
                            spi_miso <= tx_shift(7);  -- Output MSB
                            if bitcnt = 7 then
                                -- Byte complete, move to next byte
                                if byte_in_word = 3 then
                                    -- Word complete
                                    byte_in_word <= 0;
                                    if word_idx = 1023 then
                                        -- All data sent
                                        spi_state <= IDLE;
                                    else
                                        -- Load next word
                                        word_idx <= word_idx + 1;
                                        cur_word <= cap_ram(word_idx + 1);
                                        tx_shift <= cap_ram(word_idx + 1)(7 downto 0);
                                    end if;
                                else
                                    -- Move to next byte in current word
                                    byte_in_word <= byte_in_word + 1;
                                    case byte_in_word is
                                        when 0 => tx_shift <= cur_word(15 downto 8);
                                        when 1 => tx_shift <= cur_word(23 downto 16);
                                        when others => tx_shift <= cur_word(31 downto 24);
                                    end case;
                                end if;
                                bitcnt <= 0;
                            else
                                tx_shift <= tx_shift(6 downto 0) & '0';
                            end if;
                        end if;

                    when others =>
                        spi_state <= IDLE;
                end case;
            end if;
        end if;
    end process;

end rtl;