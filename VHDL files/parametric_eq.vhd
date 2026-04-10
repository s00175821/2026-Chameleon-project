----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
--
-- Create Date: 16.03.2026 13:09:08
-- Module Name: parametric_eq - rtl
-- Project Name: Chameleon
-- Description:
--   18-band parametric EQ using a single shared biquad core.
--   Serial processing: one biquad instance processes all bands
--   sequentially, reusing DSP48 blocks for every band.
--
--   Processing order per sample pair (interleaved L/R):
--     band0_L, band0_R, band1_L, band1_R, ... band17_L, band17_R
--   Total: 36 biquad computations x 6 cycles = 216 cycles
--   Budget: 1000 cycles (96MHz / 96kHz). Very comfortable.
--
--   State machine:
--     IDLE        - wait for valid_in
--     PROCESSING  - cycle through all bands, alternating L/R
--     OUTPUT      - assert valid_out for one cycle
--
--   Coefficient storage: 18 x 5 x 32-bit = 2880 bits (RAM)
--   Delay line state:    18 x 4 x 24-bit per channel = two BRAMs
--
--   Coefficient format: Q2.29 signed 32-bit (same as biquad_filter)
--   Audio format:       Q23 signed 24-bit
--
-- Revision:
-- Revision 0.01 - File Created (parallel 8-band)
-- Revision 0.02 - Rewritten for serial 18-band processing
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity parametric_eq is
    generic (
        NUM_BANDS : integer := 18
    );
    Port (
        sys_clk     : in  std_logic;
        rst         : in  std_logic;

        -- Global bypass: when high, input passes straight to output
        bypass_all  : in  std_logic;

        -- Coefficient write port (from SPI handler)
        coeff_band  : in  integer range 0 to 17;
        coeff_sel   : in  integer range 0 to 4;  -- 0=b0,1=b1,2=b2,3=a1,4=a2
        coeff_val   : in  signed(31 downto 0);
        coeff_we    : in  std_logic;

        -- Per-band bypass write port (from SPI handler)
        bypass_band : in  integer range 0 to 17;
        bypass_val  : in  std_logic;
        bypass_we   : in  std_logic;

        -- Audio (Q23 stereo)
        left_in     : in  std_logic_vector(23 downto 0);
        right_in    : in  std_logic_vector(23 downto 0);
        valid_in    : in  std_logic;

        left_out    : out std_logic_vector(23 downto 0);
        right_out   : out std_logic_vector(23 downto 0);
        valid_out   : out std_logic
    );
end parametric_eq;

architecture rtl of parametric_eq is

    -- ----------------------------------------------------------------
    -- Coefficient RAM: 18 bands x 5 coefficients
    -- ----------------------------------------------------------------
    constant COEFF_UNITY : signed(31 downto 0) := to_signed(536870912, 32); -- 1.0 Q2.29
    constant COEFF_ZERO  : signed(31 downto 0) := to_signed(0, 32);

    type coeff_array_t is array(0 to NUM_BANDS-1) of signed(31 downto 0);
    signal coeff_b0 : coeff_array_t := (others => COEFF_UNITY);
    signal coeff_b1 : coeff_array_t := (others => COEFF_ZERO);
    signal coeff_b2 : coeff_array_t := (others => COEFF_ZERO);
    signal coeff_a1 : coeff_array_t := (others => COEFF_ZERO);
    signal coeff_a2 : coeff_array_t := (others => COEFF_ZERO);

    -- Per-band bypass flags
    signal band_bypass : std_logic_vector(NUM_BANDS-1 downto 0) := (others => '0');

    -- ----------------------------------------------------------------
    -- Delay line state: x[n-1], x[n-2], y[n-1], y[n-2] per band per channel
    -- Separate arrays for left and right
    -- ----------------------------------------------------------------
    type state_array_t is array(0 to NUM_BANDS-1) of signed(23 downto 0);
    signal lx1 : state_array_t := (others => (others => '0'));
    signal lx2 : state_array_t := (others => (others => '0'));
    signal ly1 : state_array_t := (others => (others => '0'));
    signal ly2 : state_array_t := (others => (others => '0'));
    signal rx1 : state_array_t := (others => (others => '0'));
    signal rx2 : state_array_t := (others => (others => '0'));
    signal ry1 : state_array_t := (others => (others => '0'));
    signal ry2 : state_array_t := (others => (others => '0'));

    -- ----------------------------------------------------------------
    -- Single shared biquad pipeline signals
    -- Direct Form I, Q2.29 coefficients, Q23 audio
    -- ----------------------------------------------------------------

    -- DSP48 inference attributes
    attribute use_dsp : string;

    -- Multiply stage inputs (registered)
    signal p_x0   : signed(23 downto 0) := (others => '0'); -- current input
    signal p_x1   : signed(23 downto 0) := (others => '0'); -- x[n-1]
    signal p_x2   : signed(23 downto 0) := (others => '0'); -- x[n-2]
    signal p_y1   : signed(23 downto 0) := (others => '0'); -- y[n-1]
    signal p_y2   : signed(23 downto 0) := (others => '0'); -- y[n-2]
    signal p_b0   : signed(31 downto 0) := COEFF_UNITY;
    signal p_b1   : signed(31 downto 0) := COEFF_ZERO;
    signal p_b2   : signed(31 downto 0) := COEFF_ZERO;
    signal p_a1   : signed(31 downto 0) := COEFF_ZERO;
    signal p_a2   : signed(31 downto 0) := COEFF_ZERO;
    signal p_bypass : std_logic := '0';
    signal p_x0_passthrough : signed(23 downto 0) := (others => '0');

    -- Products: Q23 * Q2.29 = 56-bit result in 64-bit container
    signal prod_b0x0 : signed(63 downto 0) := (others => '0');
    signal prod_b1x1 : signed(63 downto 0) := (others => '0');
    signal prod_b2x2 : signed(63 downto 0) := (others => '0');
    signal prod_a1y1 : signed(63 downto 0) := (others => '0');
    signal prod_a2y2 : signed(63 downto 0) := (others => '0');

    attribute use_dsp of prod_b0x0 : signal is "yes";
    attribute use_dsp of prod_b1x1 : signal is "yes";
    attribute use_dsp of prod_b2x2 : signal is "yes";
    attribute use_dsp of prod_a1y1 : signal is "yes";
    attribute use_dsp of prod_a2y2 : signal is "yes";

    -- Accumulator and result
    signal accum     : signed(63 downto 0) := (others => '0');
    signal bq_result : signed(23 downto 0) := (others => '0');

    -- Bypass passthrough aligned to pipeline output
    signal bypass_pipe : std_logic_vector(3 downto 0) := (others => '0');
    signal x0_pipe     : signed(23 downto 0) := (others => '0'); -- not needed separately

    -- Saturation function: clamp Q25.52 result to Q23
    function saturate(x : signed(55 downto 0)) return signed is
        constant MAX_POS : signed(23 downto 0) := to_signed( 8388607, 24);
        constant MAX_NEG : signed(23 downto 0) := to_signed(-8388608, 24);
    begin
        if x(55 downto 52) = "0000" or x(55 downto 52) = "1111" then
            return x(52 downto 29);
        elsif x(55) = '0' then
            return MAX_POS;
        else
            return MAX_NEG;
        end if;
    end function;

    -- ----------------------------------------------------------------
    -- State machine
    -- ----------------------------------------------------------------
    type eq_state_t is (IDLE, LOAD, MULTIPLY, ACCUMULATE, RESULT, OUTPUT_SAMPLE);
    signal eq_state : eq_state_t := IDLE;

    -- Current band and channel being processed
    signal cur_band    : integer range 0 to NUM_BANDS-1 := 0;
    signal cur_channel : std_logic := '0'; -- 0=left, 1=right

    -- Audio samples latched at start of processing
    signal left_latch  : signed(23 downto 0) := (others => '0');
    signal right_latch : signed(23 downto 0) := (others => '0');

    -- Intermediate results travelling through the band chain
    signal left_current  : signed(23 downto 0) := (others => '0');
    signal right_current : signed(23 downto 0) := (others => '0');

    -- Output registers
    signal left_out_reg  : std_logic_vector(23 downto 0) := (others => '0');
    signal right_out_reg : std_logic_vector(23 downto 0) := (others => '0');
    signal valid_out_reg : std_logic := '0';

    -- Pipeline stage tracking: which band/channel result is coming out
    -- Pipeline is 4 stages deep (LOAD->MULTIPLY->ACCUMULATE->RESULT)
    type pipe_band_t is array(0 to 3) of integer range 0 to NUM_BANDS-1;
    type pipe_ch_t   is array(0 to 3) of std_logic;
    signal pipe_band    : pipe_band_t := (others => 0);
    signal pipe_channel : pipe_ch_t   := (others => '0');
    signal pipe_valid   : std_logic_vector(3 downto 0) := (others => '0');
    signal pipe_bypass  : std_logic_vector(3 downto 0) := (others => '0');
    signal pipe_x0      : signed(23 downto 0) := (others => '0');

    -- Writeback: result from pipeline going into state arrays
    signal wb_valid   : std_logic := '0';
    signal wb_band    : integer range 0 to NUM_BANDS-1 := 0;
    signal wb_channel : std_logic := '0';
    signal wb_result  : signed(23 downto 0) := (others => '0');
    signal wb_x0      : signed(23 downto 0) := (others => '0');
    signal wb_bypass  : std_logic := '0';

    -- Per-band per-channel current input (used in writeback for state update)
    signal load_input : signed(23 downto 0) := (others => '0');

begin

    left_out  <= left_out_reg;
    right_out <= right_out_reg;
    valid_out <= valid_out_reg;

    -- ----------------------------------------------------------------
    -- Coefficient and bypass write process
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if coeff_we = '1' then
                case coeff_sel is
                    when 0 => coeff_b0(coeff_band) <= coeff_val;
                    when 1 => coeff_b1(coeff_band) <= coeff_val;
                    when 2 => coeff_b2(coeff_band) <= coeff_val;
                    when 3 => coeff_a1(coeff_band) <= coeff_val;
                    when 4 => coeff_a2(coeff_band) <= coeff_val;
                    when others => null;
                end case;
            end if;
            if bypass_we = '1' then
                band_bypass(bypass_band) <= bypass_val;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Main serial processing state machine + biquad pipeline
    -- ----------------------------------------------------------------
    process(sys_clk)
        variable v_input : signed(23 downto 0);
    begin
        if rising_edge(sys_clk) then
            valid_out_reg <= '0';
            wb_valid      <= '0';

            if rst = '1' then
                eq_state     <= IDLE;
                cur_band     <= 0;
                cur_channel  <= '0';
                pipe_valid   <= (others => '0');
                left_out_reg <= (others => '0');
                right_out_reg<= (others => '0');

            else
                -- --------------------------------------------------------
                -- Pipeline stage 1: LOAD coefficients and state
                -- Triggered by state machine asserting a band/channel
                -- --------------------------------------------------------
                -- (done inline in state machine below)

                -- --------------------------------------------------------
                -- Pipeline stage 2: MULTIPLY (one cycle after load)
                -- --------------------------------------------------------
                prod_b0x0 <= p_b0 * resize(p_x0, 32);
                prod_b1x1 <= p_b1 * resize(p_x1, 32);
                prod_b2x2 <= p_b2 * resize(p_x2, 32);
                prod_a1y1 <= p_a1 * resize(p_y1, 32);
                prod_a2y2 <= p_a2 * resize(p_y2, 32);

                -- Propagate pipeline tracking
                pipe_band(1)    <= pipe_band(0);
                pipe_channel(1) <= pipe_channel(0);
                pipe_valid(1)   <= pipe_valid(0);
                pipe_bypass(1)  <= pipe_bypass(0);

                -- --------------------------------------------------------
                -- Pipeline stage 3: ACCUMULATE
                -- --------------------------------------------------------
                accum <= prod_b0x0 + prod_b1x1 + prod_b2x2
                         - prod_a1y1 - prod_a2y2;

                pipe_band(2)    <= pipe_band(1);
                pipe_channel(2) <= pipe_channel(1);
                pipe_valid(2)   <= pipe_valid(1);
                pipe_bypass(2)  <= pipe_bypass(1);

                -- --------------------------------------------------------
                -- Pipeline stage 4: RESULT - saturate to Q23
                -- --------------------------------------------------------
                if pipe_bypass(2) = '1' or bypass_all = '1' then
                    -- Bypass: pass input through unchanged
                    -- pipe_x0 carries the original input aligned to this stage
                    bq_result <= pipe_x0;
                else
                    bq_result <= saturate(accum(55 downto 0));
                end if;

                pipe_band(3)    <= pipe_band(2);
                pipe_channel(3) <= pipe_channel(2);
                pipe_valid(3)   <= pipe_valid(2);
                pipe_bypass(3)  <= pipe_bypass(2);

                -- --------------------------------------------------------
                -- Writeback: take result from pipeline stage 4
                -- Update delay line state and current sample value
                -- --------------------------------------------------------
                wb_valid   <= pipe_valid(3);
                wb_band    <= pipe_band(3);
                wb_channel <= pipe_channel(3);
                wb_result  <= bq_result;
                wb_bypass  <= pipe_bypass(3);

                if pipe_valid(3) = '1' then
                    if pipe_channel(3) = '0' then
                        -- Left channel result
                        -- Update left delay line for this band
                        lx2(pipe_band(3)) <= lx1(pipe_band(3));
                        lx1(pipe_band(3)) <= p_x0_passthrough; -- original input to this band
                        ly2(pipe_band(3)) <= ly1(pipe_band(3));
                        ly1(pipe_band(3)) <= bq_result;
                        -- Feed result to next band's left input
                        left_current <= bq_result;
                    else
                        -- Right channel result
                        rx2(pipe_band(3)) <= rx1(pipe_band(3));
                        rx1(pipe_band(3)) <= p_x0_passthrough;
                        ry2(pipe_band(3)) <= ry1(pipe_band(3));
                        ry1(pipe_band(3)) <= bq_result;
                        right_current <= bq_result;
                    end if;
                end if;

                -- --------------------------------------------------------
                -- State machine: schedule band/channel computations
                -- --------------------------------------------------------
                case eq_state is

                    when IDLE =>
                        if valid_in = '1' then
                            -- Latch input samples
                            left_latch    <= signed(left_in);
                            right_latch   <= signed(right_in);
                            left_current  <= signed(left_in);
                            right_current <= signed(right_in);
                            cur_band      <= 0;
                            cur_channel   <= '0'; -- start with left
                            eq_state      <= LOAD;
                        end if;

                    when LOAD =>
                        -- Load coefficients and state for cur_band/cur_channel
                        -- into pipeline stage 1 registers
                        if cur_channel = '0' then
                            v_input := left_current;
                            p_x1    <= lx1(cur_band);
                            p_x2    <= lx2(cur_band);
                            p_y1    <= ly1(cur_band);
                            p_y2    <= ly2(cur_band);
                        else
                            v_input := right_current;
                            p_x1    <= rx1(cur_band);
                            p_x2    <= rx2(cur_band);
                            p_y1    <= ry1(cur_band);
                            p_y2    <= ry2(cur_band);
                        end if;

                        p_x0             <= v_input;
                        p_x0_passthrough <= v_input;
                        p_b0             <= coeff_b0(cur_band);
                        p_b1             <= coeff_b1(cur_band);
                        p_b2             <= coeff_b2(cur_band);
                        p_a1             <= coeff_a1(cur_band);
                        p_a2             <= coeff_a2(cur_band);
                        p_bypass         <= band_bypass(cur_band);
                        pipe_x0          <= v_input;

                        -- Tag this pipeline slot
                        pipe_band(0)    <= cur_band;
                        pipe_channel(0) <= cur_channel;
                        pipe_valid(0)   <= '1';
                        pipe_bypass(0)  <= band_bypass(cur_band);

                        -- Advance to next band/channel
                        if cur_channel = '0' then
                            -- Move to right channel of same band
                            cur_channel <= '1';
                        else
                            -- Move to left channel of next band
                            cur_channel <= '0';
                            if cur_band = NUM_BANDS-1 then
                                -- All bands scheduled, wait for pipeline to drain
                                eq_state <= OUTPUT_SAMPLE;
                            else
                                cur_band <= cur_band + 1;
                            end if;
                        end if;

                    when OUTPUT_SAMPLE =>
                        -- Wait for last pipeline result (band17_R) to emerge
                        -- Pipeline is 4 stages deep so we need to wait 4 cycles
                        -- after last LOAD before results are valid.
                        -- We use pipe_valid(3) for band17/right as the trigger.
                        pipe_valid(0) <= '0'; -- stop feeding pipeline
                        if wb_valid = '1' and
                           wb_band = NUM_BANDS-1 and
                           wb_channel = '1' then
                            -- Last result is in right_current
                            left_out_reg  <= std_logic_vector(left_current);
                            right_out_reg <= std_logic_vector(right_current);
                            valid_out_reg <= '1';
                            eq_state      <= IDLE;
                        end if;

                    when others =>
                        eq_state <= IDLE;

                end case;
            end if;
        end if;
    end process;

end rtl;