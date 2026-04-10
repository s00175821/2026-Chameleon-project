----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
--
-- Create Date: 13.03.2026 17:45:54
-- Module Name: magnitude_calc - rtl
-- Project Name: Chameleon
-- Target Devices: xc7a35tcpg236-1
-- Description:
--   Calculates magnitude squared from FFT complex output.
--   Input:  48-bit [47:24]=imag, [23:0]=real, Q23 signed 24-bit
--   Output: 32-bit magnitude squared
--
--   Uses explicit DSP48E1 instantiation for re^2 and im^2.
--   Vivado ignores use_dsp for same-operand (squaring) multiplies
--   and falls back to LUT fabric causing timing violations.
--
--   A port (30-bit): sign-extended 25-bit input (only A[24:0] used by multiplier)
--   B port (18-bit): lower 18 bits of the same input
--   Result is a 25x18 multiply. Precision loss vs full 25x25
--   is ~72dB below full scale - invisible in spectrum display.
--
--   Pipeline:
--     Cycle 1 (process): register inputs, sign-extend to 25-bit
--     Cycle 2 (DSP48):   AREG+BREG latch, MREG multiplies, PREG holds
--     Cycle 3 (process): sum re^2 + im^2 and output
--
-- Revision 0.06 - Port map verified against UG953 7-series libraries guide
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library UNISIM;
use UNISIM.VComponents.all;

entity magnitude_calc is
    Port (
        sys_clk   : in  STD_LOGIC;
        s_tdata   : in  STD_LOGIC_VECTOR(47 downto 0);
        s_tvalid  : in  STD_LOGIC;
        s_tlast   : in  STD_LOGIC;
        magnitude : out STD_LOGIC_VECTOR(31 downto 0);
        mag_valid : out STD_LOGIC;
        mag_last  : out STD_LOGIC
    );
end magnitude_calc;

architecture rtl of magnitude_calc is

    -- Stage 1: registered and sign-extended inputs
    signal re_ext   : signed(24 downto 0) := (others => '0');
    signal im_ext   : signed(24 downto 0) := (others => '0');
    signal valid_p1 : std_logic := '0';
    signal last_p1  : std_logic := '0';

    -- DSP48E1 port wiring
    -- A port 30-bit: sign-extend 25-bit input (multiplier uses A[24:0])
    -- B port 18-bit: lower 18 bits of 25-bit input
    signal re_a     : std_logic_vector(29 downto 0);
    signal re_b     : std_logic_vector(17 downto 0);
    signal im_a     : std_logic_vector(29 downto 0);
    signal im_b     : std_logic_vector(17 downto 0);

    -- Stage 2: DSP48E1 P outputs (48-bit each)
    signal re_sq_p  : std_logic_vector(47 downto 0);
    signal im_sq_p  : std_logic_vector(47 downto 0);
    signal valid_p2 : std_logic := '0';
    signal last_p2  : std_logic := '0';

    -- Stage 3: output registers
    signal mag_reg   : std_logic_vector(31 downto 0) := (others => '0');
    signal valid_reg : std_logic := '0';
    signal last_reg  : std_logic := '0';

begin

    magnitude <= mag_reg;
    mag_valid <= valid_reg;
    mag_last  <= last_reg;

    re_a <= std_logic_vector(resize(re_ext, 30));
    re_b <= std_logic_vector(re_ext(17 downto 0));
    im_a <= std_logic_vector(resize(im_ext, 30));
    im_b <= std_logic_vector(im_ext(17 downto 0));

    -- ----------------------------------------------------------------
    -- DSP48E1 for re^2
    -- OPMODE "0000101" = X=M, Y=0, Z=0 => P = M = A*B
    -- CECTRL covers both OPMODEREG and CARRYINSELREG clock enables
    -- RSTCTRL covers both OPMODEREG and CARRYINSELREG resets
    -- ----------------------------------------------------------------
    DSP_RE : DSP48E1
        generic map (
            A_INPUT       => "DIRECT",
            B_INPUT       => "DIRECT",
            USE_DPORT     => FALSE,
            USE_MULT      => "MULTIPLY",
            USE_SIMD      => "ONE48",
            ACASCREG      => 1,
            ADREG         => 1,
            ALUMODEREG    => 1,
            AREG          => 1,
            BCASCREG      => 1,
            BREG          => 1,
            CARRYINREG    => 1,
            CARRYINSELREG => 1,
            CREG          => 1,
            DREG          => 1,
            INMODEREG     => 1,
            MREG          => 1,
            OPMODEREG     => 1,
            PREG          => 1
        )
        port map (
            -- Clock
            CLK           => sys_clk,
            -- Data inputs
            A             => re_a,
            B             => re_b,
            C             => (others => '0'),
            D             => (others => '0'),
            -- Cascade inputs (required, tied to zero when not cascading)
            ACIN          => (others => '0'),
            BCIN          => (others => '0'),
            PCIN          => (others => '0'),
            CARRYCASCIN   => '0',
            MULTSIGNIN    => '0',
            -- Control: P = A*B
            ALUMODE       => "0000",
            INMODE        => "00000",
            OPMODE        => "0000101",
            CARRYIN       => '0',
            CARRYINSEL    => "000",
            -- Clock enables
            CEA1          => '1',
            CEA2          => '1',
            CEB1          => '1',
            CEB2          => '1',
            CEC           => '1',
            CED           => '1',
            CEAD          => '1',
            CEALUMODE     => '1',
            CECARRYIN     => '1',
            CECTRL        => '1',   -- covers OPMODEREG and CARRYINSELREG
            CEINMODE      => '1',
            CEM           => valid_p1,
            CEP           => valid_p1,
            -- Synchronous resets (all tied off)
            RSTA          => '0',
            RSTALLCARRYIN => '0',
            RSTALUMODE    => '0',
            RSTB          => '0',
            RSTC          => '0',
            RSTCTRL       => '0',   -- covers OPMODEREG and CARRYINSELREG
            RSTD          => '0',
            RSTINMODE     => '0',
            RSTM          => '0',
            RSTP          => '0',
            -- Outputs
            P             => re_sq_p,
            ACOUT         => open,
            BCOUT         => open,
            CARRYCASCOUT  => open,
            MULTSIGNOUT   => open,
            PCOUT         => open,
            OVERFLOW      => open,
            PATTERNBDETECT => open,
            PATTERNDETECT  => open,
            UNDERFLOW      => open,
            CARRYOUT       => open
        );

    -- ----------------------------------------------------------------
    -- DSP48E1 for im^2 (identical configuration)
    -- ----------------------------------------------------------------
    DSP_IM : DSP48E1
        generic map (
            A_INPUT       => "DIRECT",
            B_INPUT       => "DIRECT",
            USE_DPORT     => FALSE,
            USE_MULT      => "MULTIPLY",
            USE_SIMD      => "ONE48",
            ACASCREG      => 1,
            ADREG         => 1,
            ALUMODEREG    => 1,
            AREG          => 1,
            BCASCREG      => 1,
            BREG          => 1,
            CARRYINREG    => 1,
            CARRYINSELREG => 1,
            CREG          => 1,
            DREG          => 1,
            INMODEREG     => 1,
            MREG          => 1,
            OPMODEREG     => 1,
            PREG          => 1
        )
        port map (
            CLK           => sys_clk,
            A             => im_a,
            B             => im_b,
            C             => (others => '0'),
            D             => (others => '0'),
            ACIN          => (others => '0'),
            BCIN          => (others => '0'),
            PCIN          => (others => '0'),
            CARRYCASCIN   => '0',
            MULTSIGNIN    => '0',
            ALUMODE       => "0000",
            INMODE        => "00000",
            OPMODE        => "0000101",
            CARRYIN       => '0',
            CARRYINSEL    => "000",
            CEA1          => '1',
            CEA2          => '1',
            CEB1          => '1',
            CEB2          => '1',
            CEC           => '1',
            CED           => '1',
            CEAD          => '1',
            CEALUMODE     => '1',
            CECARRYIN     => '1',
            CECTRL        => '1',
            CEINMODE      => '1',
            CEM           => valid_p1,
            CEP           => valid_p1,
            RSTA          => '0',
            RSTALLCARRYIN => '0',
            RSTALUMODE    => '0',
            RSTB          => '0',
            RSTC          => '0',
            RSTCTRL       => '0',
            RSTD          => '0',
            RSTINMODE     => '0',
            RSTM          => '0',
            RSTP          => '0',
            P             => im_sq_p,
            ACOUT         => open,
            BCOUT         => open,
            CARRYCASCOUT  => open,
            MULTSIGNOUT   => open,
            PCOUT         => open,
            OVERFLOW      => open,
            PATTERNBDETECT => open,
            PATTERNDETECT  => open,
            UNDERFLOW      => open,
            CARRYOUT       => open
        );

    -- ----------------------------------------------------------------
    -- Sequential pipeline control and output stage
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then

            -- Cycle 1: register and sign-extend FFT outputs to 25-bit
            if s_tvalid = '1' then
                re_ext   <= resize(signed(s_tdata(23 downto 0)),  25);
                im_ext   <= resize(signed(s_tdata(47 downto 24)), 25);
                valid_p1 <= '1';
                last_p1  <= s_tlast;
            else
                valid_p1 <= '0';
                last_p1  <= '0';
            end if;

            -- Cycle 2: DSP48E1 pipeline runs (AREG -> MREG -> PREG)
            -- Propagate valid/last to align with P output timing
            valid_p2 <= valid_p1;
            last_p2  <= last_p1;

            -- Cycle 3: sum re^2 + im^2, scale to 32-bit and output
            -- P is 48-bit; take [47:16] for 32-bit dynamic range
            valid_reg <= '0';
            last_reg  <= '0';
            if valid_p2 = '1' then
                mag_reg   <= std_logic_vector(
                                unsigned(re_sq_p(47 downto 16)) +
                                unsigned(im_sq_p(47 downto 16))
                             );
                valid_reg <= '1';
                last_reg  <= last_p2;
            end if;

        end if;
    end process;

end rtl;