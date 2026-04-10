----------------------------------------------------------------------------------
-- Company: Vega
-- Engineer: Grzegorz Chojnacki
-- 
-- Create Date: 15.03.2026 09:54:22
-- Design Name: Midnight mode
-- Module Name: midnight_mode - rtl
-- Project Name: Chameleon
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- Executes Midnight Mode filter
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

-- Midnight Mode Dynamic Range Compressor
--
-- Fixed parameters (tuned for movie/TV dialogue enhancement):
--   Threshold high : -18 dBFS  compression above this level
--   Threshold low  : -45 dBFS  expansion below this level
--   Ratio high     : 4:1       downward compression
--   Ratio low      : 2:1       upward expansion
--   Attack         : 10ms      fast gain reduction on transients
--   Release        : 200ms     slow gain recovery
--   Peak ceiling   : -3 dBFS   hard limiter
--   Makeup gain    : +6dB
--
-- All multiplications use DSP48 blocks (use_dsp attribute)
-- RMS sqrt uses a pre-computed 4096-entry Block RAM LUT
-- No runtime parameter adjustment - values are synthesis constants

entity midnight_mode is
    Port (
        sys_clk         : in std_logic;
        rst             : in std_logic;
        enable          : in std_logic;
        
        -- Input samples from i2s_receiver_slave (Q23, 24-bit signed)
        left_in         : in std_logic_vector(23 downto 0);
        right_in        : in std_logic_vector(23 downto 0);
        sample_valid_in : in std_logic;
        
        -- Output samples to I2S transmiter / PCM3168A (Q23)
        left_out        : out std_logic_vector(23 downto 0);
        right_out       : out std_logic_vector(23 downto 0);
        sample_valid_out: out std_logic;
        
        -- status for debug / VU meter
        gain_db_out     : out std_logic_vector(7 downto 0);
        peak_limiting   : out std_logic
     );
end midnight_mode;

architecture rtl of midnight_mode is
    -- ----------------------------------------------------------------
    -- Constants: all thresholds in Q23 (full scale = 2^23 = 8388608)
    -- ----------------------------------------------------------------
    -- -18 dBFS = 8388608 * 10^(-18/20) = 1056063
    constant THRESH_HIGH        : signed(23 downto 0) := to_signed(1056063, 24);
    -- -45 dBFS = 8388608 * 10^(-45/20) = 47173
    constant THRESH_LOW         : signed(23 downto 0) := to_signed(47173, 24);
    -- -3 dBFS = 8388608 * 10^(-3/20) = 5938680
    constant PEAK_CEILING       : signed(23 downto 0) := to_signed(5938680, 24);
    
    -- Gain constants in Q15 (32768 = unity = 0dB)
    -- +6dB makeup = 1.995 * 32768 = 65372
    -- Clamped to 16-bit max 65535 - use 65372 for +6dB exactly
    constant GAIN_UNITY         : unsigned(15 downto 0) := to_unsigned(32768, 16);
    constant GAIN_MAKEUP        : unsigned(15 downto 0) := to_unsigned(65372, 16);
    constant GAIN_MAX           : unsigned(15 downto 0) := to_unsigned(65372, 16);
    -- -36 dB minimum = 0.016 * 32768 = 524
    constant GAIN_MIN           : unsigned(15 downto 0) := to_unsigned(524, 16);
    
    -- Envelope follower coefficients in Q15
    -- Attack  10ms  at 96kHz: coeff = 1 - exp(-1/960)   = 0.001041 * 32768 = 34
    -- Release 200ms at 96kHz: coeff = 1 - exp(-1/19200) = 0.0000521 * 32768 = 2 
    constant ATTACK_COEFF       : unsigned(15 downto 0) := TO_UNSIGNED(34, 16);
    constant RELEASE_COEFF      : unsigned(15 downto 0) := TO_UNSIGNED(2, 16);
    
    -- RMS window: 1024 samples (~10.7ms at 96kHz)
    constant RMS_WINDOW_LOG2    : integer := 10;
    constant RMS_WINDOW         : integer := 1024;
    
    constant SMOOTH_GAIN_INIT : unsigned(31 downto 0) :=
        GAIN_MAKEUP & to_unsigned(0, 16);
    
    -- ----------------------------------------------------------------
    -- DSP48 inference attributes
    -- Applied to every product signal to force DSP48 usage
    -- ----------------------------------------------------------------
    attribute use_dsp : string;
    
    -- ----------------------------------------------------------------
    -- Stage 1: RMS detector signals
    -- ----------------------------------------------------------------
    signal mono_in              : signed(23 downto 0) := (others => '0');
    signal mono_sq              : signed(47 downto 0) := (others => '0');
    signal rms_accum            : unsigned(47 downto 0) := (others => '0');
    signal rms_count            : integer range 0 to RMS_WINDOW-1 :=0;
    signal rms_scaled           : unsigned(23 downto 0) := (others => '0');
    signal rms_level            : unsigned(23 downto 0) := (others => '0');
    signal rms_valid            : std_logic := '0';
    
    -- Force DSP48 for the squaring operation
    attribute use_dsp of mono_sq : signal is "yes";
    
    -- ----------------------------------------------------------------
    -- sqrt LUT (Block RAM)
    -- Address: upper 12 bits of rms_scaled (4096 entries)
    -- Output:  12-bit sqrt approximation, scaled to match Q23 input
    -- The LUT maps normalised RMS energy to normalised RMS amplitude.
    -- After lookup, scale output back to Q23 range.
    -- ----------------------------------------------------------------
    type sqrt_lut_t is array (0 to 4095) of unsigned(11 downto 0);
    constant SQRT_LUT : sqrt_lut_t := (
    to_unsigned(0, 12), to_unsigned(64, 12), to_unsigned(90, 12), to_unsigned(111, 12), to_unsigned(128, 12), to_unsigned(143, 12), to_unsigned(157, 12), to_unsigned(169, 12),
    to_unsigned(181, 12), to_unsigned(192, 12), to_unsigned(202, 12), to_unsigned(212, 12), to_unsigned(222, 12), to_unsigned(231, 12), to_unsigned(239, 12), to_unsigned(248, 12),
    to_unsigned(256, 12), to_unsigned(264, 12), to_unsigned(271, 12), to_unsigned(279, 12), to_unsigned(286, 12), to_unsigned(293, 12), to_unsigned(300, 12), to_unsigned(307, 12),
    to_unsigned(313, 12), to_unsigned(320, 12), to_unsigned(326, 12), to_unsigned(332, 12), to_unsigned(339, 12), to_unsigned(345, 12), to_unsigned(350, 12), to_unsigned(356, 12),
    to_unsigned(362, 12), to_unsigned(368, 12), to_unsigned(373, 12), to_unsigned(379, 12), to_unsigned(384, 12), to_unsigned(389, 12), to_unsigned(394, 12), to_unsigned(400, 12),
    to_unsigned(405, 12), to_unsigned(410, 12), to_unsigned(415, 12), to_unsigned(420, 12), to_unsigned(424, 12), to_unsigned(429, 12), to_unsigned(434, 12), to_unsigned(439, 12),
    to_unsigned(443, 12), to_unsigned(448, 12), to_unsigned(452, 12), to_unsigned(457, 12), to_unsigned(461, 12), to_unsigned(466, 12), to_unsigned(470, 12), to_unsigned(475, 12),
    to_unsigned(479, 12), to_unsigned(483, 12), to_unsigned(487, 12), to_unsigned(491, 12), to_unsigned(496, 12), to_unsigned(500, 12), to_unsigned(504, 12), to_unsigned(508, 12),
    to_unsigned(512, 12), to_unsigned(516, 12), to_unsigned(520, 12), to_unsigned(524, 12), to_unsigned(528, 12), to_unsigned(531, 12), to_unsigned(535, 12), to_unsigned(539, 12),
    to_unsigned(543, 12), to_unsigned(547, 12), to_unsigned(550, 12), to_unsigned(554, 12), to_unsigned(558, 12), to_unsigned(561, 12), to_unsigned(565, 12), to_unsigned(569, 12),
    to_unsigned(572, 12), to_unsigned(576, 12), to_unsigned(579, 12), to_unsigned(583, 12), to_unsigned(586, 12), to_unsigned(590, 12), to_unsigned(593, 12), to_unsigned(597, 12),
    to_unsigned(600, 12), to_unsigned(604, 12), to_unsigned(607, 12), to_unsigned(610, 12), to_unsigned(614, 12), to_unsigned(617, 12), to_unsigned(620, 12), to_unsigned(624, 12),
    to_unsigned(627, 12), to_unsigned(630, 12), to_unsigned(633, 12), to_unsigned(637, 12), to_unsigned(640, 12), to_unsigned(643, 12), to_unsigned(646, 12), to_unsigned(649, 12),
    to_unsigned(653, 12), to_unsigned(656, 12), to_unsigned(659, 12), to_unsigned(662, 12), to_unsigned(665, 12), to_unsigned(668, 12), to_unsigned(671, 12), to_unsigned(674, 12),
    to_unsigned(677, 12), to_unsigned(680, 12), to_unsigned(683, 12), to_unsigned(686, 12), to_unsigned(689, 12), to_unsigned(692, 12), to_unsigned(695, 12), to_unsigned(698, 12),
    to_unsigned(701, 12), to_unsigned(704, 12), to_unsigned(707, 12), to_unsigned(710, 12), to_unsigned(712, 12), to_unsigned(715, 12), to_unsigned(718, 12), to_unsigned(721, 12),
    to_unsigned(724, 12), to_unsigned(727, 12), to_unsigned(730, 12), to_unsigned(732, 12), to_unsigned(735, 12), to_unsigned(738, 12), to_unsigned(741, 12), to_unsigned(743, 12),
    to_unsigned(746, 12), to_unsigned(749, 12), to_unsigned(752, 12), to_unsigned(754, 12), to_unsigned(757, 12), to_unsigned(760, 12), to_unsigned(762, 12), to_unsigned(765, 12),
    to_unsigned(768, 12), to_unsigned(770, 12), to_unsigned(773, 12), to_unsigned(776, 12), to_unsigned(778, 12), to_unsigned(781, 12), to_unsigned(784, 12), to_unsigned(786, 12),
    to_unsigned(789, 12), to_unsigned(791, 12), to_unsigned(794, 12), to_unsigned(797, 12), to_unsigned(799, 12), to_unsigned(802, 12), to_unsigned(804, 12), to_unsigned(807, 12),
    to_unsigned(809, 12), to_unsigned(812, 12), to_unsigned(814, 12), to_unsigned(817, 12), to_unsigned(819, 12), to_unsigned(822, 12), to_unsigned(824, 12), to_unsigned(827, 12),
    to_unsigned(829, 12), to_unsigned(832, 12), to_unsigned(834, 12), to_unsigned(837, 12), to_unsigned(839, 12), to_unsigned(842, 12), to_unsigned(844, 12), to_unsigned(846, 12),
    to_unsigned(849, 12), to_unsigned(851, 12), to_unsigned(854, 12), to_unsigned(856, 12), to_unsigned(858, 12), to_unsigned(861, 12), to_unsigned(863, 12), to_unsigned(866, 12),
    to_unsigned(868, 12), to_unsigned(870, 12), to_unsigned(873, 12), to_unsigned(875, 12), to_unsigned(877, 12), to_unsigned(880, 12), to_unsigned(882, 12), to_unsigned(884, 12),
    to_unsigned(887, 12), to_unsigned(889, 12), to_unsigned(891, 12), to_unsigned(893, 12), to_unsigned(896, 12), to_unsigned(898, 12), to_unsigned(900, 12), to_unsigned(903, 12),
    to_unsigned(905, 12), to_unsigned(907, 12), to_unsigned(909, 12), to_unsigned(912, 12), to_unsigned(914, 12), to_unsigned(916, 12), to_unsigned(918, 12), to_unsigned(921, 12),
    to_unsigned(923, 12), to_unsigned(925, 12), to_unsigned(927, 12), to_unsigned(929, 12), to_unsigned(932, 12), to_unsigned(934, 12), to_unsigned(936, 12), to_unsigned(938, 12),
    to_unsigned(940, 12), to_unsigned(943, 12), to_unsigned(945, 12), to_unsigned(947, 12), to_unsigned(949, 12), to_unsigned(951, 12), to_unsigned(953, 12), to_unsigned(955, 12),
    to_unsigned(958, 12), to_unsigned(960, 12), to_unsigned(962, 12), to_unsigned(964, 12), to_unsigned(966, 12), to_unsigned(968, 12), to_unsigned(970, 12), to_unsigned(972, 12),
    to_unsigned(975, 12), to_unsigned(977, 12), to_unsigned(979, 12), to_unsigned(981, 12), to_unsigned(983, 12), to_unsigned(985, 12), to_unsigned(987, 12), to_unsigned(989, 12),
    to_unsigned(991, 12), to_unsigned(993, 12), to_unsigned(995, 12), to_unsigned(997, 12), to_unsigned(999, 12), to_unsigned(1002, 12), to_unsigned(1004, 12), to_unsigned(1006, 12),
    to_unsigned(1008, 12), to_unsigned(1010, 12), to_unsigned(1012, 12), to_unsigned(1014, 12), to_unsigned(1016, 12), to_unsigned(1018, 12), to_unsigned(1020, 12), to_unsigned(1022, 12),
    to_unsigned(1024, 12), to_unsigned(1026, 12), to_unsigned(1028, 12), to_unsigned(1030, 12), to_unsigned(1032, 12), to_unsigned(1034, 12), to_unsigned(1036, 12), to_unsigned(1038, 12),
    to_unsigned(1040, 12), to_unsigned(1042, 12), to_unsigned(1044, 12), to_unsigned(1046, 12), to_unsigned(1047, 12), to_unsigned(1049, 12), to_unsigned(1051, 12), to_unsigned(1053, 12),
    to_unsigned(1055, 12), to_unsigned(1057, 12), to_unsigned(1059, 12), to_unsigned(1061, 12), to_unsigned(1063, 12), to_unsigned(1065, 12), to_unsigned(1067, 12), to_unsigned(1069, 12),
    to_unsigned(1071, 12), to_unsigned(1073, 12), to_unsigned(1074, 12), to_unsigned(1076, 12), to_unsigned(1078, 12), to_unsigned(1080, 12), to_unsigned(1082, 12), to_unsigned(1084, 12),
    to_unsigned(1086, 12), to_unsigned(1088, 12), to_unsigned(1090, 12), to_unsigned(1091, 12), to_unsigned(1093, 12), to_unsigned(1095, 12), to_unsigned(1097, 12), to_unsigned(1099, 12),
    to_unsigned(1101, 12), to_unsigned(1103, 12), to_unsigned(1105, 12), to_unsigned(1106, 12), to_unsigned(1108, 12), to_unsigned(1110, 12), to_unsigned(1112, 12), to_unsigned(1114, 12),
    to_unsigned(1116, 12), to_unsigned(1117, 12), to_unsigned(1119, 12), to_unsigned(1121, 12), to_unsigned(1123, 12), to_unsigned(1125, 12), to_unsigned(1127, 12), to_unsigned(1128, 12),
    to_unsigned(1130, 12), to_unsigned(1132, 12), to_unsigned(1134, 12), to_unsigned(1136, 12), to_unsigned(1137, 12), to_unsigned(1139, 12), to_unsigned(1141, 12), to_unsigned(1143, 12),
    to_unsigned(1145, 12), to_unsigned(1146, 12), to_unsigned(1148, 12), to_unsigned(1150, 12), to_unsigned(1152, 12), to_unsigned(1153, 12), to_unsigned(1155, 12), to_unsigned(1157, 12),
    to_unsigned(1159, 12), to_unsigned(1161, 12), to_unsigned(1162, 12), to_unsigned(1164, 12), to_unsigned(1166, 12), to_unsigned(1168, 12), to_unsigned(1169, 12), to_unsigned(1171, 12),
    to_unsigned(1173, 12), to_unsigned(1175, 12), to_unsigned(1176, 12), to_unsigned(1178, 12), to_unsigned(1180, 12), to_unsigned(1182, 12), to_unsigned(1183, 12), to_unsigned(1185, 12),
    to_unsigned(1187, 12), to_unsigned(1188, 12), to_unsigned(1190, 12), to_unsigned(1192, 12), to_unsigned(1194, 12), to_unsigned(1195, 12), to_unsigned(1197, 12), to_unsigned(1199, 12),
    to_unsigned(1200, 12), to_unsigned(1202, 12), to_unsigned(1204, 12), to_unsigned(1206, 12), to_unsigned(1207, 12), to_unsigned(1209, 12), to_unsigned(1211, 12), to_unsigned(1212, 12),
    to_unsigned(1214, 12), to_unsigned(1216, 12), to_unsigned(1217, 12), to_unsigned(1219, 12), to_unsigned(1221, 12), to_unsigned(1222, 12), to_unsigned(1224, 12), to_unsigned(1226, 12),
    to_unsigned(1227, 12), to_unsigned(1229, 12), to_unsigned(1231, 12), to_unsigned(1232, 12), to_unsigned(1234, 12), to_unsigned(1236, 12), to_unsigned(1237, 12), to_unsigned(1239, 12),
    to_unsigned(1241, 12), to_unsigned(1242, 12), to_unsigned(1244, 12), to_unsigned(1246, 12), to_unsigned(1247, 12), to_unsigned(1249, 12), to_unsigned(1251, 12), to_unsigned(1252, 12),
    to_unsigned(1254, 12), to_unsigned(1255, 12), to_unsigned(1257, 12), to_unsigned(1259, 12), to_unsigned(1260, 12), to_unsigned(1262, 12), to_unsigned(1264, 12), to_unsigned(1265, 12),
    to_unsigned(1267, 12), to_unsigned(1268, 12), to_unsigned(1270, 12), to_unsigned(1272, 12), to_unsigned(1273, 12), to_unsigned(1275, 12), to_unsigned(1276, 12), to_unsigned(1278, 12),
    to_unsigned(1280, 12), to_unsigned(1281, 12), to_unsigned(1283, 12), to_unsigned(1284, 12), to_unsigned(1286, 12), to_unsigned(1288, 12), to_unsigned(1289, 12), to_unsigned(1291, 12),
    to_unsigned(1292, 12), to_unsigned(1294, 12), to_unsigned(1296, 12), to_unsigned(1297, 12), to_unsigned(1299, 12), to_unsigned(1300, 12), to_unsigned(1302, 12), to_unsigned(1303, 12),
    to_unsigned(1305, 12), to_unsigned(1307, 12), to_unsigned(1308, 12), to_unsigned(1310, 12), to_unsigned(1311, 12), to_unsigned(1313, 12), to_unsigned(1314, 12), to_unsigned(1316, 12),
    to_unsigned(1318, 12), to_unsigned(1319, 12), to_unsigned(1321, 12), to_unsigned(1322, 12), to_unsigned(1324, 12), to_unsigned(1325, 12), to_unsigned(1327, 12), to_unsigned(1328, 12),
    to_unsigned(1330, 12), to_unsigned(1331, 12), to_unsigned(1333, 12), to_unsigned(1334, 12), to_unsigned(1336, 12), to_unsigned(1338, 12), to_unsigned(1339, 12), to_unsigned(1341, 12),
    to_unsigned(1342, 12), to_unsigned(1344, 12), to_unsigned(1345, 12), to_unsigned(1347, 12), to_unsigned(1348, 12), to_unsigned(1350, 12), to_unsigned(1351, 12), to_unsigned(1353, 12),
    to_unsigned(1354, 12), to_unsigned(1356, 12), to_unsigned(1357, 12), to_unsigned(1359, 12), to_unsigned(1360, 12), to_unsigned(1362, 12), to_unsigned(1363, 12), to_unsigned(1365, 12),
    to_unsigned(1366, 12), to_unsigned(1368, 12), to_unsigned(1369, 12), to_unsigned(1371, 12), to_unsigned(1372, 12), to_unsigned(1374, 12), to_unsigned(1375, 12), to_unsigned(1377, 12),
    to_unsigned(1378, 12), to_unsigned(1380, 12), to_unsigned(1381, 12), to_unsigned(1383, 12), to_unsigned(1384, 12), to_unsigned(1386, 12), to_unsigned(1387, 12), to_unsigned(1389, 12),
    to_unsigned(1390, 12), to_unsigned(1392, 12), to_unsigned(1393, 12), to_unsigned(1395, 12), to_unsigned(1396, 12), to_unsigned(1397, 12), to_unsigned(1399, 12), to_unsigned(1400, 12),
    to_unsigned(1402, 12), to_unsigned(1403, 12), to_unsigned(1405, 12), to_unsigned(1406, 12), to_unsigned(1408, 12), to_unsigned(1409, 12), to_unsigned(1411, 12), to_unsigned(1412, 12),
    to_unsigned(1413, 12), to_unsigned(1415, 12), to_unsigned(1416, 12), to_unsigned(1418, 12), to_unsigned(1419, 12), to_unsigned(1421, 12), to_unsigned(1422, 12), to_unsigned(1424, 12),
    to_unsigned(1425, 12), to_unsigned(1426, 12), to_unsigned(1428, 12), to_unsigned(1429, 12), to_unsigned(1431, 12), to_unsigned(1432, 12), to_unsigned(1434, 12), to_unsigned(1435, 12),
    to_unsigned(1436, 12), to_unsigned(1438, 12), to_unsigned(1439, 12), to_unsigned(1441, 12), to_unsigned(1442, 12), to_unsigned(1444, 12), to_unsigned(1445, 12), to_unsigned(1446, 12),
    to_unsigned(1448, 12), to_unsigned(1449, 12), to_unsigned(1451, 12), to_unsigned(1452, 12), to_unsigned(1453, 12), to_unsigned(1455, 12), to_unsigned(1456, 12), to_unsigned(1458, 12),
    to_unsigned(1459, 12), to_unsigned(1460, 12), to_unsigned(1462, 12), to_unsigned(1463, 12), to_unsigned(1465, 12), to_unsigned(1466, 12), to_unsigned(1467, 12), to_unsigned(1469, 12),
    to_unsigned(1470, 12), to_unsigned(1472, 12), to_unsigned(1473, 12), to_unsigned(1474, 12), to_unsigned(1476, 12), to_unsigned(1477, 12), to_unsigned(1479, 12), to_unsigned(1480, 12),
    to_unsigned(1481, 12), to_unsigned(1483, 12), to_unsigned(1484, 12), to_unsigned(1485, 12), to_unsigned(1487, 12), to_unsigned(1488, 12), to_unsigned(1490, 12), to_unsigned(1491, 12),
    to_unsigned(1492, 12), to_unsigned(1494, 12), to_unsigned(1495, 12), to_unsigned(1496, 12), to_unsigned(1498, 12), to_unsigned(1499, 12), to_unsigned(1501, 12), to_unsigned(1502, 12),
    to_unsigned(1503, 12), to_unsigned(1505, 12), to_unsigned(1506, 12), to_unsigned(1507, 12), to_unsigned(1509, 12), to_unsigned(1510, 12), to_unsigned(1511, 12), to_unsigned(1513, 12),
    to_unsigned(1514, 12), to_unsigned(1515, 12), to_unsigned(1517, 12), to_unsigned(1518, 12), to_unsigned(1520, 12), to_unsigned(1521, 12), to_unsigned(1522, 12), to_unsigned(1524, 12),
    to_unsigned(1525, 12), to_unsigned(1526, 12), to_unsigned(1528, 12), to_unsigned(1529, 12), to_unsigned(1530, 12), to_unsigned(1532, 12), to_unsigned(1533, 12), to_unsigned(1534, 12),
    to_unsigned(1536, 12), to_unsigned(1537, 12), to_unsigned(1538, 12), to_unsigned(1540, 12), to_unsigned(1541, 12), to_unsigned(1542, 12), to_unsigned(1544, 12), to_unsigned(1545, 12),
    to_unsigned(1546, 12), to_unsigned(1548, 12), to_unsigned(1549, 12), to_unsigned(1550, 12), to_unsigned(1552, 12), to_unsigned(1553, 12), to_unsigned(1554, 12), to_unsigned(1555, 12),
    to_unsigned(1557, 12), to_unsigned(1558, 12), to_unsigned(1559, 12), to_unsigned(1561, 12), to_unsigned(1562, 12), to_unsigned(1563, 12), to_unsigned(1565, 12), to_unsigned(1566, 12),
    to_unsigned(1567, 12), to_unsigned(1569, 12), to_unsigned(1570, 12), to_unsigned(1571, 12), to_unsigned(1573, 12), to_unsigned(1574, 12), to_unsigned(1575, 12), to_unsigned(1576, 12),
    to_unsigned(1578, 12), to_unsigned(1579, 12), to_unsigned(1580, 12), to_unsigned(1582, 12), to_unsigned(1583, 12), to_unsigned(1584, 12), to_unsigned(1585, 12), to_unsigned(1587, 12),
    to_unsigned(1588, 12), to_unsigned(1589, 12), to_unsigned(1591, 12), to_unsigned(1592, 12), to_unsigned(1593, 12), to_unsigned(1594, 12), to_unsigned(1596, 12), to_unsigned(1597, 12),
    to_unsigned(1598, 12), to_unsigned(1600, 12), to_unsigned(1601, 12), to_unsigned(1602, 12), to_unsigned(1603, 12), to_unsigned(1605, 12), to_unsigned(1606, 12), to_unsigned(1607, 12),
    to_unsigned(1609, 12), to_unsigned(1610, 12), to_unsigned(1611, 12), to_unsigned(1612, 12), to_unsigned(1614, 12), to_unsigned(1615, 12), to_unsigned(1616, 12), to_unsigned(1617, 12),
    to_unsigned(1619, 12), to_unsigned(1620, 12), to_unsigned(1621, 12), to_unsigned(1622, 12), to_unsigned(1624, 12), to_unsigned(1625, 12), to_unsigned(1626, 12), to_unsigned(1628, 12),
    to_unsigned(1629, 12), to_unsigned(1630, 12), to_unsigned(1631, 12), to_unsigned(1633, 12), to_unsigned(1634, 12), to_unsigned(1635, 12), to_unsigned(1636, 12), to_unsigned(1638, 12),
    to_unsigned(1639, 12), to_unsigned(1640, 12), to_unsigned(1641, 12), to_unsigned(1643, 12), to_unsigned(1644, 12), to_unsigned(1645, 12), to_unsigned(1646, 12), to_unsigned(1648, 12),
    to_unsigned(1649, 12), to_unsigned(1650, 12), to_unsigned(1651, 12), to_unsigned(1652, 12), to_unsigned(1654, 12), to_unsigned(1655, 12), to_unsigned(1656, 12), to_unsigned(1657, 12),
    to_unsigned(1659, 12), to_unsigned(1660, 12), to_unsigned(1661, 12), to_unsigned(1662, 12), to_unsigned(1664, 12), to_unsigned(1665, 12), to_unsigned(1666, 12), to_unsigned(1667, 12),
    to_unsigned(1669, 12), to_unsigned(1670, 12), to_unsigned(1671, 12), to_unsigned(1672, 12), to_unsigned(1673, 12), to_unsigned(1675, 12), to_unsigned(1676, 12), to_unsigned(1677, 12),
    to_unsigned(1678, 12), to_unsigned(1680, 12), to_unsigned(1681, 12), to_unsigned(1682, 12), to_unsigned(1683, 12), to_unsigned(1684, 12), to_unsigned(1686, 12), to_unsigned(1687, 12),
    to_unsigned(1688, 12), to_unsigned(1689, 12), to_unsigned(1690, 12), to_unsigned(1692, 12), to_unsigned(1693, 12), to_unsigned(1694, 12), to_unsigned(1695, 12), to_unsigned(1696, 12),
    to_unsigned(1698, 12), to_unsigned(1699, 12), to_unsigned(1700, 12), to_unsigned(1701, 12), to_unsigned(1703, 12), to_unsigned(1704, 12), to_unsigned(1705, 12), to_unsigned(1706, 12),
    to_unsigned(1707, 12), to_unsigned(1709, 12), to_unsigned(1710, 12), to_unsigned(1711, 12), to_unsigned(1712, 12), to_unsigned(1713, 12), to_unsigned(1714, 12), to_unsigned(1716, 12),
    to_unsigned(1717, 12), to_unsigned(1718, 12), to_unsigned(1719, 12), to_unsigned(1720, 12), to_unsigned(1722, 12), to_unsigned(1723, 12), to_unsigned(1724, 12), to_unsigned(1725, 12),
    to_unsigned(1726, 12), to_unsigned(1728, 12), to_unsigned(1729, 12), to_unsigned(1730, 12), to_unsigned(1731, 12), to_unsigned(1732, 12), to_unsigned(1733, 12), to_unsigned(1735, 12),
    to_unsigned(1736, 12), to_unsigned(1737, 12), to_unsigned(1738, 12), to_unsigned(1739, 12), to_unsigned(1741, 12), to_unsigned(1742, 12), to_unsigned(1743, 12), to_unsigned(1744, 12),
    to_unsigned(1745, 12), to_unsigned(1746, 12), to_unsigned(1748, 12), to_unsigned(1749, 12), to_unsigned(1750, 12), to_unsigned(1751, 12), to_unsigned(1752, 12), to_unsigned(1753, 12),
    to_unsigned(1755, 12), to_unsigned(1756, 12), to_unsigned(1757, 12), to_unsigned(1758, 12), to_unsigned(1759, 12), to_unsigned(1760, 12), to_unsigned(1762, 12), to_unsigned(1763, 12),
    to_unsigned(1764, 12), to_unsigned(1765, 12), to_unsigned(1766, 12), to_unsigned(1767, 12), to_unsigned(1769, 12), to_unsigned(1770, 12), to_unsigned(1771, 12), to_unsigned(1772, 12),
    to_unsigned(1773, 12), to_unsigned(1774, 12), to_unsigned(1775, 12), to_unsigned(1777, 12), to_unsigned(1778, 12), to_unsigned(1779, 12), to_unsigned(1780, 12), to_unsigned(1781, 12),
    to_unsigned(1782, 12), to_unsigned(1784, 12), to_unsigned(1785, 12), to_unsigned(1786, 12), to_unsigned(1787, 12), to_unsigned(1788, 12), to_unsigned(1789, 12), to_unsigned(1790, 12),
    to_unsigned(1792, 12), to_unsigned(1793, 12), to_unsigned(1794, 12), to_unsigned(1795, 12), to_unsigned(1796, 12), to_unsigned(1797, 12), to_unsigned(1798, 12), to_unsigned(1800, 12),
    to_unsigned(1801, 12), to_unsigned(1802, 12), to_unsigned(1803, 12), to_unsigned(1804, 12), to_unsigned(1805, 12), to_unsigned(1806, 12), to_unsigned(1807, 12), to_unsigned(1809, 12),
    to_unsigned(1810, 12), to_unsigned(1811, 12), to_unsigned(1812, 12), to_unsigned(1813, 12), to_unsigned(1814, 12), to_unsigned(1815, 12), to_unsigned(1817, 12), to_unsigned(1818, 12),
    to_unsigned(1819, 12), to_unsigned(1820, 12), to_unsigned(1821, 12), to_unsigned(1822, 12), to_unsigned(1823, 12), to_unsigned(1824, 12), to_unsigned(1826, 12), to_unsigned(1827, 12),
    to_unsigned(1828, 12), to_unsigned(1829, 12), to_unsigned(1830, 12), to_unsigned(1831, 12), to_unsigned(1832, 12), to_unsigned(1833, 12), to_unsigned(1834, 12), to_unsigned(1836, 12),
    to_unsigned(1837, 12), to_unsigned(1838, 12), to_unsigned(1839, 12), to_unsigned(1840, 12), to_unsigned(1841, 12), to_unsigned(1842, 12), to_unsigned(1843, 12), to_unsigned(1844, 12),
    to_unsigned(1846, 12), to_unsigned(1847, 12), to_unsigned(1848, 12), to_unsigned(1849, 12), to_unsigned(1850, 12), to_unsigned(1851, 12), to_unsigned(1852, 12), to_unsigned(1853, 12),
    to_unsigned(1854, 12), to_unsigned(1856, 12), to_unsigned(1857, 12), to_unsigned(1858, 12), to_unsigned(1859, 12), to_unsigned(1860, 12), to_unsigned(1861, 12), to_unsigned(1862, 12),
    to_unsigned(1863, 12), to_unsigned(1864, 12), to_unsigned(1865, 12), to_unsigned(1867, 12), to_unsigned(1868, 12), to_unsigned(1869, 12), to_unsigned(1870, 12), to_unsigned(1871, 12),
    to_unsigned(1872, 12), to_unsigned(1873, 12), to_unsigned(1874, 12), to_unsigned(1875, 12), to_unsigned(1876, 12), to_unsigned(1877, 12), to_unsigned(1879, 12), to_unsigned(1880, 12),
    to_unsigned(1881, 12), to_unsigned(1882, 12), to_unsigned(1883, 12), to_unsigned(1884, 12), to_unsigned(1885, 12), to_unsigned(1886, 12), to_unsigned(1887, 12), to_unsigned(1888, 12),
    to_unsigned(1889, 12), to_unsigned(1891, 12), to_unsigned(1892, 12), to_unsigned(1893, 12), to_unsigned(1894, 12), to_unsigned(1895, 12), to_unsigned(1896, 12), to_unsigned(1897, 12),
    to_unsigned(1898, 12), to_unsigned(1899, 12), to_unsigned(1900, 12), to_unsigned(1901, 12), to_unsigned(1902, 12), to_unsigned(1903, 12), to_unsigned(1905, 12), to_unsigned(1906, 12),
    to_unsigned(1907, 12), to_unsigned(1908, 12), to_unsigned(1909, 12), to_unsigned(1910, 12), to_unsigned(1911, 12), to_unsigned(1912, 12), to_unsigned(1913, 12), to_unsigned(1914, 12),
    to_unsigned(1915, 12), to_unsigned(1916, 12), to_unsigned(1917, 12), to_unsigned(1918, 12), to_unsigned(1920, 12), to_unsigned(1921, 12), to_unsigned(1922, 12), to_unsigned(1923, 12),
    to_unsigned(1924, 12), to_unsigned(1925, 12), to_unsigned(1926, 12), to_unsigned(1927, 12), to_unsigned(1928, 12), to_unsigned(1929, 12), to_unsigned(1930, 12), to_unsigned(1931, 12),
    to_unsigned(1932, 12), to_unsigned(1933, 12), to_unsigned(1934, 12), to_unsigned(1935, 12), to_unsigned(1937, 12), to_unsigned(1938, 12), to_unsigned(1939, 12), to_unsigned(1940, 12),
    to_unsigned(1941, 12), to_unsigned(1942, 12), to_unsigned(1943, 12), to_unsigned(1944, 12), to_unsigned(1945, 12), to_unsigned(1946, 12), to_unsigned(1947, 12), to_unsigned(1948, 12),
    to_unsigned(1949, 12), to_unsigned(1950, 12), to_unsigned(1951, 12), to_unsigned(1952, 12), to_unsigned(1953, 12), to_unsigned(1954, 12), to_unsigned(1955, 12), to_unsigned(1956, 12),
    to_unsigned(1958, 12), to_unsigned(1959, 12), to_unsigned(1960, 12), to_unsigned(1961, 12), to_unsigned(1962, 12), to_unsigned(1963, 12), to_unsigned(1964, 12), to_unsigned(1965, 12),
    to_unsigned(1966, 12), to_unsigned(1967, 12), to_unsigned(1968, 12), to_unsigned(1969, 12), to_unsigned(1970, 12), to_unsigned(1971, 12), to_unsigned(1972, 12), to_unsigned(1973, 12),
    to_unsigned(1974, 12), to_unsigned(1975, 12), to_unsigned(1976, 12), to_unsigned(1977, 12), to_unsigned(1978, 12), to_unsigned(1979, 12), to_unsigned(1980, 12), to_unsigned(1981, 12),
    to_unsigned(1982, 12), to_unsigned(1984, 12), to_unsigned(1985, 12), to_unsigned(1986, 12), to_unsigned(1987, 12), to_unsigned(1988, 12), to_unsigned(1989, 12), to_unsigned(1990, 12),
    to_unsigned(1991, 12), to_unsigned(1992, 12), to_unsigned(1993, 12), to_unsigned(1994, 12), to_unsigned(1995, 12), to_unsigned(1996, 12), to_unsigned(1997, 12), to_unsigned(1998, 12),
    to_unsigned(1999, 12), to_unsigned(2000, 12), to_unsigned(2001, 12), to_unsigned(2002, 12), to_unsigned(2003, 12), to_unsigned(2004, 12), to_unsigned(2005, 12), to_unsigned(2006, 12),
    to_unsigned(2007, 12), to_unsigned(2008, 12), to_unsigned(2009, 12), to_unsigned(2010, 12), to_unsigned(2011, 12), to_unsigned(2012, 12), to_unsigned(2013, 12), to_unsigned(2014, 12),
    to_unsigned(2015, 12), to_unsigned(2016, 12), to_unsigned(2017, 12), to_unsigned(2018, 12), to_unsigned(2019, 12), to_unsigned(2020, 12), to_unsigned(2021, 12), to_unsigned(2022, 12),
    to_unsigned(2023, 12), to_unsigned(2024, 12), to_unsigned(2025, 12), to_unsigned(2026, 12), to_unsigned(2027, 12), to_unsigned(2028, 12), to_unsigned(2029, 12), to_unsigned(2030, 12),
    to_unsigned(2031, 12), to_unsigned(2032, 12), to_unsigned(2033, 12), to_unsigned(2034, 12), to_unsigned(2035, 12), to_unsigned(2036, 12), to_unsigned(2037, 12), to_unsigned(2038, 12),
    to_unsigned(2039, 12), to_unsigned(2040, 12), to_unsigned(2041, 12), to_unsigned(2042, 12), to_unsigned(2043, 12), to_unsigned(2044, 12), to_unsigned(2045, 12), to_unsigned(2046, 12),
    to_unsigned(2048, 12), to_unsigned(2048, 12), to_unsigned(2049, 12), to_unsigned(2050, 12), to_unsigned(2051, 12), to_unsigned(2052, 12), to_unsigned(2053, 12), to_unsigned(2054, 12),
    to_unsigned(2055, 12), to_unsigned(2056, 12), to_unsigned(2057, 12), to_unsigned(2058, 12), to_unsigned(2059, 12), to_unsigned(2060, 12), to_unsigned(2061, 12), to_unsigned(2062, 12),
    to_unsigned(2063, 12), to_unsigned(2064, 12), to_unsigned(2065, 12), to_unsigned(2066, 12), to_unsigned(2067, 12), to_unsigned(2068, 12), to_unsigned(2069, 12), to_unsigned(2070, 12),
    to_unsigned(2071, 12), to_unsigned(2072, 12), to_unsigned(2073, 12), to_unsigned(2074, 12), to_unsigned(2075, 12), to_unsigned(2076, 12), to_unsigned(2077, 12), to_unsigned(2078, 12),
    to_unsigned(2079, 12), to_unsigned(2080, 12), to_unsigned(2081, 12), to_unsigned(2082, 12), to_unsigned(2083, 12), to_unsigned(2084, 12), to_unsigned(2085, 12), to_unsigned(2086, 12),
    to_unsigned(2087, 12), to_unsigned(2088, 12), to_unsigned(2089, 12), to_unsigned(2090, 12), to_unsigned(2091, 12), to_unsigned(2092, 12), to_unsigned(2093, 12), to_unsigned(2094, 12),
    to_unsigned(2095, 12), to_unsigned(2096, 12), to_unsigned(2097, 12), to_unsigned(2098, 12), to_unsigned(2099, 12), to_unsigned(2100, 12), to_unsigned(2101, 12), to_unsigned(2102, 12),
    to_unsigned(2103, 12), to_unsigned(2104, 12), to_unsigned(2105, 12), to_unsigned(2106, 12), to_unsigned(2107, 12), to_unsigned(2108, 12), to_unsigned(2109, 12), to_unsigned(2110, 12),
    to_unsigned(2111, 12), to_unsigned(2111, 12), to_unsigned(2112, 12), to_unsigned(2113, 12), to_unsigned(2114, 12), to_unsigned(2115, 12), to_unsigned(2116, 12), to_unsigned(2117, 12),
    to_unsigned(2118, 12), to_unsigned(2119, 12), to_unsigned(2120, 12), to_unsigned(2121, 12), to_unsigned(2122, 12), to_unsigned(2123, 12), to_unsigned(2124, 12), to_unsigned(2125, 12),
    to_unsigned(2126, 12), to_unsigned(2127, 12), to_unsigned(2128, 12), to_unsigned(2129, 12), to_unsigned(2130, 12), to_unsigned(2131, 12), to_unsigned(2132, 12), to_unsigned(2133, 12),
    to_unsigned(2134, 12), to_unsigned(2135, 12), to_unsigned(2136, 12), to_unsigned(2137, 12), to_unsigned(2137, 12), to_unsigned(2138, 12), to_unsigned(2139, 12), to_unsigned(2140, 12),
    to_unsigned(2141, 12), to_unsigned(2142, 12), to_unsigned(2143, 12), to_unsigned(2144, 12), to_unsigned(2145, 12), to_unsigned(2146, 12), to_unsigned(2147, 12), to_unsigned(2148, 12),
    to_unsigned(2149, 12), to_unsigned(2150, 12), to_unsigned(2151, 12), to_unsigned(2152, 12), to_unsigned(2153, 12), to_unsigned(2154, 12), to_unsigned(2155, 12), to_unsigned(2156, 12),
    to_unsigned(2157, 12), to_unsigned(2158, 12), to_unsigned(2158, 12), to_unsigned(2159, 12), to_unsigned(2160, 12), to_unsigned(2161, 12), to_unsigned(2162, 12), to_unsigned(2163, 12),
    to_unsigned(2164, 12), to_unsigned(2165, 12), to_unsigned(2166, 12), to_unsigned(2167, 12), to_unsigned(2168, 12), to_unsigned(2169, 12), to_unsigned(2170, 12), to_unsigned(2171, 12),
    to_unsigned(2172, 12), to_unsigned(2173, 12), to_unsigned(2174, 12), to_unsigned(2175, 12), to_unsigned(2175, 12), to_unsigned(2176, 12), to_unsigned(2177, 12), to_unsigned(2178, 12),
    to_unsigned(2179, 12), to_unsigned(2180, 12), to_unsigned(2181, 12), to_unsigned(2182, 12), to_unsigned(2183, 12), to_unsigned(2184, 12), to_unsigned(2185, 12), to_unsigned(2186, 12),
    to_unsigned(2187, 12), to_unsigned(2188, 12), to_unsigned(2189, 12), to_unsigned(2190, 12), to_unsigned(2190, 12), to_unsigned(2191, 12), to_unsigned(2192, 12), to_unsigned(2193, 12),
    to_unsigned(2194, 12), to_unsigned(2195, 12), to_unsigned(2196, 12), to_unsigned(2197, 12), to_unsigned(2198, 12), to_unsigned(2199, 12), to_unsigned(2200, 12), to_unsigned(2201, 12),
    to_unsigned(2202, 12), to_unsigned(2203, 12), to_unsigned(2204, 12), to_unsigned(2204, 12), to_unsigned(2205, 12), to_unsigned(2206, 12), to_unsigned(2207, 12), to_unsigned(2208, 12),
    to_unsigned(2209, 12), to_unsigned(2210, 12), to_unsigned(2211, 12), to_unsigned(2212, 12), to_unsigned(2213, 12), to_unsigned(2214, 12), to_unsigned(2215, 12), to_unsigned(2216, 12),
    to_unsigned(2216, 12), to_unsigned(2217, 12), to_unsigned(2218, 12), to_unsigned(2219, 12), to_unsigned(2220, 12), to_unsigned(2221, 12), to_unsigned(2222, 12), to_unsigned(2223, 12),
    to_unsigned(2224, 12), to_unsigned(2225, 12), to_unsigned(2226, 12), to_unsigned(2227, 12), to_unsigned(2228, 12), to_unsigned(2228, 12), to_unsigned(2229, 12), to_unsigned(2230, 12),
    to_unsigned(2231, 12), to_unsigned(2232, 12), to_unsigned(2233, 12), to_unsigned(2234, 12), to_unsigned(2235, 12), to_unsigned(2236, 12), to_unsigned(2237, 12), to_unsigned(2238, 12),
    to_unsigned(2239, 12), to_unsigned(2239, 12), to_unsigned(2240, 12), to_unsigned(2241, 12), to_unsigned(2242, 12), to_unsigned(2243, 12), to_unsigned(2244, 12), to_unsigned(2245, 12),
    to_unsigned(2246, 12), to_unsigned(2247, 12), to_unsigned(2248, 12), to_unsigned(2249, 12), to_unsigned(2249, 12), to_unsigned(2250, 12), to_unsigned(2251, 12), to_unsigned(2252, 12),
    to_unsigned(2253, 12), to_unsigned(2254, 12), to_unsigned(2255, 12), to_unsigned(2256, 12), to_unsigned(2257, 12), to_unsigned(2258, 12), to_unsigned(2259, 12), to_unsigned(2259, 12),
    to_unsigned(2260, 12), to_unsigned(2261, 12), to_unsigned(2262, 12), to_unsigned(2263, 12), to_unsigned(2264, 12), to_unsigned(2265, 12), to_unsigned(2266, 12), to_unsigned(2267, 12),
    to_unsigned(2268, 12), to_unsigned(2269, 12), to_unsigned(2269, 12), to_unsigned(2270, 12), to_unsigned(2271, 12), to_unsigned(2272, 12), to_unsigned(2273, 12), to_unsigned(2274, 12),
    to_unsigned(2275, 12), to_unsigned(2276, 12), to_unsigned(2277, 12), to_unsigned(2278, 12), to_unsigned(2278, 12), to_unsigned(2279, 12), to_unsigned(2280, 12), to_unsigned(2281, 12),
    to_unsigned(2282, 12), to_unsigned(2283, 12), to_unsigned(2284, 12), to_unsigned(2285, 12), to_unsigned(2286, 12), to_unsigned(2286, 12), to_unsigned(2287, 12), to_unsigned(2288, 12),
    to_unsigned(2289, 12), to_unsigned(2290, 12), to_unsigned(2291, 12), to_unsigned(2292, 12), to_unsigned(2293, 12), to_unsigned(2294, 12), to_unsigned(2295, 12), to_unsigned(2295, 12),
    to_unsigned(2296, 12), to_unsigned(2297, 12), to_unsigned(2298, 12), to_unsigned(2299, 12), to_unsigned(2300, 12), to_unsigned(2301, 12), to_unsigned(2302, 12), to_unsigned(2303, 12),
    to_unsigned(2303, 12), to_unsigned(2304, 12), to_unsigned(2305, 12), to_unsigned(2306, 12), to_unsigned(2307, 12), to_unsigned(2308, 12), to_unsigned(2309, 12), to_unsigned(2310, 12),
    to_unsigned(2311, 12), to_unsigned(2311, 12), to_unsigned(2312, 12), to_unsigned(2313, 12), to_unsigned(2314, 12), to_unsigned(2315, 12), to_unsigned(2316, 12), to_unsigned(2317, 12),
    to_unsigned(2318, 12), to_unsigned(2318, 12), to_unsigned(2319, 12), to_unsigned(2320, 12), to_unsigned(2321, 12), to_unsigned(2322, 12), to_unsigned(2323, 12), to_unsigned(2324, 12),
    to_unsigned(2325, 12), to_unsigned(2326, 12), to_unsigned(2326, 12), to_unsigned(2327, 12), to_unsigned(2328, 12), to_unsigned(2329, 12), to_unsigned(2330, 12), to_unsigned(2331, 12),
    to_unsigned(2332, 12), to_unsigned(2333, 12), to_unsigned(2333, 12), to_unsigned(2334, 12), to_unsigned(2335, 12), to_unsigned(2336, 12), to_unsigned(2337, 12), to_unsigned(2338, 12),
    to_unsigned(2339, 12), to_unsigned(2340, 12), to_unsigned(2340, 12), to_unsigned(2341, 12), to_unsigned(2342, 12), to_unsigned(2343, 12), to_unsigned(2344, 12), to_unsigned(2345, 12),
    to_unsigned(2346, 12), to_unsigned(2347, 12), to_unsigned(2347, 12), to_unsigned(2348, 12), to_unsigned(2349, 12), to_unsigned(2350, 12), to_unsigned(2351, 12), to_unsigned(2352, 12),
    to_unsigned(2353, 12), to_unsigned(2354, 12), to_unsigned(2354, 12), to_unsigned(2355, 12), to_unsigned(2356, 12), to_unsigned(2357, 12), to_unsigned(2358, 12), to_unsigned(2359, 12),
    to_unsigned(2360, 12), to_unsigned(2360, 12), to_unsigned(2361, 12), to_unsigned(2362, 12), to_unsigned(2363, 12), to_unsigned(2364, 12), to_unsigned(2365, 12), to_unsigned(2366, 12),
    to_unsigned(2367, 12), to_unsigned(2367, 12), to_unsigned(2368, 12), to_unsigned(2369, 12), to_unsigned(2370, 12), to_unsigned(2371, 12), to_unsigned(2372, 12), to_unsigned(2373, 12),
    to_unsigned(2373, 12), to_unsigned(2374, 12), to_unsigned(2375, 12), to_unsigned(2376, 12), to_unsigned(2377, 12), to_unsigned(2378, 12), to_unsigned(2379, 12), to_unsigned(2379, 12),
    to_unsigned(2380, 12), to_unsigned(2381, 12), to_unsigned(2382, 12), to_unsigned(2383, 12), to_unsigned(2384, 12), to_unsigned(2385, 12), to_unsigned(2386, 12), to_unsigned(2386, 12),
    to_unsigned(2387, 12), to_unsigned(2388, 12), to_unsigned(2389, 12), to_unsigned(2390, 12), to_unsigned(2391, 12), to_unsigned(2392, 12), to_unsigned(2392, 12), to_unsigned(2393, 12),
    to_unsigned(2394, 12), to_unsigned(2395, 12), to_unsigned(2396, 12), to_unsigned(2397, 12), to_unsigned(2397, 12), to_unsigned(2398, 12), to_unsigned(2399, 12), to_unsigned(2400, 12),
    to_unsigned(2401, 12), to_unsigned(2402, 12), to_unsigned(2403, 12), to_unsigned(2403, 12), to_unsigned(2404, 12), to_unsigned(2405, 12), to_unsigned(2406, 12), to_unsigned(2407, 12),
    to_unsigned(2408, 12), to_unsigned(2409, 12), to_unsigned(2409, 12), to_unsigned(2410, 12), to_unsigned(2411, 12), to_unsigned(2412, 12), to_unsigned(2413, 12), to_unsigned(2414, 12),
    to_unsigned(2415, 12), to_unsigned(2415, 12), to_unsigned(2416, 12), to_unsigned(2417, 12), to_unsigned(2418, 12), to_unsigned(2419, 12), to_unsigned(2420, 12), to_unsigned(2420, 12),
    to_unsigned(2421, 12), to_unsigned(2422, 12), to_unsigned(2423, 12), to_unsigned(2424, 12), to_unsigned(2425, 12), to_unsigned(2426, 12), to_unsigned(2426, 12), to_unsigned(2427, 12),
    to_unsigned(2428, 12), to_unsigned(2429, 12), to_unsigned(2430, 12), to_unsigned(2431, 12), to_unsigned(2431, 12), to_unsigned(2432, 12), to_unsigned(2433, 12), to_unsigned(2434, 12),
    to_unsigned(2435, 12), to_unsigned(2436, 12), to_unsigned(2436, 12), to_unsigned(2437, 12), to_unsigned(2438, 12), to_unsigned(2439, 12), to_unsigned(2440, 12), to_unsigned(2441, 12),
    to_unsigned(2441, 12), to_unsigned(2442, 12), to_unsigned(2443, 12), to_unsigned(2444, 12), to_unsigned(2445, 12), to_unsigned(2446, 12), to_unsigned(2447, 12), to_unsigned(2447, 12),
    to_unsigned(2448, 12), to_unsigned(2449, 12), to_unsigned(2450, 12), to_unsigned(2451, 12), to_unsigned(2452, 12), to_unsigned(2452, 12), to_unsigned(2453, 12), to_unsigned(2454, 12),
    to_unsigned(2455, 12), to_unsigned(2456, 12), to_unsigned(2457, 12), to_unsigned(2457, 12), to_unsigned(2458, 12), to_unsigned(2459, 12), to_unsigned(2460, 12), to_unsigned(2461, 12),
    to_unsigned(2462, 12), to_unsigned(2462, 12), to_unsigned(2463, 12), to_unsigned(2464, 12), to_unsigned(2465, 12), to_unsigned(2466, 12), to_unsigned(2467, 12), to_unsigned(2467, 12),
    to_unsigned(2468, 12), to_unsigned(2469, 12), to_unsigned(2470, 12), to_unsigned(2471, 12), to_unsigned(2471, 12), to_unsigned(2472, 12), to_unsigned(2473, 12), to_unsigned(2474, 12),
    to_unsigned(2475, 12), to_unsigned(2476, 12), to_unsigned(2476, 12), to_unsigned(2477, 12), to_unsigned(2478, 12), to_unsigned(2479, 12), to_unsigned(2480, 12), to_unsigned(2481, 12),
    to_unsigned(2481, 12), to_unsigned(2482, 12), to_unsigned(2483, 12), to_unsigned(2484, 12), to_unsigned(2485, 12), to_unsigned(2486, 12), to_unsigned(2486, 12), to_unsigned(2487, 12),
    to_unsigned(2488, 12), to_unsigned(2489, 12), to_unsigned(2490, 12), to_unsigned(2490, 12), to_unsigned(2491, 12), to_unsigned(2492, 12), to_unsigned(2493, 12), to_unsigned(2494, 12),
    to_unsigned(2495, 12), to_unsigned(2495, 12), to_unsigned(2496, 12), to_unsigned(2497, 12), to_unsigned(2498, 12), to_unsigned(2499, 12), to_unsigned(2499, 12), to_unsigned(2500, 12),
    to_unsigned(2501, 12), to_unsigned(2502, 12), to_unsigned(2503, 12), to_unsigned(2504, 12), to_unsigned(2504, 12), to_unsigned(2505, 12), to_unsigned(2506, 12), to_unsigned(2507, 12),
    to_unsigned(2508, 12), to_unsigned(2508, 12), to_unsigned(2509, 12), to_unsigned(2510, 12), to_unsigned(2511, 12), to_unsigned(2512, 12), to_unsigned(2513, 12), to_unsigned(2513, 12),
    to_unsigned(2514, 12), to_unsigned(2515, 12), to_unsigned(2516, 12), to_unsigned(2517, 12), to_unsigned(2517, 12), to_unsigned(2518, 12), to_unsigned(2519, 12), to_unsigned(2520, 12),
    to_unsigned(2521, 12), to_unsigned(2522, 12), to_unsigned(2522, 12), to_unsigned(2523, 12), to_unsigned(2524, 12), to_unsigned(2525, 12), to_unsigned(2526, 12), to_unsigned(2526, 12),
    to_unsigned(2527, 12), to_unsigned(2528, 12), to_unsigned(2529, 12), to_unsigned(2530, 12), to_unsigned(2530, 12), to_unsigned(2531, 12), to_unsigned(2532, 12), to_unsigned(2533, 12),
    to_unsigned(2534, 12), to_unsigned(2534, 12), to_unsigned(2535, 12), to_unsigned(2536, 12), to_unsigned(2537, 12), to_unsigned(2538, 12), to_unsigned(2538, 12), to_unsigned(2539, 12),
    to_unsigned(2540, 12), to_unsigned(2541, 12), to_unsigned(2542, 12), to_unsigned(2543, 12), to_unsigned(2543, 12), to_unsigned(2544, 12), to_unsigned(2545, 12), to_unsigned(2546, 12),
    to_unsigned(2547, 12), to_unsigned(2547, 12), to_unsigned(2548, 12), to_unsigned(2549, 12), to_unsigned(2550, 12), to_unsigned(2551, 12), to_unsigned(2551, 12), to_unsigned(2552, 12),
    to_unsigned(2553, 12), to_unsigned(2554, 12), to_unsigned(2555, 12), to_unsigned(2555, 12), to_unsigned(2556, 12), to_unsigned(2557, 12), to_unsigned(2558, 12), to_unsigned(2559, 12),
    to_unsigned(2559, 12), to_unsigned(2560, 12), to_unsigned(2561, 12), to_unsigned(2562, 12), to_unsigned(2563, 12), to_unsigned(2563, 12), to_unsigned(2564, 12), to_unsigned(2565, 12),
    to_unsigned(2566, 12), to_unsigned(2567, 12), to_unsigned(2567, 12), to_unsigned(2568, 12), to_unsigned(2569, 12), to_unsigned(2570, 12), to_unsigned(2571, 12), to_unsigned(2571, 12),
    to_unsigned(2572, 12), to_unsigned(2573, 12), to_unsigned(2574, 12), to_unsigned(2575, 12), to_unsigned(2575, 12), to_unsigned(2576, 12), to_unsigned(2577, 12), to_unsigned(2578, 12),
    to_unsigned(2578, 12), to_unsigned(2579, 12), to_unsigned(2580, 12), to_unsigned(2581, 12), to_unsigned(2582, 12), to_unsigned(2582, 12), to_unsigned(2583, 12), to_unsigned(2584, 12),
    to_unsigned(2585, 12), to_unsigned(2586, 12), to_unsigned(2586, 12), to_unsigned(2587, 12), to_unsigned(2588, 12), to_unsigned(2589, 12), to_unsigned(2590, 12), to_unsigned(2590, 12),
    to_unsigned(2591, 12), to_unsigned(2592, 12), to_unsigned(2593, 12), to_unsigned(2594, 12), to_unsigned(2594, 12), to_unsigned(2595, 12), to_unsigned(2596, 12), to_unsigned(2597, 12),
    to_unsigned(2597, 12), to_unsigned(2598, 12), to_unsigned(2599, 12), to_unsigned(2600, 12), to_unsigned(2601, 12), to_unsigned(2601, 12), to_unsigned(2602, 12), to_unsigned(2603, 12),
    to_unsigned(2604, 12), to_unsigned(2605, 12), to_unsigned(2605, 12), to_unsigned(2606, 12), to_unsigned(2607, 12), to_unsigned(2608, 12), to_unsigned(2608, 12), to_unsigned(2609, 12),
    to_unsigned(2610, 12), to_unsigned(2611, 12), to_unsigned(2612, 12), to_unsigned(2612, 12), to_unsigned(2613, 12), to_unsigned(2614, 12), to_unsigned(2615, 12), to_unsigned(2616, 12),
    to_unsigned(2616, 12), to_unsigned(2617, 12), to_unsigned(2618, 12), to_unsigned(2619, 12), to_unsigned(2619, 12), to_unsigned(2620, 12), to_unsigned(2621, 12), to_unsigned(2622, 12),
    to_unsigned(2623, 12), to_unsigned(2623, 12), to_unsigned(2624, 12), to_unsigned(2625, 12), to_unsigned(2626, 12), to_unsigned(2626, 12), to_unsigned(2627, 12), to_unsigned(2628, 12),
    to_unsigned(2629, 12), to_unsigned(2630, 12), to_unsigned(2630, 12), to_unsigned(2631, 12), to_unsigned(2632, 12), to_unsigned(2633, 12), to_unsigned(2633, 12), to_unsigned(2634, 12),
    to_unsigned(2635, 12), to_unsigned(2636, 12), to_unsigned(2637, 12), to_unsigned(2637, 12), to_unsigned(2638, 12), to_unsigned(2639, 12), to_unsigned(2640, 12), to_unsigned(2640, 12),
    to_unsigned(2641, 12), to_unsigned(2642, 12), to_unsigned(2643, 12), to_unsigned(2644, 12), to_unsigned(2644, 12), to_unsigned(2645, 12), to_unsigned(2646, 12), to_unsigned(2647, 12),
    to_unsigned(2647, 12), to_unsigned(2648, 12), to_unsigned(2649, 12), to_unsigned(2650, 12), to_unsigned(2651, 12), to_unsigned(2651, 12), to_unsigned(2652, 12), to_unsigned(2653, 12),
    to_unsigned(2654, 12), to_unsigned(2654, 12), to_unsigned(2655, 12), to_unsigned(2656, 12), to_unsigned(2657, 12), to_unsigned(2657, 12), to_unsigned(2658, 12), to_unsigned(2659, 12),
    to_unsigned(2660, 12), to_unsigned(2661, 12), to_unsigned(2661, 12), to_unsigned(2662, 12), to_unsigned(2663, 12), to_unsigned(2664, 12), to_unsigned(2664, 12), to_unsigned(2665, 12),
    to_unsigned(2666, 12), to_unsigned(2667, 12), to_unsigned(2667, 12), to_unsigned(2668, 12), to_unsigned(2669, 12), to_unsigned(2670, 12), to_unsigned(2671, 12), to_unsigned(2671, 12),
    to_unsigned(2672, 12), to_unsigned(2673, 12), to_unsigned(2674, 12), to_unsigned(2674, 12), to_unsigned(2675, 12), to_unsigned(2676, 12), to_unsigned(2677, 12), to_unsigned(2677, 12),
    to_unsigned(2678, 12), to_unsigned(2679, 12), to_unsigned(2680, 12), to_unsigned(2680, 12), to_unsigned(2681, 12), to_unsigned(2682, 12), to_unsigned(2683, 12), to_unsigned(2684, 12),
    to_unsigned(2684, 12), to_unsigned(2685, 12), to_unsigned(2686, 12), to_unsigned(2687, 12), to_unsigned(2687, 12), to_unsigned(2688, 12), to_unsigned(2689, 12), to_unsigned(2690, 12),
    to_unsigned(2690, 12), to_unsigned(2691, 12), to_unsigned(2692, 12), to_unsigned(2693, 12), to_unsigned(2693, 12), to_unsigned(2694, 12), to_unsigned(2695, 12), to_unsigned(2696, 12),
    to_unsigned(2696, 12), to_unsigned(2697, 12), to_unsigned(2698, 12), to_unsigned(2699, 12), to_unsigned(2700, 12), to_unsigned(2700, 12), to_unsigned(2701, 12), to_unsigned(2702, 12),
    to_unsigned(2703, 12), to_unsigned(2703, 12), to_unsigned(2704, 12), to_unsigned(2705, 12), to_unsigned(2706, 12), to_unsigned(2706, 12), to_unsigned(2707, 12), to_unsigned(2708, 12),
    to_unsigned(2709, 12), to_unsigned(2709, 12), to_unsigned(2710, 12), to_unsigned(2711, 12), to_unsigned(2712, 12), to_unsigned(2712, 12), to_unsigned(2713, 12), to_unsigned(2714, 12),
    to_unsigned(2715, 12), to_unsigned(2715, 12), to_unsigned(2716, 12), to_unsigned(2717, 12), to_unsigned(2718, 12), to_unsigned(2718, 12), to_unsigned(2719, 12), to_unsigned(2720, 12),
    to_unsigned(2721, 12), to_unsigned(2721, 12), to_unsigned(2722, 12), to_unsigned(2723, 12), to_unsigned(2724, 12), to_unsigned(2724, 12), to_unsigned(2725, 12), to_unsigned(2726, 12),
    to_unsigned(2727, 12), to_unsigned(2727, 12), to_unsigned(2728, 12), to_unsigned(2729, 12), to_unsigned(2730, 12), to_unsigned(2730, 12), to_unsigned(2731, 12), to_unsigned(2732, 12),
    to_unsigned(2733, 12), to_unsigned(2733, 12), to_unsigned(2734, 12), to_unsigned(2735, 12), to_unsigned(2736, 12), to_unsigned(2736, 12), to_unsigned(2737, 12), to_unsigned(2738, 12),
    to_unsigned(2739, 12), to_unsigned(2739, 12), to_unsigned(2740, 12), to_unsigned(2741, 12), to_unsigned(2742, 12), to_unsigned(2742, 12), to_unsigned(2743, 12), to_unsigned(2744, 12),
    to_unsigned(2745, 12), to_unsigned(2745, 12), to_unsigned(2746, 12), to_unsigned(2747, 12), to_unsigned(2748, 12), to_unsigned(2748, 12), to_unsigned(2749, 12), to_unsigned(2750, 12),
    to_unsigned(2751, 12), to_unsigned(2751, 12), to_unsigned(2752, 12), to_unsigned(2753, 12), to_unsigned(2754, 12), to_unsigned(2754, 12), to_unsigned(2755, 12), to_unsigned(2756, 12),
    to_unsigned(2757, 12), to_unsigned(2757, 12), to_unsigned(2758, 12), to_unsigned(2759, 12), to_unsigned(2760, 12), to_unsigned(2760, 12), to_unsigned(2761, 12), to_unsigned(2762, 12),
    to_unsigned(2762, 12), to_unsigned(2763, 12), to_unsigned(2764, 12), to_unsigned(2765, 12), to_unsigned(2765, 12), to_unsigned(2766, 12), to_unsigned(2767, 12), to_unsigned(2768, 12),
    to_unsigned(2768, 12), to_unsigned(2769, 12), to_unsigned(2770, 12), to_unsigned(2771, 12), to_unsigned(2771, 12), to_unsigned(2772, 12), to_unsigned(2773, 12), to_unsigned(2774, 12),
    to_unsigned(2774, 12), to_unsigned(2775, 12), to_unsigned(2776, 12), to_unsigned(2777, 12), to_unsigned(2777, 12), to_unsigned(2778, 12), to_unsigned(2779, 12), to_unsigned(2779, 12),
    to_unsigned(2780, 12), to_unsigned(2781, 12), to_unsigned(2782, 12), to_unsigned(2782, 12), to_unsigned(2783, 12), to_unsigned(2784, 12), to_unsigned(2785, 12), to_unsigned(2785, 12),
    to_unsigned(2786, 12), to_unsigned(2787, 12), to_unsigned(2788, 12), to_unsigned(2788, 12), to_unsigned(2789, 12), to_unsigned(2790, 12), to_unsigned(2790, 12), to_unsigned(2791, 12),
    to_unsigned(2792, 12), to_unsigned(2793, 12), to_unsigned(2793, 12), to_unsigned(2794, 12), to_unsigned(2795, 12), to_unsigned(2796, 12), to_unsigned(2796, 12), to_unsigned(2797, 12),
    to_unsigned(2798, 12), to_unsigned(2799, 12), to_unsigned(2799, 12), to_unsigned(2800, 12), to_unsigned(2801, 12), to_unsigned(2801, 12), to_unsigned(2802, 12), to_unsigned(2803, 12),
    to_unsigned(2804, 12), to_unsigned(2804, 12), to_unsigned(2805, 12), to_unsigned(2806, 12), to_unsigned(2807, 12), to_unsigned(2807, 12), to_unsigned(2808, 12), to_unsigned(2809, 12),
    to_unsigned(2809, 12), to_unsigned(2810, 12), to_unsigned(2811, 12), to_unsigned(2812, 12), to_unsigned(2812, 12), to_unsigned(2813, 12), to_unsigned(2814, 12), to_unsigned(2815, 12),
    to_unsigned(2815, 12), to_unsigned(2816, 12), to_unsigned(2817, 12), to_unsigned(2817, 12), to_unsigned(2818, 12), to_unsigned(2819, 12), to_unsigned(2820, 12), to_unsigned(2820, 12),
    to_unsigned(2821, 12), to_unsigned(2822, 12), to_unsigned(2823, 12), to_unsigned(2823, 12), to_unsigned(2824, 12), to_unsigned(2825, 12), to_unsigned(2825, 12), to_unsigned(2826, 12),
    to_unsigned(2827, 12), to_unsigned(2828, 12), to_unsigned(2828, 12), to_unsigned(2829, 12), to_unsigned(2830, 12), to_unsigned(2831, 12), to_unsigned(2831, 12), to_unsigned(2832, 12),
    to_unsigned(2833, 12), to_unsigned(2833, 12), to_unsigned(2834, 12), to_unsigned(2835, 12), to_unsigned(2836, 12), to_unsigned(2836, 12), to_unsigned(2837, 12), to_unsigned(2838, 12),
    to_unsigned(2838, 12), to_unsigned(2839, 12), to_unsigned(2840, 12), to_unsigned(2841, 12), to_unsigned(2841, 12), to_unsigned(2842, 12), to_unsigned(2843, 12), to_unsigned(2844, 12),
    to_unsigned(2844, 12), to_unsigned(2845, 12), to_unsigned(2846, 12), to_unsigned(2846, 12), to_unsigned(2847, 12), to_unsigned(2848, 12), to_unsigned(2849, 12), to_unsigned(2849, 12),
    to_unsigned(2850, 12), to_unsigned(2851, 12), to_unsigned(2851, 12), to_unsigned(2852, 12), to_unsigned(2853, 12), to_unsigned(2854, 12), to_unsigned(2854, 12), to_unsigned(2855, 12),
    to_unsigned(2856, 12), to_unsigned(2856, 12), to_unsigned(2857, 12), to_unsigned(2858, 12), to_unsigned(2859, 12), to_unsigned(2859, 12), to_unsigned(2860, 12), to_unsigned(2861, 12),
    to_unsigned(2861, 12), to_unsigned(2862, 12), to_unsigned(2863, 12), to_unsigned(2864, 12), to_unsigned(2864, 12), to_unsigned(2865, 12), to_unsigned(2866, 12), to_unsigned(2866, 12),
    to_unsigned(2867, 12), to_unsigned(2868, 12), to_unsigned(2869, 12), to_unsigned(2869, 12), to_unsigned(2870, 12), to_unsigned(2871, 12), to_unsigned(2871, 12), to_unsigned(2872, 12),
    to_unsigned(2873, 12), to_unsigned(2874, 12), to_unsigned(2874, 12), to_unsigned(2875, 12), to_unsigned(2876, 12), to_unsigned(2876, 12), to_unsigned(2877, 12), to_unsigned(2878, 12),
    to_unsigned(2879, 12), to_unsigned(2879, 12), to_unsigned(2880, 12), to_unsigned(2881, 12), to_unsigned(2881, 12), to_unsigned(2882, 12), to_unsigned(2883, 12), to_unsigned(2884, 12),
    to_unsigned(2884, 12), to_unsigned(2885, 12), to_unsigned(2886, 12), to_unsigned(2886, 12), to_unsigned(2887, 12), to_unsigned(2888, 12), to_unsigned(2889, 12), to_unsigned(2889, 12),
    to_unsigned(2890, 12), to_unsigned(2891, 12), to_unsigned(2891, 12), to_unsigned(2892, 12), to_unsigned(2893, 12), to_unsigned(2893, 12), to_unsigned(2894, 12), to_unsigned(2895, 12),
    to_unsigned(2896, 12), to_unsigned(2896, 12), to_unsigned(2897, 12), to_unsigned(2898, 12), to_unsigned(2898, 12), to_unsigned(2899, 12), to_unsigned(2900, 12), to_unsigned(2901, 12),
    to_unsigned(2901, 12), to_unsigned(2902, 12), to_unsigned(2903, 12), to_unsigned(2903, 12), to_unsigned(2904, 12), to_unsigned(2905, 12), to_unsigned(2905, 12), to_unsigned(2906, 12),
    to_unsigned(2907, 12), to_unsigned(2908, 12), to_unsigned(2908, 12), to_unsigned(2909, 12), to_unsigned(2910, 12), to_unsigned(2910, 12), to_unsigned(2911, 12), to_unsigned(2912, 12),
    to_unsigned(2913, 12), to_unsigned(2913, 12), to_unsigned(2914, 12), to_unsigned(2915, 12), to_unsigned(2915, 12), to_unsigned(2916, 12), to_unsigned(2917, 12), to_unsigned(2917, 12),
    to_unsigned(2918, 12), to_unsigned(2919, 12), to_unsigned(2920, 12), to_unsigned(2920, 12), to_unsigned(2921, 12), to_unsigned(2922, 12), to_unsigned(2922, 12), to_unsigned(2923, 12),
    to_unsigned(2924, 12), to_unsigned(2924, 12), to_unsigned(2925, 12), to_unsigned(2926, 12), to_unsigned(2927, 12), to_unsigned(2927, 12), to_unsigned(2928, 12), to_unsigned(2929, 12),
    to_unsigned(2929, 12), to_unsigned(2930, 12), to_unsigned(2931, 12), to_unsigned(2931, 12), to_unsigned(2932, 12), to_unsigned(2933, 12), to_unsigned(2934, 12), to_unsigned(2934, 12),
    to_unsigned(2935, 12), to_unsigned(2936, 12), to_unsigned(2936, 12), to_unsigned(2937, 12), to_unsigned(2938, 12), to_unsigned(2938, 12), to_unsigned(2939, 12), to_unsigned(2940, 12),
    to_unsigned(2940, 12), to_unsigned(2941, 12), to_unsigned(2942, 12), to_unsigned(2943, 12), to_unsigned(2943, 12), to_unsigned(2944, 12), to_unsigned(2945, 12), to_unsigned(2945, 12),
    to_unsigned(2946, 12), to_unsigned(2947, 12), to_unsigned(2947, 12), to_unsigned(2948, 12), to_unsigned(2949, 12), to_unsigned(2950, 12), to_unsigned(2950, 12), to_unsigned(2951, 12),
    to_unsigned(2952, 12), to_unsigned(2952, 12), to_unsigned(2953, 12), to_unsigned(2954, 12), to_unsigned(2954, 12), to_unsigned(2955, 12), to_unsigned(2956, 12), to_unsigned(2956, 12),
    to_unsigned(2957, 12), to_unsigned(2958, 12), to_unsigned(2959, 12), to_unsigned(2959, 12), to_unsigned(2960, 12), to_unsigned(2961, 12), to_unsigned(2961, 12), to_unsigned(2962, 12),
    to_unsigned(2963, 12), to_unsigned(2963, 12), to_unsigned(2964, 12), to_unsigned(2965, 12), to_unsigned(2965, 12), to_unsigned(2966, 12), to_unsigned(2967, 12), to_unsigned(2968, 12),
    to_unsigned(2968, 12), to_unsigned(2969, 12), to_unsigned(2970, 12), to_unsigned(2970, 12), to_unsigned(2971, 12), to_unsigned(2972, 12), to_unsigned(2972, 12), to_unsigned(2973, 12),
    to_unsigned(2974, 12), to_unsigned(2974, 12), to_unsigned(2975, 12), to_unsigned(2976, 12), to_unsigned(2976, 12), to_unsigned(2977, 12), to_unsigned(2978, 12), to_unsigned(2979, 12),
    to_unsigned(2979, 12), to_unsigned(2980, 12), to_unsigned(2981, 12), to_unsigned(2981, 12), to_unsigned(2982, 12), to_unsigned(2983, 12), to_unsigned(2983, 12), to_unsigned(2984, 12),
    to_unsigned(2985, 12), to_unsigned(2985, 12), to_unsigned(2986, 12), to_unsigned(2987, 12), to_unsigned(2987, 12), to_unsigned(2988, 12), to_unsigned(2989, 12), to_unsigned(2990, 12),
    to_unsigned(2990, 12), to_unsigned(2991, 12), to_unsigned(2992, 12), to_unsigned(2992, 12), to_unsigned(2993, 12), to_unsigned(2994, 12), to_unsigned(2994, 12), to_unsigned(2995, 12),
    to_unsigned(2996, 12), to_unsigned(2996, 12), to_unsigned(2997, 12), to_unsigned(2998, 12), to_unsigned(2998, 12), to_unsigned(2999, 12), to_unsigned(3000, 12), to_unsigned(3000, 12),
    to_unsigned(3001, 12), to_unsigned(3002, 12), to_unsigned(3002, 12), to_unsigned(3003, 12), to_unsigned(3004, 12), to_unsigned(3005, 12), to_unsigned(3005, 12), to_unsigned(3006, 12),
    to_unsigned(3007, 12), to_unsigned(3007, 12), to_unsigned(3008, 12), to_unsigned(3009, 12), to_unsigned(3009, 12), to_unsigned(3010, 12), to_unsigned(3011, 12), to_unsigned(3011, 12),
    to_unsigned(3012, 12), to_unsigned(3013, 12), to_unsigned(3013, 12), to_unsigned(3014, 12), to_unsigned(3015, 12), to_unsigned(3015, 12), to_unsigned(3016, 12), to_unsigned(3017, 12),
    to_unsigned(3017, 12), to_unsigned(3018, 12), to_unsigned(3019, 12), to_unsigned(3019, 12), to_unsigned(3020, 12), to_unsigned(3021, 12), to_unsigned(3022, 12), to_unsigned(3022, 12),
    to_unsigned(3023, 12), to_unsigned(3024, 12), to_unsigned(3024, 12), to_unsigned(3025, 12), to_unsigned(3026, 12), to_unsigned(3026, 12), to_unsigned(3027, 12), to_unsigned(3028, 12),
    to_unsigned(3028, 12), to_unsigned(3029, 12), to_unsigned(3030, 12), to_unsigned(3030, 12), to_unsigned(3031, 12), to_unsigned(3032, 12), to_unsigned(3032, 12), to_unsigned(3033, 12),
    to_unsigned(3034, 12), to_unsigned(3034, 12), to_unsigned(3035, 12), to_unsigned(3036, 12), to_unsigned(3036, 12), to_unsigned(3037, 12), to_unsigned(3038, 12), to_unsigned(3038, 12),
    to_unsigned(3039, 12), to_unsigned(3040, 12), to_unsigned(3040, 12), to_unsigned(3041, 12), to_unsigned(3042, 12), to_unsigned(3042, 12), to_unsigned(3043, 12), to_unsigned(3044, 12),
    to_unsigned(3044, 12), to_unsigned(3045, 12), to_unsigned(3046, 12), to_unsigned(3046, 12), to_unsigned(3047, 12), to_unsigned(3048, 12), to_unsigned(3049, 12), to_unsigned(3049, 12),
    to_unsigned(3050, 12), to_unsigned(3051, 12), to_unsigned(3051, 12), to_unsigned(3052, 12), to_unsigned(3053, 12), to_unsigned(3053, 12), to_unsigned(3054, 12), to_unsigned(3055, 12),
    to_unsigned(3055, 12), to_unsigned(3056, 12), to_unsigned(3057, 12), to_unsigned(3057, 12), to_unsigned(3058, 12), to_unsigned(3059, 12), to_unsigned(3059, 12), to_unsigned(3060, 12),
    to_unsigned(3061, 12), to_unsigned(3061, 12), to_unsigned(3062, 12), to_unsigned(3063, 12), to_unsigned(3063, 12), to_unsigned(3064, 12), to_unsigned(3065, 12), to_unsigned(3065, 12),
    to_unsigned(3066, 12), to_unsigned(3067, 12), to_unsigned(3067, 12), to_unsigned(3068, 12), to_unsigned(3069, 12), to_unsigned(3069, 12), to_unsigned(3070, 12), to_unsigned(3071, 12),
    to_unsigned(3071, 12), to_unsigned(3072, 12), to_unsigned(3073, 12), to_unsigned(3073, 12), to_unsigned(3074, 12), to_unsigned(3075, 12), to_unsigned(3075, 12), to_unsigned(3076, 12),
    to_unsigned(3077, 12), to_unsigned(3077, 12), to_unsigned(3078, 12), to_unsigned(3079, 12), to_unsigned(3079, 12), to_unsigned(3080, 12), to_unsigned(3081, 12), to_unsigned(3081, 12),
    to_unsigned(3082, 12), to_unsigned(3083, 12), to_unsigned(3083, 12), to_unsigned(3084, 12), to_unsigned(3085, 12), to_unsigned(3085, 12), to_unsigned(3086, 12), to_unsigned(3087, 12),
    to_unsigned(3087, 12), to_unsigned(3088, 12), to_unsigned(3089, 12), to_unsigned(3089, 12), to_unsigned(3090, 12), to_unsigned(3091, 12), to_unsigned(3091, 12), to_unsigned(3092, 12),
    to_unsigned(3093, 12), to_unsigned(3093, 12), to_unsigned(3094, 12), to_unsigned(3094, 12), to_unsigned(3095, 12), to_unsigned(3096, 12), to_unsigned(3096, 12), to_unsigned(3097, 12),
    to_unsigned(3098, 12), to_unsigned(3098, 12), to_unsigned(3099, 12), to_unsigned(3100, 12), to_unsigned(3100, 12), to_unsigned(3101, 12), to_unsigned(3102, 12), to_unsigned(3102, 12),
    to_unsigned(3103, 12), to_unsigned(3104, 12), to_unsigned(3104, 12), to_unsigned(3105, 12), to_unsigned(3106, 12), to_unsigned(3106, 12), to_unsigned(3107, 12), to_unsigned(3108, 12),
    to_unsigned(3108, 12), to_unsigned(3109, 12), to_unsigned(3110, 12), to_unsigned(3110, 12), to_unsigned(3111, 12), to_unsigned(3112, 12), to_unsigned(3112, 12), to_unsigned(3113, 12),
    to_unsigned(3114, 12), to_unsigned(3114, 12), to_unsigned(3115, 12), to_unsigned(3116, 12), to_unsigned(3116, 12), to_unsigned(3117, 12), to_unsigned(3118, 12), to_unsigned(3118, 12),
    to_unsigned(3119, 12), to_unsigned(3120, 12), to_unsigned(3120, 12), to_unsigned(3121, 12), to_unsigned(3121, 12), to_unsigned(3122, 12), to_unsigned(3123, 12), to_unsigned(3123, 12),
    to_unsigned(3124, 12), to_unsigned(3125, 12), to_unsigned(3125, 12), to_unsigned(3126, 12), to_unsigned(3127, 12), to_unsigned(3127, 12), to_unsigned(3128, 12), to_unsigned(3129, 12),
    to_unsigned(3129, 12), to_unsigned(3130, 12), to_unsigned(3131, 12), to_unsigned(3131, 12), to_unsigned(3132, 12), to_unsigned(3133, 12), to_unsigned(3133, 12), to_unsigned(3134, 12),
    to_unsigned(3135, 12), to_unsigned(3135, 12), to_unsigned(3136, 12), to_unsigned(3137, 12), to_unsigned(3137, 12), to_unsigned(3138, 12), to_unsigned(3138, 12), to_unsigned(3139, 12),
    to_unsigned(3140, 12), to_unsigned(3140, 12), to_unsigned(3141, 12), to_unsigned(3142, 12), to_unsigned(3142, 12), to_unsigned(3143, 12), to_unsigned(3144, 12), to_unsigned(3144, 12),
    to_unsigned(3145, 12), to_unsigned(3146, 12), to_unsigned(3146, 12), to_unsigned(3147, 12), to_unsigned(3148, 12), to_unsigned(3148, 12), to_unsigned(3149, 12), to_unsigned(3150, 12),
    to_unsigned(3150, 12), to_unsigned(3151, 12), to_unsigned(3152, 12), to_unsigned(3152, 12), to_unsigned(3153, 12), to_unsigned(3153, 12), to_unsigned(3154, 12), to_unsigned(3155, 12),
    to_unsigned(3155, 12), to_unsigned(3156, 12), to_unsigned(3157, 12), to_unsigned(3157, 12), to_unsigned(3158, 12), to_unsigned(3159, 12), to_unsigned(3159, 12), to_unsigned(3160, 12),
    to_unsigned(3161, 12), to_unsigned(3161, 12), to_unsigned(3162, 12), to_unsigned(3163, 12), to_unsigned(3163, 12), to_unsigned(3164, 12), to_unsigned(3164, 12), to_unsigned(3165, 12),
    to_unsigned(3166, 12), to_unsigned(3166, 12), to_unsigned(3167, 12), to_unsigned(3168, 12), to_unsigned(3168, 12), to_unsigned(3169, 12), to_unsigned(3170, 12), to_unsigned(3170, 12),
    to_unsigned(3171, 12), to_unsigned(3172, 12), to_unsigned(3172, 12), to_unsigned(3173, 12), to_unsigned(3174, 12), to_unsigned(3174, 12), to_unsigned(3175, 12), to_unsigned(3175, 12),
    to_unsigned(3176, 12), to_unsigned(3177, 12), to_unsigned(3177, 12), to_unsigned(3178, 12), to_unsigned(3179, 12), to_unsigned(3179, 12), to_unsigned(3180, 12), to_unsigned(3181, 12),
    to_unsigned(3181, 12), to_unsigned(3182, 12), to_unsigned(3183, 12), to_unsigned(3183, 12), to_unsigned(3184, 12), to_unsigned(3184, 12), to_unsigned(3185, 12), to_unsigned(3186, 12),
    to_unsigned(3186, 12), to_unsigned(3187, 12), to_unsigned(3188, 12), to_unsigned(3188, 12), to_unsigned(3189, 12), to_unsigned(3190, 12), to_unsigned(3190, 12), to_unsigned(3191, 12),
    to_unsigned(3192, 12), to_unsigned(3192, 12), to_unsigned(3193, 12), to_unsigned(3193, 12), to_unsigned(3194, 12), to_unsigned(3195, 12), to_unsigned(3195, 12), to_unsigned(3196, 12),
    to_unsigned(3197, 12), to_unsigned(3197, 12), to_unsigned(3198, 12), to_unsigned(3199, 12), to_unsigned(3199, 12), to_unsigned(3200, 12), to_unsigned(3200, 12), to_unsigned(3201, 12),
    to_unsigned(3202, 12), to_unsigned(3202, 12), to_unsigned(3203, 12), to_unsigned(3204, 12), to_unsigned(3204, 12), to_unsigned(3205, 12), to_unsigned(3206, 12), to_unsigned(3206, 12),
    to_unsigned(3207, 12), to_unsigned(3208, 12), to_unsigned(3208, 12), to_unsigned(3209, 12), to_unsigned(3209, 12), to_unsigned(3210, 12), to_unsigned(3211, 12), to_unsigned(3211, 12),
    to_unsigned(3212, 12), to_unsigned(3213, 12), to_unsigned(3213, 12), to_unsigned(3214, 12), to_unsigned(3215, 12), to_unsigned(3215, 12), to_unsigned(3216, 12), to_unsigned(3216, 12),
    to_unsigned(3217, 12), to_unsigned(3218, 12), to_unsigned(3218, 12), to_unsigned(3219, 12), to_unsigned(3220, 12), to_unsigned(3220, 12), to_unsigned(3221, 12), to_unsigned(3222, 12),
    to_unsigned(3222, 12), to_unsigned(3223, 12), to_unsigned(3223, 12), to_unsigned(3224, 12), to_unsigned(3225, 12), to_unsigned(3225, 12), to_unsigned(3226, 12), to_unsigned(3227, 12),
    to_unsigned(3227, 12), to_unsigned(3228, 12), to_unsigned(3229, 12), to_unsigned(3229, 12), to_unsigned(3230, 12), to_unsigned(3230, 12), to_unsigned(3231, 12), to_unsigned(3232, 12),
    to_unsigned(3232, 12), to_unsigned(3233, 12), to_unsigned(3234, 12), to_unsigned(3234, 12), to_unsigned(3235, 12), to_unsigned(3235, 12), to_unsigned(3236, 12), to_unsigned(3237, 12),
    to_unsigned(3237, 12), to_unsigned(3238, 12), to_unsigned(3239, 12), to_unsigned(3239, 12), to_unsigned(3240, 12), to_unsigned(3241, 12), to_unsigned(3241, 12), to_unsigned(3242, 12),
    to_unsigned(3242, 12), to_unsigned(3243, 12), to_unsigned(3244, 12), to_unsigned(3244, 12), to_unsigned(3245, 12), to_unsigned(3246, 12), to_unsigned(3246, 12), to_unsigned(3247, 12),
    to_unsigned(3247, 12), to_unsigned(3248, 12), to_unsigned(3249, 12), to_unsigned(3249, 12), to_unsigned(3250, 12), to_unsigned(3251, 12), to_unsigned(3251, 12), to_unsigned(3252, 12),
    to_unsigned(3253, 12), to_unsigned(3253, 12), to_unsigned(3254, 12), to_unsigned(3254, 12), to_unsigned(3255, 12), to_unsigned(3256, 12), to_unsigned(3256, 12), to_unsigned(3257, 12),
    to_unsigned(3258, 12), to_unsigned(3258, 12), to_unsigned(3259, 12), to_unsigned(3259, 12), to_unsigned(3260, 12), to_unsigned(3261, 12), to_unsigned(3261, 12), to_unsigned(3262, 12),
    to_unsigned(3263, 12), to_unsigned(3263, 12), to_unsigned(3264, 12), to_unsigned(3264, 12), to_unsigned(3265, 12), to_unsigned(3266, 12), to_unsigned(3266, 12), to_unsigned(3267, 12),
    to_unsigned(3268, 12), to_unsigned(3268, 12), to_unsigned(3269, 12), to_unsigned(3269, 12), to_unsigned(3270, 12), to_unsigned(3271, 12), to_unsigned(3271, 12), to_unsigned(3272, 12),
    to_unsigned(3273, 12), to_unsigned(3273, 12), to_unsigned(3274, 12), to_unsigned(3274, 12), to_unsigned(3275, 12), to_unsigned(3276, 12), to_unsigned(3276, 12), to_unsigned(3277, 12),
    to_unsigned(3278, 12), to_unsigned(3278, 12), to_unsigned(3279, 12), to_unsigned(3279, 12), to_unsigned(3280, 12), to_unsigned(3281, 12), to_unsigned(3281, 12), to_unsigned(3282, 12),
    to_unsigned(3283, 12), to_unsigned(3283, 12), to_unsigned(3284, 12), to_unsigned(3284, 12), to_unsigned(3285, 12), to_unsigned(3286, 12), to_unsigned(3286, 12), to_unsigned(3287, 12),
    to_unsigned(3288, 12), to_unsigned(3288, 12), to_unsigned(3289, 12), to_unsigned(3289, 12), to_unsigned(3290, 12), to_unsigned(3291, 12), to_unsigned(3291, 12), to_unsigned(3292, 12),
    to_unsigned(3293, 12), to_unsigned(3293, 12), to_unsigned(3294, 12), to_unsigned(3294, 12), to_unsigned(3295, 12), to_unsigned(3296, 12), to_unsigned(3296, 12), to_unsigned(3297, 12),
    to_unsigned(3298, 12), to_unsigned(3298, 12), to_unsigned(3299, 12), to_unsigned(3299, 12), to_unsigned(3300, 12), to_unsigned(3301, 12), to_unsigned(3301, 12), to_unsigned(3302, 12),
    to_unsigned(3302, 12), to_unsigned(3303, 12), to_unsigned(3304, 12), to_unsigned(3304, 12), to_unsigned(3305, 12), to_unsigned(3306, 12), to_unsigned(3306, 12), to_unsigned(3307, 12),
    to_unsigned(3307, 12), to_unsigned(3308, 12), to_unsigned(3309, 12), to_unsigned(3309, 12), to_unsigned(3310, 12), to_unsigned(3311, 12), to_unsigned(3311, 12), to_unsigned(3312, 12),
    to_unsigned(3312, 12), to_unsigned(3313, 12), to_unsigned(3314, 12), to_unsigned(3314, 12), to_unsigned(3315, 12), to_unsigned(3315, 12), to_unsigned(3316, 12), to_unsigned(3317, 12),
    to_unsigned(3317, 12), to_unsigned(3318, 12), to_unsigned(3319, 12), to_unsigned(3319, 12), to_unsigned(3320, 12), to_unsigned(3320, 12), to_unsigned(3321, 12), to_unsigned(3322, 12),
    to_unsigned(3322, 12), to_unsigned(3323, 12), to_unsigned(3323, 12), to_unsigned(3324, 12), to_unsigned(3325, 12), to_unsigned(3325, 12), to_unsigned(3326, 12), to_unsigned(3327, 12),
    to_unsigned(3327, 12), to_unsigned(3328, 12), to_unsigned(3328, 12), to_unsigned(3329, 12), to_unsigned(3330, 12), to_unsigned(3330, 12), to_unsigned(3331, 12), to_unsigned(3331, 12),
    to_unsigned(3332, 12), to_unsigned(3333, 12), to_unsigned(3333, 12), to_unsigned(3334, 12), to_unsigned(3335, 12), to_unsigned(3335, 12), to_unsigned(3336, 12), to_unsigned(3336, 12),
    to_unsigned(3337, 12), to_unsigned(3338, 12), to_unsigned(3338, 12), to_unsigned(3339, 12), to_unsigned(3339, 12), to_unsigned(3340, 12), to_unsigned(3341, 12), to_unsigned(3341, 12),
    to_unsigned(3342, 12), to_unsigned(3343, 12), to_unsigned(3343, 12), to_unsigned(3344, 12), to_unsigned(3344, 12), to_unsigned(3345, 12), to_unsigned(3346, 12), to_unsigned(3346, 12),
    to_unsigned(3347, 12), to_unsigned(3347, 12), to_unsigned(3348, 12), to_unsigned(3349, 12), to_unsigned(3349, 12), to_unsigned(3350, 12), to_unsigned(3350, 12), to_unsigned(3351, 12),
    to_unsigned(3352, 12), to_unsigned(3352, 12), to_unsigned(3353, 12), to_unsigned(3354, 12), to_unsigned(3354, 12), to_unsigned(3355, 12), to_unsigned(3355, 12), to_unsigned(3356, 12),
    to_unsigned(3357, 12), to_unsigned(3357, 12), to_unsigned(3358, 12), to_unsigned(3358, 12), to_unsigned(3359, 12), to_unsigned(3360, 12), to_unsigned(3360, 12), to_unsigned(3361, 12),
    to_unsigned(3361, 12), to_unsigned(3362, 12), to_unsigned(3363, 12), to_unsigned(3363, 12), to_unsigned(3364, 12), to_unsigned(3365, 12), to_unsigned(3365, 12), to_unsigned(3366, 12),
    to_unsigned(3366, 12), to_unsigned(3367, 12), to_unsigned(3368, 12), to_unsigned(3368, 12), to_unsigned(3369, 12), to_unsigned(3369, 12), to_unsigned(3370, 12), to_unsigned(3371, 12),
    to_unsigned(3371, 12), to_unsigned(3372, 12), to_unsigned(3372, 12), to_unsigned(3373, 12), to_unsigned(3374, 12), to_unsigned(3374, 12), to_unsigned(3375, 12), to_unsigned(3375, 12),
    to_unsigned(3376, 12), to_unsigned(3377, 12), to_unsigned(3377, 12), to_unsigned(3378, 12), to_unsigned(3378, 12), to_unsigned(3379, 12), to_unsigned(3380, 12), to_unsigned(3380, 12),
    to_unsigned(3381, 12), to_unsigned(3382, 12), to_unsigned(3382, 12), to_unsigned(3383, 12), to_unsigned(3383, 12), to_unsigned(3384, 12), to_unsigned(3385, 12), to_unsigned(3385, 12),
    to_unsigned(3386, 12), to_unsigned(3386, 12), to_unsigned(3387, 12), to_unsigned(3388, 12), to_unsigned(3388, 12), to_unsigned(3389, 12), to_unsigned(3389, 12), to_unsigned(3390, 12),
    to_unsigned(3391, 12), to_unsigned(3391, 12), to_unsigned(3392, 12), to_unsigned(3392, 12), to_unsigned(3393, 12), to_unsigned(3394, 12), to_unsigned(3394, 12), to_unsigned(3395, 12),
    to_unsigned(3395, 12), to_unsigned(3396, 12), to_unsigned(3397, 12), to_unsigned(3397, 12), to_unsigned(3398, 12), to_unsigned(3398, 12), to_unsigned(3399, 12), to_unsigned(3400, 12),
    to_unsigned(3400, 12), to_unsigned(3401, 12), to_unsigned(3401, 12), to_unsigned(3402, 12), to_unsigned(3403, 12), to_unsigned(3403, 12), to_unsigned(3404, 12), to_unsigned(3404, 12),
    to_unsigned(3405, 12), to_unsigned(3406, 12), to_unsigned(3406, 12), to_unsigned(3407, 12), to_unsigned(3407, 12), to_unsigned(3408, 12), to_unsigned(3409, 12), to_unsigned(3409, 12),
    to_unsigned(3410, 12), to_unsigned(3410, 12), to_unsigned(3411, 12), to_unsigned(3412, 12), to_unsigned(3412, 12), to_unsigned(3413, 12), to_unsigned(3413, 12), to_unsigned(3414, 12),
    to_unsigned(3415, 12), to_unsigned(3415, 12), to_unsigned(3416, 12), to_unsigned(3416, 12), to_unsigned(3417, 12), to_unsigned(3418, 12), to_unsigned(3418, 12), to_unsigned(3419, 12),
    to_unsigned(3419, 12), to_unsigned(3420, 12), to_unsigned(3421, 12), to_unsigned(3421, 12), to_unsigned(3422, 12), to_unsigned(3422, 12), to_unsigned(3423, 12), to_unsigned(3424, 12),
    to_unsigned(3424, 12), to_unsigned(3425, 12), to_unsigned(3425, 12), to_unsigned(3426, 12), to_unsigned(3427, 12), to_unsigned(3427, 12), to_unsigned(3428, 12), to_unsigned(3428, 12),
    to_unsigned(3429, 12), to_unsigned(3430, 12), to_unsigned(3430, 12), to_unsigned(3431, 12), to_unsigned(3431, 12), to_unsigned(3432, 12), to_unsigned(3433, 12), to_unsigned(3433, 12),
    to_unsigned(3434, 12), to_unsigned(3434, 12), to_unsigned(3435, 12), to_unsigned(3436, 12), to_unsigned(3436, 12), to_unsigned(3437, 12), to_unsigned(3437, 12), to_unsigned(3438, 12),
    to_unsigned(3439, 12), to_unsigned(3439, 12), to_unsigned(3440, 12), to_unsigned(3440, 12), to_unsigned(3441, 12), to_unsigned(3442, 12), to_unsigned(3442, 12), to_unsigned(3443, 12),
    to_unsigned(3443, 12), to_unsigned(3444, 12), to_unsigned(3444, 12), to_unsigned(3445, 12), to_unsigned(3446, 12), to_unsigned(3446, 12), to_unsigned(3447, 12), to_unsigned(3447, 12),
    to_unsigned(3448, 12), to_unsigned(3449, 12), to_unsigned(3449, 12), to_unsigned(3450, 12), to_unsigned(3450, 12), to_unsigned(3451, 12), to_unsigned(3452, 12), to_unsigned(3452, 12),
    to_unsigned(3453, 12), to_unsigned(3453, 12), to_unsigned(3454, 12), to_unsigned(3455, 12), to_unsigned(3455, 12), to_unsigned(3456, 12), to_unsigned(3456, 12), to_unsigned(3457, 12),
    to_unsigned(3458, 12), to_unsigned(3458, 12), to_unsigned(3459, 12), to_unsigned(3459, 12), to_unsigned(3460, 12), to_unsigned(3460, 12), to_unsigned(3461, 12), to_unsigned(3462, 12),
    to_unsigned(3462, 12), to_unsigned(3463, 12), to_unsigned(3463, 12), to_unsigned(3464, 12), to_unsigned(3465, 12), to_unsigned(3465, 12), to_unsigned(3466, 12), to_unsigned(3466, 12),
    to_unsigned(3467, 12), to_unsigned(3468, 12), to_unsigned(3468, 12), to_unsigned(3469, 12), to_unsigned(3469, 12), to_unsigned(3470, 12), to_unsigned(3471, 12), to_unsigned(3471, 12),
    to_unsigned(3472, 12), to_unsigned(3472, 12), to_unsigned(3473, 12), to_unsigned(3473, 12), to_unsigned(3474, 12), to_unsigned(3475, 12), to_unsigned(3475, 12), to_unsigned(3476, 12),
    to_unsigned(3476, 12), to_unsigned(3477, 12), to_unsigned(3478, 12), to_unsigned(3478, 12), to_unsigned(3479, 12), to_unsigned(3479, 12), to_unsigned(3480, 12), to_unsigned(3481, 12),
    to_unsigned(3481, 12), to_unsigned(3482, 12), to_unsigned(3482, 12), to_unsigned(3483, 12), to_unsigned(3483, 12), to_unsigned(3484, 12), to_unsigned(3485, 12), to_unsigned(3485, 12),
    to_unsigned(3486, 12), to_unsigned(3486, 12), to_unsigned(3487, 12), to_unsigned(3488, 12), to_unsigned(3488, 12), to_unsigned(3489, 12), to_unsigned(3489, 12), to_unsigned(3490, 12),
    to_unsigned(3491, 12), to_unsigned(3491, 12), to_unsigned(3492, 12), to_unsigned(3492, 12), to_unsigned(3493, 12), to_unsigned(3493, 12), to_unsigned(3494, 12), to_unsigned(3495, 12),
    to_unsigned(3495, 12), to_unsigned(3496, 12), to_unsigned(3496, 12), to_unsigned(3497, 12), to_unsigned(3498, 12), to_unsigned(3498, 12), to_unsigned(3499, 12), to_unsigned(3499, 12),
    to_unsigned(3500, 12), to_unsigned(3500, 12), to_unsigned(3501, 12), to_unsigned(3502, 12), to_unsigned(3502, 12), to_unsigned(3503, 12), to_unsigned(3503, 12), to_unsigned(3504, 12),
    to_unsigned(3505, 12), to_unsigned(3505, 12), to_unsigned(3506, 12), to_unsigned(3506, 12), to_unsigned(3507, 12), to_unsigned(3507, 12), to_unsigned(3508, 12), to_unsigned(3509, 12),
    to_unsigned(3509, 12), to_unsigned(3510, 12), to_unsigned(3510, 12), to_unsigned(3511, 12), to_unsigned(3512, 12), to_unsigned(3512, 12), to_unsigned(3513, 12), to_unsigned(3513, 12),
    to_unsigned(3514, 12), to_unsigned(3514, 12), to_unsigned(3515, 12), to_unsigned(3516, 12), to_unsigned(3516, 12), to_unsigned(3517, 12), to_unsigned(3517, 12), to_unsigned(3518, 12),
    to_unsigned(3519, 12), to_unsigned(3519, 12), to_unsigned(3520, 12), to_unsigned(3520, 12), to_unsigned(3521, 12), to_unsigned(3521, 12), to_unsigned(3522, 12), to_unsigned(3523, 12),
    to_unsigned(3523, 12), to_unsigned(3524, 12), to_unsigned(3524, 12), to_unsigned(3525, 12), to_unsigned(3526, 12), to_unsigned(3526, 12), to_unsigned(3527, 12), to_unsigned(3527, 12),
    to_unsigned(3528, 12), to_unsigned(3528, 12), to_unsigned(3529, 12), to_unsigned(3530, 12), to_unsigned(3530, 12), to_unsigned(3531, 12), to_unsigned(3531, 12), to_unsigned(3532, 12),
    to_unsigned(3532, 12), to_unsigned(3533, 12), to_unsigned(3534, 12), to_unsigned(3534, 12), to_unsigned(3535, 12), to_unsigned(3535, 12), to_unsigned(3536, 12), to_unsigned(3537, 12),
    to_unsigned(3537, 12), to_unsigned(3538, 12), to_unsigned(3538, 12), to_unsigned(3539, 12), to_unsigned(3539, 12), to_unsigned(3540, 12), to_unsigned(3541, 12), to_unsigned(3541, 12),
    to_unsigned(3542, 12), to_unsigned(3542, 12), to_unsigned(3543, 12), to_unsigned(3543, 12), to_unsigned(3544, 12), to_unsigned(3545, 12), to_unsigned(3545, 12), to_unsigned(3546, 12),
    to_unsigned(3546, 12), to_unsigned(3547, 12), to_unsigned(3548, 12), to_unsigned(3548, 12), to_unsigned(3549, 12), to_unsigned(3549, 12), to_unsigned(3550, 12), to_unsigned(3550, 12),
    to_unsigned(3551, 12), to_unsigned(3552, 12), to_unsigned(3552, 12), to_unsigned(3553, 12), to_unsigned(3553, 12), to_unsigned(3554, 12), to_unsigned(3554, 12), to_unsigned(3555, 12),
    to_unsigned(3556, 12), to_unsigned(3556, 12), to_unsigned(3557, 12), to_unsigned(3557, 12), to_unsigned(3558, 12), to_unsigned(3558, 12), to_unsigned(3559, 12), to_unsigned(3560, 12),
    to_unsigned(3560, 12), to_unsigned(3561, 12), to_unsigned(3561, 12), to_unsigned(3562, 12), to_unsigned(3562, 12), to_unsigned(3563, 12), to_unsigned(3564, 12), to_unsigned(3564, 12),
    to_unsigned(3565, 12), to_unsigned(3565, 12), to_unsigned(3566, 12), to_unsigned(3567, 12), to_unsigned(3567, 12), to_unsigned(3568, 12), to_unsigned(3568, 12), to_unsigned(3569, 12),
    to_unsigned(3569, 12), to_unsigned(3570, 12), to_unsigned(3571, 12), to_unsigned(3571, 12), to_unsigned(3572, 12), to_unsigned(3572, 12), to_unsigned(3573, 12), to_unsigned(3573, 12),
    to_unsigned(3574, 12), to_unsigned(3575, 12), to_unsigned(3575, 12), to_unsigned(3576, 12), to_unsigned(3576, 12), to_unsigned(3577, 12), to_unsigned(3577, 12), to_unsigned(3578, 12),
    to_unsigned(3579, 12), to_unsigned(3579, 12), to_unsigned(3580, 12), to_unsigned(3580, 12), to_unsigned(3581, 12), to_unsigned(3581, 12), to_unsigned(3582, 12), to_unsigned(3583, 12),
    to_unsigned(3583, 12), to_unsigned(3584, 12), to_unsigned(3584, 12), to_unsigned(3585, 12), to_unsigned(3585, 12), to_unsigned(3586, 12), to_unsigned(3587, 12), to_unsigned(3587, 12),
    to_unsigned(3588, 12), to_unsigned(3588, 12), to_unsigned(3589, 12), to_unsigned(3589, 12), to_unsigned(3590, 12), to_unsigned(3591, 12), to_unsigned(3591, 12), to_unsigned(3592, 12),
    to_unsigned(3592, 12), to_unsigned(3593, 12), to_unsigned(3593, 12), to_unsigned(3594, 12), to_unsigned(3595, 12), to_unsigned(3595, 12), to_unsigned(3596, 12), to_unsigned(3596, 12),
    to_unsigned(3597, 12), to_unsigned(3597, 12), to_unsigned(3598, 12), to_unsigned(3599, 12), to_unsigned(3599, 12), to_unsigned(3600, 12), to_unsigned(3600, 12), to_unsigned(3601, 12),
    to_unsigned(3601, 12), to_unsigned(3602, 12), to_unsigned(3602, 12), to_unsigned(3603, 12), to_unsigned(3604, 12), to_unsigned(3604, 12), to_unsigned(3605, 12), to_unsigned(3605, 12),
    to_unsigned(3606, 12), to_unsigned(3606, 12), to_unsigned(3607, 12), to_unsigned(3608, 12), to_unsigned(3608, 12), to_unsigned(3609, 12), to_unsigned(3609, 12), to_unsigned(3610, 12),
    to_unsigned(3610, 12), to_unsigned(3611, 12), to_unsigned(3612, 12), to_unsigned(3612, 12), to_unsigned(3613, 12), to_unsigned(3613, 12), to_unsigned(3614, 12), to_unsigned(3614, 12),
    to_unsigned(3615, 12), to_unsigned(3616, 12), to_unsigned(3616, 12), to_unsigned(3617, 12), to_unsigned(3617, 12), to_unsigned(3618, 12), to_unsigned(3618, 12), to_unsigned(3619, 12),
    to_unsigned(3620, 12), to_unsigned(3620, 12), to_unsigned(3621, 12), to_unsigned(3621, 12), to_unsigned(3622, 12), to_unsigned(3622, 12), to_unsigned(3623, 12), to_unsigned(3623, 12),
    to_unsigned(3624, 12), to_unsigned(3625, 12), to_unsigned(3625, 12), to_unsigned(3626, 12), to_unsigned(3626, 12), to_unsigned(3627, 12), to_unsigned(3627, 12), to_unsigned(3628, 12),
    to_unsigned(3629, 12), to_unsigned(3629, 12), to_unsigned(3630, 12), to_unsigned(3630, 12), to_unsigned(3631, 12), to_unsigned(3631, 12), to_unsigned(3632, 12), to_unsigned(3632, 12),
    to_unsigned(3633, 12), to_unsigned(3634, 12), to_unsigned(3634, 12), to_unsigned(3635, 12), to_unsigned(3635, 12), to_unsigned(3636, 12), to_unsigned(3636, 12), to_unsigned(3637, 12),
    to_unsigned(3638, 12), to_unsigned(3638, 12), to_unsigned(3639, 12), to_unsigned(3639, 12), to_unsigned(3640, 12), to_unsigned(3640, 12), to_unsigned(3641, 12), to_unsigned(3641, 12),
    to_unsigned(3642, 12), to_unsigned(3643, 12), to_unsigned(3643, 12), to_unsigned(3644, 12), to_unsigned(3644, 12), to_unsigned(3645, 12), to_unsigned(3645, 12), to_unsigned(3646, 12),
    to_unsigned(3647, 12), to_unsigned(3647, 12), to_unsigned(3648, 12), to_unsigned(3648, 12), to_unsigned(3649, 12), to_unsigned(3649, 12), to_unsigned(3650, 12), to_unsigned(3650, 12),
    to_unsigned(3651, 12), to_unsigned(3652, 12), to_unsigned(3652, 12), to_unsigned(3653, 12), to_unsigned(3653, 12), to_unsigned(3654, 12), to_unsigned(3654, 12), to_unsigned(3655, 12),
    to_unsigned(3656, 12), to_unsigned(3656, 12), to_unsigned(3657, 12), to_unsigned(3657, 12), to_unsigned(3658, 12), to_unsigned(3658, 12), to_unsigned(3659, 12), to_unsigned(3659, 12),
    to_unsigned(3660, 12), to_unsigned(3661, 12), to_unsigned(3661, 12), to_unsigned(3662, 12), to_unsigned(3662, 12), to_unsigned(3663, 12), to_unsigned(3663, 12), to_unsigned(3664, 12),
    to_unsigned(3664, 12), to_unsigned(3665, 12), to_unsigned(3666, 12), to_unsigned(3666, 12), to_unsigned(3667, 12), to_unsigned(3667, 12), to_unsigned(3668, 12), to_unsigned(3668, 12),
    to_unsigned(3669, 12), to_unsigned(3669, 12), to_unsigned(3670, 12), to_unsigned(3671, 12), to_unsigned(3671, 12), to_unsigned(3672, 12), to_unsigned(3672, 12), to_unsigned(3673, 12),
    to_unsigned(3673, 12), to_unsigned(3674, 12), to_unsigned(3675, 12), to_unsigned(3675, 12), to_unsigned(3676, 12), to_unsigned(3676, 12), to_unsigned(3677, 12), to_unsigned(3677, 12),
    to_unsigned(3678, 12), to_unsigned(3678, 12), to_unsigned(3679, 12), to_unsigned(3680, 12), to_unsigned(3680, 12), to_unsigned(3681, 12), to_unsigned(3681, 12), to_unsigned(3682, 12),
    to_unsigned(3682, 12), to_unsigned(3683, 12), to_unsigned(3683, 12), to_unsigned(3684, 12), to_unsigned(3685, 12), to_unsigned(3685, 12), to_unsigned(3686, 12), to_unsigned(3686, 12),
    to_unsigned(3687, 12), to_unsigned(3687, 12), to_unsigned(3688, 12), to_unsigned(3688, 12), to_unsigned(3689, 12), to_unsigned(3690, 12), to_unsigned(3690, 12), to_unsigned(3691, 12),
    to_unsigned(3691, 12), to_unsigned(3692, 12), to_unsigned(3692, 12), to_unsigned(3693, 12), to_unsigned(3693, 12), to_unsigned(3694, 12), to_unsigned(3695, 12), to_unsigned(3695, 12),
    to_unsigned(3696, 12), to_unsigned(3696, 12), to_unsigned(3697, 12), to_unsigned(3697, 12), to_unsigned(3698, 12), to_unsigned(3698, 12), to_unsigned(3699, 12), to_unsigned(3699, 12),
    to_unsigned(3700, 12), to_unsigned(3701, 12), to_unsigned(3701, 12), to_unsigned(3702, 12), to_unsigned(3702, 12), to_unsigned(3703, 12), to_unsigned(3703, 12), to_unsigned(3704, 12),
    to_unsigned(3704, 12), to_unsigned(3705, 12), to_unsigned(3706, 12), to_unsigned(3706, 12), to_unsigned(3707, 12), to_unsigned(3707, 12), to_unsigned(3708, 12), to_unsigned(3708, 12),
    to_unsigned(3709, 12), to_unsigned(3709, 12), to_unsigned(3710, 12), to_unsigned(3711, 12), to_unsigned(3711, 12), to_unsigned(3712, 12), to_unsigned(3712, 12), to_unsigned(3713, 12),
    to_unsigned(3713, 12), to_unsigned(3714, 12), to_unsigned(3714, 12), to_unsigned(3715, 12), to_unsigned(3716, 12), to_unsigned(3716, 12), to_unsigned(3717, 12), to_unsigned(3717, 12),
    to_unsigned(3718, 12), to_unsigned(3718, 12), to_unsigned(3719, 12), to_unsigned(3719, 12), to_unsigned(3720, 12), to_unsigned(3720, 12), to_unsigned(3721, 12), to_unsigned(3722, 12),
    to_unsigned(3722, 12), to_unsigned(3723, 12), to_unsigned(3723, 12), to_unsigned(3724, 12), to_unsigned(3724, 12), to_unsigned(3725, 12), to_unsigned(3725, 12), to_unsigned(3726, 12),
    to_unsigned(3727, 12), to_unsigned(3727, 12), to_unsigned(3728, 12), to_unsigned(3728, 12), to_unsigned(3729, 12), to_unsigned(3729, 12), to_unsigned(3730, 12), to_unsigned(3730, 12),
    to_unsigned(3731, 12), to_unsigned(3731, 12), to_unsigned(3732, 12), to_unsigned(3733, 12), to_unsigned(3733, 12), to_unsigned(3734, 12), to_unsigned(3734, 12), to_unsigned(3735, 12),
    to_unsigned(3735, 12), to_unsigned(3736, 12), to_unsigned(3736, 12), to_unsigned(3737, 12), to_unsigned(3737, 12), to_unsigned(3738, 12), to_unsigned(3739, 12), to_unsigned(3739, 12),
    to_unsigned(3740, 12), to_unsigned(3740, 12), to_unsigned(3741, 12), to_unsigned(3741, 12), to_unsigned(3742, 12), to_unsigned(3742, 12), to_unsigned(3743, 12), to_unsigned(3743, 12),
    to_unsigned(3744, 12), to_unsigned(3745, 12), to_unsigned(3745, 12), to_unsigned(3746, 12), to_unsigned(3746, 12), to_unsigned(3747, 12), to_unsigned(3747, 12), to_unsigned(3748, 12),
    to_unsigned(3748, 12), to_unsigned(3749, 12), to_unsigned(3750, 12), to_unsigned(3750, 12), to_unsigned(3751, 12), to_unsigned(3751, 12), to_unsigned(3752, 12), to_unsigned(3752, 12),
    to_unsigned(3753, 12), to_unsigned(3753, 12), to_unsigned(3754, 12), to_unsigned(3754, 12), to_unsigned(3755, 12), to_unsigned(3756, 12), to_unsigned(3756, 12), to_unsigned(3757, 12),
    to_unsigned(3757, 12), to_unsigned(3758, 12), to_unsigned(3758, 12), to_unsigned(3759, 12), to_unsigned(3759, 12), to_unsigned(3760, 12), to_unsigned(3760, 12), to_unsigned(3761, 12),
    to_unsigned(3761, 12), to_unsigned(3762, 12), to_unsigned(3763, 12), to_unsigned(3763, 12), to_unsigned(3764, 12), to_unsigned(3764, 12), to_unsigned(3765, 12), to_unsigned(3765, 12),
    to_unsigned(3766, 12), to_unsigned(3766, 12), to_unsigned(3767, 12), to_unsigned(3767, 12), to_unsigned(3768, 12), to_unsigned(3769, 12), to_unsigned(3769, 12), to_unsigned(3770, 12),
    to_unsigned(3770, 12), to_unsigned(3771, 12), to_unsigned(3771, 12), to_unsigned(3772, 12), to_unsigned(3772, 12), to_unsigned(3773, 12), to_unsigned(3773, 12), to_unsigned(3774, 12),
    to_unsigned(3775, 12), to_unsigned(3775, 12), to_unsigned(3776, 12), to_unsigned(3776, 12), to_unsigned(3777, 12), to_unsigned(3777, 12), to_unsigned(3778, 12), to_unsigned(3778, 12),
    to_unsigned(3779, 12), to_unsigned(3779, 12), to_unsigned(3780, 12), to_unsigned(3780, 12), to_unsigned(3781, 12), to_unsigned(3782, 12), to_unsigned(3782, 12), to_unsigned(3783, 12),
    to_unsigned(3783, 12), to_unsigned(3784, 12), to_unsigned(3784, 12), to_unsigned(3785, 12), to_unsigned(3785, 12), to_unsigned(3786, 12), to_unsigned(3786, 12), to_unsigned(3787, 12),
    to_unsigned(3788, 12), to_unsigned(3788, 12), to_unsigned(3789, 12), to_unsigned(3789, 12), to_unsigned(3790, 12), to_unsigned(3790, 12), to_unsigned(3791, 12), to_unsigned(3791, 12),
    to_unsigned(3792, 12), to_unsigned(3792, 12), to_unsigned(3793, 12), to_unsigned(3793, 12), to_unsigned(3794, 12), to_unsigned(3795, 12), to_unsigned(3795, 12), to_unsigned(3796, 12),
    to_unsigned(3796, 12), to_unsigned(3797, 12), to_unsigned(3797, 12), to_unsigned(3798, 12), to_unsigned(3798, 12), to_unsigned(3799, 12), to_unsigned(3799, 12), to_unsigned(3800, 12),
    to_unsigned(3800, 12), to_unsigned(3801, 12), to_unsigned(3802, 12), to_unsigned(3802, 12), to_unsigned(3803, 12), to_unsigned(3803, 12), to_unsigned(3804, 12), to_unsigned(3804, 12),
    to_unsigned(3805, 12), to_unsigned(3805, 12), to_unsigned(3806, 12), to_unsigned(3806, 12), to_unsigned(3807, 12), to_unsigned(3807, 12), to_unsigned(3808, 12), to_unsigned(3809, 12),
    to_unsigned(3809, 12), to_unsigned(3810, 12), to_unsigned(3810, 12), to_unsigned(3811, 12), to_unsigned(3811, 12), to_unsigned(3812, 12), to_unsigned(3812, 12), to_unsigned(3813, 12),
    to_unsigned(3813, 12), to_unsigned(3814, 12), to_unsigned(3814, 12), to_unsigned(3815, 12), to_unsigned(3816, 12), to_unsigned(3816, 12), to_unsigned(3817, 12), to_unsigned(3817, 12),
    to_unsigned(3818, 12), to_unsigned(3818, 12), to_unsigned(3819, 12), to_unsigned(3819, 12), to_unsigned(3820, 12), to_unsigned(3820, 12), to_unsigned(3821, 12), to_unsigned(3821, 12),
    to_unsigned(3822, 12), to_unsigned(3822, 12), to_unsigned(3823, 12), to_unsigned(3824, 12), to_unsigned(3824, 12), to_unsigned(3825, 12), to_unsigned(3825, 12), to_unsigned(3826, 12),
    to_unsigned(3826, 12), to_unsigned(3827, 12), to_unsigned(3827, 12), to_unsigned(3828, 12), to_unsigned(3828, 12), to_unsigned(3829, 12), to_unsigned(3829, 12), to_unsigned(3830, 12),
    to_unsigned(3831, 12), to_unsigned(3831, 12), to_unsigned(3832, 12), to_unsigned(3832, 12), to_unsigned(3833, 12), to_unsigned(3833, 12), to_unsigned(3834, 12), to_unsigned(3834, 12),
    to_unsigned(3835, 12), to_unsigned(3835, 12), to_unsigned(3836, 12), to_unsigned(3836, 12), to_unsigned(3837, 12), to_unsigned(3837, 12), to_unsigned(3838, 12), to_unsigned(3839, 12),
    to_unsigned(3839, 12), to_unsigned(3840, 12), to_unsigned(3840, 12), to_unsigned(3841, 12), to_unsigned(3841, 12), to_unsigned(3842, 12), to_unsigned(3842, 12), to_unsigned(3843, 12),
    to_unsigned(3843, 12), to_unsigned(3844, 12), to_unsigned(3844, 12), to_unsigned(3845, 12), to_unsigned(3845, 12), to_unsigned(3846, 12), to_unsigned(3847, 12), to_unsigned(3847, 12),
    to_unsigned(3848, 12), to_unsigned(3848, 12), to_unsigned(3849, 12), to_unsigned(3849, 12), to_unsigned(3850, 12), to_unsigned(3850, 12), to_unsigned(3851, 12), to_unsigned(3851, 12),
    to_unsigned(3852, 12), to_unsigned(3852, 12), to_unsigned(3853, 12), to_unsigned(3853, 12), to_unsigned(3854, 12), to_unsigned(3854, 12), to_unsigned(3855, 12), to_unsigned(3856, 12),
    to_unsigned(3856, 12), to_unsigned(3857, 12), to_unsigned(3857, 12), to_unsigned(3858, 12), to_unsigned(3858, 12), to_unsigned(3859, 12), to_unsigned(3859, 12), to_unsigned(3860, 12),
    to_unsigned(3860, 12), to_unsigned(3861, 12), to_unsigned(3861, 12), to_unsigned(3862, 12), to_unsigned(3862, 12), to_unsigned(3863, 12), to_unsigned(3864, 12), to_unsigned(3864, 12),
    to_unsigned(3865, 12), to_unsigned(3865, 12), to_unsigned(3866, 12), to_unsigned(3866, 12), to_unsigned(3867, 12), to_unsigned(3867, 12), to_unsigned(3868, 12), to_unsigned(3868, 12),
    to_unsigned(3869, 12), to_unsigned(3869, 12), to_unsigned(3870, 12), to_unsigned(3870, 12), to_unsigned(3871, 12), to_unsigned(3871, 12), to_unsigned(3872, 12), to_unsigned(3873, 12),
    to_unsigned(3873, 12), to_unsigned(3874, 12), to_unsigned(3874, 12), to_unsigned(3875, 12), to_unsigned(3875, 12), to_unsigned(3876, 12), to_unsigned(3876, 12), to_unsigned(3877, 12),
    to_unsigned(3877, 12), to_unsigned(3878, 12), to_unsigned(3878, 12), to_unsigned(3879, 12), to_unsigned(3879, 12), to_unsigned(3880, 12), to_unsigned(3880, 12), to_unsigned(3881, 12),
    to_unsigned(3881, 12), to_unsigned(3882, 12), to_unsigned(3883, 12), to_unsigned(3883, 12), to_unsigned(3884, 12), to_unsigned(3884, 12), to_unsigned(3885, 12), to_unsigned(3885, 12),
    to_unsigned(3886, 12), to_unsigned(3886, 12), to_unsigned(3887, 12), to_unsigned(3887, 12), to_unsigned(3888, 12), to_unsigned(3888, 12), to_unsigned(3889, 12), to_unsigned(3889, 12),
    to_unsigned(3890, 12), to_unsigned(3890, 12), to_unsigned(3891, 12), to_unsigned(3891, 12), to_unsigned(3892, 12), to_unsigned(3893, 12), to_unsigned(3893, 12), to_unsigned(3894, 12),
    to_unsigned(3894, 12), to_unsigned(3895, 12), to_unsigned(3895, 12), to_unsigned(3896, 12), to_unsigned(3896, 12), to_unsigned(3897, 12), to_unsigned(3897, 12), to_unsigned(3898, 12),
    to_unsigned(3898, 12), to_unsigned(3899, 12), to_unsigned(3899, 12), to_unsigned(3900, 12), to_unsigned(3900, 12), to_unsigned(3901, 12), to_unsigned(3901, 12), to_unsigned(3902, 12),
    to_unsigned(3903, 12), to_unsigned(3903, 12), to_unsigned(3904, 12), to_unsigned(3904, 12), to_unsigned(3905, 12), to_unsigned(3905, 12), to_unsigned(3906, 12), to_unsigned(3906, 12),
    to_unsigned(3907, 12), to_unsigned(3907, 12), to_unsigned(3908, 12), to_unsigned(3908, 12), to_unsigned(3909, 12), to_unsigned(3909, 12), to_unsigned(3910, 12), to_unsigned(3910, 12),
    to_unsigned(3911, 12), to_unsigned(3911, 12), to_unsigned(3912, 12), to_unsigned(3912, 12), to_unsigned(3913, 12), to_unsigned(3914, 12), to_unsigned(3914, 12), to_unsigned(3915, 12),
    to_unsigned(3915, 12), to_unsigned(3916, 12), to_unsigned(3916, 12), to_unsigned(3917, 12), to_unsigned(3917, 12), to_unsigned(3918, 12), to_unsigned(3918, 12), to_unsigned(3919, 12),
    to_unsigned(3919, 12), to_unsigned(3920, 12), to_unsigned(3920, 12), to_unsigned(3921, 12), to_unsigned(3921, 12), to_unsigned(3922, 12), to_unsigned(3922, 12), to_unsigned(3923, 12),
    to_unsigned(3923, 12), to_unsigned(3924, 12), to_unsigned(3924, 12), to_unsigned(3925, 12), to_unsigned(3926, 12), to_unsigned(3926, 12), to_unsigned(3927, 12), to_unsigned(3927, 12),
    to_unsigned(3928, 12), to_unsigned(3928, 12), to_unsigned(3929, 12), to_unsigned(3929, 12), to_unsigned(3930, 12), to_unsigned(3930, 12), to_unsigned(3931, 12), to_unsigned(3931, 12),
    to_unsigned(3932, 12), to_unsigned(3932, 12), to_unsigned(3933, 12), to_unsigned(3933, 12), to_unsigned(3934, 12), to_unsigned(3934, 12), to_unsigned(3935, 12), to_unsigned(3935, 12),
    to_unsigned(3936, 12), to_unsigned(3936, 12), to_unsigned(3937, 12), to_unsigned(3938, 12), to_unsigned(3938, 12), to_unsigned(3939, 12), to_unsigned(3939, 12), to_unsigned(3940, 12),
    to_unsigned(3940, 12), to_unsigned(3941, 12), to_unsigned(3941, 12), to_unsigned(3942, 12), to_unsigned(3942, 12), to_unsigned(3943, 12), to_unsigned(3943, 12), to_unsigned(3944, 12),
    to_unsigned(3944, 12), to_unsigned(3945, 12), to_unsigned(3945, 12), to_unsigned(3946, 12), to_unsigned(3946, 12), to_unsigned(3947, 12), to_unsigned(3947, 12), to_unsigned(3948, 12),
    to_unsigned(3948, 12), to_unsigned(3949, 12), to_unsigned(3949, 12), to_unsigned(3950, 12), to_unsigned(3950, 12), to_unsigned(3951, 12), to_unsigned(3952, 12), to_unsigned(3952, 12),
    to_unsigned(3953, 12), to_unsigned(3953, 12), to_unsigned(3954, 12), to_unsigned(3954, 12), to_unsigned(3955, 12), to_unsigned(3955, 12), to_unsigned(3956, 12), to_unsigned(3956, 12),
    to_unsigned(3957, 12), to_unsigned(3957, 12), to_unsigned(3958, 12), to_unsigned(3958, 12), to_unsigned(3959, 12), to_unsigned(3959, 12), to_unsigned(3960, 12), to_unsigned(3960, 12),
    to_unsigned(3961, 12), to_unsigned(3961, 12), to_unsigned(3962, 12), to_unsigned(3962, 12), to_unsigned(3963, 12), to_unsigned(3963, 12), to_unsigned(3964, 12), to_unsigned(3964, 12),
    to_unsigned(3965, 12), to_unsigned(3965, 12), to_unsigned(3966, 12), to_unsigned(3967, 12), to_unsigned(3967, 12), to_unsigned(3968, 12), to_unsigned(3968, 12), to_unsigned(3969, 12),
    to_unsigned(3969, 12), to_unsigned(3970, 12), to_unsigned(3970, 12), to_unsigned(3971, 12), to_unsigned(3971, 12), to_unsigned(3972, 12), to_unsigned(3972, 12), to_unsigned(3973, 12),
    to_unsigned(3973, 12), to_unsigned(3974, 12), to_unsigned(3974, 12), to_unsigned(3975, 12), to_unsigned(3975, 12), to_unsigned(3976, 12), to_unsigned(3976, 12), to_unsigned(3977, 12),
    to_unsigned(3977, 12), to_unsigned(3978, 12), to_unsigned(3978, 12), to_unsigned(3979, 12), to_unsigned(3979, 12), to_unsigned(3980, 12), to_unsigned(3980, 12), to_unsigned(3981, 12),
    to_unsigned(3981, 12), to_unsigned(3982, 12), to_unsigned(3982, 12), to_unsigned(3983, 12), to_unsigned(3984, 12), to_unsigned(3984, 12), to_unsigned(3985, 12), to_unsigned(3985, 12),
    to_unsigned(3986, 12), to_unsigned(3986, 12), to_unsigned(3987, 12), to_unsigned(3987, 12), to_unsigned(3988, 12), to_unsigned(3988, 12), to_unsigned(3989, 12), to_unsigned(3989, 12),
    to_unsigned(3990, 12), to_unsigned(3990, 12), to_unsigned(3991, 12), to_unsigned(3991, 12), to_unsigned(3992, 12), to_unsigned(3992, 12), to_unsigned(3993, 12), to_unsigned(3993, 12),
    to_unsigned(3994, 12), to_unsigned(3994, 12), to_unsigned(3995, 12), to_unsigned(3995, 12), to_unsigned(3996, 12), to_unsigned(3996, 12), to_unsigned(3997, 12), to_unsigned(3997, 12),
    to_unsigned(3998, 12), to_unsigned(3998, 12), to_unsigned(3999, 12), to_unsigned(3999, 12), to_unsigned(4000, 12), to_unsigned(4000, 12), to_unsigned(4001, 12), to_unsigned(4001, 12),
    to_unsigned(4002, 12), to_unsigned(4002, 12), to_unsigned(4003, 12), to_unsigned(4003, 12), to_unsigned(4004, 12), to_unsigned(4005, 12), to_unsigned(4005, 12), to_unsigned(4006, 12),
    to_unsigned(4006, 12), to_unsigned(4007, 12), to_unsigned(4007, 12), to_unsigned(4008, 12), to_unsigned(4008, 12), to_unsigned(4009, 12), to_unsigned(4009, 12), to_unsigned(4010, 12),
    to_unsigned(4010, 12), to_unsigned(4011, 12), to_unsigned(4011, 12), to_unsigned(4012, 12), to_unsigned(4012, 12), to_unsigned(4013, 12), to_unsigned(4013, 12), to_unsigned(4014, 12),
    to_unsigned(4014, 12), to_unsigned(4015, 12), to_unsigned(4015, 12), to_unsigned(4016, 12), to_unsigned(4016, 12), to_unsigned(4017, 12), to_unsigned(4017, 12), to_unsigned(4018, 12),
    to_unsigned(4018, 12), to_unsigned(4019, 12), to_unsigned(4019, 12), to_unsigned(4020, 12), to_unsigned(4020, 12), to_unsigned(4021, 12), to_unsigned(4021, 12), to_unsigned(4022, 12),
    to_unsigned(4022, 12), to_unsigned(4023, 12), to_unsigned(4023, 12), to_unsigned(4024, 12), to_unsigned(4024, 12), to_unsigned(4025, 12), to_unsigned(4025, 12), to_unsigned(4026, 12),
    to_unsigned(4026, 12), to_unsigned(4027, 12), to_unsigned(4027, 12), to_unsigned(4028, 12), to_unsigned(4028, 12), to_unsigned(4029, 12), to_unsigned(4029, 12), to_unsigned(4030, 12),
    to_unsigned(4031, 12), to_unsigned(4031, 12), to_unsigned(4032, 12), to_unsigned(4032, 12), to_unsigned(4033, 12), to_unsigned(4033, 12), to_unsigned(4034, 12), to_unsigned(4034, 12),
    to_unsigned(4035, 12), to_unsigned(4035, 12), to_unsigned(4036, 12), to_unsigned(4036, 12), to_unsigned(4037, 12), to_unsigned(4037, 12), to_unsigned(4038, 12), to_unsigned(4038, 12),
    to_unsigned(4039, 12), to_unsigned(4039, 12), to_unsigned(4040, 12), to_unsigned(4040, 12), to_unsigned(4041, 12), to_unsigned(4041, 12), to_unsigned(4042, 12), to_unsigned(4042, 12),
    to_unsigned(4043, 12), to_unsigned(4043, 12), to_unsigned(4044, 12), to_unsigned(4044, 12), to_unsigned(4045, 12), to_unsigned(4045, 12), to_unsigned(4046, 12), to_unsigned(4046, 12),
    to_unsigned(4047, 12), to_unsigned(4047, 12), to_unsigned(4048, 12), to_unsigned(4048, 12), to_unsigned(4049, 12), to_unsigned(4049, 12), to_unsigned(4050, 12), to_unsigned(4050, 12),
    to_unsigned(4051, 12), to_unsigned(4051, 12), to_unsigned(4052, 12), to_unsigned(4052, 12), to_unsigned(4053, 12), to_unsigned(4053, 12), to_unsigned(4054, 12), to_unsigned(4054, 12),
    to_unsigned(4055, 12), to_unsigned(4055, 12), to_unsigned(4056, 12), to_unsigned(4056, 12), to_unsigned(4057, 12), to_unsigned(4057, 12), to_unsigned(4058, 12), to_unsigned(4058, 12),
    to_unsigned(4059, 12), to_unsigned(4059, 12), to_unsigned(4060, 12), to_unsigned(4060, 12), to_unsigned(4061, 12), to_unsigned(4061, 12), to_unsigned(4062, 12), to_unsigned(4062, 12),
    to_unsigned(4063, 12), to_unsigned(4063, 12), to_unsigned(4064, 12), to_unsigned(4064, 12), to_unsigned(4065, 12), to_unsigned(4065, 12), to_unsigned(4066, 12), to_unsigned(4066, 12),
    to_unsigned(4067, 12), to_unsigned(4067, 12), to_unsigned(4068, 12), to_unsigned(4068, 12), to_unsigned(4069, 12), to_unsigned(4069, 12), to_unsigned(4070, 12), to_unsigned(4070, 12),
    to_unsigned(4071, 12), to_unsigned(4071, 12), to_unsigned(4072, 12), to_unsigned(4072, 12), to_unsigned(4073, 12), to_unsigned(4073, 12), to_unsigned(4074, 12), to_unsigned(4074, 12),
    to_unsigned(4075, 12), to_unsigned(4075, 12), to_unsigned(4076, 12), to_unsigned(4076, 12), to_unsigned(4077, 12), to_unsigned(4077, 12), to_unsigned(4078, 12), to_unsigned(4078, 12),
    to_unsigned(4079, 12), to_unsigned(4079, 12), to_unsigned(4080, 12), to_unsigned(4080, 12), to_unsigned(4081, 12), to_unsigned(4081, 12), to_unsigned(4082, 12), to_unsigned(4082, 12),
    to_unsigned(4083, 12), to_unsigned(4083, 12), to_unsigned(4084, 12), to_unsigned(4084, 12), to_unsigned(4085, 12), to_unsigned(4085, 12), to_unsigned(4086, 12), to_unsigned(4086, 12),
    to_unsigned(4087, 12), to_unsigned(4087, 12), to_unsigned(4088, 12), to_unsigned(4088, 12), to_unsigned(4089, 12), to_unsigned(4089, 12), to_unsigned(4090, 12), to_unsigned(4090, 12),
    to_unsigned(4091, 12), to_unsigned(4091, 12), to_unsigned(4092, 12), to_unsigned(4092, 12), to_unsigned(4093, 12), to_unsigned(4094, 12), to_unsigned(4094, 12), to_unsigned(4095, 12)
    );
    
    signal sqrt_addr            : unsigned(11 downto 0) := (others => '0');
    signal sqrt_result          : unsigned(11 downto 0) := (others => '0');
    
    -- ----------------------------------------------------------------
    -- Stage 2: Peak detector signals
    -- ----------------------------------------------------------------
    signal abs_mono             : unsigned(23 downto 0) := (others => '0');
    signal peak_level           : unsigned(23 downto 0) := (others => '0');
    
    -- ----------------------------------------------------------------
    -- Stage 3: Gain computer signals
    -- ----------------------------------------------------------------
    type gain_lut_t is array (0 to 4095) of unsigned(15 downto 0);
    constant GAIN_LUT : gain_lut_t :=(
        to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65371, 16),
    to_unsigned(65371, 16), to_unsigned(65371, 16), to_unsigned(65344, 16), to_unsigned(65155, 16), to_unsigned(64967, 16), to_unsigned(64780, 16), to_unsigned(64595, 16), to_unsigned(64410, 16),
    to_unsigned(64227, 16), to_unsigned(64046, 16), to_unsigned(63865, 16), to_unsigned(63685, 16), to_unsigned(63507, 16), to_unsigned(63330, 16), to_unsigned(63154, 16), to_unsigned(62979, 16),
    to_unsigned(62805, 16), to_unsigned(62633, 16), to_unsigned(62461, 16), to_unsigned(62291, 16), to_unsigned(62121, 16), to_unsigned(61953, 16), to_unsigned(61786, 16), to_unsigned(61620, 16),
    to_unsigned(61455, 16), to_unsigned(61291, 16), to_unsigned(61127, 16), to_unsigned(60965, 16), to_unsigned(60804, 16), to_unsigned(60644, 16), to_unsigned(60485, 16), to_unsigned(60327, 16),
    to_unsigned(60170, 16), to_unsigned(60014, 16), to_unsigned(59858, 16), to_unsigned(59704, 16), to_unsigned(59551, 16), to_unsigned(59398, 16), to_unsigned(59247, 16), to_unsigned(59096, 16),
    to_unsigned(58946, 16), to_unsigned(58797, 16), to_unsigned(58649, 16), to_unsigned(58502, 16), to_unsigned(58356, 16), to_unsigned(58210, 16), to_unsigned(58065, 16), to_unsigned(57922, 16),
    to_unsigned(57779, 16), to_unsigned(57637, 16), to_unsigned(57495, 16), to_unsigned(57355, 16), to_unsigned(57215, 16), to_unsigned(57076, 16), to_unsigned(56938, 16), to_unsigned(56801, 16),
    to_unsigned(56664, 16), to_unsigned(56528, 16), to_unsigned(56393, 16), to_unsigned(56259, 16), to_unsigned(56125, 16), to_unsigned(55992, 16), to_unsigned(55860, 16), to_unsigned(55729, 16),
    to_unsigned(55598, 16), to_unsigned(55468, 16), to_unsigned(55339, 16), to_unsigned(55210, 16), to_unsigned(55083, 16), to_unsigned(54955, 16), to_unsigned(54829, 16), to_unsigned(54703, 16),
    to_unsigned(54578, 16), to_unsigned(54454, 16), to_unsigned(54330, 16), to_unsigned(54207, 16), to_unsigned(54084, 16), to_unsigned(53962, 16), to_unsigned(53841, 16), to_unsigned(53720, 16),
    to_unsigned(53600, 16), to_unsigned(53481, 16), to_unsigned(53362, 16), to_unsigned(53244, 16), to_unsigned(53127, 16), to_unsigned(53010, 16), to_unsigned(52894, 16), to_unsigned(52778, 16),
    to_unsigned(52663, 16), to_unsigned(52548, 16), to_unsigned(52434, 16), to_unsigned(52321, 16), to_unsigned(52208, 16), to_unsigned(52096, 16), to_unsigned(51984, 16), to_unsigned(51873, 16),
    to_unsigned(51763, 16), to_unsigned(51653, 16), to_unsigned(51543, 16), to_unsigned(51434, 16), to_unsigned(51326, 16), to_unsigned(51218, 16), to_unsigned(51111, 16), to_unsigned(51004, 16),
    to_unsigned(50897, 16), to_unsigned(50792, 16), to_unsigned(50686, 16), to_unsigned(50582, 16), to_unsigned(50477, 16), to_unsigned(50374, 16), to_unsigned(50270, 16), to_unsigned(50168, 16),
    to_unsigned(50065, 16), to_unsigned(49964, 16), to_unsigned(49862, 16), to_unsigned(49761, 16), to_unsigned(49661, 16), to_unsigned(49561, 16), to_unsigned(49462, 16), to_unsigned(49363, 16),
    to_unsigned(49264, 16), to_unsigned(49166, 16), to_unsigned(49069, 16), to_unsigned(48972, 16), to_unsigned(48875, 16), to_unsigned(48779, 16), to_unsigned(48683, 16), to_unsigned(48587, 16),
    to_unsigned(48493, 16), to_unsigned(48398, 16), to_unsigned(48304, 16), to_unsigned(48210, 16), to_unsigned(48117, 16), to_unsigned(48024, 16), to_unsigned(47932, 16), to_unsigned(47840, 16),
    to_unsigned(47748, 16), to_unsigned(47657, 16), to_unsigned(47566, 16), to_unsigned(47476, 16), to_unsigned(47386, 16), to_unsigned(47297, 16), to_unsigned(47207, 16), to_unsigned(47119, 16),
    to_unsigned(47030, 16), to_unsigned(46942, 16), to_unsigned(46855, 16), to_unsigned(46768, 16), to_unsigned(46681, 16), to_unsigned(46594, 16), to_unsigned(46508, 16), to_unsigned(46422, 16),
    to_unsigned(46337, 16), to_unsigned(46252, 16), to_unsigned(46167, 16), to_unsigned(46083, 16), to_unsigned(45999, 16), to_unsigned(45916, 16), to_unsigned(45832, 16), to_unsigned(45750, 16),
    to_unsigned(45667, 16), to_unsigned(45585, 16), to_unsigned(45503, 16), to_unsigned(45422, 16), to_unsigned(45340, 16), to_unsigned(45260, 16), to_unsigned(45179, 16), to_unsigned(45099, 16),
    to_unsigned(45019, 16), to_unsigned(44940, 16), to_unsigned(44861, 16), to_unsigned(44782, 16), to_unsigned(44703, 16), to_unsigned(44625, 16), to_unsigned(44547, 16), to_unsigned(44470, 16),
    to_unsigned(44393, 16), to_unsigned(44316, 16), to_unsigned(44239, 16), to_unsigned(44163, 16), to_unsigned(44087, 16), to_unsigned(44011, 16), to_unsigned(43936, 16), to_unsigned(43861, 16),
    to_unsigned(43786, 16), to_unsigned(43711, 16), to_unsigned(43637, 16), to_unsigned(43563, 16), to_unsigned(43490, 16), to_unsigned(43416, 16), to_unsigned(43343, 16), to_unsigned(43271, 16),
    to_unsigned(43198, 16), to_unsigned(43126, 16), to_unsigned(43054, 16), to_unsigned(42982, 16), to_unsigned(42911, 16), to_unsigned(42840, 16), to_unsigned(42769, 16), to_unsigned(42699, 16),
    to_unsigned(42628, 16), to_unsigned(42558, 16), to_unsigned(42489, 16), to_unsigned(42419, 16), to_unsigned(42350, 16), to_unsigned(42281, 16), to_unsigned(42213, 16), to_unsigned(42144, 16),
    to_unsigned(42076, 16), to_unsigned(42008, 16), to_unsigned(41941, 16), to_unsigned(41873, 16), to_unsigned(41806, 16), to_unsigned(41739, 16), to_unsigned(41673, 16), to_unsigned(41606, 16),
    to_unsigned(41540, 16), to_unsigned(41474, 16), to_unsigned(41408, 16), to_unsigned(41343, 16), to_unsigned(41278, 16), to_unsigned(41213, 16), to_unsigned(41148, 16), to_unsigned(41084, 16),
    to_unsigned(41020, 16), to_unsigned(40956, 16), to_unsigned(40892, 16), to_unsigned(40828, 16), to_unsigned(40765, 16), to_unsigned(40702, 16), to_unsigned(40639, 16), to_unsigned(40577, 16),
    to_unsigned(40514, 16), to_unsigned(40452, 16), to_unsigned(40390, 16), to_unsigned(40329, 16), to_unsigned(40267, 16), to_unsigned(40206, 16), to_unsigned(40145, 16), to_unsigned(40084, 16),
    to_unsigned(40023, 16), to_unsigned(39963, 16), to_unsigned(39903, 16), to_unsigned(39843, 16), to_unsigned(39783, 16), to_unsigned(39723, 16), to_unsigned(39664, 16), to_unsigned(39605, 16),
    to_unsigned(39546, 16), to_unsigned(39487, 16), to_unsigned(39429, 16), to_unsigned(39370, 16), to_unsigned(39312, 16), to_unsigned(39254, 16), to_unsigned(39196, 16), to_unsigned(39139, 16),
    to_unsigned(39081, 16), to_unsigned(39024, 16), to_unsigned(38967, 16), to_unsigned(38911, 16), to_unsigned(38854, 16), to_unsigned(38798, 16), to_unsigned(38741, 16), to_unsigned(38685, 16),
    to_unsigned(38630, 16), to_unsigned(38574, 16), to_unsigned(38519, 16), to_unsigned(38463, 16), to_unsigned(38408, 16), to_unsigned(38353, 16), to_unsigned(38299, 16), to_unsigned(38244, 16),
    to_unsigned(38190, 16), to_unsigned(38136, 16), to_unsigned(38082, 16), to_unsigned(38028, 16), to_unsigned(37974, 16), to_unsigned(37921, 16), to_unsigned(37868, 16), to_unsigned(37814, 16),
    to_unsigned(37762, 16), to_unsigned(37709, 16), to_unsigned(37656, 16), to_unsigned(37604, 16), to_unsigned(37552, 16), to_unsigned(37499, 16), to_unsigned(37448, 16), to_unsigned(37396, 16),
    to_unsigned(37344, 16), to_unsigned(37293, 16), to_unsigned(37242, 16), to_unsigned(37191, 16), to_unsigned(37140, 16), to_unsigned(37089, 16), to_unsigned(37038, 16), to_unsigned(36988, 16),
    to_unsigned(36938, 16), to_unsigned(36888, 16), to_unsigned(36838, 16), to_unsigned(36788, 16), to_unsigned(36738, 16), to_unsigned(36689, 16), to_unsigned(36639, 16), to_unsigned(36590, 16),
    to_unsigned(36541, 16), to_unsigned(36492, 16), to_unsigned(36444, 16), to_unsigned(36395, 16), to_unsigned(36347, 16), to_unsigned(36298, 16), to_unsigned(36250, 16), to_unsigned(36202, 16),
    to_unsigned(36154, 16), to_unsigned(36107, 16), to_unsigned(36059, 16), to_unsigned(36012, 16), to_unsigned(35965, 16), to_unsigned(35918, 16), to_unsigned(35871, 16), to_unsigned(35824, 16),
    to_unsigned(35777, 16), to_unsigned(35731, 16), to_unsigned(35684, 16), to_unsigned(35638, 16), to_unsigned(35592, 16), to_unsigned(35546, 16), to_unsigned(35500, 16), to_unsigned(35455, 16),
    to_unsigned(35409, 16), to_unsigned(35364, 16), to_unsigned(35318, 16), to_unsigned(35273, 16), to_unsigned(35228, 16), to_unsigned(35183, 16), to_unsigned(35139, 16), to_unsigned(35094, 16),
    to_unsigned(35050, 16), to_unsigned(35005, 16), to_unsigned(34961, 16), to_unsigned(34917, 16), to_unsigned(34873, 16), to_unsigned(34829, 16), to_unsigned(34785, 16), to_unsigned(34742, 16),
    to_unsigned(34698, 16), to_unsigned(34655, 16), to_unsigned(34612, 16), to_unsigned(34569, 16), to_unsigned(34526, 16), to_unsigned(34483, 16), to_unsigned(34440, 16), to_unsigned(34398, 16),
    to_unsigned(34355, 16), to_unsigned(34313, 16), to_unsigned(34271, 16), to_unsigned(34229, 16), to_unsigned(34187, 16), to_unsigned(34145, 16), to_unsigned(34103, 16), to_unsigned(34062, 16),
    to_unsigned(34020, 16), to_unsigned(33979, 16), to_unsigned(33938, 16), to_unsigned(33897, 16), to_unsigned(33856, 16), to_unsigned(33815, 16), to_unsigned(33774, 16), to_unsigned(33733, 16),
    to_unsigned(33693, 16), to_unsigned(33652, 16), to_unsigned(33612, 16), to_unsigned(33572, 16), to_unsigned(33532, 16), to_unsigned(33492, 16), to_unsigned(33452, 16), to_unsigned(33412, 16),
    to_unsigned(33372, 16), to_unsigned(33333, 16), to_unsigned(33293, 16), to_unsigned(33254, 16), to_unsigned(33215, 16), to_unsigned(33176, 16), to_unsigned(33137, 16), to_unsigned(33098, 16),
    to_unsigned(33059, 16), to_unsigned(33020, 16), to_unsigned(32982, 16), to_unsigned(32943, 16), to_unsigned(32905, 16), to_unsigned(32867, 16), to_unsigned(32828, 16), to_unsigned(32790, 16),
    to_unsigned(32752, 16), to_unsigned(32714, 16), to_unsigned(32677, 16), to_unsigned(32639, 16), to_unsigned(32601, 16), to_unsigned(32564, 16), to_unsigned(32527, 16), to_unsigned(32489, 16),
    to_unsigned(32452, 16), to_unsigned(32415, 16), to_unsigned(32378, 16), to_unsigned(32341, 16), to_unsigned(32305, 16), to_unsigned(32268, 16), to_unsigned(32231, 16), to_unsigned(32195, 16),
    to_unsigned(32159, 16), to_unsigned(32122, 16), to_unsigned(32086, 16), to_unsigned(32050, 16), to_unsigned(32014, 16), to_unsigned(31978, 16), to_unsigned(31942, 16), to_unsigned(31907, 16),
    to_unsigned(31871, 16), to_unsigned(31836, 16), to_unsigned(31800, 16), to_unsigned(31765, 16), to_unsigned(31729, 16), to_unsigned(31694, 16), to_unsigned(31659, 16), to_unsigned(31624, 16),
    to_unsigned(31589, 16), to_unsigned(31555, 16), to_unsigned(31520, 16), to_unsigned(31485, 16), to_unsigned(31451, 16), to_unsigned(31416, 16), to_unsigned(31382, 16), to_unsigned(31348, 16),
    to_unsigned(31314, 16), to_unsigned(31279, 16), to_unsigned(31245, 16), to_unsigned(31212, 16), to_unsigned(31178, 16), to_unsigned(31144, 16), to_unsigned(31110, 16), to_unsigned(31077, 16),
    to_unsigned(31043, 16), to_unsigned(31010, 16), to_unsigned(30976, 16), to_unsigned(30943, 16), to_unsigned(30910, 16), to_unsigned(30877, 16), to_unsigned(30844, 16), to_unsigned(30811, 16),
    to_unsigned(30778, 16), to_unsigned(30745, 16), to_unsigned(30713, 16), to_unsigned(30680, 16), to_unsigned(30648, 16), to_unsigned(30615, 16), to_unsigned(30583, 16), to_unsigned(30551, 16),
    to_unsigned(30519, 16), to_unsigned(30486, 16), to_unsigned(30454, 16), to_unsigned(30422, 16), to_unsigned(30391, 16), to_unsigned(30359, 16), to_unsigned(30327, 16), to_unsigned(30295, 16),
    to_unsigned(30264, 16), to_unsigned(30232, 16), to_unsigned(30201, 16), to_unsigned(30170, 16), to_unsigned(30138, 16), to_unsigned(30107, 16), to_unsigned(30076, 16), to_unsigned(30045, 16),
    to_unsigned(30014, 16), to_unsigned(29983, 16), to_unsigned(29952, 16), to_unsigned(29922, 16), to_unsigned(29891, 16), to_unsigned(29860, 16), to_unsigned(29830, 16), to_unsigned(29799, 16),
    to_unsigned(29769, 16), to_unsigned(29739, 16), to_unsigned(29709, 16), to_unsigned(29678, 16), to_unsigned(29648, 16), to_unsigned(29618, 16), to_unsigned(29588, 16), to_unsigned(29558, 16),
    to_unsigned(29529, 16), to_unsigned(29499, 16), to_unsigned(29469, 16), to_unsigned(29440, 16), to_unsigned(29410, 16), to_unsigned(29381, 16), to_unsigned(29351, 16), to_unsigned(29322, 16),
    to_unsigned(29293, 16), to_unsigned(29264, 16), to_unsigned(29234, 16), to_unsigned(29205, 16), to_unsigned(29176, 16), to_unsigned(29148, 16), to_unsigned(29119, 16), to_unsigned(29090, 16),
    to_unsigned(29061, 16), to_unsigned(29033, 16), to_unsigned(29004, 16), to_unsigned(28975, 16), to_unsigned(28947, 16), to_unsigned(28919, 16), to_unsigned(28890, 16), to_unsigned(28862, 16),
    to_unsigned(28834, 16), to_unsigned(28806, 16), to_unsigned(28778, 16), to_unsigned(28750, 16), to_unsigned(28722, 16), to_unsigned(28694, 16), to_unsigned(28666, 16), to_unsigned(28638, 16),
    to_unsigned(28611, 16), to_unsigned(28583, 16), to_unsigned(28555, 16), to_unsigned(28528, 16), to_unsigned(28500, 16), to_unsigned(28473, 16), to_unsigned(28446, 16), to_unsigned(28419, 16),
    to_unsigned(28391, 16), to_unsigned(28364, 16), to_unsigned(28337, 16), to_unsigned(28310, 16), to_unsigned(28283, 16), to_unsigned(28256, 16), to_unsigned(28229, 16), to_unsigned(28203, 16),
    to_unsigned(28176, 16), to_unsigned(28149, 16), to_unsigned(28123, 16), to_unsigned(28096, 16), to_unsigned(28070, 16), to_unsigned(28043, 16), to_unsigned(28017, 16), to_unsigned(27991, 16),
    to_unsigned(27964, 16), to_unsigned(27938, 16), to_unsigned(27912, 16), to_unsigned(27886, 16), to_unsigned(27860, 16), to_unsigned(27834, 16), to_unsigned(27808, 16), to_unsigned(27782, 16),
    to_unsigned(27757, 16), to_unsigned(27731, 16), to_unsigned(27705, 16), to_unsigned(27679, 16), to_unsigned(27654, 16), to_unsigned(27628, 16), to_unsigned(27603, 16), to_unsigned(27578, 16),
    to_unsigned(27552, 16), to_unsigned(27527, 16), to_unsigned(27502, 16), to_unsigned(27476, 16), to_unsigned(27451, 16), to_unsigned(27426, 16), to_unsigned(27401, 16), to_unsigned(27376, 16),
    to_unsigned(27351, 16), to_unsigned(27326, 16), to_unsigned(27302, 16), to_unsigned(27277, 16), to_unsigned(27252, 16), to_unsigned(27227, 16), to_unsigned(27203, 16), to_unsigned(27178, 16),
    to_unsigned(27154, 16), to_unsigned(27129, 16), to_unsigned(27105, 16), to_unsigned(27081, 16), to_unsigned(27056, 16), to_unsigned(27032, 16), to_unsigned(27008, 16), to_unsigned(26984, 16),
    to_unsigned(26960, 16), to_unsigned(26936, 16), to_unsigned(26912, 16), to_unsigned(26888, 16), to_unsigned(26864, 16), to_unsigned(26840, 16), to_unsigned(26816, 16), to_unsigned(26792, 16),
    to_unsigned(26769, 16), to_unsigned(26745, 16), to_unsigned(26721, 16), to_unsigned(26698, 16), to_unsigned(26674, 16), to_unsigned(26651, 16), to_unsigned(26627, 16), to_unsigned(26604, 16),
    to_unsigned(26581, 16), to_unsigned(26558, 16), to_unsigned(26534, 16), to_unsigned(26511, 16), to_unsigned(26488, 16), to_unsigned(26465, 16), to_unsigned(26442, 16), to_unsigned(26419, 16),
    to_unsigned(26396, 16), to_unsigned(26373, 16), to_unsigned(26350, 16), to_unsigned(26327, 16), to_unsigned(26305, 16), to_unsigned(26282, 16), to_unsigned(26259, 16), to_unsigned(26237, 16),
    to_unsigned(26214, 16), to_unsigned(26192, 16), to_unsigned(26169, 16), to_unsigned(26147, 16), to_unsigned(26124, 16), to_unsigned(26102, 16), to_unsigned(26080, 16), to_unsigned(26057, 16),
    to_unsigned(26035, 16), to_unsigned(26013, 16), to_unsigned(25991, 16), to_unsigned(25969, 16), to_unsigned(25947, 16), to_unsigned(25925, 16), to_unsigned(25903, 16), to_unsigned(25881, 16),
    to_unsigned(25859, 16), to_unsigned(25837, 16), to_unsigned(25816, 16), to_unsigned(25794, 16), to_unsigned(25772, 16), to_unsigned(25750, 16), to_unsigned(25729, 16), to_unsigned(25707, 16),
    to_unsigned(25686, 16), to_unsigned(25664, 16), to_unsigned(25643, 16), to_unsigned(25621, 16), to_unsigned(25600, 16), to_unsigned(25579, 16), to_unsigned(25557, 16), to_unsigned(25536, 16),
    to_unsigned(25515, 16), to_unsigned(25494, 16), to_unsigned(25473, 16), to_unsigned(25452, 16), to_unsigned(25431, 16), to_unsigned(25410, 16), to_unsigned(25389, 16), to_unsigned(25368, 16),
    to_unsigned(25347, 16), to_unsigned(25326, 16), to_unsigned(25305, 16), to_unsigned(25285, 16), to_unsigned(25264, 16), to_unsigned(25243, 16), to_unsigned(25223, 16), to_unsigned(25202, 16),
    to_unsigned(25182, 16), to_unsigned(25161, 16), to_unsigned(25141, 16), to_unsigned(25120, 16), to_unsigned(25100, 16), to_unsigned(25079, 16), to_unsigned(25059, 16), to_unsigned(25039, 16),
    to_unsigned(25019, 16), to_unsigned(24998, 16), to_unsigned(24978, 16), to_unsigned(24958, 16), to_unsigned(24938, 16), to_unsigned(24918, 16), to_unsigned(24898, 16), to_unsigned(24878, 16),
    to_unsigned(24858, 16), to_unsigned(24838, 16), to_unsigned(24818, 16), to_unsigned(24798, 16), to_unsigned(24779, 16), to_unsigned(24759, 16), to_unsigned(24739, 16), to_unsigned(24719, 16),
    to_unsigned(24700, 16), to_unsigned(24680, 16), to_unsigned(24661, 16), to_unsigned(24641, 16), to_unsigned(24622, 16), to_unsigned(24602, 16), to_unsigned(24583, 16), to_unsigned(24563, 16),
    to_unsigned(24544, 16), to_unsigned(24525, 16), to_unsigned(24505, 16), to_unsigned(24486, 16), to_unsigned(24467, 16), to_unsigned(24448, 16), to_unsigned(24429, 16), to_unsigned(24410, 16),
    to_unsigned(24390, 16), to_unsigned(24371, 16), to_unsigned(24352, 16), to_unsigned(24333, 16), to_unsigned(24315, 16), to_unsigned(24296, 16), to_unsigned(24277, 16), to_unsigned(24258, 16),
    to_unsigned(24239, 16), to_unsigned(24220, 16), to_unsigned(24202, 16), to_unsigned(24183, 16), to_unsigned(24164, 16), to_unsigned(24146, 16), to_unsigned(24127, 16), to_unsigned(24108, 16),
    to_unsigned(24090, 16), to_unsigned(24071, 16), to_unsigned(24053, 16), to_unsigned(24035, 16), to_unsigned(24016, 16), to_unsigned(23998, 16), to_unsigned(23979, 16), to_unsigned(23961, 16),
    to_unsigned(23943, 16), to_unsigned(23925, 16), to_unsigned(23906, 16), to_unsigned(23888, 16), to_unsigned(23870, 16), to_unsigned(23852, 16), to_unsigned(23834, 16), to_unsigned(23816, 16),
    to_unsigned(23798, 16), to_unsigned(23780, 16), to_unsigned(23762, 16), to_unsigned(23744, 16), to_unsigned(23726, 16), to_unsigned(23708, 16), to_unsigned(23691, 16), to_unsigned(23673, 16),
    to_unsigned(23655, 16), to_unsigned(23637, 16), to_unsigned(23620, 16), to_unsigned(23602, 16), to_unsigned(23584, 16), to_unsigned(23567, 16), to_unsigned(23549, 16), to_unsigned(23532, 16),
    to_unsigned(23514, 16), to_unsigned(23497, 16), to_unsigned(23479, 16), to_unsigned(23462, 16), to_unsigned(23444, 16), to_unsigned(23427, 16), to_unsigned(23410, 16), to_unsigned(23392, 16),
    to_unsigned(23375, 16), to_unsigned(23358, 16), to_unsigned(23341, 16), to_unsigned(23323, 16), to_unsigned(23306, 16), to_unsigned(23289, 16), to_unsigned(23272, 16), to_unsigned(23255, 16),
    to_unsigned(23238, 16), to_unsigned(23221, 16), to_unsigned(23204, 16), to_unsigned(23187, 16), to_unsigned(23170, 16), to_unsigned(23153, 16), to_unsigned(23136, 16), to_unsigned(23120, 16),
    to_unsigned(23103, 16), to_unsigned(23086, 16), to_unsigned(23069, 16), to_unsigned(23052, 16), to_unsigned(23036, 16), to_unsigned(23019, 16), to_unsigned(23003, 16), to_unsigned(22986, 16),
    to_unsigned(22969, 16), to_unsigned(22953, 16), to_unsigned(22936, 16), to_unsigned(22920, 16), to_unsigned(22903, 16), to_unsigned(22887, 16), to_unsigned(22870, 16), to_unsigned(22854, 16),
    to_unsigned(22838, 16), to_unsigned(22821, 16), to_unsigned(22805, 16), to_unsigned(22789, 16), to_unsigned(22773, 16), to_unsigned(22756, 16), to_unsigned(22740, 16), to_unsigned(22724, 16),
    to_unsigned(22708, 16), to_unsigned(22692, 16), to_unsigned(22676, 16), to_unsigned(22660, 16), to_unsigned(22644, 16), to_unsigned(22628, 16), to_unsigned(22612, 16), to_unsigned(22596, 16),
    to_unsigned(22580, 16), to_unsigned(22564, 16), to_unsigned(22548, 16), to_unsigned(22532, 16), to_unsigned(22516, 16), to_unsigned(22500, 16), to_unsigned(22485, 16), to_unsigned(22469, 16),
    to_unsigned(22453, 16), to_unsigned(22437, 16), to_unsigned(22422, 16), to_unsigned(22406, 16), to_unsigned(22391, 16), to_unsigned(22375, 16), to_unsigned(22359, 16), to_unsigned(22344, 16),
    to_unsigned(22328, 16), to_unsigned(22313, 16), to_unsigned(22297, 16), to_unsigned(22282, 16), to_unsigned(22266, 16), to_unsigned(22251, 16), to_unsigned(22236, 16), to_unsigned(22220, 16),
    to_unsigned(22205, 16), to_unsigned(22190, 16), to_unsigned(22174, 16), to_unsigned(22159, 16), to_unsigned(22144, 16), to_unsigned(22129, 16), to_unsigned(22114, 16), to_unsigned(22098, 16),
    to_unsigned(22083, 16), to_unsigned(22068, 16), to_unsigned(22053, 16), to_unsigned(22038, 16), to_unsigned(22023, 16), to_unsigned(22008, 16), to_unsigned(21993, 16), to_unsigned(21978, 16),
    to_unsigned(21963, 16), to_unsigned(21948, 16), to_unsigned(21933, 16), to_unsigned(21919, 16), to_unsigned(21904, 16), to_unsigned(21889, 16), to_unsigned(21874, 16), to_unsigned(21859, 16),
    to_unsigned(21845, 16), to_unsigned(21830, 16), to_unsigned(21815, 16), to_unsigned(21801, 16), to_unsigned(21786, 16), to_unsigned(21771, 16), to_unsigned(21757, 16), to_unsigned(21742, 16),
    to_unsigned(21727, 16), to_unsigned(21713, 16), to_unsigned(21698, 16), to_unsigned(21684, 16), to_unsigned(21669, 16), to_unsigned(21655, 16), to_unsigned(21641, 16), to_unsigned(21626, 16),
    to_unsigned(21612, 16), to_unsigned(21597, 16), to_unsigned(21583, 16), to_unsigned(21569, 16), to_unsigned(21555, 16), to_unsigned(21540, 16), to_unsigned(21526, 16), to_unsigned(21512, 16),
    to_unsigned(21498, 16), to_unsigned(21483, 16), to_unsigned(21469, 16), to_unsigned(21455, 16), to_unsigned(21441, 16), to_unsigned(21427, 16), to_unsigned(21413, 16), to_unsigned(21399, 16),
    to_unsigned(21385, 16), to_unsigned(21371, 16), to_unsigned(21357, 16), to_unsigned(21343, 16), to_unsigned(21329, 16), to_unsigned(21315, 16), to_unsigned(21301, 16), to_unsigned(21287, 16),
    to_unsigned(21273, 16), to_unsigned(21259, 16), to_unsigned(21246, 16), to_unsigned(21232, 16), to_unsigned(21218, 16), to_unsigned(21204, 16), to_unsigned(21191, 16), to_unsigned(21177, 16),
    to_unsigned(21163, 16), to_unsigned(21149, 16), to_unsigned(21136, 16), to_unsigned(21122, 16), to_unsigned(21109, 16), to_unsigned(21095, 16), to_unsigned(21081, 16), to_unsigned(21068, 16),
    to_unsigned(21054, 16), to_unsigned(21041, 16), to_unsigned(21027, 16), to_unsigned(21014, 16), to_unsigned(21000, 16), to_unsigned(20987, 16), to_unsigned(20974, 16), to_unsigned(20960, 16),
    to_unsigned(20947, 16), to_unsigned(20933, 16), to_unsigned(20920, 16), to_unsigned(20907, 16), to_unsigned(20894, 16), to_unsigned(20880, 16), to_unsigned(20867, 16), to_unsigned(20854, 16),
    to_unsigned(20841, 16), to_unsigned(20827, 16), to_unsigned(20814, 16), to_unsigned(20801, 16), to_unsigned(20788, 16), to_unsigned(20775, 16), to_unsigned(20762, 16), to_unsigned(20749, 16),
    to_unsigned(20736, 16), to_unsigned(20723, 16), to_unsigned(20710, 16), to_unsigned(20697, 16), to_unsigned(20684, 16), to_unsigned(20671, 16), to_unsigned(20658, 16), to_unsigned(20645, 16),
    to_unsigned(20632, 16), to_unsigned(20619, 16), to_unsigned(20606, 16), to_unsigned(20593, 16), to_unsigned(20580, 16), to_unsigned(20568, 16), to_unsigned(20555, 16), to_unsigned(20542, 16),
    to_unsigned(20529, 16), to_unsigned(20517, 16), to_unsigned(20504, 16), to_unsigned(20491, 16), to_unsigned(20478, 16), to_unsigned(20466, 16), to_unsigned(20453, 16), to_unsigned(20440, 16),
    to_unsigned(20428, 16), to_unsigned(20415, 16), to_unsigned(20403, 16), to_unsigned(20390, 16), to_unsigned(20378, 16), to_unsigned(20365, 16), to_unsigned(20353, 16), to_unsigned(20340, 16),
    to_unsigned(20328, 16), to_unsigned(20315, 16), to_unsigned(20303, 16), to_unsigned(20290, 16), to_unsigned(20278, 16), to_unsigned(20266, 16), to_unsigned(20253, 16), to_unsigned(20241, 16),
    to_unsigned(20229, 16), to_unsigned(20216, 16), to_unsigned(20204, 16), to_unsigned(20192, 16), to_unsigned(20179, 16), to_unsigned(20167, 16), to_unsigned(20155, 16), to_unsigned(20143, 16),
    to_unsigned(20131, 16), to_unsigned(20118, 16), to_unsigned(20106, 16), to_unsigned(20094, 16), to_unsigned(20082, 16), to_unsigned(20070, 16), to_unsigned(20058, 16), to_unsigned(20046, 16),
    to_unsigned(20034, 16), to_unsigned(20022, 16), to_unsigned(20010, 16), to_unsigned(19998, 16), to_unsigned(19986, 16), to_unsigned(19974, 16), to_unsigned(19962, 16), to_unsigned(19950, 16),
    to_unsigned(19938, 16), to_unsigned(19926, 16), to_unsigned(19914, 16), to_unsigned(19902, 16), to_unsigned(19890, 16), to_unsigned(19879, 16), to_unsigned(19867, 16), to_unsigned(19855, 16),
    to_unsigned(19843, 16), to_unsigned(19831, 16), to_unsigned(19820, 16), to_unsigned(19808, 16), to_unsigned(19796, 16), to_unsigned(19785, 16), to_unsigned(19773, 16), to_unsigned(19761, 16),
    to_unsigned(19750, 16), to_unsigned(19738, 16), to_unsigned(19726, 16), to_unsigned(19715, 16), to_unsigned(19703, 16), to_unsigned(19692, 16), to_unsigned(19680, 16), to_unsigned(19668, 16),
    to_unsigned(19657, 16), to_unsigned(19645, 16), to_unsigned(19634, 16), to_unsigned(19622, 16), to_unsigned(19611, 16), to_unsigned(19600, 16), to_unsigned(19588, 16), to_unsigned(19577, 16),
    to_unsigned(19565, 16), to_unsigned(19554, 16), to_unsigned(19543, 16), to_unsigned(19531, 16), to_unsigned(19520, 16), to_unsigned(19509, 16), to_unsigned(19497, 16), to_unsigned(19486, 16),
    to_unsigned(19475, 16), to_unsigned(19463, 16), to_unsigned(19452, 16), to_unsigned(19441, 16), to_unsigned(19430, 16), to_unsigned(19418, 16), to_unsigned(19407, 16), to_unsigned(19396, 16),
    to_unsigned(19385, 16), to_unsigned(19374, 16), to_unsigned(19363, 16), to_unsigned(19352, 16), to_unsigned(19340, 16), to_unsigned(19329, 16), to_unsigned(19318, 16), to_unsigned(19307, 16),
    to_unsigned(19296, 16), to_unsigned(19285, 16), to_unsigned(19274, 16), to_unsigned(19263, 16), to_unsigned(19252, 16), to_unsigned(19241, 16), to_unsigned(19230, 16), to_unsigned(19219, 16),
    to_unsigned(19208, 16), to_unsigned(19198, 16), to_unsigned(19187, 16), to_unsigned(19176, 16), to_unsigned(19165, 16), to_unsigned(19154, 16), to_unsigned(19143, 16), to_unsigned(19132, 16),
    to_unsigned(19122, 16), to_unsigned(19111, 16), to_unsigned(19100, 16), to_unsigned(19089, 16), to_unsigned(19079, 16), to_unsigned(19068, 16), to_unsigned(19057, 16), to_unsigned(19046, 16),
    to_unsigned(19036, 16), to_unsigned(19025, 16), to_unsigned(19014, 16), to_unsigned(19004, 16), to_unsigned(18993, 16), to_unsigned(18982, 16), to_unsigned(18972, 16), to_unsigned(18961, 16),
    to_unsigned(18951, 16), to_unsigned(18940, 16), to_unsigned(18930, 16), to_unsigned(18919, 16), to_unsigned(18908, 16), to_unsigned(18898, 16), to_unsigned(18887, 16), to_unsigned(18877, 16),
    to_unsigned(18866, 16), to_unsigned(18856, 16), to_unsigned(18846, 16), to_unsigned(18835, 16), to_unsigned(18825, 16), to_unsigned(18814, 16), to_unsigned(18804, 16), to_unsigned(18794, 16),
    to_unsigned(18783, 16), to_unsigned(18773, 16), to_unsigned(18762, 16), to_unsigned(18752, 16), to_unsigned(18742, 16), to_unsigned(18732, 16), to_unsigned(18721, 16), to_unsigned(18711, 16),
    to_unsigned(18701, 16), to_unsigned(18690, 16), to_unsigned(18680, 16), to_unsigned(18670, 16), to_unsigned(18660, 16), to_unsigned(18650, 16), to_unsigned(18639, 16), to_unsigned(18629, 16),
    to_unsigned(18619, 16), to_unsigned(18609, 16), to_unsigned(18599, 16), to_unsigned(18589, 16), to_unsigned(18579, 16), to_unsigned(18569, 16), to_unsigned(18558, 16), to_unsigned(18548, 16),
    to_unsigned(18538, 16), to_unsigned(18528, 16), to_unsigned(18518, 16), to_unsigned(18508, 16), to_unsigned(18498, 16), to_unsigned(18488, 16), to_unsigned(18478, 16), to_unsigned(18468, 16),
    to_unsigned(18458, 16), to_unsigned(18448, 16), to_unsigned(18439, 16), to_unsigned(18429, 16), to_unsigned(18419, 16), to_unsigned(18409, 16), to_unsigned(18399, 16), to_unsigned(18389, 16),
    to_unsigned(18379, 16), to_unsigned(18369, 16), to_unsigned(18360, 16), to_unsigned(18350, 16), to_unsigned(18340, 16), to_unsigned(18330, 16), to_unsigned(18320, 16), to_unsigned(18311, 16),
    to_unsigned(18301, 16), to_unsigned(18291, 16), to_unsigned(18281, 16), to_unsigned(18272, 16), to_unsigned(18262, 16), to_unsigned(18252, 16), to_unsigned(18243, 16), to_unsigned(18233, 16),
    to_unsigned(18223, 16), to_unsigned(18214, 16), to_unsigned(18204, 16), to_unsigned(18194, 16), to_unsigned(18185, 16), to_unsigned(18175, 16), to_unsigned(18166, 16), to_unsigned(18156, 16),
    to_unsigned(18146, 16), to_unsigned(18137, 16), to_unsigned(18127, 16), to_unsigned(18118, 16), to_unsigned(18108, 16), to_unsigned(18099, 16), to_unsigned(18089, 16), to_unsigned(18080, 16),
    to_unsigned(18070, 16), to_unsigned(18061, 16), to_unsigned(18051, 16), to_unsigned(18042, 16), to_unsigned(18033, 16), to_unsigned(18023, 16), to_unsigned(18014, 16), to_unsigned(18004, 16),
    to_unsigned(17995, 16), to_unsigned(17986, 16), to_unsigned(17976, 16), to_unsigned(17967, 16), to_unsigned(17958, 16), to_unsigned(17948, 16), to_unsigned(17939, 16), to_unsigned(17930, 16),
    to_unsigned(17920, 16), to_unsigned(17911, 16), to_unsigned(17902, 16), to_unsigned(17893, 16), to_unsigned(17883, 16), to_unsigned(17874, 16), to_unsigned(17865, 16), to_unsigned(17856, 16),
    to_unsigned(17846, 16), to_unsigned(17837, 16), to_unsigned(17828, 16), to_unsigned(17819, 16), to_unsigned(17810, 16), to_unsigned(17801, 16), to_unsigned(17792, 16), to_unsigned(17782, 16),
    to_unsigned(17773, 16), to_unsigned(17764, 16), to_unsigned(17755, 16), to_unsigned(17746, 16), to_unsigned(17737, 16), to_unsigned(17728, 16), to_unsigned(17719, 16), to_unsigned(17710, 16),
    to_unsigned(17701, 16), to_unsigned(17692, 16), to_unsigned(17683, 16), to_unsigned(17674, 16), to_unsigned(17665, 16), to_unsigned(17656, 16), to_unsigned(17647, 16), to_unsigned(17638, 16),
    to_unsigned(17629, 16), to_unsigned(17620, 16), to_unsigned(17611, 16), to_unsigned(17602, 16), to_unsigned(17593, 16), to_unsigned(17584, 16), to_unsigned(17576, 16), to_unsigned(17567, 16),
    to_unsigned(17558, 16), to_unsigned(17549, 16), to_unsigned(17540, 16), to_unsigned(17531, 16), to_unsigned(17523, 16), to_unsigned(17514, 16), to_unsigned(17505, 16), to_unsigned(17496, 16),
    to_unsigned(17487, 16), to_unsigned(17479, 16), to_unsigned(17470, 16), to_unsigned(17461, 16), to_unsigned(17452, 16), to_unsigned(17444, 16), to_unsigned(17435, 16), to_unsigned(17426, 16),
    to_unsigned(17418, 16), to_unsigned(17409, 16), to_unsigned(17400, 16), to_unsigned(17392, 16), to_unsigned(17383, 16), to_unsigned(17374, 16), to_unsigned(17366, 16), to_unsigned(17357, 16),
    to_unsigned(17348, 16), to_unsigned(17340, 16), to_unsigned(17331, 16), to_unsigned(17323, 16), to_unsigned(17314, 16), to_unsigned(17305, 16), to_unsigned(17297, 16), to_unsigned(17288, 16),
    to_unsigned(17280, 16), to_unsigned(17271, 16), to_unsigned(17263, 16), to_unsigned(17254, 16), to_unsigned(17246, 16), to_unsigned(17237, 16), to_unsigned(17229, 16), to_unsigned(17220, 16),
    to_unsigned(17212, 16), to_unsigned(17204, 16), to_unsigned(17195, 16), to_unsigned(17187, 16), to_unsigned(17178, 16), to_unsigned(17170, 16), to_unsigned(17161, 16), to_unsigned(17153, 16),
    to_unsigned(17145, 16), to_unsigned(17136, 16), to_unsigned(17128, 16), to_unsigned(17120, 16), to_unsigned(17111, 16), to_unsigned(17103, 16), to_unsigned(17095, 16), to_unsigned(17086, 16),
    to_unsigned(17078, 16), to_unsigned(17070, 16), to_unsigned(17061, 16), to_unsigned(17053, 16), to_unsigned(17045, 16), to_unsigned(17037, 16), to_unsigned(17028, 16), to_unsigned(17020, 16),
    to_unsigned(17012, 16), to_unsigned(17004, 16), to_unsigned(16996, 16), to_unsigned(16987, 16), to_unsigned(16979, 16), to_unsigned(16971, 16), to_unsigned(16963, 16), to_unsigned(16955, 16),
    to_unsigned(16946, 16), to_unsigned(16938, 16), to_unsigned(16930, 16), to_unsigned(16922, 16), to_unsigned(16914, 16), to_unsigned(16906, 16), to_unsigned(16898, 16), to_unsigned(16890, 16),
    to_unsigned(16882, 16), to_unsigned(16874, 16), to_unsigned(16865, 16), to_unsigned(16857, 16), to_unsigned(16849, 16), to_unsigned(16841, 16), to_unsigned(16833, 16), to_unsigned(16825, 16),
    to_unsigned(16817, 16), to_unsigned(16809, 16), to_unsigned(16801, 16), to_unsigned(16793, 16), to_unsigned(16785, 16), to_unsigned(16777, 16), to_unsigned(16769, 16), to_unsigned(16761, 16),
    to_unsigned(16754, 16), to_unsigned(16746, 16), to_unsigned(16738, 16), to_unsigned(16730, 16), to_unsigned(16722, 16), to_unsigned(16714, 16), to_unsigned(16706, 16), to_unsigned(16698, 16),
    to_unsigned(16690, 16), to_unsigned(16683, 16), to_unsigned(16675, 16), to_unsigned(16667, 16), to_unsigned(16659, 16), to_unsigned(16651, 16), to_unsigned(16643, 16), to_unsigned(16636, 16),
    to_unsigned(16628, 16), to_unsigned(16620, 16), to_unsigned(16612, 16), to_unsigned(16604, 16), to_unsigned(16597, 16), to_unsigned(16589, 16), to_unsigned(16581, 16), to_unsigned(16573, 16),
    to_unsigned(16566, 16), to_unsigned(16558, 16), to_unsigned(16550, 16), to_unsigned(16543, 16), to_unsigned(16535, 16), to_unsigned(16527, 16), to_unsigned(16519, 16), to_unsigned(16512, 16),
    to_unsigned(16504, 16), to_unsigned(16496, 16), to_unsigned(16489, 16), to_unsigned(16481, 16), to_unsigned(16474, 16), to_unsigned(16466, 16), to_unsigned(16458, 16), to_unsigned(16451, 16),
    to_unsigned(16443, 16), to_unsigned(16436, 16), to_unsigned(16428, 16), to_unsigned(16420, 16), to_unsigned(16413, 16), to_unsigned(16405, 16), to_unsigned(16398, 16), to_unsigned(16390, 16),
    to_unsigned(16383, 16), to_unsigned(16375, 16), to_unsigned(16368, 16), to_unsigned(16360, 16), to_unsigned(16353, 16), to_unsigned(16345, 16), to_unsigned(16338, 16), to_unsigned(16330, 16),
    to_unsigned(16323, 16), to_unsigned(16315, 16), to_unsigned(16308, 16), to_unsigned(16300, 16), to_unsigned(16293, 16), to_unsigned(16285, 16), to_unsigned(16278, 16), to_unsigned(16271, 16),
    to_unsigned(16263, 16), to_unsigned(16256, 16), to_unsigned(16248, 16), to_unsigned(16241, 16), to_unsigned(16234, 16), to_unsigned(16226, 16), to_unsigned(16219, 16), to_unsigned(16212, 16),
    to_unsigned(16204, 16), to_unsigned(16197, 16), to_unsigned(16190, 16), to_unsigned(16182, 16), to_unsigned(16175, 16), to_unsigned(16168, 16), to_unsigned(16160, 16), to_unsigned(16153, 16),
    to_unsigned(16146, 16), to_unsigned(16138, 16), to_unsigned(16131, 16), to_unsigned(16124, 16), to_unsigned(16117, 16), to_unsigned(16109, 16), to_unsigned(16102, 16), to_unsigned(16095, 16),
    to_unsigned(16088, 16), to_unsigned(16081, 16), to_unsigned(16073, 16), to_unsigned(16066, 16), to_unsigned(16059, 16), to_unsigned(16052, 16), to_unsigned(16045, 16), to_unsigned(16037, 16),
    to_unsigned(16030, 16), to_unsigned(16023, 16), to_unsigned(16016, 16), to_unsigned(16009, 16), to_unsigned(16002, 16), to_unsigned(15995, 16), to_unsigned(15987, 16), to_unsigned(15980, 16),
    to_unsigned(15973, 16), to_unsigned(15966, 16), to_unsigned(15959, 16), to_unsigned(15952, 16), to_unsigned(15945, 16), to_unsigned(15938, 16), to_unsigned(15931, 16), to_unsigned(15924, 16),
    to_unsigned(15917, 16), to_unsigned(15910, 16), to_unsigned(15903, 16), to_unsigned(15896, 16), to_unsigned(15889, 16), to_unsigned(15882, 16), to_unsigned(15875, 16), to_unsigned(15868, 16),
    to_unsigned(15861, 16), to_unsigned(15854, 16), to_unsigned(15847, 16), to_unsigned(15840, 16), to_unsigned(15833, 16), to_unsigned(15826, 16), to_unsigned(15819, 16), to_unsigned(15812, 16),
    to_unsigned(15805, 16), to_unsigned(15798, 16), to_unsigned(15791, 16), to_unsigned(15784, 16), to_unsigned(15777, 16), to_unsigned(15771, 16), to_unsigned(15764, 16), to_unsigned(15757, 16),
    to_unsigned(15750, 16), to_unsigned(15743, 16), to_unsigned(15736, 16), to_unsigned(15729, 16), to_unsigned(15722, 16), to_unsigned(15716, 16), to_unsigned(15709, 16), to_unsigned(15702, 16),
    to_unsigned(15695, 16), to_unsigned(15688, 16), to_unsigned(15682, 16), to_unsigned(15675, 16), to_unsigned(15668, 16), to_unsigned(15661, 16), to_unsigned(15654, 16), to_unsigned(15648, 16),
    to_unsigned(15641, 16), to_unsigned(15634, 16), to_unsigned(15627, 16), to_unsigned(15621, 16), to_unsigned(15614, 16), to_unsigned(15607, 16), to_unsigned(15600, 16), to_unsigned(15594, 16),
    to_unsigned(15587, 16), to_unsigned(15580, 16), to_unsigned(15574, 16), to_unsigned(15567, 16), to_unsigned(15560, 16), to_unsigned(15554, 16), to_unsigned(15547, 16), to_unsigned(15540, 16),
    to_unsigned(15534, 16), to_unsigned(15527, 16), to_unsigned(15520, 16), to_unsigned(15514, 16), to_unsigned(15507, 16), to_unsigned(15500, 16), to_unsigned(15494, 16), to_unsigned(15487, 16),
    to_unsigned(15481, 16), to_unsigned(15474, 16), to_unsigned(15467, 16), to_unsigned(15461, 16), to_unsigned(15454, 16), to_unsigned(15448, 16), to_unsigned(15441, 16), to_unsigned(15435, 16),
    to_unsigned(15428, 16), to_unsigned(15422, 16), to_unsigned(15415, 16), to_unsigned(15408, 16), to_unsigned(15402, 16), to_unsigned(15395, 16), to_unsigned(15389, 16), to_unsigned(15382, 16),
    to_unsigned(15376, 16), to_unsigned(15369, 16), to_unsigned(15363, 16), to_unsigned(15356, 16), to_unsigned(15350, 16), to_unsigned(15344, 16), to_unsigned(15337, 16), to_unsigned(15331, 16),
    to_unsigned(15324, 16), to_unsigned(15318, 16), to_unsigned(15311, 16), to_unsigned(15305, 16), to_unsigned(15298, 16), to_unsigned(15292, 16), to_unsigned(15286, 16), to_unsigned(15279, 16),
    to_unsigned(15273, 16), to_unsigned(15266, 16), to_unsigned(15260, 16), to_unsigned(15254, 16), to_unsigned(15247, 16), to_unsigned(15241, 16), to_unsigned(15235, 16), to_unsigned(15228, 16),
    to_unsigned(15222, 16), to_unsigned(15216, 16), to_unsigned(15209, 16), to_unsigned(15203, 16), to_unsigned(15197, 16), to_unsigned(15190, 16), to_unsigned(15184, 16), to_unsigned(15178, 16),
    to_unsigned(15171, 16), to_unsigned(15165, 16), to_unsigned(15159, 16), to_unsigned(15153, 16), to_unsigned(15146, 16), to_unsigned(15140, 16), to_unsigned(15134, 16), to_unsigned(15127, 16),
    to_unsigned(15121, 16), to_unsigned(15115, 16), to_unsigned(15109, 16), to_unsigned(15102, 16), to_unsigned(15096, 16), to_unsigned(15090, 16), to_unsigned(15084, 16), to_unsigned(15078, 16),
    to_unsigned(15071, 16), to_unsigned(15065, 16), to_unsigned(15059, 16), to_unsigned(15053, 16), to_unsigned(15047, 16), to_unsigned(15041, 16), to_unsigned(15034, 16), to_unsigned(15028, 16),
    to_unsigned(15022, 16), to_unsigned(15016, 16), to_unsigned(15010, 16), to_unsigned(15004, 16), to_unsigned(14997, 16), to_unsigned(14991, 16), to_unsigned(14985, 16), to_unsigned(14979, 16),
    to_unsigned(14973, 16), to_unsigned(14967, 16), to_unsigned(14961, 16), to_unsigned(14955, 16), to_unsigned(14949, 16), to_unsigned(14943, 16), to_unsigned(14937, 16), to_unsigned(14930, 16),
    to_unsigned(14924, 16), to_unsigned(14918, 16), to_unsigned(14912, 16), to_unsigned(14906, 16), to_unsigned(14900, 16), to_unsigned(14894, 16), to_unsigned(14888, 16), to_unsigned(14882, 16),
    to_unsigned(14876, 16), to_unsigned(14870, 16), to_unsigned(14864, 16), to_unsigned(14858, 16), to_unsigned(14852, 16), to_unsigned(14846, 16), to_unsigned(14840, 16), to_unsigned(14834, 16),
    to_unsigned(14828, 16), to_unsigned(14822, 16), to_unsigned(14816, 16), to_unsigned(14810, 16), to_unsigned(14804, 16), to_unsigned(14798, 16), to_unsigned(14793, 16), to_unsigned(14787, 16),
    to_unsigned(14781, 16), to_unsigned(14775, 16), to_unsigned(14769, 16), to_unsigned(14763, 16), to_unsigned(14757, 16), to_unsigned(14751, 16), to_unsigned(14745, 16), to_unsigned(14739, 16),
    to_unsigned(14733, 16), to_unsigned(14728, 16), to_unsigned(14722, 16), to_unsigned(14716, 16), to_unsigned(14710, 16), to_unsigned(14704, 16), to_unsigned(14698, 16), to_unsigned(14692, 16),
    to_unsigned(14687, 16), to_unsigned(14681, 16), to_unsigned(14675, 16), to_unsigned(14669, 16), to_unsigned(14663, 16), to_unsigned(14658, 16), to_unsigned(14652, 16), to_unsigned(14646, 16),
    to_unsigned(14640, 16), to_unsigned(14634, 16), to_unsigned(14629, 16), to_unsigned(14623, 16), to_unsigned(14617, 16), to_unsigned(14611, 16), to_unsigned(14605, 16), to_unsigned(14600, 16),
    to_unsigned(14594, 16), to_unsigned(14588, 16), to_unsigned(14582, 16), to_unsigned(14577, 16), to_unsigned(14571, 16), to_unsigned(14565, 16), to_unsigned(14560, 16), to_unsigned(14554, 16),
    to_unsigned(14548, 16), to_unsigned(14542, 16), to_unsigned(14537, 16), to_unsigned(14531, 16), to_unsigned(14525, 16), to_unsigned(14520, 16), to_unsigned(14514, 16), to_unsigned(14508, 16),
    to_unsigned(14503, 16), to_unsigned(14497, 16), to_unsigned(14491, 16), to_unsigned(14486, 16), to_unsigned(14480, 16), to_unsigned(14474, 16), to_unsigned(14469, 16), to_unsigned(14463, 16),
    to_unsigned(14457, 16), to_unsigned(14452, 16), to_unsigned(14446, 16), to_unsigned(14441, 16), to_unsigned(14435, 16), to_unsigned(14429, 16), to_unsigned(14424, 16), to_unsigned(14418, 16),
    to_unsigned(14413, 16), to_unsigned(14407, 16), to_unsigned(14402, 16), to_unsigned(14396, 16), to_unsigned(14390, 16), to_unsigned(14385, 16), to_unsigned(14379, 16), to_unsigned(14374, 16),
    to_unsigned(14368, 16), to_unsigned(14363, 16), to_unsigned(14357, 16), to_unsigned(14352, 16), to_unsigned(14346, 16), to_unsigned(14341, 16), to_unsigned(14335, 16), to_unsigned(14329, 16),
    to_unsigned(14324, 16), to_unsigned(14318, 16), to_unsigned(14313, 16), to_unsigned(14307, 16), to_unsigned(14302, 16), to_unsigned(14297, 16), to_unsigned(14291, 16), to_unsigned(14286, 16),
    to_unsigned(14280, 16), to_unsigned(14275, 16), to_unsigned(14269, 16), to_unsigned(14264, 16), to_unsigned(14258, 16), to_unsigned(14253, 16), to_unsigned(14247, 16), to_unsigned(14242, 16),
    to_unsigned(14237, 16), to_unsigned(14231, 16), to_unsigned(14226, 16), to_unsigned(14220, 16), to_unsigned(14215, 16), to_unsigned(14209, 16), to_unsigned(14204, 16), to_unsigned(14199, 16),
    to_unsigned(14193, 16), to_unsigned(14188, 16), to_unsigned(14183, 16), to_unsigned(14177, 16), to_unsigned(14172, 16), to_unsigned(14166, 16), to_unsigned(14161, 16), to_unsigned(14156, 16),
    to_unsigned(14150, 16), to_unsigned(14145, 16), to_unsigned(14140, 16), to_unsigned(14134, 16), to_unsigned(14129, 16), to_unsigned(14124, 16), to_unsigned(14118, 16), to_unsigned(14113, 16),
    to_unsigned(14108, 16), to_unsigned(14102, 16), to_unsigned(14097, 16), to_unsigned(14092, 16), to_unsigned(14086, 16), to_unsigned(14081, 16), to_unsigned(14076, 16), to_unsigned(14071, 16),
    to_unsigned(14065, 16), to_unsigned(14060, 16), to_unsigned(14055, 16), to_unsigned(14050, 16), to_unsigned(14044, 16), to_unsigned(14039, 16), to_unsigned(14034, 16), to_unsigned(14029, 16),
    to_unsigned(14023, 16), to_unsigned(14018, 16), to_unsigned(14013, 16), to_unsigned(14008, 16), to_unsigned(14002, 16), to_unsigned(13997, 16), to_unsigned(13992, 16), to_unsigned(13987, 16),
    to_unsigned(13982, 16), to_unsigned(13976, 16), to_unsigned(13971, 16), to_unsigned(13966, 16), to_unsigned(13961, 16), to_unsigned(13956, 16), to_unsigned(13950, 16), to_unsigned(13945, 16),
    to_unsigned(13940, 16), to_unsigned(13935, 16), to_unsigned(13930, 16), to_unsigned(13925, 16), to_unsigned(13919, 16), to_unsigned(13914, 16), to_unsigned(13909, 16), to_unsigned(13904, 16),
    to_unsigned(13899, 16), to_unsigned(13894, 16), to_unsigned(13889, 16), to_unsigned(13884, 16), to_unsigned(13878, 16), to_unsigned(13873, 16), to_unsigned(13868, 16), to_unsigned(13863, 16),
    to_unsigned(13858, 16), to_unsigned(13853, 16), to_unsigned(13848, 16), to_unsigned(13843, 16), to_unsigned(13838, 16), to_unsigned(13833, 16), to_unsigned(13828, 16), to_unsigned(13822, 16),
    to_unsigned(13817, 16), to_unsigned(13812, 16), to_unsigned(13807, 16), to_unsigned(13802, 16), to_unsigned(13797, 16), to_unsigned(13792, 16), to_unsigned(13787, 16), to_unsigned(13782, 16),
    to_unsigned(13777, 16), to_unsigned(13772, 16), to_unsigned(13767, 16), to_unsigned(13762, 16), to_unsigned(13757, 16), to_unsigned(13752, 16), to_unsigned(13747, 16), to_unsigned(13742, 16),
    to_unsigned(13737, 16), to_unsigned(13732, 16), to_unsigned(13727, 16), to_unsigned(13722, 16), to_unsigned(13717, 16), to_unsigned(13712, 16), to_unsigned(13707, 16), to_unsigned(13702, 16),
    to_unsigned(13697, 16), to_unsigned(13692, 16), to_unsigned(13687, 16), to_unsigned(13682, 16), to_unsigned(13677, 16), to_unsigned(13672, 16), to_unsigned(13668, 16), to_unsigned(13663, 16),
    to_unsigned(13658, 16), to_unsigned(13653, 16), to_unsigned(13648, 16), to_unsigned(13643, 16), to_unsigned(13638, 16), to_unsigned(13633, 16), to_unsigned(13628, 16), to_unsigned(13623, 16),
    to_unsigned(13618, 16), to_unsigned(13613, 16), to_unsigned(13609, 16), to_unsigned(13604, 16), to_unsigned(13599, 16), to_unsigned(13594, 16), to_unsigned(13589, 16), to_unsigned(13584, 16),
    to_unsigned(13579, 16), to_unsigned(13575, 16), to_unsigned(13570, 16), to_unsigned(13565, 16), to_unsigned(13560, 16), to_unsigned(13555, 16), to_unsigned(13550, 16), to_unsigned(13545, 16),
    to_unsigned(13541, 16), to_unsigned(13536, 16), to_unsigned(13531, 16), to_unsigned(13526, 16), to_unsigned(13521, 16), to_unsigned(13517, 16), to_unsigned(13512, 16), to_unsigned(13507, 16),
    to_unsigned(13502, 16), to_unsigned(13497, 16), to_unsigned(13493, 16), to_unsigned(13488, 16), to_unsigned(13483, 16), to_unsigned(13478, 16), to_unsigned(13473, 16), to_unsigned(13469, 16),
    to_unsigned(13464, 16), to_unsigned(13459, 16), to_unsigned(13454, 16), to_unsigned(13450, 16), to_unsigned(13445, 16), to_unsigned(13440, 16), to_unsigned(13435, 16), to_unsigned(13431, 16),
    to_unsigned(13426, 16), to_unsigned(13421, 16), to_unsigned(13416, 16), to_unsigned(13412, 16), to_unsigned(13407, 16), to_unsigned(13402, 16), to_unsigned(13398, 16), to_unsigned(13393, 16),
    to_unsigned(13388, 16), to_unsigned(13384, 16), to_unsigned(13379, 16), to_unsigned(13374, 16), to_unsigned(13369, 16), to_unsigned(13365, 16), to_unsigned(13360, 16), to_unsigned(13355, 16),
    to_unsigned(13351, 16), to_unsigned(13346, 16), to_unsigned(13341, 16), to_unsigned(13337, 16), to_unsigned(13332, 16), to_unsigned(13327, 16), to_unsigned(13323, 16), to_unsigned(13318, 16),
    to_unsigned(13313, 16), to_unsigned(13309, 16), to_unsigned(13304, 16), to_unsigned(13300, 16), to_unsigned(13295, 16), to_unsigned(13290, 16), to_unsigned(13286, 16), to_unsigned(13281, 16),
    to_unsigned(13276, 16), to_unsigned(13272, 16), to_unsigned(13267, 16), to_unsigned(13263, 16), to_unsigned(13258, 16), to_unsigned(13253, 16), to_unsigned(13249, 16), to_unsigned(13244, 16),
    to_unsigned(13240, 16), to_unsigned(13235, 16), to_unsigned(13231, 16), to_unsigned(13226, 16), to_unsigned(13221, 16), to_unsigned(13217, 16), to_unsigned(13212, 16), to_unsigned(13208, 16),
    to_unsigned(13203, 16), to_unsigned(13199, 16), to_unsigned(13194, 16), to_unsigned(13190, 16), to_unsigned(13185, 16), to_unsigned(13180, 16), to_unsigned(13176, 16), to_unsigned(13171, 16),
    to_unsigned(13167, 16), to_unsigned(13162, 16), to_unsigned(13158, 16), to_unsigned(13153, 16), to_unsigned(13149, 16), to_unsigned(13144, 16), to_unsigned(13140, 16), to_unsigned(13135, 16),
    to_unsigned(13131, 16), to_unsigned(13126, 16), to_unsigned(13122, 16), to_unsigned(13117, 16), to_unsigned(13113, 16), to_unsigned(13108, 16), to_unsigned(13104, 16), to_unsigned(13099, 16),
    to_unsigned(13095, 16), to_unsigned(13091, 16), to_unsigned(13086, 16), to_unsigned(13082, 16), to_unsigned(13077, 16), to_unsigned(13073, 16), to_unsigned(13068, 16), to_unsigned(13064, 16),
    to_unsigned(13059, 16), to_unsigned(13055, 16), to_unsigned(13051, 16), to_unsigned(13046, 16), to_unsigned(13042, 16), to_unsigned(13037, 16), to_unsigned(13033, 16), to_unsigned(13028, 16),
    to_unsigned(13024, 16), to_unsigned(13020, 16), to_unsigned(13015, 16), to_unsigned(13011, 16), to_unsigned(13006, 16), to_unsigned(13002, 16), to_unsigned(12998, 16), to_unsigned(12993, 16),
    to_unsigned(12989, 16), to_unsigned(12985, 16), to_unsigned(12980, 16), to_unsigned(12976, 16), to_unsigned(12971, 16), to_unsigned(12967, 16), to_unsigned(12963, 16), to_unsigned(12958, 16),
    to_unsigned(12954, 16), to_unsigned(12950, 16), to_unsigned(12945, 16), to_unsigned(12941, 16), to_unsigned(12937, 16), to_unsigned(12932, 16), to_unsigned(12928, 16), to_unsigned(12924, 16),
    to_unsigned(12919, 16), to_unsigned(12915, 16), to_unsigned(12911, 16), to_unsigned(12906, 16), to_unsigned(12902, 16), to_unsigned(12898, 16), to_unsigned(12893, 16), to_unsigned(12889, 16),
    to_unsigned(12885, 16), to_unsigned(12880, 16), to_unsigned(12876, 16), to_unsigned(12872, 16), to_unsigned(12868, 16), to_unsigned(12863, 16), to_unsigned(12859, 16), to_unsigned(12855, 16),
    to_unsigned(12850, 16), to_unsigned(12846, 16), to_unsigned(12842, 16), to_unsigned(12838, 16), to_unsigned(12833, 16), to_unsigned(12829, 16), to_unsigned(12825, 16), to_unsigned(12821, 16),
    to_unsigned(12816, 16), to_unsigned(12812, 16), to_unsigned(12808, 16), to_unsigned(12804, 16), to_unsigned(12799, 16), to_unsigned(12795, 16), to_unsigned(12791, 16), to_unsigned(12787, 16),
    to_unsigned(12783, 16), to_unsigned(12778, 16), to_unsigned(12774, 16), to_unsigned(12770, 16), to_unsigned(12766, 16), to_unsigned(12761, 16), to_unsigned(12757, 16), to_unsigned(12753, 16),
    to_unsigned(12749, 16), to_unsigned(12745, 16), to_unsigned(12741, 16), to_unsigned(12736, 16), to_unsigned(12732, 16), to_unsigned(12728, 16), to_unsigned(12724, 16), to_unsigned(12720, 16),
    to_unsigned(12715, 16), to_unsigned(12711, 16), to_unsigned(12707, 16), to_unsigned(12703, 16), to_unsigned(12699, 16), to_unsigned(12695, 16), to_unsigned(12690, 16), to_unsigned(12686, 16),
    to_unsigned(12682, 16), to_unsigned(12678, 16), to_unsigned(12674, 16), to_unsigned(12670, 16), to_unsigned(12666, 16), to_unsigned(12662, 16), to_unsigned(12657, 16), to_unsigned(12653, 16),
    to_unsigned(12649, 16), to_unsigned(12645, 16), to_unsigned(12641, 16), to_unsigned(12637, 16), to_unsigned(12633, 16), to_unsigned(12629, 16), to_unsigned(12625, 16), to_unsigned(12620, 16),
    to_unsigned(12616, 16), to_unsigned(12612, 16), to_unsigned(12608, 16), to_unsigned(12604, 16), to_unsigned(12600, 16), to_unsigned(12596, 16), to_unsigned(12592, 16), to_unsigned(12588, 16),
    to_unsigned(12584, 16), to_unsigned(12580, 16), to_unsigned(12576, 16), to_unsigned(12571, 16), to_unsigned(12567, 16), to_unsigned(12563, 16), to_unsigned(12559, 16), to_unsigned(12555, 16),
    to_unsigned(12551, 16), to_unsigned(12547, 16), to_unsigned(12543, 16), to_unsigned(12539, 16), to_unsigned(12535, 16), to_unsigned(12531, 16), to_unsigned(12527, 16), to_unsigned(12523, 16),
    to_unsigned(12519, 16), to_unsigned(12515, 16), to_unsigned(12511, 16), to_unsigned(12507, 16), to_unsigned(12503, 16), to_unsigned(12499, 16), to_unsigned(12495, 16), to_unsigned(12491, 16),
    to_unsigned(12487, 16), to_unsigned(12483, 16), to_unsigned(12479, 16), to_unsigned(12475, 16), to_unsigned(12471, 16), to_unsigned(12467, 16), to_unsigned(12463, 16), to_unsigned(12459, 16),
    to_unsigned(12455, 16), to_unsigned(12451, 16), to_unsigned(12447, 16), to_unsigned(12443, 16), to_unsigned(12439, 16), to_unsigned(12435, 16), to_unsigned(12431, 16), to_unsigned(12427, 16),
    to_unsigned(12423, 16), to_unsigned(12419, 16), to_unsigned(12415, 16), to_unsigned(12412, 16), to_unsigned(12408, 16), to_unsigned(12404, 16), to_unsigned(12400, 16), to_unsigned(12396, 16),
    to_unsigned(12392, 16), to_unsigned(12388, 16), to_unsigned(12384, 16), to_unsigned(12380, 16), to_unsigned(12376, 16), to_unsigned(12372, 16), to_unsigned(12368, 16), to_unsigned(12364, 16),
    to_unsigned(12361, 16), to_unsigned(12357, 16), to_unsigned(12353, 16), to_unsigned(12349, 16), to_unsigned(12345, 16), to_unsigned(12341, 16), to_unsigned(12337, 16), to_unsigned(12333, 16),
    to_unsigned(12329, 16), to_unsigned(12326, 16), to_unsigned(12322, 16), to_unsigned(12318, 16), to_unsigned(12314, 16), to_unsigned(12310, 16), to_unsigned(12306, 16), to_unsigned(12302, 16),
    to_unsigned(12299, 16), to_unsigned(12295, 16), to_unsigned(12291, 16), to_unsigned(12287, 16), to_unsigned(12283, 16), to_unsigned(12279, 16), to_unsigned(12275, 16), to_unsigned(12272, 16),
    to_unsigned(12268, 16), to_unsigned(12264, 16), to_unsigned(12260, 16), to_unsigned(12256, 16), to_unsigned(12252, 16), to_unsigned(12249, 16), to_unsigned(12245, 16), to_unsigned(12241, 16),
    to_unsigned(12237, 16), to_unsigned(12233, 16), to_unsigned(12230, 16), to_unsigned(12226, 16), to_unsigned(12222, 16), to_unsigned(12218, 16), to_unsigned(12214, 16), to_unsigned(12211, 16),
    to_unsigned(12207, 16), to_unsigned(12203, 16), to_unsigned(12199, 16), to_unsigned(12195, 16), to_unsigned(12192, 16), to_unsigned(12188, 16), to_unsigned(12184, 16), to_unsigned(12180, 16),
    to_unsigned(12177, 16), to_unsigned(12173, 16), to_unsigned(12169, 16), to_unsigned(12165, 16), to_unsigned(12161, 16), to_unsigned(12158, 16), to_unsigned(12154, 16), to_unsigned(12150, 16),
    to_unsigned(12146, 16), to_unsigned(12143, 16), to_unsigned(12139, 16), to_unsigned(12135, 16), to_unsigned(12132, 16), to_unsigned(12128, 16), to_unsigned(12124, 16), to_unsigned(12120, 16),
    to_unsigned(12117, 16), to_unsigned(12113, 16), to_unsigned(12109, 16), to_unsigned(12105, 16), to_unsigned(12102, 16), to_unsigned(12098, 16), to_unsigned(12094, 16), to_unsigned(12091, 16),
    to_unsigned(12087, 16), to_unsigned(12083, 16), to_unsigned(12079, 16), to_unsigned(12076, 16), to_unsigned(12072, 16), to_unsigned(12068, 16), to_unsigned(12065, 16), to_unsigned(12061, 16),
    to_unsigned(12057, 16), to_unsigned(12054, 16), to_unsigned(12050, 16), to_unsigned(12046, 16), to_unsigned(12043, 16), to_unsigned(12039, 16), to_unsigned(12035, 16), to_unsigned(12032, 16),
    to_unsigned(12028, 16), to_unsigned(12024, 16), to_unsigned(12021, 16), to_unsigned(12017, 16), to_unsigned(12013, 16), to_unsigned(12010, 16), to_unsigned(12006, 16), to_unsigned(12002, 16),
    to_unsigned(11999, 16), to_unsigned(11995, 16), to_unsigned(11991, 16), to_unsigned(11988, 16), to_unsigned(11984, 16), to_unsigned(11981, 16), to_unsigned(11977, 16), to_unsigned(11973, 16),
    to_unsigned(11970, 16), to_unsigned(11966, 16), to_unsigned(11962, 16), to_unsigned(11959, 16), to_unsigned(11955, 16), to_unsigned(11952, 16), to_unsigned(11948, 16), to_unsigned(11944, 16),
    to_unsigned(11941, 16), to_unsigned(11937, 16), to_unsigned(11934, 16), to_unsigned(11930, 16), to_unsigned(11926, 16), to_unsigned(11923, 16), to_unsigned(11919, 16), to_unsigned(11916, 16),
    to_unsigned(11912, 16), to_unsigned(11909, 16), to_unsigned(11905, 16), to_unsigned(11901, 16), to_unsigned(11898, 16), to_unsigned(11894, 16), to_unsigned(11891, 16), to_unsigned(11887, 16),
    to_unsigned(11884, 16), to_unsigned(11880, 16), to_unsigned(11876, 16), to_unsigned(11873, 16), to_unsigned(11869, 16), to_unsigned(11866, 16), to_unsigned(11862, 16), to_unsigned(11859, 16),
    to_unsigned(11855, 16), to_unsigned(11852, 16), to_unsigned(11848, 16), to_unsigned(11845, 16), to_unsigned(11841, 16), to_unsigned(11838, 16), to_unsigned(11834, 16), to_unsigned(11830, 16),
    to_unsigned(11827, 16), to_unsigned(11823, 16), to_unsigned(11820, 16), to_unsigned(11816, 16), to_unsigned(11813, 16), to_unsigned(11809, 16), to_unsigned(11806, 16), to_unsigned(11802, 16),
    to_unsigned(11799, 16), to_unsigned(11795, 16), to_unsigned(11792, 16), to_unsigned(11788, 16), to_unsigned(11785, 16), to_unsigned(11781, 16), to_unsigned(11778, 16), to_unsigned(11774, 16),
    to_unsigned(11771, 16), to_unsigned(11767, 16), to_unsigned(11764, 16), to_unsigned(11761, 16), to_unsigned(11757, 16), to_unsigned(11754, 16), to_unsigned(11750, 16), to_unsigned(11747, 16),
    to_unsigned(11743, 16), to_unsigned(11740, 16), to_unsigned(11736, 16), to_unsigned(11733, 16), to_unsigned(11729, 16), to_unsigned(11726, 16), to_unsigned(11722, 16), to_unsigned(11719, 16),
    to_unsigned(11716, 16), to_unsigned(11712, 16), to_unsigned(11709, 16), to_unsigned(11705, 16), to_unsigned(11702, 16), to_unsigned(11698, 16), to_unsigned(11695, 16), to_unsigned(11692, 16),
    to_unsigned(11688, 16), to_unsigned(11685, 16), to_unsigned(11681, 16), to_unsigned(11678, 16), to_unsigned(11674, 16), to_unsigned(11671, 16), to_unsigned(11668, 16), to_unsigned(11664, 16),
    to_unsigned(11661, 16), to_unsigned(11657, 16), to_unsigned(11654, 16), to_unsigned(11651, 16), to_unsigned(11647, 16), to_unsigned(11644, 16), to_unsigned(11640, 16), to_unsigned(11637, 16),
    to_unsigned(11634, 16), to_unsigned(11630, 16), to_unsigned(11627, 16), to_unsigned(11623, 16), to_unsigned(11620, 16), to_unsigned(11617, 16), to_unsigned(11613, 16), to_unsigned(11610, 16),
    to_unsigned(11607, 16), to_unsigned(11603, 16), to_unsigned(11600, 16), to_unsigned(11596, 16), to_unsigned(11593, 16), to_unsigned(11590, 16), to_unsigned(11586, 16), to_unsigned(11583, 16),
    to_unsigned(11580, 16), to_unsigned(11576, 16), to_unsigned(11573, 16), to_unsigned(11570, 16), to_unsigned(11566, 16), to_unsigned(11563, 16), to_unsigned(11560, 16), to_unsigned(11556, 16),
    to_unsigned(11553, 16), to_unsigned(11550, 16), to_unsigned(11546, 16), to_unsigned(11543, 16), to_unsigned(11540, 16), to_unsigned(11536, 16), to_unsigned(11533, 16), to_unsigned(11530, 16),
    to_unsigned(11526, 16), to_unsigned(11523, 16), to_unsigned(11520, 16), to_unsigned(11516, 16), to_unsigned(11513, 16), to_unsigned(11510, 16), to_unsigned(11507, 16), to_unsigned(11503, 16),
    to_unsigned(11500, 16), to_unsigned(11497, 16), to_unsigned(11493, 16), to_unsigned(11490, 16), to_unsigned(11487, 16), to_unsigned(11483, 16), to_unsigned(11480, 16), to_unsigned(11477, 16),
    to_unsigned(11474, 16), to_unsigned(11470, 16), to_unsigned(11467, 16), to_unsigned(11464, 16), to_unsigned(11461, 16), to_unsigned(11457, 16), to_unsigned(11454, 16), to_unsigned(11451, 16),
    to_unsigned(11447, 16), to_unsigned(11444, 16), to_unsigned(11441, 16), to_unsigned(11438, 16), to_unsigned(11434, 16), to_unsigned(11431, 16), to_unsigned(11428, 16), to_unsigned(11425, 16),
    to_unsigned(11421, 16), to_unsigned(11418, 16), to_unsigned(11415, 16), to_unsigned(11412, 16), to_unsigned(11408, 16), to_unsigned(11405, 16), to_unsigned(11402, 16), to_unsigned(11399, 16),
    to_unsigned(11396, 16), to_unsigned(11392, 16), to_unsigned(11389, 16), to_unsigned(11386, 16), to_unsigned(11383, 16), to_unsigned(11379, 16), to_unsigned(11376, 16), to_unsigned(11373, 16),
    to_unsigned(11370, 16), to_unsigned(11367, 16), to_unsigned(11363, 16), to_unsigned(11360, 16), to_unsigned(11357, 16), to_unsigned(11354, 16), to_unsigned(11351, 16), to_unsigned(11347, 16),
    to_unsigned(11344, 16), to_unsigned(11341, 16), to_unsigned(11338, 16), to_unsigned(11335, 16), to_unsigned(11331, 16), to_unsigned(11328, 16), to_unsigned(11325, 16), to_unsigned(11322, 16),
    to_unsigned(11319, 16), to_unsigned(11316, 16), to_unsigned(11312, 16), to_unsigned(11309, 16), to_unsigned(11306, 16), to_unsigned(11303, 16), to_unsigned(11300, 16), to_unsigned(11296, 16),
    to_unsigned(11293, 16), to_unsigned(11290, 16), to_unsigned(11287, 16), to_unsigned(11284, 16), to_unsigned(11281, 16), to_unsigned(11278, 16), to_unsigned(11274, 16), to_unsigned(11271, 16),
    to_unsigned(11268, 16), to_unsigned(11265, 16), to_unsigned(11262, 16), to_unsigned(11259, 16), to_unsigned(11256, 16), to_unsigned(11252, 16), to_unsigned(11249, 16), to_unsigned(11246, 16),
    to_unsigned(11243, 16), to_unsigned(11240, 16), to_unsigned(11237, 16), to_unsigned(11234, 16), to_unsigned(11231, 16), to_unsigned(11227, 16), to_unsigned(11224, 16), to_unsigned(11221, 16),
    to_unsigned(11218, 16), to_unsigned(11215, 16), to_unsigned(11212, 16), to_unsigned(11209, 16), to_unsigned(11206, 16), to_unsigned(11203, 16), to_unsigned(11199, 16), to_unsigned(11196, 16),
    to_unsigned(11193, 16), to_unsigned(11190, 16), to_unsigned(11187, 16), to_unsigned(11184, 16), to_unsigned(11181, 16), to_unsigned(11178, 16), to_unsigned(11175, 16), to_unsigned(11172, 16),
    to_unsigned(11169, 16), to_unsigned(11165, 16), to_unsigned(11162, 16), to_unsigned(11159, 16), to_unsigned(11156, 16), to_unsigned(11153, 16), to_unsigned(11150, 16), to_unsigned(11147, 16),
    to_unsigned(11144, 16), to_unsigned(11141, 16), to_unsigned(11138, 16), to_unsigned(11135, 16), to_unsigned(11132, 16), to_unsigned(11129, 16), to_unsigned(11126, 16), to_unsigned(11123, 16),
    to_unsigned(11120, 16), to_unsigned(11116, 16), to_unsigned(11113, 16), to_unsigned(11110, 16), to_unsigned(11107, 16), to_unsigned(11104, 16), to_unsigned(11101, 16), to_unsigned(11098, 16),
    to_unsigned(11095, 16), to_unsigned(11092, 16), to_unsigned(11089, 16), to_unsigned(11086, 16), to_unsigned(11083, 16), to_unsigned(11080, 16), to_unsigned(11077, 16), to_unsigned(11074, 16),
    to_unsigned(11071, 16), to_unsigned(11068, 16), to_unsigned(11065, 16), to_unsigned(11062, 16), to_unsigned(11059, 16), to_unsigned(11056, 16), to_unsigned(11053, 16), to_unsigned(11050, 16),
    to_unsigned(11047, 16), to_unsigned(11044, 16), to_unsigned(11041, 16), to_unsigned(11038, 16), to_unsigned(11035, 16), to_unsigned(11032, 16), to_unsigned(11029, 16), to_unsigned(11026, 16),
    to_unsigned(11023, 16), to_unsigned(11020, 16), to_unsigned(11017, 16), to_unsigned(11014, 16), to_unsigned(11011, 16), to_unsigned(11008, 16), to_unsigned(11005, 16), to_unsigned(11002, 16),
    to_unsigned(10999, 16), to_unsigned(10996, 16), to_unsigned(10993, 16), to_unsigned(10990, 16), to_unsigned(10987, 16), to_unsigned(10984, 16), to_unsigned(10981, 16), to_unsigned(10978, 16),
    to_unsigned(10975, 16), to_unsigned(10972, 16), to_unsigned(10970, 16), to_unsigned(10967, 16), to_unsigned(10964, 16), to_unsigned(10961, 16), to_unsigned(10958, 16), to_unsigned(10955, 16),
    to_unsigned(10952, 16), to_unsigned(10949, 16), to_unsigned(10946, 16), to_unsigned(10943, 16), to_unsigned(10940, 16), to_unsigned(10937, 16), to_unsigned(10934, 16), to_unsigned(10931, 16),
    to_unsigned(10928, 16), to_unsigned(10925, 16), to_unsigned(10923, 16), to_unsigned(10920, 16), to_unsigned(10917, 16), to_unsigned(10914, 16), to_unsigned(10911, 16), to_unsigned(10908, 16),
    to_unsigned(10905, 16), to_unsigned(10902, 16), to_unsigned(10899, 16), to_unsigned(10896, 16), to_unsigned(10893, 16), to_unsigned(10890, 16), to_unsigned(10888, 16), to_unsigned(10885, 16),
    to_unsigned(10882, 16), to_unsigned(10879, 16), to_unsigned(10876, 16), to_unsigned(10873, 16), to_unsigned(10870, 16), to_unsigned(10867, 16), to_unsigned(10864, 16), to_unsigned(10862, 16),
    to_unsigned(10859, 16), to_unsigned(10856, 16), to_unsigned(10853, 16), to_unsigned(10850, 16), to_unsigned(10847, 16), to_unsigned(10844, 16), to_unsigned(10841, 16), to_unsigned(10838, 16),
    to_unsigned(10836, 16), to_unsigned(10833, 16), to_unsigned(10830, 16), to_unsigned(10827, 16), to_unsigned(10824, 16), to_unsigned(10821, 16), to_unsigned(10818, 16), to_unsigned(10816, 16),
    to_unsigned(10813, 16), to_unsigned(10810, 16), to_unsigned(10807, 16), to_unsigned(10804, 16), to_unsigned(10801, 16), to_unsigned(10798, 16), to_unsigned(10796, 16), to_unsigned(10793, 16),
    to_unsigned(10790, 16), to_unsigned(10787, 16), to_unsigned(10784, 16), to_unsigned(10781, 16), to_unsigned(10779, 16), to_unsigned(10776, 16), to_unsigned(10773, 16), to_unsigned(10770, 16),
    to_unsigned(10767, 16), to_unsigned(10764, 16), to_unsigned(10762, 16), to_unsigned(10759, 16), to_unsigned(10756, 16), to_unsigned(10753, 16), to_unsigned(10750, 16), to_unsigned(10747, 16),
    to_unsigned(10745, 16), to_unsigned(10742, 16), to_unsigned(10739, 16), to_unsigned(10736, 16), to_unsigned(10733, 16), to_unsigned(10731, 16), to_unsigned(10728, 16), to_unsigned(10725, 16),
    to_unsigned(10722, 16), to_unsigned(10719, 16), to_unsigned(10717, 16), to_unsigned(10714, 16), to_unsigned(10711, 16), to_unsigned(10708, 16), to_unsigned(10705, 16), to_unsigned(10703, 16),
    to_unsigned(10700, 16), to_unsigned(10697, 16), to_unsigned(10694, 16), to_unsigned(10692, 16), to_unsigned(10689, 16), to_unsigned(10686, 16), to_unsigned(10683, 16), to_unsigned(10680, 16),
    to_unsigned(10678, 16), to_unsigned(10675, 16), to_unsigned(10672, 16), to_unsigned(10669, 16), to_unsigned(10667, 16), to_unsigned(10664, 16), to_unsigned(10661, 16), to_unsigned(10658, 16),
    to_unsigned(10656, 16), to_unsigned(10653, 16), to_unsigned(10650, 16), to_unsigned(10647, 16), to_unsigned(10644, 16), to_unsigned(10642, 16), to_unsigned(10639, 16), to_unsigned(10636, 16),
    to_unsigned(10633, 16), to_unsigned(10631, 16), to_unsigned(10628, 16), to_unsigned(10625, 16), to_unsigned(10623, 16), to_unsigned(10620, 16), to_unsigned(10617, 16), to_unsigned(10614, 16),
    to_unsigned(10612, 16), to_unsigned(10609, 16), to_unsigned(10606, 16), to_unsigned(10603, 16), to_unsigned(10601, 16), to_unsigned(10598, 16), to_unsigned(10595, 16), to_unsigned(10592, 16),
    to_unsigned(10590, 16), to_unsigned(10587, 16), to_unsigned(10584, 16), to_unsigned(10582, 16), to_unsigned(10579, 16), to_unsigned(10576, 16), to_unsigned(10573, 16), to_unsigned(10571, 16),
    to_unsigned(10568, 16), to_unsigned(10565, 16), to_unsigned(10563, 16), to_unsigned(10560, 16), to_unsigned(10557, 16), to_unsigned(10555, 16), to_unsigned(10552, 16), to_unsigned(10549, 16),
    to_unsigned(10546, 16), to_unsigned(10544, 16), to_unsigned(10541, 16), to_unsigned(10538, 16), to_unsigned(10536, 16), to_unsigned(10533, 16), to_unsigned(10530, 16), to_unsigned(10528, 16),
    to_unsigned(10525, 16), to_unsigned(10522, 16), to_unsigned(10520, 16), to_unsigned(10517, 16), to_unsigned(10514, 16), to_unsigned(10512, 16), to_unsigned(10509, 16), to_unsigned(10506, 16),
    to_unsigned(10504, 16), to_unsigned(10501, 16), to_unsigned(10498, 16), to_unsigned(10496, 16), to_unsigned(10493, 16), to_unsigned(10490, 16), to_unsigned(10488, 16), to_unsigned(10485, 16),
    to_unsigned(10482, 16), to_unsigned(10480, 16), to_unsigned(10477, 16), to_unsigned(10474, 16), to_unsigned(10472, 16), to_unsigned(10469, 16), to_unsigned(10466, 16), to_unsigned(10464, 16),
    to_unsigned(10461, 16), to_unsigned(10458, 16), to_unsigned(10456, 16), to_unsigned(10453, 16), to_unsigned(10450, 16), to_unsigned(10448, 16), to_unsigned(10445, 16), to_unsigned(10443, 16),
    to_unsigned(10440, 16), to_unsigned(10437, 16), to_unsigned(10435, 16), to_unsigned(10432, 16), to_unsigned(10429, 16), to_unsigned(10427, 16), to_unsigned(10424, 16), to_unsigned(10422, 16),
    to_unsigned(10419, 16), to_unsigned(10416, 16), to_unsigned(10414, 16), to_unsigned(10411, 16), to_unsigned(10408, 16), to_unsigned(10406, 16), to_unsigned(10403, 16), to_unsigned(10401, 16),
    to_unsigned(10398, 16), to_unsigned(10395, 16), to_unsigned(10393, 16), to_unsigned(10390, 16), to_unsigned(10388, 16), to_unsigned(10385, 16), to_unsigned(10382, 16), to_unsigned(10380, 16),
    to_unsigned(10377, 16), to_unsigned(10375, 16), to_unsigned(10372, 16), to_unsigned(10369, 16), to_unsigned(10367, 16), to_unsigned(10364, 16), to_unsigned(10362, 16), to_unsigned(10359, 16),
    to_unsigned(10357, 16), to_unsigned(10354, 16), to_unsigned(10351, 16), to_unsigned(10349, 16), to_unsigned(10346, 16), to_unsigned(10344, 16), to_unsigned(10341, 16), to_unsigned(10339, 16),
    to_unsigned(10336, 16), to_unsigned(10333, 16), to_unsigned(10331, 16), to_unsigned(10328, 16), to_unsigned(10326, 16), to_unsigned(10323, 16), to_unsigned(10321, 16), to_unsigned(10318, 16),
    to_unsigned(10315, 16), to_unsigned(10313, 16), to_unsigned(10310, 16), to_unsigned(10308, 16), to_unsigned(10305, 16), to_unsigned(10303, 16), to_unsigned(10300, 16), to_unsigned(10298, 16),
    to_unsigned(10295, 16), to_unsigned(10292, 16), to_unsigned(10290, 16), to_unsigned(10287, 16), to_unsigned(10285, 16), to_unsigned(10282, 16), to_unsigned(10280, 16), to_unsigned(10277, 16),
    to_unsigned(10275, 16), to_unsigned(10272, 16), to_unsigned(10270, 16), to_unsigned(10267, 16), to_unsigned(10265, 16), to_unsigned(10262, 16), to_unsigned(10259, 16), to_unsigned(10257, 16),
    to_unsigned(10254, 16), to_unsigned(10252, 16), to_unsigned(10249, 16), to_unsigned(10247, 16), to_unsigned(10244, 16), to_unsigned(10242, 16), to_unsigned(10239, 16), to_unsigned(10237, 16),
    to_unsigned(10234, 16), to_unsigned(10232, 16), to_unsigned(10229, 16), to_unsigned(10227, 16), to_unsigned(10224, 16), to_unsigned(10222, 16), to_unsigned(10219, 16), to_unsigned(10217, 16),
    to_unsigned(10214, 16), to_unsigned(10212, 16), to_unsigned(10209, 16), to_unsigned(10207, 16), to_unsigned(10204, 16), to_unsigned(10202, 16), to_unsigned(10199, 16), to_unsigned(10197, 16),
    to_unsigned(10194, 16), to_unsigned(10192, 16), to_unsigned(10189, 16), to_unsigned(10187, 16), to_unsigned(10184, 16), to_unsigned(10182, 16), to_unsigned(10179, 16), to_unsigned(10177, 16),
    to_unsigned(10174, 16), to_unsigned(10172, 16), to_unsigned(10169, 16), to_unsigned(10167, 16), to_unsigned(10165, 16), to_unsigned(10162, 16), to_unsigned(10160, 16), to_unsigned(10157, 16),
    to_unsigned(10155, 16), to_unsigned(10152, 16), to_unsigned(10150, 16), to_unsigned(10147, 16), to_unsigned(10145, 16), to_unsigned(10142, 16), to_unsigned(10140, 16), to_unsigned(10137, 16),
    to_unsigned(10135, 16), to_unsigned(10133, 16), to_unsigned(10130, 16), to_unsigned(10128, 16), to_unsigned(10125, 16), to_unsigned(10123, 16), to_unsigned(10120, 16), to_unsigned(10118, 16),
    to_unsigned(10115, 16), to_unsigned(10113, 16), to_unsigned(10110, 16), to_unsigned(10108, 16), to_unsigned(10106, 16), to_unsigned(10103, 16), to_unsigned(10101, 16), to_unsigned(10098, 16),
    to_unsigned(10096, 16), to_unsigned(10093, 16), to_unsigned(10091, 16), to_unsigned(10089, 16), to_unsigned(10086, 16), to_unsigned(10084, 16), to_unsigned(10081, 16), to_unsigned(10079, 16),
    to_unsigned(10076, 16), to_unsigned(10074, 16), to_unsigned(10072, 16), to_unsigned(10069, 16), to_unsigned(10067, 16), to_unsigned(10064, 16), to_unsigned(10062, 16), to_unsigned(10060, 16),
    to_unsigned(10057, 16), to_unsigned(10055, 16), to_unsigned(10052, 16), to_unsigned(10050, 16), to_unsigned(10047, 16), to_unsigned(10045, 16), to_unsigned(10043, 16), to_unsigned(10040, 16),
    to_unsigned(10038, 16), to_unsigned(10035, 16), to_unsigned(10033, 16), to_unsigned(10031, 16), to_unsigned(10028, 16), to_unsigned(10026, 16), to_unsigned(10023, 16), to_unsigned(10021, 16),
    to_unsigned(10019, 16), to_unsigned(10016, 16), to_unsigned(10014, 16), to_unsigned(10012, 16), to_unsigned(10009, 16), to_unsigned(10007, 16), to_unsigned(10004, 16), to_unsigned(10002, 16),
    to_unsigned(10000, 16), to_unsigned(9997, 16), to_unsigned(9995, 16), to_unsigned(9992, 16), to_unsigned(9990, 16), to_unsigned(9988, 16), to_unsigned(9985, 16), to_unsigned(9983, 16),
    to_unsigned(9981, 16), to_unsigned(9978, 16), to_unsigned(9976, 16), to_unsigned(9974, 16), to_unsigned(9971, 16), to_unsigned(9969, 16), to_unsigned(9966, 16), to_unsigned(9964, 16),
    to_unsigned(9962, 16), to_unsigned(9959, 16), to_unsigned(9957, 16), to_unsigned(9955, 16), to_unsigned(9952, 16), to_unsigned(9950, 16), to_unsigned(9948, 16), to_unsigned(9945, 16),
    to_unsigned(9943, 16), to_unsigned(9941, 16), to_unsigned(9938, 16), to_unsigned(9936, 16), to_unsigned(9934, 16), to_unsigned(9931, 16), to_unsigned(9929, 16), to_unsigned(9926, 16),
    to_unsigned(9924, 16), to_unsigned(9922, 16), to_unsigned(9919, 16), to_unsigned(9917, 16), to_unsigned(9915, 16), to_unsigned(9912, 16), to_unsigned(9910, 16), to_unsigned(9908, 16),
    to_unsigned(9905, 16), to_unsigned(9903, 16), to_unsigned(9901, 16), to_unsigned(9899, 16), to_unsigned(9896, 16), to_unsigned(9894, 16), to_unsigned(9892, 16), to_unsigned(9889, 16),
    to_unsigned(9887, 16), to_unsigned(9885, 16), to_unsigned(9882, 16), to_unsigned(9880, 16), to_unsigned(9878, 16), to_unsigned(9875, 16), to_unsigned(9873, 16), to_unsigned(9871, 16),
    to_unsigned(9868, 16), to_unsigned(9866, 16), to_unsigned(9864, 16), to_unsigned(9862, 16), to_unsigned(9859, 16), to_unsigned(9857, 16), to_unsigned(9855, 16), to_unsigned(9852, 16),
    to_unsigned(9850, 16), to_unsigned(9848, 16), to_unsigned(9845, 16), to_unsigned(9843, 16), to_unsigned(9841, 16), to_unsigned(9839, 16), to_unsigned(9836, 16), to_unsigned(9834, 16),
    to_unsigned(9832, 16), to_unsigned(9829, 16), to_unsigned(9827, 16), to_unsigned(9825, 16), to_unsigned(9823, 16), to_unsigned(9820, 16), to_unsigned(9818, 16), to_unsigned(9816, 16),
    to_unsigned(9813, 16), to_unsigned(9811, 16), to_unsigned(9809, 16), to_unsigned(9807, 16), to_unsigned(9804, 16), to_unsigned(9802, 16), to_unsigned(9800, 16), to_unsigned(9797, 16),
    to_unsigned(9795, 16), to_unsigned(9793, 16), to_unsigned(9791, 16), to_unsigned(9788, 16), to_unsigned(9786, 16), to_unsigned(9784, 16), to_unsigned(9782, 16), to_unsigned(9779, 16),
    to_unsigned(9777, 16), to_unsigned(9775, 16), to_unsigned(9773, 16), to_unsigned(9770, 16), to_unsigned(9768, 16), to_unsigned(9766, 16), to_unsigned(9764, 16), to_unsigned(9761, 16),
    to_unsigned(9759, 16), to_unsigned(9757, 16), to_unsigned(9755, 16), to_unsigned(9752, 16), to_unsigned(9750, 16), to_unsigned(9748, 16), to_unsigned(9746, 16), to_unsigned(9743, 16),
    to_unsigned(9741, 16), to_unsigned(9739, 16), to_unsigned(9737, 16), to_unsigned(9734, 16), to_unsigned(9732, 16), to_unsigned(9730, 16), to_unsigned(9728, 16), to_unsigned(9726, 16),
    to_unsigned(9723, 16), to_unsigned(9721, 16), to_unsigned(9719, 16), to_unsigned(9717, 16), to_unsigned(9714, 16), to_unsigned(9712, 16), to_unsigned(9710, 16), to_unsigned(9708, 16),
    to_unsigned(9705, 16), to_unsigned(9703, 16), to_unsigned(9701, 16), to_unsigned(9699, 16), to_unsigned(9697, 16), to_unsigned(9694, 16), to_unsigned(9692, 16), to_unsigned(9690, 16),
    to_unsigned(9688, 16), to_unsigned(9686, 16), to_unsigned(9683, 16), to_unsigned(9681, 16), to_unsigned(9679, 16), to_unsigned(9677, 16), to_unsigned(9675, 16), to_unsigned(9672, 16),
    to_unsigned(9670, 16), to_unsigned(9668, 16), to_unsigned(9666, 16), to_unsigned(9664, 16), to_unsigned(9661, 16), to_unsigned(9659, 16), to_unsigned(9657, 16), to_unsigned(9655, 16),
    to_unsigned(9653, 16), to_unsigned(9650, 16), to_unsigned(9648, 16), to_unsigned(9646, 16), to_unsigned(9644, 16), to_unsigned(9642, 16), to_unsigned(9639, 16), to_unsigned(9637, 16),
    to_unsigned(9635, 16), to_unsigned(9633, 16), to_unsigned(9631, 16), to_unsigned(9629, 16), to_unsigned(9626, 16), to_unsigned(9624, 16), to_unsigned(9622, 16), to_unsigned(9620, 16),
    to_unsigned(9618, 16), to_unsigned(9615, 16), to_unsigned(9613, 16), to_unsigned(9611, 16), to_unsigned(9609, 16), to_unsigned(9607, 16), to_unsigned(9605, 16), to_unsigned(9602, 16),
    to_unsigned(9600, 16), to_unsigned(9598, 16), to_unsigned(9596, 16), to_unsigned(9594, 16), to_unsigned(9592, 16), to_unsigned(9590, 16), to_unsigned(9587, 16), to_unsigned(9585, 16),
    to_unsigned(9583, 16), to_unsigned(9581, 16), to_unsigned(9579, 16), to_unsigned(9577, 16), to_unsigned(9574, 16), to_unsigned(9572, 16), to_unsigned(9570, 16), to_unsigned(9568, 16),
    to_unsigned(9566, 16), to_unsigned(9564, 16), to_unsigned(9562, 16), to_unsigned(9559, 16), to_unsigned(9557, 16), to_unsigned(9555, 16), to_unsigned(9553, 16), to_unsigned(9551, 16),
    to_unsigned(9549, 16), to_unsigned(9547, 16), to_unsigned(9544, 16), to_unsigned(9542, 16), to_unsigned(9540, 16), to_unsigned(9538, 16), to_unsigned(9536, 16), to_unsigned(9534, 16),
    to_unsigned(9532, 16), to_unsigned(9530, 16), to_unsigned(9527, 16), to_unsigned(9525, 16), to_unsigned(9523, 16), to_unsigned(9521, 16), to_unsigned(9519, 16), to_unsigned(9517, 16),
    to_unsigned(9515, 16), to_unsigned(9513, 16), to_unsigned(9510, 16), to_unsigned(9508, 16), to_unsigned(9506, 16), to_unsigned(9504, 16), to_unsigned(9502, 16), to_unsigned(9500, 16),
    to_unsigned(9498, 16), to_unsigned(9496, 16), to_unsigned(9494, 16), to_unsigned(9491, 16), to_unsigned(9489, 16), to_unsigned(9487, 16), to_unsigned(9485, 16), to_unsigned(9483, 16),
    to_unsigned(9481, 16), to_unsigned(9479, 16), to_unsigned(9477, 16), to_unsigned(9475, 16), to_unsigned(9473, 16), to_unsigned(9470, 16), to_unsigned(9468, 16), to_unsigned(9466, 16),
    to_unsigned(9464, 16), to_unsigned(9462, 16), to_unsigned(9460, 16), to_unsigned(9458, 16), to_unsigned(9456, 16), to_unsigned(9454, 16), to_unsigned(9452, 16), to_unsigned(9450, 16),
    to_unsigned(9447, 16), to_unsigned(9445, 16), to_unsigned(9443, 16), to_unsigned(9441, 16), to_unsigned(9439, 16), to_unsigned(9437, 16), to_unsigned(9435, 16), to_unsigned(9433, 16),
    to_unsigned(9431, 16), to_unsigned(9429, 16), to_unsigned(9427, 16), to_unsigned(9425, 16), to_unsigned(9423, 16), to_unsigned(9420, 16), to_unsigned(9418, 16), to_unsigned(9416, 16),
    to_unsigned(9414, 16), to_unsigned(9412, 16), to_unsigned(9410, 16), to_unsigned(9408, 16), to_unsigned(9406, 16), to_unsigned(9404, 16), to_unsigned(9402, 16), to_unsigned(9400, 16),
    to_unsigned(9398, 16), to_unsigned(9396, 16), to_unsigned(9394, 16), to_unsigned(9392, 16), to_unsigned(9390, 16), to_unsigned(9387, 16), to_unsigned(9385, 16), to_unsigned(9383, 16),
    to_unsigned(9381, 16), to_unsigned(9379, 16), to_unsigned(9377, 16), to_unsigned(9375, 16), to_unsigned(9373, 16), to_unsigned(9371, 16), to_unsigned(9369, 16), to_unsigned(9367, 16),
    to_unsigned(9365, 16), to_unsigned(9363, 16), to_unsigned(9361, 16), to_unsigned(9359, 16), to_unsigned(9357, 16), to_unsigned(9355, 16), to_unsigned(9353, 16), to_unsigned(9351, 16),
    to_unsigned(9349, 16), to_unsigned(9347, 16), to_unsigned(9345, 16), to_unsigned(9343, 16), to_unsigned(9341, 16), to_unsigned(9338, 16), to_unsigned(9336, 16), to_unsigned(9334, 16),
    to_unsigned(9332, 16), to_unsigned(9330, 16), to_unsigned(9328, 16), to_unsigned(9326, 16), to_unsigned(9324, 16), to_unsigned(9322, 16), to_unsigned(9320, 16), to_unsigned(9318, 16),
    to_unsigned(9316, 16), to_unsigned(9314, 16), to_unsigned(9312, 16), to_unsigned(9310, 16), to_unsigned(9308, 16), to_unsigned(9306, 16), to_unsigned(9304, 16), to_unsigned(9302, 16),
    to_unsigned(9300, 16), to_unsigned(9298, 16), to_unsigned(9296, 16), to_unsigned(9294, 16), to_unsigned(9292, 16), to_unsigned(9290, 16), to_unsigned(9288, 16), to_unsigned(9286, 16),
    to_unsigned(9284, 16), to_unsigned(9282, 16), to_unsigned(9280, 16), to_unsigned(9278, 16), to_unsigned(9276, 16), to_unsigned(9274, 16), to_unsigned(9272, 16), to_unsigned(9270, 16),
    to_unsigned(9268, 16), to_unsigned(9266, 16), to_unsigned(9264, 16), to_unsigned(9262, 16), to_unsigned(9260, 16), to_unsigned(9258, 16), to_unsigned(9256, 16), to_unsigned(9254, 16),
    to_unsigned(9252, 16), to_unsigned(9250, 16), to_unsigned(9248, 16), to_unsigned(9246, 16), to_unsigned(9244, 16), to_unsigned(9242, 16), to_unsigned(9240, 16), to_unsigned(9238, 16),
    to_unsigned(9236, 16), to_unsigned(9234, 16), to_unsigned(9232, 16), to_unsigned(9230, 16), to_unsigned(9228, 16), to_unsigned(9226, 16), to_unsigned(9225, 16), to_unsigned(9223, 16),
    to_unsigned(9221, 16), to_unsigned(9219, 16), to_unsigned(9217, 16), to_unsigned(9215, 16), to_unsigned(9213, 16), to_unsigned(9211, 16), to_unsigned(9209, 16), to_unsigned(9207, 16),
    to_unsigned(9205, 16), to_unsigned(9203, 16), to_unsigned(9201, 16), to_unsigned(9199, 16), to_unsigned(9197, 16), to_unsigned(9195, 16), to_unsigned(9193, 16), to_unsigned(9191, 16),
    to_unsigned(9189, 16), to_unsigned(9187, 16), to_unsigned(9185, 16), to_unsigned(9183, 16), to_unsigned(9181, 16), to_unsigned(9179, 16), to_unsigned(9177, 16), to_unsigned(9176, 16),
    to_unsigned(9174, 16), to_unsigned(9172, 16), to_unsigned(9170, 16), to_unsigned(9168, 16), to_unsigned(9166, 16), to_unsigned(9164, 16), to_unsigned(9162, 16), to_unsigned(9160, 16),
    to_unsigned(9158, 16), to_unsigned(9156, 16), to_unsigned(9154, 16), to_unsigned(9152, 16), to_unsigned(9150, 16), to_unsigned(9148, 16), to_unsigned(9146, 16), to_unsigned(9145, 16),
    to_unsigned(9143, 16), to_unsigned(9141, 16), to_unsigned(9139, 16), to_unsigned(9137, 16), to_unsigned(9135, 16), to_unsigned(9133, 16), to_unsigned(9131, 16), to_unsigned(9129, 16),
    to_unsigned(9127, 16), to_unsigned(9125, 16), to_unsigned(9123, 16), to_unsigned(9121, 16), to_unsigned(9119, 16), to_unsigned(9118, 16), to_unsigned(9116, 16), to_unsigned(9114, 16),
    to_unsigned(9112, 16), to_unsigned(9110, 16), to_unsigned(9108, 16), to_unsigned(9106, 16), to_unsigned(9104, 16), to_unsigned(9102, 16), to_unsigned(9100, 16), to_unsigned(9098, 16),
    to_unsigned(9097, 16), to_unsigned(9095, 16), to_unsigned(9093, 16), to_unsigned(9091, 16), to_unsigned(9089, 16), to_unsigned(9087, 16), to_unsigned(9085, 16), to_unsigned(9083, 16),
    to_unsigned(9081, 16), to_unsigned(9079, 16), to_unsigned(9077, 16), to_unsigned(9076, 16), to_unsigned(9074, 16), to_unsigned(9072, 16), to_unsigned(9070, 16), to_unsigned(9068, 16),
    to_unsigned(9066, 16), to_unsigned(9064, 16), to_unsigned(9062, 16), to_unsigned(9060, 16), to_unsigned(9059, 16), to_unsigned(9057, 16), to_unsigned(9055, 16), to_unsigned(9053, 16),
    to_unsigned(9051, 16), to_unsigned(9049, 16), to_unsigned(9047, 16), to_unsigned(9045, 16), to_unsigned(9043, 16), to_unsigned(9042, 16), to_unsigned(9040, 16), to_unsigned(9038, 16),
    to_unsigned(9036, 16), to_unsigned(9034, 16), to_unsigned(9032, 16), to_unsigned(9030, 16), to_unsigned(9028, 16), to_unsigned(9027, 16), to_unsigned(9025, 16), to_unsigned(9023, 16),
    to_unsigned(9021, 16), to_unsigned(9019, 16), to_unsigned(9017, 16), to_unsigned(9015, 16), to_unsigned(9013, 16), to_unsigned(9012, 16), to_unsigned(9010, 16), to_unsigned(9008, 16),
    to_unsigned(9006, 16), to_unsigned(9004, 16), to_unsigned(9002, 16), to_unsigned(9000, 16), to_unsigned(8999, 16), to_unsigned(8997, 16), to_unsigned(8995, 16), to_unsigned(8993, 16),
    to_unsigned(8991, 16), to_unsigned(8989, 16), to_unsigned(8987, 16), to_unsigned(8986, 16), to_unsigned(8984, 16), to_unsigned(8982, 16), to_unsigned(8980, 16), to_unsigned(8978, 16),
    to_unsigned(8976, 16), to_unsigned(8974, 16), to_unsigned(8973, 16), to_unsigned(8971, 16), to_unsigned(8969, 16), to_unsigned(8967, 16), to_unsigned(8965, 16), to_unsigned(8963, 16),
    to_unsigned(8962, 16), to_unsigned(8960, 16), to_unsigned(8958, 16), to_unsigned(8956, 16), to_unsigned(8954, 16), to_unsigned(8952, 16), to_unsigned(8950, 16), to_unsigned(8949, 16),
    to_unsigned(8947, 16), to_unsigned(8945, 16), to_unsigned(8943, 16), to_unsigned(8941, 16), to_unsigned(8939, 16), to_unsigned(8938, 16), to_unsigned(8936, 16), to_unsigned(8934, 16),
    to_unsigned(8932, 16), to_unsigned(8930, 16), to_unsigned(8929, 16), to_unsigned(8927, 16), to_unsigned(8925, 16), to_unsigned(8923, 16), to_unsigned(8921, 16), to_unsigned(8919, 16),
    to_unsigned(8918, 16), to_unsigned(8916, 16), to_unsigned(8914, 16), to_unsigned(8912, 16), to_unsigned(8910, 16), to_unsigned(8908, 16), to_unsigned(8907, 16), to_unsigned(8905, 16),
    to_unsigned(8903, 16), to_unsigned(8901, 16), to_unsigned(8899, 16), to_unsigned(8898, 16), to_unsigned(8896, 16), to_unsigned(8894, 16), to_unsigned(8892, 16), to_unsigned(8890, 16),
    to_unsigned(8889, 16), to_unsigned(8887, 16), to_unsigned(8885, 16), to_unsigned(8883, 16), to_unsigned(8881, 16), to_unsigned(8880, 16), to_unsigned(8878, 16), to_unsigned(8876, 16),
    to_unsigned(8874, 16), to_unsigned(8872, 16), to_unsigned(8870, 16), to_unsigned(8869, 16), to_unsigned(8867, 16), to_unsigned(8865, 16), to_unsigned(8863, 16), to_unsigned(8862, 16),
    to_unsigned(8860, 16), to_unsigned(8858, 16), to_unsigned(8856, 16), to_unsigned(8854, 16), to_unsigned(8853, 16), to_unsigned(8851, 16), to_unsigned(8849, 16), to_unsigned(8847, 16),
    to_unsigned(8845, 16), to_unsigned(8844, 16), to_unsigned(8842, 16), to_unsigned(8840, 16), to_unsigned(8838, 16), to_unsigned(8836, 16), to_unsigned(8835, 16), to_unsigned(8833, 16),
    to_unsigned(8831, 16), to_unsigned(8829, 16), to_unsigned(8828, 16), to_unsigned(8826, 16), to_unsigned(8824, 16), to_unsigned(8822, 16), to_unsigned(8820, 16), to_unsigned(8819, 16),
    to_unsigned(8817, 16), to_unsigned(8815, 16), to_unsigned(8813, 16), to_unsigned(8812, 16), to_unsigned(8810, 16), to_unsigned(8808, 16), to_unsigned(8806, 16), to_unsigned(8805, 16),
    to_unsigned(8803, 16), to_unsigned(8801, 16), to_unsigned(8799, 16), to_unsigned(8797, 16), to_unsigned(8796, 16), to_unsigned(8794, 16), to_unsigned(8792, 16), to_unsigned(8790, 16),
    to_unsigned(8789, 16), to_unsigned(8787, 16), to_unsigned(8785, 16), to_unsigned(8783, 16), to_unsigned(8782, 16), to_unsigned(8780, 16), to_unsigned(8778, 16), to_unsigned(8776, 16),
    to_unsigned(8775, 16), to_unsigned(8773, 16), to_unsigned(8771, 16), to_unsigned(8769, 16), to_unsigned(8768, 16), to_unsigned(8766, 16), to_unsigned(8764, 16), to_unsigned(8762, 16),
    to_unsigned(8761, 16), to_unsigned(8759, 16), to_unsigned(8757, 16), to_unsigned(8755, 16), to_unsigned(8754, 16), to_unsigned(8752, 16), to_unsigned(8750, 16), to_unsigned(8748, 16),
    to_unsigned(8747, 16), to_unsigned(8745, 16), to_unsigned(8743, 16), to_unsigned(8741, 16), to_unsigned(8740, 16), to_unsigned(8738, 16), to_unsigned(8736, 16), to_unsigned(8734, 16),
    to_unsigned(8733, 16), to_unsigned(8731, 16), to_unsigned(8729, 16), to_unsigned(8728, 16), to_unsigned(8726, 16), to_unsigned(8724, 16), to_unsigned(8722, 16), to_unsigned(8721, 16),
    to_unsigned(8719, 16), to_unsigned(8717, 16), to_unsigned(8715, 16), to_unsigned(8714, 16), to_unsigned(8712, 16), to_unsigned(8710, 16), to_unsigned(8709, 16), to_unsigned(8707, 16),
    to_unsigned(8705, 16), to_unsigned(8703, 16), to_unsigned(8702, 16), to_unsigned(8700, 16), to_unsigned(8698, 16), to_unsigned(8696, 16), to_unsigned(8695, 16), to_unsigned(8693, 16),
    to_unsigned(8691, 16), to_unsigned(8690, 16), to_unsigned(8688, 16), to_unsigned(8686, 16), to_unsigned(8684, 16), to_unsigned(8683, 16), to_unsigned(8681, 16), to_unsigned(8679, 16),
    to_unsigned(8678, 16), to_unsigned(8676, 16), to_unsigned(8674, 16), to_unsigned(8672, 16), to_unsigned(8671, 16), to_unsigned(8669, 16), to_unsigned(8667, 16), to_unsigned(8666, 16),
    to_unsigned(8664, 16), to_unsigned(8662, 16), to_unsigned(8661, 16), to_unsigned(8659, 16), to_unsigned(8657, 16), to_unsigned(8655, 16), to_unsigned(8654, 16), to_unsigned(8652, 16),
    to_unsigned(8650, 16), to_unsigned(8649, 16), to_unsigned(8647, 16), to_unsigned(8645, 16), to_unsigned(8644, 16), to_unsigned(8642, 16), to_unsigned(8640, 16), to_unsigned(8639, 16),
    to_unsigned(8637, 16), to_unsigned(8635, 16), to_unsigned(8633, 16), to_unsigned(8632, 16), to_unsigned(8630, 16), to_unsigned(8628, 16), to_unsigned(8627, 16), to_unsigned(8625, 16),
    to_unsigned(8623, 16), to_unsigned(8622, 16), to_unsigned(8620, 16), to_unsigned(8618, 16), to_unsigned(8617, 16), to_unsigned(8615, 16), to_unsigned(8613, 16), to_unsigned(8612, 16),
    to_unsigned(8610, 16), to_unsigned(8608, 16), to_unsigned(8607, 16), to_unsigned(8605, 16), to_unsigned(8603, 16), to_unsigned(8601, 16), to_unsigned(8600, 16), to_unsigned(8598, 16),
    to_unsigned(8596, 16), to_unsigned(8595, 16), to_unsigned(8593, 16), to_unsigned(8591, 16), to_unsigned(8590, 16), to_unsigned(8588, 16), to_unsigned(8586, 16), to_unsigned(8585, 16),
    to_unsigned(8583, 16), to_unsigned(8581, 16), to_unsigned(8580, 16), to_unsigned(8578, 16), to_unsigned(8576, 16), to_unsigned(8575, 16), to_unsigned(8573, 16), to_unsigned(8571, 16),
    to_unsigned(8570, 16), to_unsigned(8568, 16), to_unsigned(8567, 16), to_unsigned(8565, 16), to_unsigned(8563, 16), to_unsigned(8562, 16), to_unsigned(8560, 16), to_unsigned(8558, 16),
    to_unsigned(8557, 16), to_unsigned(8555, 16), to_unsigned(8553, 16), to_unsigned(8552, 16), to_unsigned(8550, 16), to_unsigned(8548, 16), to_unsigned(8547, 16), to_unsigned(8545, 16),
    to_unsigned(8543, 16), to_unsigned(8542, 16), to_unsigned(8540, 16), to_unsigned(8538, 16), to_unsigned(8537, 16), to_unsigned(8535, 16), to_unsigned(8533, 16), to_unsigned(8532, 16),
    to_unsigned(8530, 16), to_unsigned(8529, 16), to_unsigned(8527, 16), to_unsigned(8525, 16), to_unsigned(8524, 16), to_unsigned(8522, 16), to_unsigned(8520, 16), to_unsigned(8519, 16),
    to_unsigned(8517, 16), to_unsigned(8515, 16), to_unsigned(8514, 16), to_unsigned(8512, 16), to_unsigned(8511, 16), to_unsigned(8509, 16), to_unsigned(8507, 16), to_unsigned(8506, 16),
    to_unsigned(8504, 16), to_unsigned(8502, 16), to_unsigned(8501, 16), to_unsigned(8499, 16), to_unsigned(8498, 16), to_unsigned(8496, 16), to_unsigned(8494, 16), to_unsigned(8493, 16),
    to_unsigned(8491, 16), to_unsigned(8489, 16), to_unsigned(8488, 16), to_unsigned(8486, 16), to_unsigned(8485, 16), to_unsigned(8483, 16), to_unsigned(8481, 16), to_unsigned(8480, 16),
    to_unsigned(8478, 16), to_unsigned(8476, 16), to_unsigned(8475, 16), to_unsigned(8473, 16), to_unsigned(8472, 16), to_unsigned(8470, 16), to_unsigned(8468, 16), to_unsigned(8467, 16),
    to_unsigned(8465, 16), to_unsigned(8463, 16), to_unsigned(8462, 16), to_unsigned(8460, 16), to_unsigned(8459, 16), to_unsigned(8457, 16), to_unsigned(8455, 16), to_unsigned(8454, 16),
    to_unsigned(8452, 16), to_unsigned(8451, 16), to_unsigned(8449, 16), to_unsigned(8447, 16), to_unsigned(8446, 16), to_unsigned(8444, 16), to_unsigned(8443, 16), to_unsigned(8441, 16),
    to_unsigned(8439, 16), to_unsigned(8438, 16), to_unsigned(8436, 16), to_unsigned(8435, 16), to_unsigned(8433, 16), to_unsigned(8431, 16), to_unsigned(8430, 16), to_unsigned(8428, 16),
    to_unsigned(8427, 16), to_unsigned(8425, 16), to_unsigned(8423, 16), to_unsigned(8422, 16), to_unsigned(8420, 16), to_unsigned(8419, 16), to_unsigned(8417, 16), to_unsigned(8415, 16),
    to_unsigned(8414, 16), to_unsigned(8412, 16), to_unsigned(8411, 16), to_unsigned(8409, 16), to_unsigned(8407, 16), to_unsigned(8406, 16), to_unsigned(8404, 16), to_unsigned(8403, 16),
    to_unsigned(8401, 16), to_unsigned(8400, 16), to_unsigned(8398, 16), to_unsigned(8396, 16), to_unsigned(8395, 16), to_unsigned(8393, 16), to_unsigned(8392, 16), to_unsigned(8390, 16),
    to_unsigned(8388, 16), to_unsigned(8387, 16), to_unsigned(8385, 16), to_unsigned(8384, 16), to_unsigned(8382, 16), to_unsigned(8381, 16), to_unsigned(8379, 16), to_unsigned(8377, 16),
    to_unsigned(8376, 16), to_unsigned(8374, 16), to_unsigned(8373, 16), to_unsigned(8371, 16), to_unsigned(8370, 16), to_unsigned(8368, 16), to_unsigned(8366, 16), to_unsigned(8365, 16),
    to_unsigned(8363, 16), to_unsigned(8362, 16), to_unsigned(8360, 16), to_unsigned(8359, 16), to_unsigned(8357, 16), to_unsigned(8355, 16), to_unsigned(8354, 16), to_unsigned(8352, 16),
    to_unsigned(8351, 16), to_unsigned(8349, 16), to_unsigned(8348, 16), to_unsigned(8346, 16), to_unsigned(8345, 16), to_unsigned(8343, 16), to_unsigned(8341, 16), to_unsigned(8340, 16),
    to_unsigned(8338, 16), to_unsigned(8337, 16), to_unsigned(8335, 16), to_unsigned(8334, 16), to_unsigned(8332, 16), to_unsigned(8331, 16), to_unsigned(8329, 16), to_unsigned(8327, 16),
    to_unsigned(8326, 16), to_unsigned(8324, 16), to_unsigned(8323, 16), to_unsigned(8321, 16), to_unsigned(8320, 16), to_unsigned(8318, 16), to_unsigned(8317, 16), to_unsigned(8315, 16),
    to_unsigned(8313, 16), to_unsigned(8312, 16), to_unsigned(8310, 16), to_unsigned(8309, 16), to_unsigned(8307, 16), to_unsigned(8306, 16), to_unsigned(8304, 16), to_unsigned(8303, 16),
    to_unsigned(8301, 16), to_unsigned(8300, 16), to_unsigned(8298, 16), to_unsigned(8297, 16), to_unsigned(8295, 16), to_unsigned(8293, 16), to_unsigned(8292, 16), to_unsigned(8290, 16),
    to_unsigned(8289, 16), to_unsigned(8287, 16), to_unsigned(8286, 16), to_unsigned(8284, 16), to_unsigned(8283, 16), to_unsigned(8281, 16), to_unsigned(8280, 16), to_unsigned(8278, 16),
    to_unsigned(8277, 16), to_unsigned(8275, 16), to_unsigned(8273, 16), to_unsigned(8272, 16), to_unsigned(8270, 16), to_unsigned(8269, 16), to_unsigned(8267, 16), to_unsigned(8266, 16),
    to_unsigned(8264, 16), to_unsigned(8263, 16), to_unsigned(8261, 16), to_unsigned(8260, 16), to_unsigned(8258, 16), to_unsigned(8257, 16), to_unsigned(8255, 16), to_unsigned(8254, 16),
    to_unsigned(8252, 16), to_unsigned(8251, 16), to_unsigned(8249, 16), to_unsigned(8248, 16), to_unsigned(8246, 16), to_unsigned(8245, 16), to_unsigned(8243, 16), to_unsigned(8242, 16),
    to_unsigned(8240, 16), to_unsigned(8239, 16), to_unsigned(8237, 16), to_unsigned(8235, 16), to_unsigned(8234, 16), to_unsigned(8232, 16), to_unsigned(8231, 16), to_unsigned(8229, 16),
    to_unsigned(8228, 16), to_unsigned(8226, 16), to_unsigned(8225, 16), to_unsigned(8223, 16), to_unsigned(8222, 16), to_unsigned(8220, 16), to_unsigned(8219, 16), to_unsigned(8217, 16)
);
    signal target_gain          : unsigned(15 downto 0) := GAIN_MAKEUP;
    
    -- ----------------------------------------------------------------
    -- Stage 4: Envelope follower signals
    -- ----------------------------------------------------------------
    -- smooth_gain stored as Q31 (upper 16 bits = Q15 gain value)
    -- Extra 16 bits of fractional precision prevent stairstepping
    signal smooth_gain          : unsigned(31 downto 0) := (others => '0');
    signal smooth_gain_q15       : unsigned(15 downto 0) := GAIN_MAKEUP;
    
    -- smoothing intermediate products
    signal attack_delta         : unsigned(31 downto 0) := (others => '0');
    signal release_delta        : unsigned(31 downto 0) := (others => '0');
    
    attribute use_dsp of attack_delta : signal is "yes";
    attribute use_dsp of release_delta : signal is "yes";
    
    -- ----------------------------------------------------------------
    -- Stage 5: Gain application signals
    -- ----------------------------------------------------------------
    signal left_gained          : signed (40 downto 0) := (others => '0');
    signal right_gained         : signed (40 downto 0) := (others => '0');
    signal left_out_int         : signed (23 downto 0) := (others => '0');
    signal right_out_int        : signed (23 downto 0) := (others => '0');
    signal peak_limit_int       : std_logic := '0';
    
    -- Force DSP48 for the audio sample multiply
    attribute use_dsp of left_gained : signal is "yes";
    attribute use_dsp of right_gained : signal is "yes";
    
    -- pipeline delay registers
    signal valid_d1             : std_logic := '0';
    signal valid_d2             : std_logic := '0';
    signal left_d1              : signed (23 downto 0) := (others => '0');
    signal right_d1             : signed (23 downto 0) := (others => '0');
    
begin
    -- Output connections
    left_out <= std_logic_vector(left_out_int);
    right_out <= std_logic_vector(right_out_int);
    sample_valid_out <= valid_d2;
    peak_limiting <= peak_limit_int;
    
    -- ----------------------------------------------------------------
    -- Stage 1a: RMS accumulator
    -- Sums squared mono samples over RMS_WINDOW samples
    -- ----------------------------------------------------------------
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            rms_valid <= '0';
            
            if rst = '1' then
                mono_in <= (others => '0');
                mono_sq <= (others => '0');
                rms_accum <= (others => '0');
                rms_count <= 0;
                
            elsif sample_valid_in = '1' then
                -- sum L+R, halve to mono (shift right = divide by 2, no DSP needed)
                mono_in <= shift_right(
                    signed(left_in)+signed(right_in), 1
                );
                 -- Square mono sample
                mono_sq <= mono_in * mono_in;
                 
                 -- accumulate the sum of squares
                rms_accum <= rms_accum + unsigned(mono_sq(47 downto 0));
                 
                if rms_count = RMS_WINDOW -1 then
                    rms_count <= 0;
                    -- Divide accumulator by window size (exact power-of-2 shift)
                    -- Result is mean square in Q46
                    -- Take upper 24 bits as LUT address input
                    rms_scaled <= rms_accum(47 downto 24);
                    rms_accum <= (others => '0');
                    rms_valid <= '1';
                else
                    rms_count <= rms_count + 1;
                end if;                    
            end if;                
        end if;
    end process;
    
    -- ----------------------------------------------------------------
    -- Stage 1b: sqrt LUT lookup
    -- Address is upper 12 bits of rms_scaled
    -- Registered output (Block RAM read latency = 1 cycle)
    -- ----------------------------------------------------------------
    process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                sqrt_addr <= (others => '0');
                rms_level <= (others => '0');
            else
                -- Address: upper 12 bits of rms_scaled
                sqrt_addr <= rms_scaled(23 downto 12);
                -- LUT output registered (Block RAM behaviour)
                sqrt_result <= SQRT_LUT(to_integer(sqrt_addr));
                 -- Scale 12-bit LUT output back to Q23 range
                -- LUT output range 0..4095, scale to 0..8388608 (Q23)
                -- Multiply by 2048 = shift left 11
                rms_level <= resize(
                    shift_left(resize(sqrt_result, 24), 11), 24
                );
            end if;
        end if;
    end process;
    
    -- ----------------------------------------------------------------
    -- Stage 2: Peak detector
    -- Instant attack (follows signal up every sample)
    -- Slow release (decays by 1 LSB per sample = ~87ms full scale decay)
    -- ----------------------------------------------------------------
    
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                abs_mono <= (others => '0');
                peak_level <= (others => '0');
            elsif sample_valid_in = '1' then
                -- Rectify: absolute value of mono input
                if mono_in(23) = '1' then
                    abs_mono <= unsigned(-mono_in);
                else
                    abs_mono <= unsigned(mono_in);
                end if;
                
                -- Instant attack, slow release
                if abs_mono > peak_level then
                    peak_level <= abs_mono;
                elsif peak_level > 0 then
                    peak_level <= peak_level -1;
                end if;
            end if;
        end if;
    end process;
    
    -- ----------------------------------------------------------------
    -- Stage 3: Gain computer
    -- Uses RMS level to determine target gain in Q15
    --
    -- Compression curve:
    --   rms > THRESH_HIGH : compress downward (ratio 4:1)
    --     gain = (THRESH_HIGH / rms) ^ (1 - 1/ratio)
    --          = (THRESH_HIGH / rms) ^ 0.75
    --     Approximated as: gain = THRESH_HIGH * GAIN_MAKEUP / rms
    --     with DSP48 for the numerator multiply
    --
    --   rms < THRESH_LOW  : expand upward (ratio 2:1)
    --     gain = GAIN_MAKEUP * (THRESH_LOW / rms) ^ 0.5
    --     Approximated as linear boost toward GAIN_MAX
    --
    --   otherwise         : apply makeup gain only
    -- ----------------------------------------------------------------
    
    process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                target_gain <= GAIN_MAKEUP;
            elsif rms_valid = '1' then
                target_gain <= GAIN_LUT(
                    to_integer(rms_level(23 downto 12))
                );
            end if;
        end if;
    end process;                                    
    
    -- ----------------------------------------------------------------
    -- Stage 4: Envelope follower
    -- First-order IIR: smooth = smooth + coeff * (target - smooth)
    -- Asymmetric: fast attack (gain going down), slow release (going up)
    -- Stored as Q31 for sub-LSB precision to prevent gain stairstepping
    -- DSP48 inferred for delta computations via use_dsp attribute
    -- ----------------------------------------------------------------
    process(sys_clk)
        variable sum_33 : unsigned(32 downto 0);
    begin
        if rising_edge(sys_clk) then
            if rst = '1' then
                 smooth_gain <= SMOOTH_GAIN_INIT;
                 smooth_gain_q15 <= GAIN_MAKEUP;
            elsif sample_valid_in = '1' then
                -- extract current Q15 gain from upper 16 bits
                smooth_gain_q15 <= smooth_gain(31 downto 16);
               
                if target_gain < smooth_gain(31 downto 16) then
                    --gain going DOWN: fast attack
                    attack_delta <= resize(
                        shift_right(
                            (smooth_gain(31 downto 16) - target_gain) *
                            ATTACK_COEFF, 15
                        ), 32
                    );                   
                    if smooth_gain > attack_delta then
                        smooth_gain <= smooth_gain - attack_delta;
                    else
                        smooth_gain <= (others => '0');
                    end if;
                elsif target_gain > smooth_gain(31 downto 16) then
                    -- gain going UP: slow release
                    release_delta <= RESIZE(
                        shift_right(
                            (target_gain - smooth_gain(31 downto 16)) * 
                            RELEASE_COEFF,
                            15
                        ), 32
                    );
                    
                    -- Check overflow safely using a 33-bit addition
                    -- If result fits in 32 bits, apply it; otherwise clamp to max
                    
                    sum_33 := resize(smooth_gain, 33) + resize(release_delta, 33);
                      
                    if sum_33(32) = '0' then
                        smooth_gain <= smooth_gain + release_delta;
                    else
                        smooth_gain <= (others => '1');
                    end if;
                end if;
            end if;
        end if;
    end process;
    
     -- ----------------------------------------------------------------
    -- Stage 5: Apply gain to audio samples
    -- Cycle 1: multiply Q23 sample by Q15 gain (DSP48)
    -- Cycle 2: scale Q38 → Q23, peak limit, saturate
    -- ----------------------------------------------------------------
    
    process(sys_clk)
    begin
        if rising_edge (sys_clk) then
            valid_d1 <= '0';
            valid_d2 <= '0';
            if rst = '1' then
                left_gained <= (others => '0');
                right_gained <= (others => '0');
                left_out_int <= (others => '0');
                right_out_int <= (others => '0');
                peak_limit_int <= '0';
                gain_db_out <= (others => '0');    
            else
                -- Cycle 1: multiply (DSP48 inferred via use_dsp)
                if sample_valid_in = '1' then
                    if enable = '1' then
                        left_gained <= signed(left_in) *
                                        signed('0' & smooth_gain_q15);
                        right_gained <= signed(right_in) * 
                                        signed('0' & smooth_gain_q15);
                    else
                        -- Bypass: pass through unchanged, padded to 40 bits
                        left_gained <= resize(signed(left_in), 41);
                        right_gained <= resize(signed(right_in), 41);
                    end if;
                    valid_d1 <= '1';
                    left_d1 <= signed(left_in);
                    right_d1 <= signed(right_in);
                end if;
                
                -- Cycle 2: scale back to Q23 and apply peak limiter                                                    
                if valid_d1 = '1' then
                    valid_d2 <= '1';
                    peak_limit_int <='0';
                    
                    -- Scale: Q38 → Q23 (shift right 15)
                    left_out_int <= left_gained(38 downto 15);
                    right_out_int <= right_gained(38 downto 15);
                    
                    -- Hard peak limiter: left channel
                    if left_gained(38 downto 15) > PEAK_CEILING then
                        left_out_int <= PEAK_CEILING;
                        peak_limit_int <= '1';
                    elsif left_gained(38 downto 15) < -PEAK_CEILING then
                        left_out_int <= -PEAK_CEILING;
                        peak_limit_int <= '1';
                    end if;
                    
                     
                    -- Hard peak limiter: right channel
                    if right_gained(38 downto 15) > PEAK_CEILING then
                        right_out_int <= PEAK_CEILING;
                        peak_limit_int <= '1';
                    elsif right_gained(38 downto 15) < -PEAK_CEILING then
                        right_out_int <= -PEAK_CEILING;
                        peak_limit_int <= '1';
                    end if; 
                    
                    -- Gain meter output: dB*2 encoded, 0dB = 64
                    gain_db_out <= std_logic_vector(
                        to_unsigned(64, 8) +
                        resize(smooth_gain(31 downto 28), 8) -
                        to_unsigned(8,8)
                    );
                end if;
            end if;
        end if;
    end process;
                                                                                                                                                                                                            
end rtl;