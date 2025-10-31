-- ============================================================================
--  Testbench for uart_isa.vhd (Self-checking, ModelSim ready)
--  Проверяет:
--    1) Сброс устройства
--    2) Запись/чтение регистров ISA
--    3) SPI-трафик от MCU и IRQ
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity uart_isa_tb is
end uart_isa_tb;

architecture behavior of uart_isa_tb is

    -- === DUT signals ===
    signal isa_addr    : std_logic_vector(9 downto 0) := (others => '0');
    signal isa_data    : std_logic_vector(7 downto 0);
    signal isa_iow     : std_logic := '1';
    signal isa_ior     : std_logic := '1';
    signal isa_aen     : std_logic := '0';
    signal isa_reset   : std_logic := '0';

    signal mcu_isa_res : std_logic;
    signal mcu_DTR     : std_logic;
    signal mcu_rxd     : std_logic := '1';
    signal mcu_clk     : std_logic := '1';
    signal mcu_res     : std_logic := '1';

    signal IRQ3, IRQ4, IRQX : std_logic := 'Z';
    signal tmp : std_logic_vector(6 downto 0);

    -- Драйвер данных на ISA-шину
    signal isa_data_drv : std_logic_vector(7 downto 0) := (others => 'Z');

    constant spi_pattern : std_logic_vector(6 downto 0) := "1010101";
    constant COM1_def_pat : std_logic_vector(6 downto 0) := "0000000";
    constant COM2_def_pat : std_logic_vector(6 downto 0) := "0000001";
    constant COM3_def_pat : std_logic_vector(6 downto 0) := "0000010";
    constant COM4_def_pat : std_logic_vector(6 downto 0) := "0000011";
    constant COM1_irqx_pat : std_logic_vector(6 downto 0) := "0000100";
    constant COM2_irqx_pat : std_logic_vector(6 downto 0) := "0000101";
    constant COM3_irqx_pat : std_logic_vector(6 downto 0) := "0000110";
    constant COM4_irqx_pat : std_logic_vector(6 downto 0) := "0000111";

    constant COM1_base_addr : std_logic_vector(9 downto 0) := "1111111000";
    constant COM2_base_addr : std_logic_vector(9 downto 0) := "1011111000";
    constant COM3_base_addr : std_logic_vector(9 downto 0) := "1111101000";
    constant COM4_base_addr : std_logic_vector(9 downto 0) := "1011101000";

    constant CLK_PERIOD : time := 50 ns;

    constant RDR_addr : std_logic_vector(2 downto 0) := "000";
    constant IER_addr : std_logic_vector(2 downto 0) := "001";
    constant LCR_addr : std_logic_vector(2 downto 0) := "011";
    constant MCR_addr : std_logic_vector(2 downto 0) := "100";

begin
    --------------------------------------------------------------------------
    -- Подключение DUT
    --------------------------------------------------------------------------
    uut: entity work.uart_isa
        port map (
            isa_addr     => isa_addr,
            isa_data     => isa_data,
            isa_iow      => isa_iow,
            isa_ior      => isa_ior,
            isa_aen      => isa_aen,
            isa_reset    => isa_reset,
            mcu_isa_res  => mcu_isa_res,
            mcu_DTR      => mcu_DTR,
            mcu_rxd      => mcu_rxd,
            mcu_clk      => mcu_clk,
            mcu_res      => mcu_res,
            IRQ3         => IRQ3,
            IRQ4         => IRQ4,
            IRQX         => IRQX
        );

    -- ISA data bus tristate handling
    isa_data <= isa_data_drv when isa_iow = '0' else (others => 'Z');

    --------------------------------------------------------------------------
    -- MCU SPI clock generation
    --------------------------------------------------------------------------
--    clk_proc : process
--    begin
--        mcu_clk <= '0';
--        wait for CLK_PERIOD/2;
--        mcu_clk <= '1';
--        wait for CLK_PERIOD/2;
--    end process;

    --------------------------------------------------------------------------
    -- Main stimulus + assertions
    --------------------------------------------------------------------------

    -- Main test process
    stim_proc: process

    procedure spi_send_byte(databit: in std_logic_vector(6 downto 0)) is
    begin
        for i in 6 downto 0 loop
            mcu_clk <= '0';
            mcu_rxd <= databit(i);
            wait for CLK_PERIOD;
            mcu_clk <= '1';
            wait for CLK_PERIOD;
        end loop;
        mcu_clk <= '0';
        mcu_rxd <= '0';
    end procedure;

    -- ISA write operation
    procedure isa_write(addr : std_logic_vector(9 downto 0); data : std_logic_vector(7 downto 0)) is
    begin
        isa_addr <= addr;
        isa_data_drv <= data;
        isa_iow <= '0';
        wait for CLK_PERIOD;
        isa_iow <= '1';
        isa_data_drv <= (others => 'Z');
        wait for 10 ns;
    end procedure;

    -- ISA read operation
    procedure isa_read(addr : std_logic_vector(9 downto 0); variable data_out : out std_logic_vector(7 downto 0)) is
    begin
        isa_addr <= addr;
        isa_ior <= '0';
        wait for CLK_PERIOD;
        data_out := isa_data;
        isa_ior <= '1';
        wait for 10 ns;
    end procedure;

    procedure isa_res is
    begin
        isa_reset     <= '1';
        wait for CLK_PERIOD;
        isa_reset     <= '0';
    end procedure;

    procedure spi_reset is
    begin
        mcu_res     <= '1';
        wait for CLK_PERIOD;
        mcu_res     <= '0';
    end procedure;

    procedure spi_full_reset is
    begin
        mcu_clk     <= '1';
        wait for CLK_PERIOD;
        mcu_rxd     <= '1';
        wait for CLK_PERIOD;
        mcu_res     <= '1';
        wait for CLK_PERIOD;
        mcu_clk     <= '0';
        wait for CLK_PERIOD;
        mcu_rxd     <= '0';
        wait for CLK_PERIOD;
        mcu_res     <= '0';
    end procedure;

    procedure init_to_comX(options : std_logic_vector(6 downto 0)) is
    begin
        isa_res;
        spi_full_reset;
        spi_send_byte(options);
        wait for CLK_PERIOD;
    end procedure;

    procedure write_reg(
        base_addr : in  std_logic_vector(9 downto 0);
        reg_index : in std_logic_vector(2 downto 0);
        val : in std_logic_vector(7 downto 0)
    ) is
    begin
        isa_write(std_logic_vector(unsigned(base_addr) + unsigned(reg_index)), val);
    end procedure;

    procedure read_reg(
        base_addr : in  std_logic_vector(9 downto 0);
        reg_index : in std_logic_vector(2 downto 0);
        data_out  : out std_logic_vector(7 downto 0)
    ) is
    begin
        isa_read(std_logic_vector(unsigned(base_addr) + unsigned(reg_index)), data_out);
    end procedure;

    procedure writeLCR(
        base_addr : in std_logic_vector(9 downto 0);
        val : in std_logic_vector(7 downto 0)) is
    begin
        write_reg(base_addr, LCR_addr, val);
    end procedure;

    procedure readLCR(
        base_addr : in  std_logic_vector(9 downto 0);
        data_out  : out std_logic_vector(7 downto 0)
    ) is
    begin
        read_reg(base_addr, LCR_addr, data_out);
    end procedure;

    procedure writeMCR(
        base_addr : in std_logic_vector(9 downto 0);
        val : in std_logic_vector(7 downto 0)) is
    begin
        write_reg(base_addr, LCR_addr, val);
    end procedure;

    procedure readMCR(
        base_addr : in  std_logic_vector(9 downto 0);
        data_out  : out std_logic_vector(7 downto 0)
    ) is
    begin
        read_reg(base_addr, LCR_addr, data_out);
    end procedure;

    variable read_val : std_logic_vector(7 downto 0);
    begin
        report "===== TEST START =====";

    ----------------------------------------------------------------------
        report "=== TEST 1: RESET sequence ===";
        isa_reset <= '1';
        wait for CLK_PERIOD;

        assert mcu_isa_res = '0'
            report "FAIL: MCU reset line error"
            severity error;

        isa_reset <= '0';
        wait for CLK_PERIOD;

        assert mcu_isa_res = '1'
            report "FAIL: MCU reset line should be low after ISA reset release"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 2: Access to uninitialized device ===";
        isa_write("0000000001", "00001111"); -- addr=1
        wait for CLK_PERIOD;

        isa_read("0000000001", read_val); -- same address

        assert read_val(7 downto 0) = "ZZZZZZZZ"
            report "FAIL: Interrupt enable register readback mismatch"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 3: MCU SPI config transmission ===";

        init_to_comX(COM1_def_pat);
        read_reg(COM1_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        init_to_comX(COM2_def_pat);
        read_reg(COM2_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;
        wait for CLK_PERIOD;

        init_to_comX(COM3_def_pat);
        read_reg(COM3_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        init_to_comX(COM4_def_pat);
        read_reg(COM4_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 4: MCU SPI config transmission part 2 ===";
        init_to_comX(COM1_irqx_pat);

        read_reg(COM1_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        init_to_comX(COM2_irqx_pat);

        read_reg(COM2_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        init_to_comX(COM3_irqx_pat);

        read_reg(COM3_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        init_to_comX(COM4_irqx_pat);

        read_reg(COM4_base_addr, RDR_addr, read_val);
        -- Проверяем, что данные не все 'Z' (устройство отвечает)
        assert read_val /= "ZZZZZZZZ" 
            report "Device not responding after SPI initialization COM1"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 5: IRQ generation check ===";
        init_to_comX(COM1_def_pat);

        spi_send_byte("1011010");
        wait for CLK_PERIOD;

        assert (IRQ4 = 'Z')
            report "FAIL: Expected IRQ line active after data reception"
            severity error;

        write_reg(COM1_base_addr, LCR_addr, "00000001");
        read_reg(COM1_base_addr, LCR_addr, read_val);
        assert read_val(7 downto 0) = "00000001"
            report "FAIL: Read LCR, expected 00000001, got " & to_string(read_val)
            severity error;

        write_reg(COM1_base_addr, IER_addr, "00000001");
        read_reg(COM1_base_addr, IER_addr, read_val);
        assert read_val(7 downto 0) = "00000001"
            report "FAIL: Read IER, expected 00000001, got " & to_string(read_val)
            severity error;

        assert (IRQ4 = 'Z')
            report "FAIL: Expected IRQ line active after data reception"
            severity error;

        write_reg(COM1_base_addr, MCR_addr, "00001011");
        read_reg(COM1_base_addr, MCR_addr, read_val);
        assert read_val(7 downto 0) = "00001011"
            report "FAIL: Read MCR, expected 00001011, got " & to_string(read_val)
            severity error;

        assert (IRQ4 = 'Z')
            report "FAIL: Expected IRQ line active after data reception"
            severity error;

        read_reg(COM1_base_addr, RDR_addr, read_val);

        assert read_val(7 downto 0) = "01011010"
            report "FAIL: Read received data, expected 01011010, got " & to_string(read_val)
            severity error;

        assert (IRQ4 = '0')
            report "FAIL: Expected IRQ line still active after read data"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 6: RESET clears IRQ ===";
        isa_reset <= '1';
        wait for CLK_PERIOD;
        isa_reset <= '0';

        assert (IRQ4 = 'Z')
            report "FAIL: Expected IRQ line active after data reception"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 7: Check genegic registers";

        write_reg(COM1_base_addr, LCR_addr, "01111111");
        read_reg(COM1_base_addr, LCR_addr, read_val);
        assert read_val(7 downto 0) = "01111111"
            report "FAIL: Read LCR, expected 0111111, got " & to_string(read_val)
            severity error;

        write_reg(COM1_base_addr, RDR_addr, "10101101");
        read_reg(COM1_base_addr, LCR_addr, read_val);
        assert read_val(7 downto 0) /= "10101101"
            report "FAIL: Read reg 0, expected any, except 10101101, got " & to_string(read_val)
            severity error;

        isa_write("1111111001", "00010010"); -- gen 1
        wait for CLK_PERIOD;

        isa_read("1111111001", read_val); -- gen 1
        wait for CLK_PERIOD;

        assert read_val(7 downto 0) /= "00010010"
            report "FAIL: Read reg 1, expected any, except 00010010, got " & to_string(read_val)
            severity error;

        isa_write("1111111011", "11111111"); -- LCR
        wait for CLK_PERIOD;

        isa_read("1111111011", read_val); -- LCR
        wait for CLK_PERIOD;

        assert read_val(7 downto 0) = "11111111"
            report "FAIL: Read LCR, expected 0111111, got " & to_string(read_val)
            severity error;

        isa_write(COM1_base_addr, "00101101"); -- gen 0
        wait for CLK_PERIOD;

        isa_read(COM1_base_addr, read_val); -- gen 0
        wait for CLK_PERIOD;

        assert read_val(7 downto 0) = "00101101"
            report "FAIL: Read reg 0, expected 00101101, got " & to_string(read_val)
            severity error;

--        isa_write("1111111001", "00010010"); -- gen 1
--        wait for CLK_PERIOD;

--        isa_read("1111111001", read_val); -- gen 1
--        wait for CLK_PERIOD;

--        assert read_val(7 downto 0) = "00010010"
--            report "FAIL: Read reg 1, expected 00010010, got " & to_string(read_val)
--            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 8: Send and read data";

        write_reg(COM1_base_addr, LCR_addr, "01111111");
        read_reg(COM1_base_addr, LCR_addr, read_val);
        assert read_val(7 downto 0) = "01111111"
            report "FAIL: Read LCR, expected 0111111, got " & to_string(read_val)
            severity error;


        write_reg(COM1_base_addr, IER_addr, "00000001");
        read_reg(COM1_base_addr, IER_addr, read_val);
        assert read_val(7 downto 0) = "00000001"
            report "FAIL: Read interrupt enable register, expected 00000001, got " & to_string(read_val)
            severity error;

        write_reg(COM1_base_addr, MCR_addr, "00001011");
        read_reg(COM1_base_addr, MCR_addr, read_val);
        assert read_val(7 downto 0) = "00001011"
            report "FAIL: Read MCR, expected 00001011, got " & to_string(read_val)
            severity error;

        read_reg(COM1_base_addr, RDR_addr, read_val);

        spi_reset;
        spi_send_byte("0100101");
        wait for CLK_PERIOD;

        assert (IRQ4 = 'Z')
            report "FAIL: Expected IRQ line active after data reception, got " & to_string(IRQ4)
            severity error;

        read_reg(COM1_base_addr, RDR_addr, read_val);
        assert read_val(7 downto 0) = "00100101"
            report "FAIL: Read received data, expected 00100101, got " & to_string(read_val)
            severity error;


        assert (IRQ4 = '0')
            report "FAIL: Expected IRQ line not active after data read, got " & to_string(IRQ4)
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 9: Stability test with multiple cycles";
        for i in 0 to 50 loop
            spi_send_byte(std_logic_vector(to_unsigned((i*5 + 3) mod 128, 7)));
            read_reg(COM1_base_addr, RDR_addr, read_val);
        end loop;

        -- Проверяем, что устройство все еще отвечает
        read_reg(COM1_base_addr, RDR_addr, read_val);
        assert read_val /= "ZZZZZZZZ" 
            report "Device stopped responding during stability test"
            severity error;

        ----------------------------------------------------------------------
        report "=== TEST 10: Stability test with transmit/receive cycles";
        spi_reset;
        for i in 0 to 500 loop
            tmp <= std_logic_vector(to_unsigned((i*5 + 3) mod 128, 7));
            wait for CLK_PERIOD;
            spi_send_byte(tmp);
            wait for CLK_PERIOD;

            assert (IRQ4 = 'Z')
                report "FAIL: Expected IRQ line active after data reception, got " & to_string(IRQ4)
                severity error;

            read_reg(COM1_base_addr, RDR_addr, read_val);
            assert read_val(7 downto 0) = "0" & tmp
                report "FAIL: Read received data, expected " & to_string("0" & tmp) & ", got " & to_string(read_val)
                severity error;


            assert (IRQ4 = '0')
                report "FAIL: Expected IRQ line not active after data read, got " & to_string(IRQ4)
                severity error;
        end loop;

        ----------------------------------------------------------------------
        report "=== TEST 11: BIOS detect test";
        write_reg(COM1_base_addr, "010", "00111000"); -- FCR reg
        read_reg(COM1_base_addr, "010", read_val);
        assert read_val(7 downto 0) /= "11111111"
            report "FAIL: got " & to_string(read_val)
            severity error;

        read_reg(COM1_base_addr, LCR_addr, read_val);

        write_reg(COM1_base_addr, LCR_addr, "1" & read_val(6 downto 0));
        write_reg(COM1_base_addr, RDR_addr, "01010101"); -- FCR reg

        write_reg(COM1_base_addr, IER_addr, "00000000");

        read_reg(COM1_base_addr, RDR_addr, read_val);
        assert read_val(7 downto 0) = "01010101"
            report "FAIL: got " & to_string(read_val)
            severity error;

        report "=== ALL TESTS EXECUTED SUCCESSFULLY ===";
        wait;
    end process;

end behavior;
