-- ============================================================================
--  Project : PS/2-to-ISA Mouse Adapter — Replica
--  File    : uart_isa.vhd
--  Author  : Pyshchev Alexander aka Yftul
--  License : GPL-3.0
--
--  Description :
--      Implements ISA bus interface logic for adapting a PS/2 mouse
--      to legacy PC hardware. Includes address decoding, control signal
--      generation, and data flow between the MCU and ISA connector.
--
--  Notes :
--      Designed for retro hardware restoration and compatibility testing.
--      Part of a mixed hardware/firmware project including CPLD logic
--      and MCU-based PS/2 protocol handling.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity uart_isa is
    Port (
        -- ISA Bus Interface
        isa_addr     : in  std_logic_vector(9 downto 0);   -- pins 15, 14, 13, 12, 10, 8, 6, 5, 3, 2
        isa_data     : inout std_logic_vector(7 downto 0); -- pins 31, 28, 27, 25, 23, 22, 21, 20
        isa_iow      : in  std_logic; -- pin 37
        isa_ior      : in  std_logic; -- pin 18
        isa_aen      : in  std_logic; -- pin 19
        isa_reset    : in  std_logic; -- pin 33

        -- MCU Interface
        mcu_isa_res  : out std_logic; -- pin 34, сброс MCU
        mcu_DTR      : out std_logic; -- pin 35, Признак включения мыши для MCU
        mcu_rxd      : in  std_logic; -- pin 38, SPI, данные в регистр приема от MCU
        mcu_clk      : in  std_logic; -- pin 40, SPI, такты от MCU
        mcu_res      : in  std_logic; -- pin 39, SPI, синхронизация от MCU

        -- IRQ out
        IRQ3         : out  std_logic := 'Z'; -- pin 44
        IRQ4         : out  std_logic := 'Z'; -- pin 42
        IRQX         : out  std_logic := 'Z'  -- pin 43
      );
end uart_isa;

architecture Behavioral of uart_isa is
    -- Регистры UART
    signal rx_data_reg    : std_logic_vector(6 downto 0) := (others => '0');  -- Received data register
    signal int_ena_reg    : std_logic_vector(3 downto 0) := (others => '0');  -- Int enable register
    signal mdm_ctl_reg    : std_logic_vector(4 downto 0) := (others => '0');  -- Modem control register
    signal gen_reg        : std_logic_vector(7 downto 0) := (others => '0');  -- Fake register

    -- Регистры накопления данных UART
    signal rx_acc_reg     : std_logic_vector(6 downto 0) := (others => '0');    -- Аккумулятор бит от MCU
    signal bit_counter    : std_logic_vector(2 downto 0) := (others => '0');    -- Счётчик бит от MCU

    -- Сигналы управления IRQ
    signal SET_RxD_IRQ    : std_logic := '0'; -- сигнал запроса установки RxD_IRQ
    signal RES_RxD_IRQ    : std_logic := '0'; -- сигнал запроса сброса RxD_IRQ
    signal IRQ_state      : std_logic := '0'; -- текущее состояние IRQ
    signal RxD_IRQ        : std_logic := '0'; -- прерывание приема данных
    signal enable_IRQ     : std_logic := '0';

    -- Сигналы адресного декодирования
    signal device_select  : std_logic;
    signal data_out       : std_logic_vector(7 downto 0) := (others => '0'); -- Буфер вывода данных ISA
    signal base_addr_val  : std_logic_vector(1 downto 0) := "00";            -- Компорт по умолчанию после включения
    signal use_opt_irq    : std_logic := '0';                                -- IRQ по умолчанию после включения

    -- Разное
    signal device_rdy     : std_logic := '0'; -- Устройство не готово к обмену по ISA сразу после включения;
    signal sig_DLAB       : std_logic := '0';

    alias enable_mouse is mdm_ctl_reg(1); -- (RTS)

     -- Сигналы адресного компаратора
    type base_addr_array_t is array (0 to 3) of std_logic_vector(6 downto 0);
    constant BASE_ADDR_ROM : base_addr_array_t := (
        0 => "1111111", -- COM1 = 0x3F8 >> 3 = 0x7F
        1 => "1011111", -- COM2 = 0x2F8 >> 3 = 0x5F
        2 => "1111101", -- COM3 = 0x3E8 >> 3 = 0x7D
        3 => "1011101");-- COM4 = 0x2E8 >> 3 = 0x5D

begin
    process(isa_ior, isa_reset, sig_DLAB, RxD_IRQ) -- Асинхронное чтение регистров UART c ISA шины
    begin
        if isa_reset = '0' then
              if falling_edge(isa_ior) then -- Чтение из регистров UART
                if (device_select = '1') then
                    data_out <= (others => '0');
                    case isa_addr(2 downto 0) is
                        when "000" => -- Регистр данных
                            if sig_DLAB = '0' then -- !DLAB check
                                data_out <= "0" & rx_data_reg;  -- Прочитали данные UART
                            else
                                data_out <= gen_reg;
                            end if;
                        when "001" => -- Регистр разрешения прерывания
                            if sig_DLAB = '0' then -- !DLAB check
                                data_out <= "0000" & int_ena_reg;
                            else
                                data_out <= gen_reg;
                            end if;
                        when "010" => -- причина прерывания: xxxxx10x = принят символ; сбрасывается чтением приемника
                            if RxD_IRQ = '1' then             -- Прерывание готовности принятого символа
                                data_out <= "00000100";       -- Сигнализация готовности принятого символа
                            else
                                if int_ena_reg(1) = '1' then -- Прерывание готовности передачи символа
                                    data_out <= "00000010";   -- Сигнализация готовности передачи символа
                                else
                                    data_out <= "00000001";   -- Нет прерываний для обработки
                                end if;
                            end if;
                        when "011" =>                         -- Line control register
                                data_out <= sig_DLAB & gen_reg(6 downto 0);
                        when "100" =>                         -- Modem control register
                            data_out <= "000" & mdm_ctl_reg;
                        when "101" =>                         -- Line status register
                            data_out <= "0110000" & RxD_IRQ; 
                        when "110" =>                         -- Modem status register
                            data_out <= "00" & mdm_ctl_reg(0) & mdm_ctl_reg(1) & "0000"; -- CTS = RTS , DSR = DTR
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process;

    process(isa_iow, isa_reset, sig_DLAB) begin -- Асинхронная запись в регистры UART
        if isa_reset = '1' then -- Сброс ISA шины
            int_ena_reg <= (others => '0'); -- Запрет прерываний
            mdm_ctl_reg <= (others => '0');
            sig_DLAB <= '0';
        else
            if rising_edge(isa_iow) then -- Запись в регистры UART
                if (device_select = '1') then
                    case isa_addr(2 downto 0) is
                        when "000" => -- Регистр разрешения прерываний
                            if sig_DLAB = '1' then -- DLAB check
                                gen_reg <= isa_data;
                            end if;
                        when "001" => -- Регистр разрешения прерываний
                            if sig_DLAB = '0' then -- !DLAB check
                                int_ena_reg <= isa_data(3 downto 0);
                            else
--                                gen_reg <= isa_data; -- конфликтует с определением порта в биос
                            end if;
                        when "011" =>
                            gen_reg(6 downto 0) <= isa_data(6 downto 0);
                            sig_DLAB <= isa_data(7);
                        when "100" =>
                            mdm_ctl_reg <= isa_data(4 downto 0);
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process;

    process(mcu_clk, mcu_res, device_rdy) begin
        if mcu_res = '1' then
            bit_counter <= "000";
        else
            if rising_edge(mcu_clk) then -- Записываем данные по фронту сигнала
                case bit_counter is
                    when "000" => rx_acc_reg(6) <= mcu_rxd;
                                  bit_counter <= "001";
                    when "001" => rx_acc_reg(5) <= mcu_rxd;
                                  bit_counter <= "010";
                    when "010" => rx_acc_reg(4) <= mcu_rxd;
                                  bit_counter <= "011";
                    when "011" => rx_acc_reg(3) <= mcu_rxd;
                                  bit_counter <= "100";
                    when "100" =>
                            if (device_rdy = '1') then
                                rx_acc_reg(2) <= mcu_rxd;
                            else 
                                use_opt_irq <= mcu_rxd;
                            end if;
                            bit_counter <= "101";
                    when "101" => -- rx_acc_reg(1) <= mcu_rxd;
                            if (device_rdy = '1') then
                                rx_acc_reg(1) <= mcu_rxd;
                            else 
                                base_addr_val(1) <= mcu_rxd;
                            end if;
                            bit_counter <= "110";
                    when "110" => -- rx_acc_reg(0) <= mcu_rxd;
                            if (device_rdy = '1') then
                                rx_acc_reg(0) <= mcu_rxd;
                            else 
                                base_addr_val(0) <= mcu_rxd;
                            end if;
                            bit_counter <= "000";
                    when others =>
                            bit_counter <= "000";
                    end case;
            end if;
        end if;
    end process;

    -- Работа с MCU
    process(isa_reset, mcu_clk, mcu_res, mcu_rxd, bit_counter) begin
        if mcu_res = '1' then
            if (mcu_rxd = '1') then -- Последовательность для установки готовности
                device_rdy <= '0';  -- Сбрасываем готовность устройства
            end if;
        else
            if falling_edge(mcu_clk) then -- Верифицируем по спаду
                if (bit_counter = "000") then
                    if (device_rdy = '0') then -- Первый байт данных от MCU - приходит базовый адрес устройства и номер IRQ
                        device_rdy <= '1'; -- Готовность устройства(если не установлено по умолчанию)
                    else  -- Последующие данные от мыши
                        rx_data_reg <= rx_acc_reg;
                    end if;
                end if;
            end if;
        end if;
    end process;

    process(isa_reset, mcu_res, mcu_clk, device_rdy, bit_counter, RxD_IRQ) -- Процесс формирования сигнала установки прерывания
    begin
        if isa_reset = '1' or mcu_res = '1' or RxD_IRQ = '1' then
            SET_RxD_IRQ <= '0';    -- Сбрасываем признак прерывания
        elsif falling_edge(mcu_clk) then -- Верифицируем по спаду
            if (bit_counter = "000") and (device_rdy = '1') then -- Если байт принят и есть готовность данных
                SET_RxD_IRQ <= '1'; -- Устанавливаем признак прерывания
            end if;
        end if;
    end process;

    process(isa_ior, isa_reset, RxD_IRQ) -- Процесс формирования сигнала сброса прерывания
    begin
        if isa_reset = '1' then
            RES_RxD_IRQ <= '0';
        elsif RxD_IRQ = '0' then
            RES_RxD_IRQ <= '0'; -- Сбрасываем признак прерывания
        elsif falling_edge(isa_ior) then
            if (device_select = '1' and isa_addr(2 downto 0) = "000") then -- при чтении принятых данных cбрасываем прерывание  
                RES_RxD_IRQ <= '1'; -- Устанавливаем признак сброса прерывания
            else
                RES_RxD_IRQ <= '0';
            end if;
        end if;
    end process;

    process(isa_reset, SET_RxD_IRQ, mcu_res, RES_RxD_IRQ) begin -- RS триггер состояния прерывания приёма данных
        if isa_reset = '1' or mcu_res = '1' or RES_RxD_IRQ = '1' then
            RxD_IRQ <= '0';
        elsif SET_RxD_IRQ = '1' then
            RxD_IRQ <= '1';
        end if;
    end process;

    -- Комбинаторная логика
    mcu_isa_res <= not isa_reset; -- Передача сигнала сброса ISA шины на MCU
    mcu_DTR <= enable_mouse;

    device_select <= '1' when (isa_aen = '0') and (device_rdy = '1') and 
                            (isa_addr(9 downto 3) = BASE_ADDR_ROM(to_integer(unsigned(base_addr_val)))) else '0';

    isa_data <= data_out when (isa_reset = '0') and (device_select = '1') and (isa_ior = '0') else (others => 'Z');

    enable_IRQ <= enable_mouse and mdm_ctl_reg(3);               -- OUT2 разрешает прерывания
--    IRQ_state <= (RxD_IRQ and int_ena_reg(0)) or int_ena_reg(1); -- TxD_IRQ всегда выставлен
    IRQ_state <= (RxD_IRQ and int_ena_reg(0));                   -- Игнорируем TxD_IRQ

     -- OUT2 разрешает прерывания
    IRQ4 <= '0' when (IRQ_state = '0') and (enable_IRQ = '1') and (base_addr_val(0) = '0') and
                                                (use_opt_irq = '0') and (device_rdy = '1') else 'Z'; -- COM1/COM3
    IRQ3 <= '0' when (IRQ_state = '0') and (enable_IRQ = '1') and (base_addr_val(0) = '1') and
                                                (use_opt_irq = '0') and (device_rdy = '1') else 'Z'; -- COM2/COM4
    IRQX <= '0' when (IRQ_state = '0') and (enable_IRQ = '1') and (use_opt_irq = '1') and
                                                                        (device_rdy = '1') else 'Z'; -- Custom

end Behavioral;
