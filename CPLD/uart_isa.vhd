library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity uart_isa is
    Port (
        -- ISA Bus Interface
        isa_addr     : in  std_logic_vector(9 downto 0);
        isa_data     : inout std_logic_vector(7 downto 0);
        isa_iow      : in  std_logic;
        isa_ior      : in  std_logic;
        isa_aen      : in  std_logic;
        isa_reset    : in  std_logic;
        
        -- MCU Interface
        mcu_isa_res  : out std_logic := '0'; -- сброс MCU
        mcu_txd      : out std_logic := '0'; -- Аналог SPI, данные в регистр передачи в MCU
        mcu_rxd      : in  std_logic;        -- Аналог SPI, данные в регистр приема от MCU
        mcu_clk      : in  std_logic;        -- Аналог SPI, такты от MCU
        mcu_res      : in  std_logic;        -- Аналог SPI, синхронизация от MCU

        -- IRQ out
        IRQ3         : out  std_logic := 'Z';
        IRQ4         : out  std_logic := 'Z';
        IRQX         : out  std_logic := 'Z'
	  );
end uart_isa;

architecture Behavioral of uart_isa is
    -- Регистры UART
    signal rx_data_reg    : std_logic_vector(7 downto 0) := (others => '0');  -- 0x3F8h IN
    signal mdm_ctl_reg    : std_logic_vector(7 downto 0) := (others => '0');  -- 0x3FEh IN
    signal line_ctl_reg   : std_logic_vector(7 downto 0) := (others => '0');  -- 0x3FBh OUT

    -- Регистры накопления данных UART
    signal rx_acc_reg     : std_logic_vector(7 downto 0) := (others => '0');	-- Аккумулятор бит от MCU
	 signal bit_counter	  : unsigned(2 downto 0) := (others => '0');			   -- Счётчик бит от MCU

    -- Сигналы управления IRQ
    signal SET_RxD_IRQ    : std_logic := '0'; -- сигнал запроса установки RxD_IRQ
    signal RES_RxD_IRQ    : std_logic := '0'; -- сигнал запроса сброса RxD_IRQ
    signal Enable_IRQ     : std_logic := '0'; -- разрешение использования IRQ
    signal RxD_IE         : std_logic := '0'; -- разрешение прерывания приема данных
    signal RxD_IRQ        : std_logic := '0'; -- прерывание приема данных

    -- Сигналы адресного декодирования
    signal device_select  : std_logic;
    signal data_out       : std_logic_vector(7 downto 0) := (others => '0'); -- Буфер вывода данных ISA
    signal base_irq_val   : std_logic_vector(1 downto 0) := "00";            -- IRQ по умолчанию после включения
    signal base_addr_val  : std_logic_vector(1 downto 0) := "00";            -- Компорт по умолчанию после включения
    signal base_addr_rdy  : std_logic := '1'; -- Устройство не готово к обмену по ISA сразу после включения;

	 -- Сигналы адресного компаратора
	 type base_addr_array_t is array (0 to 3) of std_logic_vector(6 downto 0);
	 constant BASE_ADDR_ROM : base_addr_array_t := (
		0 => "1111111", -- COM1 = 0x3F8 >> 3 = 0x7F
		1 => "1111101", -- COM2 = 0x3E8 >> 3 = 0x7D
		2 => "1011111", -- COM3 = 0x2F8 >> 3 = 0x5F
		3 => "1011101"  -- COM4 = 0x2E8 >> 3 = 0x5D
);
begin
	process(isa_reset, RES_RxD_IRQ, SET_RxD_IRQ) begin -- RS триггер состояния прерывания приёма данных
		if isa_reset = '1' or RES_RxD_IRQ = '1' then
			RxD_IRQ <= '0';
		elsif SET_RxD_IRQ = '1' then
			RxD_IRQ <= '1';
		end if;
	end process;

	process(isa_ior, isa_reset) -- Асинхронное чтение регистров UART c ISA шины
	begin
		if isa_reset = '0' then
			if falling_edge(isa_ior) then -- Чтение из регистров UART
				if (device_select = '1') then
					case isa_addr(2 downto 0) is
						 when "000" => -- Регистр данных
							if line_ctl_reg(7) = '0' then
								data_out <= rx_data_reg; -- Прочитали данные UART
							end if;
						 when "001" => -- Регистр разрешения прерывания
							if line_ctl_reg(7) = '0' then -- DLAB check
								data_out <= "0000000" & not RxD_IRQ;
							end if;
						 when "010" => -- причина прерывания: xxxxx10x = принят символ; сбрасывается чтением приемника
								data_out <= "00000" & RxD_IRQ & "00";
						 when "011" => data_out <= line_ctl_reg;
						 when "100" => data_out <= std_logic_vector(bit_counter) & mdm_ctl_reg(4 downto 0);
						 when others => data_out <= (others => '0');
					end case;
				else
					data_out <= (others => '0');
				end if;
			end if;
		end if;
	end process;

	process(isa_iow, isa_reset) begin -- Асинхронная запись в регистры UART
		if isa_reset = '1' then -- Сброс ISA шины
			RxD_IE <= '0'; -- Запрет прерывания приема данных
		else
			if rising_edge(isa_iow) then -- Запись в регистры UART
				if (device_select = '1') then
					case isa_addr(2 downto 0) is
						when "001" =>
							if line_ctl_reg(7) = '0' then -- DLAB check
								RxD_IE <= isa_data(0); -- Регистр разрешения прерываний
							end if;
						when "011" => line_ctl_reg <= isa_data;
						when "100" => mdm_ctl_reg <= isa_data;
						when others => null;
					end case;
				end if;
			end if;
		end if;
	end process;

   process(mcu_clk, mcu_res, bit_counter) begin -- Работа с MCU
		if mcu_res = '1' then 		-- Сброс от MCU
			bit_counter <= "000"; 	-- Сбрасываем счётчик
			base_addr_rdy 	<= '0';	-- Сбрасываем готовность базового адреса устройства
		else
			if rising_edge(mcu_clk) then -- Записываем данные по фронту сигнала
				rx_acc_reg(to_integer(bit_counter)) <= mcu_rxd;
				bit_counter <= bit_counter + 1;
			end if;
			if falling_edge(mcu_clk) and (bit_counter = "000") then -- Верифицируем по спаду
				if (base_addr_rdy = '0') then -- Первый байт данных от MCU - приходит базовый адрес устройства и номер IRQ
					base_addr_val(1 downto 0) <= rx_acc_reg(1 downto 0); -- Базовый адрес устройства
					base_irq_val(1 downto 0) <= rx_acc_reg(3 downto 2); -- IRQ устройства
					base_addr_rdy <= '1'; -- Готовность устройства(если не установлено по умолчанию)
				else  -- Последующие данные от мыши
					rx_data_reg <= rx_acc_reg;
				end if;
			end if;
		end if;
	end process;

	process(isa_reset, mcu_res, mcu_clk) -- Процесс формирования сигнала установки прерывания
	begin
		if isa_reset = '1' or mcu_res = '1' then -- Сброс от ISA или MCU
			SET_RxD_IRQ <= '0';	-- Сбрасываем признак прерывания
		elsif falling_edge(mcu_clk) then -- Верифицируем по спаду
			if (bit_counter = "000") and (base_addr_rdy = '1') then -- Если байт принят и есть готовность настроек
				SET_RxD_IRQ <= '1'; -- Устанавливаем признак прерывания
			end if;
		end if;
	end process;

	process(isa_ior, isa_reset) -- Процесс формирования сигнала сброса прерывания
	begin
		 if isa_reset = '1' then
			  RES_RxD_IRQ <= '0';
		 elsif falling_edge(isa_ior) then
			  if (device_select = '1' and isa_addr(2 downto 0) = "000") then -- при чтении принятых данных cбрасываем прерывание  
					RES_RxD_IRQ <= '1'; -- Устанавливаем признак сброса прерывания
			  else
					RES_RxD_IRQ <= '0';
			  end if;
		 end if;
	end process;

	-- Комбинаторная логика
	mcu_isa_res	<= not isa_reset; -- Передача сигнала сброса ISA шины на MCU

	mcu_txd <= mdm_ctl_reg(to_integer(bit_counter)); -- Последовательный сигнал передачи данных в MCU

	device_select <= '1' when 	isa_aen = '1' and base_addr_rdy = '1' and 
										isa_addr(9 downto 3) = BASE_ADDR_ROM(to_integer(unsigned(base_addr_val))) else '0';

	isa_data				<= data_out when isa_reset = '0' and device_select = '1' and
												  isa_ior = '0' and isa_iow = '1' else (others => 'Z');

	Enable_IRQ			<= RxD_IE or mdm_ctl_reg(3); -- OUT2 также разрешает прерывания
	IRQ4					<= RxD_IRQ when base_irq_val = "01" and Enable_IRQ = '1' else 'Z'; -- COM1/COM3
	IRQ3					<= RxD_IRQ when base_irq_val = "10" and Enable_IRQ = '1' else 'Z'; -- COM2/COM4
	IRQX					<= RxD_IRQ when base_irq_val = "11" and Enable_IRQ = '1' else 'Z'; -- Custom

end Behavioral;