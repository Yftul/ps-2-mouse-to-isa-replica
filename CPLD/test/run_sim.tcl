transcript on
echo "=== Starting ModelSim Simulation for uart_isa ==="

# Очистка и создание библиотеки
if { [file exists work] } {
    vdel -all
}
vlib work
vmap work work

# Компиляция исходников
echo "Compiling VHDL sources..."
vcom -2008 uart_isa.vhd
vcom -2008 uart_isa_tb.vhd

# Запуск симуляции
echo "Starting simulation..."
vsim work.uart_isa_tb -voptargs="+acc"

# Добавляем все сигналы в waveform
add wave -r /*

# Настройка отображения
configure wave -namecolwidth 200
configure wave -valuecolwidth 100
configure wave -timelineunits ns
configure wave -signalnamewidth 1

# Запуск симуляции
echo "Running simulation for 500 us..."
run 500 us

# Стоп и удержание результатов
echo "Simulation completed. You can inspect waveforms."
