#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#define DEBUG 0
#define F_CPU 8000000UL

#include <util/delay.h>
#include "stddef.h"

#define EEPROM_OFFSET_MULTIPLIER    0

#define PS2_BUF_SIZE 256       // Размер приёмного буфера PS/2 порта
#define SPI_TX_BUFFER_SIZE 16 // Размер буфера передачи SPI

#define TIMER0_CONST 0x9F // Регулирует скорость передачи данных мыши

//===========================================================================
// pin definitions
//===========================================================================
//#define SPI_MISO_PORT    D            // TODO: переделать на выход MCU_reset
//#define SPI_MISO_PIN    0            // PD0 - нога 30
#define SPI_RESOUT_PORT   D
#define SPI_RESOUT_PIN    0            // PD0 - нога 30
#define SPI_MOSI_PORT     D
#define SPI_MOSI_PIN      1            // PD1 - нога 31
#define DTR_PORT          D
#define DTR_PIN           2            // PD2 - нога 32
#define SPI_SCK_PORT      C
#define SPI_SCK_PIN       5            // PC5 - нога 28

#define LED_PORT          C
#define LED_PIN           1            // PC1 - нога 24

#define PS2_CLK_PORT      D            // PD3 - нога 1
#define PS2_CLK_PIN       3
#define PS2_DATA_PORT     B            // PB7 - нога 8
#define PS2_DATA_PIN      7

#define COM_SEL1_PORT     C
#define COM_SEL1_PIN      0            // PC0 - нога 23
#define COM_SEL2_PORT     B
#define COM_SEL2_PIN      1            // PB1 - нога 13

#define WHEEL_SEL_PORT    B
#define WHEEL_SEL_PIN     2            // PB2 - нога 14

#define DUTY_SEL1_PORT    B
#define DUTY_SEL1_PIN     0            // PB0 - нога 12
#define DUTY_SEL2_PORT    D
#define DUTY_SEL2_PIN     7            // PD7 - нога 11

#define RATE_SEL1_PORT    D
#define RATE_SEL1_PIN     6            // PD6 - нога 10
#define RATE_SEL2_PORT    D
#define RATE_SEL2_PIN     5            // PD5 - нога 9

//===========================================================================
// ADC каналы
//===========================================================================
#define ADC_PORT  C
#define IRQ3_PIN  2            // PC2 - нога 25 IRQ3
#define IRQX_PIN  3            // PC3 - нога 26 IRQX
#define IRQ4_PIN  4            // PC4 - нога 27 IRQ4

//===========================================================================
// IRQ
//===========================================================================
#define IRQX_threshold 512
#define get_IRQ4_state() (PIN(ADC_PORT) & _BV(IRQ4_PIN))
#define get_IRQX_state() (PIN(ADC_PORT) & _BV(IRQX_PIN))
#define get_IRQ3_state() (PIN(ADC_PORT) & _BV(IRQ3_PIN))

//===========================================================================
// Светодиод
//===========================================================================
#define led_enable()        PORT(LED_PORT) &= ~_BV(LED_PIN);
#define led_disable()       PORT(LED_PORT) |= _BV(LED_PIN);

//===========================================================================
// Soft SPI
//===========================================================================
#define spi_mosi_high()    PORT(SPI_MOSI_PORT) |= _BV(SPI_MOSI_PIN)
#define spi_mosi_low()     PORT(SPI_MOSI_PORT) &= ~_BV(SPI_MOSI_PIN)
#define spi_sck_high()     PORT(SPI_SCK_PORT) |= _BV(SPI_SCK_PIN)
#define spi_sck_low()      PORT(SPI_SCK_PORT) &= ~_BV(SPI_SCK_PIN)
#define spi_reset_high()   PORT(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN)
#define spi_reset_low()    PORT(SPI_RESOUT_PORT) &= ~_BV(SPI_RESOUT_PIN)

#define spi_timer_stop()   {TCCR1B &= ~(1 << CS12 | 1 << CS11 | 1 << CS10); TCNT1 = 0;}
#define spi_timer_fast()   {TCCR1B &= ~(1 << CS12 | 1 << CS11 | 1 << CS10); TCNT1 = 0; TCCR1B |= (1 << CS10);}
#define spi_timer_slow()   {TCCR1B &= ~(1 << CS12 | 1 << CS11 | 1 << CS10); TCNT1 = 0; TCCR1B |= (1 << CS12) | (1 << CS10);}

//===========================================================================
// PS/2
//===========================================================================
#define ps2_data()              (PIN(PS2_DATA_PORT) & _BV(PS2_DATA_PIN))
#define get_mouse_power_state() (PIN(DTR_PORT) & _BV(DTR_PIN))

//===========================================================================
// Select UART base address
//===========================================================================
#define SELECT_COM1 0
#define SELECT_COM2 1
#define SELECT_COM3 2
#define SELECT_COM4 3

//===========================================================================
// Select UART IRQ
//===========================================================================
#define USE_IRQ4 1
#define USE_IRQ3 2
#define USE_IRQX 3

//===========================================================================
// Select mouse Duty cycles
//===========================================================================
#define DUTY_100 0
#define DUTY_75  1
#define DUTY_50  2

//===========================================================================
// Select data rate
//===========================================================================
// Скорость мыши
#define PS2_SAMPLES_PER_SEC_FAST    180    // 10..200 
#define PS2_SAMPLES_PER_SEC_MID     90     // 10..200
#define PS2_SAMPLES_PER_SEC_SLOW    40     // 10..200

//===========================================================================
// Глобальные переменные
//===========================================================================
enum ps_state_t { 
    ps2_state_error, 
    ps2_state_read, 
    ps2_state_write 
};

volatile uint8_t ps2_state;                // состояние порта (ps_state_t)
volatile uint8_t ps2_bitcount;             // счётчик битов обработчика
volatile uint8_t ps2_data;                 // буфер на байт
volatile uint8_t ps2_parity;
volatile uint8_t ps2_rx_buf[PS2_BUF_SIZE]; // Приёмный буфер PS/2 порта
volatile uint8_t ps2_rx_buf_w;
volatile uint8_t ps2_rx_buf_r;
volatile uint8_t ps2_rx_buf_count;

volatile bool send_PS2_data_flag;

volatile uint8_t opt_com_settings;
volatile uint8_t opt_wheel_enabled;
volatile uint8_t opt_duty_settings;
volatile uint8_t opt_rate_settings;
volatile uint8_t opt_irq_settings;

volatile uint8_t ps2m_wheel; // Используемый протокол: 0=без колеса, 1=с колесом
volatile uint8_t ps2m_multiplier; // Масштабирование координат
volatile uint8_t ps2m_b; // Нажатые кнопки
volatile int16_t ps2m_x; // Координаты мыши
volatile int16_t ps2m_y;
volatile int16_t ps2m_z;

uint8_t spi_tx_buf[SPI_TX_BUFFER_SIZE];
volatile uint8_t spi_tx_buf_w;
volatile uint8_t spi_tx_buf_r;
volatile uint8_t spi_tx_buf_count;
volatile uint8_t spi_state_machine;
volatile uint8_t spi_busy;

volatile bool mouse_reset; // Необходима инициализация мыши
volatile bool mouse_enabled;
volatile bool device_init; // Произведена инициализация адресов/IRQ устройства

#define PROTOCOL_MICROSOFT      0
#define PROTOCOL_EM84520        1

uint8_t mouse_protocol = PROTOCOL_MICROSOFT; // Используемый протокол: 0=MSMouse, 1=EM84520

const uint8_t EM84520_ID[61] = {
    0x4D, 0x5A, 0x40, 0x00, 0x00, 0x00, 0x08, 0x01, 0x24, 0x25, 0x2D, 0x23,
    0x10, 0x10, 0x10, 0x11, 0x3C, 0x3C, 0x2D, 0x2F, 0x35, 0x33, 0x25, 0x3C,
    0x30, 0x2E, 0x30, 0x10, 0x26, 0x10, 0x21, 0x3C, 0x25, 0x2D, 0x23, 0x00,
    0x33, 0x23, 0x32, 0x2F, 0x2C, 0x2C, 0x29, 0x2E, 0x27, 0x00, 0x33, 0x25,
    0x32, 0x29, 0x21, 0x2C, 0x00, 0x2D, 0x2F, 0x35, 0x33, 0x25, 0x21, 0x15,
    0x09
};

//---------------------------------------------------------------------------
// Чтение джамперов
uint8_t readCOMsettings(void) {
    bool t = false;
    if (((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN)) == 0) &&
        ((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN)) == 1))
            return SELECT_COM2;
    if (((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN)) == 1) &&
        ((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN)) == 0))
            return SELECT_COM4;

    // Перемычки на землю не обнаружены, проверяем перемычу между джамперами
    DDR(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);    // COM_SEL1 теперь выход
    PORT(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);  // Запишем туда 0
    t = ((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN)) == 0);
    DDR(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);   // COM_SEL1 теперь вход
    PORT(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);   // Вернём подтяжку
    if (t)
        return SELECT_COM3;
    else
        return SELECT_COM1;
}

bool readWheelsettings(void) {
    if ((PIN(WHEEL_SEL_PORT) & _BV(WHEEL_SEL_PIN)) == 0)
        return true;
    return false;
}

uint8_t readDutysettings(void) {
    if (((PIN(DUTY_SEL1_PORT) & _BV(DUTY_SEL1_PIN)) == 0) &&
        ((PIN(DUTY_SEL2_PORT) & _BV(DUTY_SEL2_PIN)) == 1))
            return DUTY_50;
    if (((PIN(DUTY_SEL1_PORT) & _BV(DUTY_SEL1_PIN)) == 1) &&
        ((PIN(DUTY_SEL2_PORT) & _BV(DUTY_SEL2_PIN)) == 0))
            return DUTY_75;
    return DUTY_100;
}

uint8_t readRatesettings(void) {
    if (((PIN(RATE_SEL1_PORT) & _BV(RATE_SEL1_PIN)) == 0) &&
        ((PIN(RATE_SEL2_PORT) & _BV(RATE_SEL2_PIN)) == 1))
            return PS2_SAMPLES_PER_SEC_SLOW;
    if (((PIN(RATE_SEL1_PORT) & _BV(RATE_SEL1_PIN)) == 1) &&
        ((PIN(RATE_SEL2_PORT) & _BV(RATE_SEL2_PIN)) == 0))
            return PS2_SAMPLES_PER_SEC_MID;
    return PS2_SAMPLES_PER_SEC_FAST;
}

//---------------------------------------------------------------------------
// Сохранить принятый байт в буфер приёма PS/2 порта. Вызывается только из обработчика прерывания.
void ps2_rx_push(uint8_t c) {
    // Если буфер переполнен и потерян байт, то программа не сможет правильно 
    // расшифровать все дальнейшие пакеты, поэтому перезагружаем контроллер.
    if (ps2_rx_buf_count >= PS2_BUF_SIZE) {
        ps2_state = ps2_state_error;
        return;
    }
    // Сохраняем в буфер
    ps2_rx_buf[ps2_rx_buf_w] = c;
    ps2_rx_buf_count++;
    if (++ps2_rx_buf_w == PS2_BUF_SIZE) {
        ps2_rx_buf_w = 0;
    }
}

//---------------------------------------------------------------------------
// Получить байт из приёмного буфера PS/2 порта
uint8_t ps2_read(void) {
    uint8_t data;
    
    cli(); // Выключаем прерывания, так как обработчик прерывания тоже модифицирует эти переменные.
    // Если буфер пуст, возвращаем ноль
    if (ps2_rx_buf_count == 0) {
        data = 0;
    } else {
        // Читаем байт из буфера
        data = ps2_rx_buf[ps2_rx_buf_r];
        ps2_rx_buf_count--;
        if (++ps2_rx_buf_r == PS2_BUF_SIZE) {
            ps2_rx_buf_r = 0;
        }
    }

    sei(); // Включаем прерывания
    return data;
}

//---------------------------------------------------------------------------
// Вычисление бита чётности
const uint8_t nibble_parity[16] = {
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
};

uint8_t parity(uint8_t p) {
    return nibble_parity[p >> 4] ^ nibble_parity[p & 0x0F];
}

//---------------------------------------------------------------------------
// Изменение тактового сигнала PS/2
ISR (INT1_vect) {
    if (ps2_state == ps2_state_error) {
        return;
    }

    if (ps2_state == ps2_state_write) {
        switch (ps2_bitcount) {
            default: // Данные
                if (ps2_data & 1) {
                    DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
                } else {
                    DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);
                }
                ps2_data >>= 1;
                break;
            case 3: // Бит чётности
                if (ps2_parity) {
                    DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
                } else {
                    DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);
                }
                break;
            case 2: // Стоп бит
                DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
                break;
            case 1: // Подтверждение приёма
                if (ps2_data()) {
                    ps2_state = ps2_state_error;
                } else {
                    ps2_state = ps2_state_read; 
                }
                ps2_bitcount = 12;
                break;
                     
        } 
    } else {
        switch (ps2_bitcount) {
            case 11: // Старт бит
                if (ps2_data()) {
                    ps2_state = ps2_state_error;
                }
                break;
            default: // Данные
                ps2_data >>= 1;
                if (ps2_data()) {
                    ps2_data |= 0x80;
                }
                break;
            case 2: // Бит четности 
                if (parity(ps2_data) != (ps2_data() != 0)) {
                    ps2_state = ps2_state_error;
                }
                break;
            case 1: // Стоп бит 
                if (ps2_data()) {
                    ps2_rx_push(ps2_data);
                } else {
                    ps2_state = ps2_state_error;
                }
                ps2_bitcount = 12;
        }
    }
    ps2_bitcount--;
}

//---------------------------------------------------------------------------
// Инициализация PS/2
void ps2_init(void) {
    // Переключаем PS/2 порт на приём
    DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);
    DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);

    // Очищаем приёмный буфер
    ps2_rx_buf_w = 0;
    ps2_rx_buf_r = 0;
    ps2_rx_buf_count = 0;

    // Устанавливаем переменные обработчика прерывания
    ps2_state = ps2_state_read;
    ps2_bitcount = 11;

    // Прерывание по срезу тактового сигнала
    GIFR = _BV(INTF1);
    GICR |= _BV(INT1);
    MCUCR = (MCUCR & 0xFC) | 2;
}

//---------------------------------------------------------------------------
// Отправка байта в PS/2 порт без подтверждения
void ps2_write(uint8_t a) {
    // Отключаем прерывание по изменению тактового сигнала PS/2
    GIFR = _BV(INTF1);
    GICR &= ~_BV(INT1);

    // Замыкаем тактовый сигнал PS/2 на землю
    PORT(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);
    DDR(PS2_CLK_PORT) |= _BV(PS2_CLK_PIN);

    // ждём в течение 100 мкс
    _delay_us(100);

    // Замыкаем линию данных PS/2 на землю
    PORT(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);
    DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);

    // Освобождаем тактовый сигнал
    DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);

    // Очищаем приёмный буфер
    ps2_rx_buf_count = 0;
    ps2_rx_buf_w = 0;
    ps2_rx_buf_r = 0;

    // Настраиваем переменные обработчика прерывания
    ps2_state = ps2_state_write;
    ps2_bitcount = 11;
    ps2_data = a;             
    ps2_parity = parity(a);

    // Включаем прерывание по срезу тактового сигнала PS/2
    GIFR = _BV(INTF1);
    GICR |= _BV(INT1);
    MCUCR = (MCUCR & 0xFC) | 2;    
}

//---------------------------------------------------------------------------
// Получение байта из PS/2 порта с ожиданием
uint8_t ps2_recv(void) {
    while (ps2_rx_buf_count == 0);
    return ps2_read();
}

//---------------------------------------------------------------------------
// Отправка байта в PS/2 порт с подтверждением
void ps2_send(uint8_t c) {
    ps2_write(c);
    if (ps2_recv() != 0xFA) {
        ps2_state = ps2_state_error;
    }
}

// Изменилось состояние линий DTR или RTS
ISR (INT0_vect) {
    // Сохраняем состояние в переменную
    mouse_enabled = get_mouse_power_state();

    // Сохраняем признак сброса
    mouse_reset = true;
}

//===========================================================================
// Soft SPI порт
//===========================================================================
// Инициализация Soft SPI
void spi_init(void) {
    // Установка начальных состояний
    spi_tx_buf_w = 0;
    spi_tx_buf_r = 0;
    spi_tx_buf_count = 0;
    spi_state_machine = 0;
    spi_busy = 0;

    spi_sck_low();    // SCK низкий
    spi_mosi_low();   // MOSI низкий
    spi_reset_low();  // Reset низкий

    mouse_enabled = get_mouse_power_state();
    if (mouse_enabled) {
        mouse_reset = true;
    }
}

//---------------------------------------------------------------------------
// Проверка готовности порта для приёма
bool ready2receive(void) {
    if (opt_irq_settings == USE_IRQ4) {
        return !get_IRQ4_state();
    }
    if (opt_irq_settings == USE_IRQ3) {
        return !get_IRQ3_state();
    }
    if (opt_irq_settings == USE_IRQX) {
        return !get_IRQX_state();
    }

    return true;
}

//---------------------------------------------------------------------------
// Отправка данных через SPI
void spi_send(uint8_t c) {
    while (spi_tx_buf_count == SPI_TX_BUFFER_SIZE) ; // Ждём, если буфер передачи полон

    cli(); // не допускаем конфликтов, эти переменные могут изменяться в прерывании
    spi_tx_buf[spi_tx_buf_w] = c;
    if (spi_tx_buf_count++ == 0) {
        if (!spi_busy) // если spi бездействует, запускаем передачу
            spi_timer_slow();
    }
    sei();
    if (++spi_tx_buf_w == SPI_TX_BUFFER_SIZE) {
        spi_tx_buf_w = 0;
    }
}

// Обработчик прерывания таймера
ISR(TIMER1_COMPA_vect) {
    static uint8_t current_byte;
    static uint8_t current_bit;

    switch (spi_state_machine) {
        case 0: if (ready2receive() || !device_init) {
                    device_init = 1;
                    spi_busy = 1;
                    current_byte = spi_tx_buf[spi_tx_buf_r];
                    current_bit = 7;
                    if (++spi_tx_buf_r == (SPI_TX_BUFFER_SIZE)) {
                        spi_tx_buf_r = 0;
                    }
                    spi_tx_buf_count--;
                    spi_timer_fast(); // Устанавливаем период для передачи данных
                } else {
                    break;
                }
        case 2:
        case 4:
        case 6:
        case 8:
        case 10:
        case 12:
        case 14:
                spi_sck_low();
                if (current_byte & (1 << current_bit--))
                    spi_mosi_high();
                else 
                    spi_mosi_low();
                spi_state_machine++;
                break;

        case 1:
        case 3:
        case 5:
        case 7:
        case 9:
        case 11:
        case 13:
        case 15:
                spi_sck_high();
                spi_state_machine++;
                break;

        case 16: // Конец передачи
                if (!spi_tx_buf_count) { // Буфер пуст
                    spi_timer_stop(); // остановить таймер
                    spi_busy = 0;
                } else {
                    spi_timer_slow(); // Устанавливаем период проверки возможности передачи
                }
                spi_sck_low();
                spi_mosi_low();
                spi_state_machine = 0;
                break;

        default: // Ошибка, сбрасываем устройство
                ps2_state = ps2_state_error;
    }
}

// Отправка конфигурации устройства через SPI
void spi_send_config(uint8_t opt_com, uint8_t opt_irq) {
    uint8_t config_data = 0;
    config_data |= (opt_com & 0x03);
    config_data |= ((opt_irq & 0x03) << 2);
    spi_send(config_data);
}

//===========================================================================
//  Управление светодиодом
//===========================================================================
// Включить светодиод
void flash_led() {
    led_enable();
    TCNT2 = 0x7F;
    TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20); // делитель на 1024
}

//---------------------------------------------------------------------------
// Выключение светодиода через некоторое время
ISR (TIMER2_OVF_vect) {
    TCCR2 = 0; // Стоп таймер
    led_disable();
}

//===========================================================================
// PS/2 мышь
//===========================================================================

//---------------------------------------------------------------------------
// Инициализация PS/2 мыши
static void ps2m_init() {
    // Посылаем команду "Сброс"
    ps2_send(0xFF);
    if (ps2_recv() != 0xAA) { 
        ps2_state = ps2_state_error; 
        return; 
    }
    if (ps2_recv() != 0x00) { 
        ps2_state = ps2_state_error; 
        return; 
    }
    
    if (opt_wheel_enabled) {
        // Включаем колесо и побочно устанавливаем 80 пакетов в секунду.    
        ps2_send(0xF3);
        ps2_send(0xC8);
        ps2_send(0xF3);
        ps2_send(0x64);
        ps2_send(0xF3);
        ps2_send(0x50);

        // Узнаём, получилось ли включить колесо
        ps2_send(0xF2);
        ps2m_wheel = ps2_recv();
    }

    // Разрешение 8 точек на мм
    ps2_send(0xE8);
    ps2_send(0x03);

    // Задаём количество сэмплов/сек
    ps2_send(0xF3);
    ps2_send(opt_rate_settings);

    // Включаем потоковый режим.
    ps2_send(0xF4);
}

//---------------------------------------------------------------------------
// Обработка поступивших с PS/2 порта данных
void ps2m_process() {
//    while (ps2_rx_buf_count < (3 + (ps2m_wheel ? 1 : 0))) {
//    }
    while (ps2_rx_buf_count >= (3 + (ps2m_wheel ? 1 : 0))) {
        ps2m_b = ps2_read() & 7; //! Тут старшие биты!!!
        ps2m_x += (int8_t)ps2_read();
        ps2m_y -= (int8_t)ps2_read();
        if (ps2m_wheel) {
            ps2m_z += (int8_t)ps2_read();
        }
    }
}

//---------------------------------------------------------------------------
ISR (TIMER0_OVF_vect) { // 80.13 Hz
    TCNT0 = TIMER0_CONST;

    static uint8_t cnt = 0;
    send_PS2_data_flag = cnt++ & 1;
}

//===========================================================================
// Интерфейс с компьютером
//===========================================================================
//---------------------------------------------------------------------------
// Отправка данных мыши через SPI
void spi_m_send(int8_t x, int8_t y, int8_t z, uint8_t b) {
    uint8_t lb, rb, mb;
    static uint8_t mb1;

    // Обработка сброса      
    if (mouse_reset) {
        mouse_reset = false; 
        _delay_ms(14);
        if (mouse_protocol == PROTOCOL_EM84520) {
            // Приветствие EM84520
            for (uint8_t i = 0; i < sizeof(EM84520_ID); i++) {
                spi_send(EM84520_ID[i]);
            }
        } else {
            // Приветствие Logitech/Microsoft Plus
            spi_send(0x4D);
            _delay_ms(63);
            spi_send(0x33);
        }
    }

    // Клавиши мыши
    lb = b & 1;
    rb = (b >> 1) & 1;
    mb = (b >> 2) & 1;

    // Стандартная часть протокола 
    spi_send((1 << 6) | (lb << 5) | (rb << 4) | ((y & 0xC0) >> 4) | ((x & 0xC0) >> 6));
    spi_send(x & 0x3F);
    spi_send(y & 0x3F);

    if (mouse_protocol == PROTOCOL_EM84520) {
        // Расширение EM84520
        spi_send((mb << 4) | (z & 0x0F));
    } else { 
        // Расширение Logitech/Microsoft Plus
        if (mb || mb1) {
            spi_send(mb << 5);
            mb1 = mb;
        }
    }
}

//===========================================================================
// Программа
//===========================================================================

static void init(void) {
// Настройка портов ввода-вывода и подтягивающих резисторов
//---------------------------------------------------------------------------
// Port B
    DDR(DUTY_SEL1_PORT) &= ~_BV(DUTY_SEL1_PIN);  // pin 12 (PB0) вход duty 75%
    PORT(DUTY_SEL1_PORT) |= _BV(DUTY_SEL1_PIN);  // Подтяжка на PB0

    DDR(COM_SEL2_PORT) &= ~_BV(COM_SEL2_PIN);  // pin 13 (PB1) вход COM3/COM4
    PORT(COM_SEL2_PORT) |= _BV(COM_SEL2_PIN);  // Подтяжка на PB1

    DDR(WHEEL_SEL_PORT) &= ~_BV(WHEEL_SEL_PIN);  // pin 14 (PB2) вход Wheel
    PORT(WHEEL_SEL_PORT) |= _BV(WHEEL_SEL_PIN);  // Подтяжка на PB2

    DDRB |=  _BV(3);  // pin 15 (PB3) как выход (MOSI)
    PORTB |= _BV(3);  // PB3 = 1

    DDRB &= ~_BV(4);  // pin 16 (PB4) как вход (MISO)
    PORTB |= _BV(4);  // Подтяжка на PB4

    DDRB |=  _BV(5);  // pin 17 (PB5) как выход (SCK)
    PORTB |= _BV(5);  // PB5 = 1

    DDRB  |= _BV(6);  // pin 7  (PB6) как выход (N/C)
    PORTB |= _BV(6);  // PB6 = 1

    DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN);  // pin 8  (PB7) как вход (Mouse data)
    PORT(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN); // Отключить подтяжку PB7

//---------------------------------------------------------------------------
// Port C
    DDR(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);  // pin 23 (PC0) вход COM2/COM3
    PORT(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);  // Подтяжка на PC0

    DDR(LED_PORT)  |= _BV(LED_PIN);  // pin 24 (PC1) как выход (LED)
    PORT(LED_PORT) |= _BV(LED_PIN);  // PC1 = 1, LED OFF

    DDR(ADC_PORT) &= ~_BV(IRQ3_PIN);  // pin 25 (PC2) вход IRQ3
    PORT(ADC_PORT) &= ~_BV(IRQ3_PIN); // Отключить подтяжку PC2

    DDR(ADC_PORT) &= ~_BV(IRQX_PIN);  // pin 26 (PC3) вход IRQX
    PORT(ADC_PORT) &= ~_BV(IRQX_PIN); // Отключить подтяжку PC3

    DDR(ADC_PORT) &= ~_BV(IRQ4_PIN);  // pin 27 (PC4) вход IRQ4
    PORT(ADC_PORT) &= ~_BV(IRQ4_PIN); // Отключить подтяжку PC4

    DDR(SPI_SCK_PORT) |= _BV(SPI_SCK_PIN);   // pin 28 (PC5) как выход soft SPI CLC
    PORT(SPI_SCK_PORT) &= ~_BV(SPI_SCK_PIN); // PC5 = 0

    DDRC &= ~_BV(6);  // pin 29 (PC7) как вход(аппаратный #Reset)
    PORTC |= _BV(6);  // Подтяжка на PC7

//---------------------------------------------------------------------------
// Port D
    DDR(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN);  // pin 30 (PD0) выход soft SPI reset
    PORT(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN); // PD0 = 1 Сброс подсистемы SPI slave

    DDR(SPI_MOSI_PORT) |= _BV(SPI_MOSI_PIN);   // pin 31 (PD1) выход soft SPI MOSI
    PORT(SPI_MOSI_PORT) &= ~_BV(SPI_MOSI_PIN); // PD1 = 0

    DDR(DTR_PORT) &= ~_BV(DTR_PIN); // pin 32 (PD2) вход DTR|RTS
    PORT(DTR_PORT) |= _BV(DTR_PIN); // Подтяжка на PD2

    DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);  // pin 1 (PD3) вход (Mouse clock)
    PORT(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN); // Отключить подтяжку PD3

    DDRD  |= _BV(4);  // pin 2 (PD4) как выход (N/C)
    PORTD |= _BV(4);  // PD4 = 1

    DDR(RATE_SEL2_PORT) &= ~_BV(RATE_SEL2_PIN);  // pin 9 (PD5) вход data rate 1/4
    PORT(RATE_SEL2_PORT) |= _BV(RATE_SEL2_PIN);  // Подтяжка на PD5

    DDR(RATE_SEL1_PORT) &= ~_BV(RATE_SEL1_PIN);  // pin 10 (PD6) вход data rate 1/2
    PORT(RATE_SEL1_PORT) |= _BV(RATE_SEL1_PIN);  // Подтяжка на PD6

    DDR(DUTY_SEL2_PORT) &= ~_BV(DUTY_SEL2_PIN);  // pin 11 (PD7) вход duty 50%
    PORT(DUTY_SEL2_PORT) |= _BV(DUTY_SEL2_PIN);  // Подтяжка на PD7

    // Timer 0
    // Clock source: System Clock (8 MHZ)
    // Частота на выходе таймера ~ 80.13Hz
    TCCR0 = _BV(CS02) | _BV(CS00); // CLK/1024
    TCNT0 = TIMER0_CONST; //0x9F

    // Таймер 1
    // Clock source: System Clock (8 MHZ)
    // Режим: CTC (Clear Timer on Compare Match) с OCR1A
    TCCR1A = 0;  // Normal port operation
    TCCR1B = (1 << WGM12);  // CTC mode
    
    // Расчет значения для сравнения (2mks)
    // Необходимое количество тактов = Время / Период_такта
    // 2mks / 125ns = 2000mks / 125ns = 16 тактов
    OCR1A = 15;  // 0-15 = 16 тактов

    // Таймер 2
    // Clock source: System Clock (8 MHZ)
    ASSR = 0; 
    TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20); // CLK/1024
    TCNT2 = 0x7F;
    OCR2 = 0;

    // Включение прерывания по изменению на входе INT0 - детектирование включения питания мыши
    GICR |= _BV(INT0);     // Разрешить прерывание INT0 (PD2)
    MCUCR = _BV(ISC10);    // Прерывание по любому изменению сигнала
    GIFR = _BV(INTF0);     // Очистить флаг прерывания (если был установлен раньше)

    // Timer(s)/Counter(s) Interrupt(s) initialization
    // Включаем прерывания от таймеров 0 и 2
    TIMSK = _BV(TOIE0)|_BV(OCIE1A)|_BV(TOIE2);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR = _BV(ACD);
    SFIOR = 0;

    // Инициализация ADC
    ADMUX = (0 << REFS1) | (0 << REFS0); // Vref = AVCC
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // Включить ADC, делитель 32

    // Настройка Watchdog-таймера
    wdt_enable(WDTO_1S);

    device_init = false;

    // Включение прерываний
    sei();
}

//---------------------------------------------------------------------------
static void sentToSpi() {
    static uint8_t smb1 = 0;
    
    send_PS2_data_flag = false;

    if (!mouse_enabled)
                return;

    if (ps2m_b != smb1 || ps2m_x != 0 || ps2m_y != 0 || ps2m_z != 0) {
        int8_t cx = ps2m_x < -128 ? -128 : (ps2m_x > 127 ? 127 : ps2m_x); 
        ps2m_x -= cx;
        int8_t cy = ps2m_y < -128 ? -128 : (ps2m_y > 127 ? 127 : ps2m_y); 
        ps2m_y -= cy;
        int8_t cz = ps2m_z < -8   ? -8   : (ps2m_z > 7   ?   7 : ps2m_z); 
        ps2m_z -= cz;
        
        smb1 = ps2m_b;
        spi_m_send(cx, cy, cz, ps2m_b);
        flash_led();
    }
}

static void checkJumpers() {
    opt_com_settings = readCOMsettings();
    opt_wheel_enabled = readWheelsettings();
    opt_duty_settings = readDutysettings();
    opt_rate_settings = readRatesettings();
}

// Чтение ADC канала
uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) ;
    return ADC;
}

uint8_t checkIRQ(uint8_t opt_com) {
    uint16_t tmp;

    PORT(ADC_PORT) |= _BV(IRQX_PIN); // Включить подтяжку PC3
    _delay_us(25); // ждём стабилизацию уровня
    tmp = adc_read(IRQX_PIN);
    PORT(ADC_PORT) &= ~_BV(IRQX_PIN); // Отключить подтяжку PC3

    // Определяем используется ли прерывание IRQX (джампер опускает напряжение к 0)
    if (tmp > IRQX_threshold) { // Напряжение на выводе IRQ выше заданного значения
        if (opt_com == SELECT_COM1 || opt_com == SELECT_COM3) {
            return USE_IRQ4;
        }
        return USE_IRQ3;
    }

    return USE_IRQX;
}

int main(void) {
    // Восстанавливаем настройки
    ps2m_multiplier = eeprom_read_byte(EEPROM_OFFSET_MULTIPLIER);
    if (ps2m_multiplier > 2) {
        ps2m_multiplier = 1; 
    }

    init();
    checkJumpers(); // Определяем конфигурацию джамперов
    opt_irq_settings = checkIRQ(opt_com_settings);

    spi_init();  // Инициализация SPI вместо UART
    spi_send_config(opt_com_settings, opt_irq_settings);

    ps2_init();
    ps2m_init();

    flash_led();

    for(;;) {
        // читаем данные из PS/2
        ps2m_process();

        // Отправляем компьютеру пакет, если в буфере отправки есть место, мышь включена, 
        // изменились нажатые кнопки или положение мыши
        if (send_PS2_data_flag) {
            sentToSpi();
        }

        // Регулирование скорости мыши прямо с мыши
        if (mouse_protocol == PROTOCOL_MICROSOFT && (ps2m_b & 3) == 3) {
            if (ps2m_z < 0) { 
                if (ps2m_multiplier > 0) {
                    ps2m_multiplier--; 
                }
                ps2m_z = 0; 
            } else if (ps2m_z > 0) { 
                if (ps2m_multiplier < 2) {
                    ps2m_multiplier++;
                }
                ps2m_z = 0; 
            }
        }

        // В случае ошибки перезагружаемся
        if (ps2_state != ps2_state_error) {
            wdt_reset();
        }
    }
}
