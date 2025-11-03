/******************************************************************************
 * Project : PS/2-to-ISA Mouse Adapter — Replica
 * File    : main.c
 * Author  : Pyshchev Alexander aka Yftul
 * License : GPL-3.0
 *
 * Description :
 *     Firmware module for microcontroller side of PS/2-to-ISA adapter.
 *     Handles PS/2 mouse communication, event parsing, buffering, and
 *     translation into ISA bus signals understood by legacy hardware.
 *
 * Notes :
 *     - Designed for embedded 8-bit MCU (e.g., AVR).
 *     - Uses interrupt-driven PS/2 input and simple state machine for
 *       compatibility and low latency.
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define F_CPU 8000000UL

#include <util/delay.h>
#include <util/atomic.h>

#define PS2_BUF_SIZE 255       // Размер приёмного буфера PS/2 порта
#define SPI_TX_BUFFER_SIZE 255 // Размер буфера передачи SPI

#define TIMER0_CONST 0x9F // Регулирует скорость передачи данных мыши
#define TIMER1_CONST 0x4F // Регулирует скорость передачи SPI
#define TIMER2_CONST 0x7F // Регулирует длительность послесвечения светодиода

#define serial_rate 8     // Задержка в мс при передаче данных без проверки 

#define GLUE(a, b)     a##b

#define PORT(x)        GLUE(PORT, x)
#define PIN(x)         GLUE(PIN, x)
#define DDR(x)         GLUE(DDR, x)
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

//===========================================================================
// pin definitions
//===========================================================================
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
// IRQ/ADC
//===========================================================================
#define ADC_PORT  C
#define IRQ3_PIN  2            // PC2 - нога 25 IRQ3
#define IRQX_PIN  3            // PC3 - нога 26 IRQX
#define IRQ4_PIN  4            // PC4 - нога 27 IRQ4

//===========================================================================
// Состояние UART IRQ
//===========================================================================
// 1 - не готовность приема символа, 0 - готовность
#define not_rdy_2rcv(irq_pin) (PIN(ADC_PORT) & _BV(irq_pin))

//===========================================================================
// Светодиод
//===========================================================================
#define led_on()        (PORT(LED_PORT) &= ~_BV(LED_PIN));
#define led_off()       (PORT(LED_PORT) |= _BV(LED_PIN));

//===========================================================================
// Soft SPI
//===========================================================================
#define spi_mosi_high()    (PORT(SPI_MOSI_PORT) |= _BV(SPI_MOSI_PIN))
#define spi_mosi_low()     (PORT(SPI_MOSI_PORT) &= ~_BV(SPI_MOSI_PIN))
#define spi_sck_high()     (PORT(SPI_SCK_PORT) |= _BV(SPI_SCK_PIN))
#define spi_sck_low()      (PORT(SPI_SCK_PORT) &= ~_BV(SPI_SCK_PIN))
#define spi_reset_high()   (PORT(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN))
#define spi_reset_low()    (PORT(SPI_RESOUT_PORT) &= ~_BV(SPI_RESOUT_PIN))

#define spi_timer_stop()   do {TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); TCNT1 = 0;} while (0);
#define spi_timer_fast()   do {TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); TCNT1 = 0; TCCR1B |= (1 << CS10);} while (0);
#define spi_timer_slow()   do {TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); TCNT1 = 0; TCCR1B |= (1 << CS11);} while (0);

#define SPI_SEND_BIT(bit_mask, tx_data) do { \
    spi_sck_low(); \
    asm volatile("nop\n\t"); \
    if ((tx_data) & (bit_mask)) spi_mosi_high(); else spi_mosi_low(); \
    asm volatile("nop\n\t"); \
    spi_sck_high(); \
    asm volatile("nop\n\t"); \
} while (0)

#define SPI_SEND_PACKET(tx_data) do {\
    SPI_SEND_BIT(0x40, (tx_data));\
    SPI_SEND_BIT(0x20, (tx_data));\
    SPI_SEND_BIT(0x10, (tx_data));\
    SPI_SEND_BIT(0x08, (tx_data));\
    SPI_SEND_BIT(0x04, (tx_data));\
    SPI_SEND_BIT(0x02, (tx_data));\
    SPI_SEND_BIT(0x01, (tx_data));\
    /*Завершение передачи*/\
    spi_sck_low();\
    spi_mosi_low();\
}while (0);

#define SPI_RESET() do {\
    spi_reset_high();\
    spi_reset_low();} while (0);

//===========================================================================
// PS/2
//===========================================================================
#define ps2_data_in()           (PIN(PS2_DATA_PORT) & _BV(PS2_DATA_PIN))
#define ps2_data_set_in()       (DDR(PS2_DATA_PORT) &= ~_BV(PS2_DATA_PIN))
#define ps2_data_set_out()      (DDR(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN))
#define get_mouse_power_state() (PIN(DTR_PORT) & _BV(DTR_PIN))

//===========================================================================
// Select UART base address
//===========================================================================
typedef enum {
    SELECT_COM1 = 0,
    SELECT_COM2 = 1,
    SELECT_COM3 = 2,
    SELECT_COM4 = 3,
} com_select_t;

//===========================================================================
// Select mouse Duty cycles
//===========================================================================
typedef enum {
    DUTY_100 = 0,
    DUTY_75 = 1,
    DUTY_50 = 2,
} duty_t;

//===========================================================================
// Select data rate
//===========================================================================
// Скорость мыши
// Допустимые значения: 10, 20, 40, 60, 80, 100, 200
#define PS2_SAMPLES_PER_SEC_FAST    80 
#define PS2_SAMPLES_PER_SEC_MID     40
#define PS2_SAMPLES_PER_SEC_SLOW    20

//===========================================================================
// Глобальные переменные
//===========================================================================
enum ps_state_t {
    ps2_state_error,
    ps2_state_read,
    ps2_state_write
};

volatile uint8_t ps2m_wheel;
volatile uint8_t ps2_state;
volatile uint8_t ps2_bitcount;
volatile uint8_t ps2_data;
volatile uint8_t ps2_parity;
uint8_t ps2_rx_buf[PS2_BUF_SIZE];
volatile uint8_t ps2_rx_buf_w;
volatile uint8_t ps2_rx_buf_r;
volatile uint8_t ps2_rx_buf_count;

uint8_t opt_com_settings;
uint8_t opt_wheel_enabled;
uint8_t opt_duty_settings;
uint8_t opt_rate_settings;
uint8_t opt_irq_settings;

uint8_t spi_tx_buf[SPI_TX_BUFFER_SIZE];
volatile uint8_t spi_tx_buf_w;
volatile uint8_t spi_tx_buf_r;
volatile uint8_t spi_tx_buf_count;

volatile uint8_t mouse_start;      // Необходимо вывести приветствие мыши
volatile uint8_t mouse_enabled;    // Мышь активна
volatile uint8_t device_init;      // Произведена инициализация адресов/IRQ устройства
volatile uint8_t allow_send_data;  // Флаг синхронизации передачи данных

//===========================================================================
// Декларации функций
//===========================================================================
static inline void ps2_rx_push(uint8_t c);
void spi_send_config(uint8_t opt_com, uint8_t opt_irq);
void spi_send(uint8_t c);

//===========================================================================
// Общие функции
//===========================================================================
// Отправить MCU в сон
//---------------------------------------------------------------------------
static inline void mcu_sleep(void) {
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

//---------------------------------------------------------------------------
// Ограничение переменной значениями [min, max]
static inline int8_t clamp(int16_t val, int8_t min, int8_t max) {
    if (val < (int16_t)min) return min;
    if (val > (int16_t)max) return max;
    return (int8_t)val;
}

//===========================================================================
// Прерывания
//===========================================================================
// Изменилось состояние линий DTR или RTS
ISR (INT0_vect) {
    // Сохраняем состояние питания мыши
    mouse_enabled = get_mouse_power_state();

    // Устанавливаем признак сброса
    if (mouse_enabled) {
        mouse_start = true;
        allow_send_data = true;
    }
}

//---------------------------------------------------------------------------
// Изменение тактового сигнала PS/2
ISR (INT1_vect) {
    if (unlikely(ps2_state == ps2_state_error)) {
        return;
    }

    if (ps2_state == ps2_state_write) {
        switch (ps2_bitcount) {
            default: // Данные
                (ps2_data & 1)?ps2_data_set_in():ps2_data_set_out();
                ps2_data >>= 1;
                break;
            case 3: // Бит чётности
                (ps2_parity)?ps2_data_set_in():ps2_data_set_out();
                break;
            case 2: // Стоп бит
                ps2_data_set_in();
                break;
            case 1: // Подтверждение приёма
                ps2_state = (unlikely(ps2_data_in()))?ps2_state_error:ps2_state_read;
                ps2_bitcount = 12;
                break;
        }
    } else {
        switch (ps2_bitcount) {
            case 11: // Старт бит
                if (unlikely(ps2_data_in())) {
                    ps2_state = ps2_state_error;
                }
                break;
            default: // Данные
                ps2_data >>= 1;
                if (ps2_data_in()) {
                    ps2_data |= 0x80;
                }
                break;
            case 2: // Бит четности
                if (unlikely(__builtin_parity(ps2_data) == ps2_data_in())) {
                    ps2_state = ps2_state_error;
                }
                break;
            case 1: // Стоп бит
                if (likely(ps2_data_in())) {
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
ISR (TIMER0_OVF_vect) { // 80.13 Hz
    TCNT0 = TIMER0_CONST;

    allow_send_data = true;
}

//---------------------------------------------------------------------------
// Обработчик прерывания таймера
ISR(TIMER1_COMPA_vect) {
    uint8_t current_byte;

    // Проверяем возможность передачи
    if (unlikely(not_rdy_2rcv(opt_irq_settings) && device_init)) {
        // Медленно продолжаем проверять возможность передачи
        spi_timer_slow();
        return;
    }

    // Нечего передавать, останавливаемся
    if (unlikely(!spi_tx_buf_count)) {
        spi_timer_stop();
        return;
    }

    device_init = true;

    // Забираем байт из буфера
    current_byte = spi_tx_buf[spi_tx_buf_r];
    if (unlikely(++spi_tx_buf_r == SPI_TX_BUFFER_SIZE)) {
        spi_tx_buf_r = 0;
    }
    spi_tx_buf_count--;

    // Линейная передача без циклов
    SPI_SEND_PACKET(current_byte);

    // Управление таймером
    if (likely(!spi_tx_buf_count)) { // Нечего передавать, останавливаемся
        spi_timer_stop();
    } else {                         // Продолжаем передавать
        spi_timer_fast();
    }
}

//---------------------------------------------------------------------------
// Выключение светодиода через некоторое время
ISR (TIMER2_OVF_vect) {
    TCCR2 = 0; // Стоп таймер

    led_off();
}

//---------------------------------------------------------------------------
// Включаем прерывание по спаду тактового сигнала PS/2
static inline void enable_ps2_falling_int(void) {
    GIFR = _BV(INTF1);
    GICR |= _BV(INT1);
    MCUCR = (MCUCR & 0xF3) | _BV(ISC11);
}

//---------------------------------------------------------------------------
// Чтение джамперов
uint8_t readCOMsettings(void) {
    bool tmp = false;
    if (!((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN))) &&
        ((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN))))
            return SELECT_COM2;
    if (((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN))) &&
        !((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN))))
            return SELECT_COM4;

    // Перемычки на землю не обнаружены, проверяем перемычу между джамперами
    DDR(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);    // COM_SEL1 теперь выход
    PORT(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);  // Запишем туда 0
    _delay_us(1);                               // Ждём стабилизацию сигнала
    tmp = !(PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN));
    DDR(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);   // COM_SEL1 теперь вход
    PORT(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);   // Вернём подтяжку

    return tmp?SELECT_COM3:SELECT_COM1;
}

bool readWheelsettings(void) {
    return !(PIN(WHEEL_SEL_PORT) & _BV(WHEEL_SEL_PIN));
}

uint8_t readDutysettings(void) {
    if (!((PIN(DUTY_SEL1_PORT) & _BV(DUTY_SEL1_PIN))) &&
        ((PIN(DUTY_SEL2_PORT) & _BV(DUTY_SEL2_PIN))))
            return DUTY_50;
    if (((PIN(DUTY_SEL1_PORT) & _BV(DUTY_SEL1_PIN))) &&
        !((PIN(DUTY_SEL2_PORT) & _BV(DUTY_SEL2_PIN))))
            return DUTY_75;
    return DUTY_100;
}

uint8_t readRatesettings(void) {
    if (!((PIN(RATE_SEL1_PORT) & _BV(RATE_SEL1_PIN))) &&
        ((PIN(RATE_SEL2_PORT) & _BV(RATE_SEL2_PIN))))
            return PS2_SAMPLES_PER_SEC_SLOW;
    if (((PIN(RATE_SEL1_PORT) & _BV(RATE_SEL1_PIN))) &&
        !((PIN(RATE_SEL2_PORT) & _BV(RATE_SEL2_PIN))))
            return PS2_SAMPLES_PER_SEC_MID;
    return PS2_SAMPLES_PER_SEC_FAST;
}

//---------------------------------------------------------------------------
// Сохранить принятый байт в буфер приёма PS/2 порта. Вызывается только из обработчика прерывания.
static inline void ps2_rx_push(uint8_t c) {
    // Если буфер переполнен и потерян байт, то программа не сможет правильно
    // расшифровать все дальнейшие пакеты, поэтому перезагружаем контроллер.
    if (unlikely(ps2_rx_buf_count >= PS2_BUF_SIZE)) {
        ps2_state = ps2_state_error;
        return;
    }
    // Сохраняем в буфер
    ps2_rx_buf[ps2_rx_buf_w] = c;
    ps2_rx_buf_count++;
    if (unlikely(++ps2_rx_buf_w == PS2_BUF_SIZE)) {
        ps2_rx_buf_w = 0;
    }
}

//---------------------------------------------------------------------------
// Получить байт из приёмного буфера PS/2 порта
uint8_t ps2_read(void) {
    uint8_t data;

    // Выключаем прерывания, так как обработчик прерывания тоже модифицирует эти переменные.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Если буфер пуст, возвращаем ноль
        if (!ps2_rx_buf_count) {
            data = 0;
        } else {
            // Читаем байт из буфера
            data = ps2_rx_buf[ps2_rx_buf_r];
            ps2_rx_buf_count--;
            if (unlikely(++ps2_rx_buf_r == PS2_BUF_SIZE)) {
                ps2_rx_buf_r = 0;
            }
        }
    }
    return data;
}

//---------------------------------------------------------------------------
// Инициализация PS/2
void ps2_init(void) {
    // Очищаем приёмный буфер
    ps2_rx_buf_w = 0;
    ps2_rx_buf_r = 0;
    ps2_rx_buf_count = 0;

    // Устанавливаем переменные обработчика прерывания
    ps2_state = ps2_state_read;
    ps2_bitcount = 11;

    // Прерывание по спаду тактового сигнала
    enable_ps2_falling_int();
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
    ps2_parity = !__builtin_parity(a);

    enable_ps2_falling_int();
}

//---------------------------------------------------------------------------
// Получение байта из PS/2 порта с ожиданием
uint8_t ps2_recv(void) {
    while (likely(!ps2_rx_buf_count)) {
        mcu_sleep();
    }

    return ps2_read();
}

//---------------------------------------------------------------------------
// Отправка байта в PS/2 порт с подтверждением
void ps2_send(uint8_t c) {
    ps2_write(c);
    if (unlikely(ps2_recv() != 0xFA)) {
        ps2_state = ps2_state_error;
    }
}

//===========================================================================
// Soft SPI порт
//===========================================================================
// Инициализация Soft SPI
void spi_init(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        spi_tx_buf_w = 0;
        spi_tx_buf_r = 0;
        spi_tx_buf_count = 0;

        // CPLD сброшен, требуется передача байта конфигурации
        if (!device_init) {
            // Сбрасываем базовые настройки устройства
            spi_mosi_high();
            _delay_us(1);
            spi_mosi_low();
            // Инициализация порта и IRQ
            spi_send_config(opt_com_settings, opt_irq_settings);
        }
        _delay_us(1);
        spi_reset_low();
    }

    // Ждём инициализацию устройства
    while (!device_init) {
        mcu_sleep();
    }
}

//---------------------------------------------------------------------------
// Отправка данных через SPI
void spi_send(uint8_t c) {
    while (unlikely(spi_tx_buf_count == SPI_TX_BUFFER_SIZE)) {
        mcu_sleep();
    } ;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        spi_tx_buf[spi_tx_buf_w] = c;
        if (spi_tx_buf_count++ == 0)
                        spi_timer_fast();
        if (unlikely(++spi_tx_buf_w == SPI_TX_BUFFER_SIZE)) {
            spi_tx_buf_w = 0;
        }
    }
}

//---------------------------------------------------------------------------
// Отправка конфигурации устройства через SPI
void spi_send_config(uint8_t opt_com, uint8_t opt_irq) {
    uint8_t config_data = 0;
    config_data |= (opt_com & 0x03);            // Base adress
    config_data |= (opt_irq == IRQX_PIN)?0x4:0; // Не использовать стандартное IRQ

    spi_send(config_data);
}

//===========================================================================
//  Управление светодиодом
//===========================================================================
// Мигнуть светодиодом
static inline void flash_led() {
    led_on();
    TCNT2 = TIMER2_CONST;
    TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20); // делитель на 1024
}

//===========================================================================
// PS/2 мышь
//===========================================================================
// Инициализация PS/2 мыши
static void ps2m_init(void) {
    ps2_send(0xFF); // Сброс
    if (ps2_recv() != 0xAA) {
        ps2_state = ps2_state_error;
        return;
    }
    if (ps2_recv() != 0x00) {
        ps2_state = ps2_state_error;
        return;
    }

    if (opt_wheel_enabled) {
        // Пробуем включить колесо
        ps2_send(0xF3); ps2_send(200); // Sample rate 200
        ps2_send(0xF3); ps2_send(100); // Sample rate 100
        ps2_send(0xF3); ps2_send(80);  // Sample rate 80
        ps2_send(0xF2); // Получить ID мыши
        ps2m_wheel = ps2_recv();
    } else {
        ps2m_wheel = false;
    }

    flash_led(); // Мышь инициализирована

    // Разрешение 8 точек на мм
    ps2_send(0xE8);
    ps2_send(0x03);

    // Задаём количество сэмплов/сек
    ps2_send(0xF3);
    ps2_send(opt_rate_settings);

    // Включаем потоковый режим.
    ps2_send(0xF4);
}

//===========================================================================
// Интерфейс с компьютером
//===========================================================================
//---------------------------------------------------------------------------
// Отправка данных мыши через SPI
void spi_m_send(int8_t x, int8_t y, int8_t z, uint8_t b) {
    uint8_t left_b, right_b, middle_b;

    left_b = b & 1;
    right_b = (b >> 1) & 1;
    middle_b = (b >> 2) & 1;

    spi_send((1 << 6) | (left_b<< 5) | (right_b << 4) |
                      ((y & 0xC0) >> 4) | ((x & 0xC0) >> 6));
    spi_send(x & 0x3F);
    spi_send(y & 0x3F);
    if (ps2m_wheel)
        spi_send((middle_b << 4) | (z & 0x0F));

    flash_led();
}

//===========================================================================
// Инициализация и тело программы
//===========================================================================
static inline uint8_t get_reset_source(void) {
    uint8_t cause = MCUSR; // читаем причину
    MCUSR = 0;             // сбрасываем все флаги, чтобы не влияли на будущее
    return cause;
}

//---------------------------------------------------------------------------
static inline uint16_t adc_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= _BV(ADSC);
    while (likely(ADCSRA & _BV(ADSC))) ;
    return ADC;
}

//---------------------------------------------------------------------------
// Определение используемого IRQ
//---------------------------------------------------------------------------
uint8_t checkIRQ(uint8_t opt_com)
{
    const uint16_t threshold_mean = 1000;
    const uint16_t threshold_var  = 60;
    const uint8_t ADC_SAMPLES = 12;
    uint32_t sumsq = 0;
    uint16_t sum = 0;
    uint16_t val;

    // Кратковременно подаём 0 на вход ADC
    DDR(ADC_PORT) |= _BV(IRQX_PIN);   // pin 26 IRQX как выход с лог 0
    _delay_us(5);
    DDR(ADC_PORT) &= ~_BV(IRQX_PIN);  // pin 26 IRQX как вход
    // Если вывод не подключен, он должен какое-то время держать 0
    _delay_us(1);

    // Собираем ADC выборки
    for (uint8_t i = 0; i < ADC_SAMPLES; i++)
    {
        val = adc_read(IRQX_PIN);
        sum += val;
        sumsq += (uint32_t)val * val;
    }

    // Среднее значение
    uint16_t mean = sum / ADC_SAMPLES;

    // Целочисленная дисперсия (средний квадрат отклонений)
    // var = (E[x²] - (E[x])²)
    uint32_t mean_sq = (uint32_t)mean * mean;
    uint32_t var = (sumsq / ADC_SAMPLES);
    if (var > mean_sq) {
        var -= mean_sq;
    } else {
        var = 0;
    }

    /*
     * Теперь проверяем:
     * - сигнал стабильно высокий (mean > threshold_high)
     * - сигнал не шумный (var < threshold_var)
     * 
     * Пороговые значения подобраны эмпирически:
     *   mean > 1000  → уровень лог.1
     *   var < 3600   → низкий шум, квадрат дисперсии
     */
    if (mean > threshold_mean && var < (threshold_var*threshold_var))
        return IRQX_PIN;

    if (opt_com == SELECT_COM1 || opt_com == SELECT_COM3) {
        return IRQ4_PIN;
    }
    return IRQ3_PIN;

}

//---------------------------------------------------------------------------
static void init(void) {
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
    PORT(PS2_DATA_PORT) |= _BV(PS2_DATA_PIN);  // Включить подтяжку PB7(DATA)

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

    DDRC &= ~_BV(6);  // pin 29 (PC6) как вход(аппаратный #Reset)
    PORTC |= _BV(6);  // Подтяжка на PC6

//---------------------------------------------------------------------------
// Port D
    DDR(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN);  // pin 30 (PD0) выход soft SPI reset
    PORT(SPI_RESOUT_PORT) |= _BV(SPI_RESOUT_PIN); // PD0 = 1 Сброс подсистемы SPI

    DDR(SPI_MOSI_PORT) |= _BV(SPI_MOSI_PIN);   // pin 31 (PD1) выход soft SPI MOSI
    PORT(SPI_MOSI_PORT) &= ~_BV(SPI_MOSI_PIN); // PD1 = 0

    DDR(DTR_PORT) &= ~_BV(DTR_PIN); // pin 32 (PD2) вход DTR|RTS
    PORT(DTR_PORT) |= _BV(DTR_PIN); // Подтяжка на PD2

    DDR(PS2_CLK_PORT) &= ~_BV(PS2_CLK_PIN);  // pin 1 (PD3) вход (Mouse clock)
    PORT(PS2_CLK_PORT) |= _BV(PS2_CLK_PIN);  // Включить подтяжку PD3(CLK)

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

    // Расчет значения для сравнения
    // Необходимое количество тактов = Время / Период_такта
    // Число тактов = Период(ns) / 125(ns)
    OCR1A = TIMER1_CONST;

    // Таймер 2
    // Clock source: System Clock (8 MHZ)
    ASSR = 0;
    TCCR2 = _BV(CS22)|_BV(CS21)|_BV(CS20); // CLK/1024
    TCNT2 = TIMER2_CONST;
    OCR2 = 0;

    // Включение прерывания по изменению на входе INT0 - детектирование включения питания мыши
    GIFR = _BV(INTF0);  // Очистить флаг прерывания (если был установлен ранее)
    GICR  = _BV(INT0);  // Разрешить прерывание INT0 (PD2)
    MCUCR = _BV(ISC00); // Прерывание по любому изменению сигнала

    // Timer(s)/Counter(s) Interrupt(s) initialization
    // Включаем прерывания от таймеров 0 и 2
    TIMSK = _BV(TOIE0)|_BV(OCIE1A)|_BV(TOIE2);

    // Analog Comparator initialization
    // Analog Comparator: Off
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR = _BV(ACD);
    SFIOR = 0;

    // Инициализация ADC
    ADMUX = (0 << REFS1) | (1 << REFS0); // Vref = AVCC
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // Включить ADC, делитель 32

    // Определяем конфигурацию джамперов
    opt_com_settings = readCOMsettings();
    opt_wheel_enabled = readWheelsettings();
    opt_duty_settings = readDutysettings();
    opt_rate_settings = readRatesettings();
    opt_irq_settings = checkIRQ(opt_com_settings);

    switch(get_reset_source()) {
        case _BV(WDRF): // Watchdog reset
            device_init = true;
            break;
        case _BV(EXTRF): // External reset
            led_on();    // Сигнализируем исправность светодиода
            _delay_ms(50);
        case _BV(PORF):    // Power-on reset
        case _BV(BORF):    // Brown-out reset
            _delay_ms(50); // Ждём окончания переходных процессов
            led_off();
            device_init = false;
            break;
    }

    // Настройка Watchdog-таймера
    wdt_enable(WDTO_1S);

    // Включение прерываний
    sei();

    // Инициализация CPLD части
    spi_init();

    // Инициализация PS2 части
    ps2_init();
    ps2m_init();

    mouse_enabled = get_mouse_power_state();
    mouse_start = true;
}

//---------------------------------------------------------------------------
static inline void send_mouse_id(void) {
    // Имитация переходных процессов при включении
    _delay_ms(20);

    // Не передаем ID при кратковременных включениях
    if (!mouse_enabled)
                    return;

    // Приветствие Logitech/Microsoft Plus
    // Прерывание может быть запрещено, поэтому передаём
    // без проверки готовности, со скоростью COM мыши
    SPI_SEND_PACKET(0x4D); _delay_ms(serial_rate); // Сигнатура MS mouse "M"
    if (ps2m_wheel) {
        SPI_SEND_PACKET(0x5A); _delay_ms(serial_rate); // Сигнатура мыши с колёсиком "Z"
        SPI_SEND_PACKET(0x40); _delay_ms(serial_rate); // Четырехбайтные пакеты
        SPI_SEND_PACKET(0x00); _delay_ms(serial_rate); // Пустой пакет байт 2
        SPI_SEND_PACKET(0x00); _delay_ms(serial_rate); // Пустой пакет байт 3
        SPI_SEND_PACKET(0x00); _delay_ms(serial_rate); // Пустой пакет байт 4
    } else {
        SPI_SEND_PACKET(0x33); _delay_ms(serial_rate); // Сигнатура 3х кнопочной мыши "3"
        SPI_SEND_PACKET(0x40); _delay_ms(serial_rate); // Трёхбайтные пакеты
        SPI_SEND_PACKET(0x00); _delay_ms(serial_rate); // Пустой пакет байт 2
        SPI_SEND_PACKET(0x00); _delay_ms(serial_rate); // Пустой пакет байт 3
    }
}

//---------------------------------------------------------------------------
static inline void do_process(void) {
    static uint8_t st_m_bt = 0;
    static uint8_t dr_ctr = 0;
    static uint8_t m_bt;
    static int16_t m_cx;
    static int16_t m_cy;
    static int16_t m_cz;

    // Полностью выбираем принятые пакеты
    while (ps2_rx_buf_count >= (ps2m_wheel ? 4 : 3)) {
        m_bt = ps2_read() & (ps2m_wheel?7:3);
        m_cx += (int8_t)ps2_read();
        m_cy -= (int8_t)ps2_read();
        m_cz += ps2m_wheel?(int8_t)ps2_read():0;
    }

    // Мышь включена и таймер следующей посылки активен
    if (allow_send_data && mouse_enabled) {
        allow_send_data = false;
        dr_ctr = (dr_ctr + 1) & 0x03;

        // Инициализация мыши
        if (unlikely(mouse_start)) {
            mouse_start = false;

            // Очистка данных
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                m_cx = 0; m_cy = 0;
                m_cz = 0; m_bt = 0;

                spi_tx_buf_r = spi_tx_buf_w;
                spi_tx_buf_count = 0;
                spi_timer_stop();
                SPI_RESET();
            }

            send_mouse_id();
        }

        // пропускаем 0, 1 или 2 такта из 4
        if (likely(dr_ctr >= opt_duty_settings)) {
            // Передаём, если что-то изменилось
            if (m_bt != st_m_bt || m_cx || m_cy || m_cz) {
                int8_t cx = clamp(m_cx, -128, 127);
                m_cx -= cx;
                int8_t cy = clamp(m_cy, -128, 127);
                m_cy -= cy;
                int8_t cz = clamp(m_cz, -8, 7);
                m_cz -= cz;

                st_m_bt = m_bt;

                spi_m_send(cx, cy, cz, m_bt);
            }
        }
    }
}

//---------------------------------------------------------------------------
int main(void) {
    init();

    for(;;) {
        if (likely(ps2_state != ps2_state_error)) {
            wdt_reset();
        }

        do_process();

        // Усыпляем процессор, если нет данных для обработки
        if (likely((ps2_state == ps2_state_error) ||
                        (!allow_send_data &&
                         !ps2_rx_buf_count &&
                         !spi_tx_buf_count))
            ) mcu_sleep();
    }
}
