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
#include <util/atomic.h>

#define PS2_BUF_SIZE 128       // Размер приёмного буфера PS/2 порта
#define SPI_TX_BUFFER_SIZE 128 // Размер буфера передачи SPI

#define TIMER0_CONST 0x9F // Регулирует скорость передачи данных мыши
#define TIMER1_CONST 0x4F // Регулирует скорость передачи SPI
#define TIMER2_CONST 0x7F // Регулирует длительность послесвечения светодиода

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
#define led_on()        PORT(LED_PORT) &= ~_BV(LED_PIN);
#define led_off()       PORT(LED_PORT) |= _BV(LED_PIN);

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
#define spi_timer_slow()   {TCCR1B &= ~(1 << CS12 | 1 << CS11 | 1 << CS10); TCNT1 = 0; TCCR1B |= (1 << CS11);}

#define SPI_SEND_BIT(bit_mask, tx_byte) \
    spi_sck_low(); \
    asm volatile("nop\n\t");\
    if (tx_byte & (bit_mask)) spi_mosi_high(); else spi_mosi_low(); \
    asm volatile("nop\n\t");\
    spi_sck_high();\
    asm volatile("nop\n\t");

#define SPI_SEND_BYTE(tx_byte) \
    SPI_SEND_BIT(0x80, tx_byte);\
    SPI_SEND_BIT(0x40, tx_byte);\
    SPI_SEND_BIT(0x20, tx_byte);\
    SPI_SEND_BIT(0x10, tx_byte);\
    SPI_SEND_BIT(0x08, tx_byte);\
    SPI_SEND_BIT(0x04, tx_byte);\
    SPI_SEND_BIT(0x02, tx_byte);\
    SPI_SEND_BIT(0x01, tx_byte);

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
// Select UART IRQ
//===========================================================================
typedef enum {
    NO_IRQ = 0,
    USE_IRQ4 = 1,
    USE_IRQ3 = 2,
    USE_IRQX = 3,
} use_irq_t;

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
#define PS2_SAMPLES_PER_SEC_FAST    80 // 10, 20, 40, 60, 80, 100, 200
#define PS2_SAMPLES_PER_SEC_MID     40 // 10, 20, 40, 60, 80, 100, 200
#define PS2_SAMPLES_PER_SEC_SLOW    20 // 10, 20, 40, 60, 80, 100, 200

//===========================================================================
// Глобальные переменные
//===========================================================================
enum ps_state_t { 
    ps2_state_error, 
    ps2_state_read, 
    ps2_state_write 
};

volatile uint8_t ps2_state;       // состояние порта (ps_state_t)
volatile uint8_t ps2_bitcount;    // счётчик битов обработчика
volatile uint8_t ps2_data;        // буфер на байт
volatile uint8_t ps2_parity;
uint8_t ps2_rx_buf[PS2_BUF_SIZE]; // Приёмный буфер PS/2 порта
volatile uint8_t ps2_rx_buf_w;
volatile uint8_t ps2_rx_buf_r;
volatile uint8_t ps2_rx_buf_count;

volatile bool send_PS2_data_flag;

volatile uint8_t opt_com_settings;
volatile uint8_t opt_wheel_enabled;
volatile uint8_t opt_duty_settings;
volatile uint8_t opt_rate_settings;
volatile uint8_t opt_irq_settings;

volatile bool ps2m_wheel; // Используемый протокол: с колесом или без
volatile uint8_t ps2m_b; // Нажатые кнопки
volatile int16_t ps2m_x; // Координаты мыши
volatile int16_t ps2m_y;
volatile int16_t ps2m_z;

uint8_t spi_tx_buf[SPI_TX_BUFFER_SIZE];
volatile uint8_t spi_tx_buf_w;
volatile uint8_t spi_tx_buf_r;
volatile uint8_t spi_tx_buf_count;
volatile uint8_t spi_state_machine;

volatile bool mouse_reset; // Необходима инициализация мыши
volatile bool mouse_enabled;
volatile bool device_init; // Произведена инициализация адресов/IRQ устройства

//===========================================================================
// Декларации функций
//===========================================================================
static inline void ps2_rx_push(uint8_t c);
static inline bool ready2receive(void);
void spi_send_config(uint8_t opt_com, uint8_t opt_irq);

//===========================================================================
// Прерывания
//===========================================================================
// Изменилось состояние линий DTR или RTS
ISR (INT0_vect) {
    // Сохраняем состояние в переменную
    mouse_enabled = get_mouse_power_state();

    // Сохраняем признак сброса
    mouse_reset = true;

    // Очищаем буфер SPI
    spi_tx_buf_r = spi_tx_buf_w;
    spi_tx_buf_count = 0;
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

    send_PS2_data_flag = true;
}

//---------------------------------------------------------------------------
// Обработчик прерывания таймера
ISR(TIMER1_COMPA_vect) {
    uint8_t current_byte;

    // Проверяем возможность передачи
    if (unlikely(!ready2receive() && device_init)) {
        spi_timer_slow();
        return;
    }

    if (unlikely(!spi_tx_buf_count)) {
        spi_timer_stop();
        return;
    }
    
    device_init = 1;

    // Забираем байт из буфера
    current_byte = spi_tx_buf[spi_tx_buf_r];
    if (unlikely(++spi_tx_buf_r == SPI_TX_BUFFER_SIZE)) {
        spi_tx_buf_r = 0;
    }
    spi_tx_buf_count--;
    
    // Линейная передача без циклов
    SPI_SEND_BYTE(current_byte);
    
    // Завершение передачи
    spi_sck_low();
    spi_mosi_low();
    
    // Управление таймером
    if (likely(!spi_tx_buf_count)) {
        spi_timer_stop();
    } else {
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
    register bool tmp = false;
    if (!((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN))) &&
        ((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN))))
            return SELECT_COM2;
    if (((PIN(COM_SEL1_PORT) & _BV(COM_SEL1_PIN))) &&
        !((PIN(COM_SEL2_PORT) & _BV(COM_SEL2_PIN))))
            return SELECT_COM4;

    // Перемычки на землю не обнаружены, проверяем перемычу между джамперами
    DDR(COM_SEL1_PORT) |= _BV(COM_SEL1_PIN);    // COM_SEL1 теперь выход
    PORT(COM_SEL1_PORT) &= ~_BV(COM_SEL1_PIN);  // Запишем туда 0
    _delay_us(1);                               // Подождём стабилизации сигнала
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
    register uint8_t data;
    
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

    // Прерывание по срезу тактового сигнала
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
    ps2_parity = !__builtin_parity(a);

    enable_ps2_falling_int();
}

//---------------------------------------------------------------------------
// Получение байта из PS/2 порта с ожиданием
uint8_t ps2_recv(void) {
    while (likely(!ps2_rx_buf_count)) ;
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
    spi_reset_high(); // Сброс подсистемы SPI CPLD
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Установка начальных состояний
        spi_tx_buf_w = 0;
        spi_tx_buf_r = 0;
        spi_tx_buf_count = 0;
        spi_state_machine = 0;

        // CPLD сброшен, требуется передача байта конфигурации
        device_init = false;

        spi_reset_low();  // CPLD активен
        spi_sck_low();    // SCK низкий
        spi_mosi_low();   // MOSI низкий

        mouse_enabled = get_mouse_power_state();
        mouse_reset = true;
    }

    spi_send_config(opt_com_settings, opt_irq_settings); // Инициализация CPLD
}

//---------------------------------------------------------------------------
// Проверка готовности порта для приёма
static inline bool ready2receive(void) {
    const uint8_t irq_pins[] = {NO_IRQ, IRQ4_PIN, IRQ3_PIN, IRQX_PIN};
    return !(PIN(ADC_PORT) & _BV(irq_pins[opt_irq_settings]));
}

//---------------------------------------------------------------------------
// Отправка данных через SPI
void spi_send(uint8_t c) {
    while (unlikely(spi_tx_buf_count == SPI_TX_BUFFER_SIZE)) ; // Ждём, если буфер передачи полон

    // не допускаем конфликтов, эти переменные могут изменяться в прерывании
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
    config_data |= (opt_com & 0x03);
    config_data |= ((opt_irq & 0x03) << 2);
    spi_send(config_data);
}

//===========================================================================
//  Управление светодиодом
//===========================================================================
// Включить светодиод
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
    ps2_send(0xFF); // Reset
    if (ps2_recv() != 0xAA) { 
        ps2_state = ps2_state_error; 
        return; 
    }
    if (ps2_recv() != 0x00) { 
        ps2_state = ps2_state_error; 
        return; 
    }

    if (opt_wheel_enabled) {
        // Попробуем включить колесо
        ps2_send(0xF3); ps2_send(200); // Sample rate 200
        ps2_send(0xF3); ps2_send(100);
        ps2_send(0xF3); ps2_send(80);
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

//---------------------------------------------------------------------------
// Обработка поступивших с PS/2 порта данных
void ps2m_process() {
    while (ps2_rx_buf_count >= (ps2m_wheel ? 4 : 3)) {
        ps2m_b = ps2_read() & 7; //! Тут старшие биты!!!
        ps2m_x += (int8_t)ps2_read();
        ps2m_y -= (int8_t)ps2_read();
        if (ps2m_wheel) {
            ps2m_z += (int8_t)ps2_read();
        }
    }
}

//===========================================================================
// Интерфейс с компьютером
//===========================================================================
//---------------------------------------------------------------------------
// Отправка данных мыши через SPI
void spi_m_send(int8_t x, int8_t y, int8_t z, uint8_t b) {
    register uint8_t lb, rb, mb;

    // Клавиши мыши
    lb = b & 1;         // левая кнопка
    rb = (b >> 1) & 1;  // правая кнопка
    mb = (b >> 2) & 1;  // средняя кнопка

    // Стандартная часть протокола 
    spi_send((1 << 6) | (lb << 5) | (rb << 4) | ((y & 0xC0) >> 4) | ((x & 0xC0) >> 6));
    spi_send(x & 0x3F);
    spi_send(y & 0x3F);

    // Расширенная часть протокола для мыши с колёсиком
    if (ps2m_wheel)
        spi_send(((mb & 1) << 4) | (z & 0x0F));

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
uint8_t checkIRQ(uint8_t opt_com) {
    PORT(ADC_PORT) |= _BV(IRQX_PIN); // Включить подтяжку PC3
    _delay_us(5); // ждём стабилизацию уровня
    uint16_t tmp = adc_read(IRQX_PIN);
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
    
    // Расчет значения для сравнения (2mks)
    // Необходимое количество тактов = Время / Период_такта
    // 2mks / 125ns = 2000mks / 125ns = 16 тактов (0-15)
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
    ADMUX = (0 << REFS1) | (0 << REFS0); // Vref = AVCC
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);  // Включить ADC, делитель 32

    // Определяем конфигурацию джамперов
    opt_com_settings = readCOMsettings();
    opt_wheel_enabled = readWheelsettings();
    opt_duty_settings = readDutysettings();
    opt_rate_settings = readRatesettings();
    opt_irq_settings = checkIRQ(opt_com_settings);

    switch(get_reset_source()) {
        case _BV(EXTRF): // External reset
                led_on();
                _delay_ms(100); // Сигнализируем исправность светодиода
                led_off();
                break;
        case _BV(WDRF): // Watchdog reset
        case _BV(PORF): // Power-on reset
        case _BV(BORF): // Brown-out reset
            break;
    }

    // Настройка Watchdog-таймера
    wdt_enable(WDTO_1S);

    // Включение прерываний
    sei();
}

//---------------------------------------------------------------------------
static void sentToSpi() {
    static uint8_t smb1 = 0;
    static uint8_t dr_ctr = 0;
    
    if (!send_PS2_data_flag || !mouse_enabled) return;

    send_PS2_data_flag = false;

    if (unlikely(mouse_reset)) {
        mouse_reset = false; 

        _delay_ms(14); // ?
        // Приветствие Logitech/Microsoft Plus
        spi_send(0x4D);
        _delay_ms(63);
        spi_send(0x33);
    }

    dr_ctr = (dr_ctr + 1) & 0x03;
    if (likely(dr_ctr >= opt_duty_settings)) { // пропускаем 0, 1 или 2 такта из 4
        if (ps2m_b != smb1 || ps2m_x != 0 || ps2m_y != 0 || ps2m_z != 0) {
            int8_t cx = ps2m_x < -128 ? -128 : (ps2m_x > 127 ? 127 : ps2m_x); 
            ps2m_x -= cx;
            int8_t cy = ps2m_y < -128 ? -128 : (ps2m_y > 127 ? 127 : ps2m_y); 
            ps2m_y -= cy;
            int8_t cz = ps2m_z < -8   ? -8   : (ps2m_z > 7   ?   7 : ps2m_z); 
            ps2m_z -= cz;
            
            smb1 = ps2m_b;

            spi_m_send(cx, cy, cz, ps2m_b);
        }
    }
}

int main(void) {
    init();

    spi_init();
    ps2_init();
    ps2m_init();

    for(;;) {
        if (likely(ps2_state != ps2_state_error)) {
            wdt_reset();
        }

        ps2m_process();
        sentToSpi();

        // Усыпляем процессор, если нет данных для обработки
        if (likely(!send_PS2_data_flag && !ps2_rx_buf_count && !spi_tx_buf_count)) {
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sleep_cpu();
            sleep_disable();
        }
    }
}
