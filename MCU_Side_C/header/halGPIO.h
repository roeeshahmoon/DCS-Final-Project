#ifndef _halGPIO_H_
#define _halGPIO_H_


#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer

typedef struct{
    char Name[10];        // File name
    char Data[70];  // 10 lines, max 7 chars (with ',') need to be 70
    char Script[10][7];
    int Address;
    char *P_File;
    unsigned int Size;
} File;

extern enum FSMstate state;   // global variable
extern enum SYSmode lpm_mode; // global variable
extern enum File_Mode File_Select; // global variable

extern void Py_Process_Data();
extern void  get_KB(char* p);
extern void reset_state_flag(int i);
extern void UART_CONFING();
extern void timer_delay();
extern void RGB_clear();
extern void  LPM0_GIE_SET();
extern void UpdatePWM(int f, int y,int i);
extern void sysConfig(void);
extern void SetByteToPort(char); // Added By RK
extern void clrPortByte(char);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void write_freq_tmp_LCD();
extern void write_signal_shape_tmp_LCD();
extern void ADC_config(void);
extern void StartSamplingADC12();
extern unsigned long  mult(signed short operand1, signed short operand2);
extern unsigned long   div(unsigned long Divided, unsigned short Divisor, unsigned long* Remainder);
extern void StartTimerA(unsigned int delay,int clk);
extern void StopTimerA();
extern void StartTimerB1();
extern void StartTimerA1();
extern void StartTimerB3();
extern void EnableInterruptTB3();
extern void StopTB0CTL();
extern void DMA_Transfer();
extern void write_char_flash(int adress, char value);
extern void erase_segment(int adress);
extern void MemToArr(char input[], char output[][7]);
extern void Script_Mod(File* P_myFile, int Num_file);
extern void clear_Flush();
extern void Write_File(File* P_myFile, int Num_file);
extern void Print_File_LCD(int i);


extern __interrupt void PBs_handler(void);
extern __interrupt void PBs_handler_P2(void);
extern __interrupt void Timer_A(void);
extern __interrupt void Timer_A1(void);
extern __interrupt void ADC12_ISR(void);
extern __interrupt void USART1_RX (void);
extern __interrupt void USART1_TX (void);
extern __interrupt void DMA_ISR(void);

extern File File_A;
extern File File_B;
extern File File_C;

extern int flag_exe_file_A;
extern int flag_write_file_A;

extern int flag_exe_file_B;
extern int flag_write_file_B;

extern int flag_exe_file_C;
extern int flag_write_file_C;

extern int flag_Clear_Flush;

#endif

// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~0X40) : (P2OUT|=0X40)) // P2.6 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X40) : (P2DIR|=0X40)) // P2.6 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~0X80) : (P2OUT|=0X80)) // P2.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X80) : (P2DIR|=0X80)) // P2.7 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off()          lcd_cmd(0x0C)
#define cursor_on()           lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line        lcd_cmd(0xC0)
#define DMA2_OFF()         DMA2CTL &= ~DMAEN
#define GIE_RESET_PUSHBUTTON()    P1IE &= ~0x0f;
#define GIE_RESET_PUSHBUTTON4()    P2IE &= ~0x01;
extern void StopTimerB();
extern void StopAllTimers();
extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void intToStr(int num,char* strnum);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
extern void RGB_color(const int x);
extern int GET_DIST(int i);
extern int GET_ARRAYDIST();
extern int GET_LDR_ARR(int i);
extern void RX_BYTE();
extern void scaning_Objects();
extern void calibaration();
extern void scaning_Lights();
extern void PWM_Trig_Ultrasonic();
extern void DistMeas_Echo(int i);
extern void TX_ARRAY();
extern void TX_INT(int var);
extern void TX_Telemeter();
extern void disable_interrupts_exept(int i);
extern void moving_servo(unsigned int Angle);
extern void calculate_voltage(int i);
extern void Light_Detector_Calibaration();
extern void enable_unterrupts_perpihal();
extern void disable_interrupts_perpihal();
extern void read_RXBUFF();
extern int get_flag_Calibaration();
extern void hex_char_to_int(char c, int *P_digit);
extern void hex_string_to_int(const char *hex_string, int* value);
extern void Instraction_Decoder(const char *Hex_Instruction);
extern void Servo_Scan(unsigned int L, unsigned int R);
extern void Servo_Deg(unsigned int P);
extern void Write_Calibration_Flush();
extern void write_int_flash(int adress, int value);
extern void Read_Calibration_Flush();
extern void read_seg_flash_int(int adress_source, int* adress_destiny);
extern void read_seg_flash_int(int adress_source, int* adress_destiny);
extern void Read_flash_file(File* P_myFile);
extern void Print_Servo_Scan_LCD(unsigned int L, unsigned int R);
extern void Print_Servo_Deg_LCD(unsigned int P);
extern void Print_Sleep_LCD();
extern void Clear_Flash();
extern void Print_Delete_Files_LCD();
extern void Print_Welcome_LCD();
extern void scaning_object_and_light();
extern void TX_STATE4();

/*
 *  Delay functions for HI-TECH C on the PIC18
 *
 *  Functions available:
 *      DelayUs(x)  Delay specified number of microseconds
 *      DelayMs(x)  Delay specified number of milliseconds
*/


#define NSMCLK_180_degree    2622
#define NSMCLK_135_degree    2124
#define NSMCLK_90_degree    1626
#define NSMCLK_45_degree    1128
#define NSMCLK_0_degree    630
#define NSMCLK_3_degree    34
#define NSMCLK_40_HZ    26290
#define NACLK_delay_60ms    2000
#define NACLK_delay_sec    32767
#define NSMCLK_17_HZ   61000
#define NSMCLK_widthHigh_29us  20
#define Made_By_TBCRR1  1
#define Made_By_TACRR1  2
#define SMCLK_FREQ 1048576

#define ACLK_FREQ 32768


