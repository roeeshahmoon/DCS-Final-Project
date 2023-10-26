#include  "../header/halGPIO.h"     // private library - HAL layer



#define infomation_seg_A 0x1000
#define infomation_seg_B 0x1080
#define seg_length 0x0080


#define Seg_File_A 0x1000
#define Seg_File_B 0x1046
#define Seg_File_C 0x108C
#define Seg_LDR_Calibaration 0x10D2




// Global Variables

unsigned int Count = 0x0,flag1=0x0,indx=0,flag_state1 = 0,flag_state2 = 0,flag_state6 = 0,A0results[60]={0},A1results[60]={0},flag_Calibaration=0, flag_files = 0;
unsigned int REdge1, REdge2,dist_mask[2] = {0},dist_mask_state1,ANGLE=0;
unsigned int range_array[61] = {0},deg_array[61] = {0}  ,send_mesg = '0',LDR_Calibaration[20]={0},Lihgt_Means[70]={0},lights_voltage_array[60];
unsigned long lights_LDR_MSB[60]={0},lights_LDR_LSB[60]={0};
int j = 0,RXBUF1_var;
unsigned int Oprand_A, Oprand_B;
unsigned int *P_Op_A = &Oprand_A, *P_Op_B = &Oprand_B;
char Str_OP_A[3], Str_OP_B[3];
int indx_for_scaning = 0,last_telemetor_sample;

int flag_exe_file_A = 0;
int flag_write_file_A = 0;
int k_A = 0;

int flag_exe_file_B = 0;
int flag_write_file_B = 0;
int k_B = 0;

int flag_exe_file_C = 0;
int flag_write_file_C = 0;
int k_C = 0;

int flag_Clear_Flush = 0;

File File_A;
File File_B;
File File_C;


//--------------------------------------------------------------------
//             System Configuration  
//--------------------------------------------------------------------
void sysConfig(void){
    //Clear_Flash();
	GPIOconfig();
	UART_CONFING();
	//disable_interrupts_exept(0);
	//__bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 + enable interrupt
	//Light_Detector_Calibaration();
	lcd_init();
	ADC_config();
}
int get_flag_Calibaration(){
    return flag_Calibaration;
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
// 				Set Byte to Port
//--------------------------------------------------------------------
void SetByteToPort(char ch){
	PBsArrPortOut |= ch;  
} 
//--------------------------------------------------------------------
// 				Clear Port Byte
//--------------------------------------------------------------------
void clrPortByte(char ch){
	PBsArrPortOut &= ~ch;
} 
//---------------------------------------------------------------------
//            Polling based Delay function
//---------------------------------------------------------------------
void delay(unsigned int t){  //
	volatile unsigned int i;
	
	for(i=t; i>0; i--);
}

//---------------------------------------------------------------------
//            Enter from LPM0 mode
//---------------------------------------------------------------------
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//---------------------------------------------------------------------
//            Enable interrupts
//---------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}
//---------------------------------------------------------------------
//            Disable interrupts
//---------------------------------------------------------------------
void disable_interrupts(){
  _BIC_SR(GIE);
}

//---------------------------------------------------------------------
//            LCD
//---------------------------------------------------------------------
//******************************************************************
// send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
// send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
// write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){
    while(*s)
        lcd_data(*s++);
}


//******************************************************************
// initialize the LCD
//******************************************************************
void lcd_init(){

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
// lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}

//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec

}

void disable_interrupts_exept(int i){
    if(i == 0)   {//i = 0 except from RX
        ADC12IE = 0;
        P1IE = 0;
        TACTL &= ~TAIE;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        IE2     &= ~UTXIE1;
        TACCTL0 &= ~CCIE;
        IE2     |= URXIE1;
    }
    else if(i==1){//i = 0 except from TX
        ADC12IE = 0;
        P1IE = 0;
        TACTL &= ~TAIE;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        IE2     &= ~URXIE1;
        TACCTL0 &= ~CCIE;

    }
    else if(i==2){//i = 0 except from TB.2
        ADC12IE = 0;
        P1IE = 0;
        TACTL &= ~TAIE;
        TBCTL &= ~TBIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        TACCTL0 &= ~CCIE;
        TBCCTL2 |= CCIE;

    }
    else if(i==3){//i = 0 except from TB
        ADC12IE = 0;
        P1IE = 0;
        TACTL &= ~TAIE;
        TBCCTL2 &= ~CCIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        TACCTL0 &= ~CCIE;
        TBCTL |= TBIE;

    }
    else if(i==4){//i = 0 except from TA
        ADC12IE = 0;
        P1IE = 0;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        TACCTL0 &= ~CCIE;
        TACTL |= TAIE;

    }
    else if(i==5){//i = 0 except from PB0
        ADC12IE = 0;
        TACTL &= ~TAIE;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        TACCTL0 &= ~CCIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        P1IE  = 0x01;

    }
    else if(i==6){//i = 0 except from ADC
        P1IE = 0;
        TACTL &= ~TAIE;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        TACCTL0 &= ~CCIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        ADC12IE = 0x2;

    }
    else if(i==7){//i = 0 except from TA0
        ADC12IE = 0;
        P1IE = 0;
        TBCTL &= ~TBIE;
        TBCCTL2 &= ~CCIE;
        IE2     &= ~UTXIE1;
        IE2     &= ~URXIE1;
        TACTL &= ~TAIE;
        TACCTL0 |= CCIE;

    }
}
void enable_unterrupts_perpihal(){
    ADC12IE = 0x02;
    P1IE = 0x01;
    TACTL |= TAIE;
    TBCTL |= TBIE;
    TBCCTL2 |= CCIE;
    TACCTL0 |= CCIE;
    IE2     |= URXIE1;
}
void disable_interrupts_perpihal(){
    ADC12IE = 0;
    P1IE = 0;
    TBCTL &= ~TBIE;
    TBCCTL2 &= ~CCIE;
    IE2     &= ~UTXIE1;
    IE2     &= ~URXIE1;
    TACTL &= ~TAIE;
    TACCTL0 &= ~CCIE;
}
//******************************************************************
// mult and div functions q-format
//******************************************************************

unsigned long  div(unsigned long Divided, unsigned short  Divisor, unsigned long* Remainder) {
    unsigned long R9 = 32;
    unsigned long R4 = Divided;
    unsigned long R5 = 0;
    unsigned short R6 = Divisor;
	unsigned short median = Divisor >> 1;
    unsigned long R8 = 0;
    unsigned long carry = 0;
    while (R9 > 0) {
        carry = (R4 & 0x80000000) >> 31;
        R4 = (R4 << 1) ;
        R5 = (R5 << 1) ;
        R5 |= carry;
        if (R6 > R5) {
            R8 = (R8 << 1) ;
        }
        else {
            R5 -= R6;
            carry = 1;
            R8 = (R8 << 1) ;
            R8 |= carry;

        }

        R9--;

    }


   *Remainder = R5;
   return R8;

}


unsigned long  mult(signed short operand1, signed short operand2) {
    unsigned short R8 = 1;
    unsigned long temp1=0;
    unsigned short R4 = operand1;
    unsigned short R5 = operand2;
    unsigned short R6 = 0;
    unsigned long R7 = 0;
    unsigned short carry = 0;
    while (R8 != 0) {
        if (R8 & R5) {
            R7 += R4;
        }
        carry = R7 & 0x1;
        R6 = R6 >> 1;
        R6 = R6 | (carry << 15);
        R7 = R7 >> 1;

        R8 = R8 << 1;

    }
    R6 = R6;
    R7 = R7 << 16;
    temp1 = (unsigned long)R6;
    return R7 + temp1;



}

void reset_state_flag(int i){
    if(i==1){
        flag_state1 = 0;

    }
    else if(i==2){
        flag_state2 = 0;
    }
}

void LPM0_GIE_SET(){
    __bis_SR_register(LPM0_bits + GIE);
}



void intToStr(int num,char* strnum) {
    int i = 0, lenNum = 1, tempNum = num;
    while (tempNum > 9) {
        tempNum /= 10;
        lenNum++;
    }
    strnum[lenNum - 1 - i++] = '0' + (num % 10);
    num /= 10;
    while (num > 0) {
        strnum[lenNum - 1 - i++] = '0' + (num % 10);
        num /= 10;
    }
    strnum[lenNum] = '\0' ;
}


int GET_DIST(int i){
    if(i < 4){
      return dist_mask[i];
    }
    else
    {
      return  0;
    }
}
int GET_ARRAYDIST(int j){
    j = j<<1;
    return range_array[j];
}
int GET_ANGLE(){
    return ANGLE;
}

int GET_LDR_ARR(int i){
    int res;
    res = (A0results[i]+A1results[i]) >> 1;

    return res;
}
//---------------------- Timer delay---------------------------------


void StopAllTimers(){
    StopTimerA();
    StopTimerB();
}


void Read_Calibration_Flush(){
    read_seg_flash_int(Seg_LDR_Calibaration,&LDR_Calibaration);
    }

void Write_Calibration_Flush(){
    int i,j = 0;
    for(i = 0; i< 40; i = i + 2){
      write_int_flash(Seg_LDR_Calibaration + i,LDR_Calibaration[j]);
      j++;
    }
}
//************************************************************************
//function while processing data
//************************************************************************

void Py_Process_Data(){
    char * s = "In Process";
    int RGB[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
    lcd_puts(s);
    cursor_off();
}

//************************************************************************
// Read data from segment
//************************************************************************
void Read_flash_file(File* P_myFile)
{
  char* adress_destiny = &((*P_myFile).Data);
  char *Flash_ptr_A;                         // Segment Information A pointer
  unsigned char i;
  int j = 0;

  Flash_ptr_A = (char *)((*P_myFile).Address);       // Initialize Flash segment C pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  while(*Flash_ptr_A){
    adress_destiny[j] = *Flash_ptr_A++;        // copy value segment C to segment D
    j++;
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

//************************************************************************
// Read data from segment
//************************************************************************
void read_seg_flash_int(int adress_source, int* adress_destiny)
{
  int *Flash_ptr_A;                         // Segment Information A pointer
  unsigned char i;
  int j = 0;

  Flash_ptr_A = (int *)adress_source;       // Initialize Flash segment C pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  for (i=0; i<seg_length; i = i + 2)
  {
    adress_destiny[j] = *Flash_ptr_A++;        // copy value segment C to segment D
    j++;
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


//************************************************************************
// Write integer in flash
//************************************************************************
void write_int_flash(int adress, int value)
{
  int *Flash_ptr;                           // Flash pointer

  Flash_ptr = (int *)adress;                // Initialize Flash pointer
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  *Flash_ptr = value;                       // Write value to flash

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}
//************************************************************************
// Write char in flash
//************************************************************************
void write_char_flash(int adress, char value){
  char *Flash_ptr;                          // Flash pointer

  Flash_ptr = (char *)adress;               // Initialize Flash pointer
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  *Flash_ptr = value;                       // Write value to flash

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


//*************************************************************************
// Erase segment
//*************************************************************************
void erase_segment(int adress){
  int *Flash_ptr;                           // Flash pointer

  Flash_ptr = (int *)adress;                // Initialize Flash pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit

  *Flash_ptr = 0;                           // Dummy write to erase Flash segment

  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


//-------------------------------------------------------------
//                    Script Mode State 5
//-------------------------------------------------------------

void Print_File_LCD(int i){
    char const * Str_File_A ="File A in Flash!";
    char const * Str_File_B ="File B in Flash!";
    char const * Str_File_C ="File C in Flash!";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    if( i == 1)
        lcd_puts(Str_File_A);
    else if(i == 2)
        lcd_puts(Str_File_B);
    else if(i == 3)
        lcd_puts(Str_File_C);
    cursor_off();
    enable_interrupts();
    StartTimerA(8192,3);
    lcd_clear();
}


void Clear_Flash(){
    FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
    erase_segment(infomation_seg_A);          // Erase flash Information A
    erase_segment(infomation_seg_B);          // Erase flash Information B

}


void Write_File(File* P_myFile, int Num_file){
    FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
    int j = 0;
    if(Num_file == 1)
        (*P_myFile).Address = Seg_File_A;

    else if (Num_file == 2)
        (*P_myFile).Address = Seg_File_B;

    else if (Num_file == 3)
        (*P_myFile).Address = Seg_File_C;

    while((*P_myFile).Data[j]){
        write_char_flash((*P_myFile).Address + j,(*P_myFile).Data[j]);
        j++;
        (*P_myFile).Size++;
    }
    (*P_myFile).Size++;
    Print_File_LCD(Num_file);

}

//******************************************************************
// Start timer A functions
//******************************************************************
void StartTimerA(unsigned int delay,int clk){              //4ms delay for ServoMotor Proccessing Data
    disable_interrupts_exept(7);
    if(clk == 1){
        TACTL = TASSEL_1 + MC_1 + TACLR ;  // ACLK, Up to CCR0
        TACCR0 = delay;              // to get 250HZ for 4ms
    }
    else if(clk == 2){
        TACTL = TASSEL_2 + MC_1 + TACLR ;  // SMCLK, Up to CCR0
        TACCR0 = delay;              // to get 250HZ for 4ms
    }
    else if(clk == 3){
        TACTL = TASSEL_1 + MC_1 + TACLR + ID_3 ;  // ACLK, Up to CCR0
        TACCR0 = delay;              // to get 250HZ for 4ms
    }
    else if(clk == 4){
        TACCR0 = delay<<6;  // delay value of 10 ms     (15ms = 2^-6)
        TACTL = TASSEL_1 + MC_1 + TACLR + ID_3 ;  // ACLK, Up to CCR0 2^12
    }
    TACCTL0 = CCIE;
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0
    TACCTL0 &= ~CCIE;

}

void StopTimerA(){
    TA0CTL = MC_0+TACLR;
}

//******************************************************************
// timer B functions
//******************************************************************

void StartTimerB1(){
    TBCTL = TBSSEL_2 + MC_1 + TBCLR ;                  //  SMCLK,  Up to CCR0
    TBCCTL1 =  OUTMOD_7; // TBCCR1 reset/set;
}

void StartTimerA1(){
    TACTL = TBSSEL_2 + MC_1 + TBCLR ;                  //  SMCLK,  Up to CCR0
    TACCTL1 =  OUTMOD_7; // TBCCR2 reset/set;

}

void StartTimerB3(){
    TBCTL = TBSSEL_2 + MC_1 + TBCLR ;         //SMCLK , Continous up, TA0CTL2, Capture Mode
    TBCCTL2 = CM_3 + CCIS_0 + SCS + CAP   ;
}

void EnableInterruptTB3(){

   // TBCTL |=  TBIE ;
    TB0CCTL2 |= CCIE;                                // enable interrupt
    PWM_Trig_Ultrasonic();
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 + enable interrupt
}

void StopTB0CTL(){
    TBCCTL1 = OUTMOD_5; // TBCCR1 reset;
    TB0CTL = MC_0 ; // Stop Timer
}


void StopTimerB(){
    TBCTL =  MC_0 + TBCLR;                  //  SMCLK, upmode,
}

void UpdatePWM( int f, int d,int i){
    if ( i == 1){
        TBCTL = TBSSEL_2 + MC_1 + TBCLR ;                  //  SMCLK,  Up to CCR0
        TBCCTL1 =  OUTMOD_7; // TBCCR1 reset/set;
        TBCCR0 = f ; // frequency PWM
        TBCCR1 = d;
    }
    else if (i==2){

        TACTL = TASSEL_2 + MC_1 + TACLR ;                  //  SMCLK,  Up to CCA0
        TACCTL1 =  OUTMOD_7; // TACCR1 reset/set;
        //TBCCTL1 =  OUTMOD_5; // TBCCR1 reset/set;
        TACCR0 = f ;
        TACCR1 = d;
    }

}
//******************************************************************
// ADC functions
//******************************************************************
void StartSamplingADC12(int j){
    indx = j;


    ADC_config();
    ADC12CTL0 |= ENC;                       // Enable conversions
    ADC12CTL0 |= ADC12SC;                   // Start conversions
    __bis_SR_register(LPM0_bits + GIE);     // LPM0
    ADC12CTL0 &= ~ENC;
    ADC12IE = 0;



}


//******************************************************************
// Drivers to UART
//******************************************************************
void RX_BYTE(){
    int res = 0,temp=0;
    temp = mult(dist_mask[1],80);
    res = dist_mask[0] + temp;
    dist_mask_state1 = res;


}



void TX_ARRAY(){
    int c = 0;
    indx = 0;
    UART_CONFING();
    disable_interrupts_exept(1);//1 = TX
    if(state == state1){
        while(range_array[indx] != 0){
            disable_interrupts();
            send_mesg = range_array[indx] ;
            IE2 |= UTXIE1;
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            indx++;

        }
        disable_interrupts();
        send_mesg = range_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx = 0 ;
        while(deg_array[indx] != 0 ){
            disable_interrupts();
            send_mesg = deg_array[indx] ;
            IE2 |= UTXIE1;
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            indx++;

        }
        disable_interrupts();
        send_mesg = deg_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx = 0 ;
    }
    if(state == state3){
        while(lights_LDR_MSB[indx] != 0){
            disable_interrupts();
            send_mesg = lights_LDR_MSB[indx] ;
            IE2 |= UTXIE1;
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            indx++;

        }
        disable_interrupts();
        send_mesg = lights_LDR_MSB[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx = 0 ;
        while(lights_LDR_LSB[indx] != 0){
            disable_interrupts();
            send_mesg = (int)lights_LDR_LSB[indx] ;
            IE2 |= UTXIE1;
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            indx++;

        }
        disable_interrupts();
        send_mesg = lights_LDR_LSB[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx = 0 ;
        while(deg_array[indx] != 0){
            disable_interrupts();
            send_mesg = deg_array[indx] ;
            IE2 |= UTXIE1;
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            indx++;

        }
        send_mesg = deg_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx = 0 ;
    }
    indx = 0;
    IE2 &= ~UTXIE1;
}

void TX_STATE4(){
    indx = 0;
    UART_CONFING();
    disable_interrupts_exept(1);//1 = TX

    while(range_array[indx] != 0){
        disable_interrupts();
        send_mesg = range_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx++;
    }

    disable_interrupts();
    send_mesg = range_array[indx] ;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    indx = 0 ;
    while(deg_array[indx] != 0 ){
        disable_interrupts();
        send_mesg = deg_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx++;

    }
    disable_interrupts();
    send_mesg = deg_array[indx] ;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    indx = 0 ;


    while(lights_LDR_MSB[indx] != 0){
        disable_interrupts();
        send_mesg = lights_LDR_MSB[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx++;

    }
    disable_interrupts();
    send_mesg = lights_LDR_MSB[indx] ;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    indx = 0 ;
    while(lights_LDR_LSB[indx] != 0){
        disable_interrupts();
        send_mesg = (int)lights_LDR_LSB[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx++;

    }
    disable_interrupts();
    send_mesg = lights_LDR_LSB[indx] ;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    indx = 0 ;
    while(deg_array[indx] != 0){
        disable_interrupts();
        send_mesg = deg_array[indx] ;
        IE2 |= UTXIE1;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        indx++;

    }
    send_mesg = deg_array[indx] ;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    indx = 0 ;
    IE2 &= ~UTXIE1;
}

void TX_INT(int var){
    UART_CONFING();
    disable_interrupts();
    send_mesg = var;
    IE2 |= UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
    IE2 &= ~UTXIE1;
}
void TX_Telemeter(){
    int send;
    dist_mask_state1 = 480;
    StartTimerA(8196,3) ;   //2s delay
    indx_for_scaning = 0;
    DistMeas_Echo(0);
    last_telemetor_sample = range_array[0];
    TX_INT(range_array[0]);
    while(state == state2){
          indx_for_scaning = 0;
          DistMeas_Echo(0);
          if(range_array[0] != last_telemetor_sample && ((range_array[0] - last_telemetor_sample > 1) || (-range_array[0] + last_telemetor_sample > 1)))
            TX_INT(range_array[0]);
            StartTimerA(8196,3) ;   //2s delay

    }
}

//******************************************************************
// Drivres to servo
//******************************************************************
void calibaration(){

    disable_interrupts_perpihal();
    UpdatePWM(NSMCLK_40_HZ, NSMCLK_180_degree,Made_By_TBCRR1);
    StartTimerA(NACLK_delay_sec,1) ;   //1s delay

    StopTimerA();

}

void scaning_Objects(){
    int i, N_Angle=NSMCLK_0_degree;      //change the duty cycle for change the angle
    disable_interrupts_perpihal();
    for(i = 0; i < 60; i++){
        StopTimerA();
        N_Angle += NSMCLK_3_degree ;
        UpdatePWM(NSMCLK_40_HZ,N_Angle,Made_By_TBCRR1); // 40 HZ
        StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
        StopTimerA();
        DistMeas_Echo(i);



    }
    UpdatePWM(NSMCLK_40_HZ,NSMCLK_90_degree,Made_By_TBCRR1); // 40 HZ
    StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
    StopTimerA();
    indx_for_scaning = 0;
}


void moving_servo(unsigned int Angle){
    int  N_Angle=0, *p_r,temp;
    temp = mult(Angle,7);
    temp = 1666 - temp;
    N_Angle = div(SMCLK_FREQ, temp,p_r) ;
    UpdatePWM(NSMCLK_40_HZ, N_Angle,Made_By_TBCRR1);
    StartTimerA(NACLK_delay_sec,1) ;   //1s delay



}

void Light_Detector_Calibaration(){
    unsigned int j=0,i=0,r=0,t=0;
    unsigned int LDR[40];
    unsigned long temp1,temp2,*p_resdiue1,*p_resdiue2,resdiue1,resdiue2;
    char * s[10] = {"5 CM","10 CM","15 CM","20 CM","25 CM","30 CM","35 CM","40 CM","45 CM","50 CM"};
    p_resdiue1 = &resdiue1;
    p_resdiue2 = &resdiue2;
    if( flag_Calibaration == 1){
        UpdatePWM(NSMCLK_40_HZ,NSMCLK_90_degree,Made_By_TBCRR1); // 40 HZ
        StartTimerA(NACLK_delay_sec,1) ;   //1s delay
        StopTimerA();


        StopTimerA();
        disable_interrupts_exept(5);
        StopAllTimers();
        StopTB0CTL();
        while(j<10){
            LEDsArrPort = j;
            lcd_clear();
            lcd_puts(s[j]);
            cursor_off();
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            StartSamplingADC12(j);
            disable_interrupts_exept(7);
            StartTimerA(2000,3) ;   //1s delay
            StopTimerA();
            disable_interrupts_exept(5);
            j+=1;



            flag_Calibaration++;
        }
        LEDsArrPort = 0;
        lcd_cmd(0x1); //Display Clear
        while(i<10){
            temp1  = div(A0results[i],100,p_resdiue1);
            temp2  = div(A1results[i],100,p_resdiue2);
            LDR_Calibaration[i] = temp1 + temp2;
            LDR_Calibaration[i+10] = resdiue1 + resdiue2 ;
            i++;



        }




        Clear_Flash();
        Write_Calibration_Flush();
    }

    else if (flag_Calibaration == 0){
        Read_Calibration_Flush();
    }


    while(t<20){
        TX_INT(LDR_Calibaration[t]);
        t++;
    }


    flag_state6 = 0;
    lcd_clear();

}

void scaning_Lights(){
    int i, N_Angle=NSMCLK_0_degree;      //change the duty cycle for change the angle
    int k = 0;

    for(i = 0; i < 60; i++){
        N_Angle += NSMCLK_3_degree ;
        UpdatePWM(NSMCLK_40_HZ,N_Angle,Made_By_TBCRR1); // 40 HZ
        StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
        StopTimerA();
        StartSamplingADC12(i);
        StartTimerA(2048,3) ;   //0.5s delay
        disable_interrupts();


    }
    UpdatePWM(NSMCLK_40_HZ,NSMCLK_90_degree,Made_By_TBCRR1); // 40 HZ
    StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
    StopTimerA();


}

void calculate_voltage(int index){
    int volt = 0,temp=0,resdiue;
    int* p_resdiue;
    disable_interrupts();
    p_resdiue = &resdiue;
    temp = Lihgt_Means[index]*33;
    volt = temp/40950;
    temp = mult(Lihgt_Means[index],33);
    volt = div(temp,40950,p_resdiue);





}

void calculate_dis_Lights(){
    unsigned long volt0,volt1,temp0,temp1,resdiue0,resdiue1;
    int i=0,j=0;
    unsigned long* p_resdiue0,* p_resdiue1;
    disable_interrupts();
    p_resdiue1 = &resdiue1;
    p_resdiue0 = &resdiue0;


    while(i<60){
        lights_LDR_MSB[i] = 0;
        lights_LDR_LSB[i] = 0;
        deg_array[i] = 0;
        if(A0results[i] == 4095 && A1results[i] == 4095){
            i++;
            continue;
        }
        temp0  = div(A0results[i],100,p_resdiue0);
        temp1  = div(A1results[i],100,p_resdiue1);
        lights_LDR_MSB[j] = temp0+temp1 ;
        lights_LDR_LSB[j] = resdiue1+resdiue0;
        i++;
        deg_array[j] = i;
        j++;

    }

    lights_LDR_MSB[j] = 0 ;
    deg_array[j] = 0;

}


void scaning_object_and_light(){
    int i, N_Angle=NSMCLK_0_degree;      //change the duty cycle for change the angle
    int k = 0;

    for(i = 0; i < 60; i++){
        N_Angle += NSMCLK_3_degree ;
        UpdatePWM(NSMCLK_40_HZ,N_Angle,Made_By_TBCRR1); // 40 HZ
        StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
        StopTimerA();
        StartSamplingADC12(i);
        StartTimerA(2048,3) ;   //0.5s delay
       // DistMeas_Echo(i);



    }
    UpdatePWM(NSMCLK_40_HZ,NSMCLK_90_degree,Made_By_TBCRR1); // 40 HZ
    StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
    StopTimerA();



}



void DMA_Transfer(){
    int i = 0;
    P1IE &= ~0x03; //disable push button interrupt until we finish period of transfer
    DMACTL0 = DMA2TSEL_10  ;                    // CCR2 trigger
    DMA2CTL = DMADT_1 + DMASRCINCR_3 + DMASWDB + DMAEN + DMAIE; // Rpt, Inc Src,  Source word to Destination byte,
    DMA2SA = (void (*)())range_array;                  // Source block address
    DMA2DA = (void (*)())&U1TXBUF;                     // Destination single address
    DMA2SZ = 0x1;
    IE2 &= ~UTXIE1;
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt

}


//*********************************************************************
//            Script Mode 5
//*********************************************************************

void MemToArr(char input[], char output[][7]){   //input will change to adress
    int input_idx = 0,output_idx=0, j = 0, i;
    for (i = 0; i < 10; i++) {
        j = 0;
        while(input[input_idx] != ','){
            output[output_idx][j] = input[input_idx++];
            j++;
        }
        output[output_idx][j] = '\0';
        input_idx++;
        output_idx++;
    }
}

void Print_Servo_Scan_LCD(unsigned int L, unsigned int R){
    char Str_L[3];
    char Str_R[3];
    char const * Str_Servo_Scan ="Servo Scan";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Servo_Scan);
    lcd_new_line;
    intToStr((int)L,Str_L);
    intToStr((int)R,Str_R);
    lcd_data(Str_L[0]);
    if(Str_L[1])
        lcd_data(Str_L[1]);
    lcd_data(',');
    lcd_data(Str_R[0]);
    if(Str_R[1])
        lcd_data(Str_R[1]);
    cursor_off();
    StartTimerA(8192,3);
    StopTimerA();
    enable_interrupts();
}

void Print_Servo_Deg_LCD(unsigned int P){
    char Str_P[4];
    char const * Str_Servo_Deg ="Servo Deg";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Servo_Deg);
    lcd_new_line;
    intToStr((int)P,Str_P);
    lcd_data(Str_P[0]);
    if(Str_P[1])
       lcd_data(Str_P[1]);
    cursor_off();
    StartTimerA(8192,3);
    StopTimerA();
    enable_interrupts();
}

void Print_Sleep_LCD(){
    char const * Str_Sleep ="MCU Go to Sleep";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Sleep);
    cursor_off();
    enable_interrupts();
    StartTimerA(8192,3);
    lcd_clear();
    lcd_home();
}

void Print_Welcome_LCD(){
    char const *Str_Names ="Roee & Noam";
    char const *Str_Project ="Project DCS";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Names);
    lcd_new_line;
    lcd_puts(Str_Project);
    cursor_off();
    StartTimerA(8192,3);
    StopTimerA();
    enable_interrupts();
    lcd_clear();
    lcd_home();
}


void Print_Delete_Files_LCD(){
    char const * Str_Delete_Files ="Delete Flash";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Delete_Files);
    cursor_off();
    StartTimerA(8192,3);
    StopTimerA();
    enable_interrupts();
    lcd_clear();
    lcd_home();
}

void Print_Clear_LCD(){
    char const * Str_Clear_LCD ="Clear LCD";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(Str_Clear_LCD);
    cursor_off();
    StartTimerA(8192,3);
    StopTimerA();
    enable_interrupts();
    lcd_clear();
    lcd_home();
}

void Servo_Scan(unsigned int L, unsigned int R ){
    Print_Servo_Scan_LCD(L,R);
    int i, L_N_Angle, R_N_Angle;      //change the duty cycle for change the angle
    int k = 0;
    L_N_Angle = mult(11,L) + 630;
    R_N_Angle = mult(11,R) + 630;
    disable_interrupts_perpihal();
    UpdatePWM(NSMCLK_40_HZ,L_N_Angle,Made_By_TBCRR1); // 40 HZ
    StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
    StopTimerA();
    while(L_N_Angle < R_N_Angle){
        StopTimerA();
        L_N_Angle += NSMCLK_3_degree ;
        UpdatePWM(NSMCLK_40_HZ,L_N_Angle,Made_By_TBCRR1); // 40 HZ
        StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
        StopTimerA();
        DistMeas_Echo(i);
    }
    UpdatePWM(NSMCLK_40_HZ,NSMCLK_90_degree,Made_By_TBCRR1); // 40 HZ
    StartTimerA(NACLK_delay_60ms,1) ;   //60ms delay
    StopTimerA();
}


void Servo_Deg(unsigned int P){
    Print_Servo_Deg_LCD(P);
    int P_N_Angle;      //change the duty cycle for change the angle
    P_N_Angle = mult(11,P) + 630;
    UpdatePWM(NSMCLK_40_HZ, P_N_Angle,Made_By_TBCRR1);
    StartTimerA(8196,3) ;   //2s delay
    StopAllTimers();
    StopTB0CTL();
    TX_Telemeter();
}



void hex_char_to_int(char c, int *P_digit) {
    if (c >= '0' && c <= '9')
        *P_digit = c - '0';
    else if (c >= 'A' && c <= 'F')
        *P_digit = c - 'A' + 10;
    else if (c >= 'a' && c <= 'f')
        *P_digit = c - 'a' + 10;
    else
        *P_digit = -1; // Invalid character
}

void hex_string_to_int(const char *hex_string, int* value){
    int result = 0;
    int len = 0;
    int i;

    while (hex_string[len] != '\0')
        len++;

    for (i = 0; i < len; i++) {
        int digit;
        int *P_digit = &digit;
        hex_char_to_int(hex_string[i], P_digit);
        if (digit == -1)
            *value = -1; // Invalid character
        result = (result << 4) | digit;
    }
    *value = result;
}

void Instraction_Decoder(const char *Hex_Instruction){
    char Opcode[3];
    Opcode[0] = Hex_Instruction[0];
    Opcode[1] = Hex_Instruction[1];
    Opcode[2] = '\0';

    if(Opcode[0] == '0' && Opcode[1] == '1'){
        Str_OP_A[0] =  Hex_Instruction[2];
        Str_OP_A[1] =  Hex_Instruction[3];
        hex_string_to_int(Str_OP_A, P_Op_A);
        Inc_LCD(Oprand_A);     // Increment LCD
        }

    else if(Opcode[0] == '0' && Opcode[1] == '2'){
        Str_OP_A[0] =  Hex_Instruction[2];
        Str_OP_A[1] =  Hex_Instruction[3];
        hex_string_to_int(Str_OP_A, P_Op_A);
        Dec_LCD(Oprand_A);     // Decrement LCD
    }

    else if(Opcode[0] == '0' && Opcode[1] == '3'){
        Str_OP_A[0] =  Hex_Instruction[2];
        Str_OP_A[1] =  Hex_Instruction[3];
        hex_string_to_int(Str_OP_A, P_Op_A);
        RRA_LCD(Oprand_A);     // Rotate Right LCD
    }
     else if(Opcode[0] == '0' && Opcode[1] == '4'){
         Str_OP_A[0] =  Hex_Instruction[2];
         Str_OP_A[1] =  Hex_Instruction[3];
         hex_string_to_int(Str_OP_A, P_Op_A);
         Set_Delay(Oprand_A);   //Set Delay
   }
     else if(Opcode[0] == '0' && Opcode[1] == '5'){
        lcd_clear();       //Clear LCD
    }
     else if(Opcode[0] == '0' && Opcode[1] == '6'){
         Str_OP_A[0] =  Hex_Instruction[2];
         Str_OP_A[1] =  Hex_Instruction[3];
         hex_string_to_int(Str_OP_A, P_Op_A);
         Servo_Deg(Oprand_A);    //Servo Deg
    }
     else if(Opcode[0] == '0' && Opcode[1] == '7'){
         Str_OP_A[0] =  Hex_Instruction[2];
         Str_OP_A[1] =  Hex_Instruction[3];
         hex_string_to_int(Str_OP_A, P_Op_A);
         Str_OP_B[0] =  Hex_Instruction[4];
         Str_OP_B[1] =  Hex_Instruction[5];
         hex_string_to_int(Str_OP_B, P_Op_B);
         Servo_Scan(Oprand_A, Oprand_B); //Servo Scan
    }
     else if(Opcode[0] == '0' && Opcode[1] == '8'){
         Print_Sleep_LCD();
         enable_interrupts();
         state = state0;
    }
}


void Script_Mod(File* P_myFile, int Num_file){
    int i;
    if(Num_file == 1)
        (*P_myFile).Address = Seg_File_A;

    else if (Num_file == 2)
        (*P_myFile).Address = Seg_File_B;

    else if (Num_file == 3)
        (*P_myFile).Address = Seg_File_C;

    Read_flash_file(P_myFile);
    MemToArr((*P_myFile).Data, (*P_myFile).Script);
    for (i = 0; i < 10; i++) {
        Instraction_Decoder((*P_myFile).Script[i]);
    }
}


//*********************************************************************
//            DMA Interrupt Service Routine
//*********************************************************************
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void){

    switch( DMAIV )
    {
    case DMAIV_DMA0IFG :
        DMA0CTL &= ~DMAIFG;
        break;
    case DMAIV_DMA1IFG :
        DMA1CTL &= ~DMAIFG;
        break;
    case DMAIV_DMA2IFG :
        DMA2CTL &= ~DMAIFG;
        IE2 |= UTXIE1;


        break;
    }
    //---------------------------------------------------------------------
    //            Exit from a given LPM
    //---------------------------------------------------------------------
            switch(lpm_mode){
            case mode0:
             LPM0_EXIT; // must be called from ISR only
             break;

            case mode1:
             LPM1_EXIT; // must be called from ISR only
             break;

            case mode2:
             LPM2_EXIT; // must be called from ISR only
             break;

                    case mode3:
             LPM3_EXIT; // must be called from ISR only
             break;

                    case mode4:
             LPM4_EXIT; // must be called from ISR only
             break;
        }

}

//******************************************************************
// Drivers to ultra sonic sensor
//******************************************************************
void PWM_Trig_Ultrasonic(){
    UpdatePWM(NSMCLK_17_HZ,NSMCLK_widthHigh_29us,Made_By_TACRR1); //the pulse is 17 HZ and 50% duty cycle
}

void DistMeas_Echo(int i){
    int k = 0;
    int res=0;
    signed short half_speed_of_sound = 17000;
    unsigned long N_ACLK;
    unsigned long *p_freq,*p_resdiue,*p_Range_cm,resdiue;
    unsigned long freq, Range_cm;
    unsigned long real_freq;
    p_freq = &freq;
    p_resdiue = &resdiue;
    p_Range_cm = &Range_cm;
    range_array[indx_for_scaning] = 0;
    StopTimerA();
    disable_interrupts();
    REdge2 = REdge1 =  0;
    StartTimerB3();
    EnableInterruptTB3();// enable interrupt + Enter LPM0
    disable_interrupts();
    if(REdge2 < REdge1){
        N_ACLK = (REdge1 - REdge2);
    }
    else{
        N_ACLK = (REdge2 - REdge1);
    }
    freq = div(1048567,N_ACLK,p_resdiue);
    real_freq =  (unsigned int) freq ;
    Range_cm = div(half_speed_of_sound,real_freq, p_resdiue);
    j++;
    if(Range_cm < dist_mask_state1 && Range_cm>0){
        Range_cm = div(Range_cm,2, p_resdiue);
        range_array[indx_for_scaning] = (Range_cm>>1) + resdiue;
        deg_array[indx_for_scaning] = i+1;
        res = (Range_cm>>1) + resdiue;
        indx_for_scaning++;
        range_array[indx_for_scaning] = 0;
        deg_array[indx_for_scaning] = 0;


    }
    StartTimerA(16384,1);   //0.5s delay
    StopTimerA();
    enable_interrupts();
}


//*********************************************************************
//            Port1 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT1_VECTOR
  __interrupt void PBs_handler(void){
   
	delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------
	if(PBsArrIntPend & PB0){
	  PBsArrIntPend &= ~PB0;

        }
        else if(PBsArrIntPend & PB1){
	  PBsArrIntPend &= ~PB1; 
        }
	else if(PBsArrIntPend & PB2){ 
	  PBsArrIntPend &= ~PB2;
        }
//---------------------------------------------------------------------
//            Exit from a given LPM 
//---------------------------------------------------------------------	
        switch(lpm_mode){
		case mode0:
		 LPM0_EXIT; // must be called from ISR only
		 break;
		 
		case mode1:
		 LPM1_EXIT; // must be called from ISR only
		 break;
		 
		case mode2:
		 LPM2_EXIT; // must be called from ISR only
		 break;
                 
                case mode3:
		 LPM3_EXIT; // must be called from ISR only
		 break;
                 
                case mode4:
		 LPM4_EXIT; // must be called from ISR only
		 break;
	}
        
}


  //-------------------------------------------------------------------------------------
  //            ADC configuration
  //-------------------------------------------------------------------------------------
  void ADC_config(void){
      ADC12CTL0 = SHT0_8 + MSC + ADC12ON;       // Turn on ADC12, use int. osc.
                                                // extend sampling time so won't
                                                // get overflow
                                                // Set MSC so conversions triggered
                                                // automatically
      ADC12CTL1 = SHP + CONSEQ_1;               // Use sampling timer, set mode
      ADC12IE = 0x02;                           // Enable ADC12IFG.1 for ADC12MEM3
      ADC12MCTL0 = INCH_0;                      // A0 goes to MEM0
      ADC12MCTL1 = EOS + INCH_1;                      // A1 goes to MEM1 , end of sequence
  }



//-------------------------------------------------------------------------------------
//           USCI configuration
//-------------------------------------------------------------------------------------


void UART_CONFING(){

  volatile unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  FLL_CTL0 |= XCAP14PF;                     // Configure load caps

    do
    {
    IFG1 &= ~OFIFG;                           // Clear OSCFault flag
    for (i = 0x47FF; i > 0; i--);             // Time for flag to set
    }
    while ((IFG1 & OFIFG));                   // OSCFault flag still set?

    P4SEL |= 0x03;                            // P4.1,0 = USART1 TXD/RXD
    ME2 |= UTXE1 + URXE1;                     // Enable USART1 TXD/RXD
    U1CTL |= CHAR;                            // 8-bit character
    U1TCTL |= SSEL0;                          // UCLK = ACLK
    U1BR0 = 0x03;                             // 32k/9600 - 3.41
    U1BR1 = 0x00;                             //
    U1MCTL = 0x4A;                            // Modulation
    U1CTL &= ~SWRST;                          // Initialize USART state machine
    IE2 |= URXIE1;                            // Enable USART1

}

void read_RXBUFF(){
    RXBUF1_var = U1RXBUF;
}

//*********************************************************************
//-------------------------------------------------------------------------------------
//           USCI ISR
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
//                           TX ISR
//-------------------------------------------------------------------------------------

#pragma vector=USART1TX_VECTOR
__interrupt void USART1_TX (void){
    //transmiter from MCU to PC. need for processing data?
   // while (!(IFG2 & UTXIFG1));
    TXBUF1 = send_mesg;

    //---------------------------------------------------------------------
     //            Exit from a given LPM
     //---------------------------------------------------------------------
         switch(lpm_mode){
             case mode0:
                 LPM0_EXIT; // must be called from ISR only
                 break;

             case mode1:
                 LPM1_EXIT; // must be called from ISR only
                 break;

             case mode2:
                 LPM2_EXIT; // must be called from ISR only
                 break;

             case mode3:
                 LPM3_EXIT; // must be called from ISR only
                 break;

             case mode4:
                 LPM4_EXIT; // must be called from ISR only
                 break;
         }

}


//-------------------------------------------------------------------------------------
//                           RX ISR
//-------------------------------------------------------------------------------------

#pragma vector=USART1RX_VECTOR
__interrupt void USART1_RX (void){
    RXBUF1_var = RXBUF1;

    if((RXBUF1_var == '0' && (!(flag_state1||flag_state2||flag_state6)) && !flag_files)){
        lcd_clear();
        state = state0;

     }
    else if(((RXBUF1_var == '1'||flag_state1)&&(!(flag_state2||flag_state6)) && !flag_files)){
        flag_state1 = 1;
        flag1++;
        dist_mask[flag1-2] = RXBUF1_var - 48;

        if(flag1 == 3){
            state = state1;
            flag1 = 0;
        }


    }
    else if(((RXBUF1_var == '2'||flag_state2) && (!(flag_state1||flag_state6)) && !flag_files)){
        flag_state2++;
        if(flag_state2==2){
            ANGLE = RXBUF1_var;
            state = state2;
        }


    }
    else if(RXBUF1_var == '3'&& (!(flag_state1||flag_state2||flag_state6)) && !flag_files){
        state = state3;

    }
    else if(RXBUF1_var == '4'&& (!(flag_state1||flag_state2||flag_state6)) && !flag_files){
        dist_mask_state1 = 50;
        state = state4;

    }

    else if(RXBUF1_var == '5' && !flag_files){
        flag_files = 1;
        state = state5;
    }
    else if (flag_files && RXBUF1_var == 'X'){
        File_Select = File_Sel_A;
    }

    else if (flag_files && RXBUF1_var == 'x'){
        state = state5;
        flag_write_file_A = 1;
        flag_files = 0;
    }
    else if (RXBUF1_var == 'i' && flag_files){
        state = state5;
        flag_exe_file_A = 1;
    }

    else if (flag_files && RXBUF1_var == 'Y'){
        File_Select = File_Sel_B;
    }

    else if (flag_files && RXBUF1_var == 'y'){
        state = state5;
        flag_write_file_B = 1;
        flag_files = 0;
    }
    else if (RXBUF1_var == 'j'){
        state = state5;
        flag_exe_file_B = 1;
    }
    else if (flag_files && RXBUF1_var == 'Z'){
        File_Select = File_Sel_C;
    }
    else if (flag_files && RXBUF1_var == 'z'){
        state = state5;
        flag_write_file_C = 1;
        flag_files = 0;
    }
    else if (RXBUF1_var == 'k' && flag_files){
        state = state5;
        flag_exe_file_C = 1;
    }

    else if (RXBUF1_var == 'G' && flag_files){
        File_Select = None;
        state = state5;
        flag_Clear_Flush = 1;
    }

    else if(flag_files){
        switch(File_Select){
            case None:
                break;
            case File_Sel_A:
                File_A.Data[k_A] = RXBUF1_var;
                k_A+=1;
                break;

            case File_Sel_B:
                File_B.Data[k_B] = RXBUF1_var;
                k_B+=1;
                break;

            case File_Sel_C:
                File_C.Data[k_C] = RXBUF1_var;
                k_C+=1;
                break;

        }
    }
    else if(flag1){
        dist_mask[flag1-1] = RXBUF1_var - 48;
    }
    else if ((RXBUF1_var == '6'&& (!(flag_state1||flag_state2||flag_files)))  || flag_state6==1) {
            if(flag_state6==1){
                state = state6;
            }
            if (RXBUF1_var == 'y'){

                flag_Calibaration = 1;
            }

            else if(RXBUF1_var == 'n'){
                flag_Calibaration = 0;
                flag_state6 = 0;
            }



        flag_state6 = 1;
        read_RXBUFF();

    }


 //---------------------------------------------------------------------
 //            Exit from a given LPM
 //---------------------------------------------------------------------
             switch(lpm_mode){
             case mode0:
                 LPM0_EXIT; // must be called from ISR only
                 break;

             case mode1:
                 LPM1_EXIT; // must be called from ISR only
                 break;

             case mode2:
                 LPM2_EXIT; // must be called from ISR only
                 break;

             case mode3:
                 LPM3_EXIT; // must be called from ISR only
                 break;

             case mode4:
                 LPM4_EXIT; // must be called from ISR only
                 break;
             }
}

//*********************************************************************
//            Port2 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler_P2(void){
      delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------

          PB3sArrIntPend = 0x00;

//---------------------------------------------------------------------
//            Exit from a given LPM
//---------------------------------------------------------------------
      switch(lpm_mode){
      case mode0:
          LPM0_EXIT; // must be called from ISR only
          break;

      case mode1:
          LPM1_EXIT; // must be called from ISR only
          break;

      case mode2:
          LPM2_EXIT; // must be called from ISR only
          break;

      case mode3:
          LPM3_EXIT; // must be called from ISR only
          break;

      case mode4:
          LPM4_EXIT; // must be called from ISR only
          break;
      }
  }
  //*********************************************************************
  //            TimerB Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_B1_VECTOR
  __interrupt void TIMER0_B1_ISR(void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_VECTOR (void)
  #else
  #error Compiler not supported!
  #endif
  {
    switch(__even_in_range(TBIV, 0x0A))
    {

        case  TBIV_NONE: break;              // Vector  0:  No interrupt
        case  TBIV_TBCCR1:                   // Vector  2:  TBCCR1 CCIFG
            TBCTL &= ~(TBIFG);
          break;
        case TBIV_TBCCR2 :

           // Rising Edge was captured
           if (!Count)
           {
               REdge1 = TBCCR2;
               Count++;
           }
           else
           {
               REdge2 = TBCCR2;
               TBCCTL2 &= ~CCIE;
               Count=0x0;
               __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to api
           }



            break;                  // Vector  4:  Reserved CCIFG

        case TBIV_TBCCR3:                    // Vector  6:  TBCCR2 CCIFG

           break;
        case TBIV_TBCCR4: break;                  // Vector  8:  Reserved CCIFG
        case TBIV_TBIFG:
                TBCTL &= ~(TBIFG);
                break;              // Vector 10:  TBIFG
        default:  break;
    }
    TBCTL &= ~(TBIFG);
  }
  //*********************************************************************
  //            TimerA0 Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_A0_VECTOR
  __interrupt void Timer_A (void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
  #else
  #error Compiler not supported!
  #endif
  {



      LPM0_EXIT;
      TACTL &= ~(TAIFG);
      TACCTL0 &= ~CCIE;
  }
  //*********************************************************************
  //            TimerA1-2 Interrupt Service Routine
  //*********************************************************************
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
  #pragma vector = TIMER0_A1_VECTOR
  __interrupt void Timer_A1 (void)
  #elif defined(__GNUC__)
  void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
  #else
  #error Compiler not supported!
  #endif
  {
      switch(__even_in_range(TAIV, 0x0A))
         {

             case  TAIV_NONE: break;              // Vector  0:  No interrupt
             case  TAIV_TACCR1:                   // Vector  2:  TACCR1 CCIFG
                 TACTL &= ~(TAIFG);
               break;
             case TAIV_TACCR2:                    // Vector  4:  TACCR2 CCIFG

                 break;

             case TAIV_TAIFG:
                     TACTL &= ~(TAIFG);
                     break;              // Vector 10:  TBIFG
             default:  break;
         }

       }
  //*********************************************************************
  //            ADC12 Vector Interrupt Service Routine
  //*********************************************************************
  #pragma vector = ADC12_VECTOR
  __interrupt void ADC12_ISR(void)
  {
      A0results[indx] = ADC12MEM0;             // Move A0 results, IFG is cleared
      A1results[indx] = ADC12MEM1;             // Move A1 results, IFG is cleared
      __no_operation();                         // SET BREAKPOINT HERE
      __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
  }

