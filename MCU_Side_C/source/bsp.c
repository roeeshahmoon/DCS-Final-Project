#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------
void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  // LCD configuration
  LCD_DATA_WRITE &= ~0xFF;
  LCD_DATA_DIR |= 0xF0;    // P8.4-P8.7 To Output('1')
  LCD_DATA_SEL &= ~0xF0;   // Bit clear P8.4-P8.7
  LCD_CTL_SEL  &= ~0xE0;   // Bit clear P2.5-P2.7

  // LED configuration
  LEDsArrPortSel &= ~0xFF;
  LEDsArrPort  &= ~0xFF;
  LEDsArrPortDir |= 0xFF;
  
  //Setup ServoMotor PWM P2.2 -> TB1
  ServoMotor_PortDir |= BIT2;             // P2.2 Output  - '1'
  ServoMotor_PortSel |= BIT2;             // P2.2 Select = '1'
  //ServoMotor_PortOut &= ~BIT2;            // P2.2 out = '0'

  // Setup UltraSonic_Trigger PULSE P1.2  -> TA1
  UltraSonic_Trigger_PortDir |= BIT2 ;       // set P1.2  to Output
  UltraSonic_Trigger_PortSel |= BIT2;       // P1.2  Select = '0'
  UltraSonic_Trigger_PortOut &= ~BIT2;      // P1.2  out = '0'

  // Setup UltraSonic_Echo Input Capture P2.3  -> TB2
  UltraSonic_Echo_PortDir &=  ~BIT3;             // P2.3 Input Capture = '0'
  UltraSonic_Echo_PortSel |=  BIT3;              // P2.3 Select = '1'

  // Setup LDR One
     LDR_One_PortSel |= BIT0;
     LDR_One_PortDir &= ~BIT0;

  // Setup LDR Two
     LDR_Two_PortSel |= BIT1;
     LDR_Two_PortDir &= ~BIT1;
    // P6SEL |= BIT0 + BIT1;                            // Enable A/D inputs

    // PushButtons Setup
 PBsArrPortSel &= ~BIT0;           //
 PBsArrPortOut &= ~BIT0;            // Set P1Out to '0'
 PBsArrPortDir &= ~BIT0;            // P1.0 - Input ('0')
 PBsArrIntEdgeSel |= BIT0;         // pull-up mode   P1.0-- '1'
 PBsArrIntEn |= BIT0;              // P1.0 - '1'
 PBsArrIntPend &= ~BIT0;            // clear pending interrupts P1.0-P1.3 all P1



// RGB P1.0-P1.2 setup

  // RGBPortSel &= ~0x07; // set P1.0 - P1.2 to I/O
   //RGBPortDir |= 0x07;// set P1.0 - P1.2 to Output
 //  RGBPortOut &= ~0x07; // Reset P1.0 - P1.2 Output

// UART INIT
   //P2DIR = 0xFF;                             // All P2.x outputs
  // P2OUT = 0;                                // All P2.x reset
  // P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  // P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  // P1DIR |= RXLED + TXLED;
 //  P1OUT &= 0x00;

  _BIS_SR(GIE);                     // enable interrupts globally


}




