#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"

// Global Variables
unsigned int REdge1, REdge2;
unsigned int d_glob = 50;
unsigned int *P_d =&d_glob;


//-------------------------------------------------------------
//                    Objects Detector State 1
//-------------------------------------------------------------

void Objects_Detector(){
    RX_BYTE();
    scaning_Objects();
    TX_ARRAY();
    reset_state_flag(1);
    StopAllTimers();
    Py_Process_Data();
}

//-------------------------------------------------------------
//                    Telemeter State 2
//-------------------------------------------------------------

void Telemeter(){
    int Angle, N_Angle=0, *p_r,temp;
    Angle = GET_ANGLE();
    reset_state_flag(2);
    temp = mult(Angle,10);
    N_Angle = 630 + temp;
    UpdatePWM(NSMCLK_40_HZ, N_Angle,Made_By_TBCRR1);
    StartTimerA(8196,3) ;   //2s delay
    StopAllTimers();
    TX_Telemeter();

}

//-------------------------------------------------------------
//                    Light Detector State 3
//-------------------------------------------------------------

void Light_Detector(){
    int volt1;
    scaning_Lights();
    calculate_dis_Lights();
    TX_ARRAY();
    Py_Process_Data();

}

//-------------------------------------------------------------
//                    Light_and_Objects State 4
//-------------------------------------------------------------
void Light_and_Objects(){
    scaning_object_and_light();
    calculate_dis_Lights();
    TX_STATE4();
    Py_Process_Data();
}


//-------------------------------------------------------------
//                    Script Mode State 5
//-------------------------------------------------------------

void Script_Mode(){
    while(state == state5){
        if(flag_write_file_A){
            Write_File(&File_A,1);
            flag_write_file_A = 0;
            break;
        }

        else if(flag_exe_file_A){
            Script_Mod(&File_A,1);
            flag_exe_file_A = 0;
            break;
        }

        else if(flag_write_file_B){
            Write_File(&File_B,2);
            flag_write_file_B = 0;
            break;
        }

        else if(flag_exe_file_B){
            Script_Mod(&File_B,2);
            flag_exe_file_B = 0;
            break;
        }

        else if(flag_write_file_C){
            Write_File(&File_C,3);
            flag_write_file_C= 0;
            break;
        }
        else if(flag_exe_file_C){
            Script_Mod(&File_C,3);
            flag_exe_file_C = 0;
            break;
        }
        else if(flag_Clear_Flush){
            Clear_Flash();
            Print_Delete_Files_LCD();
            break;
        }
    }
    state = state0;
}


void Inc_LCD(unsigned int X){
    char const * StrShowCounter ="Script Counter:";
    char Str_CNT[16];
    int CNT = 0;
    lcd_clear();
    lcd_home();
    lcd_puts(StrShowCounter);
    StartTimerA(1024,3);
    while(CNT < X + 1){
       disable_interrupts();
       lcd_new_line;
       intToStr(CNT,Str_CNT);
       lcd_puts(Str_CNT);
       cursor_off();
       enable_interrupts();
       StartTimerA(d_glob,4);
       CNT++;
   }
}

void RRA_LCD(char X){
    int Pixel = 0,i, Cruser_R = 0, flag_new_line =0;
    while(Pixel < 32){
        lcd_home();
        lcd_clear();
        if(Pixel > 15 ){
            lcd_new_line;
        }
        if(Pixel == 16 && !flag_new_line){
            Cruser_R -= 16;
        }
        for(i = 0; i < Cruser_R; i++ ){
            lcd_cursor_right();
        }
        lcd_putchar(X);
        cursor_off();
        StartTimerA(d_glob,4);
        Pixel++;
        Cruser_R++;
    }

}

void Dec_LCD(unsigned int X){
    char const * StrShowCounter ="Script Counter:";
    char Str_X[16];
    lcd_clear();
    lcd_home();
    lcd_puts(StrShowCounter);
    StartTimerA(1024,3);
    while(X + 1 > 0){
       if(X  == 9){
           lcd_clear();
           lcd_home();
           lcd_puts(StrShowCounter);
       }
       disable_interrupts();
       lcd_new_line;
       intToStr(X,Str_X);
       lcd_puts(Str_X);
       cursor_off();
       enable_interrupts();
       StartTimerA(d_glob,4);
       X--;
   }
}

void Set_Delay(unsigned int delay){
    char Str_d[16];
    const char Ms[] = "[10Ms]";
    char const * StrShowCounter ="Delay set to:";
    disable_interrupts();
    lcd_clear();
    lcd_home();
    lcd_puts(StrShowCounter);
    lcd_new_line;
    intToStr(delay,Str_d);
    lcd_puts(Str_d);
    lcd_cursor_right();
    lcd_puts(Ms);
    cursor_off();
    enable_interrupts();
    *P_d = delay;
    StartTimerA(8192,3);
}



