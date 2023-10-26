#include  "../header/api.h"         // private library - API layer
#include  "../header/app.h"         // private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;
enum File_Mode File_Select;


void main(void){
    int i,k;
    int  Counter=0;
    int* P_Counter = &Counter;
    int flag_Calibaration=1;
    disable_interrupts();
    sysConfig();
    File_Select = None;
    state = state0;  // start in idle state on RESET
    lpm_mode = mode0;     // start in idle state on RESET
    lcd_init();
    lcd_clear();
    enable_interrupts();
    Print_Welcome_LCD();

  while(1){
    switch(state){
      case state0:                  //char '0' from GUI
          read_RXBUFF();
          disable_interrupts_exept(0);
          enable_interrupts();
          enterLPM(lpm_mode);
          //flag_Calibaration = get_flag_Calibaration();
         // if(flag_Calibaration == 'y'){
             // Light_Detector_Calibaration();
         // }
          break;

      case state1:                 //char '1' from GUI
          Objects_Detector();     //1) Objects Detector System
          state = state0;
          break;

      case state2:                //char '2' from GUI
          Telemeter();      //2) Telemeter
          state = state0;
          break;

      case state3:            //char '3' from GUI
         Light_Detector();     //3) Light Sources Detector System
         state = state0;
        break;

      case state4: //char '4' from GUI
          Light_and_Objects(); //4) Light Sources and Objects Detector System
          state = state0;
          break;

      case state5: //char '5' from GUI
          Script_Mode(); //5) Script Mode
          break;

      case state6: //char '6' from GUI
          Light_Detector_Calibaration();
          lcd_clear();
          state = state0;
          //clear_Flush();
          //Write_Calibration_Flush();
         //Read_Calibration_Flush();
          break;

    }
  }
}

  
  
  
  
  
  
