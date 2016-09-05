/*
This program is writen by J.Teda for sliding/pan/tilt, real time and time lapse videos

27/06/2012 EOS SUB control function removed as it is not going to be used in Hardware V2

03/06/2012 Edited to work with V2 Dynamixel_Serial library

18/04/2012 Added Pan and tilt control to realtime

!! Timer1 used for Software Serial control
!! Timer2 used for Shutter interval trigger

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.


*/

#define noDEBUG                // Change this to DEBUG if needed

#include <MsTimer2.h>          // Libray for Timer interrupt 2 used for timelapse mode shutter release trigger, NOTE:timer 1 is used in Software Serial so we must use timer 2 for shutter relase

#include <Dynamixel_Serial.h>  // Libray to control Dynamixel MX-28
#include <SoftwareSerial.h>    // Software Serial to contorl/talk RN-42 (BlueTooth)

#define MX28_ControlPin 0x10   // TX/RX control pin for full to half duplex control
#define MX28_ID_All    0xFE    // ID for broadcast to all Dynamixel MX-28( talk to servos at onces ) NOTE: this is the Dynamixel number as per the robotis manual and can not be changed
#define MX28_ID_slide  0x01    // slide ID of Dynamixel MX-28  ( B00000001 )
#define MX28_ID_pan    0x02    // pan ID of Dynamixel MX-28    ( B00000010 )
#define MX28_ID_tilt   0x04    // tilt ID of Dynamixel MX-28   ( B00000100 )
#define MX28_ID_focus  0x08    // focus ID of Dynamixel MX-28  ( B00001000 )
#define MX28_Baudrate 1000000  // Comms Speed TX/RX of Dynamixel MX-28, this speed is not the Dynamixel default speed and MUST be changed on the Dynamixal if you use this speed
#define MX28_TORQUE_DEFALULT 0x3FF // Default holding torque of Dynamixel MX-28
#define MX28_RW_Delay 0x100

#define Limit_Left_Pin 0x03    // Must be pin 3 for hardware interrupt 1
#define Limit_Right_Pin 0x02   // Must be pin 2 for hardware interrupt 0
#define Status_LED 0x06        // LED on swtich

#define BT_Rx 0x05            // Recive data from BlueTooth  
#define BT_Tx 0x04            // Transmit data to BlueTooth

#define Wireless_Cap_Pin  0x0E // wired remote shutter/capture relay pin
#define Wireless_Foc_Pin  0x0F // wired remote focus relay pin  

                               // TL_Flag define bits (timelaspe)
#define Timelapse_On            B00000001    // Byte 0
#define Timelapse_Off           B00000000    // Byte 0
#define Step_Mode_True          B00000010    // Byte 1
#define Step_Mode_False         B00000000    // Byte 1
#define Focus_Mode_True         B00000100    // Byte 2
#define Focus_Mode_False        B00000000    // Byte 2
#define TimeLapse_Release_HIGH  B00010000    // Byte 4
#define TimeLapse_Release_LOW   B00100000    // Byte 5
#define TimeLapse_Servo_Move    B01000000    // Byte 6
#define TimeLapse_Servo_Stop    B10000000    // Byte 7
#define TimeLapse_Continues     B11110000    // Byte 4 & 5 & 6 & 7

                                //KF_Flag define bits (Keyfaming)
#define Keyframe_Home           B00000001
#define Keyframe_Moving_Home    B00000010
#define Keyframe_Begin          B00000100
#define Keyframe_End            B00001000
                                
                                //MX28_DIR_Flag define bits                              
#define MX28_DIR_Slide_Left     B00000000    // Byte 0
#define MX28_DIR_Slide_Right    B00000001    // Byte 0
#define MX28_DIR_Pan_Clock      B00000000    // Byte 1
#define MX28_DIR_Pan_AntiClock  B00000010    // Byte 1
#define MX28_DIR_Tilt_Up        B00000000    // Byte 2
#define MX28_DIR_Tilt_Down      B00000100    // Byte 2

#define MX28_DIR_Slide_True     B00010000    // Byte 4 
#define MX28_DIR_Slide_False    B00000000    // Byte 4
#define MX28_DIR_Pan_True       B00100000    // Byte 5 
#define MX28_DIR_Pan_False      B00000000    // Byte 5
#define MX28_DIR_Tilt_True      B01000000    // Byte 6
#define MX28_DIR_Tilt_False     B00000000    // Byte 6


#define TimeLapse_Release_HOLD 120       // Time between setting wired shutter release HIGH to LOW, in mills()    

#define Focus_Step_I            B00000001
#define Focus_Step_II           B00000010
#define Focus_Step_III          B00000100

#define TL_Pre_Delay_Min 15    // Min dealy between stopping servo and shutter release, in mills()
#define TL_Interval_Min 170    // Max shoots pre sec on canon 7D is 6, there for 1000/6 = 166 is the min time between shoots, in mills()

unsigned char MX28_Temp;
unsigned char MX28_Volt;
unsigned int MX28_Read_Speeds;
unsigned int MX28_Load;
unsigned int MX28_Pos;
//unsigned int  MX28_Speed;
unsigned int MX28_TL_speeds[4]; // array to hold MX28 speeds when in Timelapse 0 = Slide, 1 = Pan, 2 = Tilt, 3 = Focus

byte MX28_ID;

unsigned long TL_Timer;
unsigned long TL_Delay_previousMillis;
unsigned int  TL_Interval;
unsigned int  TL_PostDelay;
unsigned int  TL_Pre_Delay;
unsigned int  TL_Shot_Value;
unsigned int  TL_Shot_Count;
byte  TL_Flag;
byte  TL_Focus_Step;
byte MX28_DIR_Flag;

byte KF_Flag;
unsigned int  KF_Slide_Begin;
unsigned int  KF_Pan_Begin;
unsigned int  KF_Tilt_Begin;
unsigned int  KF_Slide_End;
unsigned int  KF_Pan_End;
unsigned int  KF_Tilt_End;

boolean RT_Capture;
boolean Limit_Active;
boolean MX28_LED_State;

SoftwareSerial BT_Serial(BT_Rx,BT_Tx); // Setup Blue tooth software serial.

//###################################### Setup
void setup(){
  
   pinMode(Limit_Left_Pin, INPUT); 
   pinMode(Limit_Right_Pin, INPUT);
   pinMode(Wireless_Cap_Pin, OUTPUT);
   pinMode(Wireless_Foc_Pin, OUTPUT); 
   pinMode(Status_LED, OUTPUT);
   
   attachInterrupt(0,Limit_Right, RISING);            // Setup limit switch to be hardware interrups 0
   attachInterrupt(1,Limit_Left, RISING);             // Setup limit switch to be hardware interrups 1
   
   BT_Serial.begin(57600);                                // Set Bluetooth BaudRate to default baud rate 57600
   Dynamixel.begin(MX28_Baudrate,MX28_ControlPin,100);    // Set Arduino Serial to Baudrate for Dynamxel  
   
   TL_Flag = 0x00;
   KF_Flag = 0x00; 
   TL_Focus_Step = 0x00;  
   TL_Interval = TL_Interval_Min;
   TL_Pre_Delay = TL_Pre_Delay_Min;
   TL_Shot_Value = 0x00;
   TL_Shot_Count = 0x00;   
   KF_Slide_Begin = 0x00;
   KF_Pan_Begin = 0x00;
   KF_Tilt_Begin = 0x00;
   KF_Slide_End = 0x00;
   KF_Pan_End = 0x00;
   KF_Tilt_End = 0x00;
   Limit_Active = false;
   
   MX28_Setup();
   
   digitalWrite (Wireless_Cap_Pin, LOW);
   digitalWrite (Wireless_Foc_Pin, LOW);

   digitalWrite(Status_LED, HIGH);                    // Turn Status LED to on when startup is finsihed        
  
   #ifdef DEBUG
     BT_Serial.println("Statup Finish");
   #endif
       
}

//###################################### Main Loop


void loop(){  
 
 if(Limit_Active == true){
      #ifdef DEBUG
      BT_Serial.println("STOP!");
      #endif
    Limit_Active = false;
    TL_Flag &= B1110;                                  // Stop Timelapse if active
    Dynamixel.setHoldingTorque(MX28_ID_All, OFF);              // Disable holding torque and stop servo
    delay(1);
    Dynamixel.ledState(MX28_ID_All, OFF);
    MX28_LED_State = false; 
  } 
   
BlueToothRead();

switch (TL_Flag & B00000001){                        // Mask bits and check if in Timelapse or real time mode.
  case Timelapse_On:
  TimeLapseMode();
  break;
  case Timelapse_Off:
  RealTimeMode();
  break;
  }  
}

//###################################### Subs

void Limit_Left(){                                       // Left Limit switch on slider, keep interrupts to minimal code

  #ifdef DEBUG
  BT_Serial.println("Left Limit ACTIVE");
    if (digitalRead(Limit_Left_Pin) == HIGH){
    BT_Serial.println("Left Limit HIGH");
      }else if (digitalRead(Limit_Right_Pin) == HIGH) {  
      BT_Serial.println("RIght Limit HIGH");
      }
  #endif 
  
  Limit_Active = true;			 
}

void Limit_Right(){                                        // Right Limit switch on slider, keep interrupts to minimal code
  #ifdef DEBUG
    BT_Serial.println("Right Limit ACTIVE");
      if (digitalRead(Limit_Left_Pin) == HIGH){
      BT_Serial.println("Left Limit HIGH");
        }else if (digitalRead(Limit_Right_Pin) == HIGH) {  
        BT_Serial.println("RIght Limit HIGH");
         }
  #endif
  
  Limit_Active = true;			  
}

void MX28_Setup(){
  
  Dynamixel.setStatusPaket(MX28_ID_All,READ);                      // Tell Dynamixel only to return read status packets, this is done to limit the about of data on the Half duplex line.
  delay(10);
  Dynamixel.setStatusPaketReturnDelay(MX28_ID_All,50);             // Tell Dynamixel when it does send a status packet return it  50 millis() affer instuction packet
   delay(10);
   Dynamixel.setMode(MX28_ID_All, WHEEL, 0, 0);                    // Set to wheel mode to all servos
   delay(10);
   Dynamixel.setHoldingTorque(MX28_ID_All, OFF);                   // Disable Dynamixel holding torque
   delay(10);
   Dynamixel.setAlarmShutdown(MX28_ID_All, 0x7F);                  // Set all Alarms to active on Dynamixel
  
}

void MX28_Status(){  
//  Dynamixel.readTemperature(MX28_ID);                       // Read MX28 temperature
//  Dynamixel.readVoltage(MX28_ID);                           // Read MX28 voltage 
//  Dynamixel.readSpeed(MX28_ID);                             // Read MX28 present speed
//  Dynamixel.readPosition(MX28_ID);                          // Read MX28 present position
//  Dynamixel.readLoad(MX28_ID);                              // Read MX28 Load and directional force of load on servo
}

void RealTimeMode(){
   if(KF_Flag & B00000001 == Keyframe_Home){                 // Check if servos needs to move home  
    KF_Flag &= B11111110;                                    // Clear home flag
    KF_Flag |= B00000010;                                    // Set moving home flag
//    Dynamixel.setMode(MX28_ID_All, SERVO, 0x001, 0xFFF);                      // Set to servo mode for all servos
//    delay(10);
//    Dynamixel.servo(MX28_ID_pan, KF_Pan_Begin, 0x3FF);    // Send home angle to servo (Pan)
//    delay(10);
//    Dynamixel.servo(MX28_ID_tilt, KF_Tilt_Begin, 0x3FF);  // Send home angle to servo (Tilt)
     
   }else if ( KF_Flag & B00000010 == Keyframe_Moving_Home){
       if (Dynamixel.checkMovement(MX28_ID_pan) == 0 & Dynamixel.checkMovement(MX28_ID_tilt) == 0){ // check to see if the servo is still moving, 0 = completed , 1 = still moving
        Dynamixel.setMode(MX28_ID_All, WHEEL,0,0);          // Set to wheel mode to all servos
        KF_Flag &= B11111101;                               // Clear moving home flag
       }
     
   }
  
  
   if (RT_Capture == true){ 
    RT_Capture = false;

            digitalWrite(Wireless_Foc_Pin, HIGH);
            digitalWrite(Wireless_Cap_Pin, HIGH);
            delay(110); 
            digitalWrite(Wireless_Foc_Pin, LOW);
            digitalWrite(Wireless_Cap_Pin, LOW);
  }
}



void TimeLapseMode(){ 
  
  long currentMillis= millis();

  if (currentMillis - TL_Delay_previousMillis >= TL_Timer){
   
  TL_Delay_previousMillis = currentMillis; 

  switch (TL_Flag & B11110000){                                     // Switch between timelapse condistions
    case TimeLapse_Release_HIGH:                                               
     #ifdef DEBUG
     BT_Serial.println("TimeLapse_Shoot");
     #endif  

      switch (TL_Flag & B00000010){                                 // check if in "Step" mode
          case Step_Mode_True:
           TL_Timer = TL_PostDelay;
           TL_Flag &= B00001111;                                    // Clear flags
           TL_Flag |= TimeLapse_Servo_Move;                         // Set stop flag to write servo speed and direction
         break;
         case Step_Mode_False:                                      // Else run in deflaut "contiues mode"
           TL_Timer = TL_Interval;
           TL_Flag &= B00001111;                                    // Clear flags 
           TL_Flag |= TimeLapse_Release_LOW;                        // Set Shutter release flag to LOW(Off)            
         break;
   }
  
     digitalWrite(Wireless_Foc_Pin, LOW);
     digitalWrite(Wireless_Cap_Pin, LOW); 


    
    if (TL_Shot_Value != TL_Shot_Count && TL_Shot_Value != 0x00){    // Duration couter, cout how many shots taken if not zero
         TL_Shot_Count ++;
         BT_Serial.print("$td");
         BT_Serial.print(TL_Shot_Count); 
         BT_Serial.print("!");                                       // Send Shot count to phone
//         BT_Serial.print("$td" + TL_Shot_Count + "!"); 
      }else if (TL_Shot_Value == TL_Shot_Count && TL_Shot_Value != 0x00){        
            MsTimer2::stop();
            TL_Flag &= B00001110;                                    // Clear timelapse flags 
            Dynamixel.wheel(MX28_ID_All,LEFT,0x00);      
            delay(1);
            Dynamixel.ledState(MX28_ID_All, OFF); 
            MX28_LED_State = false;
            }
         
    break;
    
    case TimeLapse_Servo_Move :
    #ifdef DEBUG
    BT_Serial.println("TimeLapse_Servo_Move");
    #endif 
    
    TL_Timer = ((TL_Interval - TL_PostDelay) - TL_Pre_Delay);               // Load timer with new value  
 
    Dynamixel.wheelSync(MX28_ID_slide, (MX28_DIR_Flag & B00000001), MX28_TL_speeds[0], MX28_ID_pan, ((MX28_DIR_Flag >> 1) & B00000001), MX28_TL_speeds[1], MX28_ID_tilt, ((MX28_DIR_Flag >> 2) & B00000001), MX28_TL_speeds[2]); // write to slide/pan/tilt in one go.
    
        TL_Flag &= B00001111;                                                // Clear flags
        TL_Flag |= TimeLapse_Servo_Stop;                                     // Set Stop flag

    break;
    
    case TimeLapse_Servo_Stop:
    #ifdef DEBUG
    BT_Serial.println("TimeLapse_Servo_Stop");
    #endif     
    
    TL_Timer = TL_Interval;                                                  // Load timer with new value 
    
    Dynamixel.wheel(MX28_ID_All,LEFT,0x00);                                  // Stop all servos and hold present pos
    
        TL_Flag &= B00001111;                                                // Clear flags
        TL_Flag |= TimeLapse_Release_LOW;                                    // Set Shot flag
        
    break;
    
    case TimeLapse_Continues:
    #ifdef DEBUG
    BT_Serial.println("TimeLapse_Continues");
    #endif     
    
    Dynamixel.wheelSync(MX28_ID_slide, (MX28_DIR_Flag & B00000001), MX28_TL_speeds[0], MX28_ID_pan, ((MX28_DIR_Flag >> 1) & B00000001), MX28_TL_speeds[1], MX28_ID_tilt, ((MX28_DIR_Flag >> 2) & B00000001), MX28_TL_speeds[2]); // write to slide/pan/tilt in one go.
    
    TL_Flag &= B00001111;                                                // Clear flags
    TL_Flag |= TimeLapse_Release_LOW;                                    // Set Shot flag       
        
    break;

  }
 }
}

void Shutter_Interrupt(){                                                  // Keep this section to min and don't used delay as it will lockup program when runnung inside a interrupt
  TL_Delay_previousMillis = millis(); 
  TL_Timer = TimeLapse_Release_HOLD; 
  TL_Flag &= B00001111;                                                   // Clear flags
  TL_Flag |= TimeLapse_Release_HIGH;                                      // Set shutter release pin HIGH flag 
  
      digitalWrite(Wireless_Foc_Pin, HIGH);
      digitalWrite(Wireless_Cap_Pin, HIGH);
  
}


void BlueToothRead(){
char queryType;  
     
  if(BT_Serial.available() > 1 ) { 
    if ( BT_Serial.read() == '$'){                                //"header" data
      queryType = BT_Serial.read();
         switch(queryType){
         // ***  Real Time Mode ***  
         case 'r':
                 queryType = BT_Serial.read();
                     switch(queryType){
                     case'm': 
                     
                      queryType = BT_Serial.read(); // Select which servo to move, Slide = "s"/Pan = "p" /Tilt = "t"
                      if (queryType == 's'){
                      MX28_ID = MX28_ID_slide;  
                        }else if (queryType == 'p'){
                        MX28_ID = MX28_ID_pan;  
                          }else if (queryType == 't'){
                          MX28_ID = MX28_ID_tilt;  
                          }else{
                            break;
                            }
                     
                      queryType = BT_Serial.read();
//                    if (digitalRead(Limit_Left_Pin) != HIGH && queryType != LEFT){
                        Dynamixel.wheel(MX28_ID, queryType, readFloatSerial());
//                    }else if (digitalRead(Limit_Right_Pin) != HIGH && queryType != RIGHT){
//                        Dynamixel.wheel(MX28_ID, queryType, readFloatSerial());
//                          }  

                      if (MX28_LED_State == false){
                        delay(1);
                        Dynamixel.ledState(MX28_ID, ON);
                        MX28_LED_State = true;
                      }
                      break; 
                      case 'c':

                      break; 
                      case 'f':
                      queryType = BT_Serial.read();

                           switch(queryType){
                             //Focus IN
                             case '+':
                             queryType = readFloatSerial();
                                switch(queryType){
                                  case Focus_Step_I:
                                  
                                  break;
                                  case Focus_Step_II:
                                  
                                  break;
                                  case Focus_Step_III:
                                  
                                  break; 
                                }
                             break;  
                             // Focus OUT                           
                             case '-':
                             queryType = readFloatSerial();
                                switch(queryType){
                                  case Focus_Step_I:
                                  
                                  break;
                                  case Focus_Step_II:
                                  
                                  break;
                                  case Focus_Step_III:
                                  
                                  break; 
                                }
                             break;
                           }

                      break;                        
                      case 'h':                         
                          Dynamixel.setHoldingTorque(MX28_ID_All, readFloatSerial());                     // Enable holding torque 
                      break;
                        
                      }
                      
          
         break;
         
        // *** Timelapse Mode ***         
         case 't':                                                  
                  queryType = BT_Serial.read();
                     switch(queryType){
                     case'm':
                     
                      queryType = BT_Serial.read(); // Select which servo to move, Slide = "s"/Pan = "p" /Tilt = "t"
                        switch(queryType){
                        case 's':                        
                        MX28_DIR_Flag |= MX28_DIR_Slide_True ;   // Set Slide flag                       
                        queryType = BT_Serial.read();
                          if (queryType == 'l'){                            
                          MX28_DIR_Flag &= B11111110;            // clear move Right flag                   
                          }else if(queryType == 'r'){
                          MX28_DIR_Flag |= B00000001;            // Set move Right flag  
                          } 
                        MX28_TL_speeds[0] = readFloatSerial();
                        break;
                        
                        case 'p':
                        MX28_DIR_Flag |= MX28_DIR_Pan_True ;    // Set Pan flag                       
                        queryType = BT_Serial.read();
                        if (queryType == 'l'){                            
                          MX28_DIR_Flag &= B11111101;           // clear move Clockwise flag                    
                          }else if(queryType == 'r'){
                          MX28_DIR_Flag |= B00000010;           // Set move Anti-Clockwise flag  
                        }                         
                        MX28_TL_speeds[1] = readFloatSerial();
                        break;
                        
                        case 't':
                        MX28_DIR_Flag |= MX28_DIR_Tilt_True ;  // Set Pan flag                       
                          queryType = BT_Serial.read();
                          if (queryType == 'l'){                            
                            MX28_DIR_Flag &= B11111011;        // clear move Clockwise flag                    
                            }else if(queryType == 'r'){
                            MX28_DIR_Flag |= B00000100;        // Set move Anti-Clockwise flag  
                          }                          
                        MX28_TL_speeds[2] = readFloatSerial();
                        break;
                        }  
                            
          
                     break;
                     case'i':
                     TL_Interval = readFloatSerial();
                       if (TL_Interval <= TL_Interval_Min){ 
                       TL_Interval = TL_Interval_Min;  
                       }
                     MsTimer2::set(TL_Interval, Shutter_Interrupt);    // Set interrupt timer 2 to interval durration and attach overflow event to "shutter_interrupt"
                     
                     break;
                     case'p':
                     TL_Pre_Delay = readFloatSerial();
                       if ( TL_Pre_Delay < TL_Pre_Delay_Min){           // maker sure that value is not less then min.   
                        TL_Pre_Delay = TL_Pre_Delay_Min; 
                       }                          
                     break;
                     case's':
                     TL_PostDelay = readFloatSerial();
                     break;                     
                     case'c':
                     if (readFloatSerial() == 1){    // Set Stepper mode flag if true.
                     TL_Flag |= B00000010;           // set step mode flag
                     }else{
                     TL_Flag &= B11111101;           // clear step mode flag
                     
                     }                     
                     break;
                     case'd':
                     TL_Shot_Value = readFloatSerial();
                     break;
                     
                     /* DOES NOT WORK, becasue when capture via usb is used it auto focus with every capture command
                     case 'f':                
                     
                      queryType = BT_Serial.read();

                           switch(queryType){                             
                             //Focus IN
                             case '+':
                             TL_Flag |= Focus_Mode_True;      // Set Focus mode flag to true
                             queryType = readFloatSerial();
                                switch(queryType){
                                  case Focus_Step_I:
                                  TL_Focus_Step = 0x1;
                                  break;
                                  case Focus_Step_II:
                                  TL_Focus_Step = 0x2;
                                  break;
                                  case Focus_Step_III:
                                  TL_Focus_Step = 0x3;
                                  break; 
                                }
                             break;  
                             // Focus OUT                           
                             case '-':
                             TL_Flag |= Focus_Mode_True;      // Set Focus mode flag to true
                             queryType = readFloatSerial();
                                switch(queryType){
                                  case Focus_Step_I:
                                  TL_Focus_Step = 0x8001;
                                  break;
                                  case Focus_Step_II:
                                  TL_Focus_Step = 0x8002;
                                  break;
                                  case Focus_Step_III:
                                  TL_Focus_Step = 0x8003;
                                  break; 
                                }
                             break;
                             case 'x':
                             TL_Flag &= B11111011;      // clear Focus step flag
                             break;
                           }
                          */
                     break;
               
               
                     }
         break;
         case'g':           
           if(BT_Serial.read() == 'o'){                         // Start Timelapse 
            Dynamixel.ledState(MX28_ID_All, ON); 
            MX28_LED_State = true;
           
             switch (TL_Flag & B00000010){                      // check if in "Step" mode             
                 case Step_Mode_True:
                 TL_Flag |= B10000001;                          // Set timelapse active flag, Stop flag so that data is pre-loaded into MX-28 
                 break;
                 case Step_Mode_False:                          // Else run in deflaut "contiues mode"
                 TL_Flag &= B11111101;                          // Clear Step mode flag if set
                 TL_Flag |= B11110001;                          // Set flag as Timelapse Continues   
                 break;
                 
             }
             
            MsTimer2::start();                                  // Start interrupt timer 2 for overflow event
            Dynamixel.setHoldingTorque(MX28_ID, MX28_TORQUE_DEFALULT); // Set Torque value "power"
             #ifdef DEBUG
             BT_Serial.println("GO!");
             #endif 
           }
         
         break;
         
         case'd':
         break;
         
         case'k':
              queryType = BT_Serial.read();
              switch(queryType){
                case'h':
                KF_Flag |= Keyframe_Home;                             // set Home flag
                break;
                
                case'b':
                KF_Flag |= Keyframe_Begin;                               // set Begin keyframe flag
                KF_Slide_Begin = Dynamixel.readPosition(MX28_ID_slide);  // Read Slide MX28 present position
                KF_Pan_Begin = Dynamixel.readPosition(MX28_ID_pan);      // Read Pan MX28 present position
                KF_Tilt_Begin = Dynamixel.readPosition(MX28_ID_tilt);    // Read Tilt MX28 present position
                break;
                
                case'e':
                KF_Flag |= Keyframe_End;                                  // set end keyframe flag
                KF_Slide_End = Dynamixel.readPosition(MX28_ID_slide);     // Read Slide MX28 present position
                KF_Pan_End = Dynamixel.readPosition(MX28_ID_pan);         // Read Pan MX28 present position
                KF_Tilt_End = Dynamixel.readPosition(MX28_ID_tilt);       // Read Tilt MX28 present position
                break;
                
              }
         
         break;
         
         
         
         case'x':
           if(BT_Serial.read() == 'x'){

          MsTimer2::stop();                                     // Stop interrupt timer 2
           TL_Shot_Count = 0x00;                                // reset shot count 
           TL_Flag &= B00001110;                                // Clear timelapse flags
           MX28_DIR_Flag &= B00000000;
           Dynamixel.wheel(MX28_ID_All,RIGHT,0x00);

//           if (digitalRead(Wireless_Cap_Pin) == HIGH){        // Set shutter pin low if HIGH 
               digitalWrite(Wireless_Foc_Pin, LOW);
               digitalWrite(Wireless_Cap_Pin, LOW);                
 //          }
           delay(1);
           Dynamixel.ledState(MX28_ID_All, OFF);
           MX28_LED_State = false;
           }
         break;

          case '?':
          #ifdef DEBUG // Serial print info if in debaug mode to phone , serial termial must be used on phone to read 
          BT_Serial.println();
          delay(50);
          BT_Serial.print("TimeLapse mode:");
          delay(50);
            if (B0001 == (TL_Flag & B00000001 )){
            BT_Serial.println("Active");  
            }else if (B0000 == (TL_Flag & B00000001)){
            BT_Serial.println("Inactive");
            }
          delay(50);
          BT_Serial.print("TimeLapse speed:");
          delay(50);
          BT_Serial.println(MX28_Speed);
          delay(50);
//          BT_Serial.print("TimeLapse Dir:");
//          delay(50);
//          if (MX28_DIR == true){
//            BT_Serial.println("Right");
//            }else if (MX28_DIR == false){
//            BT_Serial.println("Left");  
//            }
          delay(50);
          BT_Serial.print("TimeLapse int:");
          delay(50);
          BT_Serial.println(TL_Interval);
           delay(50);
          BT_Serial.print("TimeLapse shut:");
          delay(50);
          BT_Serial.println(TL_PostDelay);
          delay(50);
          BT_Serial.print("Slide mode:");
          delay(50);
            if (Step_Mode_True == (TL_Flag & B00000010)){
            BT_Serial.println("Step"); 
            }else if (Step_Mode_False == (TL_Flag & B00000010)){
            BT_Serial.println("Contius");
            }
            
              KF_Slide_Begin = Dynamixel.readPosition(MX28_ID_slide);  // Read Slide MX28 present position
              BT_Serial.print("Slide Angle");
              BT_Serial.println(KF_Slide_Begin);
                KF_Pan_Begin = Dynamixel.readPosition(MX28_ID_pan);      // Read Pan MX28 present position
                BT_Serial.print("Pan Angle");
                BT_Serial.println(KF_Pan_Begin);
                  KF_Tilt_Begin = Dynamixel.readPosition(MX28_ID_tilt);    // Read Tilt MX28 present position
                  BT_Serial.print("Tilt Angle");
                  BT_Serial.println(KF_Tilt_Begin);
            
         
          #endif // Serial print info if in debaug mode to phone stop         
          
          break;
          
          }
    }
   
  }         
}

float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (BT_Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = BT_Serial.read();
      timeout = 0;
      index++;
    }
  }  
  while ((data[constrain(index-1, 0, 128)] != '!') && (timeout < 5) && (index < 128));  // check if "!" is readed, if not found check time out count
  return atof(data);
}

