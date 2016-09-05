#include <SPI.h>
#include "pins_arduino.h"
byte command = 0;
boolean Servomode;
boolean PositionLimit;
boolean Motormode;
boolean IDreset;
boolean ServoRst;
int Command[20]; 
int reclength = 0;
int Pos;
int ID,newID;
int velocity;
int PosLimit;
int upperLimit;
#define startByte 0xFF

#define P_MODEL_NUMBER_L 0
#define P_MODEL_NUMBER_H 1
#define P_VERSION 2
#define P_ID 3
#define P_BAUD_RATE 4
#define P_RETURN_DELAY_TIME 5
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_SYSTEM_DATA2 10
#define P_LIMIT_TEMPERATURE 11
#define P_DOWN_LIMIT_VOLTAGE 12
#define P_UP_LIMIT_VOLTAGE 13
#define P_MAX_TORQUE_L 14
#define P_MAX_TORQUE_H 15
#define P_RETURN_LEVEL 16
#define P_ALARM_LED 17
#define P_ALARM_SHUTDOWN 18
#define P_OPERATING_MODE 19
#define P_DOWN_CALIBRATION_L 20
#define P_DOWN_CALIBRATION_H 21
#define P_UP_CALIBRATION_L 22
#define P_UP_CALIBRATION_H 23
#define P_TORQUE_ENABLE (24)
#define P_LED (25)
#define P_CW_COMPLIANCE_MARGIN (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE (28)
#define P_CCW_COMPLIANCE_SLOPE (29)
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
#define P_TORQUE_LIMIT_L (34)
#define P_TORQUE_LIMIT_H (35)
#define P_PRESENT_POSITION_L (36)
#define P_PRESENT_POSITION_H (37)
#define P_PRESENT_SPEED_L (38)
#define P_PRESENT_SPEED_H (39)
#define P_PRESENT_LOAD_L (40)
#define P_PRESENT_LOAD_H (41)
#define P_PRESENT_VOLTAGE (42)
#define P_PRESENT_TEMPERATURE (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME (45)
#define P_MOVING (46)
#define P_LOCK (47)
#define P_PUNCH_L (48)
#define P_PUNCH_H (49)

//— Instruction —
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_DIGITAL_RESET 0x07
#define INST_SYSTEM_READ 0x0C
#define INST_SYSTEM_WRITE 0x0D
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_REG_WRITE 0x84

void setup()
{
  Serial.begin(1000000);                        // start serial port at 1000000 bps:
//    Serial.begin(115200);                        // start serial port at 1000000 bps:
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  Servomode=false;
  PositionLimit=false;
  Motormode=false;
  IDreset=false;
  ServoRst=false;
}

ISR (SPI_STC_vect)
{
  byte c = SPDR;

  switch (command)
  {
  case 0:     
    command = c;
    break;

  case 'p':
    Command[reclength]= c;
    reclength ++;
    if( Command[reclength-3]=='\t' && Command[reclength-2]=='\r' && Command[reclength-1]=='\n'){
      ID=Command[0];
      Pos=Command[1]*256+Command[2];
      velocity=Command[3]*256+Command[4];
      velocity=map(velocity,0,300,0x0000,0x03FF);
      Pos=map(Pos,0,300,0x0000,0x03FF);
      Servomode=true;
      reclength=0;       
    }
    break;
    
    case 's':
    Command[reclength]= c;
    reclength ++;
    if( Command[reclength-3]=='\t' && Command[reclength-2]=='\r' && Command[reclength-1]=='\n'){
      ID=Command[0];
      PosLimit=Command[1]*256+Command[2];
      PosLimit=map(PosLimit,0,300,0x0000,0x03FF);  
      PositionLimit=true;
      reclength=0;       
    }
    break;
  
    case 'm':
    Command[reclength]= c;
    reclength ++;
    if( Command[reclength-3]=='\t' && Command[reclength-2]=='\r' && Command[reclength-1]=='\n'){
      ID=Command[0];     
      velocity=Command[1]*256+Command[2];
      if(velocity>=0)  velocity=map(velocity,0,300,0x0000,0x03FF);     
      else velocity=map(velocity,0,-300,0x0400,0x07FF);
//      Serial.writeln(velocity,HEX);
      Motormode=true;
      reclength=0;       
    }
    break;
   
    case 'i':
    Command[reclength]= c;
    reclength ++;
    if( Command[reclength-3]=='\t' && Command[reclength-2]=='\r' && Command[reclength-1]=='\n'){
      ID=Command[0];
      newID=Command[1];
      IDreset=true;
      reclength=0;       
    }
    break;
    
     case 'r':
    Command[reclength]= c;
    reclength ++;
    if( Command[reclength-3]=='\t' && Command[reclength-2]=='\r' && Command[reclength-1]=='\n'){
      ID=Command[0];
      ServoRst=true;
      reclength=0;       
    }
    break;
    
    
        
  default:
    reclength=0;
    break;
  }

} 
void loop()
{

  if(Servomode){  
    WritePos(ID,Pos,velocity);
    Servomode=false;
  }
  
    if(PositionLimit){  
    SetServoLimit(ID,PosLimit);
    PositionLimit=false;
  }
  
  
  if(Motormode){  
    ServoRotate(ID,velocity); 
    Motormode=false;
  }
  
    if(IDreset){  
    SetID(ID,newID);   
    Motormode=false;
  }
  
     if(ServoRst){
      Reset(ID); 
      ServoRst=false; 
     }
 
  if (digitalRead (SS) == HIGH)
    command = 0;

}  


void WritePos(int ID,int Pos,int velocity){
  int messageLength = 7;
  byte pos2 = (Pos>>8 & 0xff);
  byte pos = (Pos & 0xff); 	

  byte vel2 =  (velocity>>8 & 0xff);
  byte vel =  (velocity & 0xff);

  Serial.write(startByte);              // send some data
  Serial.write(startByte);
  Serial.write(ID);
  Serial.write(messageLength);
  Serial.write(INST_WRITE);
  Serial.write(P_GOAL_POSITION_L);
  Serial.write(pos);
  Serial.write(pos2);
  Serial.write(vel);
  Serial.write(vel2);
  Serial.write((~(ID + (messageLength) + INST_WRITE + P_GOAL_POSITION_L + pos + pos2 + vel + vel2))&0xFF);
}


void SetServoLimit(int ID, int PosLimit){
  int messageLength = 5;
  byte PosLimitB = (PosLimit>>8 & 0xff);
  byte PosLimitS = (PosLimit& 0xff);
  Serial.write(startByte);              // send some data
  Serial.write(startByte);
  Serial.write(ID);
  Serial.write(messageLength);
  Serial.write(INST_WRITE);
  Serial.write(0x08);
  Serial.write(PosLimitB);
  Serial.write(PosLimitS);	
  Serial.write((~(ID + (messageLength) + INST_WRITE+ 0x08 +  PosLimitB+ PosLimitS))&0xFF);
}

void ServoRotate(int ID, int velocity){
   int messageLength = 5;
  byte velocityB = (velocity>>8 & 0xff);
  byte velocityS = (velocity & 0xff);
  Serial.write(startByte);              // send some data
  Serial.write(startByte);
  Serial.write(ID);
  Serial.write(messageLength);
  Serial.write(INST_WRITE);
  Serial.write(0x20);
  Serial.write(velocityS);
  Serial.write(velocityB);	
  Serial.write((~(ID + (messageLength) + INST_WRITE+ 0x20 +  velocityB+ velocityS))&0xFF);  
}


void SetID(int ID, int newID){
  int messageLength = 4;
  Serial.write(startByte);              // send some data
  Serial.write(startByte);
  Serial.write(ID);
  Serial.write(messageLength);
  Serial.write(INST_WRITE);
  Serial.write(P_ID);
  Serial.write(newID);
  Serial.write((~(ID + (messageLength) + INST_WRITE+ P_ID + newID))&0xFF);
}


void Reset(int ID){    
  // renew ID:1    Baund:1M
  Serial.write(startByte);           
  Serial.write(startByte);
  Serial.write(ID);
  Serial.write(0x02);
  Serial.write(0x06);
  Serial.write((~(ID + 0x02 +0x06))&0xFF);
}



