 #include <Metro.h>
 #inclue <sc7-can-libinclude.h>
 
 #define BIT(n)                  ( 1<<(n) ) 
 #define BIT_SET(y, mask)        ( y |=  (mask) ) 
 #define BIT_CLEAR(y, mask)      ( y &= ~(mask) ) 
 #define BIT_FLIP(y, mask)       ( y ^=  (mask) )
/* Bit masks */
#define FWD_GEAR BIT(0)
#define REV_GEAR BIT(1)
#define HEADLIGHT BIT(2)
#define HAZARDLIGHT BIT(3)
#define CRUISE_CONTROL BIT(4)
#define HORN BIT(5)
#define LEFT_TURN BIT(6)
#define RIGHT_TURN BIT(7)
byte young;
byte old;

//set up pins that connect to switch terminals
   const int fgp = ;
   const int rgp = ;
   const int hp = ;
   const int hzp = ;
   const int ccp = ;
   const int hornp = ;
   const int ltp = ;
   const int rtp = ;
   
pinMode(fgp, INPUT);
pinMode(rgp, INPUT);
pinMode(hp, INPUT);
pinMode(hzp, INPUT);
pinMode(ccp, INPUT);
pinMode(hornp, INPUT);
pinMode(ltp, INPUT);
pinMode(rtp, INPUT);

//set up metro timer
  //1st: switch state reading timer
  Metro switch_timer = Metro(20);
  //2nd: CAN Transmission timer
  Metro CAN_TX = Metro(1000);
  //3rd: CAN Reception timer
  //Metro CAN_RX = Metro(1000);

 void setup() {
  // put your setup code here, to run once:

/*SPST - left turn, right turn, horn, cruise control
SPDT - forward/neutral/reverse, headlight/no light/hazard*/

//set baud rate to 9600bps
Serial.begin(9600);

}

inline void setyoungbit(pin,bite,biit){
  if (digitalRead(pin)==1){
    BIT_SET(bite,biit);
  }
  else{
    BIT_CLEAR(bite,biit);
  }
}

void loop() {    
/*if the metro timer runs out, then check the states of all the switches
    assign the values to the 'young' byte. Reset switch timer.*/
  if (switch_timer.check() == 1){
    setyoungbit(fgp,young,FWD_GEAR);
    setyoungbit(rgp,young,REV_GEAR);
    setyoungbit(hp,young,HEADLIGHT);
    setyoungbit(hzp,young,HAZARDLIGHT);
    setyoungbit(ccp,young,CRUISE_CONTROL);
    setyoungbit(hornp,young,HORN);
    setyoungbit(ltp,young,LEFT_TURN);
    setyoungbit(rtp,young,RIGHT_TURN);
    switch_timer.reset();
  }
    //   ^ if (digtalRead(fgp)) 
//    {BIT_SET(status, FWD_GEAR);}
//    else
//    {BIT_CLEAR(status,FWD_GEAR);}
//    reverse_gear = digitalRead(rgp);
//    headlights = digitalRead(hp);
//    hazardlights = digitalRead(hzp);
//    cruise_control = digitalRead(ccp);
//    horn = digitalRead(hornp);
//    left_turn = digitalRead(ltp);
//    right_turn = digitalRead(rtp);
    
  /*If this byte is different from the one in the void setup(), send CAN packet
    , reassign the new values to the initial bit flags in the setup byte variable
    and reset CAN_TX timer.*/
    
  /*elseIf CAN_TX timer runs out, send CAN packet in the form of the setup byte variable
    and reset CAN_TX timer*/

}                       

