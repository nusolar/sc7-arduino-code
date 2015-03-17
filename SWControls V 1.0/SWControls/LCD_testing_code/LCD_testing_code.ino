#include <serLCD.h>

serLCD screen(Serial1);
void setup(){
screen.begin();
screen.clear();

}

void blnk(int a, int b){
 screen.setCursor(1,a);
 if (b == 1){
 screen.print("<<");
 }
 else{
   screen.print(">>");
 }
 screen.setCursor(2,a);
 if (b == 1){
 screen.print("<<");
 }
 else{
   screen.print(">>");
 }
 delay(500);
 screen.setCursor(1,a);
 screen.print("  ");
 screen.setCursor(2,a);
 screen.print("  ");
 delay(500);
}

void loop(){
blnk(1,1);
//blnk(14,0);
screen.setCursor(1,4);
screen.print("SOC:  %");
screen.setCursor(1,11);
screen.print(" HZ");
screen.setCursor(2,4);
screen.print("V:  ");
screen.setCursor(2,11);
screen.print("C ");
screen.setCursor(2,13);
screen.print("F");

}
