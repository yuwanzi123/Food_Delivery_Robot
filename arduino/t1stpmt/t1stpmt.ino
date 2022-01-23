 int PUL=22; //define Pulse pin
int DIR=2; //define Direction pin
int ENA=3; //define Enable Pin

void setup() {
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
}

void loop() {

  for (int i=0; i<200; i++)    //Forward 6400 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(1250);
    digitalWrite(PUL,LOW);
    delayMicroseconds(1250);
  }
  
//  for (int i=0; i<6400; i++)   //Backward 5000 steps
//  {
//    digitalWrite(DIR,HIGH);
//    digitalWrite(ENA,HIGH);
//    digitalWrite(PUL,HIGH);
//    delayMicroseconds(50);
//    digitalWrite(PUL,LOW);
//    delayMicroseconds(50);
//  }
}
