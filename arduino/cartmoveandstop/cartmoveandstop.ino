//cartmove final version
//control wheels
//3.27.2018

int PULL=22; //define Pulse pin
int DIRL=2; //define Direction pin
int ENAL=3; //define Enable Pin

int PULR=24;
int DIRR=4;
int ENAR=5;

void setup() {  
  pinMode (PULL, OUTPUT);
  pinMode (DIRL, OUTPUT);
  pinMode (ENAL, OUTPUT);
  pinMode (PULR, OUTPUT);
  pinMode (DIRR, OUTPUT); 
  pinMode (ENAR, OUTPUT);   
}

void loop() {
    for (int i=0; i<500; i++)    //Stop
  {
    digitalWrite(DIRL,HIGH);
    digitalWrite(ENAL,LOW);
    digitalWrite(PULL,LOW);
    
    digitalWrite(DIRR,HIGH);
    digitalWrite(ENAR,HIGH);
    digitalWrite(PULR,HIGH);
    delayMicroseconds(2000);
    
    digitalWrite(PULL,LOW);
    digitalWrite(PULR,HIGH);
    delayMicroseconds(2000);
  }
  
  for (int i=0; i<500; i++)    //Forward
  {
    digitalWrite(DIRL,LOW);
    digitalWrite(ENAL,LOW);
    digitalWrite(PULL,LOW);
    
    digitalWrite(DIRR,LOW);
    digitalWrite(ENAR,HIGH);
    digitalWrite(PULR,HIGH);
    delayMicroseconds(2000);
    
    digitalWrite(PULL,HIGH);
    digitalWrite(PULR,LOW);
    delayMicroseconds(2000);
  }

  

}


