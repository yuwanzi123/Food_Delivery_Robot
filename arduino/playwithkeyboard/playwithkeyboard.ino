//manually test move and stop
//finally work:
//the cart move forward continousely until we type a b in keyboard
//3.28
  
  int PULL=22; //define Pulse pin
  int DIRL=2; //define Direction pin
  int ENAL=3; //define Enable Pin

  int PULR=52;
  int DIRR=4;
  int ENAR=5;
  //int n=0;

void setup() {
  pinMode (PULL, OUTPUT);
  pinMode (DIRL, OUTPUT);
  pinMode (ENAL, OUTPUT);
  pinMode (PULR, OUTPUT);
  pinMode (DIRR, OUTPUT);
  pinMode (ENAR, OUTPUT);

  Serial.begin(9600);
  Serial.println("Enter a or b");
}

void loop() {
  cartforward();
  if(Serial.available()>0){
    int inByte = Serial.read();
    switch (inByte){
      case '1':
        cartforward();
        Serial.println(inByte);
        break;
      case '2':
        cartstop();
        Serial.println(inByte);
        break;
      }
    }
}

void cartforward(){
  //for (int i=0; i<1000; i++)    //Forward
  //{
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
  //}
}

void cartstop(){
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
}
