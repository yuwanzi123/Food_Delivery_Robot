  int PULL=22; //define Pulse pin
  int DIRL=2; //define Direction pin
  int ENAL=3; //define Enable Pin

  int PULR=52;
  int DIRR=4;
  int ENAR=5;
  int n=0;

void setup() {
  pinMode (PULL, OUTPUT);
  pinMode (DIRL, OUTPUT);
  pinMode (ENAL, OUTPUT);
  pinMode (PULR, OUTPUT);
  pinMode (DIRR, OUTPUT);
  pinMode (ENAR, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if(Serial.available()>0){
    n = (Serial.read() - '0');
    if(n == 1){
      cartstop();
      }
    else{
      cartforward();
      print n;
      } 
    }

}

void cartforward(){
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

void cartstop(){
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
