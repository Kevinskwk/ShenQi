/*
 * Arduino code for testing reedswitches.
 * Tested on Arduino Mega 
 */

// Setting Variables
const int reedRow[10] = {43, 53, 51, 41, 39, 49, 47, 37, 35, 45};
const int reedCol[9] = {50, 48, 46, 44, 40, 42, 38, 36, 34};
const char num2letter[9] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

int reeds[90];


void setup(){
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  for (int i=0; i<10; ++i){
    pinMode(reedRow[i], INPUT);
  }
  for (int i=0; i<9; ++i){
    pinMode(reedCol[i], OUTPUT);
    digitalWrite(reedCol[i], LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  checkReed();
  delay(800);
  Serial.println("-----");
}

void checkReed(){
  for (int a=0; a<9; ++a){
    digitalWrite(reedCol[a], HIGH);
    delay(10);
    for (int b=0; b<10; ++b){
      int state = digitalRead(reedRow[b]);
      if (state == 1) {
        Serial.print(num2letter[a]);
        Serial.println(b+1);
      }
    }
    digitalWrite(reedCol[a], LOW);
  }
}

void Hbot(){
  
}
