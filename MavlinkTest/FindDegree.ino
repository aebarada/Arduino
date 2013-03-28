 /**********************************************************
 Find Degree 
 
 Takes outputs from RDF board and translates them
 to the degree pointing to the radio source
 
 Updated 3/28/2013
 With new mega pin numbers
 Added RSSI input, only serial prints RSSI right now
 Will anyone ever actually read this useful header?
 Probably Not
 
 ************************************************************/


//Global Variables  

int DegreeFinder[8][4];

//Setup pins, Degree matrix

void setup() {
  // initialize serial communication at 9600 bits per second:
   //start serial connection
  Serial.begin(9600);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(49, INPUT); //B4
  pinMode(48, INPUT);  //B3
  pinMode(51, INPUT);  //B2

  pinMode(46, INPUT); //Q2
  pinMode(47, INPUT); //Q3
  pinMode(44, INPUT); //Q4
  pinMode(45, INPUT); //Q5
  
 // pinMode(A15, INPUT); //RSSI
  pinMode(53 , INPUT); //Digital Output; Not Used
  
  BuildDegree();
}

// Main function for finding degree

void loop() {
  int Y = findY();
  int Q = findQ();
  int degree = DegreeFinder[Y][Q];
  int RSSI = analogRead(A15);
  Serial.print("RSSI: ");
  Serial.println(RSSI);
  Serial.print("Degree: ");
  Serial.println(degree);
  delay(500);
}

//Finds the Y part to find degree

int findY(){

  int A2 = digitalRead(49); //B4
  int A1 = digitalRead(48);  //B3
  int A0 = digitalRead(51);  //B2
  
  Serial.print(A2); //B4
  Serial.print(A1); //B3
  Serial.print(A0); //B2
  Serial.println();
  //Serial.print((A2 << 2) | (A1 << 1) | A0);
  
  return (A2 << 2) | (A1 << 1) | A0;
}

//Finds the Q for finding degree

int findQ(){
  int x = 0, q0 =0, q1 = 0, q2 = 0, q3 = 0;
  q0 = digitalRead(46); //Q2
  q1 = digitalRead(47); //Q3
  q2 = digitalRead(44); //Q4
  q3 = digitalRead(45); //Q5
  
  if(q0){
    x = 0;
  }
  else if(q1){
    x = 1;
  }
  else if(q2){
    x = 2;
  }
  else if(q3){
    x = 3;
  }
Serial.print(x);
Serial.println();
 return x;
}

//Builds 2 dimensional array holding degree values

void BuildDegree(){
 
 DegreeFinder[0][0] = 0;
 DegreeFinder[0][1] = 90;
 DegreeFinder[0][2] = 180;
 DegreeFinder[0][3] = 270;
 DegreeFinder[1][0] = 11;
 DegreeFinder[1][1] = 101;
 DegreeFinder[1][2] = 191;
 DegreeFinder[1][3] = 281;
 DegreeFinder[2][0] = 22;
 DegreeFinder[2][1] = 112;
 DegreeFinder[2][2] = 202;
 DegreeFinder[2][3] = 292;
 DegreeFinder[3][0] = 34;
 DegreeFinder[3][1] = 124;
 DegreeFinder[3][2] = 214;
 DegreeFinder[3][3] = 304;
 DegreeFinder[4][0] = 45;
 DegreeFinder[4][1] = 135;
 DegreeFinder[4][2] = 225;
 DegreeFinder[4][3] = 315;
 DegreeFinder[5][0] = 56;
 DegreeFinder[5][1] = 146;
 DegreeFinder[5][2] = 236;
 DegreeFinder[5][3] = 326;
 DegreeFinder[6][0] = 67;
 DegreeFinder[6][1] = 157;
 DegreeFinder[6][2] = 247;
 DegreeFinder[6][3] = 337;
 DegreeFinder[7][0] = 79;
 DegreeFinder[7][1] = 169;
 DegreeFinder[7][2] = 259;
 DegreeFinder[7][3] = 349;

}

