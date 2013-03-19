 
 
 
  int DegreeFinder[8][4];
void setup() {
  // initialize serial communication at 9600 bits per second:
   //start serial connection
  Serial.begin(9600);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(22, INPUT);
  pinMode(24, INPUT);
  pinMode(26, INPUT);

  pinMode(28, INPUT);
  pinMode(30, INPUT);
  pinMode(32, INPUT); 
  pinMode(34, INPUT);
  BuildDegree();
}

// the loop routine runs over and over again forever:
void loop() {
  int Y = findY();
  int Q = findQ();
  int degree = DegreeFinder[Y][Q];
  Serial.println(degree);
  delay(2000);
}

int findY(){
  //read the pushbutton value into a variable
  int A2 = digitalRead(22);
  int A1 = digitalRead(24);
  int A0 = digitalRead(26);
  
  //print out the value of the pushbutton
  Serial.print(A2);
  Serial.print(A1);
  Serial.print(A0);
  Serial.println();
  Serial.print((A2 << 2) | (A1 << 1) | A0);
  
  return (A2 << 2) | (A1 << 1) | A0;
}

int findQ(){
  int x = 0, q0 =0, q1 = 0, q2 = 0, q3 = 0;
  q0 = digitalRead(28);
  q1 = digitalRead(30);
  q2 = digitalRead(32);
  q3 = digitalRead(34);
  
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

