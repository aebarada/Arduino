unsigned long echo = 0;
int frontSensor = 34;
int rightSensor = 37;
int leftSensor = 36;
int backSensor = 35;
int input1 = 22;
int input2 = 23;
int input3 = 24;
int input4 = 25;
int enable = 26;
unsigned long ultrasoundValue = 0;
 
void setup()
{
   Serial.begin(9600);
   pinMode(frontSensor,OUTPUT);
   pinMode(rightSensor,OUTPUT);
   pinMode(leftSensor,OUTPUT);
   pinMode(backSensor,OUTPUT);
   pinMode(input1, INPUT);
   pinMode(input2, INPUT);
   pinMode(input3, INPUT);
   pinMode(input4, INPUT);
   pinMode(enable, INPUT);
}
 
unsigned long ping(int Pin){
  ultrasoundValue = 0;
  pinMode(Pin, OUTPUT); // Switch signalpin to output
  digitalWrite(Pin, LOW); // Send low pulse
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(Pin, HIGH); // Send high pulse
  delayMicroseconds(5); // Wait for 5 microseconds
  digitalWrite(Pin, LOW); // Holdoff
  pinMode(Pin, INPUT); // Switch signalpin to input
  digitalWrite(Pin, HIGH); // Turn on pullup resistor
  echo = pulseIn(Pin, HIGH); //Listen for echo
  ultrasoundValue = (echo / 58.138) * .39; //convert to CM then to inches
  return ultrasoundValue;
}

void printOutput(long front, long left, long right, long back, long dir)
{
  String output = "\nF:";
  output += front;
  output += "\tL:";
  output += left;
  output += "\tR:";
  output += right;
  output += "\tB:";
  output += back;
  output += "\tDir:";

  output += dir;
  output += " degrees";
  Serial.println(output);
}

int getDirection(){
 int number = 0;
 if(digitalRead(enable)){
   int one = digitalRead(input1);
   int two = digitalRead(input2);
   int three = digitalRead(input3);
   int four = digitalRead(input4);
   number |= one<<0;
   number |= two<<1;
   number |= three<<2;
   number |= four<<3;
 }  
 return number;
}
 
void loop()
{
   long front = 0, left=0, right=0, back=0, dir=0;
   //front = ping(frontSensor);
   //left = ping(leftSensor);
   //right = ping(rightSensor);
   //back = ping(backSensor);
   dir = getDirection();
   printOutput(front, left, right, back, dir);
   delay(100);
}
