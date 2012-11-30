unsigned long echo = 0;
int frontSensor = 34;
int rightSensor = 37;
int leftSensor = 36;
int backSensor = 35;
unsigned long ultrasoundValue = 0;
 
void setup()
{
   Serial.begin(9600);
   pinMode(frontSensor,OUTPUT);
   pinMode(rightSensor,OUTPUT);
   pinMode(leftSensor,OUTPUT);
   pinMode(backSensor,OUTPUT);
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

void printOutput(long front, long left, long right, long back)
{ 
  String output = "\nF:";
  output += front;
  output += "\tL:";
  output += left;
  output += "\tR:";
  output += right;
  output += "\tB:";
  output += back;
  Serial.println(output);
}
 
void loop()
{
   long front = 0, left=0, right=0, back=0;
   front = ping(frontSensor);
   left = ping(leftSensor);
   right = ping(rightSensor);
   back = ping(backSensor);
   printOutput(front, left, right, back);
   delay(250); //delay 1/4 seconds.
}
