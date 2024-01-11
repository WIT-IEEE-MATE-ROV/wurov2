// Define pin connections & motor's steps per revolution
const int dirPin = 4;
const int stepPin = 2;
const int stepsPerRevolution = 200;
const int rotations = 5;
const int delays = 1500;
void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);

  for(int i = rotations; i > 0; i--){
       // Spin motor slowly
    for(int x = 0; x < stepsPerRevolution; x++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delays);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delays);
    }
  }
	delay(1000); // Wait a second
	
	// // Set motor direction counterclockwise
	// digitalWrite(dirPin, LOW);

	// // Spin motor quickly
	// for(int x = 0; x < stepsPerRevolution; x++)
	// {
	// 	digitalWrite(stepPin, HIGH);
	// 	delayMicroseconds(delays);
	// 	digitalWrite(stepPin, LOW);
	// 	delayMicroseconds(delays);
	// }
	// delay(1000); // Wait a second
}