// Define motor control pins
const int motor = 3;

// Variable to store the current motor state
char motorState = '1'; // Start with the motor off
int trigPin = 9;
int echoPin = 10;
int ptt;
float ptd;
float distance;

void setup() {

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);

  // Initialize motor pin as output
  pinMode(motor, OUTPUT);

  // Initialize Serial communication for Bluetooth
  Serial.begin(9600); // Ensure this matches the baud rate of your Bluetooth module

  // Turn off the motor initially
  digitalWrite(motor, LOW);
}

void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  ptt = pulseIn(echoPin, HIGH);


  ptd = 343.0 * ptt / 1000000; // Corrected assignment and added 0 to float value
  distance = ptd * 1000.0 / 2; // Dividing by 2 to get one-way distance
  
  Serial.print("Distance=");
  Serial.print(distance); // Fixed missing closing parenthesis
  Serial.println(" mm");
delay(1);
  if (Serial.available() > 0) { // Check if data is available to read
    char command = Serial.read(); // Read the command sent from the mobile app
    if (command == '1' || command == '2' || command == '3') {
      motorState = command; // Update the motor state based on the command
    }
  }

  // Perform action based on the current motor state
  switch (motorState) {
    case '1': // Stop the motor
      digitalWrite(motor, LOW);
      break;

    case '2': // Open (1700 microsecond pulse for servo)
      digitalWrite(motor, HIGH);
      delayMicroseconds(1700);
      digitalWrite(motor, LOW);
      delayMicroseconds(20000); // Rest time for servo pulse
            if (distance>120){
              motorState='1';
            }
      break;

    case '3': // Close (1300 microsecond pulse for servo)
      digitalWrite(motor, HIGH);
      delayMicroseconds(1300);
      digitalWrite(motor, LOW);
      delayMicroseconds(20000); // Rest time for servo pulse

      break;

    default:
      // Do nothing if an invalid state somehow occurs
      break;
  }
}
