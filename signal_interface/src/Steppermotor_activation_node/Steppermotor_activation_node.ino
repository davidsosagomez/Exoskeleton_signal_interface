// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 2;
const int stepsPerRevolution = 200;

// Variables to keep track of the last command and its state
bool lastCommandWasTrue = false;
bool motorMovedSinceLastFalse = false;

void setup() {
    // Initialize Serial communication
    Serial.begin(9600);

    // Declare pins as Outputs
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
        // Read the incoming string from Serial Monitor
        String input = Serial.readStringUntil('\n');

        // Clear the Serial buffer to avoid processing accumulated commands
        while(Serial.available() > 0) {
            Serial.read();
        }

        // Check if the input is 'true' or 'false'
        if (input.equals("true")) {
            if (!motorMovedSinceLastFalse) {
                // Set motor direction counter-clockwise
                digitalWrite(dirPin, LOW);
                rotateMotor(15); // Rotate 10 steps
                motorMovedSinceLastFalse = true;
            }
            lastCommandWasTrue = true;
        } else if (input.equals("false")) {
            // Set motor direction clockwise and rotate if last command was 'true'
            if (lastCommandWasTrue) {
                digitalWrite(dirPin, HIGH);
                rotateMotor(15); // Rotate 10 steps
            }
            lastCommandWasTrue = false;
            motorMovedSinceLastFalse = false;
        }

        // A short delay to debounce the input and allow the motor to settle
        delay(100);
    }
}

void rotateMotor(int steps) {
    // Spin motor for the specified number of steps
    for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1200);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1200);
    }
    delay(1000); // Wait a second after rotation
}
