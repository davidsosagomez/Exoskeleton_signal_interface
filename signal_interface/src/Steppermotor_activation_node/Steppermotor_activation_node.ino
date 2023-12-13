#include <ros.h>
#include <std_msgs/Bool.h>

const int dirPin = 5;
const int stepPin = 2;
const int stepsPerRevolution = 200;

bool lastCommandWasTrue = false;
bool motorMovedSinceLastFalse = false;

ros::NodeHandle nh;

void collisionCallback(const std_msgs::Bool& msg) {
    bool input = msg.data;

    // Check if the input is true or false
    if (input) {
        if (!motorMovedSinceLastFalse) {
            digitalWrite(dirPin, LOW);
            rotateMotor(8);
            motorMovedSinceLastFalse = true;
        }
        lastCommandWasTrue = true;
    } else {
        if (lastCommandWasTrue) {
            digitalWrite(dirPin, HIGH);
            rotateMotor(8);
        }
        lastCommandWasTrue = false;
        motorMovedSinceLastFalse = false;
    }
}

ros::Subscriber<std_msgs::Bool> collision_sub("/collision", &collisionCallback);

void setup() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    nh.initNode();
    nh.subscribe(collision_sub);
}

void rotateMotor(int steps) {
    for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1200);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1200);
    }
    delay(1000);
}

void loop() {
    nh.spinOnce();
    delay(10);
}

