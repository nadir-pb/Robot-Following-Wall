const int sensorPin1 = A0;
const int sensorPin2 = A1;
const int motorPin1 = 3;
const int motorPin2 = 5;
const int maxDistanceDifference = 20;
const int minBrightness = -120;
const int maxBrightness = 120;
const int w = 13; // Assuming 13 for the width 'w'
const int l = 22;
const int distanceReference = 27;
const int trigPin = 6;
const int echoPin = 11;

// PID Constants
float K = -10;
float KP = 3.0;  // Proportional constant
float KI = 0;  // Integral constant
float KD = 0;  // Derivative constant

// PID Variables
float alphaError = 0;
float previousAlphaError = 0;
float integral = 0;

int sensorValue1 = 0;
float distance1 = 0;
int sensorValue2 = 0;
float distance2 = 0;

long int t2 = 0;
long int t1 = 0;
float alpha = 0;
float duration = 0;
float USdistance = 0;
float distance = 0;
float distanceError = 0;
float alphaRef = 0;
float proportional = 0;
float derivative = 0;
float controlSignal = 0;
int motorSpeed =0;

void setup() {

  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  Serial.begin(9600);
  digitalWrite(8, LOW);
  digitalWrite(7, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {

  t1 = millis();

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  USdistance = (duration*.0343)/2;

  if(USdistance < 20){

  analogWrite(motorPin1, 80);
  analogWrite(motorPin2, 0);
  delay(1200);

  }

  distance1 = 0;
  distance2 = 0;
  
  for (int n = 0; n < 10; n++) {
    sensorValue1 = analogRead(sensorPin1);
    distance1 += convertToDistance(sensorValue1);
    sensorValue2 = analogRead(sensorPin2);
    distance2 += convertToDistance(sensorValue2);
  }

  distance1 /= 10;
  distance2 /= 10;

  if (distance2 > 70 || distance2 < 5){
  analogWrite(motorPin1, 60);
  analogWrite(motorPin2, 120);
  delay(1200);
  }
  
  // Calculate and print the degree
  alpha = atan((distance1 - distance2) / l); // Assuming 10.0 for the length 'l'

  // Calculate and print the distance
  distance = ((distance1 + distance2) / 2 + w / 2) * cos(alpha);

  // Calculate alpha error
  distanceError = distanceReference - distance;
  alphaRef = K * distanceError/60.0;
  alphaError = alphaRef - alpha;

  // Calculate PID control output
  proportional = KP * alphaError;
  integral += alphaError;
  derivative = KD * (alphaError - previousAlphaError);

  // Compute the control signal
  controlSignal = proportional + KI * integral + derivative;

  // Apply constraints to control signal
  if (controlSignal > maxBrightness) {
    controlSignal = maxBrightness;
  } else if (controlSignal < minBrightness) {
    controlSignal = minBrightness;
  }

  // Update previous error
  previousAlphaError = alphaError;

  // Update motor speed
  motorSpeed = controlSignal;

  analogWrite(motorPin1, 100 - motorSpeed);
  analogWrite(motorPin2, 100 + 15 +motorSpeed);

  delay(5);
}

float convertToDistance(int value) {
  // Approximate values for model GP2Y0A21YK0F
  float distance = (1.0 / (0.0002391473 * value - 0.0100251467));
  return distance;
}
