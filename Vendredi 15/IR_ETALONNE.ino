const int irSensorPin = A0;

void setup() {
  pinMode(irSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  float irValue = analogRead(irSensorPin);
  Serial.print("IR Value = ");
  Serial.println(irValue);
  delay(2000);
}

//inferieur à 20 : rien devant 
//150-160 = 30cm 
//250-280 = 20cm
//~460,470 = 10cm
//>600 = inférieur à 5cm. Si trop proche la valeur rebaisse. 
