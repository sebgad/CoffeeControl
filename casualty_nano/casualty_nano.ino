float f_pwm =0;
float f_pct =0;
int potiValue=0;
int tempValue=0;

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  potiValue = analogRead(A2);
  tempValue = analogRead(A1);
  
  f_pct = map(potiValue, 0, 1023, 0.0, 100.0);
  f_pwm = map(potiValue, 0, 1023, 0, 255);
  f_pct = constrain(f_pct, 0, 100);
  f_pwm = constrain(f_pwm, 0, 255);

  //float voltage = sensorValue * (5.0 / 1023.0);// 5'V is 1023 int
  //float f_pct = voltage * (100.0)/4.7;// V-> %
  //float f_pwm = voltage * (255.0)/4.7;// V-> pwm
  // print out the value you read:

  //Serial.println(voltage);
  Serial.println(f_pct);

  //
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(100);                       // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  //delay(100);
  analogWrite(3,f_pwm);

  digitalWrite(6, HIGH);
  delayMicroseconds(10*f_pct); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(6, LOW);
  delayMicroseconds(1000 - 10*f_pct);

}
