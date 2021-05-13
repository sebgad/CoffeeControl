float f_pwm =0;
float f_pct =0;
int sensorValue=0;

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
  sensorValue = analogRead(A2);
  f_pct = map(sensorValue, 0, 1023, 0.0, 100.0);
  f_pwm = map(sensorValue, 100, 1023, 0, 255);
  f_pct = constrain(f_pct, 0, 100);
  f_pwm = constrain(f_pwm, 0, 255);

  //float voltage = sensorValue * (5.0 / 1023.0);// 5'V is 1023 int
  //float f_pct = voltage * (100.0)/4.7;// V-> %
  //float f_pwm = voltage * (255.0)/4.7;// V-> pwm
  // print out the value you read:

  //Serial.println(voltage);
  //Serial.pintln(f_pct);

  //
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(100);                       // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  //delay(100);
  digitalWrite(3,f_pwm);
  digitalWrite(6, HIGH);
  delayMicroseconds(100*f_pct); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(6, LOW);
  delayMicroseconds(100 - 100*f_pct);

}
