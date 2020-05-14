
void setup() {

  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);

  pinMode(PB8, OUTPUT);
  pinMode(PB9, OUTPUT);

  analogWrite(PB6, 0);
  analogWrite(PB7, 0);

  analogWrite(PB8, 0);
  analogWrite(PB9, 0);
}

void PrawyM(int v){
  if (v > 0 ) {
    analogWrite(PB6, 0);
    analogWrite(PB7, v);
  }
  else {
    analogWrite(PB7, 0);
    analogWrite(PB6, -v);
  }
}

void LewyM(int v){
  if (v < 0 ) {
    analogWrite(PB8, 0);
    analogWrite(PB9, -v);
  }
  else {
    analogWrite(PB9, 0);
    analogWrite(PB8, v);
  }
}

void loop() {
int a = 50;
  PrawyM(-a);
  LewyM(-a);
  delay(1000);

  PrawyM(a);
  LewyM(a);
  delay(1000);
}
