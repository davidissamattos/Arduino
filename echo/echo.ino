
int n=0;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(4800);

}

// the loop routine runs over and over again forever:
void loop() {
  String content = "";
  char character;

  while(Serial.available()) {
      character = Serial.read();
      content.concat(character);
  }

  if (content != "") {
    Serial.print(content);Serial.print("\r");
  }
  delay(1000);
}



