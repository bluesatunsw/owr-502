bool mosi_1a[] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 0, 0,
};

bool mosi_1b[] {
  1, 0, 0, 0, 0, 0, 0, 0,
  1, 0, 1, 0, 0, 0, 0, 1,
};

bool mosi_2a[] = {
  1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 0, 1,
};

bool mosi_2b[] {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 0, 0,
};

bool nop[] {
  1, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

bool mosi_1r[] = {
  1, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 0, 0,
};

bool mosi_2r[] = {
  0, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 1, 0, 0, 1,
};

bool mosi_pw[] {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 1,
};

bool mosi_pa[] {
  1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1,
};

bool mosi_pb[] {
  1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 1, 0, 0, 0,
};

bool mosi_pr[] {
  1, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 1,
};

#define CS 9
#define CLK 10
#define MOSI 11
#define MISO 12

void do_frame(bool* data, int n = 16) {
  digitalWrite(CS, HIGH);
  digitalWrite(CLK, LOW);
  digitalWrite(MOSI, LOW);
  delay(1);
  digitalWrite(CS, LOW);
  delay(1);

  for (int i = 0; i < n; i++) {
    digitalWrite(MOSI, data[i]);
    digitalWrite(CLK, HIGH);
    delay(1);
    digitalWrite(CLK, LOW);
    Serial.print(digitalRead(MISO), BIN);
    delay(1);
  }
  digitalWrite(CS, HIGH);
  digitalWrite(MOSI, LOW);
  Serial.println("");
  delay(10);
}

void setup() {
  pinMode(CS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CLK, OUTPUT);

  digitalWrite(CS, HIGH);
  digitalWrite(CLK, LOW);
  digitalWrite(MOSI, LOW);

  Serial.begin(9600);
  Serial.println("\n\nBitbang programmer starting.");
  Serial.println("Press any key to start.");
  while (Serial.read() == -1) {}

  Serial.println("Initial values");
  do_frame(mosi_1r);
  do_frame(mosi_2r);
  do_frame(nop);

  Serial.println("Writing");
  do_frame(mosi_1a);
  do_frame(mosi_1b);
  do_frame(mosi_2a);
  do_frame(mosi_2b);
  do_frame(nop);

  Serial.println("Verifying");
  do_frame(mosi_1r);
  do_frame(mosi_2r);
  do_frame(nop);

  Serial.println("Press any key to continue to burning.");
  while (Serial.read() == -1) {}

  do_frame(mosi_pw);
  do_frame(mosi_pa);
  do_frame(mosi_pw);
  do_frame(mosi_pb);
  do_frame(nop);

  Serial.println("Bitbang programmer complete.");
  while (true) {
    Serial.println("Press any key to check progress.");
    while (Serial.read() == -1) {}
    do_frame(mosi_pr);
    do_frame(nop);
  }
}

void loop() { }
