#define NUM_SENSORS 2
const int sensorPins[NUM_SENSORS] = { A0, A1 };

void setup() {
	for(int i = 0; i < NUM_SENSORS; i++)
		pinMode(sensorPins[i], INPUT);
	Serial.begin(2000000);

	pinMode(2, OUTPUT);
}

void loop() {
	unsigned long t = micros();
	Serial.write((byte*)&t, 2);
	for(int i = 0; i < NUM_SENSORS; i++) {
		unsigned int val = analogRead(sensorPins[i]);
		Serial.write((byte*)&val, 2);
	}

	if(t % 100000 < 20000)
		digitalWrite(2, HIGH);
	else
		digitalWrite(2, LOW);
}

