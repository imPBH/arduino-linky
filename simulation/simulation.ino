/************* INCLUDES *************/

/************* DEFINES *************/
#define GREEN_LED 13
#define RED_LED 12
#define YELLOW_LED 4
#define BUZZER_PIN 8
#define TRIG_PIN 2
#define ECHO_PIN 3
#define MOTOR_PIN 7
#define CONSUMPTION_LIMIT 400
#define DISTANCE_LIMIT 15.0 

/************* VARIABLES *************/
bool isAlertDistanceOn = false;
bool isAlertConsoOn = false;
bool alertDistanceState = false;
bool alertConsoState = false;
unsigned long previousMillisAlertDistance = 0;

long dureeDistance;
float distance;

unsigned long previousMillisNumber = 0;
unsigned long intervalNumber = 500;

unsigned long previousMillisHourly = 0;
unsigned long intervalHourly = 500;
float totalPappHourly = 0.0;
unsigned int pappCounterHourly = 0;
unsigned long currentMillisHourly = 0;

long newNumber = 0;
long number = 0;

unsigned long previousBlinkMillis = 0;
const long blinkInterval = 500;
boolean blinkState = false;

unsigned long previousMillisAlertConso = 0;
const long intervalAlert = 250;
boolean ledStateAlertConso = false;
boolean ledStateAlertDistance = false;
boolean buzzerStateAlert = false;

/************* FUNCTIONS *************/
long getNumber() {                                                      // LINKY SIMULATION
  unsigned long currentMillis = millis();                               // get actual time
  if (currentMillis - previousMillisNumber >= intervalNumber) {         // check if delay is exceeded
    previousMillisNumber = currentMillis;                               // store current time as the last change
    return(random(300, 600));                                           // return a random number between 300 and 600 VA
  } else {                                                              // if delay not exceeded
  	return(long(0));                                                    // return 0
  }
}

void blink() {                                                          // POWER LED BLINKING
  unsigned long currentMillis = millis();                               // get actual time
  if (currentMillis - previousBlinkMillis >= blinkInterval) {           // check if delay is exceeded
    previousBlinkMillis = currentMillis;                                // store current time as the last change
    blinkState = !blinkState;                                           // invert led state
    digitalWrite(GREEN_LED, blinkState);                                // write the new state
  }
}

void alertConso() {                                                     // CONSUMPTION ALERT
  unsigned long currentMillis = millis();                               // get actual time
  if (currentMillis - previousMillisAlertConso >= intervalAlert) {      // check if delay is exceeded
    previousMillisAlertConso = currentMillis;                           // store current time as the last change
    ledStateAlertConso = !ledStateAlertConso;                           // invert led state
    buzzerStateAlert = !buzzerStateAlert;                               // invert buzzer state
    digitalWrite(RED_LED, ledStateAlertConso);                          // write the new state
    Serial.println("Alerte ! Consommation anormale " + String(number) + "W !");
    if(buzzerStateAlert) {
      tone(BUZZER_PIN,800);                                             // turn the buzzer on
    } else {
      noTone(BUZZER_PIN);                                               // or turn it off
    }
  }
}

void alertDistance() {                                                  // DISTANCE ALLERT
  unsigned long currentMillis = millis();                               // get actual time
  if (currentMillis - previousMillisAlertDistance >= intervalAlert) {   // check if delay is exceeded
    previousMillisAlertDistance = currentMillis;                        // store current time as the last change
    ledStateAlertDistance = !ledStateAlertDistance;                     // invert led state
    digitalWrite(YELLOW_LED, ledStateAlertDistance);                    // write the new state
    Serial.println();
    Serial.println("Alerte intrusion !");                               // write the alert in the serial
  }
}

/************* SETUP *************/
void setup() {
  Serial.begin(9600);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, HIGH);                                        // turn the motor on
  Serial.println("Entrez 'M' pour voir les moyennes de consommation, 'A' pour activer/desactiver l'alerte de consommation et 'I' pour activer/desactiver l'alerte d'intrusion");
}

/************* LOOP *************/
void loop() {
  digitalWrite(TRIG_PIN, LOW);                                          // measure the distance
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  dureeDistance = pulseIn(ECHO_PIN, HIGH);
  distance = dureeDistance * 0.017;
  
  if(distance < DISTANCE_LIMIT) {                                       // turn the alert on or off
    alertDistanceState = true;
  } else {
  	alertDistanceState = false;
  }
  
  blink();                                                              // blink the power led
  newNumber = getNumber();                                              // generate a new number for linky simulation
  
  if (newNumber != 0) {                                                 // if we get a new number
    number = newNumber;
 
  	currentMillisHourly = millis();
  	if(currentMillisHourly - previousMillisHourly >= intervalHourly) {  // display hourly, daily and monthly averages
    	previousMillisHourly = currentMillisHourly;
    
    	totalPappHourly += number;
    	pappCounterHourly += 1;
  	}
  }
  
  if(number > CONSUMPTION_LIMIT) {                                     // if we reach the consumption threshold
    alertConsoState = true;                                            // change alert state
  } else {
  	alertConsoState = false;
  }
  
  if(alertDistanceState && isAlertDistanceOn) {                        // call alerts if needed
  	alertDistance();
  }
  if(alertConsoState && isAlertConsoOn) {
  	alertConso();
    digitalWrite(MOTOR_PIN, LOW);
  } else {
  	digitalWrite(MOTOR_PIN, HIGH);
  }

  if(Serial.available() > 0) {
    char input = Serial.read();
    if(input == 'M') {
      float moyenneHourly = totalPappHourly / (float)pappCounterHourly / 1000;
      Serial.println();
      Serial.println("Nombre d'enregistrements : " + String(pappCounterHourly));
      Serial.println();
      Serial.println("Consommation actuelle : " + String(number) + "W");
      Serial.println("Moyenne horraire : " + String(moyenneHourly) + "kWh/heure");
      Serial.println("Moyenne journaliere : " + String(moyenneHourly*24) + "kWh/jour");
      Serial.println("Moyenne mensuelle : " + String(moyenneHourly*24*31) + "kWh/mois");
    }
    if(input == 'A') {
      isAlertConsoOn = !isAlertConsoOn;
      if(isAlertConsoOn) {
        Serial.println("L'alerte de consommation est activee");
      } else {
        Serial.println("L'alerte de consommation est desactivee");
      }
    }
    if(input == 'I') {
      isAlertDistanceOn = !isAlertDistanceOn;
      if(isAlertDistanceOn) {
        Serial.println("L'alerte d'intrustion est activee");
      } else {
        Serial.println("L'alerte d'intrustion est desactivee");
      }
    }
    Serial.println("Entrez 'M' pour voir les moyennes de consommation, 'A' pour activer/desactiver l'alerte de consommation et 'I' pour activer/desactiver l'alerte d'intrusion");
  }
}