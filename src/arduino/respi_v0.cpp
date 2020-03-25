#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <AnalogButtons.h>

#define DEBUG // décommenter pour envoyer les messages de debug en série
// #define DEBUG_VERBOSE
// #define SIMULATION // décommenter pour simuler des valeurs de capteur de pression au lieu de lire les vraies

// Période en ms de la boucle de traitement
const uint32_t PERIODE_DE_TRAITEMENT = 10ul;

// borne pour le capteur de pression
const int CAPT_PRESSION_MINI = 0;   // a adapter lors de la calibration
const int CAPT_PRESSION_MAXI = 800; // on ne va pas jusqu'à 1024 à cause de la saturation de l'AOP -> à adapter avec meilleur AOP

// servomoteur blower : connecte le flux d'air vers le Air Transistor patient ou vers l'extérieur
// 90° → tout est fermé
// entre 45° (90 - ANGLE_OUVERTURE_MAXI) et 82° (90 - ANGLE_OUVERTURE_MINI)
// entre 98° (90 + ANGLE_OUVERTURE_MINI) et 135° (90 + ANGLE_OUVERTURE_MAXI)
Servo blower;
#define BLOWER_CLOSED 75 // Consigne pour fermer la vanne blower → envoi du flux vers le Air Transistor patient
#define BLOWER_OPEN 40   //Consigne pour ouvrir la vanne blower → envoi du flux vers l'extérieur

//temporaire pour la démo
#define BLOWER_PLATEAU 70

// servomoteur patient : connecte le patient au flux d'air entrant ou à l'extérieur
// 90° → tout est fermé
// entre 45° (90 - ANGLE_OUVERTURE_MAXI) et 82° (90 - ANGLE_OUVERTURE_MINI)
// entre 98° (90 + ANGLE_OUVERTURE_MINI) et 135° (90 + ANGLE_OUVERTURE_MAXI)
Servo patient;

//Consignes servo patient
//Le Y patient relie le blower vers le patient → envoi du flux vers le patient
#define PATIENT_IN 70

#define PATIENT_PLATEAU 70
//le y ferme hermétiquement le patient
#define PATIENT_CLOSED 35

//Le Y laisse l'air s'échapper un petit peu du patient → échappe un tout petit peu l'air du patient vers l'extérieur
#define PATIENT_CLOSED_LEAK 30

// Le Y laisse l'air s'échapper du patient → échappe l'air du patient vers l'extérieur
#define PATIENT_OUT 30

// entrées-sorties
#define PIN_CAPTEUR_PRESSION A4
const int PIN_SERVO_BLOWER = 4;  // D4
const int PIN_SERVO_PATIENT = 2; // D2
#define BUZZER_PIN 13            //D13
#define ANALOG_PIN A0
const int BTN_FREE2 = 913;
const int BTN_FREE1 = 821;
const int BTN_ALARM_OFF = 745;
const int BTN_ALARM_ON = 607;
const int BTN_CYCLE_MINUS = 509;
const int BTN_CYCLE_PLUS = 413;
const int BTN_PRESSION_PEP_MINUS = 292;
const int BTN_PRESSION_PEP_PLUS = 215;
const int BTN_PRESSION_PLATEAU_MINUS = 109;
const int BTN_PRESSION_PLATEAU_PLUS = 0;

// contrôle de l'écran LCD
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// #define LCD_20_CHARS              // commenter pour utiliser un écran LCD 16 caratères
const int LCD_UPDATE_PERIOD = 20; // période (en centièmes de secondes) de mise à jour du feedback des consignes sur le LCD

// phases possibles du cycle
// pousée d'inspiration : on envoie l'air jusqu'à la pression crête paramétrée : valve blower ouverte à consigne, flux d'air vers le patient
#define PHASE_PUSH_INSPI 1
// plateau d'inspiration : on a depassé la pression crête, la pression descend depuis plus d'un 1/10sec (arbitraire EFL) : 2 valves fermées
#define PHASE_HOLD_INSPI 2
// expiration : flux d'air vers l'extérieur, air patient vers l'extérieur
#define PHASE_EXPIRATION 3
// expiration bloquée : les valves sont fermées car la pression est en dessous de la PEP
#define PHASE_HOLD_EXPI 4

// minimums et maximums possible des paramètres modifiables à l'exécution
const int BORNE_SUP_PRESSION_CRETE = 70;   // arbitraire
const int BORNE_INF_PRESSION_CRETE = 10;   // arbitraire
const int BORNE_SUP_PRESSION_PLATEAU = 30; // PP MAX SDRA = 30
const int BORNE_INF_PRESSION_PLATEAU = 10; // arbitraire
const int BORNE_SUP_PRESSION_PEP = 30;     // PP MAX = 30, or PEP < PP
const int BORNE_INF_PRESSION_PEP = 5;      // arbitraire mais > 0

const int BORNE_SUP_CYCLE = 35; // demande medical
const int BORNE_INF_CYCLE = 5;  // demande medical

// durée d'appui des boutons (en centièmes de secondes) avant prise en compte
const int MAINTIEN_PARAMETRAGE = 21;

// valeurs de sécurité pour les actionneurs
int secu_coupureBlower = 45;
int secu_ouvertureExpi = 45;

// nombre de cycles par minute (cycle = inspi + plateau + expi)
int consigneNbCycle = 20;
int futureConsigneNbCycle = consigneNbCycle;

// degré d'ouverture de la valve blower (quantité d'air du blower qu'on envoie vers le Air Transistor patient)
int consigneOuverture = 45;
int futureConsigneOuverture = consigneOuverture;

// consigne de pression de crête maximum
int consignePressionCrete = 75;

// consigne de pression plateau maximum
int consignePressionPlateauMax = 40;
int futureConsignePressionPlateauMax = consignePressionPlateauMax;

// consigne de pression PEP
int consignePressionPEP = 10;
int futureConsignePressionPEP = consignePressionPEP;

// données pour affichage (du cycle précédent pour ne pas afficher des valeurs aberrantes)
int previousPressionCrete = -1;
int previousPressionPlateau = -1;
int previousPressionPep = -1;

void onFree2()
{
#ifdef DEBUG
  Serial.println("free2");
#endif
}

void onFree1()
{
#ifdef DEBUG
  Serial.println("free1");
#endif
}

void onAlarmOff()
{
#ifdef DEBUG
  Serial.println("alarm OFF");
#endif
  digitalWrite(BUZZER_PIN, LOW);
}

void onAlarmOn()
{
#ifdef DEBUG
  Serial.println("alarm ON");
#endif
  // digitalWrite(BUZZER_PIN, HIGH);
}

void onCycleMinus()
{
#ifdef DEBUG
  Serial.println("nb cycle --");
#endif
  futureConsigneNbCycle--;
  if (futureConsigneNbCycle < BORNE_INF_CYCLE)
  {
    futureConsigneNbCycle = BORNE_INF_CYCLE;
  }
}

void onCyclePlus()
{
#ifdef DEBUG
  Serial.println("nb cycle ++");
#endif
  futureConsigneNbCycle++;
  if (futureConsigneNbCycle > BORNE_SUP_CYCLE)
  {
    futureConsigneNbCycle = BORNE_SUP_CYCLE;
  }
}

void onPressionPepMinus()
{
#ifdef DEBUG
  Serial.println("pression PEP --");
#endif
  futureConsignePressionPEP--;
  if (futureConsignePressionPEP < BORNE_INF_PRESSION_PEP)
  {
    futureConsignePressionPEP = BORNE_INF_PRESSION_PEP;
  }
}

void onPressionPepPlus()
{
#ifdef DEBUG
  Serial.println("pression PEP ++");
#endif
  futureConsignePressionPEP++;
  if (futureConsignePressionPEP > BORNE_SUP_PRESSION_PEP)
  {
    futureConsignePressionPEP = BORNE_SUP_PRESSION_PEP;
  }
}

void onPressionPlateauMinus()
{
#ifdef DEBUG
  Serial.println("pression plateau --");
#endif
  futureConsignePressionPlateauMax--;
  if (futureConsignePressionPlateauMax < BORNE_INF_PRESSION_PLATEAU)
  {
    futureConsignePressionPlateauMax = BORNE_INF_PRESSION_PLATEAU;
  }
}

void onPressionPlateauPlus()
{
#ifdef DEBUG
  Serial.println("pression plateau ++");
#endif
  futureConsignePressionPlateauMax++;
  if (futureConsignePressionPlateauMax > BORNE_SUP_PRESSION_PLATEAU)
  {
    futureConsignePressionPlateauMax = BORNE_SUP_PRESSION_PLATEAU;
  }
}

// boutons
AnalogButtons analogButtons(ANALOG_PIN, INPUT);
Button btnFree2(BTN_FREE2, &onFree2);
Button btnFree1(BTN_FREE1, &onFree1);
Button btnAlarmOff(BTN_ALARM_OFF, &onAlarmOff);
Button btnAlarmOn(BTN_ALARM_ON, &onAlarmOn);
Button btnCycleMinus(BTN_CYCLE_MINUS, &onCycleMinus);
Button btnCyclePlus(BTN_CYCLE_PLUS, &onCyclePlus);
Button btnPressionPepMinus(BTN_PRESSION_PEP_MINUS, &onPressionPepMinus);
Button btnPressionPepPlus(BTN_PRESSION_PEP_PLUS, &onPressionPepPlus);
Button btnPressionPlateauMinus(BTN_PRESSION_PLATEAU_MINUS, &onPressionPlateauMinus);
Button btnPressionPlateauPlus(BTN_PRESSION_PLATEAU_PLUS, &onPressionPlateauPlus);

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("demarrage");
#endif
  patient.attach(PIN_SERVO_PATIENT);
  blower.attach(PIN_SERVO_BLOWER);

#ifdef DEBUG
  Serial.print("mise en secu initiale");
#endif
  blower.write(secu_coupureBlower);
  patient.write(secu_ouvertureExpi);

#ifdef LCD_20_CHARS
  lcd.begin(20, 2);
#else
  lcd.begin(16, 2);
#endif

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_CAPTEUR_PRESSION, INPUT);

  analogButtons.add(btnFree2);
  analogButtons.add(btnFree1);
  analogButtons.add(btnAlarmOff);
  analogButtons.add(btnAlarmOn);
  analogButtons.add(btnCycleMinus);
  analogButtons.add(btnCyclePlus);
  analogButtons.add(btnPressionPepMinus);
  analogButtons.add(btnPressionPepPlus);
  analogButtons.add(btnPressionPlateauMinus);
  analogButtons.add(btnPressionPlateauPlus);
}

/*
//Call this in loop() to help find analog values of buttons
void calibrateButtons() {
	unsigned int value = analogRead(ANALOG_PIN);
	Serial.println(value);
	delay(250);
}
*/

void loop()
{
  int nbreCentiemeSecParCycle = 60 * 100 / consigneNbCycle;
  int nbreCentiemeSecParInspi = nbreCentiemeSecParCycle / 3; // inspiration = 1/3 du cycle, expiration = 2/3 du cycle

#ifdef DEBUG
  Serial.println();
  Serial.println("------ Starting cycle ------");
  Serial.print("nbreCentiemeSecParCycle = ");
  Serial.println(nbreCentiemeSecParCycle);
  Serial.print("nbreCentiemeSecParInspi = ");
  Serial.println(nbreCentiemeSecParInspi);
#endif

  int currentPressionCrete = -1;
  int currentPressionPlateau = -1;
  int currentPressionPep = -1;

  //int currentPositionBlower = secu_coupureBlower;

  int dureeBaissePression = 0; // compteur de centièmes pour la détection du pic de pression (pression crête)

  // phase courante du cycle
  int currentPhase = PHASE_PUSH_INSPI;

  //
  bool isAlarm = false;

  // état des actionneurs au tick précédent
  int positionBlower = 90;
  int positionPatient = 90;

  // nouvelles consignes pour les actionneurs
  int consigneBlower = 90;
  int consignePatient = 90;

  consigneNbCycle = futureConsigneNbCycle;
  consigneOuverture = futureConsigneOuverture;
  consignePressionPEP = futureConsignePressionPEP;
  consignePressionPlateauMax = futureConsignePressionPlateauMax;
#ifdef DEBUG
  Serial.print("consigneNbCycle = ");
  Serial.println(consigneNbCycle);
  Serial.print("consigneOuverture = ");
  Serial.println(consigneOuverture);
  Serial.print("consignePressionPEP = ");
  Serial.println(consignePressionPEP);
  Serial.print("consignePressionPlateauMax = ");
  Serial.println(consignePressionPlateauMax);
#endif

  /********************************************/
  // Affichage une fois par cycle respiratoire
  /********************************************/
  lcd.setCursor(0, 0);
#ifdef LCD_20_CHARS
  lcd.print("pc=");
  lcd.print(previousPressionCrete);
  lcd.print("/pp=");
  lcd.print(previousPressionPlateau);
  lcd.print("/pep=");
  lcd.print(previousPressionPep);
  lcd.print("  ");
#else
  lcd.print("pc");
  lcd.print(previousPressionCrete);
  lcd.print("/pp");
  lcd.print(previousPressionPlateau);
  lcd.print("/pep");
  lcd.print(previousPressionPep);
  lcd.print("  ");
#endif

  /********************************************/
  // Début d'un cycle
  /********************************************/
  int currentCentieme = 0;

  while (currentCentieme < nbreCentiemeSecParCycle)
  {

    static uint32_t dateDernierTraitement = 0ul;
    uint32_t dateCourante = millis();
    if (dateCourante - dateDernierTraitement >= PERIODE_DE_TRAITEMENT)
    {

      currentCentieme++;
      /********************************************/
      // Le traitement est effectué toutes les
      // PERIODE_DE_TRAITEMENT millisecondes
      // Note sur la gestion du temps. La date en
      // millisecondes de l'Arduino déborde tous
      // les 40 jours environ. Il faut donc comparer
      // des intervalles et non des dates absolues
      // Sinon, au bout de 40 jours, ça plante.
      /********************************************/
      dateDernierTraitement = dateCourante;
/********************************************/
// Mesure pression pour rétro-action
/********************************************/
#ifdef SIMULATION
      int currentPression = 0;
      if (currentCentieme < 50)
      {
        currentPression = 60;
      }
      else
      {
        currentPression = 30;
      }
      if (currentCentieme > nbreCentiemeSecParInspi)
      {
        currentPression = 5;
      }
#else
      int currentPression = map(analogRead(PIN_CAPTEUR_PRESSION), 194, 245, 0, 600) / 10;
#endif

      /********************************************/
      // Calcul des consignes normales
      /********************************************/
      if (currentCentieme <= nbreCentiemeSecParInspi)
      { // on est dans la phase temporelle d'inspiration (poussée puis plateau)
        // if (currentPression >= currentPressionCrete) {
        if (currentCentieme <= (nbreCentiemeSecParInspi / 2))
        { // on est dans la phase temporelle d'inspiration (poussée puis plateau)
          currentPhase = PHASE_PUSH_INSPI;
          currentPressionCrete = currentPression;

          consigneBlower = BLOWER_CLOSED;
          consignePatient = PATIENT_IN; // on ouvre le flux IN patient
        }
        else
        {
          currentPhase = PHASE_HOLD_INSPI;
          currentPressionPlateau = currentPression;

          consigneBlower = BLOWER_PLATEAU;   // on shunt vers l'extérieur
          consignePatient = PATIENT_PLATEAU; // on bloque les flux patient
        }
      }
      else
      { // on gère l'expiration on est phase PHASE_EXPIRATION
        currentPhase = PHASE_EXPIRATION;
        currentPressionPep = currentPression;
        consigneBlower = BLOWER_OPEN;  // on shunt vers l'extérieur
        consignePatient = PATIENT_OUT; // on ouvre le flux OUT patient (expiration vers l'extérieur)
      }

      /********************************************/
      // Calcul des consignes de mise en sécurité
      /********************************************/
      // si pression crête > max, alors fermeture blower de 2°
      if (currentPression > consignePressionCrete)
      {
#ifdef DEBUG_VERBOSE
        if (currentCentieme % 80)
        {
          Serial.println("Mise en securite : pression crete trop importante");
        }
#endif
        consigneBlower = positionBlower - 2;
      }
      // si pression plateau > consigne param, alors ouverture expiration de 1°
      if (currentPhase == PHASE_HOLD_INSPI && currentPression > consignePressionPlateauMax)
      {
#ifdef DEBUG_VERBOSE
        if (currentCentieme % 80)
        {
          Serial.println("Mise en securite : pression plateau trop importante");
        }
#endif
        consignePatient = positionBlower + 1;
      }
      // si pression PEP < PEP mini, alors fermeture complète valve expiration
      if (currentPression < consignePressionPEP)
      {
#ifdef DEBUG_VERBOSE
        if (currentCentieme % 80)
        {
          Serial.println("Mise en securite : pression d'expiration positive (PEP) trop faible");
        }
#endif
        consignePatient = 90;
        currentPhase = PHASE_HOLD_EXPI;
      }

#ifdef DEBUG
      if (currentCentieme % 50 == 0)
      {
        Serial.print("Phase : ");
        Serial.println(currentPhase);
        Serial.print("Pression : ");
        Serial.println(currentPression);
      }
#endif

      /********************************************/
      // Envoi des nouvelles valeurs aux actionneurs
      /********************************************/
      if (consigneBlower != positionBlower)
      {
        blower.write(consigneBlower);
        positionBlower = consigneBlower;
      }

      if (consignePatient != positionPatient)
      {
        patient.write(consignePatient);
        positionPatient = consignePatient;
      }

      /********************************************/
      // Préparation des valeurs pour affichage
      /********************************************/
      previousPressionCrete = currentPressionCrete;
      previousPressionPlateau = currentPressionPlateau;
      previousPressionPep = currentPressionPep;

      /********************************************/
      // Écoute des appuis boutons
      /********************************************/
      analogButtons.check();
      //calibrateButtons();

      /********************************************/
      // Affichage pendant le cycle
      /********************************************/
      if (currentCentieme % LCD_UPDATE_PERIOD == 0)
      {
        lcd.setCursor(0, 1);
#ifdef LCD_20_CHARS
        lcd.print("c");
        lcd.print(futureConsigneNbCycle);
        lcd.print("/pl=");
        lcd.print(futureConsignePressionPlateauMax);
        lcd.print("/pep=");
        lcd.print(futureConsignePressionPEP);
        lcd.print("|");
        lcd.print(currentPression);
#else
        // lcd.print("c");
        // lcd.print(futureConsigneNbCycle);
        // lcd.print("/pl");
        // lcd.print(futureConsignePressionPlateauMax);
        // lcd.print("/pep");
        // lcd.print(futureConsignePressionPEP);
        // lcd.print("|");
        lcd.print(analogRead(PIN_CAPTEUR_PRESSION));
#endif
      }
    }
  }
}
/********************************************/
// Fin du cycle
/********************************************/