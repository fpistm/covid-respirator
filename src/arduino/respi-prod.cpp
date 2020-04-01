/*=============================================================================
 * @file respi-prod.h
 *
 * COVID Respirator
 *
 * @section copyright Copyright
 *
 * Makers For Life
 *
 * @section descr File description
 *
 * This file execute the Makair program
 */

// INCLUDES ==================================================================

// External
#include <AnalogButtons.h>
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// Internal
#include "affichage.h"
#include "clavier.h"
#include "common.h"
#include "config.h"
#include "debug.h"
#include "parameters.h"
#include "pressure_controller.h"
#include "pression.h"

// PROGRAM =====================================================================

AirTransistor servoBlower;
AirTransistor servoY;
AirTransistor servoPatient;
void setup()
{
    pinMode(PIN_CAPTEUR_PRESSION, INPUT);

    DBG_DO(Serial.begin(115200);)
    DBG_DO(Serial.println("demarrage");)

    servoBlower = AirTransistor(
        BLOWER_FERME,
        BLOWER_OUVERT,
        BLOWER_FERME,
        BLOWER_OUVERT
    );

    servoY = AirTransistor(
        BLOWER_FERME,
        BLOWER_OUVERT,
        BLOWER_FERME,
        BLOWER_OUVERT
    );

    servoPatient = AirTransistor(
        PATIENT_FERME,
        PATIENT_OUVERT,
        PATIENT_FERME,
        PATIENT_FERME
    );

    pController = PressureController(
            INITIAL_CYCLE_NB,
            DEFAULT_MIN_PEEP_COMMAND,
            BORNE_SUP_PRESSION_PLATEAU,
            ANGLE_OUVERTURE_MAXI,
            BORNE_SUP_PRESSION_CRETE,
            servoBlower,
            servoY,
            servoPatient
    );
    pController.setup();
    startScreen();
    initKeyboard();
}

void loop()
{
    /********************************************/
    // INITIALIZE THE RESPIRATORY CYCLE
    /********************************************/

    pController.initRespiratoryCycle();

    /********************************************/
    // START THE RESPIRATORY CYCLE
    /********************************************/
    uint16_t centiSec = 0;

    while (centiSec < pController.centiSecPerCycle())
    {
        static uint32_t lastpControllerComputeDate = 0ul;
        uint32_t currentDate = millis();
        if (currentDate - lastpControllerComputeDate >= PCONTROLLER_COMPUTE_PERIOD)
        {
            lastpControllerComputeDate = currentDate;

            pController.updatePressure(readPressureSensor(centiSec));

            // Perform the pressure control
            pController.compute(centiSec);

            // Check if some buttons have been pushed
            keyboardLoop();

            // Display relevant information during the cycle
            if (centiSec % LCD_UPDATE_PERIOD == 0)
            {
                displaySubPhase(pController.subPhase());

                displayEveryRespiratoryCycle(
                                pController.peakPressure(),
                                pController.plateauPressure(),
                                pController.peep(),
                                pController.pressure());

                displayDuringCycle(0,
                                pController.maxPlateauPressureCommand(),
                                pController.minPeepCommand(),
                                pController.cyclesPerMinuteCommand()
                );
            }

            // next tick
            centiSec++;
        }
    }
    /********************************************/
    // END OF THE RESPIRATORY CYCLE
    /********************************************/
}
