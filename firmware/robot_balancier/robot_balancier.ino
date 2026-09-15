/* =====================================================================
   ROBOT BALANCIER - ESP32 WROOM-32D + MPU6050 + 2x A4988 + NEMA17
   ---------------------------------------------------------------------
   ARCHITECTURE (changement principal vs version precedente)

     Boucle INTERNE  (200 Hz, cadencee par l INT du MPU)
         accel_roue [pas/s2] = Kp*err_angle + Ki*integ_angle + Kd*gyro
         vitesse_roue       += accel_roue * dt        <-- INTEGRATEUR
     Boucle EXTERNE  (40 Hz)
         angle_cible = angleOffset - ( Kp_v*err_vitesse + Ki_v*err_position )
     AUTO-TRIM (tres lent)
         recale angleOffset pour que la correction externe tende vers 0

   Pourquoi : pour un pendule inverse c est l ACCELERATION du chariot qui
   redresse le corps (theta'' = (g/L)*theta - a/L). Commander une vitesse
   proportionnelle a l angle ne peut pas stabiliser : la boucle fermee
   garde toujours un pole reel positif.

   Compatible core ESP32 Arduino 2.x ET 3.x (voir les macros TIMER_* plus bas)
   ===================================================================== */

#include <Wire.h>
#include <Preferences.h>
#include "soc/gpio_reg.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

// ------------------------------------------ POLITIQUE APPRISE (agent RL)
// 1 634 poids en flash, deux couches de 32, activation tanh. Genere par
// 11_export_c.py et verifie contre PyTorch a 2e-7 pres par 12_valider_c.py.
// Elle remplace la boucle interne ET la boucle externe : elle a deja l ecart
// de position et la consigne de vitesse dans ses entrees.
// Mode d emploi complet : firmware/INTEGRATION.md.
#include "politique.h"

bool useAgent  = false;   // false = la cascade reglee a la main, true = l agent
bool testSigne = false;   // verification roues en l air, moteurs muets
unsigned long lastSigneMsg = 0;

// ---------------------------------------------- SECOUSSE (cmd '4')
// Reproduit l agitation de l agent -- inversions a pleine amplitude a 5 Hz --
// SANS l agent, SANS l IMU et SANS aucune boucle d asservissement. Si la carte
// plante aussi dans ce mode, la cause est le chemin moteur ; si elle tient,
// c est l agent ou sa lecture de l IMU. Une seule experience, deux hypotheses
// separees.
bool testSecousse = false;
unsigned long secousseT0 = 0;
int   secousseSigne = 1;
float SECOUSSE_AMP = 1400.0f;          // pas/s, l amplitude relevee sur l agent
unsigned long SECOUSSE_MS = 100;       // demi-periode -> 5 Hz, sa frequence minimale

// ------------------------------------- BOITE NOIRE, survit au redemarrage
// La RAM RTC n est pas effacee par un reset logiciel. On y ecrit l etape en
// cours a chaque phase de loop(). Apres un plantage, le demarrage la relit :
// on sait OU la boucle s est arretee, ce qu aucune trace serie ne peut dire
// puisque justement plus rien ne s imprime.
RTC_NOINIT_ATTR uint32_t rtcMagie;
RTC_NOINIT_ATTR uint32_t rtcEtape;
RTC_NOINIT_ATTR int32_t  rtcSps;
RTC_NOINIT_ATTR uint32_t rtcHalf;
// Marqueur propre a la tache IMU. Sans lui, un plantage residuel serait
// indiscernable : on saurait que loop() allait bien sans savoir ou en etait
// la lecture.
RTC_NOINIT_ATTR uint32_t rtcEtapeImu;
#define ETAPE_IMU(n) (rtcEtapeImu = (n))
static const char *nomEtapeImu(uint32_t e) {
  switch (e) {
    case 1: return "en attente du reveil";
    case 2: return "ecriture du registre 0x3D";
    case 3: return "requestFrom, lecture des 8 octets";
    case 4: return "extraction et publication";
    default: return "inconnue";
  }
}
#define RTC_MAGIE 0xBA1A0CE5u
#define ETAPE(n)  (rtcEtape = (n))
static const char *nomEtape(uint32_t e) {
  switch (e) {
    case  1: return "pollSerial";
    case  2: return "garde-fou IMU";
    case  3: return "transaction I2C";
    case  4: return "filtres et securites";
    case  5: return "boucle externe";
    case  6: return "boucle interne (politique comprise)";
    case  7: return "applyMotorSpeed";
    case  8: return "mode manuel";
    case  9: return "affinage du biais gyro";
    case 10: return "fin de deplacement minute";
    case 11: return "test de signe";
    case 12: return "secousse";
    case 13: return "sauvegarde des reglages";
    case 14: return "telemetrie";
    case 15: return "fin de boucle";
    case 16: return "surveillance du bus I2C";
    default: return "inconnue";
  }
}

// ------------------------------------------- COMPAT CORE ESP32 2.x / 3.x
// Le core 3.x a supprime timerAlarmWrite / timerAlarmEnable et change la
// signature de timerBegin. Ces macros donnent la meme API dans les 2 cas.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define TIMER_BEGIN_1MHZ()        timerBegin(1000000)
  #define TIMER_ATTACH(t, fn)       timerAttachInterrupt((t), (fn))
  #define TIMER_SET_PERIOD(t, us)   timerAlarm((t), (us), true, 0)
  #define TIMER_ENABLE(t)           ((void)0)
#else
  #define TIMER_BEGIN_1MHZ()        timerBegin(0, 80, true)
  #define TIMER_ATTACH(t, fn)       timerAttachInterrupt((t), (fn), true)
  #define TIMER_SET_PERIOD(t, us)   timerAlarmWrite((t), (us), true)
  #define TIMER_ENABLE(t)           timerAlarmEnable((t))
#endif

// ------------------------------------------------------------------ PINS
const int DIR_M1  = 5;
const int STEP_M1 = 18;
const int DIR_M2  = 17;
const int STEP_M2 = 16;
const int MPU_INT_PIN = 19;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint8_t MPU_ADDR = 0x68;

// Si un moteur est monte en miroir, passe son flag a true.
const bool INVERT_M1 = false;
const bool INVERT_M2 = false;

// 400 kHz s est revele trop rapide pour le cablage : NACK a repetition.
const uint32_t I2C_FREQ = 100000;

// --------------------------------------------------------- MECA / LIMITES
const int   STEPS_PER_REV = 1600;          // 200 * 1/8 de pas
float MAX_SPS        = 3000.0f;            // plafond vitesse roue  (cmd 'M')
const float MIN_SPS  = 6.0f;               // en dessous : moteur au repos
float MAX_ACCEL      = 25000.0f;           // saturation sortie PID interne (cmd 'R')
// Le couple d un pas-a-pas s effondre avec la vitesse. Si on demande la
// meme acceleration a 3000 pas/s qu a l arret, le moteur DECROCHE : il
// vibre sans tourner, le couple tombe a zero et le robot part au sol.
// A pleine vitesse on n autorise plus que (1 - ACCEL_DERATE) de l accel.
const float ACCEL_DERATE = 0.60f;
const float MAX_TILT_CORR = 6.0f;          // deg, autorite max boucle externe
const float MAX_POS_ERR   = 30000.0f;      // pas, saturation terme position
float MAX_LEAD            = 1600.0f;       // pas, avance max de la cible (cmd 'E')
const float FALL_LIMIT    = 30.0f;         // deg -> coupure
const float ARM_LIMIT     = 5.0f;          // deg -> autorise l armement

// ------------------------------------------------------------ FILTRAGE
const float TAU_COMP  = 1.0f;              // s, filtre complementaire
const float TAU_GYRO  = 0.012f;            // s, lissage du gyro pour le terme D
// Lissage de la vitesse dans la boucle externe (cmd 'L').
// A 0.15 s la coupure tombait a 1.06 Hz, soit exactement la frequence propre
// du pendule (mesuree a 1.35 Hz => centre de masse a ~13.6 cm de l axe).
// La boucle externe entretenait le mode propre au lieu de l ignorer.
// A 0.40 s la coupure descend a 0.40 Hz : elle laisse le pendule tranquille.
float TAU_SPEED = 0.40f;                   // s, lissage vitesse boucle externe
const int   OUTER_DIV = 5;                 // boucle externe = 200/5 = 40 Hz

// --------------------------------------------------------------- MOTEUR
hw_timer_t *motorTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

const uint32_t STEP_MASK   = (1UL << STEP_M1) | (1UL << STEP_M2);
const uint32_t STEP_MASK_D = (1UL << STEP_M1);
const uint32_t STEP_MASK_G = (1UL << STEP_M2);

// ===================================================== DEUX ROUES SEPAREES
// Ancienne version : un timer cadence a la demi-periode, les deux STEP sur le
// meme masque. Consequence, les deux roues recevaient rigoureusement les memes
// impulsions et le robot ne pouvait PAS tourner. C etait la limite ouverte
// depuis le debut du projet.
//
// Nouvelle version : le timer bat a cadence FIXE, et chaque moteur porte son
// propre accumulateur en virgule fixe 16.16. A chaque top on ajoute son
// increment ; quand l accumulateur deborde, ce moteur-la emet un pas. C est un
// Bresenham : deux vitesses independantes sur un seul timer, sans derive, et
// avec une erreur de phase bornee a un pas.
//
// Effet de bord bienvenu : la demi-periode ne se reprogramme plus jamais. On
// avait mesure 186 reprogrammations par seconde le 9 septembre en cherchant la
// cause d un plantage -- il n y en a plus aucune.
const uint32_t F_ISR         = 25000;               // Hz
const uint32_t ISR_PERIODE_US = 1000000UL / F_ISR;  // 40 us
// Un pas occupe un top pour monter et le suivant pour redescendre. En bornant
// l increment a la moitie de l echelle, on garantit au moins un top de repos
// entre deux pas, donc une impulsion propre de 40 us -- tres au-dessus du
// microseconde exigee par l A4988. Plafond : 12 500 pas/s par roue.
const uint32_t INCR_MAX      = 32768u;

volatile bool     motorRunning  = false;
volatile uint32_t accu[2]       = {0, 0};   // 16.16
volatile uint32_t incrPas[2]    = {0, 0};   // sps * 65536 / F_ISR
volatile int8_t   dirSigne[2]   = {1, 1};
volatile int32_t  pasFaits[2]   = {0, 0};   // odometrie, par roue
volatile uint32_t masqueImpulsion = 0;      // a faire retomber au top suivant
volatile uint32_t halfPeriod_us = ISR_PERIODE_US;   // conserve pour la trace

// HOMME MORT. L ISR compte ses propres declenchements ; la boucle de commande
// remet le compteur a zero a chaque mise a jour. Si la boucle se bloque, l ISR
// continue seule et le robot part a la derniere vitesse connue -- c est ce qu on
// voit sur les videos de panne. Passe ce seuil, l ISR s arrete d elle-meme.
// L ISR bat maintenant a 25 kHz quelle que soit la vitesse, et la commande est
// rafraichie 200 fois par seconde, soit 125 tops entre deux mises a jour.
// 1250 laisse un facteur 10, pour 50 ms de securite.
volatile uint32_t isrTicks = 0;
const uint32_t DEADMAN_TICKS = 1250;

// ------------------------------------------------------------- ETAT IMU
float pitch = 0.0f;
float gyroRate = 0.0f;                     // deg/s brut, axe X
float gyroFilt = 0.0f;                     // deg/s filtre, utilise par le terme D
float gyroX_offset = 0.0f;
unsigned long lastMeasureTime = 0;
unsigned long lastDisplayTime = 0;
volatile bool mpuDataReady = false;

// ------------------------------------------- ECHANTILLON IMU PARTAGE
// La lecture I2C vit maintenant dans sa propre tache, sur le coeur 0. Elle
// publie ici ; la boucle de commande, sur le coeur 1, consomme.
//
// Pourquoi : la boite noire du 9 septembre a montre quatre plantages sur
// quatre bloques dans la transaction I2C -- en secousse, en cascade et avec
// l agent, donc independamment de ce qui commande. Tant que la lecture etait
// dans loop(), la boucle de commande mourait avec elle et la carte redemarrait
// 5 s plus tard, robot par terre. Separee, elle survit et peut couper.
//
// La tache IMU n est VOLONTAIREMENT PAS abonnee au chien de garde de tache :
// si elle se bloque, elle ne doit pas faire redemarrer la carte. C est la
// boucle de commande qui constate le silence et desarme.
struct EchantillonImu {
  uint32_t seq;      // incremente a chaque publication : le consommateur compare
  uint32_t t_us;     // date de la transaction
  uint32_t d_us;     // sa duree
  int16_t  ay, az, gx;
  bool     ok;
};
static volatile EchantillonImu imuEch = {0, 0, 0, 0, 0, 0, false};
static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t hTacheImu = NULL;
static uint32_t seqTraitee = 0;
uint32_t i2cBloc = 0;              // nombre de fois ou le bus s est fige
uint32_t silenceDepuis = 0;        // date du debut du silence, 0 si le bus va bien

// ==================================================== ENREGISTREUR 200 Hz
// La telemetrie sort a 10 Hz. C est assez pour surveiller, beaucoup trop lent
// pour identifier : la constante de temps du pendule inverse est predite a
// 132 ms, et l agent oscille au-dela de 5 Hz. On capture donc en RAM a la
// cadence de la boucle, puis on vide tranquillement sur le port serie.
//
// 1200 echantillons a 200 Hz = 6,0 s. Resolution spectrale 0,17 Hz, jusqu a
// 100 Hz. 33 ko sur les 304 ko libres.
#define ENR_N 1200
struct Echantillon {
  uint32_t t;        // us depuis le debut de la capture
  float    p;        // tangage brut, deg
  float    gf;       // gyro filtre, deg/s -- ce que voit la commande
  float    gb;       // gyro brut,   deg/s -- ce qu il faut pour le spectre
  float    s;        // vitesse commandee, pas/s
  float    inj;      // acceleration injectee, pas/s2 (0 hors balayage)
  float    pa;       // angle vu par l ACCELEROMETRE seul, deg
  float    mg;       // module de l acceleration, en g (1.000 au repos)
  int32_t  q;        // position, pas
  uint8_t  arme;     // 1 si la commande pilote, 0 si roues bloquees
};
static Echantillon enr[ENR_N];
static uint16_t enrN = 0;          // combien d echantillons ecrits
static bool     enrActif = false;
static bool     enrVidage = false;
static uint16_t enrVide = 0;       // ou en est le vidage
static uint32_t enrT0 = 0;
static const char *enrNom = "";

// Essai de lacher : on fige les roues un court instant pendant que le robot
// tient debout, et on regarde diverger. Aucune manipulation, aucune usure.
static bool     lacherActif = false;
static uint32_t lacherFin = 0;
// Trois lachers dans UNE seule capture, de durees croissantes.
//
// Un seul lacher de 145 ms n a pas suffi : sur un intervalle aussi court le
// cosinus hyperbolique est indiscernable d une parabole, et l ajustement a
// donne tau entre 0,22 et 0,60 s selon la fenetre. Il faut t/tau >= 2, donc
// au moins 400 ms si tau vaut 0,2 s.
//
// 400 ms est aussi la limite prudente : SI le modele avait raison avec
// tau = 0,14 s, l angle atteindrait 4,4 deg, ce que la cascade rattrape
// (6 essais sur 6 a 10 deg). A 500 ms elle serait a 9 deg, trop pres du bord.
//
// Et les trois d un coup, c est une seule chute au vidage au lieu de trois.
// Le lacher depuis l equilibre a echoue : le robot part de 0,2 deg et le gyro
// a 2 deg/s de bruit, donc le signal ne sort du bruit que sur les 100 dernieres
// millisecondes. tau n est identifiable qu entre 0,11 et 0,17 s.
//
// Nouveau protocole, en deux temps.
//   POUSSEE  : on impose une acceleration de roue CONNUE, hors asservissement.
//              Le robot part en arriere de plusieurs degres. Bonus : la reponse
//              a une entree connue donne aussi le gain d actionnement, l autre
//              chiffre qui manque au modele.
//   GEL      : roues bloquees. La divergence part alors d un angle franc, et
//              on la suit sur une decade au lieu d un facteur 3.
//
// Le gel s arrete des que l angle depasse LACHER_ANGLE_MAX, pas apres une duree
// fixe : ainsi l excursion est toujours grande ET toujours rattrapable, quelle
// que soit la vraie valeur de tau -- que justement on ne connait pas.
static const float    POUSSEE_ACCEL   = 10000.0f;  // pas/s2, entree connue
static const uint32_t POUSSEE_MS      = 150;
static const uint32_t LACHER_MAX_MS   = 600;
static const float    LACHER_ANGLE_MAX = 8.0f;     // deg, la cascade rattrape 10
// ---------------------------------------------- BALAYAGE EN FREQUENCE
// Le protocole poussee-puis-gel deplacait le robot de plusieurs dizaines de
// centimetres et le faisait tomber. Celui-ci ne fait ni l un ni l autre : la
// cascade reste aux commandes du debut a la fin, elle tient le robot debout ET
// le ramene a sa place. On se contente d AJOUTER une petite acceleration
// sinusoidale a sa sortie, dont la frequence monte de 0,5 a 20 Hz en 5 s.
//
// La reponse de l angle a cette entree connue, frequence par frequence, donne
// la dynamique complete : la constante de temps du pendule ET la bande passante
// reelle du moteur. C est cette derniere qui manque au modele, et c est elle
// qui explique que l agent s agite.
//
// 2000 pas/s2 est petit devant les 25000 disponibles : la cascade encaisse.
static const float BALAYAGE_AMP = 2000.0f;
static const float BALAYAGE_F0  = 0.5f;
static const float BALAYAGE_F1  = 20.0f;
static const float BALAYAGE_DUREE = 5.0f;
// Second balayage, sur la CONSIGNE D ANGLE et non sur l acceleration.
//
// Pourquoi il en faut un deuxieme : le terme de gravite ne pese que sous
// 1/(2 pi tau) = 1,14 Hz, et c est precisement la que la boucle externe rejette
// la perturbation d acceleration -- coherence 0,17 mesuree contre 0,90 a 4 Hz.
// La boucle efface le signal qu on cherche.
//
// En bougeant la CONSIGNE, on ne lutte plus contre l asservissement : on s en
// sert. Il incline activement le robot, l angle devient grand et propre a
// basse frequence, et la relation entre l inclinaison et l acceleration de roue
// revele la dynamique.
//
// Et ca ne deplace presque rien. Tenir 1 deg d inclinaison demande
// a = g*theta = 0,17 m/s2 ; a 0,5 Hz cela ne fait que 17 mm d excursion.
static const float BALANGLE_AMP   = 1.0f;    // deg
static const float BALANGLE_F0    = 0.2f;
static const float BALANGLE_F1    = 3.0f;
static const float BALANGLE_DUREE = 5.5f;
// ------------------------------------------------------ ESSAI D AVANCE
// Un echelon de vitesse commande, de duree connue, enregistre a 200 Hz. C est
// le seul regime OU LE ROBOT QUITTE SON POINT D EQUILIBRE de facon controlee :
// pour avancer il doit d abord se pencher en arriere, puis se redresser. Le
// modele doit reproduire ce transitoire-la, pas seulement l equilibre.
//
// 400 pas/s pendant 2 s = 800 pas = 102 mm. Il repart ensuite a sa place.
// ------------------------------------------------- ESSAI DE DECROCHAGE
// Roues en l air, robot tenu a la main. On fait monter la vitesse commandee
// lentement jusqu a ce que le pas-a-pas decroche. Le decrochage s entend, et
// il se voit dans le gyro : la vibration reguliere du pas devient un
// battement desordonne des que le rotor perd le champ.
//
// C est le seul chiffre qui manque au modele du moteur, et le firmware
// l anticipait deja : en mode manuel il autorise volontairement de depasser
// MAX_SPS, "c est justement le test qui sert a determiner MAX_SPS".
//
// 800 pas/s2 pendant 6 s -> jusqu a 4800 pas/s, soit 3 tours/s. Aucun risque :
// un pas-a-pas qui decroche fait du bruit et ne casse rien, et les roues
// tournent dans le vide.
static const float RAMPE_ACCEL = 800.0f;      // pas/s2
static bool     rampeActive = false;
static uint32_t rampeT0 = 0;
static const float    AVANCE_SPS = 400.0f;
static const uint32_t AVANCE_MS  = 2000;
static uint32_t avanceDebut = 0, avanceFin = 0;
static bool     balAngleActif = false;
static uint32_t balAngleT0 = 0;
static bool     balayageActif = false;
static uint32_t balayageT0 = 0;
static float    injCourant = 0.0f;
static bool     pousseeActive = false;
static float    pousseeSigne  = 1.0f;
static uint32_t pousseeFin    = 0;
static uint8_t  lacherIdx = 0;
static uint32_t lacherProchain = 0;
static uint32_t LACHER_MS = 150;
static uint32_t lacherArme = 0;    // conserve pour compatibilite, non utilise

static void enrDemarrer(const char *nom) {
  enrN = 0; enrVide = 0; enrVidage = false;
  enrT0 = micros();
  enrNom = nom;
  enrActif = true;
}
uint32_t i2cReadOK = 0, i2cReadFail = 0, i2cRecover = 0;
uint8_t  i2cFailStreak = 0;
float loopHz = 0.0f;

// --------------------------------------------------- DIAGNOSTIC BLOCAGE
uint32_t maxLoopUs = 0;                    // pire duree d une iteration
uint32_t maxI2cUs  = 0;                    // pire duree d une transaction I2C
uint32_t maxPolUs  = 0;                    // pire duree d un appel a la politique
// Combien de fois, par fenetre de telemetrie, on reprogramme le timer de pas
// et on rallume le train d impulsions. C est la charge que l agent impose au
// pilote de timer, et l hypothese a mesurer pour le plantage du 9 septembre.
uint32_t nSetPeriod = 0;
uint32_t nRallumage = 0;

// ------------------------------------------------------- REGULATION
float angleOffset = -92.34f;               // angle d equilibre mecanique

// Boucle interne : angle -> acceleration roue
// Mesure du 19:39 : lors d une poussee, gyro a 17.7 deg/s et erreur 0.96 deg
// ne produisaient que 8190 pas/s2 sur les 25000 disponibles. Le robot
// reagissait trop tard et devait aller chercher trop de vitesse de pointe.
// Gains montes de 50 % pour exploiter la reserve d acceleration.
float Kp_a = 4500.0f;   // pas/s2 par degre        (etait 3000)
float Ki_a =   40.0f;   // pas/s2 par (degre*s)
float Kd_a =  450.0f;   // pas/s2 par (degre/s)    (etait 300)

// Boucle externe : vitesse/position roue -> angle cible
// Kp_spd/Ki_spd est le temps integral de la boucle de position. A 1,2 s il
// etait trop court : la boucle accumulait, depassait, et produisait un cycle
// limite de 10 s avec des a-coups de 900 pas/s. Porte a 4 s.
float Kp_spd = 0.002f;     // deg par (pas/s)
float Ki_spd = 0.0005f;    // deg par pas : 700 pas -> 0.35 deg de correction

float wheel_sps        = 0.0f;   // ETAT INTEGRATEUR = commande vitesse roue
float angleIntegral    = 0.0f;
float filtered_speed   = 0.0f;
float target_pos       = 0.0f;
float speed_angle_corr = 0.0f;
float target_angle     = 0.0f;
float target_speed_sps = 0.0f;
// Consigne de LACET, en pas/s de difference entre les deux roues.
// **Positif = le robot tourne a DROITE**, ce qui correspond au joystick poussee
// a droite. Le sens etait inverse au premier essai du 10 septembre : c est la
// roue GAUCHE qu il faut accelerer pour partir a droite.
// Elle n avait aucun sens tant que les deux STEP partageaient un timer ; elle
// en a un depuis que chaque roue a son accumulateur. Commande serie : '#'.
float target_lacet_sps = 0.0f;
bool  posFigee = false;              // la cible de position est-elle latchee ?
const float LACET_MAX = 2000.0f;     // releve de 600 : Matthieu veut voir loin
int   speed_sign       = 1;    // +1 demontre correct. 'X' pour tester -1.
int   outerCounter     = 0;

bool  pidEnabled = false;
bool  dualPid    = false;
bool  manualMode = false;              // commande 'V', avec rampe
float manual_target_sps = 0.0f;
float MANUAL_ACCEL = 8000.0f;          // pas/s2, rampe du mode V (cmd 'B')
bool autoTrim   = true;
const float AUTOTRIM_RATE = 0.05f;   // 1/s : 3x plus lent que la boucle de position
float angleOffsetInit = 0.0f;

// Effacement progressif de la boucle externe pendant une perturbation
const float OUTER_FADE_LO = 3.0f;    // deg : autorite pleine en dessous
const float OUTER_FADE_HI = 6.0f;    // deg : autorite nulle au dessus

// Armement automatique. Un seul mecanisme sert deux usages :
//   - 'J' : le robot s arme en DUAL des le demarrage
//   - 'H' : il se re-arme tout seul apres une chute
// Dans les deux cas il attend d etre droit et immobile 1 s avant de partir.
bool autoRearm  = false;   // 'H'
bool startDual  = false;   // 'J'
bool pendingArm = false;   // armement en attente
bool wantDual   = false;   // mode a retrouver a l armement
unsigned long calmSince = 0;
unsigned long lastArmMsg = 0;

// Securite anti-emballement : une saturation de vitesse prolongee n est
// jamais un vrai rattrapage (celui-ci dure moins d une seconde). C est le
// robot en l air ou couche, roues qui tournent dans le vide -- sollicitation
// inutile des moteurs et des drivers, et c est ce qui a tue la carte a 20:24.
unsigned long satSince = 0;
const unsigned long SAT_MAX_MS = 1500;

// Deplacement minute (cmd 'W') : duree en secondes. Si elle est non nulle,
// un 'N' fait avancer le robot pendant cette duree puis remet la consigne
// a zero tout seul.
float travelDuration = 0.0f;
unsigned long travelUntil = 0;

// Coup de pouce au demarrage d un deplacement (cmd 'U'), en degres.
// Le robot ne demarre pas tant que l inclinaison n a pas vaincu le seuil
// statique (la surface de contact des pneus laisse ~2 mm de jeu). Faire
// monter l integrale depuis zero prend 3 s sur les 5 du deplacement.
// On precharge donc directement la cible de position de la quantite qui
// produit cette inclinaison : le robot part tout de suite, et l integrale
// se degonfle seule une fois qu il roule.
float startBoost = 1.2f;   // deg

// Aller-retour : un deplacement minute repart automatiquement en sens
// inverse, meme vitesse et meme duree, puis s arrete. Le robot revient
// donc pres de son point de depart au lieu de s eloigner indefiniment.
float travelSpeedOut  = 0.0f;
bool  travelReturning = false;
bool  travelPausing   = false;
const unsigned long TRAVEL_PAUSE_MS = 800;   // marquage d arret entre aller et retour

// Auto-correction lente du biais gyro quand le robot est desarme et immobile.
// Rattrape une calibration de demarrage ratee sans intervention.
float gyroBiasMean = 0.0f;
float gyroBiasDev  = 0.0f;

// Sauvegarde des reglages en memoire flash (cmd 'F' / 'Q')
Preferences prefs;
bool paramsDirty = false;
unsigned long lastParamChange = 0;

// --------------------------------------------------------------- SERIE
char cmdBuf[40];
uint8_t cmdLen = 0;
unsigned long lastCharTime = 0;

// ------------------------------------------------------------ PROTOTYPES
void mpuWrite(uint8_t reg, uint8_t val);
void calibrateGyro();
void resetLoops();
void applyMotorSpeed(float sps);
void applyMotorSpeeds(float spsD, float spsG);
int32_t pasMoyens();
void saveParams();
void loadParams();

// Precharge l integrale de position pour vaincre le seuil statique au
// demarrage : sans elle le robot met 3 s a se pencher assez pour partir.
void applyTravelPreload(float v) {
  if (startBoost <= 0.01f || Ki_spd < 1e-6f) return;
  // Le coup de pouce sert a vaincre le seuil statique, au demarrage depuis
  // l arret. Si le robot roule deja, deplacer la cible de 800 pas d un coup
  // sous un robot lance provoque un a-coup violent : on s abstient.
  if (fabsf(filtered_speed) > 60.0f) return;
  float pre = startBoost / Ki_spd;
  if (pre > MAX_LEAD) pre = MAX_LEAD;
  int32_t posNow = pasMoyens();
  target_pos = (float)posNow + ((v > 0) ? pre : -pre);
}

// ============================================== SAUVEGARDE DES REGLAGES
// Les resets intempestifs effacaient tout le reglage a chaque fois.
// Les parametres survivent maintenant a un redemarrage.
void saveParams() {
  prefs.begin("robot", false);
  prefs.putFloat("kp",     Kp_a);
  prefs.putFloat("ki",     Ki_a);
  prefs.putFloat("kd",     Kd_a);
  prefs.putFloat("off",    angleOffset);
  prefs.putFloat("kpspd",  Kp_spd);
  prefs.putFloat("kispd",  Ki_spd);
  prefs.putFloat("maxsps", MAX_SPS);
  prefs.putFloat("maxacc", MAX_ACCEL);
  prefs.putFloat("accman", MANUAL_ACCEL);
  prefs.putFloat("travdur", travelDuration);
  prefs.putFloat("tauspd",  TAU_SPEED);
  prefs.putFloat("maxlead", MAX_LEAD);
  prefs.putFloat("boost",   startBoost);
  prefs.putInt  ("sign",   speed_sign);
  prefs.putBool ("trim",   autoTrim);
  prefs.putBool ("rearm",  autoRearm);
  prefs.putBool ("startk", startDual);
  prefs.end();
  Serial.println(F("Reglages sauvegardes"));
}

void loadParams() {
  prefs.begin("robot", true);
  Kp_a         = prefs.getFloat("kp",     Kp_a);
  Ki_a         = prefs.getFloat("ki",     Ki_a);
  Kd_a         = prefs.getFloat("kd",     Kd_a);
  angleOffset  = prefs.getFloat("off",    angleOffset);
  Kp_spd       = prefs.getFloat("kpspd",  Kp_spd);
  Ki_spd       = prefs.getFloat("kispd",  Ki_spd);
  MAX_SPS      = prefs.getFloat("maxsps", MAX_SPS);
  MAX_ACCEL    = prefs.getFloat("maxacc", MAX_ACCEL);
  MANUAL_ACCEL = prefs.getFloat("accman", MANUAL_ACCEL);
  travelDuration = prefs.getFloat("travdur", travelDuration);
  TAU_SPEED      = prefs.getFloat("tauspd",  TAU_SPEED);
  MAX_LEAD       = prefs.getFloat("maxlead", MAX_LEAD);
  startBoost     = prefs.getFloat("boost",   startBoost);
  speed_sign   = prefs.getInt  ("sign",   speed_sign);
  autoTrim     = prefs.getBool ("trim",   autoTrim);
  autoRearm    = prefs.getBool ("rearm",  autoRearm);
  startDual    = prefs.getBool ("startk", startDual);
  prefs.end();
}

void markDirty() {
  paramsDirty = true;
  lastParamChange = millis();
}

// ===================================================================== ISR
void IRAM_ATTR onMpuInterrupt() {
  mpuDataReady = true;
  // Reveil direct de la tache de lecture : pas de scrutation, donc pas de
  // gigue de 1 ms sur dt.
  if (hTacheImu) {
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(hTacheImu, &hp);
    if (hp) portYIELD_FROM_ISR();
  }
}

// ------------------------------------------------------- LA TACHE IMU
// Ne fait QUE la transaction I2C et la publication. Aucune commande, aucun
// filtre : si elle se fige, rien d essentiel ne se fige avec elle.
//
// Lecture de 8 octets a partir de 0x3D : AY(2) AZ(2) TEMP(2) GX(2). AX, GY et
// GZ ne servent pas -> 40 % de temps de bus en moins, donc 40 % de fenetre
// d exposition au bruit en moins. Une seule tentative : la reprise immediate
// enchainait deux transactions dos a dos, ce qui augmente les chances de
// heurter le bug d ISR du pilote. Un echantillon manque est extrapole au gyro.
void tacheImu(void *arg) {
  (void)arg;
  for (;;) {
    ETAPE_IMU(1);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t t0 = micros();
    uint8_t n = 0;
    ETAPE_IMU(2);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3D);
    if (Wire.endTransmission(false) == 0) {
      ETAPE_IMU(3);
      n = Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)8, true);
    }
    uint32_t dt = micros() - t0;
    ETAPE_IMU(4);

    bool ok = (n == 8);
    int16_t ay = 0, az = 0, gx = 0;
    if (ok) {
      ay = (Wire.read() << 8) | Wire.read();
      az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();                   // temperature, ignoree
      gx = (Wire.read() << 8) | Wire.read();
    } else {
      while (Wire.available()) Wire.read();       // vide un reliquat
    }

    portENTER_CRITICAL(&imuMux);
    imuEch.ay = ay; imuEch.az = az; imuEch.gx = gx;
    imuEch.ok = ok; imuEch.t_us = t0; imuEch.d_us = dt;
    imuEch.seq++;
    portEXIT_CRITICAL(&imuMux);
  }
}

// Timer cadence a la DEMI-periode : on bascule STEP a chaque alarme.
// Rapport cyclique 50 % -> impulsion toujours >= 20 us, tres au dessus
// du minimum de 1 us exige par l A4988 (l ancienne boucle d attente
// "for (volatile int i=0;i<16;i++)" faisait ~0.4 us : pas rates possibles).
void IRAM_ATTR onMotorTimer() {
  // Retombee des impulsions emises au top precedent. Toujours en premier :
  // c est ce qui donne leur largeur de 40 us, quelle que soit la suite.
  if (masqueImpulsion) {
    REG_WRITE(GPIO_OUT_W1TC_REG, masqueImpulsion);
    masqueImpulsion = 0;
  }
  if (!motorRunning) return;

  // Aucune mise a jour depuis trop longtemps : la boucle principale est bloquee.
  if (++isrTicks > DEADMAN_TICKS) {
    motorRunning = false;
    REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
    return;
  }

  uint32_t m = 0;
  uint32_t a = accu[0] + incrPas[0];
  if (a >= 65536u) { a -= 65536u; pasFaits[0] += dirSigne[0]; m |= STEP_MASK_D; }
  accu[0] = a;
  a = accu[1] + incrPas[1];
  if (a >= 65536u) { a -= 65536u; pasFaits[1] += dirSigne[1]; m |= STEP_MASK_G; }
  accu[1] = a;

  if (m) {
    REG_WRITE(GPIO_OUT_W1TS_REG, m);
    masqueImpulsion = m;
  }
}

// ============================================================ I2C
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cBusRecover() {
  Wire.end();
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SDA, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
  }
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);   // 8 octets a 100 kHz = 1.2 ms : 20 ms ne se declenche que sur blocage reel
  mpuWrite(0x6B, 0x00);
  i2cRecover++;
  i2cFailStreak = 0;
}

// ========================================================= COMMANDE MOTEUR
void applyMotorSpeeds(float spsD, float spsG) {
  isrTicks = 0;                // la commande est fraiche : on rearme l homme mort

  float aD = fabsf(spsD), aG = fabsf(spsG);

  // En mode manuel on autorise a depasser MAX_SPS : c est justement le test
  // qui sert a determiner MAX_SPS (vitesse de decrochage du moteur).
  float lim = manualMode ? 20000.0f : MAX_SPS;
  if (aD > lim) aD = lim;
  if (aG > lim) aG = lim;

  if (aD < MIN_SPS && aG < MIN_SPS) {
    motorRunning = false;      // ecrivain unique : volatile suffit
    REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
    rtcSps = 0; rtcHalf = ISR_PERIODE_US;
    return;
  }

  bool fwdD = (spsD > 0.0f), fwdG = (spsG > 0.0f);
  digitalWrite(DIR_M1, (fwdD != INVERT_M1) ? HIGH : LOW);
  digitalWrite(DIR_M2, (fwdG != INVERT_M2) ? HIGH : LOW);

  uint32_t iD = (aD < MIN_SPS) ? 0u : (uint32_t)(aD * (65536.0f / F_ISR));
  uint32_t iG = (aG < MIN_SPS) ? 0u : (uint32_t)(aG * (65536.0f / F_ISR));
  if (iD > INCR_MAX) iD = INCR_MAX;
  if (iG > INCR_MAX) iG = INCR_MAX;

  // Section critique reduite au strict minimum : quatre ecritures. Le pilote
  // de timer n est plus appele du tout ici -- la periode ne change jamais.
  portENTER_CRITICAL(&timerMux);
  incrPas[0] = iD;  incrPas[1] = iG;
  dirSigne[0] = fwdD ? 1 : -1;
  dirSigne[1] = fwdG ? 1 : -1;
  portEXIT_CRITICAL(&timerMux);

  if (!motorRunning) {
    motorRunning = true;
    nRallumage++;
  }
  rtcSps  = (int32_t)(0.5f * (spsD + spsG));
  rtcHalf = ISR_PERIODE_US;
}

// Les deux roues a la meme vitesse. Tout le code qui ne sait pas tourner passe
// par ici et se comporte exactement comme avant.
void applyMotorSpeed(float sps) {
  applyMotorSpeeds(sps, sps);
}

// L odometrie que voit la commande : la MOYENNE des deux roues. C est bien la
// distance parcourue par le centre du robot, et c est ce que la politique a
// appris a lire. La difference des deux, elle, est le cap.
int32_t pasMoyens() {
  int32_t a, b;
  portENTER_CRITICAL(&timerMux);
  a = pasFaits[0]; b = pasFaits[1];
  portEXIT_CRITICAL(&timerMux);
  return (a + b) / 2;
}

void resetLoops() {
  satSince = 0;
  manualMode = false;
  manual_target_sps = 0.0f;
  wheel_sps = 0.0f;
  angleIntegral = 0.0f;
  filtered_speed = 0.0f;
  speed_angle_corr = 0.0f;
  outerCounter = 0;
  portENTER_CRITICAL(&timerMux);
  pasFaits[0] = pasFaits[1] = 0;
  accu[0] = accu[1] = 0;
  portEXIT_CRITICAL(&timerMux);
  // Un virage en cours ne doit jamais survivre a une coupure : le robot
  // repartirait en tournant sans que personne l ait demande.
  target_lacet_sps = 0.0f;
  posFigee = false;
  target_pos = 0.0f;
  target_angle = angleOffset;
  // Vide la memoire de la politique : action precedente, avant-precedente,
  // et cible de position. Sans cela l agent repart apres une chute avec
  // l action qui la precedait et une cible perimee -- il corrigerait une
  // erreur qui n existe plus.
  politique_reset();
  applyMotorSpeed(0.0f);
}

// ================================================================ AFFICHAGE
void printMode() {
  Serial.print(F("MODE = "));
  if (!pidEnabled)   Serial.println(F("OFF"));
  else if (!dualPid) Serial.println(F("ANGLE seul (G)"));
  else               Serial.println(F("DOUBLE (K) angle + vitesse"));
}

void printParams() {
  Serial.println(F("---------------- PARAMETRES ----------------"));
  Serial.print(F("P Kp_a     = ")); Serial.println(Kp_a, 1);
  Serial.print(F("I Ki_a     = ")); Serial.println(Ki_a, 2);
  Serial.print(F("D Kd_a     = ")); Serial.println(Kd_a, 1);
  Serial.print(F("O offset   = ")); Serial.println(angleOffset, 3);
  Serial.print(F("T Kp_spd   = ")); Serial.println(Kp_spd, 6);
  Serial.print(F("Y Ki_spd   = ")); Serial.println(Ki_spd, 6);
  Serial.print(F("N cible    = ")); Serial.println(target_speed_sps, 0);
  Serial.print(F("W duree    = ")); Serial.print(travelDuration, 1); Serial.println(F(" s"));
  Serial.print(F("U coup pce = ")); Serial.print(startBoost, 2); Serial.println(F(" deg"));
  Serial.print(F("E avance   = ")); Serial.print(MAX_LEAD, 0);
  Serial.print(F(" pas (")); Serial.print(Ki_spd * MAX_LEAD, 2); Serial.println(F(" deg)"));
  Serial.print(F("L tau vit  = ")); Serial.print(TAU_SPEED, 3);
  Serial.print(F(" s (")); Serial.print(1.0f / (6.2832f * TAU_SPEED), 2); Serial.println(F(" Hz)"));
  Serial.print(F("M maxSps   = ")); Serial.println(MAX_SPS, 0);
  Serial.print(F("R maxAcc   = ")); Serial.println(MAX_ACCEL, 0);
  Serial.print(F("B accelMan = ")); Serial.println(MANUAL_ACCEL, 0);
  Serial.print(F("X signe    = ")); Serial.println(speed_sign);
  Serial.print(F("A autotrim = ")); Serial.println(autoTrim ? F("ON") : F("OFF"));
  Serial.print(F("H relevage = ")); Serial.println(autoRearm ? F("ON") : F("OFF"));
  Serial.print(F("J DUAL dem = ")); Serial.println(startDual ? F("ON") : F("OFF"));
  printMode();
  Serial.println(F("G angle | K double | S stop | Z calage | C recalib gyro | ? params"));
  Serial.println(F("F sauver | Q effacer memoire | H auto-relevage | J DUAL au demarrage"));
  Serial.println(F("L tau vitesse | W duree | N vitesse | E avance max | U coup de pouce"));
  Serial.println(F("1 cascade | 2 agent | 3 signe | 4 secousse"));
  Serial.println(F("6 enregistrer 6 s | 7 balayage | 8 durees | 9 balayage angle"));
  Serial.println(F("N vitesse (pas/s) | # lacet (difference pas/s) | 0 avance"));
  Serial.println(F("--------------------------------------------"));
}

// ================================================================ COMMANDES
void execCmd(const char *s) {
  char c = s[0];
  bool hasVal = (strlen(s) > 1);
  float v = hasVal ? atof(s + 1) : 0.0f;

  switch (c) {
    case 'P': case 'p': if (hasVal) { Kp_a = v; markDirty(); }   Serial.print(F("Kp_a="));   Serial.println(Kp_a, 1); break;
    case 'I': case 'i': if (hasVal) { Ki_a = v; markDirty(); }   Serial.print(F("Ki_a="));   Serial.println(Ki_a, 2); break;
    case 'D': case 'd': if (hasVal) { Kd_a = v; markDirty(); }   Serial.print(F("Kd_a="));   Serial.println(Kd_a, 1); break;
    case 'O': case 'o': if (hasVal) { angleOffset = v; angleOffsetInit = v; markDirty(); }
                        Serial.print(F("offset=")); Serial.println(angleOffset, 3); break;
    case 'T': case 't': if (hasVal) { Kp_spd = v; markDirty(); } Serial.print(F("Kp_spd=")); Serial.println(Kp_spd, 6); break;
    case 'Y': case 'y': if (hasVal) { Ki_spd = v; markDirty(); } Serial.print(F("Ki_spd=")); Serial.println(Ki_spd, 6); break;
    case '#':
      target_lacet_sps = constrain(v, -LACET_MAX, LACET_MAX);
      Serial.print(F("lacet=")); Serial.print(target_lacet_sps, 0);
      Serial.println(F(" pas/s de difference"));
      break;

    case 'N': case 'n':
      target_speed_sps = v;
      travelUntil = 0;
      if (fabsf(v) > 0.5f) {
        if (travelDuration > 0.01f) {
          travelUntil = millis() + (unsigned long)(travelDuration * 1000.0f);
        }
        travelSpeedOut = v;         // memorise pour le retour
        travelReturning = false;
        applyTravelPreload(v);
      } else {
        travelSpeedOut = 0.0f;
        travelReturning = false;
      }
      Serial.print(F("cible=")); Serial.print(target_speed_sps, 0);
      if (travelUntil) { Serial.print(F(" pendant ")); Serial.print(travelDuration, 1); Serial.print(F(" s puis retour")); }
      Serial.println();
      break;

    case 'L': case 'l':
      if (hasVal && v > 0.02f) { TAU_SPEED = v; markDirty(); }
      Serial.print(F("tau vitesse = ")); Serial.print(TAU_SPEED, 3);
      Serial.print(F(" s  -> coupure ")); Serial.print(1.0f / (6.2832f * TAU_SPEED), 2);
      Serial.println(F(" Hz")); break;

    case 'U': case 'u':
      if (hasVal) { startBoost = v; markDirty(); }
      Serial.print(F("coup de pouce = ")); Serial.print(startBoost, 2);
      Serial.print(F(" deg -> precharge ")); Serial.print(startBoost / Ki_spd, 0);
      Serial.println(F(" pas")); break;

    case 'E': case 'e':
      if (hasVal && v >= 100.0f) { MAX_LEAD = v; markDirty(); }
      Serial.print(F("avance max cible = ")); Serial.print(MAX_LEAD, 0);
      Serial.print(F(" pas -> autorite integrale ")); Serial.print(Ki_spd * MAX_LEAD, 2);
      Serial.println(F(" deg")); break;

    case 'W': case 'w':
      if (hasVal) { travelDuration = v; markDirty(); }
      Serial.print(F("duree deplacement = ")); Serial.print(travelDuration, 1);
      Serial.println(F(" s (0 = illimite)")); break;
    case 'M': case 'm': if (hasVal) { MAX_SPS = v; markDirty(); }   Serial.print(F("maxSps=")); Serial.println(MAX_SPS, 0); break;
    case 'R': case 'r': if (hasVal) { MAX_ACCEL = v; markDirty(); } Serial.print(F("maxAcc=")); Serial.println(MAX_ACCEL, 0); break;
    case 'B': case 'b': if (hasVal) { MANUAL_ACCEL = v; markDirty(); } Serial.print(F("accelManuel=")); Serial.println(MANUAL_ACCEL, 0); break;

    case 'F': case 'f': saveParams(); paramsDirty = false; break;

    case 'Q': case 'q':
      prefs.begin("robot", false); prefs.clear(); prefs.end();
      Serial.println(F("Memoire effacee - valeurs d usine au prochain demarrage"));
      break;

    case 'H': case 'h':
      autoRearm = !autoRearm; markDirty();
      Serial.print(F("auto-relevage=")); Serial.println(autoRearm ? F("ON") : F("OFF")); break;

    case 'J': case 'j':
      startDual = !startDual; markDirty();
      Serial.print(F("DUAL au demarrage=")); Serial.println(startDual ? F("ON") : F("OFF")); break;

    case 'G': case 'g':
      pidEnabled = !pidEnabled;
      dualPid = false;
      wantDual = false;
      pendingArm = false;
      if (pidEnabled && fabsf(pitch - angleOffset) > ARM_LIMIT) {
        pidEnabled = false;
        Serial.println(F("!! Redresse le robot avant d armer"));
      }
      resetLoops(); printMode(); break;

    case 'K': case 'k':
      dualPid = !dualPid;
      wantDual = dualPid;
      pendingArm = false;
      if (dualPid && !pidEnabled) {
        if (fabsf(pitch - angleOffset) > ARM_LIMIT) {
          dualPid = false;
          Serial.println(F("!! Redresse le robot avant d armer"));
        } else pidEnabled = true;
      }
      resetLoops(); printMode(); break;

    case 'X': case 'x':
      speed_sign = -speed_sign; markDirty();
      resetLoops();
      Serial.print(F("speed_sign=")); Serial.println(speed_sign); break;

    case 'A': case 'a':
      autoTrim = !autoTrim; markDirty();
      Serial.print(F("autotrim=")); Serial.println(autoTrim ? F("ON") : F("OFF")); break;

    case 'Z': case 'z':
      angleOffset = pitch;
      angleOffsetInit = pitch;
      markDirty();
      resetLoops();
      Serial.print(F("Calage offset=")); Serial.println(angleOffset, 3); break;

    case 'C': case 'c':
      pidEnabled = false; dualPid = false; resetLoops();
      calibrateGyro();
      break;

    case 'V': case 'v': {
      pidEnabled = false; dualPid = false;
      float keep = wheel_sps;              // on repart de la vitesse actuelle
      resetLoops();
      wheel_sps = keep;
      manual_target_sps = (v * STEPS_PER_REV) / 60.0f;
      manualMode = true;
      Serial.print(F("Manuel RPM=")); Serial.print(v);
      Serial.print(F("  ->  ")); Serial.print(manual_target_sps, 0); Serial.println(F(" pas/s"));
      break;
    }

    // ---- CHOIX DU PILOTE. Toutes les lettres sont deja prises, d ou des
    // chiffres. '1' est le retour en securite : une frappe et on est revenu
    // sur la cascade, sans avoir a rearmer.
    case '1':
      useAgent = false; testSigne = false; testSecousse = false;
      pidEnabled = false; dualPid = false; pendingArm = false;
      resetLoops();
      Serial.println(F("pilote = CASCADE  (K pour armer)"));
      break;

    case '2':
      useAgent = true; testSigne = false; testSecousse = false;
      pidEnabled = false; dualPid = false; pendingArm = false;
      resetLoops();
      Serial.println(F("pilote = AGENT  (K pour armer)"));
      break;

    // ---- TEST DE SIGNE, roues en l air. Rien n est envoye aux moteurs :
    // on affiche seulement ce que l agent VOUDRAIT faire. C est le seul
    // point du portage qui peut casser le materiel s il est faux.
    case '5':
      pidEnabled = false; dualPid = false; pendingArm = false;
      useAgent = false; testSecousse = false; testSigne = false;
      resetLoops();
      manualMode = true;
      manual_target_sps = 0.0f;
      rampeT0 = micros();
      rampeActive = true;
      enrDemarrer("decrochage");
      Serial.println(F("=== DECROCHAGE : ROUES EN L AIR, ROBOT TENU ==="));
      Serial.print(F("Rampe de ")); Serial.print(RAMPE_ACCEL, 0);
      Serial.println(F(" pas/s2 pendant 6 s, soit jusqu a 4800 pas/s."));
      Serial.println(F("Dis-moi a quel moment tu l entends decrocher."));
      break;

    case '0':
      if (!pidEnabled) { Serial.println(F("Il faut d abord tenir debout : 1 puis K.")); break; }
      enrDemarrer("avance");
      avanceDebut = millis() + 1000;
      avanceFin   = avanceDebut + AVANCE_MS;
      Serial.print(F("=== AVANCE : "));
      Serial.print(AVANCE_SPS, 0);
      Serial.print(F(" pas/s pendant "));
      Serial.print(AVANCE_MS);
      Serial.println(F(" ms, apres 1 s de repos ==="));
      Serial.println(F("Environ 10 cm. Ne le touche pas, il revient tout seul."));
      break;

    case '9':
      enrDemarrer("bal_angle");
      lacherIdx = 0; pousseeActive = false; lacherActif = false;
      lacherProchain = 0; balayageActif = false;
      balAngleT0 = micros();
      balAngleActif = true;
      Serial.println(F("=== BALAYAGE SUR LA CONSIGNE D ANGLE, 0,2 a 3 Hz en 5,5 s ==="));
      Serial.println(F("Amplitude 1 deg. Il se dandine sur place, environ 2 cm."));
      Serial.println(F("Ne le touche pas."));
      break;

    case '6':
      enrDemarrer("libre");
      Serial.println(F("=== ENREGISTREMENT 6 s a 200 Hz, capture de ce qui se passe ==="));
      break;

    case '7':
      if (!pidEnabled) {
        Serial.println(F("Il faut d abord tenir debout : 1 puis K, puis 7."));
        break;
      }
      enrDemarrer("balayage");
      lacherIdx = 0; pousseeActive = false; lacherActif = false;
      lacherProchain = 0;
      balayageT0 = micros();
      balayageActif = true;
      Serial.println(F("=== BALAYAGE EN FREQUENCE, 0,5 a 20 Hz en 5 s ==="));
      Serial.println(F("La cascade garde les commandes : il ne se deplace pas et ne tombe pas."));
      Serial.println(F("Ne le touche pas, laisse-le vibrer sur place."));
      break;

    case '8':
      Serial.println(F("durees fixees a 150, 250 et 400 ms, enchainees par '7'"));
      break;

    case '4':
      testSecousse = !testSecousse;
      useAgent = false; testSigne = false;
      pidEnabled = false; dualPid = false; pendingArm = false;
      resetLoops();
      secousseT0 = millis(); secousseSigne = 1;
      if (testSecousse) {
        Serial.println(F("=== SECOUSSE : ROUES EN L AIR, ROBOT TENU ==="));
        Serial.print(F("Creneau +/-")); Serial.print(SECOUSSE_AMP, 0);
        Serial.print(F(" pas/s toutes les ")); Serial.print(SECOUSSE_MS);
        Serial.println(F(" ms, soit 5 Hz."));
        Serial.println(F("Aucune boucle, aucune lecture d angle : que le chemin moteur."));
        Serial.println(F("'4' de nouveau pour arreter."));
      } else {
        Serial.println(F("secousse arretee"));
        wheel_sps = 0.0f;
        applyMotorSpeed(0.0f);
      }
      break;

    case '3':
      testSigne = !testSigne;
      pidEnabled = false; dualPid = false; pendingArm = false;
      resetLoops();
      if (testSigne) {
        Serial.println(F("=== TEST DE SIGNE : ROUES EN L AIR ==="));
        Serial.println(F("Penche le robot a la main et lis 'a'."));
        Serial.println(F("  vers l AVANT   -> a doit etre POSITIF"));
        Serial.println(F("  vers l ARRIERE -> a doit etre NEGATIF"));
        Serial.println(F("Reference (etat neuf) :  +1 -> +4775   +3 -> +10404"));
        Serial.println(F("                         +5 -> +12712  +10 -> +13556"));
        Serial.println(F("                         a 0 deg le reseau donne +444"));
      } else {
        Serial.println(F("test de signe termine"));
      }
      break;

    case 'S': case 's':
      pidEnabled = false; dualPid = false; wantDual = false;
      useAgent = false; testSigne = false;   // STOP ramene toujours au connu
      testSecousse = false;
      pendingArm = false;          // un STOP doit vraiment arreter le robot
      travelUntil = 0; travelReturning = false; travelPausing = false; travelSpeedOut = 0.0f;
      resetLoops();
      if (paramsDirty) { saveParams(); paramsDirty = false; }
      Serial.println(F("STOP")); break;

    case '?': printParams(); break;
    default: break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    lastCharTime = millis();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; execCmd(cmdBuf); cmdLen = 0; }
    } else if (ch != ' ' && cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
  // Filet si le moniteur serie n envoie pas de fin de ligne.
  // (remplace Serial.parseFloat() qui bloquait 1000 ms)
  if (cmdLen && (millis() - lastCharTime) > 120) {
    cmdBuf[cmdLen] = 0; execCmd(cmdBuf); cmdLen = 0;
  }
}

// ================================================================ MPU SETUP
void calibrateGyro() {
  Serial.println(F("Calibration gyro : robot IMMOBILE, moteurs OFF..."));
  float best = 0.0f, bestDev = 1e9f;

  // Jusqu a 4 essais. Le critere qui compte est le BRUIT (ecart-type) :
  // un robot qui bouge donne un ecart-type eleve, et le biais mesure est
  // alors faux. Une calibration ratee rend l estimation d angle inutilisable
  // et empeche l armement automatique de se declencher.
  for (int tries = 1; tries <= 4; tries++) {
    delay(300);
    double sum = 0, sum2 = 0;
    int ok = 0;
    for (int i = 0; i < 300; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      if (Wire.endTransmission(false) == 0 &&
          Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2, true) == 2) {
        int16_t gx = (Wire.read() << 8) | Wire.read();
        double g = (double)gx / 131.0;
        sum += g; sum2 += g * g; ok++;
      }
      // Sans ca, une calibration qui echoue et recommence bloque loopTask plus
      // de 5 s et le chien de garde de tache fait redemarrer la carte. C est
      // la cause des trois plantages a 5 719 ms d uptime du 9 septembre.
      esp_task_wdt_reset();
      delay(3);
    }
    if (ok < 100) { Serial.println(F("  !! lecture I2C impossible")); continue; }

    double m = sum / ok;
    double var = sum2 / ok - m * m;
    double dev = (var > 0) ? sqrt(var) : 0;

    Serial.print(F("  essai ")); Serial.print(tries);
    Serial.print(F(" : biais ")); Serial.print((float)m, 3);
    Serial.print(F(" deg/s, bruit ")); Serial.println((float)dev, 3);

    if (dev < bestDev) { bestDev = (float)dev; best = (float)m; }
    if (dev < 1.0) break;
    Serial.println(F("  -> robot pas assez immobile, on recommence"));
  }

  gyroX_offset = best;
  Serial.print(F("gyroX_offset = ")); Serial.print(gyroX_offset, 4);
  if (bestDev > 1.0f) {
    Serial.println(F("   !! CALIBRATION DOUTEUSE - relance avec C, robot pose"));
  } else {
    Serial.println(F("   (ok)"));
  }
}

void mpuSetup() {
  mpuWrite(0x6B, 0x00);        // reveil
  delay(50);
  mpuWrite(0x1B, 0x00);        // gyro +/- 250 deg/s
  mpuWrite(0x1C, 0x00);        // accel +/- 2 g
  mpuWrite(0x1A, 0x03);        // DLPF 44 Hz / 4.9 ms (etait 21 Hz / 8.5 ms)
  mpuWrite(0x19, 0x04);        // 1000/(1+4) = 200 Hz
  mpuWrite(0x37, 0x10);        // INT efface a la lecture
  mpuWrite(0x38, 0x01);        // INT data ready
}

// ===================================================================== SETUP
void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(1024);
  delay(800);

  // Le driver i2c.master de l IDF 5.x imprime 4 lignes d erreur bloquantes
  // a chaque NACK (~11 ms mesures sur Lus:). On le fait taire : on a deja
  // notre propre compteur "f:".
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  esp_log_level_set("i2c.common", ESP_LOG_NONE);

  // CHIEN DE GARDE MATERIEL. Si loop() se bloque plus de 2 s -- typiquement
  // dans le pilote I2C -- la puce redemarre d elle-meme au lieu de rester
  // figee. Combine a 'J' (DUAL au demarrage) et 'H' (auto-relevage), le robot
  // se remet debout tout seul. Et la cause affichee au redemarrage suivant
  // ('watchdog de tache') confirmera qu il s agissait bien d un blocage.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  esp_task_wdt_config_t twdt = { .timeout_ms = 2000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_init(&twdt) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&twdt);
#else
  esp_task_wdt_init(2, true);
#endif
  esp_task_wdt_add(NULL);

  loadParams();   // reglages sauvegardes -> ils survivent a un reset

  pinMode(DIR_M1, OUTPUT);  pinMode(STEP_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);  pinMode(STEP_M2, OUTPUT);
  REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);

  motorTimer = TIMER_BEGIN_1MHZ();               // 1 tick = 1 us
  TIMER_ATTACH(motorTimer, &onMotorTimer);
  // Posee UNE fois pour toutes. Les vitesses passent desormais par les
  // increments des accumulateurs, plus par la periode du timer.
  TIMER_SET_PERIOD(motorTimer, ISR_PERIODE_US);
  TIMER_ENABLE(motorTimer);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Wire.setTimeOut(20);   // 8 octets a 100 kHz = 1.2 ms : 20 ms ne se declenche que sur blocage reel
  mpuSetup();

  calibrateGyro();

  // Initialise le pitch depuis l accelerometre, pas depuis angleOffset
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) == 0 &&
      Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true) == 6) {
    Wire.read(); Wire.read();                     // AX inutilise
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    pitch = atan2f((float)ay, (float)az) * 180.0f / PI;
  } else {
    pitch = angleOffset;
  }

  angleOffsetInit = angleOffset;
  target_angle = angleOffset;

  pinMode(MPU_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMpuInterrupt, RISING);

  lastMeasureTime = micros();
  // Boite noire : on relit AVANT de reinitialiser le marqueur.
  {
    esp_reset_reason_t rr = esp_reset_reason();
    bool plantage = (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT
                                        || rr == ESP_RST_TASK_WDT);
    if (plantage && rtcMagie == RTC_MAGIE) {
      Serial.print(F(">>> BOITE NOIRE : bloque a l etape "));
      Serial.print(rtcEtape);
      Serial.print(F(" ("));
      Serial.print(nomEtape(rtcEtape));
      Serial.print(F(")  sps "));
      Serial.print(rtcSps);
      Serial.print(F("  demi-periode "));
      Serial.print(rtcHalf);
      Serial.println(F(" us"));
      Serial.print(F(">>> BOITE NOIRE : tache IMU a l etape "));
      Serial.print(rtcEtapeImu);
      Serial.print(F(" ("));
      Serial.print(nomEtapeImu(rtcEtapeImu));
      Serial.println(F(")"));
    } else if (plantage) {
      Serial.println(F(">>> BOITE NOIRE : pas de marqueur (firmware d avant l instrumentation)"));
    }
    rtcMagie = RTC_MAGIE;
    rtcEtape = 0;
    rtcEtapeImu = 0;
  }
  Serial.print(F("Cause du dernier reset : "));
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  Serial.println(F("mise sous tension / EN")); break;
    case ESP_RST_SW:       Serial.println(F("reset logiciel")); break;
    case ESP_RST_PANIC:    Serial.println(F("!! PLANTAGE LOGICIEL (panic)")); break;
    case ESP_RST_INT_WDT:  Serial.println(F("!! watchdog d interruption")); break;
    case ESP_RST_TASK_WDT: Serial.println(F("!! watchdog de tache")); break;
    case ESP_RST_BROWNOUT: Serial.println(F("!! chute de tension (brownout)")); break;
    case ESP_RST_EXT:      Serial.println(F("reset externe")); break;
    default:               Serial.println(F("autre")); break;
  }
  // La tache de lecture demarre APRES la calibration, qui utilise Wire en
  // direct : deux maitres sur le meme bus se marcheraient dessus.
  // Coeur 0 : le coeur 1 porte deja loopTask et le timer de pas.
  // Priorite 5, au-dessus de loop() (1) : un echantillon en retard fausse dt.
  // Pas d abonnement au chien de garde de tache -- c est deliberé, voir la
  // declaration de EchantillonImu.
  xTaskCreatePinnedToCore(tacheImu, "imu", 3072, NULL, 5, &hTacheImu, 0);

  Serial.println(F("=== Robot balancier pret ==="));
  printParams();

  if (startDual) {
    wantDual = true;
    pendingArm = true;
    Serial.println(F(">>> DUAL au demarrage : redresse le robot, il partira seul"));
  }
}

// ============================================================ BOUCLE INTERNE
void innerLoop(float dt) {
  // Essai de lacher : on cesse d envoyer des pas. Le driver garde le courant,
  // donc les roues sont BLOQUEES, pas libres. Le robot bascule alors comme un
  // pendule inverse rigide autour du contact au sol : c est exactement le
  // regime dont la constante de temps donne l inertie sur moment de gravite.
  if (lacherActif) {
    wheel_sps = 0.0f;
    angleIntegral = 0.0f;
    applyMotorSpeed(0.0f);
    return;
  }
  // Poussee d identification : acceleration imposee, asservissement court-circuite.
  if (pousseeActive) {
    angleIntegral = 0.0f;
    wheel_sps += pousseeSigne * POUSSEE_ACCEL * dt;
    if (wheel_sps >  MAX_SPS) wheel_sps =  MAX_SPS;
    if (wheel_sps < -MAX_SPS) wheel_sps = -MAX_SPS;
    applyMotorSpeed(wheel_sps);
    return;
  }
  // Balayage sur la consigne d angle : on ajoute une inclinaison demandee.
  if (balAngleActif) {
    float u = (micros() - balAngleT0) * 1e-6f;
    if (u >= BALANGLE_DUREE) { balAngleActif = false; injCourant = 0.0f; }
    else {
      float k = logf(BALANGLE_F1 / BALANGLE_F0);
      float phase = 2.0f * PI * BALANGLE_F0 * BALANGLE_DUREE / k
                    * (expf(k * u / BALANGLE_DUREE) - 1.0f);
      injCourant = BALANGLE_AMP * sinf(phase);
    }
  }
  float err = pitch - target_angle - (balAngleActif ? injCourant : 0.0f);
  float accel;

  if (useAgent) {
    // ---- L AGENT. Il decide a 100 Hz ; l integrateur reste a 200 Hz, donc
    // un tick sur deux, en gardant sa sortie entre-temps. C est la cadence
    // sur laquelle il a ete entraine.
    static bool  tickPol  = false;
    static float accelPol = 0.0f;
    tickPol = !tickPol;
    if (tickPol) {
      int32_t pos = pasMoyens();

      // Chronometrage : 1 568 multiplications-accumulations et 64 tanhf.
      // C etait la seule inconnue du portage -- tanhf vient de newlib et
      // n a pas d instruction materielle. Affiche en Pus dans la telemetrie.
      uint32_t tPol = micros();
      accelPol = politique_accel(pitch - angleOffset, gyroFilt,
                                 wheel_sps, (long)pos, target_speed_sps,
                                 millis());
      tPol = micros() - tPol;
      if (tPol > maxPolUs) maxPolUs = tPol;

      // Garde-fous : angle hors du domaine ou il sait se rattraper (14 deg,
      // mesure 0/6 a 12 deg), commande collee a la butee plus de 120 ms
      // (elle ne sature JAMAIS en marche normale), ou valeur non numerique.
      // Dans les trois cas politique_accel a deja renvoye 0.
      if (politique_defaut != POL_OK) {
        pidEnabled = false; dualPid = false;
        pendingArm = autoRearm;
        Serial.print(F(">>> AGENT COUPE, defaut "));
        Serial.print(politique_defaut);
        Serial.print(F("  angle ")); Serial.print(pitch - angleOffset, 2);
        Serial.print(F(" deg  sps ")); Serial.println(wheel_sps, 0);
        resetLoops();          // coupe les moteurs dans ce meme tour de boucle
        return;
      }
    }
    accel = accelPol;
    angleIntegral = 0.0f;      // l integrale de la cascade ne sert plus
  } else {
    angleIntegral += err * dt;
    angleIntegral = constrain(angleIntegral, -20.0f, 20.0f);
    accel = Kp_a * err + Ki_a * angleIntegral + Kd_a * gyroFilt;
  }

  // ---- injection du balayage, PAR-DESSUS la commande de la cascade.
  // La cascade reste aux commandes : elle tient le robot debout et le ramene a
  // sa place, donc il ne se deplace pas et il ne tombe pas.
  if (balayageActif) {
    float u = (micros() - balayageT0) * 1e-6f;
    if (u >= BALAYAGE_DUREE) { balayageActif = false; injCourant = 0.0f; }
    else {
      // Balayage logarithmique : autant de periodes par octave, donc autant
      // d information a 1 Hz qu a 16 Hz.
      float k = logf(BALAYAGE_F1 / BALAYAGE_F0);
      float phase = 2.0f * PI * BALAYAGE_F0 * BALAYAGE_DUREE / k
                    * (expf(k * u / BALAYAGE_DUREE) - 1.0f);
      injCourant = BALAYAGE_AMP * sinf(phase);
      accel += injCourant;
    }
  } else if (!balAngleActif) {
    // Ne remettre a zero QUE si aucun des deux balayages n est en cours.
    // Sans ce test, le balayage d acceleration effacait la valeur que le
    // balayage de consigne venait d ecrire, et la colonne inj du fichier
    // sortait a zero : l excitation etait bien appliquee, mais sa reference
    // etait perdue et il a fallu la reconstruire a posteriori.
    injCourant = 0.0f;
  }

  // Limite d acceleration derateee selon la vitesse (courbe couple-vitesse)
  float accelLimit = MAX_ACCEL * (1.0f - ACCEL_DERATE * fabsf(wheel_sps) / MAX_SPS);
  float accelFloor = MAX_ACCEL * (1.0f - ACCEL_DERATE);
  if (accelLimit < accelFloor) accelLimit = accelFloor;
  accel = constrain(accel, -accelLimit, accelLimit);

  wheel_sps += accel * dt;

  // Anti-emballement : sur saturation de vitesse on gele l integrale d angle
  if (wheel_sps > MAX_SPS)  { wheel_sps = MAX_SPS;  if (err > 0) angleIntegral -= err * dt; }
  if (wheel_sps < -MAX_SPS) { wheel_sps = -MAX_SPS; if (err < 0) angleIntegral -= err * dt; }

  // Surveillance de la saturation de vitesse, pour la securite dans loop()
  if (fabsf(wheel_sps) >= MAX_SPS * 0.98f) {
    if (satSince == 0) satSince = millis();
  } else {
    satSince = 0;
  }

  // La consigne de lacet se superpose a la vitesse d avance. On la retire de la
  // roue qui doit ralentir plutot que de l ajouter a l autre : sinon une
  // commande de virage a pleine vitesse ferait saturer une roue et le robot
  // avancerait au lieu de tourner.
  float d = target_lacet_sps * 0.5f;
  float sD = wheel_sps - d;      // droite ralentit
  float sG = wheel_sps + d;      // gauche accelere  -> le robot part a droite
  float debord = fmaxf(fabsf(sD), fabsf(sG)) - MAX_SPS;
  if (debord > 0.0f) {                    // on rogne l avance, pas le virage
    float k = (wheel_sps > 0.0f) ? -debord : debord;
    sD += k; sG += k;
  }
  applyMotorSpeeds(sD, sG);
}

// ============================================================ BOUCLE EXTERNE
void outerLoop(float dtOuter) {
  int32_t pos;
  portENTER_CRITICAL(&timerMux);
  pos = pasMoyens();
  portEXIT_CRITICAL(&timerMux);

  float k = dtOuter / (TAU_SPEED + dtOuter);
  filtered_speed += (wheel_sps - filtered_speed) * k;

  // ------------------------------------------------ VITESSE ou POSITION
  // Deux regimes, et il ne faut surtout pas les melanger.
  //
  // Consigne non nulle -> on pilote en VITESSE. La cible de position SUIT la
  // position reelle, donc aucune dette ne s accumule. C est le point corrige
  // le 10 septembre : avant, la cible s integrait meme quand le robot ne
  // pouvait pas suivre, et il restait penche a rembourser une dette qu il ne
  // rembourserait jamais -- puis il la rendait d un coup au relachement.
  // Matthieu l a decrit exactement : "s il n a pas fait sa consigne il tente
  // a l infini". Une consigne de vitesse est un ordre pour MAINTENANT, pas
  // une promesse de distance a parcourir.
  //
  // Consigne nulle -> on tient la POSITION ou on vient de relacher. La cible
  // est figee et le robot revient dessus s il derive. C est ce qui lui evite
  // de glisser indefiniment.
  if (fabsf(target_speed_sps) > 0.5f) {
    target_pos = (float)pos;
    posFigee = false;
  } else if (!posFigee) {
    target_pos = (float)pos;     // on latche a l instant du relachement
    posFigee = true;
  }

  float pos_err   = constrain((float)pos - target_pos, -MAX_POS_ERR, MAX_POS_ERR);
  float speed_err = filtered_speed - target_speed_sps;

  // Signe : monter l angle cible => le robot accelere en positif
  // (il faut a = g*tan(theta) pour tenir une inclinaison).
  // Donc pour freiner une vitesse trop positive il faut BAISSER la cible.
  float corr = (float)speed_sign * (Kp_spd * speed_err + Ki_spd * pos_err);
  corr = constrain(corr, -MAX_TILT_CORR, MAX_TILT_CORR);

  // Pendant une grosse perturbation la boucle externe doit s effacer : pour
  // se relever, le robot doit viser son VRAI point d equilibre, pas un angle
  // decale par une consigne de vitesse. Fondu progressif entre 3 et 6 deg
  // (surtout pas une commutation brutale, qui creerait un cycle limite).
  float distur = fabsf(pitch - angleOffset);
  float w = (OUTER_FADE_HI - distur) / (OUTER_FADE_HI - OUTER_FADE_LO);
  corr *= constrain(w, 0.0f, 1.0f);

  if (fabsf(corr) >= MAX_TILT_CORR) target_pos += (pos - target_pos) * 0.02f;

  speed_angle_corr = corr;
  target_angle = angleOffset - corr;

  // Auto-trim : deplace lentement l equilibre reel pour annuler la correction.
  // UNIQUEMENT quand le robot est calme. Sinon une bousculade fait deriver
  // angleOffset de plusieurs degres pendant que le robot se debat, et il
  // perd sa reference de verticale : il ne sait plus se relever.
  // Condition supplementaire decisive : |pos_err| petit.
  // Sans elle, l auto-trim efface la correction de position. Quand le robot
  // s eloigne, la boucle externe produit une correction pour le ramener, et
  // l auto-trim l interprete comme un mauvais reglage d equilibre : il
  // deplace angleOffset pour l annuler, donc le robot ne rentre jamais.
  // L auto-trim ne doit absorber que la derive de vitesse, jamais l ecart
  // de position.
  bool calme = fabsf(target_speed_sps) < 1.0f
            && fabsf(pitch - target_angle) < 1.5f
            && fabsf(corr) < MAX_TILT_CORR * 0.7f
            && fabsf(filtered_speed) < MAX_SPS * 0.3f
            && fabsf(pos_err) < 5000.0f;   // ~90 cm : ne bloque que la fuite
  if (autoTrim && calme) {
    angleOffset -= AUTOTRIM_RATE * corr * dtOuter;
    angleOffset = constrain(angleOffset, angleOffsetInit - 5.0f, angleOffsetInit + 5.0f);
  }
}

// ====================================================================== LOOP
void loop() {
  uint32_t tLoop = micros();

  esp_task_wdt_reset();   // on signale au chien de garde qu on est vivant

  ETAPE(1);
  pollSerial();

  // Chien de garde IMU, deux etages.
  // 40 ms : on coupe la commande, l estimation d angle n est plus fraiche.
  ETAPE(2);
  if ((pidEnabled || testSecousse) && (micros() - lastMeasureTime) > 40000UL) {
    wheel_sps = 0;
    applyMotorSpeed(0);
  }
  // 150 ms : ce n est plus un hoquet, la tache I2C est bloquee dans le pilote.
  // On desarme TOUT et on le dit. Avant la separation des taches, ce cas ne
  // pouvait pas etre traite : la boucle etait bloquee elle aussi.
  if ((pidEnabled || testSecousse || useAgent)
      && (micros() - lastMeasureTime) > 150000UL) {
    pidEnabled = false; dualPid = false;
    useAgent = false; testSecousse = false; pendingArm = false;
    i2cBloc++;
    resetLoops();
    Serial.print(F(">>> SECURITE : bus I2C fige, tout desarme  (blocage n "));
    Serial.print(i2cBloc);
    Serial.println(F(")"));
    // On NE touche PAS a Wire depuis ici. La tache est peut-etre bloquee dans
    // le pilote en tenant son verrou : appeler Wire.end() depuis cette boucle
    // la bloquerait a son tour, et on aurait tout perdu. On observe.
    silenceDepuis = micros();
  }

  // Dernier recours. Si le capteur reste muet 3 s, la tache ne repartira plus
  // toute seule : on redemarre volontairement. Un redemarrage choisi vaut mieux
  // qu un chien de garde -- le message part, la boite noire reste lisible, et
  // la carte revient en 2,5 s au lieu de 7.
  ETAPE(16);
  if (silenceDepuis && (micros() - lastMeasureTime) > 3000000UL) {
    Serial.println(F(">>> BUS I2C MUET 3 s : redemarrage volontaire"));
    Serial.flush();
    delay(20);
    ESP.restart();
  }
  if (silenceDepuis && (micros() - lastMeasureTime) < 100000UL) {
    Serial.print(F(">>> bus I2C reparti tout seul apres "));
    Serial.print((micros() - silenceDepuis) / 1000);
    Serial.println(F(" ms"));
    silenceDepuis = 0;
  }

  // ---------------------------------------- CONSOMMATION DE L ECHANTILLON
  // Plus aucune transaction I2C ici : on lit ce que la tache a publie. Si elle
  // est figee, seq n avance plus, ce bloc ne s execute pas, et le garde-fou de
  // silence plus bas desarme proprement au lieu de laisser le chien de garde
  // redemarrer la carte.
  ETAPE(3);
  uint32_t seq; int16_t rawAY, rawAZ, rawGX; bool dataOk; uint32_t tEch, dI2c;
  portENTER_CRITICAL(&imuMux);
  seq    = imuEch.seq;
  rawAY  = imuEch.ay;  rawAZ = imuEch.az;  rawGX = imuEch.gx;
  dataOk = imuEch.ok;
  tEch   = imuEch.t_us;
  dI2c   = imuEch.d_us;
  portEXIT_CRITICAL(&imuMux);

  if (seq != seqTraitee) {
    seqTraitee = seq;

    unsigned long now = tEch;
    float dt = (now - lastMeasureTime) * 1e-6f;
    lastMeasureTime = now;
    if (dt < 0.001f || dt > 0.05f) dt = 0.005f;
    loopHz += (1.0f / dt - loopHz) * 0.02f;

    if (dI2c > maxI2cUs) maxI2cUs = dI2c;

    if (dataOk) {
      i2cReadOK++;
      i2cFailStreak = 0;

      float ay = rawAY, az = rawAZ;
      float magG = sqrtf(ay * ay + az * az) / 16384.0f;
      bool accOk = (magG > 0.6f && magG < 1.4f);

      gyroRate = ((float)rawGX / 131.0f) - gyroX_offset;

      float gyroPitch = pitch + gyroRate * dt;
      if (accOk) {
        float accPitch = atan2f(ay, az) * 180.0f / PI;
        while (accPitch - gyroPitch >  180.0f) accPitch -= 360.0f;
        while (accPitch - gyroPitch < -180.0f) accPitch += 360.0f;
        float alpha = TAU_COMP / (TAU_COMP + dt);
        pitch = alpha * gyroPitch + (1.0f - alpha) * accPitch;
      } else {
        pitch = gyroPitch;
      }
    } else {
      // Echantillon manque : on NE coupe PAS les moteurs. On propage
      // l estimation avec le dernier gyro connu. Couper la commande a
      // chaque hoquet I2C (2,3 fois par seconde) desequilibrait le robot
      // bien plus que l erreur d estimation d un echantillon a 5 ms.
      i2cReadFail++;
      if (i2cFailStreak < 255) i2cFailStreak++;
      pitch += gyroRate * dt;
      if (i2cFailStreak == 20) i2cBusRecover();
    }

    // ---- capture, a la cadence de la boucle
    if (enrActif && enrN < ENR_N) {
      Echantillon &e = enr[enrN];
      e.t  = micros() - enrT0;
      e.p  = pitch - angleOffset;
      e.gf = gyroFilt;
      e.gb = gyroRate;
      e.s   = wheel_sps;
      e.inj = injCourant;
      // L accelerometre brut n avait jamais ete enregistre. Sans lui,
      // bruit_accel reste SUPPOSE dans modele.py -- c est le dernier
      // parametre de bruit qu on ne mesure pas.
      e.pa  = atan2f((float)rawAY, (float)rawAZ) * 180.0f / PI;
      e.mg  = sqrtf((float)rawAY*(float)rawAY + (float)rawAZ*(float)rawAZ) / 16384.0f;
      e.q    = pasMoyens();
      // 0 = roues figees, 3 = poussee imposee, 1 = cascade, 2 = desarme
      e.arme = (uint8_t)(lacherActif ? 0 : (pousseeActive ? 3 : (pidEnabled ? 1 : 2)));
      enrN++;
      if (enrN >= ENR_N) { enrActif = false; enrVidage = true; }
    }

    // Lissage dedie au terme D : sans lui les vibrations des pas-a-pas
    // dominent completement la sortie (cf. sps qui sautait a +/-500 alors
    // que l erreur d angle etait a 0.03 deg).
    gyroFilt += (gyroRate - gyroFilt) * (dt / (TAU_GYRO + dt));

    // Panne franche du capteur : la, on coupe.
    ETAPE(4);
    if (i2cFailStreak >= 10) {
      if (pidEnabled) {
        pidEnabled = false; dualPid = false;
        resetLoops();
        Serial.println(F(">>> SECURITE : MPU muet, PID coupe"));
      }
    } else if (pidEnabled) {
      if (fabsf(pitch - angleOffset) > FALL_LIMIT) {
        pidEnabled = false; dualPid = false;
        pendingArm = autoRearm;
        resetLoops();
        Serial.println(F(">>> SECURITE : angle hors limite, PID coupe"));
      } else if (satSince != 0 && (millis() - satSince) > SAT_MAX_MS) {
        pidEnabled = false; dualPid = false;
        pendingArm = autoRearm;
        satSince = 0;
        resetLoops();
        Serial.println(F(">>> SECURITE : vitesse saturee trop longtemps (roues dans le vide ?)"));
      } else {
        // L agent fait deja le travail de la boucle externe. Et surtout,
        // l auto-trim qu elle contient deplacerait angleOffset, c est-a-dire
        // la reference de verticale que l agent lit -- elle bougerait sous
        // ses pieds pendant qu il travaille.
        if (dualPid && !useAgent) {
          if (++outerCounter >= OUTER_DIV) {
            outerCounter = 0;
            ETAPE(5);
            outerLoop(dt * OUTER_DIV);
          }
        } else {
          target_angle = angleOffset;
          speed_angle_corr = 0.0f;
        }
        ETAPE(6);
        innerLoop(dt);
      }
    } else if (manualMode) {
      ETAPE(8);
      float d = MANUAL_ACCEL * dt;
      float e = manual_target_sps - wheel_sps;
      if (e >  d) e =  d;
      if (e < -d) e = -d;
      wheel_sps += e;
      applyMotorSpeed(wheel_sps);
    }

    // Tant que le robot est desarme et vraiment immobile, on affine le biais
    // du gyro. Le critere est la DISPERSION, pas la valeur : un biais faux de
    // 9 deg/s donne une lecture eloignee de zero mais tres stable.
    ETAPE(9);
    if (!pidEnabled && !manualMode && dataOk) {
      gyroBiasMean += (gyroRate - gyroBiasMean) * 0.01f;
      gyroBiasDev  += (fabsf(gyroRate - gyroBiasMean) - gyroBiasDev) * 0.01f;
      if (gyroBiasDev < 0.8f) gyroX_offset += gyroBiasMean * 0.002f;
    } else {
      gyroBiasMean = 0.0f;
      gyroBiasDev  = 10.0f;
    }

    // Re-armement automatique apres une chute (cmd 'H') : le robot repart
    // des qu on l a remis droit et qu il est reste immobile 1 seconde.
    // Evite d avoir a renvoyer K a la main entre deux essais de poussee.
    if (!pidEnabled && pendingArm && !manualMode && i2cFailStreak == 0
        && fabsf(pitch - angleOffset) < 2.0f && fabsf(gyroFilt) < 8.0f) {
      if (calmSince == 0) {
        calmSince = millis();
      } else if (millis() - calmSince > 1000) {
        resetLoops();
        pidEnabled = true;
        dualPid = wantDual;
        pendingArm = false;
        calmSince = 0;
        Serial.print(F(">>> Arme automatiquement en "));
        Serial.println(dualPid ? F("DUAL") : F("ANGLE"));
      }
    } else {
      calmSince = 0;
      // Sans ce message, un robot qui refuse de s armer semble simplement
      // "ne rien faire". On dit pourquoi, une fois par seconde au plus.
      if (!pidEnabled && pendingArm && !manualMode &&
          millis() - lastArmMsg > 1500) {
        lastArmMsg = millis();
        Serial.print(F(">>> en attente d armement : "));
        if (i2cFailStreak != 0) {
          Serial.println(F("capteur muet"));
        } else if (fabsf(pitch - angleOffset) >= 2.0f) {
          Serial.print(F("angle a ")); Serial.print(pitch - angleOffset, 1);
          Serial.println(F(" deg de l equilibre (il en faut moins de 2)"));
        } else {
          Serial.print(F("trop de mouvement, gyro ")); Serial.print(gyroFilt, 1);
          Serial.println(F(" deg/s (il en faut moins de 8)"));
        }
      }
    }
  }

  // Fin du deplacement minute : on repasse en maintien de position sur place.
  ETAPE(10);
  if (travelUntil && millis() >= travelUntil) {
    if (!travelReturning && fabsf(travelSpeedOut) > 0.5f && !travelPausing) {
      // Marquage d arret avant d inverser. Sans lui, on inverse la consigne
      // sous un robot encore lance : a-coups de +/-750 pas/s et erreur
      // d angle multipliee par dix pendant toute la manoeuvre.
      travelPausing = true;
      target_speed_sps = 0.0f;
      travelUntil = millis() + TRAVEL_PAUSE_MS;
      portENTER_CRITICAL(&timerMux);
      target_pos = (float)pasMoyens();
      portEXIT_CRITICAL(&timerMux);
      Serial.println(F(">>> Arret avant retour"));
    } else if (travelPausing) {
      travelPausing = false;
      travelReturning = true;
      target_speed_sps = -travelSpeedOut;
      travelUntil = millis() + (unsigned long)(travelDuration * 1000.0f);
      applyTravelPreload(target_speed_sps);
      Serial.print(F(">>> Retour a ")); Serial.print(target_speed_sps, 0);
      Serial.println(F(" pas/s"));
    } else {
      travelUntil = 0;
      travelReturning = false;
      travelPausing = false;
      travelSpeedOut = 0.0f;
      target_speed_sps = 0.0f;
      // On efface la dette de position : le robot tient la ou il est au
      // lieu de rattraper d un coup la distance non parcourue.
      portENTER_CRITICAL(&timerMux);
      target_pos = (float)pasMoyens();
      portEXIT_CRITICAL(&timerMux);
      Serial.println(F(">>> Aller-retour termine, maintien de position"));
    }
  }

  // ------------------------------------------------------ TEST DE SIGNE
  // Roues en l air, robot tenu a la main. On interroge la politique et on
  // affiche sa sortie SANS RIEN ENVOYER AUX MOTEURS. C est la seule
  // verification du portage qui ne peut pas se faire en simulation : si le
  // signe de l angle est inverse, l agent accelere du mauvais cote et le
  // robot part a pleine vitesse.
  ETAPE(11);
  if (testSigne && (millis() - lastSigneMsg) > 200) {
    lastSigneMsg = millis();
    float ang = pitch - angleOffset;
    // Etat remis a zero AVANT chaque mesure. politique_accel() memorise ses
    // deux actions precedentes (obs 9 a 12) : sans reset la valeur derive a
    // angle constant -- mesure 17489 puis 17645 puis 17739 pas/s2 a 0,20 deg,
    // alors que la table de reference y donne presque zero. En boucle fermee
    // cette memoire est correcte ; ici les roues ne repondent pas, on lui ment
    // avec sps 0 et pas 0, et il pousse de plus en plus fort.
    politique_reset();
    float a = politique_accel(ang, gyroFilt, 0.0f, 0L, 0.0f, millis());
    Serial.print(F("SIGNE  angle "));
    Serial.print(ang, 2);
    Serial.print(F(" deg  ->  a "));
    Serial.print(a, 0);
    Serial.print(F(" pas/s2"));
    if (politique_defaut == POL_DEFAUT_ANGLE) {
      Serial.print(F("   [hors domaine, > 14 deg]"));
    } else if (ang > 0.5f) {
      Serial.print(a > 0 ? F("   OK (avant -> positif)")
                         : F("   !! SIGNE INVERSE"));
    } else if (ang < -0.5f) {
      Serial.print(a < 0 ? F("   OK (arriere -> negatif)")
                         : F("   !! SIGNE INVERSE"));
    }
    Serial.println();
    politique_reset();              // le test ne doit rien laisser en travers
  }

  // Sauvegarde differee des reglages : jamais pendant une perturbation,
  // une ecriture flash peut prendre plusieurs millisecondes.
  // ------------------------------------------------------------- SECOUSSE
  ETAPE(12);
  if (testSecousse) {
    if (millis() - secousseT0 >= SECOUSSE_MS) {
      secousseT0 = millis();
      secousseSigne = -secousseSigne;
    }
    wheel_sps = (float)secousseSigne * SECOUSSE_AMP;
    applyMotorSpeed(wheel_sps);   // rearme aussi l homme mort de l ISR
  }

  ETAPE(13);
  if (paramsDirty && (millis() - lastParamChange) > 1500
      && (!pidEnabled || fabsf(pitch - target_angle) < 2.0f)) {
    saveParams();
    paramsDirty = false;
  }

  // --------------------------------------- POUSSEE PUIS GEL, DEUX FOIS
  if (lacherProchain && millis() >= lacherProchain && lacherIdx < 2) {
    pousseeSigne  = (lacherIdx == 0) ? +1.0f : -1.0f;   // avant, puis arriere
    pousseeActive = true;
    pousseeFin    = millis() + POUSSEE_MS;
    lacherIdx++;
    lacherProchain = 0;      // rearme a la fin du gel
  }
  if (pousseeActive && millis() >= pousseeFin) {
    pousseeActive = false;
    lacherActif = true;                      // le gel enchaine sans transition
    lacherFin   = millis() + LACHER_MAX_MS;
  }
  if (lacherActif) {
    bool trop = fabsf(pitch - angleOffset) > LACHER_ANGLE_MAX;
    if (trop || millis() >= lacherFin) {
      lacherActif = false;
      // 1,6 s pour que la cascade se remette d aplomb avant la seconde poussee
      if (lacherIdx < 2) lacherProchain = millis() + 1600;
    }
  }

  // ----------------------------------------------- ESSAI DE DECROCHAGE
  if (rampeActive) {
    float u = (micros() - rampeT0) * 1e-6f;
    if (u >= 6.0f) {
      rampeActive = false;
      manual_target_sps = 0.0f;
      Serial.println(F("=== rampe terminee, arret ==="));
    } else {
      manual_target_sps = RAMPE_ACCEL * u;
    }
  }

  // --------------------------------------------------- ESSAI D AVANCE
  if (avanceDebut && millis() >= avanceDebut) {
    target_speed_sps = AVANCE_SPS;
    avanceDebut = 0;
  }
  if (avanceFin && millis() >= avanceFin) {
    target_speed_sps = 0.0f;
    avanceFin = 0;
  }

  // ------------------------------------------------------------ VIDAGE
  // Une ligne fait ~55 octets ; 1200 lignes a 115200 bauds prennent 6 s. On
  // desarme pendant ce temps : l envoi bloque la boucle par paquets de 30 ms
  // et le robot ne tiendrait pas debout.
  if (enrVidage) {
    if (enrVide == 0) {
      Serial.print(F("ENR debut nom=")); Serial.print(enrNom);
      Serial.print(F(" n=")); Serial.println(enrN);
      Serial.println(F("ENR t_us,angle,gyro_f,gyro_b,sps,inj,acc_deg,acc_g,pos,etat"));
    }
    // UNE seule ligne par tour de boucle. A 200 Hz et ~60 octets la ligne, on
    // sort 12 ko/s, juste sous les 11,5 ko/s utiles du port a 115200 bauds : le
    // tampon absorbe la difference et l ecriture ne bloque jamais assez pour
    // gener la commande. Le robot reste donc DEBOUT pendant le vidage, ce qui
    // evite une chute par mesure -- c etait le vrai cout du protocole precedent.
    for (int k = 0; k < 1 && enrVide < enrN; k++, enrVide++) {
      Echantillon &e = enr[enrVide];
      Serial.print(F("ENR "));
      Serial.print(e.t);            Serial.print(',');
      Serial.print(e.p, 3);         Serial.print(',');
      Serial.print(e.gf, 2);        Serial.print(',');
      Serial.print(e.gb, 2);        Serial.print(',');
      Serial.print(e.s, 0);         Serial.print(',');
      Serial.print(e.inj, 3);       Serial.print(',');
      Serial.print(e.pa, 3);        Serial.print(',');
      Serial.print(e.mg, 4);        Serial.print(',');
      Serial.print(e.q);            Serial.print(',');
      Serial.println(e.arme);
    }
    if (enrVide >= enrN) {
      Serial.println(F("ENR fin"));
      enrVidage = false;
    }
    return;   // on saute la telemetrie, pas la commande : elle est deja passee
  }

  // ------------------------------------------------------------ TELEMETRIE
  ETAPE(14);
  if (millis() - lastDisplayTime >= 100) {
    lastDisplayTime = millis();
    int32_t pos = pasMoyens();

    Serial.print(F("P:"));    Serial.print(pitch, 2);
    Serial.print(F(" tgt:")); Serial.print(target_angle, 2);
    Serial.print(F(" e:"));   Serial.print(pitch - target_angle, 2);
    Serial.print(F(" g:"));   Serial.print(gyroFilt, 1);
    // Le pilote DOIT figurer dans le log : sans ca on ne sait pas, en
    // relisant, qui commandait -- et on s est deja fait prendre par un mode
    // qui avait bascule silencieusement.
    Serial.print(useAgent ? F(" AGENT") : F(" PID"));
    Serial.print(pidEnabled ? (dualPid ? F(" DUAL") : F(" ANG")) : F(" OFF"));
    Serial.print(F(" sps:")); Serial.print(wheel_sps, 0);
    Serial.print(F(" c:"));   Serial.print(speed_angle_corr, 2);
    Serial.print(F(" off:")); Serial.print(angleOffset, 2);
    Serial.print(F(" pos:")); Serial.print(pos);
    Serial.print(F(" hz:"));  Serial.print(loopHz, 0);
    Serial.print(F(" f:"));   Serial.print(i2cReadFail);
    Serial.print(F(" Lus:")); Serial.print(maxLoopUs);
    Serial.print(F(" Ius:")); Serial.print(maxI2cUs);
    if (useAgent) { Serial.print(F(" Pus:")); Serial.print(maxPolUs); }
    // Charge imposee au pilote de timer sur la fenetre de 100 ms :
    // Tp reprogrammations de la demi-periode, Tr rallumages du train.
    Serial.print(F(" Tp:")); Serial.print(nSetPeriod);
    Serial.print(F(" Tr:")); Serial.print(nRallumage);
    if (i2cBloc) { Serial.print(F(" BLOC:")); Serial.print(i2cBloc); }
    Serial.println();
    maxLoopUs = 0;
    maxI2cUs = 0;
    maxPolUs = 0;
    nSetPeriod = 0;
    nRallumage = 0;
  }

  ETAPE(15);
  uint32_t dLoop = micros() - tLoop;
  if (dLoop > maxLoopUs) maxLoopUs = dLoop;
}
