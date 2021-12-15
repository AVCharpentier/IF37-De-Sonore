//importation de la bibliothèque pour utiliser la centrale inertiel intégrée à l'arduino 33 BLE
#include <Arduino_LSM9DS1.h>
//importation de la bibliothèque pour utiliser le DFPlayer
#include "DFRobotDFPlayerMini.h"

//on définit les constantes pour plus de clareté
//les constantes suivantes correspond aux indices des valeur des composantes x,y et z des gyroscope/accéléromètre dans leur tableau respectif
#define X 0
#define Y 1
#define Z 2

//valeur seuil de détection d'un mouvement, plus on la réduit, plus la centrale inertielle détectera un mouvement plus rapidement et plus fréquemment
#define DETECTION_SEUIL 3
//durée dans laquelle le dé ne doit pas bouger pour etre déclaré comme à l'arrêt définitif
#define DETECTION_TEMPORISATION 1000 //en ms

//on définit les composantes liée à la partie audio : DFPlayer
#define AUDIO_VOLUME 20 //niveau sonore de l'haut parleur compris entre 0 et 30
#define AUDIO_TEMPORISATION 4000 //durée enter deux piste audio, en ms
//on définit l'index de chaque musique sur la carte SD
#define AUDIO_PISTE_1 3
#define AUDIO_PISTE_2 2
#define AUDIO_PISTE_3 4
#define AUDIO_PISTE_4 5
#define AUDIO_PISTE_5 6
#define AUDIO_PISTE_6 1

//variable qui stocke quand à était la dernière fois que l'on a joué un son
unsigned long tempsDebutTemporisationAudio = 0;
DFRobotDFPlayerMini myDFPlayer;

//variable qui stocke les valeur lu par le gyroscope/acceleromètre a chaque fois
float valeurGyroscope[3] = {0,0,0};
float valeurAccelerometre[3] = {0,0,0};

//variables qui stocke le décallage des valeurs du gyroscope/acceleromètre lorsqu'il ne bouge pas
//pour qu'elles se rapprochent le plus possible du 0
float offsetGyroscope[3] = {0,0,0};
float offsetAccelerometre[3] = {0,0,0};

//variable qui stocke quand débute la temporisation de la détection de mouvement
unsigned long tempsDebutTemporisationDetection = 0;
//variable qui stocke l'état du dé, s'il est en mouvement ou non
int arreter = 0;

//variable qui stocke la valeur du dé
int valeurDuDe = 0;

float calcModule(float tab[3]){
  //fonction qui permet de calculer un module à partir d'un tableau de trois composantes
  return sqrt(tab[X]*tab[X]+tab[Y]*tab[Y]+tab[Z]*tab[Z]);
}

void lectureValeurGyroscope(){
  //procédure qui permet de lire les valeurs du gyroscope
  float xGyr, yGyr, zGyr;

  if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(xGyr, yGyr, zGyr);
        //on n'oublit pas de retirer le décallage
        valeurGyroscope[X] = xGyr - offsetGyroscope[X];
        valeurGyroscope[Y] = yGyr - offsetGyroscope[Y];
        valeurGyroscope[Z] = zGyr - offsetGyroscope[Z];
    }
}

void lectureValeurAccelerometre(){
  //procédure qui permet de lire les valeurs de l'accéléromètre
  float xAcc, yAcc, zAcc;

  if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(xAcc, yAcc, zAcc);
        //on n'oublit pas de retirer le décallage
        valeurAccelerometre[X] = xAcc - offsetAccelerometre[X];
        valeurAccelerometre[Y] = yAcc - offsetAccelerometre[Y];
        valeurAccelerometre[Z] = zAcc - offsetAccelerometre[Z];
    }
}

void lectureCapteurMouvement(){
  //procédure qui permet de lancer une série de mesures des gyroscope & accéléromètre
  lectureValeurGyroscope();
  lectureValeurAccelerometre();
}

void affichageGyroscope(){
  //procédure qui permet d'affiner dans la console les valeurs des différentes composantes du gyroscope
  Serial.print(valeurGyroscope[X]);
  Serial.print('\t');
  Serial.print(valeurGyroscope[Y]);
  Serial.print('\t');
  Serial.println(valeurGyroscope[Z]);
}

void affichageAccelerometre(){
  //procédure qui permet d'affiner dans la console les valeurs des différentes composantes de l'accéléromètre
  Serial.print(valeurAccelerometre[X]);
  Serial.print('\t');
  Serial.print(valeurAccelerometre[Y]);
  Serial.print('\t');
  Serial.println(valeurAccelerometre[Z]);;
}

void calibrerGyroscopeAccelerometre(){
  //procédure qui permet de calibrer le gyroscope & accéléromètre
  Serial.println("Début de la calibration du gyroscope et de l'accéléromètre");
  int nbIteration = 50;

  for(int i=0;i<nbIteration;i++){
    //on lance une capture des valeur des capteurs
    lectureCapteurMouvement();

    //on calibre le gyroscope
    offsetGyroscope[X] += valeurGyroscope[X];
    offsetGyroscope[Y] += valeurGyroscope[Y];
    offsetGyroscope[Z] += valeurGyroscope[Z];

    //on calibre l'accéléromètre
    offsetAccelerometre[X] += valeurAccelerometre[X];
    offsetAccelerometre[Y] += valeurAccelerometre[Y];
    offsetAccelerometre[Z] += valeurAccelerometre[Z];


    //on attend un petit peut de temps
    delay(2);
  }

  Serial.println("Fin de la calibration");
}

int deEnMouvement(){
  //fonction qui permet de déterminer si le dé est ou mouvement ou non
  // 1 = oui, 0 = non

  //on lance des mesures du gyroscope & accéléromètre
  lectureCapteurMouvement();

  //on calcule le module des valeur obtenue
  float normeAcc = calcModule(valeurAccelerometre);
  float normeGyr = calcModule(valeurGyroscope);

  //on regarde si le dé est en mouvement ou non
  if(normeGyr+normeAcc < DETECTION_SEUIL){
    //si le dé n'est pas en mouvement
    return 0;
  }
  //si le dé est en mouvement
  return 1;
}

void detectionGenerationNombre(){
  //procédure qui permet de gérer la temporisation à l'arret en fonction des changement d'état du mouvement du dé
  int enMouvement = deEnMouvement();

  //on regarde si le dé est en mouvement ou non
  if(enMouvement == 0){
    //si le dé n'est pas en mouvement
    if(arreter == 0){
      //s'il vient tout juste de s'arrêter
      //on indique qu'il est à l'arrêt
      arreter = 1;
      //on stocke le temps à partir duquel il vient de s'arrêter
      tempsDebutTemporisationDetection = millis();
    }else{
      //si le dé était déàj à l'arrêt
      //on regarde si cela fait le temps voulu à l'arret (temporisation) et que l'on n'a pas déjà attribué une valeur au dé
      if(millis()-tempsDebutTemporisationDetection > DETECTION_TEMPORISATION && valeurDuDe == 0){
        //si le dé n'avait pas de valeur et que la tmeporisation vient tout juste de ce terminer, alors on attribut au dé ça nouvelle valeur
        //il prend une nouvelle valeur comprise entre 1 et 6
        valeurDuDe = random(1,7);
        //on affiche la nouvelle valeur du dé dans la console
        Serial.print("Nouvelle valeur du dé est ");
        Serial.println(valeurDuDe);
      }
    }
  }else{
    //si le dé est en mouvement
    //on indique comme quoi il n'est pas à l'arrêt
    arreter = 0;
    //on retire la valeur du dé lorsqu'il est en mouvement
    valeurDuDe = 0;
  }
}


void initialisationDFPlayer(){
  //on intiialise la deuxième liaison série pour le DFPlayer
  Serial1.begin(9600);

  //on affiche comme quoi on tente de l'initialiser
  Serial.println("Initialisation en cours du DFPlayer ... (Cela peut prendre entre 3~5 secondes)");

  //on regarde si la liaison série avec le dfplayer est bien établit
  if (!myDFPlayer.begin(Serial1)) {  
    //si la liaison n'est pas bien établit on l'affiche dans la console des points qu'il peut vérifier
    Serial.println("Impossible de démarer la liaison série avec le DFPlayer :");
    Serial.println("1. Verifier les connections !");
    Serial.println("2. Verifier si la carte SD est bien insérée !");
    //on boucle tant qu'elle n'est pas démarré
    while(!myDFPlayer.begin(Serial1)){
      delay(0); 
    }
  }
  //on affiche comme quoi la liaison série est bien établit entre l'arduino et le DFPlayer
  Serial.println("Le DFPlayer est opérationel.");
  //on modifie le volume sonore de l'haut parleur
  myDFPlayer.volume(AUDIO_VOLUME);  
}

void direValeurAudio(){
  //procédure qui permet de dire oralement la valeur du dé
  //on regarde dans un premier temps si une valeur est attribué au dé
  if(valeurDuDe != 0){
    //on regarde si on doit lancer une nouvelle piste audio : deux piste audios sont séparé de AUDIO_TEMPORISATION
    if((millis()-tempsDebutTemporisationAudio) > AUDIO_TEMPORISATION){
      //on enregistre quand débute la piste sonore, pour éviter de rejouer tout de suite la piste audio
      tempsDebutTemporisationAudio = millis();

      //on lance la piste audio en fonction de la valeur obtenu
      switch(valeurDuDe){
          //on regarde si c'est un 1
          case 1:
            myDFPlayer.play(AUDIO_PISTE_1);
            break;
          //on regarde si c'est un 2
          case 2:
            myDFPlayer.play(AUDIO_PISTE_2);
            break;
          //on regarde si c'est un 3
          case 3:
            myDFPlayer.play(AUDIO_PISTE_3);
            break;
          //on regarde si c'est un 4
          case 4:
            myDFPlayer.play(AUDIO_PISTE_4);
            break;
          //on regarde si c'est un 5
          case 5:
            myDFPlayer.play(AUDIO_PISTE_5);
            break;
          //on regarde si c'est un 6
          case 6:
            myDFPlayer.play(AUDIO_PISTE_6);
            break;
      }
    }
  }else{
    tempsDebutTemporisationAudio = 0;
  }
}

void initialisationIMU(){
  //on regarde si la centrale inertielle est bien démarré
  if (!IMU.begin()) {
    //si elle n'est pas démaré, on l'affiche dans la console
    Serial.println("Problème d'initialisation de la centrale inertielle");
    //on répète tant qu'elle n'est pas activé
    while (!IMU.begin());
  }
}

void setup() {
  //on initialise la liaison série pour la console
  Serial.begin(115200);
  
  //on ne regarde pas que la liaison série avec la console soit bien établit, pour éviter que cela pose des problèmes sans USB

  //on initialise le DFPLayer
  initialisationDFPlayer();
  
  //initialisation de la centrale inertielle intégrée à l'arduino 33 BLE
  initialisationIMU();
  
  //on calibre le gyroscope & accéléromètre pour qu'ils donnent par la suite des valeur le plus proche possible de 0
  calibrerGyroscopeAccelerometre();
}

void loop() {
  //on regarde s'il faut attribuer une valeur ou non au dé
  detectionGenerationNombre();
  
  //on affiche d'un point de vu sonore le résultat obtenu si besoin
  direValeurAudio();
}
