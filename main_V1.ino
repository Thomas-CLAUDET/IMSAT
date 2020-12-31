// Ordonnanceur
// V1 en monothread, on rajoutera les semaphores, threads, mutex... avec la librairie arduino_free_RTOS en V2


#define   NumTache      6 //capteur P,T,z,H / capteur accel,gyro / GPS / écriture carte SD / TX / RX en continu donc pas une tâche / Commande chaufferette / 
#define   DureeRep_ms   6000

   
unsigned char Tache_on[NumTache],Tache_off[NumTache];
unsigned long Tache_start[NumTache],i;
unsigned long Master_time=0, Tache_duree[NumTache];
unsigned char Tache_Active=0; 

void setup()
{
  // Init écheances des taches (en ms)  
  Tache_duree[0]=1000; // capteur P,T,z,H
  Tache_duree[1]=1000; // capteur accel, gyro
  Tache_duree[2]=1000; // capteur GPS
  Tache_duree[3]=1000; // commande chaufferette
  Tache_duree[4]=1000; // écriture carte SD
  Tache_duree[5]=1000; // TX

  // Init temps de début des tâches (en ms) 
  Tache_start[0]=1000;
  Tache_start[1]=2000;
  Tache_start[2]=3000;
  Tache_start[3]=4000;
  Tache_start[4]=5000;
  Tache_start[5]=6000;

}

void loop()
{
  // Lecture de l'ordonnanceur 
  Master_time=millis(); 

  // Init de l'ordonnanceur avec la période de répétition 
  Master_time=(Master_time % DureeRep_ms);
  
  // Génération des conditions de début des taches  
  for (i=0;i<NumTache;i++)
  {
    Tache_on[i]=(Master_time>Tache_start[i]);
  }

  // Génération des conditions d'arrêts   
  for (i=0;i<NumTache;i++)
  {
    Tache_off[i]=Tache_on[i] & (Master_time<(Tache_start[i]+Tache_duree[i]));
  }

  // Exécution des tâches 
  // Tâche 1 
  if(Tache_off[0])
  {
    Tache_Active=1;
  }

  // Tâche 2
  else if(Tache_off[1])
  {
    Tache_Active=2;
  }

  // Tâche 3 
  else if(Tache_off[2])
  {
    Tache_Active=3;
  }

  // Tâche 4
  else if(Tache_off[3])
  {
    Tache_Active=3;
  }

  // Tâche 5 
  else if(Tache_off[4])
  {
    Tache_Active=4;
  }

  // Tâche 5 
  else if(Tache_off[5])
  {
    Tache_Active=5;
  }

  // Autres: Désactivation des sorties 
  else 
  {
    Tache_Active=0;
  }
   
  // Affichage de la tache active 
  Serial.print(Master_time/1000);
  Serial.print(":");
  Serial.println(Master_time%1000);
}
