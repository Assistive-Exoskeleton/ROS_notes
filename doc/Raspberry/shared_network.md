# Shared network

## ROS DOMAIN ID
Il middleware usato di default da ROS 2 per le comunicazioni è `DDS`. Nel DDS, il `Domain ID` è utilizzato per poter far comunicare diversi network logici condividendo lo stesso network fisico, nello specifico, il Domain ID è usato dal DDS per settare le `porte UDP` che verranno utilizzate per la ricerca dei nodi e la comunicazione (i nodi in ROS 2 che condividono lo stesso dominio possono rintracciarsi e scambiarsi messaggi).

Di default il Domain ID utilizzato è 0, ma nel caso si vogliano evitando interferenze tra gruppi di computer sullo stesso network, è possibile impostare un diverso Domain ID per ogni gruppo.

## Implementazione automatica delle porte UDP

Ci sono diverse strategie usate per far sì che le applicazioni DDS comunichino tra di loro nonostante l’utilizzo di un firewall.

Le porte usate in una implementazione DDS sono specificate dall’ OMG DDS Interoperability Wire Protocol Specification (**DDS-RTPS**), di default, ogni DomainParticipant apre 4 porte UDP/IP: due di queste sono porte `multicast` e sono condivide su tutto il dominio dei partecipanti nello stesso Domain ID. Le altre due sono porte `unicast` e sono differenti per ciascun partecipante.

L’indirizzo IP del multicast usato per la rilevazione dei nodi è 239.255.0.1 (come da specifiche DDS-RTPS).

Le porte utilizzate dipendono da una formula legata al Domain ID e il numero di partecipanti ([clicca qui](https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through) per andare nel dettaglio).

Considerando che le porte UDP sono un unsigned 16-bit integer (massimo 65535 valori), il valore massimo impostabile come Domain ID è `232`.

## Specifiche legate alla piattaforma

Per la massima compatibilità, è utile evitare di allocare il Domain ID nel range delle `porte effimere`; Le porte effimere sono porte temporanee assegnate dallo stack IP di una macchina e vengono assegnate a questo scopo da un intervallo di porte designato. Utilizzando porte all’esterno del pool delle porte effimere permette di ridurre eventuali interferenze con servizi di rete del pc.

In base al sistema operativo, il pool delle porte effimere è:

| sistema operativo | Pool porte effimere  | Range Domain ID corrispondente | 
|-----|----|----|
|  Linux*   |  32768-60999  |  0-101 e 215-232  |  
|   Windows**  |  49152-65535  |  0-166  |  
|  macOS   |  49152-65535  | 0-166   | 



*il range delle porte effimere è configurabile editando il file in `/proc/sys/net/ipv4/ip_local_port_range`.

** il range delle porte effimere è configurabile usando `netsh`.

## Specifiche sui processi

Per ogni processo ROS 2 che viene eseguito su un computer, un partecipante DDS è creato; Siccome ogni partecipante DDS occupa fino a due porte su un computer, eseguire più di 120 processi ROS 2 su un computer può intaccare il corretto funzionamento del network (di default sono usati più Domain ID, in generale cosa non voluta). **Importante** ricordare che quando si utilizza un Domain ID al limite superiore (per esempio per linux usando 101 o 232) il massimo numero di processi diminuisce, poiché il range di porte UDP non è completo.

## Implementazione con simulink

È possibile implementare un nodo simulink scritto in C++ grazie all’integrazione di ROS morenia da matlab R2020b.

Ulteriori informazioni [qui](https://it.mathworks.com/help/ros/ug/generate-a-standalone-ros2-node-from-simulink.html).
