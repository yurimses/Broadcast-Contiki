 //Armazena o id do próprio mote
int my_id;

 //Vetor para as coordenadas X,Y e Z do próprio mote
unsigned int my_coordinate[3];

 //Vetor para as coordenadas dos eventos
unsigned int event[3];

 //Vetor para armazenar a diferença da subtração entre as coordenadas
unsigned int diff[3];

 //Carrega o ID do próprio mote
void load_id(void);

 //Carrega as coordenadas do próprio mote
void load_coordinate_mote(void);

 //Exibe a coordenada do mote
void show_coordinate_mote(void);

 //Carrega a coordenada do evento
void load_coordinate_event(int event_count);

 //Exibe a coordenada do evento
void show_coordinate_event(void);

 //Retorna count_motes
int return_count_motes(void);
