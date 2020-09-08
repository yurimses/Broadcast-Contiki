#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "node-id.h"

#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
//#include "sys/clock.h"

#include "sys/battery_kinetic.h"

/*Instances*/
#include "sys/periodic20.h"
#include "sys/periodic30.h"
//#include "sys/periodic40.h"
//#include "sys/periodic50.h"

#include "sys/instance_pool.h"

#define DEBUG DEBUG_NONE
//#define DEBUG DEBUG_FULL
//#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

//yuri
//##########################################################
//#include "rest-engine.h"
#include <math.h>
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ipv6/uip-ds6.h"
#include "simple-udp.h"
//#include "er-coap-observe.h"
#include "coordinates.h"
#include "events.h"
#include "priority_events.h"
#include "testando.h"
//##########################################################


#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define NB_SERVERS 1
#define NB_INSTANCES 1

/*
#define SEND_DELAY (60 * CLOCK_SECOND + (random_rand() % (20 * CLOCK_SECOND)))
#define SEND_TIME (20 * CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)))
#define MAX_PAYLOAD_LEN 50 //Sidnei alterou de 50 para 60
*/

#ifndef PERIOD
#define PERIOD 60 * 7 // Sidnei alterou de 60 para 10
#endif


#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		40
#define STOP_TIME		(300 * CLOCK_SECOND)

//yuri
//###############################################################################
  //Tempo de cada evento
#define SECONDS 60

  //Área de cobertura de cada mote
#define RANGE 10

#define UDP_PORT 1234

#define SEND_INTERVAL		(20 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

//###############################################################################


static struct uip_udp_conn *client_conn;
/* server addresses definition */
static uip_ipaddr_t server_ipaddr[1];
static uint8_t instance_ids[1];


//YURI
//##########################################################
//Contagem de eventos
unsigned int event_count=0; 

//Estabelecer struct broadcast para registro
static struct simple_udp_connection broadcast_connection;

//Para função clean
static struct ctimer timer;

//Para OBS
//static struct etimer add_obs_timer;

//Variáveis para vetor que armazena classificação / nível do evento
static int count = 0;

//Vetor para função clean
static int vt[10];

//Flag para resposta da função clean
static int is_event_cln = 0;
//##########################################################


//YURI
//##########################################################
//Função receiver do broadcast
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
	//Mensagem recebida
	printf("Dados recebidos do end.: ");
  	uip_debug_ipaddr_print(sender_addr);
	printf(" na porta %d oriundos da porta %d com tam.: %d: '%s'\n",
        receiver_port, sender_port, datalen, data);
	
	//Vetor que armazena ocorrência de evento, limitando o tamanho em 10
	if (count < 10){
		int j = 0;
		vt[count+1] = 1;
		for (j = 0; j<10; j++){
			printf("%d ", vt[j]);
		}	
		printf("\n");	
		count++;
	}
	printf("contador de respostas vizinhas: %d\n", count);

}
// 1 ocorreu evento crítico (nivel 1)
// 2 ocorreu evento não crítico (nivel 2)

//Função para limpar o contador e o vetor da função receiver
static void clean(void *ptr){
            int k;
	    //K-out-of-N
	    //Soma para verificar a ocorrência de evento
	    int sum = 0;
	    if (count_motes < 10){ 
		    for (k = 0; k < count_motes; k++){
			sum += vt[k];
			printf("%d ", vt[k]);
		    }
	    }
	    if (count_motes == 10){
	    		for (k = 0; k < 10; k++){
				sum += vt[k];
				printf("%d ", vt[k]);			
			}	
	    }
	    
            printf("\nSoma: %d\n", sum);
	    
            if (sum >= 1){
		printf("Verdadeiro Positivo (TP)\n");
		is_event_cln = 1;
	    }else{
		printf("Falso Positivo (FP)\n");
		is_event_cln = 0;
	    }
            
	    count = 0;
	    for (k = 0; k < 10; k++){
	   	vt[k] = 0;
	    }
}

//##########################################################


PROCESS(custom_process, "Client process 10");
AUTOSTART_PROCESSES(&custom_process);
/*---------------------------------------------------------------------------*/
static int seq_id;

int timeline[24] = { 1, 2, 4, 2, 4, 1, 3, 1, 1, 2, 4, 2, 4, 1, 3, 1, 1, 2, 4, 2, 4, 1, 3, 1};
int randomtime[3][23] = { { 0, 2, 0, 3, 0, 0, 15, 0, 0, 6, 0, 13, 0, 0, 12, 0, 0, 19, 0, 10, 0, 0, 27, 0},
                          { 0, 0, 6, 0, 9, 0, 0, 0, 0, 0, 10, 0, 17, 0, 0, 0, 0, 0, 15, 0, 3, 0, 0, 0},
                          { 0, 0, 28, 0, 13, 0, 0, 0, 0, 0, 13, 0, 17, 0, 0, 0, 0, 0, 22, 0, 14, 0, 0, 0} };
int schedule1 = 1, schedule2 = 2, schedule3 = 3, schedule4 = 4;

//static int reply;
//static int seq_instance_id;
//static struct ctimer instance_switch_timer;

static void
tcpip_handler(void)
{
/*  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    reply++;
    printf("DATA recv client '%s' (s:%d, r:%d)\n", str, seq_id, reply);
  }*/
}
/*---------------------------------------------------------------------------*/
static void send_packet() 
{
  char buf[MAX_PAYLOAD_LEN];
  seq_id++;

  printf("DATA send to %d ", server_ipaddr[(seq_id-1)%NB_SERVERS]);
  printf("'Hello %d' (via Instance %d) \n", seq_id, instance_ids[(seq_id-1)%NB_INSTANCES]);
  sprintf(buf, "Hello %d from the client (via Instance %d)", seq_id, instance_ids[(seq_id-1)%NB_INSTANCES]);

  client_conn->rpl_instance_id=instance_ids[(seq_id-1)%NB_INSTANCES];

  uip_udp_packet_sendto(client_conn, &buf, strlen(buf), &server_ipaddr[(seq_id-1)%NB_SERVERS], UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(custom_process, ev, data)
{

//yuri
//##########################################################
//Mensagem a ser enviada pelo broadcast
  char msg[100];
//##########################################################

  static struct etimer periodic_send;
  static struct ctimer backoff_timer;
  int status, hour, minute, mod;

  //Ariker> Battery Settings
  unsigned seconds=60*5;// warning: if this variable is changed, then the kinect variable the count the minutes should be 			        changed
  double fixed_perc_energy = 1;// 0 - 1 //Sidnei alterou de 0.2 para 0.1
  unsigned variation = 1;//0 - 99 //alterou de 2 para 1
  int state_client = 4;
  double batt_perc = 1;//0.3;

  unsigned event3 = 60*3;
  unsigned event5 = 60*5;

  PROCESS_BEGIN();

//YURI
//##########################################################
   is_event_cln = 0;  


   static struct etimer et;

  //Endereço a ser usado pelo broadcast
  uip_ipaddr_t addr;

  //Registrar broadcast
  simple_udp_register(&broadcast_connection, UDP_PORT,
  NULL, UDP_PORT, receiver); 

  //Armazena o id do próprio mote  
  int my_id;

  //Vetor para as coordenadas X,Y e Z do próprio mote
  unsigned int my_coordinate[3];

  //Vetor para as coordenadas dos eventos
  unsigned int event[3];

  //Vetor para armazenar a diferença na subtração entre as    coordenadas
  unsigned int diff[3];

  //Níveis de classificação / prioridade
  unsigned int priority;
//##########################################################

  instance_20(CLOCK_SECOND * event5);
  instance_30(CLOCK_SECOND * event3);

  //Ariker> add this line

  battery_start(CLOCK_SECOND * seconds, seconds, fixed_perc_energy, variation, state_client, batt_perc);


  PROCESS_PAUSE();

  PRINTF("Simple App started\n");

  /* authorized instances */
  /*rpl_add_authorized_instance_id(10);*/
  /*rpl_add_authorized_instance_id(20);*/


  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  /*Start Pool*/
  start_pool();


/* server addresses initialization */
  uip_ip6addr(&server_ipaddr[0], 0xfd00,0,0,0,0,0,0,1);
  instance_ids[0]=10;

  etimer_set(&periodic_send, SEND_INTERVAL);
  //etimer_set(&et, CLOCK_SECOND*SECONDS);
  while(1) {
    PROCESS_YIELD();
//YURI

//##########################################################
etimer_set(&et, CLOCK_SECOND*SECONDS);
	    PROCESS_WAIT_EVENT();
	change_instance_status(1,instance_ids[0]);
	change_instance_status(1,20);	
		

        //Mote busca o seu próprio id e subtrai 2 de seu valor   
        my_id=node_id-2;

        /*Mote busca sua própria coordenada X,Y e Z dentro da matriz de coordenadas
        no arquivo coordinate.h e armazena elas no vetor*/
        my_coordinate[0]=(unsigned int)(motes_coordinates[my_id][0]*100);
        my_coordinate[1]=(unsigned int)(motes_coordinates[my_id][1]*100);
        my_coordinate[2]=(unsigned int)(motes_coordinates[my_id][2]*100);

        //O mote exibe os valores X,Y e Z de sua coordenada
        printf("Coordenada X: %u\n",my_coordinate[0]);
        printf("Coordenada Y: %u\n",my_coordinate[1]);
        printf("Coordenada Z: %u\n",my_coordinate[2]);

	printf("vetor teste: 0/1 %d\n", vetor_teste[0][1]);

        //Se valor do contador de eventos for menor que total de eventos
        if(event_count<total_events){
            /*Mote busca a coordenada X,Y e Z dentro da matriz de eventos
            no arquivo events.h e armazena elas no vetor*/
            event[0]=(unsigned int)(events_coordinates[event_count][0]*100);
            event[1]=(unsigned int)(events_coordinates[event_count][1]*100);
            event[2]=(unsigned int)(events_coordinates[event_count][2]*100);

            //Mote exibe os valores X,Y e Z do evento
            printf("Coordenada X do evento: %u\n",event[0]);
            printf("Coordenada Y do evento: %u\n",event[1]);
            printf("Coordenada Z do evento: %u\n",event[2]);

            int i;
            int sender_id = node_id;

            //Calcula a diferença entre coordenadas X,Y e Z do mote e do evento
            for(i=0;i<3;i++){
                if(event[i]>my_coordinate[i]){
                    diff[i]= event[i]-my_coordinate[i];  
                }else{
                    diff[i]=my_coordinate[i]-event[i];
                }
            }

            //Calcula a distância euclidiana entre o mote e o evento
            unsigned distance = (unsigned int)((sqrt(pow(diff[0],2)+pow(diff[1],2)+pow(diff[2],2))));

            printf("Distancia: %u\n",distance);
      
            //Estabelecendo prioridade para cada evento
            priority = priority_events[event_count];

            //Se a distancia calculada for menor igual ao range, o mote vai enviar broadcast
            if((distance/100)<=RANGE){	
    	        printf("Detectou evento\n");
    	        printf("Enviando broadcast\n");	

		//Nó informa que detectou evento ao vetor indice [0]
		//indice [0] dedicado exclusivamente ao próprio nó
		vt[0] = 1;

		//Constrói mensagem broadcast
    	        sprintf(msg, "Mote: %d aconteceu com nivel %d", sender_id, priority);
		
		//Set IP address addr to the link local all-nodes multicast address
    	        uip_create_linklocal_allnodes_mcast(&addr);
 
		//Send a UDP packet to a specified IP address.
        	simple_udp_sendto(&broadcast_connection, msg, strlen(msg), &addr);
		/*
		if (priority == 2){
			change_instance_status(2, instance_ids[0]);
		}else{
			change_instance_status(2, instance_ids[0]);
			change_instance_status(2, 20);
		}
		*/	
	 	
    	
            } //fim do if distance

        } //fim do event_count<total_events

        //Acrescenta 1 para o próximo evento
        event_count++;  
        
	//Para a função clean ser chamada a cada 30 segundos	
	ctimer_set(&timer, CLOCK_SECOND*30, clean, NULL);

	printf("is_event_cln: %d\n", is_event_cln);
	printf("prioridade: %d\n", priority);
	if (is_event_cln == 1){
		if (priority == 2){
			change_instance_status(2, instance_ids[0]);
		}else{
			change_instance_status(2, instance_ids[0]);
			change_instance_status(2, 20);
		}
	}   

 
	//Se o tempo estimado expirar, reinicia a contagem
	if(etimer_expired(&et)) {
	    etimer_reset(&et);
        }
//##########################################################

    if(ev == tcpip_event) {
      tcpip_handler();
    }

    hour = ((clock_time() / CLOCK_SECOND) / 3600);
    minute = ((clock_time() / CLOCK_SECOND) / 60);
    mod = minute % 60;

    //printf("TEMPO Hora: %d Minuto: %d \n", hour, minute);
/*
    //Schedule 1
    if(timeline[hour] == schedule1)
    {
      if(mod < 3)
      {
        printf("Entrou no MOD Schedule 1\n");
        change_instance_status(2, instance_ids[0]);
      }else
      {
        printf("Saiu do MOD Schedule 1\n");
        change_instance_status(1, instance_ids[0]);
      }
      change_instance_status(3, 20);
      change_instance_status(3, 30);
    }

  
    //Schedule 2
    if(timeline[hour] == schedule2)
    {
      if(mod < 3)
      {
        printf("Entrou no MOD Schedule 2\n");
        change_instance_status(2, 10);
        //change_instance_status(2, 20);
      }
      if(mod > randomtime[0][hour] && mod < (randomtime[0][hour] + 3) )
      {
        printf("Saiu do MOD Schedule 2-1\n");
        change_instance_status(1, 10);
        change_instance_status(2, 20);
      }
      if(mod > randomtime[0][hour] + 3 && mod < (randomtime[0][hour] + 20) )
      {
        printf("Saiu do MOD Schedule 2-2\n");
        change_instance_status(1, 10);
        change_instance_status(1, 20);
      }else
      {
         printf("Saiu do MOD Schedule 2-3\n");
         change_instance_status(1, 10);
         change_instance_status(3, 20);
      }
      change_instance_status(3, 30);
    }

    //Schedule 3
    if(timeline[hour] == schedule3)
    {
      if(mod < 3)
      {
        printf("Entrou no MOD Schedule 3\n");
        change_instance_status(2, 10);
        //change_instance_status(2, 30);
      }
      if(mod > randomtime[0][hour] && mod < (randomtime[0][hour] + 3) )
      {
        printf("Saiu do MOD Schedule 3-1\n");
        change_instance_status(1, 10);
        change_instance_status(2, 30);
      }
      if(mod > randomtime[0][hour] + 3 && mod < (randomtime[0][hour] + 20) )
      {
        printf("Saiu do MOD Schedule 3-2\n");
        change_instance_status(1, 10);
        change_instance_status(1, 30);
      }else
      {
         printf("Saiu do MOD Schedule 3-3\n");
         change_instance_status(1, 10);
         change_instance_status(3, 30);
      }
      change_instance_status(3, 20);
    }

    //Schedule 4
    if(timeline[hour] == schedule4)
    {
      if(mod < 3)
      {
        printf("Entrou no MOD Schedule 4\n");
        change_instance_status(2, 10);
        //change_instance_status(2, 20);
        //change_instance_status(3, 30);
      }
      if(mod > randomtime[1][hour] && mod < (randomtime[1][hour] + 3) )
      {
        printf("Saiu do MOD Schedule 4-1\n");
        change_instance_status(1, 10);
        change_instance_status(2, 20);
        change_instance_status(3, 30);
      }
      if(mod > randomtime[1][hour]+3 && mod < (randomtime[1][hour] + 20) )
      {
        printf("Saiu do MOD Schedule 4-2\n");
        change_instance_status(1, 10);
        change_instance_status(1, 20);
        change_instance_status(3, 30);
      }
      if(mod > randomtime[2][hour] && mod < (randomtime[2][hour] + 3) )
      {
        printf("Saiu do MOD Schedule 4-3\n");
        change_instance_status(1, 10);
        change_instance_status(3, 20);
        change_instance_status(2, 30);
      }
      if(mod > randomtime[2][hour] + 3 && mod < (randomtime[2][hour] + 20) )
      {
        printf("Saiu do MOD Schedule 4-4\n");
        change_instance_status(1, 10);
        change_instance_status(3, 20);
        change_instance_status(1, 30);
      }else
      {
        printf("Saiu do MOD Schedule 4-5\n");
        change_instance_status(1, 10);
        change_instance_status(3, 20);
        change_instance_status(3, 30);
      }

    }

*/
    if(etimer_expired(&periodic_send))
    {
      int status10 = get_instance_status(instance_ids[0]);
      int status20 = get_instance_status(20);

      if(status10 == 2 || status20 == 2)
      {
        printf("Enviando mensagem\n");
        ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);
      }
      etimer_reset(&periodic_send);
    }
  }

  PROCESS_END();
}
