/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) REST Engine example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "rest-engine.h"
#include "sys/node-id.h"
#include <math.h>
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ip/uip-debug.h"
#include "net/ipv6/uip-ds6.h"
#include "simple-udp.h"



//Arquivo com as coordenadas dos motes
#include "coordinates.h"
//Arquivo com as coordenadas dos eventos
#include "events.h"
//Arquivo com flag sobre evento e a informação sobre ele
#include "resources/res-hello.h"
//Arquivo com as informações de prioridade de eventos
#include "priority_events.h"

//Ariker> add this line
//#include "../apps/powertrace/powertrace.h"
#include "powertrace.h"
//###############################################################################
  //Tempo de cada evento
#define SECONDS 60

  //Área de cobertura de cada mote
#define RANGE 10

#define UDP_PORT 1234

#define SEND_INTERVAL		(20 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

//###############################################################################


#if PLATFORM_HAS_BUTTON
#include "dev/button-sensor.h"
#endif
/*
#define DEBUG 0
#if DEBUG
#include <stdio.h>

#define PRINTF(...) printf(__VA_ARGS__)
define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif
*/
/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */

//#############################################################################

unsigned int event_count=0; 

static struct simple_udp_connection broadcast_connection;

static int count = 0;

static int vt[10];
//#############################################################################



extern resource_t
  res_hello,
  res_mirror,
  res_chunks,
  res_separate,
  res_push,
  res_event,
  res_sub,
  res_b1_sep_b2;
#if PLATFORM_HAS_LEDS
extern resource_t res_leds, res_toggle;
#endif
#if PLATFORM_HAS_LIGHT
#include "dev/light-sensor.h"
extern resource_t res_light;
#endif
/*
#if PLATFORM_HAS_BATTERY
#include "dev/battery-sensor.h"
extern resource_t res_battery;
#endif
#if PLATFORM_HAS_RADIO
#include "dev/radio-sensor.h"
extern resource_t res_radio;
#endif
#if PLATFORM_HAS_SHT11
#include "dev/sht11/sht11-sensor.h"
extern resource_t res_sht11;
#endif
*/
/*-------------------------------------------------*/
PROCESS(test_timer_process, "Test timer");
//AUTOSTART_PROCESSES(&test_timer_process);
/*-------------------------------------------------*/
//PROCESS(broadcast_example_process, "UDP broadcast example process");
//AUTOSTART_PROCESSES(&broadcast_example_process);

PROCESS(er_example_server, "Erbium Example Server");
AUTOSTART_PROCESSES(&er_example_server,&test_timer_process);

static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen/*,
         void (*receiver1)(void)*/)
{
		
	//uip_ipaddr_t addr_test;			
	
	//CONSULTA
	printf("Dados recebidos do end.: ");
  	uip_debug_ipaddr_print(sender_addr);
	//PRINT6ADDR(&sender_addr);
	printf(" na porta %d oriundos da porta %d com tam.: %d: '%s'\n",
        receiver_port, sender_port, datalen, data);
	//printf("%c\n", data[datalen-1]);
	int type_reply = (int) data[datalen-1] - 48;
	int j = 0;
	//printf("contador: %d\n", count);
	vt[count] = type_reply;
	for (j = 0; j<10; j++){
		printf("%d ", vt[j]);
	}	
	printf("\n");	
	count++;
	printf("contador: %d\n", count);
	//Espaço dedicado a confirmação de que recebeu o broadcast
	//if (((data[datalen-1] == '1') || (data[datalen-1] == '2')) /*&& is_event == 1*/){	
	//	printf("reply\n");		
		//uip_create_linklocal_allnodes_mcast(&addr_test);
		//simple_udp_sendto(&broadcast_connection, "oi", 3 , &addr_test);
	//}
	

}
// 1 ocorreu evento crítico (nivel 1)
// 2 ocorreu evento não crítico (nivel 2)


PROCESS_THREAD(er_example_server, ev, data)
{
  PROCESS_BEGIN();
// Ariker> Battery Settings
unsigned seconds=60*5;// warning: if this variable is changed, then the kinect variable the count the minutes should be changed
double fixed_perc_energy = 0.2;// 0 - 1
unsigned variation = 2;//0 - 99


//Ariker> add this line
powertrace_start(CLOCK_SECOND * seconds, seconds, fixed_perc_energy, variation);
//powertrace_start(CLOCK_SECOND * 10);


  PROCESS_PAUSE();

  PRINTF("Starting Erbium Example Server\n");



#ifdef RF_CHANNEL
  PRINTF("RF channel: %u\n", RF_CHANNEL);
#endif
#ifdef IEEE802154_PANID
  PRINTF("PAN ID: 0x%04X\n", IEEE802154_PANID);
#endif

  PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
  PRINTF("LL header: %u\n", UIP_LLH_LEN);
  PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
  PRINTF("REST max chunk: %u\n", REST_MAX_CHUNK_SIZE);

  /* Initialize the REST engine. */
  rest_init_engine();

  /*
   * Bind the resources to their Uri-Path.
   * WARNING: Activating twice only means alternate path, not two instances!
   * All static variables are the same for each URI path.
   */
  rest_activate_resource(&res_hello, "test/hello");
/*  rest_activate_resource(&res_mirror, "debug/mirror"); */
/*  rest_activate_resource(&res_chunks, "test/chunks"); */
/*  rest_activate_resource(&res_separate, "test/separate"); */
  rest_activate_resource(&res_push, "test/push");
/*  rest_activate_resource(&res_event, "sensors/button"); */
/*  rest_activate_resource(&res_sub, "test/sub"); */
/*  rest_activate_resource(&res_b1_sep_b2, "test/b1sepb2"); */
#if PLATFORM_HAS_LEDS
/*  rest_activate_resource(&res_leds, "actuators/leds"); */
  rest_activate_resource(&res_toggle, "actuators/toggle");
#endif
#if PLATFORM_HAS_LIGHT
  rest_activate_resource(&res_light, "sensors/light"); 
  SENSORS_ACTIVATE(light_sensor);  
#endif
/*
#if PLATFORM_HAS_BATTERY
  rest_activate_resource(&res_battery, "sensors/battery");  
  SENSORS_ACTIVATE(battery_sensor);  
#endif
#if PLATFORM_HAS_RADIO
  rest_activate_resource(&res_radio, "sensors/radio");  
  SENSORS_ACTIVATE(radio_sensor);  
#endif
#if PLATFORM_HAS_SHT11
  rest_activate_resource(&res_sht11, "sensors/sht11");  
  SENSORS_ACTIVATE(sht11_sensor);  
#endif
*/

  /* Define application-specific events here. */
  while(1) {
    PROCESS_WAIT_EVENT();
#if PLATFORM_HAS_BUTTON
    if(ev == sensors_event && data == &button_sensor) {
      PRINTF("*******BUTTON*******\n");

      /* Call the event_handler for this application-specific event. */
      res_event.trigger();

      /* Also call the separate response example handler. */
      res_separate.resume();
    }
#endif /* PLATFORM_HAS_BUTTON */
  }                             /* while (1) */

  PROCESS_END();
}

//###############################################################################
PROCESS_THREAD(test_timer_process, ev, data){
	char msg[100];
  	//sprintf(msg, "yuri");	

	PROCESS_BEGIN();
	static struct etimer et;
  	uip_ipaddr_t addr;

	simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver); //Register a UDP connection
	//simple_udp_register(&broadcast_connection1, UDP_PORT,
                      //NULL, UDP_PORT,
                      //receiver1);
    //Armazena o id do próprio mote  
  int my_id;

    //Vetor para as coordenadas X,Y e Z do próprio mote
  unsigned int my_coordinate[3];

    //Vetor para as coordenadas dos eventos
  unsigned int event[3];

    //Vetor para armazenar a diferença na subtração entre as coordenadas
  unsigned int diff[3];

   //Prioridade
  unsigned int priority;

	while(1) {
		etimer_set(&et, CLOCK_SECOND*SECONDS);
		PROCESS_WAIT_EVENT();
		//count = 0; 		
      //Mote busca o seu próprio id e subtrai 2 de seu valor   
    my_id=node_id-2;
    /*Mote busca sua própria coordenada X,Y e Z dentro da matriz de coordenadas
      no arquivo coordinate.h e armazena elas no vetor*/
    my_coordinate[0]=(unsigned int)(motes_coordinates[my_id][0]*100);
    my_coordinate[1]=(unsigned int)(motes_coordinates[my_id][1]*100);
    my_coordinate[2]=(unsigned int)(motes_coordinates[my_id][2]*100);

      //O mote exibe os valores X,Y e Z de sua coordenada
    //printf("Coordenada X: %u\n",my_coordinate[0]);
    //printf("Coordenada Y: %u\n",my_coordinate[1]);
    //printf("Coordenada Z: %u\n",my_coordinate[2]);

      //Se valor do contador de eventos for menor que total de eventos
    if(event_count<total_events){
         /*Mote busca a coordenada X,Y e Z dentro da matriz de eventos
          no arquivo events.h e armazena elas no vetor*/
      event[0]=(unsigned int)(events_coordinates[event_count][0]*100);
      event[1]=(unsigned int)(events_coordinates[event_count][1]*100);
      event[2]=(unsigned int)(events_coordinates[event_count][2]*100);

        //Mote exibe os valores X,Y e Z do evento
      //printf("Coordenada X do evento: %u\n",event[0]);
      //printf("Coordenada Y do evento: %u\n",event[1]);
      //printf("Coordenada Z do evento: %u\n",event[2]);

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
      //printf("Teste: %d\n", priority);

        //Se a distancia calculada for menor igual ao range, o mote exibe aviso
      if((distance/100)<=RANGE){
	is_event=1;	
	printf("Detectou evento\n");
	printf("Enviando broadcast\n");	
	sprintf(msg, "Mote: %d aconteceu com nivel %d", sender_id, priority);
	uip_create_linklocal_allnodes_mcast(&addr); //Set IP address addr to the link local all-nodes multicast address
    	simple_udp_sendto(&broadcast_connection, msg, strlen(msg), &addr); //Send a UDP packet to a specified IP address.
		
	//Ativa o flag avisando sobre evento
        

        char str[100];

          //Informação sobre o evento detectado
        snprintf(str,100,"\nMote %d:Evento a %um de distancia\n",node_id, distance/100);
        //printf("String: %s\n",str);

          //Copia a informação do evento para array em res-hello.h
        memcpy(info_event,str,sizeof(str));
      }

    }
    //printf("event_count do process %d\n", event_count);

      //Acrescenta 1 para o próximo evento
    //count = 0;
    event_count++;  
    //count = 0;

      //Se o tempo estimado expirar, reinicia a contagem
    if(etimer_expired(&et)) {
	    //count = 0;
	    etimer_reset(&et);
	    count = 0;
	  }
    
    if(count != 0){
	count = 0;
    }


	}
PROCESS_END();
}
//###############################################################################

