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
#include "er-coap-observe.h"


//Arquivo com as coordenadas dos motes
//#include "coordinates.h"
//Arquivo com as coordenadas dos eventos
//#include "events.h"

//Arquivo com os arrays de coordenadas
#include "load-coordinate.h"
//Arquivo para calcular a distancia euclidiana
#include "calculate-distance.h"

//Arquivo com flag sobre evento e a informação sobre ele
#include "resources/res-hello.h"
//Arquivo com as informações de classificação / nível de eventos
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
//Total de motes na simulação
int countmotes = 0;

//Contagem de eventos
unsigned int event_count=0; 

//Estabelecer struct broadcast para registro
static struct simple_udp_connection broadcast_connection;

//Para função clean
static struct ctimer timer;

//Para OBS
static struct etimer add_obs_timer;

//Variáveis para vetor que armazena classificação / nível do evento
static int count = 0;

//Vetor para função clean
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
	//Endereço para broadcast de confirmação	
	//uip_ipaddr_t addr_receiver;			
	
	//Mensagem recebida 
	printf("Dados recebidos do end.: ");
  	uip_debug_ipaddr_print(sender_addr);
	//PRINT6ADDR(&sender_addr);
	printf(" na porta %d oriundos da porta %d com tam.: %d: '%s'\n",
        receiver_port, sender_port, datalen, data);
	
	//Transforma o último caracter da mensagem em inteiro
	//int type = (int) data[datalen-1] - 48;
	
	
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

	//Espaço dedicado a confirmação de que recebeu o broadcast
	/*if (((type == 1) || (type == 2))){			
		uip_create_linklocal_allnodes_mcast(&addr_receiver);
		simple_udp_sendto(&broadcast_connection, "oi", 3 , &addr_receiver);
	}*/
	

}
// 1 ocorreu evento crítico (nivel 1)
// 2 ocorreu evento não crítico (nivel 2)

static void print_vector(int n){
	int ct_print_vt = 0;
	for (ct_print_vt = 0; ct_print_vt < n; ct_print_vt++){
		printf("%d ", vt[ct_print_vt]);
	}
	printf("\n");
}

static int sum_vector(int n){
	int ct_sm_vt, sm_vt = 0;
	for (ct_sm_vt = 0; ct_sm_vt < n; ct_sm_vt++){
		sm_vt += vt[ct_sm_vt]; 
	}
	return sm_vt;
}

static void k_out_of_n(int k, int sum){
	//k: limite que determina a ocorrência de evento
	//sum: soma dos indices de valor 1 	
	if (sum >= k){
		printf("Verdadeiro Positivo\n");
	}else{
		printf("Falso Positivo\n");	
	}
}


//Função para limpar o contador e o vetor da função receiver
static void clean(void *ptr){
            int k;
	    if (countmotes < 10){
		print_vector(countmotes);
	    }else{
		print_vector(10);
	    }
   
	    int sum = 0;
	    sum = sum_vector(countmotes);
	    printf("Soma %d\n", sum);
	    k_out_of_n(1 ,sum);
	    
	    count = 0;
	    for (k = 0; k < 10; k++){
	   	vt[k] = 0;
	    }
}
	

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
  rest_activate_resource(&res_hello, "res-hello");
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

//cod pra add observador
	etimer_set(&add_obs_timer, CLOCK_SECOND*50); 
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&add_obs_timer));
	static uint16_t addr_test[8];
	uip_ipaddr_t dest_addr;
	

	//aaaa::c30c:0:0:1
	addr_test[0] = 0xaaaa;
	addr_test[4] = 0x0000;
	addr_test[5] = 0x0000;
	addr_test[6] = 0x0000;
	addr_test[7] = 0x0001;
	//addr_test[8] = 0x0002;

	uip_ip6addr(&dest_addr, addr_test[0], addr_test[1], addr_test[2],
	addr_test[3], addr_test[4],addr_test[5],addr_test[6],addr_test[7]);

	add_observer(&dest_addr, 0, 0, 0, "res-hello", 9);
	printf("Endereco destino OBS\n");
	uip_debug_ipaddr_print(&dest_addr);
	printf("\n");

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
	//Mensagem a ser enviada pelo broadcast
	char msg[100];

	PROCESS_BEGIN();
	static struct etimer et;
	//Endereço a ser usado pelo broadcast
  	uip_ipaddr_t addr;

	//Registrar broadcast
	simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver); 

    	
	while(1) {
	    etimer_set(&et, CLOCK_SECOND*SECONDS);
	    PROCESS_WAIT_EVENT();
	    
            load_id();  
	    load_coordinate_mote();
            show_coordinate_mote();
	    load_coordinate_event(event_count);
	    show_coordinate_event();
            calculate_difference();
	    int distance = euclidian_distance();
	    print_distance(distance);
		
            //Definir o total de motes
	    countmotes = return_count_motes();	
	    printf("valor countmotes: %d\n", countmotes);

            int sender_id = node_id;
	    //Níveis de classificação / prioridade
    	    int priority;
      
            //Estabelecendo prioridade para cada evento
            priority = priority_events[event_count];

            //Se a distancia calculada for menor igual ao range, o mote vai enviar broadcast
            if(distance<=RANGE){	
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
    	
    	        //Ativa o flag avisando sobre evento
		is_event=1;            
    
                char str[100];
    
                //Informação sobre o evento detectado
                snprintf(str,100,"\nMote %d:Evento a %um de distancia\n",node_id, distance/100);
    
                //Copia a informação do evento para array em res-hello.h
                memcpy(info_event,str,sizeof(str));
            }

        //Acrescenta 1 para o próximo evento
        event_count++;  
        
	//Para a função clean ser chamada a cada 30 segundos	
	ctimer_set(&timer, CLOCK_SECOND*30, clean, NULL);        

	//Se o tempo estimado expirar, reinicia a contagem
	if(etimer_expired(&et)) {
	    etimer_reset(&et);
        }
        
	} //fim do while
PROCESS_END();
}

//###############################################################################

