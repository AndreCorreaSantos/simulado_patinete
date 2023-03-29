#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "sensor.h"

/* Botao da placa */
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

#define vel_PIO PIOA
#define vel_PIO_ID ID_PIOA
#define vel_IDX 6
#define vel_IDX_MASK (1u << vel_IDX)

/** RTOS  */
#define TASK_MAIN_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void io_init(void);
QueueHandle_t xQueuePotencia;
QueueHandle_t xQueueDt;

volatile int time;
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
  int valor = 1;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueuePotencia, &valor, &xHigherPriorityTaskWoken);
}

void but2_callback(void) {

}

void but3_callback(void) { 
  int valor = -1;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueuePotencia, &valor, &xHigherPriorityTaskWoken);
}

void leitura_callback(void){
  time = rtt_read_timer_value(RTT);//le o valor do timer
  rtt_init(RTT,1);//resetando valor do timer
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueueDt, &time, &xHigherPriorityTaskWoken);
}



void set_Leds(int potencia){
  //limpa todos os Leds
  pio_set(LED_1_PIO, LED_1_IDX_MASK);
  pio_set(LED_2_PIO, LED_2_IDX_MASK);
  pio_set(LED_3_PIO, LED_3_IDX_MASK);

  if(potencia){//seta primeiro led
    pio_clear(LED_1_PIO, LED_1_IDX_MASK);
    potencia--;
  }
  if(potencia){//cleara segundo led
    pio_clear(LED_2_PIO, LED_2_IDX_MASK);
    potencia--;
  }
  if(potencia){//cleara terceiro led
    pio_clear(LED_3_PIO, LED_3_IDX_MASK);
    potencia--;
  }

}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_main(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Super patinete", 0, 0, &sysfont);
  
  init_sensor();
  io_init();

  int ultimo_valor;
  int potencia = 0;
  float velocidade = 20.0;
  char velocidade_str[10];
  int last_time;
  rtt_init(RTT,1); //inicia timer pela primeira vez;
	for (;;)  {
    if(xQueueReceive(xQueuePotencia,&(ultimo_valor),(TickType_t) 0)){

      if((potencia + ultimo_valor <= 3) && (potencia+ultimo_valor >= 0)){ //incrementar potencia somente se o incremento nao passa do limite
        potencia += ultimo_valor;
      }
      set_Leds(potencia); //setando leds baseado na potencia
      patinete_power(potencia); //chamando funcao patinete power
    }
    if(xQueueReceive(xQueueDt,&last_time,(TickType_t) 0)){
      velocidade = (2*3.14/((float) last_time*0.01))*0.2*3.6*100;
      sprintf(velocidade_str,"%f km/h",velocidade);
      gfx_mono_draw_string(velocidade_str, 20, 20, &sysfont);
    }

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {
  pmc_enable_periph_clk(LED_1_PIO_ID);
  pmc_enable_periph_clk(LED_2_PIO_ID);
  pmc_enable_periph_clk(LED_3_PIO_ID);
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);
  pmc_enable_periph_clk(vel_PIO_ID);

  pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

  pio_configure(vel_PIO, PIO_INPUT, vel_IDX_MASK, PIO_DEFAULT); //configurando pio de leitura

  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
  but1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but3_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but2_callback);

  pio_handler_set(vel_PIO, vel_PIO_ID, vel_IDX_MASK, PIO_IT_RISE_EDGE, //configurando callback de leitura
  leitura_callback);

  pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
  pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
  pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

  pio_enable_interrupt(vel_PIO, vel_IDX_MASK);

  pio_get_interrupt_status(BUT_1_PIO);
  pio_get_interrupt_status(BUT_2_PIO);
  pio_get_interrupt_status(BUT_3_PIO);

  pio_get_interrupt_status(vel_PIO);

  NVIC_EnableIRQ(BUT_1_PIO_ID);
  NVIC_SetPriority(BUT_1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_2_PIO_ID);
  NVIC_SetPriority(BUT_2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_3_PIO_ID);
  NVIC_SetPriority(BUT_3_PIO_ID, 4);

  NVIC_EnableIRQ(vel_PIO_ID);
  NVIC_SetPriority(vel_PIO_ID, 4);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_main, "main", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create main task\r\n");
	}

  xQueuePotencia = xQueueCreate(100, sizeof(int));
	if (xQueuePotencia == NULL)
		printf("falha em criar a queue xQueuePotencia \n");

  xQueueDt = xQueueCreate(100, sizeof(int));
	if (xQueueDt == NULL)
		printf("falha em criar a queue xQueueDt \n");

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
