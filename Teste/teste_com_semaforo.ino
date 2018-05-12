// This example tests nilTimer1.

#include <NilRTOS.h>

// Use tiny unbuffered NilRTOS NilSerial library.
#include <NilSerial.h>

// Macro to redefine Serial as NilSerial to save RAM.
// Remove definition to use standard Arduino Serial.
#define Serial NilSerial
#include <NilTimer1.h>

/*Aline*/
// Declara o semáforo com o valor inicial de contagem com o valor zero
SEMAPHORE_DECL(sem_1, 0);
/*------------------------------------------------------------------------------
/*Declaração das variáveis globais*/
    uint8_t s = 0; /*segundos;   - 1 byte*/
    uint8_t m = 0; /*minutos;    - 1 byte*/
    uint8_t h = 0; /*horas;      - 1 byte*/
/*Fim Declaração das variáveis globais*/
/*----------------------------------------------------------------------------------------------------*/
/* Primeira Tarefa*/
  // Declare a stack with 64 bytes beyond context switch and interrupt needs.
  NIL_WORKING_AREA(waThread1, 64);

  // Declare thread function for thread 1.
  NIL_THREAD(Thread1, arg) {
    //Serial.println("Thread1");
    nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
    while (TRUE) {
      nilTimer1Wait();
      Serial.println("Thread1");
      // Wait for signal from thread 2.
      nilSemWait(&sem_1);
    }
  }
/* Fim Primeira Tarefa*/
/*------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/* Segunda Tarefa*/
  // Declare a stack with 64 bytes beyond context switch and interrupt needs.
  NIL_WORKING_AREA(waThread2, 64);

  // Declare thread function for thread 1.
  NIL_THREAD(Thread2, arg) {
    nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
    while (TRUE) {
      nilTimer1Wait();
      Serial.println("Thread2 - primeiro");
      nilThdSleepMilliseconds(2000);
      Serial.println("Thread2 - segundo");
      // Signal thread 1 to turn LED off.
      nilSemSignal(&sem_1);
      Serial.println("Thread2 - terceiro");
       // Sleep for 200 milliseconds.
      nilThdSleepMilliseconds(2000);
    }
  }
/* Fim Segunda Tarefa*/
/*------------------------------------------------------------------------------*/
/*
 * Threads static table, one entry per thread.  A thread's priority is
 * determined by its position in the table with highest priority first.
 *
 * These threads start with a null argument.  A thread's name is also
 * null to save RAM since the name is currently not used.
 */
NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY(NULL, Thread1, NULL, waThread1, sizeof(waThread1))
NIL_THREADS_TABLE_ENTRY(NULL, Thread2, NULL, waThread2, sizeof(waThread2))
NIL_THREADS_TABLE_END()
//------------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);
  // start kernel
  nilSysBegin();
}
//------------------------------------------------------------------------------
// Loop is the idle thread.  The idle thread must not invoke any
// kernel primitive able to change its state to not runnable.
void loop() {
  // Not used
}


/*Função de horas*/
  void func_horas () {
    if(s == 60){
      s = 0;
      m = m++;
    }
    
    if(m == 60){
      m = 0;
      h = h++;
    }
    
    if(h == 24){
      h = 0;
    } 
  }
/*Fim Função de horas*/
