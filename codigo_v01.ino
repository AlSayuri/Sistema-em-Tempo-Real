/*Biblioteca*/
   #include <NilRTOS.h>
  // Use tiny unbuffered NilRTOS NilSerial library.
  #include <NilSerial.h>
  // Macro to redefine Serial as NilSerial to save RAM.
  // Remove definition to use standard Arduino Serial.
  #define Serial NilSerial
  #include <NilTimer1.h>
/*Fim Biblioteca*/

/*Semáforos -- Declara o semáforo com o valor inicial de contagem com o valor zero*/
  SEMAPHORE_DECL(sem_1, 0);
/*Fim Semáforos*/

/*------------------------------------------------------------------------------
/*Declaração das variáveis globais*/
    /*Horas*/
      uint8_t s = 0; /*segundos;   - 1 byte*/
      uint8_t m = 0; /*minutos;    - 1 byte*/
      uint8_t h = 0; /*horas;      - 1 byte*/
    /*Fim Horas*/
/*Fim Declaração das variáveis globais*/

/*----------------------------------------------------------------------------------------------------*/
/*Tarefas*/
  /* Primeira Tarefa   - Horas*/
    // Declare a stack with 64 bytes beyond context switch and interrupt needs.
    NIL_WORKING_AREA(waThread1, 64);

    // Declare thread function for thread 1.
    NIL_THREAD(Thread1, arg) {
      //Serial.println("Thread1");
     nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
      while (TRUE) {
        Serial.println("Thread1");
        // Wait for signal from thread 2.
       func_horas();
       nilSemWait(&sem_1);
       Serial.print("Horas: ");
       Serial.print(h);
       Serial.print(":");
       Serial.print(m);
       Serial.print(":");
       Serial.println(s);
       Serial.print("Segindos: ");
       Serial.println(s);
       s++; 
       nilTimer1Wait();
      }
    }
  /* Fim Primeira Tarefa*/
  /*--------------------------------------------------------*/
  /* Segunda Tarefa*/
    // Declare a stack with 64 bytes beyond context switch and interrupt needs.
    NIL_WORKING_AREA(waThread2, 64);

    // Declare thread function for thread 1.
    NIL_THREAD(Thread2, arg) {
      nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
      while (TRUE) {
       nilTimer1Wait();
       Serial.println("Thread2 - primeiro");
       nilSemSignal(&sem_1); // volta para o primeiro
       func_horas();
       s++;
      }
    }
  /* Fim Segunda Tarefa*/
  
/*Fim Tarefas*/
/*------------------------------------------------------------------------------*/
/*Prioridades de Tarefas*/
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
/*Fim Prioridades de Tarefas*/
/*------------------------------------------------------------------------------*/
void setup() {
  /*Roda uma vez*/
  Serial.begin(9600);/*para o println*/
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
      m = m + 1;
    }    
    if(m == 60){
      m = 0;
      h = h + 1;
    }    
    if(h == 24){
      s = 0;
      m = 0;
      h = 0;
    } 
  }
/*Fim Função de horas*/
