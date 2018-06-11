/*Biblioteca*/
    #include <NilRTOS.h>
    #include <NilSerial.h>
    #define Serial NilSerial
    #include <NilTimer1.h>
/*Fim Biblioteca*/

/*Semáforos -- Declara o semáforo com o valor inicial de contagem com o valor zero*/
    SEMAPHORE_DECL(sem_geral, 0); /*Semáforo geral*/
/*Fim Semáforos*/

/*------------------------------------------------------------------------------
/*Declaração das variáveis globais*/
    /*Luz*/
        const uint8_t rele_luz = 8; /*pino do rele da luz*/
    /*Fim Luz*/
    /*Irrigação*/
        const uint8_t rele_bomba = 7; /*pino do rele da bomba de água*/
        const uint8_t sensor_umidade = A5; /*pino do rele da bomba de água*/
        uint16_t v_umidade_solo; /*2 byte*/
    /*Fim Irrigação*/
    /*Variáveis de Controle*/
        uint8_t estado_luz = 0; /*0 = luz desligado / 1 = luz ligado */
        uint8_t controle = 0; /*saber se é uma nova execução*/
    /*Fim Variáveis de Controle*/
/*Fim Declaração das variáveis globais*/

/*Tarefas*/
    /*--------------------------------------------------------*/
    /* Primeira Tarefa   - Irrigação*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread1, 64);

        // Declaração da função de thread para a thread 2
        NIL_THREAD(Thread1, arg) {
            //nilTimer1Start(3000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
            while (TRUE) {
                Serial.println("Thread1 - Irrigação");
                /*Umidade do solo*/
                    //v_umidade_solo = analogRead(sensor_umidade); //Recebe o valor do sensdor de umidade de solo
                    v_umidade_solo = 100;
                    /*nível de umidade do solo*/
                        //Solo umido
                        if (v_umidade_solo > 0 && v_umidade_solo < 400){
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            //DELAY por 5000 milisegundos = 5 segundos.   
                            //nilThdDelayMilliseconds(5000);
                            nilTimer1Start(5000000);
                            nilTimer1Wait();
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            nilTimer1Start(5000000);
                            //DELAY por 3595000 milisegundos = 3595 segundos. 
                            // para que o if delay por uma hora  
                            //nilThdDelayMilliseconds(3595000);
                            //nilThdDelayMilliseconds(10000);
                            nilTimer1Wait();
                        }
                        //Solo com umidade moderada
                        else if (v_umidade_solo > 400 && v_umidade_solo < 800){
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            //DELAY por 8000 milisegundos = 8 segundos.   
                            //nilThdDelayMilliseconds(8000);
                            nilThdSleepMilliseconds(500);
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            //DELAY por 3592000 milisegundos = 3592 segundos. 
                            // para que o if DELAY por uma hora    
                            //nilThdDelayMilliseconds(3592000);
                            nilThdSleepMilliseconds(500);
                        }
                        //Solo seco
                        else if (v_umidade_solo > 800 && v_umidade_solo < 1024){
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            //Sleep por 10000 milisegundos = 10 segundos.   
                            //nilThdDelayMilliseconds(10000);
                            nilThdSleepMilliseconds(500);
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            //DELAY por 3590000 milisegundos = 3590 segundos. 
                            // para que o if DELAY por uma hora 
                            nilThdSleepMilliseconds(500);
                            //nilThdDelayMilliseconds(3590000);
                        }
                /*Fim Umidade do solo*/
                if (controle == 0){ /*´primeira vez que passar em cada período*/
                  nilSemWait(&sem_geral); /*vai para a tarefa de Luz*/
                }
                else{ /*segunda vez que passar em cada período pq não vai pra tarefa de luz*/
                   /*DELAY por 18000000 milisegundos = 5 horas. */
                   controle = 0;
                   //nilThdDelayMilliseconds(18000000);
                   nilTimer1Start(10000000);
                   nilTimer1Wait();
                   //nilThdDelayMilliseconds(10000);
                }
            }
        }
  /* Fim primeira Tarefa*/
  /*--------------------------------------------------------*/
  /* Segunda Tarefa   - Luz*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread2, 64);
        
        // Declaração da função de thread para a thread 1
        NIL_THREAD(Thread2, arg) {
            Serial.println("Thread2 - luz");
            //nilTimer1Start(10000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
            nilTimer1Start(10000000);
            while (TRUE) {
                Serial.println("Thread2 - luz");
                /*if para ligar ou desligar a luz*/
                if (estado_luz == 0){ /*se aluz estiver desligado*/
                    digitalWrite(rele_luz, LOW); /*liga luz*/
                    Serial.println("===> luz ligado");
                    estado_luz = 1;
                }
                else{
                    digitalWrite(rele_luz, HIGH); /*desliga luz*/
                    Serial.println("===> luz desligado");
                    estado_luz = 0;
                }
                /*fim if para ligar ou desligar a luz*/
                /*Sleep por 3600000 milisegundos = 1 hora. */
                //nilThdDelayMilliseconds(3600000);
                //nilThdDelayMilliseconds(10000);
                nilTimer1Wait();
                controle = 1;
                nilSemSignal(&sem_geral); // volta para a primeira thread (irrigação)      
            }
        }
    /* Fim Segunda Tarefa*/

/*Fim Tarefas*/

/*------------------------------------------------------------------------------*/
/*Prioridades de Tarefas*/
  /*
  * Tabela estática de Threads, uma entrada por Thread.  Uma prioridade de Thread é
  * determinado por sua posição na tabela com maior prioridade primeiro.
  *
  * Essas Threads começam com argumento nulo. Um nome de Thread é também
  * nulo para salvar a RAM já que o nome não é usado.
  */
  NIL_THREADS_TABLE_BEGIN()
  NIL_THREADS_TABLE_ENTRY(NULL, Thread1, NULL, waThread1, sizeof(waThread1))
  NIL_THREADS_TABLE_ENTRY(NULL, Thread2, NULL, waThread2, sizeof(waThread2))
  NIL_THREADS_TABLE_END()
/*Fim Prioridades de Tarefas*/
/*------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------------*/

void setup() {
  /*Roda uma vez*/
  Serial.begin(9600);/*para o println*/
  // start kernel
  nilSysBegin();

  pinMode(sensor_umidade, INPUT);
  pinMode(rele_luz, OUTPUT);
  pinMode(rele_bomba, OUTPUT);
  digitalWrite(rele_luz, HIGH);
  digitalWrite(rele_bomba, HIGH);
}
//------------------------------------------------------------------------------
// Loop é o encadeamento inativo.  O encadeamento inativo não deve invocar nenhum
// kernel primitivo capaz de mudar seu estado para não executável.
void loop() {
  // Não é usado
}

