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
        const uint8_t estado_luz = 0; /*0 = luz desligado / 1 = luz ligado */
        const uint8_t controle = 0; /*saber se é uma nova execução*/
    /*Fim Variáveis de Controle*/
/*Fim Declaração das variáveis globais*/

/*Tarefas*/
    /*--------------------------------------------------------*/
    /* Primeira Tarefa   - Irrigação*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread1, 64);

        // Declaração da função de thread para a thread 1
        NIL_THREAD(Thread1, arg) {
                Serial.println("Thread1 - Irrigação");
                /*Umidade do solo*/
                    //v_umidade_solo = analogRead(sensor_umidade);/*recebe o valor do sensdor de umidade de solo*/
                    v_umidade_solo = 100;
                    /*nível de umidade do solo*/
                        //Solo umido
                        if (v_umidade_solo > 0 && v_umidade_solo < 400){
                            Serial.println("Solo umido");
                            digitalWrite(rele_bomba, HIGH);
                            Serial.println("===> bomba ligado");
                            //Sleep por 5000 milisegundos = 5 segundos.   
                            nilThdSleepMilliseconds(5000);
                            Serial.println("===> bomba desligado");
                            digitalWrite(rele_bomba, LOW);
                            //Sleep por 3595000 milisegundos = 3595 segundos. 
                            // para que o if durma por uma hora  
                            nilThdSleepMilliseconds(3595000);
                            
                        }
                        //Solo com umidade moderada
                        else if (v_umidade_solo > 400 && v_umidade_solo < 800){
                            Serial.println("Solo moderado");
                            digitalWrite(rele_bomba, HIGH);
                            Serial.println("bomba ligado");
                            //Sleep por 8000 milisegundos = 8 segundos.   
                            nilThdSleepMilliseconds(8000);
                            Serial.println("bomba desligado");
                            digitalWrite(rele_bomba, LOW);
                            //Sleep por 3592000 milisegundos = 3592 segundos. 
                            // para que o if durma por uma hora    
                            nilThdSleepMilliseconds(3592000);
                        }
                        //Solo seco
                        else if (v_umidade_solo > 800 && v_umidade_solo < 1024){
                            Serial.println("Solo seco");
                            digitalWrite(rele_bomba, HIGH);
                            Serial.println("bomba ligado");
                            //Sleep por 10000 milisegundos = 10 segundos.   
                            nilThdSleepMilliseconds(10000);
                            digitalWrite(rele_bomba, LOW);
                            Serial.println("bomba desligado");
                            //Sleep por 3590000 milisegundos = 3590 segundos. 
                            // para que o if durma por uma hora    
                            nilThdSleepMilliseconds(3590000);
                        }
                /*Fim Umidade do solo*/

                if (controle == 0){ /*´primeira vez que passar em cada período*/
                  nilSemWait(&sem_geral); /*vai para a tarefa de Luz*/
                }
                else{ /*segunda ve que passar em cada período*/
                   /*Sleep por 18000000 milisegundos = 5 horas. */
                   controle = 0;
                   nilThdSleepMilliseconds(18000000);
                }
        }
    /* Fim Primeira Tarefa*/
    /*--------------------------------------------------------*/
    /* Segunda Tarefa   - Luz*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread2, 64);
    
        // Declaração da função de thread para a thread 2
        NIL_THREAD(Thread2, arg) {
            Serial.println("Thread2 - Luz");
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
             nilThdSleepMilliseconds(3600000);
             controle = 1;
             nilSemSignal(&sem_geral); // volta para a primeira thread (irrigação)
        }
    /* Fim Segunda Tarefa*/
    /*------------------------------------------------------------------------------*/
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
}
//------------------------------------------------------------------------------
// Loop é o encadeamento inativo.  O encadeamento inativo não deve invocar nenhum
// kernel primitivo capaz de mudar seu estado para não executável.
void loop() {
  // Não é usado
}

