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
            while (TRUE) {
                Serial.println("Thread1 - Irrigação");
                /*Umidade do solo*/
                    v_umidade_solo = analogRead(sensor_umidade); //Recebe o valor do sensor de umidade de solo
                    /*nível de umidade do solo*/
                        //Solo umido
                        if (v_umidade_solo > 0 && v_umidade_solo < 400){
                            Serial.println("====> Bomba Ligada");
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            Serial.println("Espera 3 segundos");
                            //DELAY por 3000000 microsegundos = 3 segundos.
                            esperar(3);
                            Serial.println("====> Bomba Desligada");
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            Serial.println("Espera 3597 segundos para dar 1 hora");
                            esperar(3597);
                        }
                        //Solo com umidade moderada
                        else if (v_umidade_solo > 400 && v_umidade_solo < 800){
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            Serial.println("Espera 5 segundos");
                            //DELAY por 5000000 microsegundos = 5 segundos.
                            esperar(5);
                            Serial.println("====> Bomba Desligada");
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            Serial.println("Espera 3595 segundos para dar 1 hora");
                            esperar(3595);
                        }
                        //Solo seco
                        else if (v_umidade_solo > 800 && v_umidade_solo < 1024){
                            digitalWrite(rele_bomba, LOW); // Liga bomba de água
                            //DELAY por 8000000 microsegundos = 8 segundos. 
                            esperar(8);  
                            digitalWrite(rele_bomba, HIGH); // Desliga bomba de água
                            //DELAY por 3592000 milisegundos = 3592 segundos. 
                            esperar(3592); // para dar uma hora
                        }
                /*Fim Umidade do solo*/
                if (controle == 0){ /*´primeira vez que passar em cada período*/
                  nilSemWait(&sem_geral); /*vai para a tarefa de Luz*/
                  esperar(3600);
                }
                else{ /*segunda vez que passar em cada período pq não vai pra tarefa de luz*/
                   /*DELAY por 18000000 microsegundos = 5 horas. */
                   Serial.println("==> Não vai para thread 02");
                   Serial.println("Espera por 5 segundos");
                   //DELAY por 5000000 microsegundos = 5 segundos.
                   esperar(5);
                   controle = 0;
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
                /*Delay por 3600000000 milisegundos = 3600 segundos = 1 hora*/
                esperar(3600);
                controle = 1;
                nilSemSignal(&sem_geral); // volta para a primeira thread (irrigação)      
            }
        }
    /* Fim Segunda Tarefa*/
  /*--------------------------------------------------------*/
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
// Loop is the idle thread.  The idle thread must not invoke any
// kernel primitive able to change its state to not runnable.
void loop() {
  // Not used
}

void esperar(int segundos){ // Microsegundos
  // irrigação - Solo umido
    if (segundos == 3){ // 3 segundos
      nilTimer1Start(3000000); 
    }
    else if (segundos == 3597){ // 3597 segundos
      nilTimer1Start(3597000000);
    }
  // fim irrigação - Solo umido
  // irrigação -  umidade moderada
    if (segundos == 5){ // 5 segundos
      nilTimer1Start(5000000); 
    }
    else if (segundos == 3595){ // 3595 segundos
      nilTimer1Start(3595000000);
    }
  // fim - irrigação -  umidade moderada
  // irrigação - solo seco
    else if (segundos == 8){ // 8 segundos
      nilTimer1Start(8000000);
    }
    else if (segundos == 3592){ // 3592 segundos
      nilTimer1Start(3592000000);
    }
  // fim irrigação - solo seco
  // luz
    else if (segundos == 3600){
      nilTimer1Start(3600000000);
    }
  // fim luz
  
  nilTimer1Wait();
}


