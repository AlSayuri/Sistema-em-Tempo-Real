/*Biblioteca*/
    #include <NilRTOS.h>
    #include <NilSerial.h>
    #define Serial NilSerial
    #include <NilTimer1.h>
/*Fim Biblioteca*/

/*Semáforos -- Declara o semáforo com o valor inicial de contagem com o valor zero*/
    SEMAPHORE_DECL(sem_geral, 0); /*Semáforo geral*/
    SEMAPHORE_DECL(sem_cool, 0); /*Semáforo para o cooler*/
/*Fim Semáforos*/

/*------------------------------------------------------------------------------
/*Declaração das variáveis globais*/
    /*Horas*/
        uint8_t h = 0;  /*horas;      - 1 byte*/
        uint8_t m = 0;  /*minutos;    - 1 byte*/
        uint8_t s = 0;  /*segundos;   - 1 byte*/
    /*Fim Horas*/
    /*Luz*/
        const uint8_t rele_luz = 8; /*pino do rele da luz*/
    /*Fim Luz*/
    /*Irrigação*/
        const uint8_t rele_bomba = 7; /*pino do rele da bomba de água*/
        const uint8_t sensor_umidade = A5; /*pino do rele da bomba de água*/
        uint16_t v_umidade_solo; /*2 byte*/
    /*Fim Irrigação*/
    /*Cooler*/
        const uint8_t rele_cooler = 4; /*pino do rele do cooler*/
    /*Fim Cooler*/
/*Fim Declaração das variáveis globais*/

/*Tarefas*/
    /*--------------------------------------------------------*/
    /* Primeira Tarefa   - Luz*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread1, 64);
    
        // Declaração da função de thread para a thread 1
        NIL_THREAD(Thread1, arg) {
            Serial.println("Thread1");
            nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
            
            while (TRUE) {
                nilTimer1Wait();
                s++;
                func_horas();
                
                Serial.println("Thread1 - Luz = principal");
              
                /*if para ligar ou desligar a luz*/
                    if (h == 7 && m == 00 && s == 1){
                        digitalWrite(rele_luz, LOW); // Liga a luz
                    }
                    else if(h == 17 && m == 00 && s == 1){
                        digitalWrite(rele_luz, HIGH); // Desliga a luz
                    }
                /*Fim if para ligar ou desligar a luz*/
                
                /*if para Irrigação ou cooler*/
                      if (( h == 7  && m == 30 && s == 1 ) || // Para a irrigação - 07:30:01
                          ( h == 13 && m == 30 && s == 1 ) || // Para a irrigação - 12:30:01
                          ( h == 19 && m == 30 && s == 1 ) || // Para a irrigação - 19:30:01
                          ( h == 10 && m == 15 && s == 1 ) || // Para o cooler    - 10:15:01
                          ( h == 14 && m == 15 && s == 1 ) || // Para o cooler    - 14:15:01
                          ( h == 20 && m == 15 && s == 1 )    // Para o cooler    - 20:15:01
                          ){
                              nilSemWait(&sem_geral); // Vai para a segunda Thread (irrigação)
                      }
                /*Fim if para Irrigação*/
                
            }
        }
    /* Fim Primeira Tarefa*/
    /*--------------------------------------------------------*/
    /* Segunda Tarefa   - Irrigação*/
        // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
        NIL_WORKING_AREA(waThread2, 64);

        // Declaração da função de thread para a thread 2
        NIL_THREAD(Thread2, arg) {
            nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
            
            while (TRUE) {
                nilTimer1Wait();
                s++;
                func_horas();
                Serial.println("Thread2 - Irrigação");
                                
                /*if para o Cooler*/
                    if (( h == 10 && m == 15 && s == 1 ) || // 10:15:01
                        ( h == 14 && m == 15 && s == 1 ) || // 10:15:01
                        ( h == 20 && m == 15 && s == 1 )    // 20:15:01
                        ){
                            nilSemWait(&sem_cool); // Vai para a Thread 3 (cooler)
                    }
                /*Fim if para o Cooler*/ 

                /*Umidade do solo*/
                    v_umidade_solo = analogRead(sensor_umidade); //Recebe o valor do sensdor de umidade de solo
                    /*nível de umidade do solo*/
                        //Solo umido
                        if (v_umidade_solo > 0 && v_umidade_solo < 400){
                            digitalWrite(rele_bomba, HIGH); // Liga bomba de água
                            if(s == 10){
                                digitalWrite(rele_bomba, LOW); // Desliga bomba de água
                                nilSemSignal(&sem_geral); // volta para a primeira thread
                            }
                        }
                        //Solo com umidade moderada
                        else if (v_umidade_solo > 400 && v_umidade_solo < 800){
                            digitalWrite(rele_bomba, HIGH); // Liga bomba de água
                            if(s == 14){
                              digitalWrite(rele_bomba, LOW); // Desliga bomba de água
                              nilSemSignal(&sem_geral); // volta para a primeira thread
                            }
                        }
                        //Solo seco
                        else if (v_umidade_solo > 800 && v_umidade_solo < 1024){
                            digitalWrite(rele_bomba, HIGH); // Liga bomba de água
                            if(s == 17){
                              digitalWrite(rele_bomba, LOW); // Desliga bomba de água
                              nilSemSignal(&sem_geral); // volta para a primeira thread
                            }
                        }
                /*Fim Umidade do solo*/
            }
        }
  /* Fim Segunda Tarefa*/
  /*--------------------------------------------------------*/
  /* Terceira Tarefa   - Cooler*/
      // Declaração de pilha com 64 bytes além do necessário para o contexo de troca e interrupção
      NIL_WORKING_AREA(waThread3, 64);

      // Declaração da função de thread para a thread 3
      NIL_THREAD(Thread3, arg) {
          nilTimer1Start(1000000); /*tempo de de execução Ex. 1000000 = 1 segundo*/
      
          while (TRUE) {
              nilTimer1Wait();
              s++;
              func_horas();
              Serial.println("Thread3 - Cooler");
              digitalWrite(rele_cooler, LOW); // Liga cooler
              if(s == 50){
                  digitalWrite(rele_cooler, HIGH); // Desliga cooler
                  nilSemSignal(&sem_geral); // volta para a primeira thread
              } 
          }
      }
   /* Fim Terceira Tarefa*/  

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
  NIL_THREADS_TABLE_ENTRY(NULL, Thread3, NULL, waThread3, sizeof(waThread3))
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
  pinMode(rele_cooler, OUTPUT);
  pinMode(rele_luz, OUTPUT);
  pinMode(rele_bomba, OUTPUT);
  digitalWrite(rele_cooler, HIGH);
  digitalWrite(rele_luz, HIGH);

}
//------------------------------------------------------------------------------
// Loop é o encadeamento inativo.  O encadeamento inativo não deve invocar nenhum
// kernel primitivo capaz de mudar seu estado para não executável.
void loop() {
  // Não é usado
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
    if(h == 24 && m == 60){
      s = 0;
      m = 0;
      h = 0;
    } 
    Serial.print("Horas: ");
    Serial.print(h);
    Serial.print(":");
    Serial.print(m);
    Serial.print(":");
    Serial.println(s);
  }
/*Fim Função de horas*/
