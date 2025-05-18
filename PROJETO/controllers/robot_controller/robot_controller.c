#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define PASSO_TEMPO 32
#define QUANTIDADE_SENSORES_PROXIMIDADE 8
#define QUANTIDADE_LEDS 10
#define TAMANHO_MENSAGEM 256
#define VELOCIDADE_MAXIMA 6.28
#define LIMIAR_DISTANCIA 0.1
#define LIMIAR_OBSTACULO 80
#define TEMPO_GIRO_90 500
#define TEMPO_GIRO_45 250
#define TEMPO_RECUO 200
#define MAXIMO_GIROS_CONSECUTIVOS 5

typedef enum {
  MOVER_FRENTE,
  RECUAR,
  GIRAR_DIREITA_90,
  GIRAR_ESQUERDA_90,
  GIRAR_DIREITA_45,
  GIRAR_ESQUERDA_45,
  GIRAR_NO_LUGAR
} Estado;

int main(int argc, char **argv) {
  int i = 0;
  char Mensagem[TAMANHO_MENSAGEM] = {0};
  char temp[128] = {0};
  double LeiturasSensoresProximidade[QUANTIDADE_SENSORES_PROXIMIDADE];
  double AceleradorEsquerda = 1.0, AceleradorDireita = 1.0;
  Estado estado = MOVER_FRENTE;
  int passosGiro = 0;
  int caixaEncontrada = 0;
  int girosConsecutivos = 0;
  
  srand(time(NULL));
  wb_robot_init();
  
  printf("Iniciando o controlador do robô...\n");
  
  // Verificar se a caixa específica existe
  WbNodeRef caixaLeve = wb_supervisor_node_get_from_def("CAIXA11");
  if (caixaLeve == NULL) {
    printf("Erro: CAIXA11 não encontrada! Tentando encontrar outra caixa...\n");
    
    // Tente encontrar qualquer caixa disponível
    for (i = 1; i <= 18; i++) {
      sprintf(temp, "CAIXA%02d", i);
      caixaLeve = wb_supervisor_node_get_from_def(temp);
      if (caixaLeve != NULL) {
        printf("Encontrou caixa alternativa: %s\n", temp);
        break;
      }
    }
    
    // Se ainda não encontrou nenhuma caixa
    if (caixaLeve == NULL) {
      printf("ERRO CRÍTICO: Nenhuma caixa encontrada! Encerrando simulação.\n");
      wb_robot_cleanup();
      return 1;
    }
  } else {
    printf("CAIXA11 encontrada com sucesso!\n");
  }
  
  // Inicializar os motores
  WbDeviceTag MotorEsquerda = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireita = wb_robot_get_device("right wheel motor");
  
  if (MotorEsquerda == 0 || MotorDireita == 0) {
    printf("ERRO: Não foi possível encontrar os motores das rodas!\n");
    wb_robot_cleanup();
    return 1;
  }
  
  printf("Motores das rodas inicializados com sucesso.\n");
  
  wb_motor_set_position(MotorEsquerda, INFINITY);
  wb_motor_set_position(MotorDireita, INFINITY);
  wb_motor_set_velocity(MotorEsquerda, 0);
  wb_motor_set_velocity(MotorDireita, 0);
  
  // Inicializar os sensores de proximidade
  WbDeviceTag SensoresProximidade[QUANTIDADE_SENSORES_PROXIMIDADE];
  char NomeSensor[10] = {0};
  int sensoresEncontrados = 0;
  
  for (i = 0; i < QUANTIDADE_SENSORES_PROXIMIDADE; i++) {
    sprintf(NomeSensor, "ps%d", i);
    SensoresProximidade[i] = wb_robot_get_device(NomeSensor);
    if (SensoresProximidade[i] != 0) {
      wb_distance_sensor_enable(SensoresProximidade[i], PASSO_TEMPO);
      sensoresEncontrados++;
    }
  }
  
  printf("Sensores de proximidade inicializados: %d de %d\n", sensoresEncontrados, QUANTIDADE_SENSORES_PROXIMIDADE);
  
  // Inicializar os LEDs
  WbDeviceTag Leds[QUANTIDADE_LEDS];
  Leds[0] = wb_robot_get_device("led0");
  if (Leds[0] != 0) {
    wb_led_set(Leds[0], -1);
    printf("LED inicializado com sucesso.\n");
  } else {
    printf("AVISO: LED não encontrado.\n");
  }
  
  printf("Inicialização completa. Iniciando o loop principal...\n");
  
  // Definir velocidade inicial para testar movimento
  wb_motor_set_velocity(MotorEsquerda, 2.0);
  wb_motor_set_velocity(MotorDireita, 2.0);
  
  // Aguardar alguns passos para o robô começar a se mover
  for (i = 0; i < 10; i++) {
    wb_robot_step(PASSO_TEMPO);
  }
  
  while (wb_robot_step(PASSO_TEMPO) != -1) {
    memset(Mensagem, 0, TAMANHO_MENSAGEM);
    
    // Adicionar informações dos sensores de proximidade
    strcat(Mensagem, "Sensores: ");
    for (i = 0; i < QUANTIDADE_SENSORES_PROXIMIDADE; i++) {
      if (SensoresProximidade[i] != 0) {
        LeiturasSensoresProximidade[i] = wb_distance_sensor_get_value(SensoresProximidade[i]);
        sprintf(temp, "ps%d: %.0f | ", i, LeiturasSensoresProximidade[i]);
        strcat(Mensagem, temp);
      }
    }
    
    // Adicionar informações da posição da caixa
    if (caixaLeve != NULL) {
      const double *PosicaoCaixa = wb_supervisor_node_get_position(caixaLeve);
      if (PosicaoCaixa != NULL) {
        sprintf(temp, "Caixa em x=%.2f, y=%.2f, z=%.2f | ", 
                PosicaoCaixa[0], PosicaoCaixa[1], PosicaoCaixa[2]);
        strcat(Mensagem, temp);
      } else {
        strcat(Mensagem, "Erro ao obter posição da caixa | ");
      }
    }
    
    // Adicionar informações da posição do robô
    WbNodeRef noRobo = wb_supervisor_node_get_self();
    if (noRobo != NULL) {
      const double *posicaoRobo = wb_supervisor_node_get_position(noRobo);
      if (posicaoRobo != NULL) {
        sprintf(temp, "Robô em x=%.2f, y=%.2f | ", posicaoRobo[0], posicaoRobo[1]);
        strcat(Mensagem, temp);
        
        // Calcular e adicionar a distância
        if (caixaLeve != NULL) {
          const double *PosicaoCaixa = wb_supervisor_node_get_position(caixaLeve);
          if (PosicaoCaixa != NULL) {
            double dx = PosicaoCaixa[0] - posicaoRobo[0];
            double dy = PosicaoCaixa[1] - posicaoRobo[1];
            double distancia = sqrt(dx * dx + dy * dy);
            sprintf(temp, "Distância: %.2f | ", distancia);
            strcat(Mensagem, temp);
            
            // Verificar se encontrou a caixa
            if (!caixaEncontrada && distancia < LIMIAR_DISTANCIA) {
              caixaEncontrada = 1;
              estado = GIRAR_NO_LUGAR;
              printf("Caixa encontrada! Girando no eixo.\n");
            }
          }
        }
      } else {
        strcat(Mensagem, "Erro ao obter posição do robô | ");
      }
    } else {
      strcat(Mensagem, "Erro ao obter nó do robô | ");
    }
    
    // Adicionar informação do estado
    sprintf(temp, "Estado: %d | V_Esq: %.1f | V_Dir: %.1f", 
            estado, VELOCIDADE_MAXIMA * AceleradorEsquerda, VELOCIDADE_MAXIMA * AceleradorDireita);
    strcat(Mensagem, temp);
    
    printf("%s\n", Mensagem);
    
    if (Leds[0] != 0) {
      wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);
    }
    
    switch (estado) {
      case MOVER_FRENTE:
        AceleradorEsquerda = 1.0;
        AceleradorDireita = 1.0;
        
        // Verificar se há obstáculos
        if (SensoresProximidade[0] != 0 && SensoresProximidade[7] != 0 && 
            SensoresProximidade[1] != 0 && SensoresProximidade[6] != 0) {
          if (LeiturasSensoresProximidade[0] > LIMIAR_OBSTACULO || 
              LeiturasSensoresProximidade[7] > LIMIAR_OBSTACULO ||
              LeiturasSensoresProximidade[1] > LIMIAR_OBSTACULO || 
              LeiturasSensoresProximidade[6] > LIMIAR_OBSTACULO) {
            estado = RECUAR;
            passosGiro = TEMPO_RECUO / PASSO_TEMPO;
            girosConsecutivos++;
            printf("Obstáculo detectado! Recuando...\n");
          } else {
            girosConsecutivos = 0;
          }
        }
        break;
        
      case RECUAR:
        AceleradorEsquerda = -1.0;
        AceleradorDireita = -1.0;
        passosGiro--;
        if (passosGiro <= 0) {
          int angulo = (rand() % 2) ? 45 : 90;
          int direcao = (rand() % 2) ? 1 : -1; // 1 para direita, -1 para esquerda
          
          if (angulo == 45 && direcao == 1) {
            estado = GIRAR_DIREITA_45;
            printf("Girando 45° à direita\n");
          }
          else if (angulo == 45 && direcao == -1) {
            estado = GIRAR_ESQUERDA_45;
            printf("Girando 45° à esquerda\n");
          }
          else if (angulo == 90 && direcao == 1) {
            estado = GIRAR_DIREITA_90;
            printf("Girando 90° à direita\n");
          }
          else {
            estado = GIRAR_ESQUERDA_90;
            printf("Girando 90° à esquerda\n");
          }
          
          passosGiro = (angulo == 45 ? TEMPO_GIRO_45 : TEMPO_GIRO_90) / PASSO_TEMPO;
          if (girosConsecutivos >= MAXIMO_GIROS_CONSECUTIVOS) {
            passosGiro *= 2;
            girosConsecutivos = 0;
            printf("Muitos giros consecutivos! Girando por mais tempo...\n");
          }
        }
        break;
        
      case GIRAR_DIREITA_90:
      case GIRAR_DIREITA_45:
        AceleradorEsquerda = 1.0;
        AceleradorDireita = -1.0;
        passosGiro--;
        if (passosGiro <= 0) {
          estado = MOVER_FRENTE;
          printf("Terminando giro. Movendo para frente...\n");
        }
        break;
        
      case GIRAR_ESQUERDA_90:
      case GIRAR_ESQUERDA_45:
        AceleradorEsquerda = -1.0;
        AceleradorDireita = 1.0;
        passosGiro--;
        if (passosGiro <= 0) {
          estado = MOVER_FRENTE;
          printf("Terminando giro. Movendo para frente...\n");
        }
        break;
        
      case GIRAR_NO_LUGAR:
        AceleradorEsquerda = 1.0;
        AceleradorDireita = -1.0;
        printf("Girando no lugar - Caixa encontrada!\n");
        break;
    }
    
    // Aplicar velocidades aos motores
    wb_motor_set_velocity(MotorEsquerda, VELOCIDADE_MAXIMA * AceleradorEsquerda);
    wb_motor_set_velocity(MotorDireita, VELOCIDADE_MAXIMA * AceleradorDireita);
  };
  
  wb_robot_cleanup();
  return 0;
}
