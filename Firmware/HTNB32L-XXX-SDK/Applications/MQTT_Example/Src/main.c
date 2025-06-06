/**
 *
 * Copyright (c) 2023 HT Micron Semicondutores S.A.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
#include "semphr.h"
#include <stdbool.h>

#define USART_BUFFER_SIZE 100


static uint32_t uart_cntrl = (ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE | 
                                ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE);

extern USART_HandleTypeDef huart1;

static uint8_t rx_buffer[USART_BUFFER_SIZE] = {0};
uint8_t controlByte[1] = {0};

QueueHandle_t xFilaFrequencia;
SemaphoreHandle_t xSemaforo;
SemaphoreHandle_t xUartRxSemaphore;




void uart_receive_cmd(void *pvParameters){
    while(1) {
        printf("Freq led 1 >>\n");
        uint32_t idx = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));

        while(1){
            HAL_USART_ReceivePolling(&huart1, controlByte, 1);
                
                if (controlByte[0] == '\n') {
                    break;
                }

                if (idx < USART_BUFFER_SIZE - 1) {  // Evita ultrapassar os limites do buffer
                    rx_buffer[idx++] = controlByte[0];
                }
        }
        rx_buffer[idx] = '\0';
        printf("Valor de Freq ->: %s\n", (char *)rx_buffer);
        
        int nova_freq = atoi((char *)rx_buffer);

        if (xQueueSend(xFilaFrequencia, &nova_freq, portMAX_DELAY) == pdTRUE) {
                printf("Frequência enviada para a fila: %d Hz\n", nova_freq);
        } else {
            printf("Falha ao enviar frequência para fila.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Task1(void *pvParameters) {
    bool button_state = false;
    bool state_past = false;

    while (1) {
      button_state = (bool) HT_GPIO_PinRead(BUTTON_INSTANCE, BUTTON_PIN);
      if (button_state != state_past) {
            if (button_state) {
                printf("Botão pressionado. Liberando semáforo...\n");
                xSemaphoreGive(xSemaforo);
            }
            state_past = button_state;
      }
      vTaskDelay(pdMS_TO_TICKS(100));      
    }
}

void Task2(void *pvParameters) {
   bool state = false;
    int frequencia = 1;

    while (1) {
        // Espera por um novo valor de frequência na fila
        if (xQueueReceive(xFilaFrequencia, &frequencia, portMAX_DELAY) == pdTRUE) {
            printf("Frequência configurada para: %d Hz\n", frequencia);

            if (frequencia == 0) {
                // Desliga o LED e espera nova frequência
                state = false;
                HT_GPIO_WritePin(LED_GPIO_PIN, LED_INSTANCE, state);

                while (1) {
                    int nova_freq = 0;
                    if (xQueueReceive(xFilaFrequencia, &nova_freq, portMAX_DELAY) == pdTRUE) {
                        frequencia = nova_freq;
                        printf("Frequência configurada para: %d Hz\n", frequencia);
                        break;
                    }
                }
            }

            // Piscar enquanto frequência > 0
            while (frequencia > 0) {
                state = !state;
                HT_GPIO_WritePin(LED_GPIO_PIN, LED_INSTANCE, state);

                int periodo_ms = 1000 / frequencia;
                vTaskDelay(pdMS_TO_TICKS(periodo_ms / 2));

                int nova_freq = 0;
                if (xQueueReceive(xFilaFrequencia, &nova_freq, 0) == pdTRUE) {
                    frequencia = nova_freq;
                    printf("Frequência configurada para: %d Hz\n", frequencia);
                    break;
                }
            }
        }
    }
}

/**
  \fn          int main_entry(void)
  \brief       main entry function.
  \return
*/
void main_entry(void) {

    HAL_USART_InitPrint(&huart1, GPR_UART1ClkSel_26M, uart_cntrl, 115200);
    
    HT_GPIO_InitButton();
    HT_GPIO_InitLed();

    slpManNormalIOVoltSet(IOVOLT_3_30V);

    xFilaFrequencia = xQueueCreate(5, sizeof(int));  // Fila de até 5 inteiros

    if (xFilaFrequencia == NULL) {
        printf("Erro ao criar a fila de frequências!\n");
        while(1);
    }

    printf("Exemplo FreeRTOS\n");

    //xTaskCreate(Task1, "Blink", 128, NULL, 2, NULL);
    xTaskCreate(Task2, "Print", 128, NULL, 10, NULL);
    xTaskCreate(uart_receive_cmd, "Uart_Rcv", 1024, NULL, 1, NULL);

    vTaskStartScheduler();
  
    printf("Nao deve chegar aqui.\n");

    while(1);

    
   
}

/******** HT Micron Semicondutores S.A **END OF FILE*/
