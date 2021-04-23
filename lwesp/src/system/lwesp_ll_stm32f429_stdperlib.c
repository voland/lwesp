/**
 * \file            lwesp_ll_stm32f429zi_nucleo.c
 * \brief           Low-level communication with ESP device for STM32F429ZI-Nucleo using DMA
 */

/*
 * Copyright (c) 2021 Arkadiusz Gil
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwESP - Lightweight ESP-AT parser library.
 *
 * Author:          Arkadisz Gil <arkadiusz.gil@gmail.com>
 * Version:         v1.1.0-dev
 */

#if !__DOXYGEN__

/* #include "stm32f4xx_ll_bus.h" */
/* #include "stm32f4xx_ll_usart.h" */
/* #include "stm32f4xx_ll_gpio.h" */
/* #include "stm32f4xx_ll_dma.h" */
/* #include "stm32f4xx_ll_rcc.h" */

#include "lwesp/lwesp.h"
#include "lwesp/lwesp_mem.h"
/* #include "lwesp/lwesp_input.h" */
/* #include "system/lwesp_ll.h" */
#include "stm32f2xx.h"

/* USART */
#define LWESP_USART USART6
#define LWESP_USART_CLK RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
#define LWESP_USART_IRQ USART6_IRQn
#define LWESP_USART_IRQHANDLER USART6_IRQHandler
#define LWESP_USART_RDR_NAME DR

/* /1* DMA settings *1/ */
#define LWESP_USART_DMA DMA2
#define LWESP_USART_DMA_CLK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
/* #define LWESP_USART_DMA_RX_STREAM DMA_STREAM_1 */
/* #define LWESP_USART_DMA_RX_CH LL_DMA_CHANNEL_4 */
/* #define LWESP_USART_DMA_RX_IRQ DMA1_Stream5_IRQn */
/* #define LWESP_USART_DMA_RX_IRQHANDLER DMA1_Stream5_IRQHandler */

/* /1* DMA flags management *1/ */
/* #define LWESP_USART_DMA_RX_IS_TC LL_DMA_IsActiveFlag_TC5(LWESP_USART_DMA) */
/* #define LWESP_USART_DMA_RX_IS_HT LL_DMA_IsActiveFlag_HT5(LWESP_USART_DMA) */
/* #define LWESP_USART_DMA_RX_CLEAR_TC LL_DMA_ClearFlag_TC5(LWESP_USART_DMA) */
/* #define LWESP_USART_DMA_RX_CLEAR_HT LL_DMA_ClearFlag_HT5(LWESP_USART_DMA) */

/* USART TX PIN */
#define LWESP_USART_TX_PORT_CLK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define LWESP_USART_TX_PORT GPIOC
#define LWESP_USART_TX_PIN GPIO_Pin_6
#define LWESP_USART_TX_PIN_SOURCE GPIO_PinSource6
#define LWESP_USART_TX_PIN_AF GPIO_AF_USART6

/* USART RX PIN */
#define LWESP_USART_RX_PORT_CLK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE)
#define LWESP_USART_RX_PORT GPIOC
#define LWESP_USART_RX_PIN GPIO_Pin_7
#define LWESP_USART_RX_PIN_SOURCE GPIO_PinSource7
#define LWESP_USART_RX_PIN_AF GPIO_AF_USART6

/* /1* RESET PIN *1/ */
/* #define LWESP_RESET_PORT_CLK LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD) */
/* #define LWESP_RESET_PORT GPIOD */
/* #define LWESP_RESET_PIN LL_GPIO_PIN_1 */

/* /1* GPIO0 PIN *1/ */
/* #define LWESP_GPIO0_PORT_CLK LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD) */
/* #define LWESP_GPIO0_PORT GPIOD */
/* #define LWESP_GPIO0_PIN LL_GPIO_PIN_4 */

/* /1* GPIO2 PIN *1/ */
/* #define LWESP_GPIO2_PORT_CLK LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD) */
/* #define LWESP_GPIO2_PORT GPIOD */
/* #define LWESP_GPIO2_PIN LL_GPIO_PIN_7 */

/* /1* CH_PD PIN *1/ */
/* #define LWESP_CH_PD_PORT_CLK LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD) */
/* #define LWESP_CH_PD_PORT GPIOD */
/* #define LWESP_CH_PD_PIN LL_GPIO_PIN_3 */

#if !LWESP_CFG_INPUT_USE_PROCESS
#error "LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver."
#endif /* LWESP_CFG_INPUT_USE_PROCESS */

#if !defined(LWESP_USART_DMA_RX_BUFF_SIZE)
#define LWESP_USART_DMA_RX_BUFF_SIZE 0x1000
#endif /* !defined(LWESP_USART_DMA_RX_BUFF_SIZE) */

#if !defined(LWESP_MEM_SIZE)
#define LWESP_MEM_SIZE 0x1000
#endif /* !defined(LWESP_MEM_SIZE) */

/* #if !defined(LWESP_USART_RDR_NAME) */
/* #define LWESP_USART_RDR_NAME RDR */
/* #endif /1* !defined(LWESP_USART_RDR_NAME) *1/ */

/* USART memory */
/* static uint8_t usart_mem[LWESP_USART_DMA_RX_BUFF_SIZE]; */
static uint8_t is_running, initialized;
static size_t old_pos;

/* USART thread */
static void usart_ll_thread(void *arg);
/* static osThreadId_t usart_ll_thread_id; */

/* Message queue */
/* static osMessageQueueId_t usart_ll_mbox_id; */

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void *arg) {
    /* size_t pos; */

    /* LWESP_UNUSED(arg); */

    /* while (1) { */
    /*     void* d; */
    /*     /1* Wait for the event message from DMA or USART *1/ */
    /*     osMessageQueueGet(usart_ll_mbox_id, &d, NULL, osWaitForever); */

    /*     /1* Read data *1/ */
    /* #if defined(LWESP_USART_DMA_RX_STREAM) */
    /*     pos = sizeof(usart_mem) - LL_DMA_GetDataLength(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
    /* #else */
    /*     pos = sizeof(usart_mem) - LL_DMA_GetDataLength(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
    /* #endif /1* defined(LWESP_USART_DMA_RX_STREAM) *1/ */
    /*     if (pos != old_pos && is_running) { */
    /*         if (pos > old_pos) { */
    /*             lwesp_input_process(&usart_mem[old_pos], pos - old_pos); */
    /*         } else { */
    /*             lwesp_input_process(&usart_mem[old_pos], sizeof(usart_mem) - old_pos); */
    /*             if (pos > 0) { */
    /*                 lwesp_input_process(&usart_mem[0], pos); */
    /*             } */
    /*         } */
    /*         old_pos = pos; */
    /*         if (old_pos == sizeof(usart_mem)) { */
    /*             old_pos = 0; */
    /*         } */
    /*     } */
    /* } */
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void
configure_uart(uint32_t baudrate) {
    static USART_InitTypeDef usart_init;
    static DMA_InitTypeDef dma_init;
    GPIO_InitTypeDef gpio_init;

    if (!initialized) {
        /* Enable peripheral clocks */
        LWESP_USART_CLK;
        LWESP_USART_DMA_CLK;
        LWESP_USART_TX_PORT_CLK;
        LWESP_USART_RX_PORT_CLK;

        /* #if defined(LWESP_RESET_PIN) */
        /* #error reset pin not used phisycly */
        /* #endif /1* defined(LWESP_RESET_PIN) *1/ */

        /* #if defined(LWESP_GPIO0_PIN) */
        /* #error LWESP_GPIO0_PORT_CLK not used phisycly */
        /* #endif /1* defined(LWESP_GPIO0_PIN) *1/ */

        /* #if defined(LWESP_GPIO2_PIN) */
        /* #error LWESP_GPIO2_PORT_CLK; */
        /* #endif /1* defined(LWESP_GPIO2_PIN) *1/ */

        /* #if defined(LWESP_CH_PD_PIN) */
        /* #error LWESP_CH_PD_PORT_CLK; */
        /* #endif /1* defined(LWESP_CH_PD_PIN) *1/ */

        /* Global pin configuration */
        GPIO_StructInit(&gpio_init);
        gpio_init.GPIO_OType = GPIO_OType_PP;
        gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
        gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
        /* gpio_init.GPIO_Mode = GPIO_Mode_OUT; */

        /* #if defined(LWESP_RESET_PIN) */
        /*     /1* Configure RESET pin *1/ */
        /*     gpio_init.Pin = LWESP_RESET_PIN; */
        /*     LL_GPIO_Init(LWESP_RESET_PORT, &gpio_init); */
        /* #endif /1* defined(LWESP_RESET_PIN) *1/ */

        /* #if defined(LWESP_GPIO0_PIN) */
        /*     /1* Configure GPIO0 pin *1/ */
        /*     gpio_init.Pin = LWESP_GPIO0_PIN; */
        /*     LL_GPIO_Init(LWESP_GPIO0_PORT, &gpio_init); */
        /*     LL_GPIO_SetOutputPin(LWESP_GPIO0_PORT, LWESP_GPIO0_PIN); */
        /* #endif /1* defined(LWESP_GPIO0_PIN) *1/ */

        /* #if defined(LWESP_GPIO2_PIN) */
        /*     /1* Configure GPIO2 pin *1/ */
        /*     gpio_init.Pin = LWESP_GPIO2_PIN; */
        /*     LL_GPIO_Init(LWESP_GPIO2_PORT, &gpio_init); */
        /*     LL_GPIO_SetOutputPin(LWESP_GPIO2_PORT, LWESP_GPIO2_PIN); */
        /* #endif /1* defined(LWESP_GPIO2_PIN) *1/ */

        /* #if defined(LWESP_CH_PD_PIN) */
        /*     /1* Configure CH_PD pin *1/ */
        /*     gpio_init.Pin = LWESP_CH_PD_PIN; */
        /*     LL_GPIO_Init(LWESP_CH_PD_PORT, &gpio_init); */
        /*     LL_GPIO_SetOutputPin(LWESP_CH_PD_PORT, LWESP_CH_PD_PIN); */
        /* #endif /1* defined(LWESP_CH_PD_PIN) *1/ */

        /* Configure USART pins */
        gpio_init.GPIO_Mode = GPIO_Mode_AF;

        /* TX PIN */
        gpio_init.GPIO_Pin = LWESP_USART_TX_PIN;
        GPIO_Init(LWESP_USART_TX_PORT, &gpio_init);
        GPIO_PinAFConfig(LWESP_USART_TX_PORT, LWESP_USART_TX_PIN_SOURCE, LWESP_USART_TX_PIN_AF);

        /* RX PIN */
        gpio_init.GPIO_Pin = LWESP_USART_RX_PIN;
        GPIO_Init(LWESP_USART_RX_PORT, &gpio_init);
        GPIO_PinAFConfig(LWESP_USART_RX_PORT, LWESP_USART_RX_PIN_SOURCE, LWESP_USART_RX_PIN_AF);

        /* Configure UART */
        USART_DeInit(LWESP_USART);
        USART_StructInit(&usart_init);
        usart_init.USART_BaudRate = baudrate;
        usart_init.USART_WordLength = USART_WordLength_8b;
        usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        /* usart_init.o = LL_USART_OVERSAMPLING_16; */
        usart_init.USART_Parity = USART_Parity_No;
        usart_init.USART_StopBits = USART_StopBits_1;
        USART_Init(LWESP_USART, &usart_init);

        /* /1* / 1 * Enable USART interrupts and DMA request * 1 / * / */
        USART_ITConfig(LWESP_USART, USART_IT_IDLE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_PE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_ERR, ENABLE);
        /*     LL_USART_EnableDMAReq_RX(LWESP_USART); */

        /*     /1* Enable USART interrupts *1/ */
        /*     NVIC_SetPriority(LWESP_USART_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00)); */
        /*     NVIC_EnableIRQ(LWESP_USART_IRQ); */

        /*     /1* Configure DMA *1/ */
        /*     is_running = 0; */
        /* #if defined(LWESP_USART_DMA_RX_STREAM) */
        /*     LL_DMA_DeInit(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /*     dma_init.Channel = LWESP_USART_DMA_RX_CH; */
        /* #else */
        /*     LL_DMA_DeInit(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
        /*     dma_init.PeriphRequest = LWESP_USART_DMA_RX_REQ_NUM; */
        /* #endif /1* defined(LWESP_USART_DMA_RX_STREAM) *1/ */
        /*     dma_init.PeriphOrM2MSrcAddress = (uint32_t)&LWESP_USART->LWESP_USART_RDR_NAME; */
        /*     dma_init.MemoryOrM2MDstAddress = (uint32_t)usart_mem; */
        /*     dma_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY; */
        /*     dma_init.Mode = LL_DMA_MODE_CIRCULAR; */
        /*     dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT; */
        /*     dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT; */
        /*     dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE; */
        /*     dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE; */
        /*     dma_init.NbData = sizeof(usart_mem); */
        /*     dma_init.Priority = LL_DMA_PRIORITY_MEDIUM; */
        /* #if defined(LWESP_USART_DMA_RX_STREAM) */
        /*     LL_DMA_Init(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM, &dma_init); */
        /* #else */
        /*     LL_DMA_Init(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH, &dma_init); */
        /* #endif /1* defined(LWESP_USART_DMA_RX_STREAM) *1/ */

        /*     /1* Enable DMA interrupts *1/ */
        /* #if defined(LWESP_USART_DMA_RX_STREAM) */
        /*     LL_DMA_EnableIT_HT(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /*     LL_DMA_EnableIT_TC(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /*     LL_DMA_EnableIT_TE(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /*     LL_DMA_EnableIT_FE(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /*     LL_DMA_EnableIT_DME(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /* #else */
        /*     LL_DMA_EnableIT_HT(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
        /*     LL_DMA_EnableIT_TC(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
        /*     LL_DMA_EnableIT_TE(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
        /* #endif /1* defined(LWESP_USART_DMA_RX_STREAM) *1/ */

        /*     /1* Enable DMA interrupts *1/ */
        /*     NVIC_SetPriority(LWESP_USART_DMA_RX_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00)); */
        /*     NVIC_EnableIRQ(LWESP_USART_DMA_RX_IRQ); */

        /*     old_pos = 0; */
        /*     is_running = 1; */

        /*     /1* Start DMA and USART *1/ */
        /* #if defined(LWESP_USART_DMA_RX_STREAM) */
        /*     LL_DMA_EnableStream(LWESP_USART_DMA, LWESP_USART_DMA_RX_STREAM); */
        /* #else */
        /*     LL_DMA_EnableChannel(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
        /* #endif /1* defined(LWESP_USART_DMA_RX_STREAM) *1/ */
        USART_Cmd(LWESP_USART, ENABLE);
        print_esp("dupalala");
        return;
    } else {
        /* vTaskDelay(10); */
        /* USART_Disable(LWESP_USART); */
        /* /1*     usart_init.BaudRate = baudrate; *1/ */
        /* USART_Init(LWESP_USART, &usart_init); */
        /* USART_Enable(LWESP_USART); */
    }

    /* /1* Create mbox and start thread *1/ */
    /* if (usart_ll_mbox_id == NULL) { */
    /*     usart_ll_mbox_id = osMessageQueueNew(10, sizeof(void*), NULL); */
    /* } */
    /* if (usart_ll_thread_id == NULL) { */
    /*     const osThreadAttr_t attr = { */
    /*         .stack_size = 1024 */
    /*     }; */
    /*     usart_ll_thread_id = osThreadNew(usart_ll_thread, usart_ll_mbox_id, &attr); */
    /* } */
}

#if defined(LWESP_RESET_PIN)
/**
 * \brief           Hardware reset callback
 */
static uint8_t
reset_device(uint8_t state) {
    //not implemented
    return 1;
}
#endif /* defined(LWESP_RESET_PIN) */

/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void *data, size_t len) {
    char *d = (char *)data;
    for (int i = 0; i < len; i++) {
        while ((USART6->SR & USART_FLAG_TC) == (uint16_t)RESET) {}
        LWESP_USART->LWESP_USART_RDR_NAME = (d[i] & (uint16_t)0x01FF);
    }
    return len;
}

void print_esp(char *txt) {
    if (txt != NULL) {
        send_data(txt, strlen(txt));
    }
}

/**
 * \brief           Callback function called from initialization process
 */
lwespr_t
lwesp_ll_init(lwesp_ll_t *ll) {
#if !LWESP_CFG_MEM_CUSTOM
    static uint8_t memory[LWESP_MEM_SIZE];
    lwesp_mem_region_t mem_regions[] = {
        {memory, sizeof(memory)}};

    if (!initialized) {
        lwesp_mem_assignmemory(mem_regions, LWESP_ARRAYSIZE(mem_regions)); /* Assign memory for allocations */
    }
#endif /* !LWESP_CFG_MEM_CUSTOM */

    if (!initialized) {
        ll->send_fn = send_data; /* Set callback function to send data */
#if defined(LWESP_RESET_PIN)
        ll->reset_fn = reset_device; /* Set callback for hardware reset */
#endif                               /* defined(LWESP_RESET_PIN) */
    }

    configure_uart(ll->uart.baudrate); /* Initialize UART for communication */
    initialized = 1;
    return lwespOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 */
lwespr_t
lwesp_ll_deinit(lwesp_ll_t *ll) {
    //not implemented
    return lwespOK;
}

/**
 * \brief           UART global interrupt handler
 */
//void LWESP_USART_IRQHANDLER(void) {
/* LL_USART_ClearFlag_IDLE(LWESP_USART); */
/* LL_USART_ClearFlag_PE(LWESP_USART); */
/* LL_USART_ClearFlag_FE(LWESP_USART); */
/* LL_USART_ClearFlag_ORE(LWESP_USART); */
/* LL_USART_ClearFlag_NE(LWESP_USART); */

/* if (usart_ll_mbox_id != NULL) { */
/*     void* d = (void*)1; */
/*     osMessageQueuePut(usart_ll_mbox_id, &d, 0, 0); */
/* } */
//}

/**
 * \brief           UART DMA stream/channel handler
 */
void LWESP_USART_DMA_RX_IRQHANDLER(void) {
    /* LWESP_USART_DMA_RX_CLEAR_TC; */
    /* LWESP_USART_DMA_RX_CLEAR_HT; */

    /* if (usart_ll_mbox_id != NULL) { */
    /*     void* d = (void*)1; */
    /*     osMessageQueuePut(usart_ll_mbox_id, &d, 0, 0); */
    /* } */
}

#endif /* !__DOXYGEN__ */
