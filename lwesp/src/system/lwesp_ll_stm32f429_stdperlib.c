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

#include "lwesp_sys_port.h"
#if !__DOXYGEN__

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

/* DMA settings */
#define LWESP_USART_DMA DMA2
#define LWESP_USART_DMA_CLK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
#define LWESP_USART_DMA_RX_STREAM DMA2_Stream1
/* #define LWESP_USART_DMA_RX_STREAM DMA2_Stream2 */ //just in case
#define LWESP_USART_DMA_RX_CH DMA_Channel_5
#define LWESP_USART_DMA_RX_IRQ DMA2_Stream1_IRQn
#define LWESP_USART_DMA_RX_IRQHANDLER DMA2_Stream1_IRQHandler

/* DMA flags management */
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
static uint8_t usart_mem[LWESP_USART_DMA_RX_BUFF_SIZE];
static uint8_t is_running = 0;
static uint8_t initialized = 0;
static size_t old_pos = 0;

/* USART thread */
void usart_ll_thread(void *arg);
static lwesp_sys_thread_t usart_ll_thread_id = NULL;

/* Message queue */
static lwesp_sys_mbox_t usart_ll_mbox_id = NULL;

/**
 * \brief           USART data processing
 */

static void _PutUsartMbox() {
    if (usart_ll_mbox_id != NULL) {
        void *d = (void *)1;
        lwesp_sys_mbox_putnow(&usart_ll_mbox_id, d);
    }
}

void usart_ll_thread(void *arg) {
    size_t pos;

    LWESP_UNUSED(arg);

    while (1) {
        void *d;
        /* Wait for the event message from DMA or USART, forever */
        lwesp_sys_mbox_get(&usart_ll_mbox_id, &d, 0);

/* Read data */
#if defined(LWESP_USART_DMA_RX_STREAM)
        pos = sizeof(usart_mem) - DMA_GetCurrDataCounter(LWESP_USART_DMA_RX_STREAM);
#else
        pos = sizeof(usart_mem) - DMA_GetDataLength(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */
        if (pos != old_pos && is_running) {
            if (pos > old_pos) {
                lwesp_input_process(&usart_mem[old_pos], pos - old_pos);
            } else {
                lwesp_input_process(&usart_mem[old_pos], sizeof(usart_mem) - old_pos);
                if (pos > 0) {
                    lwesp_input_process(&usart_mem[0], pos);
                }
            }
            old_pos = pos;
            if (old_pos == sizeof(usart_mem)) {
                old_pos = 0;
            }
        }
    }
}

static void StartUsart_ll_thread() {
    if (usart_ll_mbox_id == NULL) {
        if (lwesp_sys_mbox_create(&usart_ll_mbox_id, 10)) {
        }
    }
    if (usart_ll_thread_id == NULL) {
        //TODO: consider what priority best to use for it
        lwesp_sys_thread_create(&usart_ll_thread_id, "esp_AT_usart_thread",
                                usart_ll_thread, usart_ll_mbox_id, 1024, 1);
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void configure_uart(uint32_t baudrate) {
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

        /* /1* RX PIN *1/ */
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

        /* NVIC_Configuration(); */
        // Enable USART interrupts and DMA request * /
        USART_ITConfig(LWESP_USART, USART_IT_IDLE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_PE, ENABLE);
        USART_ITConfig(LWESP_USART, USART_IT_ERR, ENABLE);
        USART_DMACmd(LWESP_USART, USART_DMAReq_Rx, ENABLE);

        /* Enable USART interrupts */
        NVIC_SetPriority(LWESP_USART_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00));
        NVIC_EnableIRQ(LWESP_USART_IRQ);

        /* Configure DMA */
        is_running = 0;
#if defined(LWESP_USART_DMA_RX_STREAM)
        DMA_DeInit(LWESP_USART_DMA_RX_STREAM);
        dma_init.DMA_Channel = LWESP_USART_DMA_RX_CH;
#else
/* DMA_DeInit(LWESP_USART_DMA_RX_STREAM); */
/* dma_init.PeriphRequest = LWESP_USART_DMA_RX_REQ_NUM; */
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */
        memset(usart_mem, 0, sizeof(usart_mem));
        dma_init.DMA_PeripheralBaseAddr = (uint32_t)&LWESP_USART->LWESP_USART_RDR_NAME;
        dma_init.DMA_Memory0BaseAddr = (uint32_t)usart_mem;
        dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma_init.DMA_Mode = DMA_Mode_Circular;
        dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma_init.DMA_BufferSize = sizeof(usart_mem);
        dma_init.DMA_Priority = DMA_Priority_Medium;
#if defined(LWESP_USART_DMA_RX_STREAM)
        DMA_Init(LWESP_USART_DMA_RX_STREAM, &dma_init);
#else
        DMA_Init(LWESP_USART_DMA_RX_CH, &dma_init);
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */

/* Enable DMA interrupts */
#if defined(LWESP_USART_DMA_RX_STREAM)
        DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_HT, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_TE, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_FE, ENABLE);
        DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_DME, ENABLE);
#else
        /* DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_HT, ENABLE); */
        /* DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_TC, ENABLE); */
        /* DMA_ITConfig(LWESP_USART_DMA_RX_STREAM, DMA_IT_TE, ENABLE); */
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */

        /* Enable DMA interrupts */
        NVIC_SetPriority(LWESP_USART_DMA_RX_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00));
        NVIC_EnableIRQ(LWESP_USART_DMA_RX_IRQ);

        old_pos = 0;
        is_running = 1;

/* Start DMA and USART */
#if defined(LWESP_USART_DMA_RX_STREAM)
        DMA_Cmd(LWESP_USART_DMA_RX_STREAM, ENABLE);
#else
/* LL_DMA_EnableChannel(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH); */
#endif /* defined(LWESP_USART_DMA_RX_STREAM) */
        USART_Cmd(LWESP_USART, ENABLE);
    } else {
        vTaskDelay(100);
        USART_Cmd(LWESP_USART, DISABLE);
        usart_init.USART_BaudRate = baudrate;
        USART_Init(LWESP_USART, &usart_init);
        USART_Cmd(LWESP_USART, ENABLE);
    }
    /* Create mbox and start thread */
    StartUsart_ll_thread();
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
#ifdef WIFI_ESP_ENABLED
#define USART_CLEARITPENDINGBIT(USART, IT) USART->SR = (uint16_t) ~(1 << (IT >> 8))
void LWESP_USART_IRQHANDLER(void) {
    USART_ClearITPendingBit(LWESP_USART, USART_IT_IDLE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_PE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_FE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_ORE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_NE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_RXNE);
    USART_ClearITPendingBit(LWESP_USART, USART_IT_ERR);
    int d;
    d = LWESP_USART->SR;
    (void)d;
    d = LWESP_USART->DR;
    (void)d;
    _PutUsartMbox();
}
#endif

/**
 * \brief           UART DMA stream/channel handler
 */
void LWESP_USART_DMA_RX_IRQHANDLER(void) {
    DMA_ClearITPendingBit(LWESP_USART_DMA_RX_STREAM, DMA_IT_TCIF1);
    DMA_ClearITPendingBit(LWESP_USART_DMA_RX_STREAM, DMA_IT_HTIF1);
    _PutUsartMbox();
}

#endif /* !__DOXYGEN__ */
