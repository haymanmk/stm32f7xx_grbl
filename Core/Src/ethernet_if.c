#include <stdio.h>
#include "stm32f7xx_grbl.h"
#include "ethernet_if.h"
#include "encoder.h"
#include "grbl.h"

#define BUFFER_SIZE 512
#define CONNECTION_CREATED_MSK 1

extern NetworkInterface_t *pxSTM32Fxx_FillInterfaceDescriptor(BaseType_t xEMACIndex,
                                                              NetworkInterface_t *pxInterface);
static void prvCreateTCPServerSocketTasks(void *pvParameters);
static void prvEchoClientRxTask(void *pvParameters);
static void prvSerialTask(void *pvParameters);
static void prvBlinkLED(void *pvParameters);

NetworkInterface_t xInterfaces[1];
struct xNetworkEndPoint xEndPoints[1];
uint8_t socketShutdownTimeout = 0;
TaskHandle_t createServerSocketTaskHandle;
TaskHandle_t echoClientRxTaskHandle;
TaskHandle_t serialTaskHandle;
static SemaphoreHandle_t connectionCreatedSemaphoreHandle;
SemaphoreHandle_t deleteTaskSemaphoreHandle;
static uint16_t listeningPort = 0;

BaseType_t tcp_server_init()
{
    /* Initialize Semaphore handler to protect the critical section of
       safely shutting down connection */
    connectionCreatedSemaphoreHandle = xSemaphoreCreateBinary();

    /* Initialise the interface descriptor for WinPCap for example. */
    pxSTM32Fxx_FillInterfaceDescriptor(0, &(xInterfaces[0]));

    uint8_t IPAddr[4] = {
        settings.ip_address_0,
        settings.ip_address_1,
        settings.ip_address_2,
        settings.ip_address_3};
    uint8_t GatewayAddr[4] = {
        settings.ip_address_0,
        settings.ip_address_1,
        settings.ip_address_2,
        1};
    listeningPort = LISTENING_PORT + settings.tcp_port;
    // set MAC address with STM32 UID
    MACAddr[0] = settings.mac_address_0;
    MACAddr[1] = settings.mac_address_1;
    MACAddr[2] = settings.mac_address_2;
    MACAddr[3] = settings.mac_address_3;
    MACAddr[4] = settings.mac_address_4;
    MACAddr[5] = settings.mac_address_5;

    // check if USER Button is continuously pressed over 5 seconds,
    // then use the default IP address and port
    if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    {
        // wait for 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));

        // check if USER Button is still pressed
        if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
        {
            // use the default IP address and port
            IPAddr[0] = 172;
            IPAddr[1] = 16;
            IPAddr[2] = 0;
            IPAddr[3] = 10;

            GatewayAddr[0] = 172;
            GatewayAddr[1] = 16;
            GatewayAddr[2] = 0;
            GatewayAddr[3] = 1;

            listeningPort = LISTENING_PORT;

            // start the LED blinking task
            xTaskCreate(prvBlinkLED, "BlinkLED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
        }
    }

    FreeRTOS_FillEndPoint(&(xInterfaces[0]), &(xEndPoints[0]), IPAddr,
                          NetMask, GatewayAddr, DNSAddr, MACAddr);
#if (ipconfigUSE_DHCP != 0)
    {
        /* End-point 0 wants to use DHCPv4. */
        xEndPoints[0].bits.bWantDHCP = pdTRUE;
    }
#endif /* ( ipconfigUSE_DHCP != 0 ) */

    /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
       are created in the vApplicationIPNetworkEventHook() hook function
       below.  The hook function is called when the network connects. */
    FreeRTOS_IPInit_Multi();

    return SUCCESS;
}

/**
 * @brief  Initializes the ETH MSP.
 * @param  ethHandle: ETH handle
 * @retval None
 */

void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspInit 0 */

        /* USER CODE END ETH_MspInit 0 */
        /* Enable Peripheral clock */
        __HAL_RCC_ETH_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB13     ------> ETH_TXD1
        PG11     ------> ETH_TX_EN
        PG13     ------> ETH_TXD0
        */
        GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_TXD1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        /* ETH interrupt Init */
        HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_IRQn);
        HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);

        /* USER CODE BEGIN ETH_MspInit 1 */

        /* USER CODE END ETH_MspInit 1 */
    }
}

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t eNetworkEvent,
                                          struct xNetworkEndPoint *pxEndPoint)
{
    static BaseType_t xTasksAlreadyCreated = pdFALSE;

    /* Both eNetworkUp and eNetworkDown events can be processed here. */
    if (eNetworkEvent == eNetworkUp)
    {
        /* Create the tasks that use the TCP/IP stack if they have not already
        been created. */
        if (xTasksAlreadyCreated == pdFALSE)
        {
            /*
             * For convenience, tasks that use FreeRTOS-Plus-TCP can be created here
             * to ensure they are not created before the network is usable.
             */

            xTasksAlreadyCreated = pdTRUE;

            vStartSimpleTCPServerTasks(4 * configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY);
        }
    }
    /* Print out the network configuration, which may have come from a DHCP
     * server. */
    // showEndPoint(pxEndPoint);
}

/* Called by FreeRTOS-Plus-TCP */
BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
    *pulNumber = HAL_GetTick();
    return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort)
{
    uint32_t pulNumber = 0;
    xApplicationGetRandomNumber(&pulNumber);
    return pulNumber;
}

/* Stores the stack size passed into vStartSimpleTCPServerTasks() so it can be
 * reused when the server listening task creates tasks to handle connections. */
static uint16_t usUsedStackSize = 0;

void vStartSimpleTCPServerTasks(uint16_t usStackSize, UBaseType_t uxPriority)
{
    xTaskCreate(prvCreateTCPServerSocketTasks, "TCPServerListener", usStackSize, NULL, uxPriority, &createServerSocketTaskHandle);

    /* Remember the requested stack size so it can be re-used by the server
     * listening task when it creates tasks to handle connections. */
    usUsedStackSize = usStackSize;
}

static void prvCreateTCPServerSocketTasks(void *pvParameters)
{
    struct freertos_sockaddr xClient, xBindAddress;
    Socket_t xListeningSocket, xConnectedSocket;
    socklen_t xSize = sizeof(xClient);
    static const TickType_t xReceiveTimeOut = portMAX_DELAY;
    const BaseType_t xBacklog = 1;

    /* Attempt to open the socket. */
    xListeningSocket = FreeRTOS_socket(FREERTOS_AF_INET4,    /* Or FREERTOS_AF_INET6 for IPv6. */
                                       FREERTOS_SOCK_STREAM, /* SOCK_STREAM for TCP. */
                                       FREERTOS_IPPROTO_TCP);

    /* Check the socket was created. */
    configASSERT(xListeningSocket != FREERTOS_INVALID_SOCKET);

    /* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
    FreeRTOS_setsockopt() to change the buffer sizes from their default then do
    it here!.  (see the FreeRTOS_setsockopt() documentation. */

    /* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
    be used with FreeRTOS_setsockopt() to change the sliding window size from
    its default then do it here! (see the FreeRTOS_setsockopt()
    documentation. */

    /* Set a time out so accept() will just wait for a connection. */
    FreeRTOS_setsockopt(xListeningSocket,
                        0,
                        FREERTOS_SO_RCVTIMEO,
                        &xReceiveTimeOut,
                        sizeof(xReceiveTimeOut));

    /* Set the listening port to 10000. */
    // xBindAddress.sin_address = (IP_Address_t)FreeRTOS_inet_addr_quick(0, 0, 0, 0);
    xBindAddress.sin_port = listeningPort;
    xBindAddress.sin_port = FreeRTOS_htons(xBindAddress.sin_port);
    xBindAddress.sin_family = FREERTOS_AF_INET;

    /* Bind the socket to the port that the client RTOS task will send to. */
    FreeRTOS_bind(xListeningSocket, &xBindAddress, sizeof(xBindAddress));

    /* Set the socket into a listening state so it can accept connections.
    The maximum number of simultaneous connections is limited to 20. */
    FreeRTOS_listen(xListeningSocket, xBacklog);

    for (;;)
    {
        if (xSocketValid(xListeningSocket) == pdTRUE)
        {
            /* Wait for incoming connections. */
            xConnectedSocket = FreeRTOS_accept(xListeningSocket, &xClient, &xSize);
            configASSERT(xConnectedSocket != FREERTOS_INVALID_SOCKET);

            /* Give semaphore a count to represent there is a connection that can be shut down. */
            xSemaphoreGive(connectionCreatedSemaphoreHandle);

            /* Spawn a RTOS task to handle the connection. */
            xTaskCreate(prvEchoClientRxTask,
                        "EchoServer",
                        usUsedStackSize,
                        (void *)xConnectedSocket,
                        tskIDLE_PRIORITY + 1,
                        &echoClientRxTaskHandle);

            // Create the serial task which is an interface between the serial port and the ethernet
            xTaskCreate(prvSerialTask,
                        "SerialTask",
                        configMINIMAL_STACK_SIZE * 2,
                        (void *)xConnectedSocket,
                        tskIDLE_PRIORITY + 1,
                        &serialTaskHandle);

            // wait here until the connection is closed
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // check if the task handle is valid
            if (echoClientRxTaskHandle != NULL)
            {
                // delete the task
                vTaskDelete(echoClientRxTaskHandle);
                echoClientRxTaskHandle = NULL;
            }
            if (serialTaskHandle != NULL)
            {
                // delete the task
                vTaskDelete(serialTaskHandle);
                serialTaskHandle = NULL;
            }

            // create binary semaphore to wait for idle task to delete the task
            deleteTaskSemaphoreHandle = xSemaphoreCreateBinary();

            if (deleteTaskSemaphoreHandle != NULL)
            {
                // wait for idle task to delete the task and free the memory
                xSemaphoreTake(deleteTaskSemaphoreHandle, portMAX_DELAY);
                // delete semaphore
                vSemaphoreDelete(deleteTaskSemaphoreHandle);
                deleteTaskSemaphoreHandle = NULL;
            }

            // clear notify value in case there is any task giving notification during the wait
            ulTaskNotifyValueClear(NULL, 0xffffffff);
        }
    }
}

void vTimeoutCallback(TimerHandle_t xTimer)
{
    socketShutdownTimeout = 1;
}

void safelyShutdownSocket(Socket_t xSocket)
{
    // beginning of critical section
    if (xSemaphoreTake(connectionCreatedSemaphoreHandle, pdMS_TO_TICKS(500)) == pdFALSE)
        return;

    vLoggingPrintf("Shutting down socket\n");
    // shutdown the socket
    FreeRTOS_shutdown(xSocket, FREERTOS_SHUT_RDWR);

    // Start a timeout watchdog to avoid blocking the task indefinitely
    socketShutdownTimeout = 0;
    TimerHandle_t xTimer = xTimerCreate("SocketShutdownTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, vTimeoutCallback);

    if (xTimer == NULL)
    {
        FreeRTOS_debug_printf(("Failed to create timer\n"));
    }

    // start timer
    if (xTimerStart(xTimer, 0) != pdPASS)
    {
        FreeRTOS_debug_printf(("Failed to start timer\n"));
    }

    while (FreeRTOS_recv(xSocket, NULL, 0, 0) >= 0 && !socketShutdownTimeout)
    {
        /* Wait for shutdown to complete.  If a receive block time is used then
        this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
        into the Blocked state anyway. */
        vTaskDelay(pdMS_TO_TICKS(250));

        /* Note - real applications should implement a timeout here, not just
        loop forever. */
    }

    // reset timer
    if (xTimerReset(xTimer, 0) != pdPASS)
    {
        FreeRTOS_debug_printf(("Failed to reset timer\n"));
    }

    // delete timer; note: this step is important to avoid memory leak
    if (xTimerDelete(xTimer, 0) != pdPASS)
    {
        FreeRTOS_debug_printf(("Failed to delete timer\n"));
    }

    /* Shutdown is complete and the socket can be safely closed. */
    FreeRTOS_closesocket(xSocket);
    xSocket = NULL;

    vLoggingPrintf("Socket shutdown complete\n");

    // end of critical section
}

uint8_t isDigit(char c)
{
    return c >= '0' && c <= '9';
}

void prvProcessData(char *cRxedData, BaseType_t lBytesReceived, Socket_t xConnectedSocket)
{
    FreeRTOS_debug_printf(("Received data: %.*s\n", lBytesReceived, cRxedData));

    BaseType_t i = 0;

    for (; i < lBytesReceived; i++)
    {
        /**
         * Process the received data
         */
        serial_rx_irq(cRxedData[i]);
    }
}

static void prvEchoClientRxTask(void *pvParameters)
{
    FreeRTOS_debug_printf(("Client connected\n"));

    Socket_t xSocket;
    static char cRxedData[BUFFER_SIZE];
    BaseType_t lBytesReceived;

    /* It is assumed the socket has already been created and connected before
    being passed into this RTOS task using the RTOS task's parameter. */
    xSocket = (Socket_t)pvParameters;

    for (;;)
    {
        /* Receive another block of data into the cRxedData buffer. */
        lBytesReceived = FreeRTOS_recv(xSocket, &cRxedData, BUFFER_SIZE, 0);

        if (lBytesReceived > 0)
        {
            /* Data was received, process it here. */
            prvProcessData(cRxedData, lBytesReceived, xSocket);
        }
        else if (lBytesReceived == 0)
        {
            /* No data was received, but FreeRTOS_recv() did not return an error.
            Timeout? */
        }
        else
        {
            /* Error (maybe the connected socket already shut down the socket?).
            Attempt graceful shutdown. */
            FreeRTOS_debug_printf(("Error on receive\n"));
            break;
        }
    }

    /* The RTOS task will get here if an error is received on a read.  Ensure the
    socket has shut down (indicated by FreeRTOS_recv() returning a -pdFREERTOS_ERRNO_EINVAL
    error before closing the socket). */
    if (xSocket)
        safelyShutdownSocket(xSocket);

    /* reset serial function in GRBL */
    serial_init();

    // clear task handle
    echoClientRxTaskHandle = NULL;

    // notify server socket task to delete the task
    xTaskNotifyGive(createServerSocketTaskHandle);

    /* Must not drop off the end of the RTOS task - delete the RTOS task. */
    // delete current task
    vTaskDelete(NULL);
}

static void prvSerialTask(void *pvParameters)
{
    Socket_t xSocket = (Socket_t)pvParameters;
    char str[105] = {'\0'};
    uint8_t strIndex = 0;

    for (;;)
    {
        // Wait for a notification from serial task in serial.c of grbl
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY); // decrement the notification count

        /**
         * TODO: Implement serial task
         * - check if ethernet is ready to send data
         * - concatenate the data to be sent until a newline character, '\n', is received
         */
        uint8_t chr = serial_tx_irq();

        // concatenate the data to be sent until a newline character, '\n', is received
        str[strIndex] = chr;

        if (++strIndex >= sizeof(str))
        {
            FreeRTOS_debug_printf(("Buffer overflow\n"));
            strIndex = 0;
        }

        // Check if the character is a newline character
        if (chr != '\n')
        {
            continue;
        }

        // Send the data
        BaseType_t bytesSent = FreeRTOS_send(xSocket, str, strIndex, 0);

        // Check if the data was sent successfully
        if (bytesSent > 0)
        {
            FreeRTOS_debug_printf(("Data sent: %s\n", str));
        }
        else if (bytesSent < 0)
        {
            FreeRTOS_debug_printf(("Failed to send data\n"));
            break;
        }

        // Clear the string buffer
        memset(str, '\0', sizeof(str));

        // Reset the string index
        strIndex = 0;
    }

    /**
     * The RTOS task will get here if an error is received on a read.
     * Ensure the socket has shut down before leaving the task.
     */
    if (xSocket)
        safelyShutdownSocket(xSocket);

    /* reset serial function in GRBL */
    serial_init();

    // clear task handle
    serialTaskHandle = NULL;

    // notify server socket task to delete the task
    xTaskNotifyGive(createServerSocketTaskHandle);

    // delete current task
    vTaskDelete(NULL);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    if ((heth->Instance->DMASR & ETH_DMA_FLAG_AIS) != 0)
    {
        /* clear interrupt flag */
        __HAL_ETH_DMA_CLEAR_FLAG(heth, ETH_DMA_FLAG_AIS);
        FreeRTOS_debug_printf(("ETH DMA Error: Abnormal interrupt summary\n"));
    }
}

static void prvBlinkLED(void *pvParameters)
{
    for (;;)
    {
        for (int i = 0; i < 3; i++)
        {
            // Turn on the LEDs, LD1 and LD2
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(200));

            // Turn off the LEDs, LD1 and LD2
            HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        // Delay for 1.5 seconds
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}