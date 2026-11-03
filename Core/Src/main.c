
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   美美小香腳－智慧除濕烘乾鞋架
 *          STM32 NUCLEO-G474RE / STM32CubeIDE / HAL
 *
 * ==================== 功能說明 ====================
 * SAFE      : PA12 綠燈亮，PWM LED 關閉
 * WET_WARN  : PB12 紅燈亮，等待 PC9 啟動
 * DRYING    : PA11 黃燈亮，PB2 PWM 20%
 *
 * PC6  : 模擬潮濕 / 恢復乾燥
 * PC8  : Demo 3 秒 PWM 漸滅
 * PC9  : 啟動烘乾 / 提前停止
 * PC13 : Reset，回到 SAFE
 *
 * 正常烘乾時間：2400 秒（40 分鐘）
 *
 * ==================== 技能檢核 ====================
 * 1. GPIO         : 控制 LED、讀取按鈕
 * 2. 中斷 EXTI    : PC6、PC8、PC13
 * 3. Non-blocking : HAL_GetTick() 非阻塞計時
 * 4. Debounce     : 按鈕 50 ms 去彈跳
 * 5. PWM          : TIM5_CH1，1 kHz、20%
 * 6. FSM          : SAFE、WET_WARN、DRYING
 * 7. UART         : 本版本尚未實作
 *
 * 注意：PB2 PWM 用於 LED 展示，不是實際馬達輸出。
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* =====================================================
 * 【技能檢核 3：Non-blocking】
 * 使用時間參數搭配 HAL_GetTick()，不使用 HAL_Delay()
 * ===================================================== */

#define DRYING_TIME_SEC       (2400U)
#define DEMO_REMAINING_SEC    (3U)
#define BUTTON_DEBOUNCE_MS    (50U)

/* =====================================================
 * 【技能檢核 5：PWM】
 *
 * TIM5 時脈 = 170 MHz
 * Prescaler = 169
 * Period = 999
 *
 * PWM 頻率：
 * 170 MHz / (170 * 1000) = 1000 Hz
 *
 * 占空比：
 * 200 / 1000 = 20%
 * ===================================================== */

#define TIM5_PRESCALER        (169U)
#define TIM5_PERIOD           (999U)

#define PWM_LED_ON_COMPARE    (200U)
#define PWM_LED_OFF_COMPARE   (0U)

#define DEMO_FADE_TIME_MS     (3000U)

/* =====================================================
 * 【技能檢核 2：中斷 EXTI】
 * 使用事件旗標記錄按鈕中斷
 * ===================================================== */

#define EVENT_HUMIDITY        (0x01U)
#define EVENT_DEMO            (0x02U)
#define EVENT_RESET           (0x04U)

/* =====================================================
 * 【技能檢核 6：FSM 有限狀態機】
 *
 * 使用 enum 定義三種系統狀態
 *
 * STATE_SAFE     : 安全待機，綠燈
 * STATE_WET_WARN : 潮濕警示，紅燈
 * STATE_DRYING   : 烘乾中，黃燈
 * ===================================================== */

typedef enum
{
    STATE_SAFE = 0,
    STATE_DRYING,
    STATE_WET_WARN
} SystemState_t;

/* =====================================================
 * 【技能檢核 4：Debounce 去彈跳】
 *
 * 記錄按鈕原始狀態、穩定狀態與變化時間
 * ===================================================== */

typedef struct
{
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t last_change_tick;
} ButtonDebounce_t;

/* Private variables ---------------------------------- */

TIM_HandleTypeDef htim5;

/* 【技能檢核 6：FSM】初始狀態為 SAFE */
static SystemState_t g_system_state = STATE_SAFE;

static uint32_t g_remaining_seconds = 0U;
static uint32_t g_last_second_tick = 0U;

static uint8_t g_demo_fade_active = 0U;
static uint32_t g_demo_fade_start_tick = 0U;

/* 【技能檢核 2：中斷】volatile 事件旗標
 * 中斷與主迴圈共同存取，因此使用 volatile
 */
static volatile uint8_t g_exti_pending = 0U;

static volatile uint32_t g_pc6_edge_tick = 0U;
static volatile uint32_t g_pc8_edge_tick = 0U;
static volatile uint32_t g_pc13_edge_tick = 0U;

/* 【技能檢核 4：Debounce】PC9 按鈕去彈跳資料 */
static ButtonDebounce_t g_pc9_button;

/* Private function prototypes ------------------------ */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);

static void SetSystemState(SystemState_t new_state);

static void HandleHumidityEvent(void);
static void HandleDemoEvent(void);
static void HandleManualButton(void);

static uint8_t ProcessExtiDebounce(void);

static void ButtonDebounce_Init(
    ButtonDebounce_t *button,
    GPIO_TypeDef *port,
    uint16_t pin
);

static uint8_t ButtonDebounce_GetPressEvent(
    ButtonDebounce_t *button,
    GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t now
);

static void UpdateDryingTimer(void);

void Error_Handler(void);

/* =====================================================
 * MAIN 主程式
 * ===================================================== */

int main(void)
{
    uint8_t exti_events;
    uint32_t now;

    HAL_Init();

    SystemClock_Config();

    /* 【技能檢核 1：GPIO】初始化 LED 與按鈕腳位 */
    MX_GPIO_Init();

    /* 【技能檢核 5：PWM】初始化 TIM5 */
    MX_TIM5_Init();

    /* 【技能檢核 5：PWM】
     * 先將比較值設為 0，避免開機瞬間亮起
     */
    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        PWM_LED_OFF_COMPARE
    );

    if (HAL_TIM_PWM_Start(
            &htim5,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        Error_Handler();
    }

    /* 【技能檢核 4：Debounce】
     * 初始化 PC9 按鈕去彈跳
     */
    ButtonDebounce_Init(
        &g_pc9_button,
        GPIOC,
        GPIO_PIN_9
    );

    /* 【技能檢核 6：FSM】
     * 開機直接進入 SAFE，PA12 綠燈亮
     */
    SetSystemState(STATE_SAFE);

    while (1)
    {
        /* 【技能檢核 3：Non-blocking】
         * 取得系統時間，不使用 HAL_Delay()
         */
        now = HAL_GetTick();

        /* 【技能檢核 2、4：中斷 + Debounce】
         * 處理 EXTI 事件並確認按鈕穩定 50 ms
         */
        exti_events = ProcessExtiDebounce();

        /* PC13 Reset 優先權最高 */
        if ((exti_events & EVENT_RESET) != 0U)
        {
            SetSystemState(STATE_SAFE);
        }
        else
        {
            /* PC6：模擬濕度變化 */
            if ((exti_events & EVENT_HUMIDITY) != 0U)
            {
                HandleHumidityEvent();
            }

            /* PC8：Demo 縮時 */
            if ((exti_events & EVENT_DEMO) != 0U)
            {
                HandleDemoEvent();
            }
        }

        /* 【技能檢核 1、4：GPIO + Debounce】
         * PC9 使用非阻塞輪詢，穩定按下才觸發
         */
        if (ButtonDebounce_GetPressEvent(
                &g_pc9_button,
                GPIOC,
                GPIO_PIN_9,
                now
            ) != 0U)
        {
            HandleManualButton();
        }

        /* 【技能檢核 3：Non-blocking】
         * 更新烘乾倒數與 Demo 漸滅
         */
        UpdateDryingTimer();

        /* 進入 Cortex-M Sleep，等待中斷喚醒。
         * SysTick 通常每 1 ms 喚醒一次。
         */
        __WFI();
    }
}

/* =====================================================
 * 【技能檢核 6：FSM】
 *
 * 有限狀態機：狀態切換與輸出控制
 *
 * SAFE     → 綠燈
 * WET_WARN → 紅燈
 * DRYING   → 黃燈 + PWM
 *
 * 透過 switch-case 判斷目前狀態
 * ===================================================== */

static void SetSystemState(SystemState_t new_state)
{
    g_system_state = new_state;

    /* 【技能檢核 1：GPIO】
     * 切換狀態前先關閉全部 LED
     */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_11 | GPIO_PIN_12,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_12,
        GPIO_PIN_RESET
    );

    /* 【技能檢核 5：PWM】
     * 切換狀態時先關閉 PWM
     */
    __HAL_TIM_SET_COMPARE(
        &htim5,
        TIM_CHANNEL_1,
        PWM_LED_OFF_COMPARE
    );

    g_remaining_seconds = 0U;
    g_demo_fade_active = 0U;

    /* 【技能檢核 6：FSM】
     * 根據狀態決定 LED 與 PWM 輸出
     */
    switch (new_state)
    {
        case STATE_SAFE:

            /* 【技能檢核 1：GPIO】
             * PA12 設為 HIGH，綠燈亮
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_12,
                GPIO_PIN_SET
            );

            break;

        case STATE_WET_WARN:

            /* 【技能檢核 1：GPIO】
             * PB12 設為 HIGH，紅燈亮
             */
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_12,
                GPIO_PIN_SET
            );

            break;

        case STATE_DRYING:

            /* 【技能檢核 1：GPIO】
             * PA11 黃燈亮
             */
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_11,
                GPIO_PIN_SET
            );

            /* 【技能檢核 5：PWM】
             * TIM5_CH1 設定為 20% 占空比
             */
            __HAL_TIM_SET_COMPARE(
                &htim5,
                TIM_CHANNEL_1,
                PWM_LED_ON_COMPARE
            );

            /* 【技能檢核 3：Non-blocking】
             * 記錄烘乾開始時間
             */
            g_remaining_seconds = DRYING_TIME_SEC;
            g_last_second_tick = HAL_GetTick();

            break;

        default:

            g_system_state = STATE_SAFE;

            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_12,
                GPIO_PIN_SET
            );

            break;
    }
}

/* =====================================================
 * 【技能檢核 6：FSM】
 *
 * PC6 模擬濕度變化
 *
 * SAFE → WET_WARN
 * WET_WARN → SAFE
 *
 * DRYING 期間忽略 PC6
 * ===================================================== */

static void HandleHumidityEvent(void)
{
    if (g_system_state == STATE_SAFE)
    {
        SetSystemState(STATE_WET_WARN);
    }
    else if (g_system_state == STATE_WET_WARN)
    {
        SetSystemState(STATE_SAFE);
    }
}

/* =====================================================
 * 【技能檢核 3、5：Non-blocking + PWM】
 *
 * PC8 Demo 模式：
 * 將烘乾時間縮短為 3 秒
 * PWM 從 20% 逐漸下降到 0%
 * ===================================================== */

static void HandleDemoEvent(void)
{
    if (g_system_state == STATE_DRYING)
    {
        g_demo_fade_active = 1U;

        /* 記錄 Demo 開始時間 */
        g_demo_fade_start_tick = HAL_GetTick();

        g_remaining_seconds = DEMO_REMAINING_SEC;
        g_last_second_tick = g_demo_fade_start_tick;

        /* PWM 恢復到 20% 後開始漸滅 */
        __HAL_TIM_SET_COMPARE(
            &htim5,
            TIM_CHANNEL_1,
            PWM_LED_ON_COMPARE
        );
    }
}

/* =====================================================
 * 【技能檢核 6：FSM】
 *
 * PC9 手動控制
 *
 * WET_WARN → DRYING
 * DRYING → SAFE
 * SAFE → 不動作
 * ===================================================== */

static void HandleManualButton(void)
{
    if (g_system_state == STATE_WET_WARN)
    {
        SetSystemState(STATE_DRYING);
    }
    else if (g_system_state == STATE_DRYING)
    {
        SetSystemState(STATE_SAFE);
    }
}

/* =====================================================
 * 【技能檢核 3：Non-blocking】
 *
 * 非阻塞式烘乾計時
 *
 * 使用 HAL_GetTick() 計算經過時間，
 * 不使用 HAL_Delay()，因此按鈕仍可操作。
 *
 * 【技能檢核 5：PWM】
 * Demo 模式時逐漸降低 PWM 占空比。
 * ===================================================== */

static void UpdateDryingTimer(void)
{
    uint32_t now;
    uint32_t elapsed_seconds;

    if (g_system_state != STATE_DRYING)
    {
        return;
    }

    now = HAL_GetTick();

    /* 【技能檢核 3、5：Non-blocking + PWM】
     * Demo 模式：3 秒內 PWM 線性漸滅
     */
    if (g_demo_fade_active != 0U)
    {
        uint32_t fade_elapsed =
            (uint32_t)(now - g_demo_fade_start_tick);

        uint32_t compare;

        if (fade_elapsed >= DEMO_FADE_TIME_MS)
        {
            SetSystemState(STATE_SAFE);
            return;
        }

        /* PWM 比較值由 200 線性降到 0 */
        compare =
            ((uint32_t)PWM_LED_ON_COMPARE *
            (DEMO_FADE_TIME_MS - fade_elapsed))
            / DEMO_FADE_TIME_MS;

        __HAL_TIM_SET_COMPARE(
            &htim5,
            TIM_CHANNEL_1,
            compare
        );

        return;
    }

    /* 【技能檢核 3：Non-blocking】
     * 以 HAL_GetTick() 計算經過的整秒數
     */
    elapsed_seconds =
        (uint32_t)(now - g_last_second_tick) / 1000U;

    if (elapsed_seconds == 0U)
    {
        return;
    }

    g_last_second_tick += elapsed_seconds * 1000U;

    if (elapsed_seconds >= g_remaining_seconds)
    {
        /* 40 分鐘結束，回到 SAFE */
        SetSystemState(STATE_SAFE);
    }
    else
    {
        g_remaining_seconds -= elapsed_seconds;
    }
}

/* =====================================================
 * 【技能檢核 4：Debounce 去彈跳】
 *
 * PC9 按鈕初始化
 * 記錄目前按鈕電位及時間
 * ===================================================== */

static void ButtonDebounce_Init(
    ButtonDebounce_t *button,
    GPIO_TypeDef *port,
    uint16_t pin
)
{
    GPIO_PinState initial_state =
        HAL_GPIO_ReadPin(port, pin);

    button->raw_state = initial_state;
    button->stable_state = initial_state;
    button->last_change_tick = HAL_GetTick();
}

/* =====================================================
 * 【技能檢核 1、4：GPIO + Debounce】
 *
 * PC9 使用 Polling 讀取 GPIO
 *
 * 只有按鈕狀態穩定 50 ms 後，
 * 才判定為有效按下。
 *
 * 不使用 HAL_Delay()，屬於非阻塞式去彈跳。
 * ===================================================== */

static uint8_t ButtonDebounce_GetPressEvent(
    ButtonDebounce_t *button,
    GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t now
)
{
    /* 【技能檢核 1：GPIO】
     * 讀取 PC9 按鈕電位
     */
    GPIO_PinState reading =
        HAL_GPIO_ReadPin(port, pin);

    if (reading != button->raw_state)
    {
        button->raw_state = reading;
        button->last_change_tick = now;
    }

    /* 【技能檢核 4：Debounce】
     * 按鈕必須穩定至少 50 ms
     */
    if (((uint32_t)(now - button->last_change_tick)
         >= BUTTON_DEBOUNCE_MS) &&
        (button->stable_state != button->raw_state))
    {
        button->stable_state = button->raw_state;

        /* PC9 上拉，按下為 LOW */
        if (button->stable_state == GPIO_PIN_RESET)
        {
            return 1U;
        }
    }

    return 0U;
}

/* =====================================================
 * 【技能檢核 2、4：中斷 + Debounce】
 *
 * EXTI 去彈跳處理
 *
 * 中斷發生後先記錄時間，
 * 等待 50 ms，再確認 GPIO 是否仍為按下狀態。
 *
 * 這樣可以避免按鈕機械彈跳造成重複觸發。
 * ===================================================== */

static uint8_t ProcessExtiDebounce(void)
{
    uint8_t ready_events = 0U;
    uint32_t now;

    uint32_t primask = __get_PRIMASK();

    /* 保護共享事件旗標，避免 ISR 同時修改 */
    __disable_irq();

    now = HAL_GetTick();

    /* 【技能檢核 4：Debounce】
     * PC6：等待 50 ms 後確認按下
     */
    if (((g_exti_pending & EVENT_HUMIDITY) != 0U) &&
        ((uint32_t)(now - g_pc6_edge_tick)
         >= BUTTON_DEBOUNCE_MS))
    {
        if (HAL_GPIO_ReadPin(
                GPIOC,
                GPIO_PIN_6
            ) == GPIO_PIN_RESET)
        {
            ready_events |= EVENT_HUMIDITY;
        }

        g_exti_pending &=
            (uint8_t)(~EVENT_HUMIDITY);
    }

    /* PC8：等待 50 ms 後確認按下 */
    if (((g_exti_pending & EVENT_DEMO) != 0U) &&
        ((uint32_t)(now - g_pc8_edge_tick)
         >= BUTTON_DEBOUNCE_MS))
    {
        if (HAL_GPIO_ReadPin(
                GPIOC,
                GPIO_PIN_8
            ) == GPIO_PIN_RESET)
        {
            ready_events |= EVENT_DEMO;
        }

        g_exti_pending &=
            (uint8_t)(~EVENT_DEMO);
    }

    /* PC13：板載 B1，按下為 HIGH */
    if (((g_exti_pending & EVENT_RESET) != 0U) &&
        ((uint32_t)(now - g_pc13_edge_tick)
         >= BUTTON_DEBOUNCE_MS))
    {
        if (HAL_GPIO_ReadPin(
                GPIOC,
                GPIO_PIN_13
            ) == GPIO_PIN_SET)
        {
            ready_events |= EVENT_RESET;
        }

        g_exti_pending &=
            (uint8_t)(~EVENT_RESET);
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return ready_events;
}

/* =====================================================
 * 【技能檢核 2：中斷 EXTI】
 *
 * HAL_GPIO_EXTI_Callback()
 *
 * PC6、PC8、PC13 發生外部中斷時，
 * 只記錄事件與時間，不在 ISR 中執行耗時工作。
 *
 * 事件會交給 main() 主迴圈處理。
 * ===================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_6)
    {
        if ((g_exti_pending & EVENT_HUMIDITY) == 0U)
        {
            g_pc6_edge_tick = now;
            g_exti_pending |= EVENT_HUMIDITY;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_8)
    {
        if ((g_exti_pending & EVENT_DEMO) == 0U)
        {
            g_pc8_edge_tick = now;
            g_exti_pending |= EVENT_DEMO;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_13)
    {
        if ((g_exti_pending & EVENT_RESET) == 0U)
        {
            g_pc13_edge_tick = now;
            g_exti_pending |= EVENT_RESET;
        }
    }
}

/* =====================================================
 * 【技能檢核 5：PWM】
 *
 * TIM5_CH1 PWM 初始化
 *
 * PB2 輸出 PWM
 * 頻率：1 kHz
 * 占空比：20%
 *
 * Prescaler = 169
 * Period = 999
 * Compare = 200
 * ===================================================== */

static void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 開啟 TIM5 與 GPIOB 時脈 */
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    htim5.Instance = TIM5;

    htim5.Init.Prescaler = TIM5_PRESCALER;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = TIM5_PERIOD;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    htim5.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_TIM_ConfigClockSource(
            &htim5,
            &sClockSourceConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim5,
            &sMasterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_LED_OFF_COMPARE;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
            &htim5,
            &sConfigOC,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        Error_Handler();
    }

    /* 【技能檢核 1、5：GPIO + PWM】
     * PB2 設為 TIM5_CH1 複用功能 AF2
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* =====================================================
 * 【技能檢核 1：GPIO】
 *
 * GPIO 腳位初始化
 *
 * PA12：綠燈
 * PB12：紅燈
 * PA11：黃燈
 * PB2 ：PWM LED
 *
 * PC6 ：濕度按鈕
 * PC8 ：Demo 按鈕
 * PC9 ：啟動 / 停止
 * PC13：Reset
 * ===================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 開啟 GPIO 時脈 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 【技能檢核 1：GPIO】
     * 設定 LED 初始輸出 LOW
     */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_11 | GPIO_PIN_12,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_12,
        GPIO_PIN_RESET
    );

    /* PA11 黃燈、PA12 綠燈 */
    GPIO_InitStruct.Pin =
        GPIO_PIN_11 | GPIO_PIN_12;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB12 紅燈 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 【技能檢核 2：中斷 EXTI】
     * PC6、PC8：下降沿觸發中斷
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_6 | GPIO_PIN_8;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PC13：板載 B1，上升沿觸發 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_RISING;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 【技能檢核 1：GPIO】
     * PC9 使用 GPIO Input 輪詢
     */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 【技能檢核 2：中斷 EXTI】
     * 啟用 EXTI9_5 與 EXTI15_10 中斷
     */
    HAL_NVIC_SetPriority(
        EXTI9_5_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(
        EXTI15_10_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* =====================================================
 * 系統時脈設定：170 MHz
 * ===================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(
            PWR_REGULATOR_VOLTAGE_SCALE1_BOOST
        ) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =====================================================
 * 錯誤處理
 * ===================================================== */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* 初始化失敗時停在這裡 */
    }
}

/* =====================================================
 * 【技能檢核 7：UART】
 *
 * 本版本尚未實作 UART。
 * 程式目前沒有 HAL_UART_Transmit()、
 * HAL_UART_Receive() 或 UART 初始化。
 *
 * 若要完成 UART 檢核，需要另外加入
 * UART 初始化與資料傳送 / 接收功能。
 * ===================================================== */

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
