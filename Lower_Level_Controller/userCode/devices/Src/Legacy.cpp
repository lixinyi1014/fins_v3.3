//
// Created by admin on 2023/11/28.
//

#include "Legacy.h"


#include "EventLog.h"

volatile float vccMoni = 0;
volatile float vccBat = 0;

namespace
{
float g_vin = 0.0f;
float g_vdda = 3.3f;
float g_vin_idle = 0.0f;       // 推进器空闲时的输入电压（慢速平均），作为跌落判据的基准
uint32_t g_last_sag_ms = 0;
bool g_power_ready = false;

bool ReadAdc(ADC_HandleTypeDef *hadc, uint32_t *value)
{
    if (HAL_ADC_Start(hadc) != HAL_OK)
        return false;
    const bool ok = HAL_ADC_PollForConversion(hadc, 1) == HAL_OK;
    if (ok)
        *value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return ok;
}
}

void Power_Sample(bool thrusters_active)
{
    if (!g_power_ready)
    {
        // VREFINT 要求采样时间 >= 10 us，分压电阻 200k 也需要长采样；原配置 3 个周期读数不准
        ADC_ChannelConfTypeDef config = {};
        config.Rank = 1;
        config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        config.Channel = ADC_CHANNEL_VREFINT;
        HAL_ADC_ConfigChannel(&hadc1, &config);
        config.Channel = ADC_CHANNEL_8;
        HAL_ADC_ConfigChannel(&hadc3, &config);
        g_power_ready = true;
    }
    uint32_t vref_raw = 0, vin_raw = 0;
    if (!ReadAdc(&hadc1, &vref_raw) || vref_raw < 1000U || !ReadAdc(&hadc3, &vin_raw))
        return;
    g_vdda = 1.21f * 4095.0f / static_cast<float>(vref_raw);        // VREFINT 典型 1.21 V
    g_vin = static_cast<float>(vin_raw) * g_vdda / 4095.0f * (222.0f / 22.0f);

    if (!thrusters_active)
        g_vin_idle = (g_vin_idle <= 0.0f) ? g_vin : g_vin_idle + 0.01f * (g_vin - g_vin_idle);

    const uint32_t now = HAL_GetTick();
    // 10-19：硬件修好后正常带载也会掉到空闲的 77%～86%，原来 85% / 每 0.5 s 一条会刷满事件记录（一次 14 分钟测试 1500 条），
    // 把真正有用的事件挤掉。改为只报明显跌落（低于 60%），最多 5 s 一条；电压曲线照常在 CSV 里
    if (now - g_last_sag_ms < 5000U)
        return;
    if (g_vin_idle > 5.0f && g_vin < 0.6f * g_vin_idle)
    {
        g_last_sag_ms = now;
        Event_PostText(EV_WARN_POWER_SAG, static_cast<int32_t>(g_vin * 1000.0f), "VIN");
    }
    else if (g_vdda < 3.1f)
    {
        g_last_sag_ms = now;
        Event_PostText(EV_WARN_POWER_SAG, static_cast<int32_t>(g_vdda * 1000.0f), "3V3");
    }
}

float Power_Vin(void) { return g_vin; }
float Power_Vdda(void) { return g_vdda; }

void bsp_ADC_vccMoni(){
    static uint8_t calibrationCplt = 0;
    static uint32_t calibrationCnt = 0;
    static uint32_t vrefSum = 0,vrefValue = 0;
    if(calibrationCplt == 0){
        if (calibrationCnt == 0){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else if(calibrationCnt < 200){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else{
            calibrationCplt = 1;
            vrefValue = vrefSum/200.0f;
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 2);
            vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

        }

    }else{
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 2);
        vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

    }
    vccBat = vccMoni/22.0f*222.0f;
}

/**
 * @brief // Using the UART redirection function, either a USB virtual COM port or a hardware UART can be selected; the corresponding driver must be installed for operation. //利用串口重定向函数，可选usb虚拟串口或硬件串口，使用需安装相应驱动
 * @param fmt
 * @param ...
 */
void usart_printf(const char *fmt,...){

        static uint8_t tx_buf[256] = {0};//TODO 爆栈？
        static va_list ap;
        static uint16_t len;

        va_start(ap,fmt);

        /*len = vsprintf((char*)tx_buf,fmt,ap);*/

        va_end(ap);

        //HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
//    CDC_Transmit_FS(tx_buf,len);

    }

    flash_data_t flashData;
    // 返回 true=写入并读回一致。擦 128 KB 扇区约 1~2 s，期间整个单片机停住（代码也在 Flash 里），
    // 只能在推进器不转时调用。原来 SectorError 传空指针（HAL 会往 0 地址写）、终点按结构体指针相加越界，一并改了
    bool bsp_flash_write(flash_data_t *_flashData){
        HAL_FLASH_Unlock();
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInit.Sector = FLASH_SECTOR_9;
        eraseInit.NbSectors = 1;
        eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        uint32_t sector_error = 0;
        bool ok = HAL_FLASHEx_Erase(&eraseInit, &sector_error) == HAL_OK;
        uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;
        const uint32_t *words = reinterpret_cast<const uint32_t *>(_flashData);
        for (const uint32_t *ptr = words; ok && ptr < words + sizeof(flash_data_t) / 4; ptr++){
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_ptr, *ptr) == HAL_OK;
            flash_ptr += 4;
        }
        HAL_FLASH_Lock();
        return ok && memcmp(reinterpret_cast<const void *>(FLASH_SECTOR_9_ADDRESS), _flashData, sizeof(flash_data_t)) == 0;
    }
void bsp_flash_read(flash_data_t *_flashData){
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;
    memcpy(_flashData,(uint32_t*)FLASH_SECTOR_9_ADDRESS,sizeof(flash_data_t)); // 原来写成 sizeof/4，只读了 4 个字节
}

