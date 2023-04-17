# STM32-LPS22HB
STM32 LPS22HB Pressure Sensor
## Usage

```c
/* USER CODE BEGIN Includes */
#include "lps22hb.h"
/* USER CODE END Includes */
```

```c
/* USER CODE BEGIN PV */
lsm6dsm_t lsm6dsm;
/* USER CODE END PV */
```
In the main function:
```c
/* LPS22HB INITIALIZATION */
volatile uin32_t lps22hb_timer = 0;
lps22hb_init(&lps22hb, LPS22HB_ODR_75_Hz, 1, 0);
```

In the while loop:
```c
if (HAL_GetTick() - lps22hb_timer >= lps22hb.compute_time) {
    lps22hb_readTemperature(&lps22hb);
    lps22hb_readPressure(&lps22hb);
    lps22hb_readAltitude(&lps22hb);

    lps22hb_timer = HAL_GetTick();
}
```

## TODO
```
[x] Add altitude calculation
[ ] Add i2c and 4-Wire SPI Support
```
