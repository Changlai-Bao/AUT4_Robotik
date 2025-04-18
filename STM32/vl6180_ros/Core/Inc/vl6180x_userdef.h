#include <string.h>

#define EXPANDER_I2C_ADDRESS  (0x42*2)
#define _DISP_SEL             (1<<11)
#define _CHIPEN               (1<<12)

#define GPSR                  0x12
#define GPDR                  0x14

#define def_i2c_time_out 100

extern I2C_HandleTypeDef hi2c1;

/**
 * Definition of OnErrLog is mandatory for the VL6180X1 API
 */
int _err=0;
void OnErrLog(void)
{
	_err++;
}

/**
 * Definition of VL6180x_I2CRead is mandatory for the VL6180X1 API
 */
int VL6180x_I2CRead(VL6180xDev_t addr, uint8_t  *buff, uint8_t len)
{
	return HAL_I2C_Master_Receive(&hi2c1,  addr, buff, len , def_i2c_time_out);
}

/**
 * Definition of VL6180x_I2CWrite is mandatory for the VL6180X1 API
 */
int VL6180x_I2CWrite(VL6180xDev_t addr, uint8_t  *buff, uint8_t len)
{
	return HAL_I2C_Master_Transmit(&hi2c1,  addr, buff, len , def_i2c_time_out);
}

int Expander_Rd(int  index,  uint8_t *data, int n_data)
{
	int status = HAL_OK;
	uint8_t RegAddr;
	RegAddr=index;

	do{
		status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, &RegAddr, 1, 100);
		if( status )
			break;
		status =HAL_I2C_Master_Receive(&hi2c1, EXPANDER_I2C_ADDRESS, data, n_data, n_data*100);
	}while(0);
	return status;
}

int Expander_WR( int index,  uint8_t *data, int n_data)
{
	int status;
	uint8_t RegAddr[0x10];
	RegAddr[0]=index;
	memcpy(RegAddr+1, data, n_data);
	status=HAL_I2C_Master_Transmit(&hi2c1, EXPANDER_I2C_ADDRESS, RegAddr, n_data+1, 100);
	return status;
}

static void Expander_GPIO_Init()
{
    uint16_t PadDir;

    GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    // PB0 = INT
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // expander config
    PadDir=~_DISP_SEL;
    Expander_WR(GPDR, (uint8_t*)&PadDir, 2);
}

/**
 * @param state 0=off, 1=on
 */
static void Expander_SetChipEn(int state)
{
	int mask = _CHIPEN;
	uint16_t padVal = 0;
	if( state)
		padVal|=mask ;
	else
		padVal&=~mask;

	Expander_WR(GPSR, (uint8_t*)&padVal,2);
}
