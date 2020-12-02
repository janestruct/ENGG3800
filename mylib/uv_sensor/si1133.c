/*
 * si1133.c
 *
 *  Created on: Oct 1, 2019
 *      Author: fadzziiieee
 */

#include "stdio.h"
#include "si1133.h"



/**
 * @brief I2C Communication
 * @param
 * @retval None
 */
void SI1133_init(void){

		uint8_t data = 0;

		SI1133_reset();

		HAL_Delay(25);

		SI1133_paramSet(SI1133_PARAM_CH_LIST,1);
		SI1133_paramRead(SI1133_PARAM_CH_LIST,&data);

		SI1133_paramSet(SI1133_PARAM_ADCCONFIG0,0x18 );
		SI1133_paramRead(SI1133_PARAM_ADCCONFIG0,&data);

	    SI1133_paramSet(SI1133_PARAM_ADCSENS0,0x05);
	    SI1133_paramRead(SI1133_PARAM_ADCSENS0,&data);

		SI1133_paramSet(SI1133_PARAM_ADCPOST0,0x00);
		SI1133_paramRead(SI1133_PARAM_ADCPOST0,&data);

		SI1133_paramSet(SI1133_PARAM_MEASCONFIG0,0);
		SI1133_paramRead(SI1133_PARAM_MEASCONFIG0,&data);

		SI1133_registerWrite(SI1133_REG_IRQ_ENABLE,0x0F);

}

/**
 * @brief Reading Register
 * @param
 * @retval None
 */
uint8_t SI1133_registerRead(unsigned char reg, uint8_t *data){

	uint8_t Buffer[1];

	if(HAL_I2C_Mem_Read(&hi2c3,SI1133_I2C_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT, Buffer, 1, 100) != HAL_OK){

		return SI1133_ERROR_I2C_TRANSACTION_FAILED;
	}

	*data = Buffer[0];

	return SI1133_OK;
}

/**
 * @brief Writing Register
 * @param
 * @retval None
 */
uint8_t SI1133_registerWrite(unsigned char reg, uint8_t data){

	uint8_t Buffer[1];

	Buffer[0] = data;

		if(HAL_I2C_Mem_Write(&hi2c3,SI1133_I2C_ADDRESS,reg,I2C_MEMADD_SIZE_8BIT, Buffer , 1, 10000) != HAL_OK){

			return SI1133_ERROR_I2C_TRANSACTION_FAILED;
		}

	return SI1133_OK;
}
/**
 * @brief Read Parameters
 * @param None
 * @retval None
 */
uint8_t SI1133_paramRead(unsigned char address, uint8_t* value){

	uint8_t Buffer[1];
	uint8_t reg = 0;
	uint8_t check_response = 0;

	reg = 0b01000000; //0b10000000
	reg |= address;

	Buffer[0] = reg;

	if(HAL_I2C_Mem_Write(&hi2c3,SI1133_I2C_ADDRESS,SI1133_REG_COMMAND,
		 					 				I2C_MEMADD_SIZE_8BIT, Buffer , 1, 100) != HAL_OK){

		return SI1133_ERROR_I2C_TRANSACTION_FAILED;
	}

	/*Check if command sent is successful*/
	SI1133_registerRead(SI1133_REG_RESPONSE0, &check_response);
		if((check_response & 16) == 1){

		 					return SI1133_ERROR_I2C_TRANSACTION_FAILED;
		 				}
		if(HAL_I2C_Mem_Read(&hi2c3,SI1133_I2C_ADDRESS,SI1133_REG_RESPONSE1,
				I2C_MEMADD_SIZE_8BIT, Buffer, 1, 100) != HAL_OK){
			return SI1133_ERROR_I2C_TRANSACTION_FAILED;
		}

		*value = Buffer[0];
		return SI1133_OK;

}
/**
 * @brief Write Parameters
 * @param None
 * @retval None
 */
uint8_t SI1133_paramSet(unsigned char address, unsigned char value){

	 	uint8_t Buffer[1];
	 	uint8_t reg = 0;
	 	uint8_t check_response = 0;

	 	Buffer[0] = value;

	 	if(HAL_I2C_IsDeviceReady(&hi2c3,SI1133_I2C_ADDRESS,5,100) == HAL_OK){

	 		if(HAL_I2C_Mem_Write(&hi2c3,SI1133_I2C_ADDRESS,SI1133_REG_HOSTIN0,
	 				I2C_MEMADD_SIZE_8BIT, Buffer , 1, 100) != HAL_OK){

	 			return SI1133_ERROR_I2C_TRANSACTION_FAILED;

	 		}

	 		reg = 0b10000000;
	 		reg |= address;

	 		Buffer[0] = reg;

	 				if(HAL_I2C_Mem_Write(&hi2c3,SI1133_I2C_ADDRESS,SI1133_REG_COMMAND,
	 					 				I2C_MEMADD_SIZE_8BIT, Buffer , 1, 100) != HAL_OK){

	 					return SI1133_ERROR_I2C_TRANSACTION_FAILED;

	 				}
	 				/*Check if command sent is successful*/
	 		SI1133_registerRead(SI1133_REG_RESPONSE0, &check_response);

	 			if((check_response & 16) == 1){

	 					return SI1133_ERROR_I2C_TRANSACTION_FAILED;
	 				}
	 	}

	 	return SI1133_OK;
}

/**
 * @brief Sending Command
 * @param None
 * @retval None
 */
uint8_t SI1133_sendCmd(unsigned char command){

	uint8_t Buffer[1];
	uint8_t check_response = 0;

	Buffer[0] = command;

	/*Send Command*/
	if(HAL_I2C_Mem_Write(&hi2c3,SI1133_I2C_ADDRESS,SI1133_REG_COMMAND,
		 					 				I2C_MEMADD_SIZE_8BIT, Buffer , 1, 100) != HAL_OK){

		 					return SI1133_ERROR_I2C_TRANSACTION_FAILED;

		 				}

	/*Check if command sent is successful*/

	SI1133_registerRead(SI1133_REG_RESPONSE0, &check_response);

		if((check_response & 16) == 1){

			return SI1133_ERROR_I2C_TRANSACTION_FAILED;
		}

		return SI1133_OK;

}
/**
 * @brief Reset all register value to default.
 * @param None
 * @retval None
 */
uint8_t SI1133_reset(void){

	uint8_t retval;

	HAL_Delay(30);

	retval = SI1133_registerWrite(SI1133_REG_COMMAND,SI1133_CMD_RESET);

	return retval;
}
/**
 * @brief Reset Command Counter value to 0
 * @param
 * @retval None
 */
uint8_t SI1133_resetCmdCtr(void){

	return SI1133_sendCmd(SI1133_CMD_RESET_CMD_CTR);
}
/**
 * @brief Measurement Start
 * @param None
 * @retval None
 */
uint8_t SI1133_measurementStart(void){

	return SI1133_sendCmd(SI1133_CMD_START);

}
/**
 * @brief Force Measurement
 * @param None
 * @retval None
 */
uint8_t SI1133_measurementForce(void){

	return SI1133_sendCmd(SI1133_CMD_FORCE_CH);
}
/**
 * @brief Pause Measurement
 * @param None
 * @retval None
 */
uint8_t SI1133_measurementPause(void){

	return SI1133_sendCmd(SI1133_CMD_PAUSE_CH);
}
/**
 * @brief Get IRQ Status
 * @param None
 * @retval None
 */
uint8_t SI1133_getIrqStatus( uint8_t *irqStatus )
 {

    uint8_t retval;

    /* Read the IRQ status register */
    retval = SI1133_registerRead( SI1133_REG_IRQ_STATUS, irqStatus );

    return retval;

 }
int getUV(void){

	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;



	SI1133_measurementForce();
	SI1133_registerRead(SI1133_REG_HOSTOUT0,&temp1);
	SI1133_registerRead(SI1133_REG_HOSTOUT1,&temp2);
	SI1133_registerRead(SI1133_REG_HOSTOUT1,&temp3);

	int16_t input;

	input =  (int8_t) temp1<<8 |(int8_t) temp2 ;

	  int uvi = 0.0187 * ((0.00391*(input*input)) + input);

	if((input < 0) & (uvi < 0) ){

		}
		if((input >= 0) & (uvi >= 0 )){

			printf("Raw data : %d \n\r",input);
			printf("UV Index : %d \n\r",uvi);

			return uvi;
	}

		return uvi;
//	printf("IN0 : %d \n\r",temp1);
//	printf("IN1 : %d \n\r",temp2);
//	printf("Raw data : %d \n\r",input);
//	printf("UV Index : %f \n\r",uvi);
}
