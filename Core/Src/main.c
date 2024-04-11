/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>
//#include "mbed.h"
//#include "wifi.h"
//#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union {
    struct {
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    } bit;
    uint8_t byte;
}_uFlag;

typedef union {
    uint8_t     u8[4];
    int8_t      i8[4];
    uint16_t    u16[2];
    int16_t     i16[2];
    uint32_t    u32;
    int32_t     i32;
    float       f;
}_work;

typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

typedef struct{
    //uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart;      //!< Indice para saber en que parte del buffer circular arranca el ID
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
}_sDato ;

typedef enum{
    ACK=0x0D,
    GETALIVE=0xF0,
    STARTCONFIG=0xEE,
    FIRMWARE=0xF1,
    LEDS=0x10,
    PULSADORES=0x12,
    SERVO = 0xA2,
    ULTRASONIDO = 0xA3,
    SEGUIDORES_LINEA = 0xA0,
    TEST_MOTORES = 0xA1,
    VELOCIDAD_MOTORES = 0xA4,
    SETMINMAXSERVO = 0xA5,
    SETSENSORSBLACK = 0xA6,
    SETSENSORSWHITE = 0xA7,
    GETSERVOANGLE = 0xA8,
    OTHERS
}_eID;

//wifiData myWifiData;

typedef enum {
    EV_PRESSED=0,
    EV_NOT_PRESSED=1,
    EV_NONE=2
}_eButtonEvent;

typedef enum {
        UP,
        DOWN,
        FALLING,
        RISING
}_eButtonsStatus;

typedef struct {
    _eButtonsStatus    	estado;
    _eButtonEvent       event;
    uint32_t            timePush;
    uint32_t            timeDiff;
} _sButton;

typedef enum {
    ONESECOND = 0,
    THREESECONDS = 1
}_eLedStatus;

typedef enum {
    WORKING = 1,
    RESTING = 0
}_eCarStatus;

typedef enum {
    IDLE = 0,
    MODE1 = 1,
    MODE2 = 2,
    MODE3 = 3
}_eCarMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define THREESECONDSINTERVAL        29
#define ONESECONDINTERVAL           9

#define NUMBUTTONS                  1

#define RINGBUFFLENGTH              256
#define POSID                       4
#define POSDATA                     5

#define	LEDMODE                     myFlags0.bit.b0
#define CARSTATUS                  	myFlags0.bit.b1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t		mode;
uint8_t		time100ms, HBintervalWidth;
uint32_t 	mask;

uint8_t 	posicionComand;
_eProtocolo estadoProtocolo;
_sDato 		datosComSerie, datosComWifi;
//Wifi 		myWifi(datosComWifi.bufferRx,&datosComWifi.indexWriteRx, sizeof(datosComWifi.bufferRx));

uint8_t		debounceTimer20ms;

_uFlag		myFlags0;
_sButton 	myButtons[NUMBUTTONS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void heartBeat();
void ledStatus();
void imAlive();

void buttonsStatus(_sButton *buttons, uint8_t index);
void inicializaButtons(_sButton *buttons);
void debounceTask();

void decodeProtocol(_sDato *);
void decodeData(_sDato *, uint8_t COMAND);
void comunicationsTask(_sDato *datosCom, uint8_t source);
void autoConnectWifi(void);

//int Decode(uint8_t index);
//void decodeData(uint8_t comand);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		if (time100ms)
			time100ms--;
		if (debounceTimer20ms)
			debounceTimer20ms--;
		if (myButtons[0].timePush < 500)
			myButtons[0].timePush++;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1){
		datosComSerie.indexWriteRx++;
		HAL_UART_Receive_IT(&huart1, &datosComSerie.bufferRx[datosComSerie.indexWriteRx], 1);
	}
}

void heartBeat() {
	static uint8_t move=0;	//Ultima modificacion
	if ((~mask) & (1<<move)) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);	//Prende el led
	}
	move++;
	if (move > HBintervalWidth)
		move = 0;
}

void ledStatus() {
    switch (mode) {
        case 0:
            HBintervalWidth = THREESECONDSINTERVAL;
            mask = 0x55555555;
            break;
        case 1:
            if (LEDMODE == THREESECONDS) {
                if (CARSTATUS) {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 0x5F;
                } else {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 1;
                }
            } else {
                HBintervalWidth = ONESECONDINTERVAL;
                mask = 1;
            }
            break;
        case 2:
            if (LEDMODE == THREESECONDS) {
                if (CARSTATUS) {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 0x15F;
                } else {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 5;
                }
            } else {
                HBintervalWidth = ONESECONDINTERVAL;
                mask = 5;
            }
            break;
        case 3:
            if (LEDMODE == THREESECONDS) {
                if (CARSTATUS) {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 0x55F;
                } else {
                    HBintervalWidth = THREESECONDSINTERVAL;
                    mask = 21;
                }
            } else {
                HBintervalWidth = ONESECONDINTERVAL;
                mask = 21;
            }
    }
}

void imAlive() {
	decodeData(&datosComSerie, GETALIVE);
}

void buttonsStatus(_sButton *buttons, uint8_t index) {
    switch(buttons->estado){
        case UP:
            if (buttons->event == EV_PRESSED) {
                buttons->estado = FALLING;
            }
        break;
        case DOWN:
            if (buttons->event == EV_NOT_PRESSED) {
                buttons->estado = RISING;
            } else {
				if ((CARSTATUS == WORKING) && (buttons->timePush > 300)) { // SE DETIENE EL MODO, LED EN PERIODO DE 3 SEGUNDOS. MAS DE 3000ms
					LEDMODE = THREESECONDS;
					CARSTATUS = RESTING;
					ledStatus();
				} else {
					if (buttons->timePush >= 500) { // SE VUELVE AL PERIODO DE 3 SEGUNDOS Y SE CANCELA EL INICIO. MAS DE 5000ms
						LEDMODE = THREESECONDS;
						ledStatus();
					} else {
						if (buttons->timePush > 100) { // SE MANTIENE PRESIONADO EL BOTON POR MAS DE UN SEGUNDO, EMPIEZA SECUENCIA DE 1 SEG. MAS DE 1000ms
							LEDMODE = ONESECOND;
							ledStatus();
						}
					}
				}
            }
        break;
        case FALLING:
            if (buttons->event == EV_PRESSED) {
                buttons->estado = DOWN;
                buttons->timePush = 0; // SE INICIA EL CRONOMETRO DE PULSADO
            } else {
                buttons->estado = UP;
            }
        break;
        case RISING:
            if (buttons->event == EV_NOT_PRESSED) {
                buttons->estado = UP;
                if ((CARSTATUS == RESTING) && (buttons->timePush >= 10)) { // HACE ALGO SOLO SI SE SUELTA EL BOTON PRESIONANDOLO POR MAS DE 100 MS
                    if (buttons->timePush <= 100) { // SI SE SUELTA ENTRE 100MS Y 1000MS SE CAMBIA DE MODO
                        mode++;
                        if (mode == MODE3+1) {
                            mode = IDLE;
                        }
                        LEDMODE = THREESECONDS;
                        ledStatus();
                    } else {
                        if ((buttons->timePush < 500) && (mode != IDLE)) { // SI EL MODO DEL AUTO NO ES IDLE SE INICIA EL MODO SELECCIONADO Y SE PONE EL LED EN MODO ON. MENOS DE 5000ms
                            LEDMODE = THREESECONDS;
                            CARSTATUS = WORKING;
                            ledStatus();
                        }
                    }
                } else {
                    if (buttons->timePush <= 300) { // SI NO SE SUELTA DESPUES DE LOS 3000 MS SE VUELVE A TRABAJAR
                        LEDMODE = THREESECONDS;
                        ledStatus();
                    }
                }
            } else {
                buttons->estado = DOWN;
            }
        break;
    }
}

void inicializaButtons(_sButton *buttons) {
    buttons->estado = UP;
    buttons->timePush = 500;
    buttons->event = EV_NONE;
}

void debounceTask() {
	debounceTimer20ms = 2;
    for (uint8_t index = 0; index < NUMBUTTONS; index++){
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) & (1 << index)){
            myButtons[index].event = EV_NOT_PRESSED;
        } else {
            myButtons[index].event = EV_PRESSED;
        }
        buttonsStatus(&myButtons[index], index);
    }
}

void decodeProtocol(_sDato *datosCom) {
    static uint8_t nBytes=0;
    uint8_t indexWriteRxCopy=datosCom->indexWriteRx;

    while (datosCom->indexReadRx!=indexWriteRxCopy) {
        switch (estadoProtocolo) {
            case START:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
                    estadoProtocolo=HEADER_2;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
			case HEADER_3:
				if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
					estadoProtocolo=NBYTES;
				else{
					datosCom->indexReadRx--;
					estadoProtocolo=START;
				}
				break;
            case NBYTES:
                datosCom->indexStart=datosCom->indexReadRx; // 4 posiciones mas adelante tenes el ID en nuestro caso, por los datos del numero de auto y uno mas
                posicionComand = datosCom->indexStart + POSID;
                nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
                    estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^nBytes^':';
                }
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
                        decodeData(datosCom, datosCom->bufferRx[posicionComand]); //!< PASO LA ESTRUCTURA DATOSCOM Y PASO EL COMANDO
                    }
                }
                break;
            default:
                estadoProtocolo = START;
                break;
        }
    }
}

void decodeData(_sDato *datosCom, uint8_t COMAND) {
    //wifiData *wifidataPtr;
    //uint8_t *ptr;
    uint8_t auxBuffTx[50], indiceAux=0, cheksum, sizeWifiData, indexBytesToCopy=0, numBytesToCopy=0;
    uint8_t QtAngle = 0, configModo = 0;  //i = 0
    _work w;

    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';
    auxBuffTx[indiceAux++]=0x0C;    //!< NUMERO DE AUTO
    auxBuffTx[indiceAux++]=0x00;    //!< PARA COMPLETAR EL UINT16

    switch (COMAND) { // NO BORRAR EL _sDato *datosCom de la funcion ya que se usa en el startconfig, agregar el comand solamente
        case GETALIVE:
            auxBuffTx[indiceAux++]=GETALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x05;
            break;
        case STARTCONFIG: //Inicia Configuración del wifi
//            auxBuffTx[indiceAux++]=STARTCONFIG;
//            auxBuffTx[indiceAux++]=ACK;
//            auxBuffTx[NBYTES]=0x05;
//            myWifi.resetWifi();
//            sizeWifiData =sizeof(myWifiData);
//            indexBytesToCopy=datosCom->indexStart+POSDATA;
//            wifidataPtr=&myWifiData;
//            if ((RINGBUFFLENGTH - indexBytesToCopy)<sizeWifiData){
//                numBytesToCopy=RINGBUFFLENGTH-indexBytesToCopy;
//                memcpy(wifidataPtr,&datosCom->bufferRx[indexBytesToCopy], numBytesToCopy);
//                indexBytesToCopy+=numBytesToCopy;
//                sizeWifiData-=numBytesToCopy;
//                ptr= (uint8_t *)wifidataPtr + numBytesToCopy;
//                memcpy(ptr,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
//            }else{
//                memcpy(&myWifiData,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
//            }
//            myWifi.configWifi(&myWifiData);
            break;
        case FIRMWARE:

            break;
        case LEDS:

            break;
        case PULSADORES:
            auxBuffTx[indiceAux++]=PULSADORES;
            auxBuffTx[indiceAux++]=myButtons[0].event;
            auxBuffTx[NBYTES]=0x05;
            break;
        case SERVO:
//            if (!movingServo) {
//                QtAngle = datosCom->bufferRx[posicionComand + 1];
//                moverServo(QtAngle);
//                auxBuffTx[indiceAux++] = SERVO;
//                auxBuffTx[indiceAux++] = 0x0D;
//                auxBuffTx[NBYTES] = 0x05;
//            } else {
//                auxBuffTx[indiceAux++] = SERVO;
//                auxBuffTx[indiceAux++] = 0x0A;
//                auxBuffTx[NBYTES] = 0x05;
//            }
            break;
        case GETSERVOANGLE:
//            auxBuffTx[indiceAux++] = GETSERVOANGLE;
//            auxBuffTx[indiceAux++] = lastAngle;
//            auxBuffTx[NBYTES] = 0x05;
            break;
        case SETMINMAXSERVO:
//            configModo = datosCom->bufferRx[posicionComand + 1];
//            if (configModo) {
//                w.u8[0] = datosCom->bufferRx[posicionComand + 2];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 3];
//                maxMsServo = w.u16[0];
//                w.u8[0] = datosCom->bufferRx[posicionComand + 4];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 5];
//                minMsServo = w.u16[0];
//            }
//            w.u16[0] = maxMsServo;
//            auxBuffTx[indiceAux++] = SETMINMAXSERVO;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = minMsServo;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            auxBuffTx[NBYTES] = 0x08;
            break;
        case ULTRASONIDO:
//            w.u32 = distancia;
//            auxBuffTx[indiceAux++] = ULTRASONIDO;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            auxBuffTx[indiceAux++] = w.u8[2];
//            auxBuffTx[indiceAux++] = w.u8[3];
//            auxBuffTx[NBYTES] = 0x08;
            break;
        case SEGUIDORES_LINEA:
//            auxBuffTx[indiceAux++] = SEGUIDORES_LINEA;
//            w.u16[0] = irSensor[0].currentValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[1].currentValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[2].currentValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            auxBuffTx[NBYTES]=0x0A;
            break;
        case SETSENSORSBLACK:
//            configModo = datosCom->bufferRx[posicionComand + 1];
//            if (configModo) {
//                w.u8[0] = datosCom->bufferRx[posicionComand + 2];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 3];
//                irSensor[0].blackValue = w.u16[0];
//                w.u8[0] = datosCom->bufferRx[posicionComand + 4];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 5];
//                irSensor[1].blackValue = w.u16[0];
//                w.u8[0] = datosCom->bufferRx[posicionComand + 6];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 7];
//                irSensor[2].blackValue = w.u16[0];
//            }
//            w.u16[0] = irSensor[0].blackValue;
//            auxBuffTx[indiceAux++] = SETSENSORSBLACK;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[1].blackValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[2].blackValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            auxBuffTx[NBYTES] = 0x0A;
            break;
        case SETSENSORSWHITE:
//            configModo = datosCom->bufferRx[posicionComand + 1];
//            if (configModo) {
//                w.u8[0] = datosCom->bufferRx[posicionComand + 2];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 3];
//                irSensor[0].whiteValue = w.u16[0];
//                w.u8[0] = datosCom->bufferRx[posicionComand + 4];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 5];
//                irSensor[1].whiteValue = w.u16[0];
//                w.u8[0] = datosCom->bufferRx[posicionComand + 6];
//                w.u8[1] = datosCom->bufferRx[posicionComand + 7];
//                irSensor[2].whiteValue = w.u16[0];
//            }
//            w.u16[0] = irSensor[0].whiteValue;
//            auxBuffTx[indiceAux++] = SETSENSORSWHITE;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[1].whiteValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            w.u16[0] = irSensor[2].whiteValue;
//            auxBuffTx[indiceAux++] = w.u8[0];
//            auxBuffTx[indiceAux++] = w.u8[1];
//            auxBuffTx[NBYTES] = 0x0A;
            break;
        case TEST_MOTORES:
//            auxBuffTx[indiceAux++] = TEST_MOTORES;
//            auxBuffTx[indiceAux++] = 0x0D;
//            auxBuffTx[NBYTES] = 0x05;
//            for (uint8_t i=1; i<5; i++) {
//                w.i8[i-1] = datosCom->bufferRx[posicionComand + i];
//            }
//            engines.RMPOWER = w.i32;
//            for (uint8_t i=5; i<9; i++) {
//                w.i8[i-5] = datosCom->bufferRx[posicionComand + i];
//            }
//            engines.LMPOWER = w.i32;
//            setEnginesPower(SELECTBOTHMOTORS, FREEWHEEL);
            break;
        case VELOCIDAD_MOTORES:
//            auxBuffTx[indiceAux++] = VELOCIDAD_MOTORES;
//            w.u32 = rightPulseSum;
//            for (uint8_t i=0; i<4; i++) {
//                auxBuffTx[indiceAux++] = w.u8[i];
//            }
//            w.u32 = leftPulseSum;
//            for (uint8_t i=0; i<4; i++) {
//                auxBuffTx[indiceAux++] = w.u8[i];
//            }
//            auxBuffTx[NBYTES] = 0x0C;
            break;

        default:
            auxBuffTx[indiceAux++]=0xFF;
            auxBuffTx[NBYTES]=0x04;
            break;
    }
    cheksum=0;
    for(uint8_t a=0 ; a<indiceAux ;a++) {
        cheksum ^= auxBuffTx[a];
        datosCom->bufferTx[datosCom->indexWriteTx++] = auxBuffTx[a];
    }
        datosCom->bufferTx[datosCom->indexWriteTx++] = cheksum;
}

void comunicationsTask(_sDato *datosCom, uint8_t source){
    if(datosCom->indexReadRx!=datosCom->indexWriteRx ){
        decodeProtocol(datosCom);
    }

    if(datosCom->indexReadTx!=datosCom->indexWriteTx){
        if(source){
        	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)){
				USART1->DR = datosCom->bufferTx[datosCom->indexReadTx++];
				//HAL_UART_Transmit_IT(&huart1, &datosCom->bufferTx[datosCom->indexReadTx++], 1); //La anterior linea hace lo mismo
				//HAL_UART_Transmit(&huart1, &datosCom->bufferTx[datosCom->indexReadTx++], 1, 0);
			}
        } //else
           // myWifi.writeWifiData(&datosCom->bufferTx[datosCom->indexReadTx++],1);
    }
}

void autoConnectWifi(){
//    #ifdef AUTOCONNECTWIFI
//        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
//        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
//        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
//        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
//        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
//        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
//        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
//        myWifi.configWifi(&myWifiData);
//    #endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1, &datosComSerie.bufferRx[datosComSerie.indexWriteRx], 1);

  time100ms=10;
  LEDMODE=THREESECONDS;
  mask=0x55555555;
  HBintervalWidth = THREESECONDSINTERVAL;

  mode=IDLE;
  CARSTATUS=RESTING;

  for (uint8_t index = 0; index < NUMBUTTONS; index++) {
	  inicializaButtons(&myButtons[index]);
  }
  debounceTimer20ms = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (!time100ms) {
		heartBeat();
		imAlive();
		time100ms = 10;
	}
	if (!debounceTimer20ms)
		debounceTask();

	//myWifi.taskWifi();
	comunicationsTask(&datosComSerie,true);
	comunicationsTask(&datosComWifi,false);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
