/*
TCC - Izabela Vasconcelos

01/2018
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

//Define os protocolos do Leitor
//#define Open 0x01
#define Open 0x01
//#define CmosLed 0x12
#define CmosLed 0x12
#define GetEnrollCount 0x20
#define CheckEnrolled 0x21
#define EnrollStart 0x22
#define Enroll1 0x23
#define Enroll2 0x24
#define Enroll3 0x25
#define IsPressFinger 0x26
#define ACK 0x30
#define NACK 0x31
#define DeleteID 0x40
#define DeleteAll 0x41
#define Identify 0x51
#define CaptureFinger 0x60

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	//Variaveis gerais da comunicação serial
	char vetor[30];//GERAL
	uint8_t vaux[240];
	char A[100];
	int recebi=0;
	//Variaveis do temp e umid
	float temperatura;
	float umidade;
	int tempWant, tUser1,tUser2;
	float auxAll;
	// Leitor okay
	int enroll;
	int acesso;
	char vet[5];
	//Dimmers
	int perc,lumUser1,lumUser2;
	int musicUser1, musicUser2,music;
	int cont=0;
	int aux=0;
	int pix=0;
	// Alarme
	int buzzer=0;
	int alarme=1;
	// var do usuario
	int usuario;
	//Variáveis da Wifi
	uint8_t vetor_at[1000], vetor_at2[1000], vetor_at3[1000], vetor_at4[1000], vetor_at5[1000], vetor_at6[1000], vetor_at7[1000], vetor_at8[1000]; 
	uint8_t vwifi[50];
	//Variáveis do leitor
	//uint8_t f_status = 0;
	//uint8_t vetor_ID[50]="";
	//uint8_t retorno_ID[2]={0,0};
	
	typedef struct //Estrutura para o leitor
	{
		char A[4];
		uint8_t AA;
	} REC;
	REC recebe;
	
	typedef struct //Estrutura para o leitor
	{
		uint8_t 	b1;		
		uint8_t 	b2;		
		uint16_t	devId;
		uint32_t	parameterID;
		uint16_t	wCmd_Resp;
		uint16_t 	wChkSum;
	} ID_struct;
	ID_struct ID_cmd; //Struct para comandos e Resposta
	ID_struct ID_resp;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void id_init(void);
char Cmd_Resp_ID(uint32_t, uint16_t);
void check_ID(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// PINAGEM DO LCD
//D4 -> PB_5
//D5 -> PB_4
//D6 -> PB_10
//D7 -> PA_8
//RS -> D8 -> PA_9
//EN -> D9 -> PC_7

//Display LDC auxiliar
void decod_lcd(int comando)
{
	if((comando & 0x01)==0x01)
		HAL_GPIO_WritePin(GPIOB,(1<<5),1);
	else
		HAL_GPIO_WritePin(GPIOB,(1<<5),0);
	
	if((comando & 0x02)==0x02)
		HAL_GPIO_WritePin(GPIOB,(1<<4),1);
	else
		HAL_GPIO_WritePin(GPIOB,(1<<4),0);
	
	if((comando & 0x04)==0x04)
		HAL_GPIO_WritePin(GPIOB,(1<<10),1);
	else
		HAL_GPIO_WritePin(GPIOB,(1<<10),0);
	
	if((comando & 0x08)==0x08)
		HAL_GPIO_WritePin(GPIOA,(1<<8),1);
	else
		HAL_GPIO_WritePin(GPIOA,(1<<8),0);
}

void lcd_comando(int comando)
{
	int c1=0,c2=0;
	c1 = comando & 0x0f;
	c2 = (comando & 0xf0)>>4;
	
	HAL_GPIO_WritePin(GPIOA,(1<<9),0);//RS=0
	
	HAL_GPIO_WritePin(GPIOB,(1<<5)+(1<<4)+(1<<10),0);//ZERA PORTA
	HAL_GPIO_WritePin(GPIOA,(1<<8),0);//RS=0
	
	decod_lcd(c2);// escreve os 1s na porta
	HAL_GPIO_WritePin(GPIOC,(1<<7),1);//EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,(1<<7),0);//EN=0
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(GPIOB,(1<<5)+(1<<4)+(1<<10),0);//ZERA PORTA
	HAL_GPIO_WritePin(GPIOA,(1<<8),0);//RS=0
	
	decod_lcd(c1);// escreve os 1s na porta
	HAL_GPIO_WritePin(GPIOC,(1<<7),1);//EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,(1<<7),0);//EN=0
	HAL_Delay(1);	
}

void lcd_dado(int comando)
{
	int c1=0,c2=0;
	c1 = comando & 0x0f;
	c2 = (comando & 0xf0)>>4;
	
	HAL_GPIO_WritePin(GPIOA,(1<<9),1);//RS=1
	
	HAL_GPIO_WritePin(GPIOB,(1<<5)+(1<<4)+(1<<10),0);//ZERA PORTA
	HAL_GPIO_WritePin(GPIOA,(1<<8),0);//RS=0
	
	decod_lcd(c2);// escreve os 1s na porta
	HAL_GPIO_WritePin(GPIOC,(1<<7),1);//EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,(1<<7),0);//EN=0
	HAL_Delay(1);
	
	HAL_GPIO_WritePin(GPIOB,(1<<5)+(1<<4)+(1<<10),0);//ZERA PORTA
	HAL_GPIO_WritePin(GPIOA,(1<<8),0);//RS=0
	
	decod_lcd(c1);// escreve os 1s na porta
	HAL_GPIO_WritePin(GPIOC,(1<<7),1);//EN=1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC,(1<<7),0);//EN=0
	HAL_Delay(1);	
}

void lcd_init(void)
{
	lcd_comando(0x28);
	lcd_comando(0x0e);
	lcd_comando(0x06);
	lcd_comando(0x01);
	HAL_Delay(100);
}

void lcd_goto(int linha, int coluna)
{
	if(linha==1)
		lcd_comando(0x80+coluna);
	else if(linha==2)
		lcd_comando(0xc0+coluna);
}


void lcd_string(char vetor[])
{
	int i,x;
	x=strlen(vetor);
	for(i=0;i<x;i++)
		lcd_dado(vetor[i]);
}
void LCD_CLR(void)
{
	lcd_goto(1,0);
	lcd_string("               ");
	lcd_goto(2,0);
	lcd_string("               ");
}
//Leitura sensor temp e umid
void le_temp ()
{
	uint16_t T0_degC, T1_degC;
	float T0_degCf, T1_degCf;
	uint8_t T_degC_x8[2], T_MSB, t01out[4],t_out8[2], aux1;
	int16_t T0_out, T1_out, t_out;

	aux1=0x81;
	HAL_I2C_Mem_Write(&hi2c1,0xbf,0x20,I2C_MEMADD_SIZE_8BIT,&aux1,1,500);

	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x32,I2C_MEMADD_SIZE_8BIT, &T_degC_x8[0] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x33,I2C_MEMADD_SIZE_8BIT, &T_degC_x8[1] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x2A,I2C_MEMADD_SIZE_8BIT,&t_out8[0] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x2B,I2C_MEMADD_SIZE_8BIT,&t_out8[1] ,1,500);
	HAL_Delay(50);

	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x3C,I2C_MEMADD_SIZE_8BIT,&t01out[0] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x3D,I2C_MEMADD_SIZE_8BIT,&t01out[1] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x3E,I2C_MEMADD_SIZE_8BIT,&t01out[2] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x3F,I2C_MEMADD_SIZE_8BIT,&t01out[3] ,1,500);
	HAL_Delay(50);
	HAL_I2C_Mem_Read(&hi2c1,0xBF,0x35,I2C_MEMADD_SIZE_8BIT,&T_MSB ,1,500);
	HAL_Delay(50);

	T0_degC = T_degC_x8[0] + ((T_MSB & 0x03)<<8);
	T1_degC = T_degC_x8[1] + ((T_MSB & 0x0C)<<6);
	T0_degCf = (float)T0_degC/8 ;
	T1_degCf = (float)T1_degC/8 ;
	T0_out = ((int16_t)t01out[0] | ((int16_t)t01out[1]<<8));
	T1_out = ((int16_t)t01out[2] | ((int16_t)t01out[3]<<8));
	t_out	 = ((int16_t)t_out8[0] | ((int16_t)t_out8[1]<<8));

	temperatura= ((int32_t)(t_out-T0_out))*((float)(T1_degCf-T0_degCf))/ (int32_t)(T1_out - T0_out) + T0_degCf;
}

void le_umidade ()
{
	unsigned short H0_rH, H1_rH;
	short H0_T0_OUT, H1_T0_OUT, H_OUT;
	uint8_t dados[2];
	
	dados[0] = 0x81;
	HAL_I2C_Mem_Write(&hi2c1,0xbe,0x20,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );

	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x30,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );
	H0_rH = dados[0]/2;

	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x31,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );
	H1_rH = dados[0]/2;

	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x36,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );
	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x37,I2C_MEMADD_SIZE_8BIT,&dados[1],1,500 );
	H0_T0_OUT = (dados[1]<<8) | dados[0];
	
	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x3A,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );
	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x3B,I2C_MEMADD_SIZE_8BIT,&dados[1],1,500 );
	H1_T0_OUT = (dados[1]<<8) | dados[0];
	
	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x28,I2C_MEMADD_SIZE_8BIT,&dados[0],1,500 );
	HAL_I2C_Mem_Read(&hi2c1,0xbf,0x29,I2C_MEMADD_SIZE_8BIT,&dados[1],1,500 );
	H_OUT = (dados[1]<<8) | dados[0];
	
	umidade = (H0_rH) + (float)((H1_rH - H0_rH)*(H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT);
}
void musica()
{
	if(musicUser1==1 && musicUser2==1) music=1;
	if(musicUser1==2 && musicUser2==1) music=3;
	if(musicUser1==3 && musicUser2==1) music=2;
	if(musicUser1==4 && musicUser2==1) music=2;
	
	if(musicUser1==1 && musicUser2==2) music=4;
	if(musicUser1==2 && musicUser2==2) music=2;
	if(musicUser1==3 && musicUser2==2) music=4;
	if(musicUser1==4 && musicUser2==2) music=3;
	
	if(musicUser1==1 && musicUser2==3) music=2;
	if(musicUser1==2 && musicUser2==3) music=4;
	if(musicUser1==3 && musicUser2==3) music=3;
	if(musicUser1==4 && musicUser2==3) music=1;
	
	if(musicUser1==1 && musicUser2==4) music=3;
	if(musicUser1==2 && musicUser2==4) music=1;
	if(musicUser1==3 && musicUser2==4) music=1;
	if(musicUser1==4 && musicUser2==4) music=4;
	

}
void Controle_Temp()
{
//Conmtrole do  relé baseado  na temperatura
		if(usuario==0)
		{
			HAL_GPIO_WritePin(GPIOC,(1<<9),0);//LIGA RELÉ 1 -> PORTA PC_9   //AQUECENDO
			HAL_GPIO_WritePin(GPIOC,(1<<8),0);//DESLIGA RELÉ 2 -> PORTA PC_8   //RESFRIA
		}
		else if (tempWant>temperatura)
		{
			//TROCAR PORTAS DE SAÍDA
			HAL_GPIO_WritePin(GPIOC,(1<<9),1);//LIGA RELÉ 1 -> PORTA PC_9   //AQUECENDO
			HAL_GPIO_WritePin(GPIOC,(1<<8),0);//DESLIGA RELÉ 2 -> PORTA PC_8   //RESFRIA
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOC,(1<<9),0);//DESLIGA RELÉ 1 -> PORTA PC_9   //AQUECENDO
			HAL_GPIO_WritePin(GPIOC,(1<<8),1);//LIGA RELÉ 2 -> PORTA PC_8   //RESFRIA
		}
		// Fim do controle tempWant

}

// Led dimmer
void HAL_SYSTICK_Callback(void)
{ 
		
		if(perc==0) HAL_GPIO_WritePin(GPIOC,(1<<2),0);
		else if(perc==8) HAL_GPIO_WritePin(GPIOC,(1<<2),1);
		else if(cont<perc)
			{
				HAL_GPIO_WritePin(GPIOC,(1<<2),1);//led=1
				cont++;
			}
			else
			{	
			HAL_GPIO_WritePin(GPIOC,(1<<2),0);//led=1	
			cont++;
			}
	if (cont==8) cont=0;
	//HAL_UART_Receive(&huart1, A, 4,100);

}

//Wifi
void init_Wifi()
{

	HAL_UART_Transmit(&huart1, "AT\r", 3, 100);		// comando teste do ESP
	HAL_UART_Receive(&huart1, vetor_at, 11, 3000);		// recebe a confirmação
	HAL_UART_Transmit(&huart2, vetor_at, 11, 300);		// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT+S.SSIDTXT=embarcados\r", 24, 500);	// comando RESET do ESP
	HAL_UART_Receive(&huart1, vetor_at2, 500, 1000);	// recebe confirmação
	HAL_UART_Transmit(&huart2, vetor_at2, 500, 1000);	// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT+S.SCFG=wifi_wpa_psk_text,embarcados\r\n", 39, 500);  // MODO=1 - roteador
	HAL_UART_Receive(&huart1, vetor_at3, 500, 1000); 	// recebe confirmação
	HAL_UART_Transmit(&huart2, vetor_at3, 500, 1000); 	// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT+S.SCFG=wifi_priv_mode,2\r", 27, 700);	// CWLAP -> lista as redes
	HAL_UART_Receive(&huart1, vetor_at4, 1000, 10000);	// recebe as redes wifi
	HAL_UART_Transmit(&huart2, vetor_at4, 1000, 10000);	// transmite via USB
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1, "AT+S.SCFG=wifi_mode,1\r", 22, 500);  // MODO=1 - roteador
	HAL_UART_Receive(&huart1, vetor_at3, 500, 1000); 	// recebe confirmação
	HAL_UART_Transmit(&huart2, vetor_at3, 500, 1000); 	// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT&W\r", 5, 700);	// CWLAP -> lista as redes
	HAL_UART_Receive(&huart1, vetor_at4, 1000, 10000);	// recebe as redes wifi
	HAL_UART_Transmit(&huart2, vetor_at4, 1000, 10000);	// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT+CFUN=1\r", 10, 700);	// CWLAP -> lista as redes
	HAL_UART_Receive(&huart1, vetor_at4, 1000, 5000);	// recebe as redes wifi
	HAL_UART_Transmit(&huart2, vetor_at4, 1000, 5000);	// transmite via USB
	HAL_Delay(20000);
	
	HAL_UART_Transmit(&huart1, "AT+S.SOCKD=32000\r", 17, 700);	// CWLAP -> lista as redes
	HAL_UART_Receive(&huart1, vetor_at4, 1000, 5000);	// recebe as redes wifi
	HAL_UART_Transmit(&huart2, vetor_at4, 1000, 5000);	// transmite via USB
	HAL_Delay(100);
	
	HAL_UART_Transmit(&huart1, "AT+S.STS=ip_ipaddr\r", 19, 700);	// CWLAP -> lista as redes
	HAL_UART_Receive(&huart1, vetor_at4, 500, 5000);	// recebe as redes wifi
	HAL_UART_Transmit(&huart2, vetor_at4, 500, 5000);	// transmite via USB
	HAL_Delay(500);
	

}
//Funções do controle de temperatura
void return_unid(void)
{
	if(recebe.A[2]=='0') tempWant=tempWant+0;
	if(recebe.A[2]=='1') tempWant=tempWant+1;
	if(recebe.A[2]=='2') tempWant=tempWant+2;
	if(recebe.A[2]=='3') tempWant=tempWant+3;
	if(recebe.A[2]=='4') tempWant=tempWant+4;
	if(recebe.A[2]=='5') tempWant=tempWant+5;
	if(recebe.A[2]=='6') tempWant=tempWant+6;
	if(recebe.A[2]=='7') tempWant=tempWant+7;
	if(recebe.A[2]=='8') tempWant=tempWant+8;
	if(recebe.A[2]=='9') tempWant=tempWant+9;
}
void return_tempWant(void)
{

	if (recebe.A[1]=='1')
	{
		tempWant = 10;
		return_unid();
	}
	if (recebe.A[1]=='2')
	{
		tempWant = 20;
		return_unid();
	}
	if (recebe.A[1]=='3')
	{
		tempWant = 30;
		return_unid();
	}
}

void return_luminosidade(void)
{
	if (recebe.A[1]=='0') perc=0;
	else if (recebe.A[1]=='1' && recebe.A[2]=='2') perc=1;
	else if (recebe.A[1]=='2' && recebe.A[2]=='4') perc=2;
	else if (recebe.A[1]=='3' && recebe.A[2]=='6') perc=3;
	else if (recebe.A[1]=='4' && recebe.A[2]=='8') perc=4;
	else if (recebe.A[1]=='6' && recebe.A[2]=='0') perc=5;
	else if (recebe.A[1]=='7' && recebe.A[2]=='2') perc=6;
	else if (recebe.A[1]=='8' && recebe.A[2]=='4') perc=7;
	else if (recebe.A[1]=='9' && recebe.A[2]=='6') perc=8;
	
}
//Funções do Leitor
void init_ID(void)
{
	Cmd_Resp_ID(0, Open); //Open
	HAL_Delay(300);
	sprintf(vaux,"\n Init okay \n");
	HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
	Cmd_Resp_ID(1, CmosLed); //LED On
	HAL_Delay(100);
	/*
	if(!Cmd_Resp_ID(1, CmosLed)) //LED On
	{
		HAL_Delay(2000);
		sprintf(vaux,"\n Led on \n");
		HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
	}
	else
		HAL_Delay(200);
		Cmd_Resp_ID(0, CmosLed); //LED Off	
	*/
}

char Cmd_Resp_ID(uint32_t	Param, uint16_t	Cmd)
{
	char Resp = 0;
	unsigned long ChkSum=0;
	
	//COMMAND PACKET
	//Command start code1
	ID_cmd.b1 = 0x55;
	ChkSum += ID_cmd.b1;
	//Command start code1
	ID_cmd.b2 = 0xAA;
	ChkSum += ID_cmd.b2;
	//Device ID: default is 0x0001, always fixed
	ID_cmd.devId = 0x0001;
	ChkSum += ID_cmd.devId;
	//Input Parameter
	ID_cmd.parameterID =Param;
	ChkSum += ID_cmd.parameterID;
	//Command Code
	ID_cmd.wCmd_Resp = Cmd;
	ChkSum += ID_cmd.wCmd_Resp;
	//CheckSum (byte addition) 
	// O check sum é o somatório de todos os OFFSETS
	ID_cmd.wChkSum = ChkSum;	
//	sprintf(vaux,"\n Chk%u \n",ChkSum);
//	HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
	//Envia a Struct da comando completa
	HAL_UART_Transmit(&huart4,(uint8_t *)&ID_cmd,sizeof(ID_struct),1000);

	//Response Packet (Acknowledge) 
	//A resposta vem como struct tbm	
	HAL_UART_Receive(&huart4,(uint8_t *)&ID_resp,sizeof(ID_struct),1000);
	//HAL_UART_Receive(&huart4,vaux,strlen(vaux),2000);
	if(ID_resp.wCmd_Resp == ACK) // OUTPUT PARAMETER
	{	Resp = 1;
	}
		else if(ID_resp.wCmd_Resp == NACK) //ERROR CODE
		{
		Resp = 0;
		}
	return Resp;		
}

void check_ID(void)
{
	
	Cmd_Resp_ID(1,CmosLed);//LED On
	
	if(Cmd_Resp_ID(0,IsPressFinger) && ID_resp.parameterID == 0) //Finger Pressed
	{
		Cmd_Resp_ID(0,CaptureFinger); //Fast Capture FingerPrint
		if(Cmd_Resp_ID(0,Identify))
		{
			enroll=1;
			if(ID_resp.parameterID==1)
			{
				sprintf(vaux,"\nuser 1\n");
				HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
				HAL_GPIO_WritePin(GPIOC,(1<<5),0);
				buzzer=0;
				HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //destravado
				HAL_Delay(800);
				HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
				if(usuario==0) {usuario=1;tempWant=tUser1;perc=lumUser1;music=musicUser1;}
				else if(usuario==2) {usuario=3; tempWant=(tUser1+tUser2)/2;perc=(lumUser1+lumUser2)/2; musica();}
				else if(usuario==1) {usuario=0;tempWant=23;perc=0;music=0;
				HAL_GPIO_WritePin(GPIOC,(1<<9),0);//LIGA RELÉ 1 -> PORTA PC_9   //AQUECENDO
				HAL_GPIO_WritePin(GPIOC,(1<<8),0);//DESLIGA RELÉ 2 -> PORTA PC_8   //RESFRIA
				}
				else if(usuario==3) {usuario=2;tempWant=tUser2;perc=lumUser2;music=musicUser2;}
				HAL_Delay(1000);
				Cmd_Resp_ID(0,CmosLed);
				HAL_Delay(1000);
			}
			
			else if(ID_resp.parameterID==0)
			{
				HAL_GPIO_WritePin(GPIOC,(1<<5),0);
				buzzer=0;
				HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //destravado
				HAL_Delay(800);
				HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
				sprintf(vaux,"Teste 0! ID:%u",ID_resp.parameterID);
				if(usuario==0) {usuario=2;tempWant=tUser2;perc=lumUser2;music=musicUser2;}
				else if(usuario==1) {usuario=3; tempWant=(tUser1+tUser2)/2;perc=(lumUser1+lumUser2)/2; musica();}
				else if(usuario==2) {usuario=0;tempWant=23;perc=0;music=0;
				HAL_GPIO_WritePin(GPIOC,(1<<9),0);//LIGA RELÉ 1 -> PORTA PC_9   //AQUECENDO
				HAL_GPIO_WritePin(GPIOC,(1<<8),0);//DESLIGA RELÉ 2 -> PORTA PC_8   //RESFRIA
				}
				else if(usuario==3) {usuario=1;tempWant=tUser1; perc=lumUser1;music=musicUser1;}
				
				
				HAL_Delay(1000);
				Cmd_Resp_ID(0,CmosLed);
				HAL_Delay(1000);

			}
		}
		else			
		{
			buzzer++;
			if(alarme==1)
			{
				if (buzzer==3)
				{
					if(pix==1)pix=0;
					else pix=1;
					HAL_GPIO_WritePin(GPIOC,(1<<5),1);
				}
			}
		}
	}

	Cmd_Resp_ID(0,CmosLed);//LED Off
//	if(HAL_GPIO_ReadPin(GPIOC,(1<<6))==1) 
//		{
//			enroll=0;
//			//HAL_GPIO_WritePin(GPIOC,(1<<9),1);
//		}else enroll=1;
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t temp;
	uint8_t vetor_at1[1000];
	int cont=0,leitor=0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	tempWant=29;
	tUser1=23;
	tUser2=25;
	perc=0;
	lumUser2=5;
	lumUser1=0;
	music=2;
	musicUser1=1;
	musicUser2=4;
	usuario=0;
	// testa serial
	sprintf(vaux,"\nOla testando6 serial\n");
	HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
	// inicializaÃ§Ã£o wifi
	HAL_GPIO_WritePin(GPIOA,(1<<5),1);//led=1
	
	//inicializaçãi Wifi
	//init_Wifi();
	init_ID();
	HAL_GPIO_WritePin(GPIOA,(1<<5),0);//led=0
	// Fim da inicializaÃ§Ã£o da wifi
	HAL_UART_Transmit(&huart1,"oih", 3, 100);
	//check_ID();
	le_temp();
	le_umidade();
	enroll = 0;
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&recebe,sizeof(recebe));	
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //destravado
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	// INICIO DO PROGRAMA
		check_ID();
		if(HAL_GPIO_ReadPin(GPIOC,(1<<13))==0)
		{
			HAL_GPIO_WritePin(GPIOA,(1<<5),1);
			//inicializaçãi Wifi
			init_Wifi();
			HAL_GPIO_WritePin(GPIOA,(1<<5),0);
			
		}
		else{
		HAL_GPIO_WritePin(GPIOA,(1<<5),0);
		}
		//HAL_GPIO_WritePin(GPIOA,(1<<5),0);
		if(usuario==1){ tUser1=tempWant; lumUser1=perc;}
		if(usuario==2){ tUser2=tempWant; lumUser2=perc;}
		//if(usuario==3) tempWant=(tUser1+tUser2)/2;
		
		sprintf(vetor,"%2.1f%2.1f%d%d%d%d%d",temperatura, umidade,tempWant,perc,usuario,pix,music);
		HAL_UART_Transmit(&huart1,vetor, strlen(vetor), 100);
		if(pix==1) pix=0;
		if(music<5) music=5;
		
		if(cont==2)
		{
		le_temp();
		le_umidade();
		cont=0;
		}
		else cont=cont+1;

		if(recebi==1)
		{
			if (recebe.A[0]=='T') // CONTROLE DE TEMPERATURA DESEJADA 
			{
//				sprintf(vaux,"\nOla entrei\n");
//				HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
				return_tempWant();
//				sprintf(vetor,"%2.1f%2.1f%d%d%d%d",temperatura, umidade,tempWant,perc,usuario,pix);
//				HAL_UART_Transmit(&huart1,vetor, strlen(vetor), 100);
			} // fim if do recebe temperatura
			else if (recebe.A[0]=='I') return_luminosidade();	// LUMINOSIDADE CONTROLL
			else if (recebe.A[0]=='S') // TRAVAS CONTROLL
			{ // PORTA!!!!!!!!!!!!! $$$$$$$$$$$$$$$##########################
				if(recebe.A[1]=='0') 
					HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //travado
				else if (recebe.A[1]=='1')
					HAL_GPIO_WritePin(GPIOC,(1<<3),0);//-> PORTA PC_2   //destravado
			} // fim if das travas			
			else if (recebe.A[0]=='L') // ALARME CONTROLL
			{
				if(recebe.A[1]=='0') 
				{//HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_3   //ALARME ATIVADO
					alarme=1;
				}
				else if (recebe.A[1]=='1'){
					//HAL_GPIO_WritePin(GPIOC,(1<<3),0);//-> PORTA PC_3   //ALARME DESATIVADO
					alarme=0;
					}
				} // fim if do alarme
			
				else if (recebe.A[0]=='O')
			{
					if(recebe.A[1]=='1' && recebe.A[1]=='1') // SAÍDA user 1
					{
						if(usuario==1) {usuario=0;tempWant=23;perc=0;}
						else if(usuario==3) {usuario=2;tempWant=tUser2;perc=lumUser2;}
						HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //travado
						HAL_Delay(1000);
						HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
					}
					if(recebe.A[1]=='0' && recebe.A[1]=='0') // SAÍDA user 2
					{
						if(usuario==2) {usuario=0;tempWant=23;perc=0;}
						else if(usuario==3) {usuario=1;tempWant=tUser1; perc=lumUser1;}
						HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //travado
						HAL_Delay(1000);
						HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
					}
			}
			else if(recebe.A[0]=='P')
			{
				if(recebe.A[1]=='1' && recebe.A[1]=='1') // ABRE PORTA
					{
							HAL_GPIO_WritePin(GPIOC,(1<<3),1);// -> PORTA PC_2   //travado
							HAL_Delay(1000);
							HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
					}
					if(recebe.A[1]=='0' && recebe.A[1]=='0') // FECHA PORTA
					{
						HAL_GPIO_WritePin(GPIOC,(1<<3),0);// -> PORTA PC_2   //travado
					}
			
			}
			else if(recebe.A[0]=='M')
			{
				if(recebe.A[1]=='0' && recebe.A[2]=='0') // ABRE PORTA
					{
						music=0;
//						if(usuario==1) {musicUser1=music;}
//						else if(usuario==2) {musicUser2=music;}
//						else if(usuario==3) {usuario=0;tempWant=23;perc=0;}
//						else if(usuario==3) {usuario=1;tempWant=tUser1; perc=lumUser1;}
					}
					if(recebe.A[1]=='1' && recebe.A[2]=='0') // FECHA PORTA
					{
						music=1;
						if(usuario==1) musicUser1=1;
						if(usuario==2) musicUser2=1;
					}
					if(recebe.A[1]=='2' && recebe.A[2]=='0') // FECHA PORTA
					{
						music=2;
						if(usuario==1) musicUser1=2;
						if(usuario==2) musicUser2=2;
					}
					if(recebe.A[1]=='3' && recebe.A[2]=='0') // FECHA PORTA
					{
						music=3;
						if(usuario==1) musicUser1=3;
						if(usuario==2) musicUser2=3;
					}
					if(recebe.A[1]=='4' && recebe.A[2]=='0') // FECHA PORTA
					{
						music=4;
						if(usuario==1) musicUser1=4;
						if(usuario==2) musicUser2=4;
					}
					if(recebe.A[1]=='9' && recebe.A[2]=='9') // ABRE PORTA
					{
						music=5;
						if(usuario==1) musicUser1=0;
						if(usuario==2) musicUser2=0;
					}
			
			}
			recebi=0;
			recebe.A[0]='0';
			recebe.A[1]='0';
			recebe.A[2]='0';
		
		} // fim do recebi
		
		Controle_Temp();
		
		
  }//FIM WHILE
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&recebe,sizeof(recebe));	
	sprintf(vaux,"\nIt entrou\n");
	HAL_UART_Transmit(&huart2,vaux,strlen(vaux),100);
	recebi=1;	
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
