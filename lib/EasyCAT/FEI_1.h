#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project FEI_1.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	6
#define CUST_BYTE_NUM_IN	7
#define TOT_BYTE_NUM_ROUND_OUT	8
#define TOT_BYTE_NUM_ROUND_IN	8


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint32_t    Output2;
		uint16_t    Output1;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint32_t    Input2;
		uint16_t    Input1;
		uint8_t     Input3;
	}Cust;
} PROCBUFFER_IN;

#endif