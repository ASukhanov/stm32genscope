/*
 * mcufec.h
 *
 *  Created on: May 17, 2022
 *      Author: Andrei
 */

#ifndef INC_MCUFEC_H_
#define INC_MCUFEC_H_
#include <cstdint>
#include <cstdlib>

#define MinI32 -2147483648
#define MaxI32 2147483647
//``````````````````Message header for all UART communications`````````````````

struct UARTMsg_Header {
  uint16_t l; //message length in bytes
  char d;     //data description, bits[0:1] number of bytes per item minus 1, bits[2:5]: number of channels minus 1
  char id;    //data ID, '<' for JSON-formatted TTY replies, 'A' for ADC
};

enum UARTMsg_ID {
	UARTID_TTY = '<', //JSON formatted message
	UARTID_ADC = 'A', //Binary data from ADCs
};

//``````````````````Message for TTY communications`````````````````````````````
#define TTY_CR '\r'
#define TTY_LF '\n'
char TTY_CRLF[] = "\r\n";
#define TTY_TXBUF_SIZE 512
struct UARTMsg_TTY {
	struct UARTMsg_Header h;
	char msg[TTY_TXBUF_SIZE];
	};

//``````````````````Process Variables``````````````````````````````````````````
union VALUE {//numpy-like coding
int8_t      b;
uint8_t     B;
int16_t     i2;
uint16_t    u2;
int32_t     i4;
uint32_t    u4;
char*       str;
uint8_t*    Bptr;
uint16_t*   u2ptr;
};

typedef	int8_t   TD_b;
typedef	uint8_t	 TD_B;
typedef uint16_t TD_u2;
typedef	int16_t	 TD_i2;
typedef	int32_t	 TD_i4;
typedef	uint32_t TD_u4;
typedef char* TD_str;
//typedef uint8_t* TD_Bptr;

enum TYPE {
    T_b = 0,
    T_B = 1,
    T_i2 = 2,
    T_u2 = 3,
    T_i4 = 4,
    T_u4 = 5,
	T_str = 6,
    //T_Bptr = 7,
    //T_u2ptr = 8
};

enum FEATURES {// like in ADO architecture
    F_W = 0x0001, //writable
    F_R = 0x0002, //readable
    F_D = 0x0004, //discrete
    F_A = 0x0008, //archivable
    F_C = 0x0010, //config
    F_I = 0x0020, //diagnostic
    F_s = 0x0040, //savable
    F_r = 0x0080, //restorable
    F_E = 0x0100  //editable
};
#define F_WE  (F_R | F_W | F_E | F_s | F_r)
#define F_WED (F_R | F_W | F_E | F_s | F_r | F_D )

class PV {// Process Variable
  //char buf[24];
  public:
	char name[32];
	char desc[128];
	uint8_t type; //enum TYPE
	uint32_t count;
	uint16_t fbits;
	char units[8];
	int32_t opLow;
	int32_t opHigh;
	char *legalValues;
	VALUE value;

	PV(const char *aname, const char *adesc, const uint8_t atype, uint32_t acount=1,
			const uint16_t afbits = F_R, const char *aunits = "",
			int32_t aopLow = MinI32, int32_t aopHi = MaxI32, char *lv = NULL){
		strncpy(name, aname, 16);
		strncpy(desc, adesc, 128);
		type = atype;
		count = acount;
		fbits = afbits;
		strncpy(units, aunits, 8);
		opLow = aopLow;
		if (opLow < 0 and (type==T_u4 or type==T_u2 or type==T_B))
			opLow = 0;
		opHigh = aopHi;
		legalValues = lv;
	};

	int set(int v) {
		if(type == T_u4){
			if((TD_u4)opLow > (TD_u4)v or (TD_u4)v > (TD_u4)opHigh)
				return 1;
			value.u4 = (TD_u4)v;
			return 0;
		}
		if(opLow > v or v > opHigh)
			return 1;
		switch (type){
		case T_b: 	{value.b = (TD_b)v; break;}
		case T_B:	{value.B = (TD_B)v; break;}
		case T_i2:	{value.i2 = (TD_i2)v; break;}
		case T_u2:	{value.u2 = (TD_u2)v; break;}
		case T_i4:	{value.i4 = (TD_i4)v; break;}
		}
		return 0;
	}
	int set(const char* str){
		if(type == T_str){
			value.str = (TD_str)str;
			return 0;
		}
		int32_t v = atol(str);
		return set(v);
	}
	int val2strn(char* str, size_t n){
		//Convert value to string, return number of characters converted.
		switch (type) {
		case T_b:	return snprintf(str, n, "%u", value.b);
		case T_B:	return snprintf(str, n, "%u", value.B);
		case T_i2:	return snprintf(str, n, "%i", value.i2);
		case T_u2:	return snprintf(str, n, "%u", value.u2);
		case T_i4:	return snprintf(str, n, "%li", value.i4);
		case T_u4:	return snprintf(str, n, "%lu", value.u4);
		case T_str: {
			if (value.Bptr[0] == '{')
				 return snprintf(str, n, "%s", value.Bptr); //return as JSON dict
			else return snprintf(str, n, "\"%s\"", value.Bptr);
		}
		//case T_u2:	return snprintf(str, n, "%u", value.u2);
		default:	return snprintf(str, n, "Not supported type %i", type);
		}
	}
	int info(char* str, size_t n){
		//Fill string with JSON-coded info, return number of characters converted.
		char *pbuf = str;
		uint16_t cnt = 0, len = 0;
		sprintf(pbuf,"{\"%s\":{",name);
		len = strlen(pbuf);
		cnt += snprintf(pbuf+len, n-len, "\"desc\":\"%s\",\"type\":%u,\"count\":%lu,\"fbits\":%u,\"value\":",
				desc, type, count, fbits);
		len = strlen(pbuf);
		cnt += val2strn(pbuf+len, n-len);
		if(strlen(units) > 0){
			len = strlen(pbuf);
			cnt += snprintf(pbuf+len, n-len, ",\"units\":\"%s\"", units);
		}
		if(opLow != MinI32){
			len = strlen(pbuf);
			cnt += snprintf(pbuf+len, n-len, ",\"opLow\":%li", opLow);
		}
		if(opHigh != MaxI32){
			len = strlen(pbuf);
			cnt += snprintf(pbuf+len, n-len, ",\"opHigh\":%li", opHigh);
		}
		if(legalValues != NULL){
			len = strlen(pbuf);
			cnt += snprintf(pbuf+len, n-len, ",\"legalValues\":\"%s\"", legalValues);
		}
		len = strlen(pbuf);
		cnt += snprintf(pbuf+len, n-len, "}}");
		if(cnt>=n) {snprintf(str, n, "ERR: Buffer[%u] too small for %i bytes", n, cnt);}
		return cnt;
	}
};
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
#endif /* INC_MCUFEC_H_ */
