 /******************** (C) Trio Mobil 2014 LPC1768 MCU   **********************
 * File Name          : P65 Util Functions
 * Author			  : Nevzat Ataklı
 * Version            : V1.0.0
 * Date               : 2014/01/04
 ****************************************************************************/

#include "Utils.h"
//#define OKKA_DEMO 1


void UTIL_appendZerosAndAddAppendix(char *source, char* appendix, int len) {
	int zero_count = len - strlen(appendix);
	int i;
	for (i = 0; i < zero_count; i++)
		source[i] = '0';
	for (i = zero_count; i < len; i++)
		source[i] = appendix[i];
}

int UTIL_isNumeric(const char * s) {
	if (s == NULL || *s == '\0' || isspace(*s))
		return 0;
	char * p;
	strtod(s, &p);
	return *p == '\0';
}


void UTIL_GetPartOfString(char*source, char*dest, char start_chr, char end_chr,
		int first_included, int last_included) {
	char buffer[50];
	int count;
	//Trace("1");
	char* start = strchr(source, start_chr);
	//Trace("2");
	char* end = strchr(source, end_chr);
	//Trace("3");
	int length;
	//Trace("4");
	if (first_included == 0)
		start++;
	//Trace("5");
	if (start == NULL || end == NULL) {
		return;
	}
	/*
	 if (end == NULL){
	 end = source[strlen(source)-1];
	 Trace("C");
	 }*/
	//Trace("7");
	length = end - start;
	//Trace("8");
	if (last_included > 0) {
		length = length + 1;
		//Trace("9");
	}

	count = sprintf(buffer, "-%d-", length);
	//UARTSend(PORT_TRACE,buffer,count);
	strncpy(dest, start, length);
	//Trace("A");
	//Terminate with null.
	dest[length] = '\0';
	//Trace("B");
}



void UTIL_getFromSplitStr(char* delim, int index, char* source, char* result) {
	char *temp = strtok(source, delim);
	int i = 0;
	while( temp != NULL ) {
		if ( i==index ){
			strcpy(result, temp);
			break;
		}
		temp = strtok(NULL, delim);
		i++;
	}
}

float median(int size, float x[]) {
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<size-1; i++) {
        for(j=i+1; j<size; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(size%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[size/2] + x[size/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[size/2];
    }
}
