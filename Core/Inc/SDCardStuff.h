/*
 * SDCardStuff.h
 *
 */

#ifndef SRC_SDCARDSTUFF_H_
#define SRC_SDCARDSTUFF_H_

int formatSD(){

	/*	formatSD
	 *
	 * 	desc:		formats the SD card to FAT file system
	 *
	 * 	inputs: 	none
	 *
	 * 	returns:	int
	 * 				 0	:	returns 0 if successful
	 * 				-1	:	returns -1 if file system cannot be made
	 * 				-2	:	returns -2 if SD card cannot be mounted
	 * 				-3	:	returns -3 if SD card cannot be unmounted
	 *
	 */

	uint8_t rtext[_MAX_SS];/* File read buffer */
	FRESULT res; /* FatFs function common result code */

	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //Toggle the state of pin BLU

	res = f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)); // Format the file
	if(res != FR_OK){return -1;}

	res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);	// mount the SD card
	if(res != FR_OK){return -2;}

	res = f_mount(0, (TCHAR const*)SDPath, 1);	// unmount the SD card
	if(res != FR_OK){return -3;}

	return 0;
}

int writeColSD(float fileName, float data[], int noElements){

	/*	writeColSD
	 *
	 * 	desc:		takes one array of floats, writes it to a file in the SD card.
	 *
	 * 	inputs: 	float filename	:	name of file to write, '.csv' will be appended to it
	 * 				float* data  	:	an array containing the floats to be written to file
	 * 				int noElements	:	an int representing the number of elements to write
	 *
	 * 	returns:	int
	 * 				-1	:	returns -1 if SD card cannot be mounted
	 * 				-2	:	returns -2 if the file to be written cannot be opened/created
	 * 				-3	:	returns -3 if a write operation fails
	 * 				-4 	: 	returns -4 if the file close operation fails
	 * 				-5	: 	returns -5 if the SD card unmount fails
	 * 				else:	returns number of bytes written to the SD card
	 *
	 */

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten = 0; /* File write/read counts */

	char buffer[_MAX_SS];
	char subBuf[_MAX_SS];
	int itt = 0;
	int offset = 0;
	int bytes = 0;


	res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);	// mount the SD card
	if(res != FR_OK){return -1;}

	sprintf(buffer, "%f.csv", fileName);	// Print filename into buffer
	//sprintf(buffer, "%d.txt", fileName);

	////////////// should have a case for if file was created or not

	res = f_open(&SDFile, buffer, FA_OPEN_APPEND | FA_WRITE); //	opens file, if exists append, ow create
	if(res != FR_OK){return -2;}

	bytes = sprintf(subBuf, "%d lines of data\n", noElements);	//  prints number of lines into buffer

	res = f_write(&SDFile, subBuf, bytes, (void *)&byteswritten);	// prints buffer to file
	if((byteswritten == 0) || (res != FR_OK)){return -3;}

		while(itt < noElements){	// while there are elements to print

			int itc = 0;			// index for buffer
			while((itc < _MAX_SS) && (itt < noElements)){	// will loop until the buffer is full or out of elements

				int inc = offset;	// index for subBuffer
				while(offset != 0){
					buffer[itc] = subBuf[inc];	// this exists in the event a float gets written in seperate packets.
					++itc;
					++inc;

					if(subBuf[inc -1] == '\n'){
						inc = 0;
						offset = 0;
						++itt;
					}
				}

				if(data[itt] == 50){ // for breakpoint
					int boof = 0;	// not important for production
				}

				bytes = sprintf(subBuf, "%f\n", data[itt]) + itc; // prints the next float into the subbuffer and keeps track of bytes written

				while((itc < bytes) && (itc < _MAX_SS)){ //copy subbuffer to buffer
					buffer[itc] = subBuf[inc];
					++itc;
					++inc;
				}

				  if(subBuf[inc -1] == '\n'){	// if the last char was a newline reset the subBuffer index
						inc = 0;
						++itt;
				  }
				  offset = inc;	// if inc isnt 0, then the last char wasnt newline and there is still data in subbuffer

			}

			res = f_write(&SDFile, buffer, itc, (void *)&byteswritten);	// write buffer to SD
			if((byteswritten == 0) || (res != FR_OK)){return -3;} // if write fails return an error

//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //Toggle the state of pin BLU
	  }

	  res = f_close(&SDFile);	// close the file
	  if(res != FR_OK){return -4;}

	  res = f_mount(0, (TCHAR const*)SDPath, 1); // unmount SD
	  if(res != FR_OK){return -5;}

	  return byteswritten;
}

int writeSD(float fileName, float data0[], float data1[], float data2[], float data3[], float data4[], float data5[], float data6[], float data7[], float data8[], float data9[], float data10[], int noElements){

	/*	writeSD
	 *
	 * 	desc:		takes eleven array of floats, writes them to a csv file in the SD card.
	 *
	 * 	inputs: 	float filename			:	name of file to write, '.csv' will be appended to it
	 * 				float* data0 - data10  	:	arrays containing the floats to be written to file. Should all be same length
	 * 				int noElements			:	an int representing the number of elements from each array to write
	 *
	 * 	returns:	int
	 * 				-1	:	returns -1 if SD card cannot be mounted
	 * 				-2	:	returns -2 if the file to be written cannot be opened/created
	 * 				-3	:	returns -3 if a write operation fails
	 * 				-4 	: 	returns -4 if the file close operation fails
	 * 				-5	: 	returns -5 if the SD card unmount fails
	 * 				else:	returns number of bytes written to the SD card
	 *
	 */

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten = 0; /* File write/read counts */
	char buffer[_MAX_SS];
	char subBuf[_MAX_SS];
	int itt = 0;
	int offset = 0;
	int bytes = 0;


	res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);	// mount the SD card
	if(res != FR_OK){return -1;}

	sprintf(buffer, "%f.csv", fileName);	// Print filename into buffer
	//sprintf(buffer, "%d.txt", fileName);

	res = f_open(&SDFile, buffer, FA_OPEN_APPEND | FA_WRITE); //	opens file, if exists append, ow create
	if(res != FR_OK){return -2;}

	// Header data to print
	bytes = sprintf(subBuf, "%d lines of data\nX Acc,Y Acc,Z Acc,Orientation1,Orientation2,Orientation3,Orientation4,Mag1,Mag2,Mag3, Time\n", noElements);

	res = f_write(&SDFile, subBuf, bytes, (void *)&byteswritten); // write header file
	if((byteswritten == 0) || (res != FR_OK)){return -3;} // if write fails throw error

	while(itt < noElements){	// do while there are elements to print

		int itc = 0;	// buffer index
		while((itc < _MAX_SS) && (itt < noElements)){	// do while there are elements to print and the buffer is not full

			int inc = offset;	// subbuffer index
			while(offset != 0){	// exists in case a float is cut off in the previous write
				buffer[itc] = subBuf[inc];
				++itc;
				++inc;

				if(subBuf[inc -1] == '\n'){
					inc = 0;
					offset = 0;
					++itt;
				}
			}

			if(data0[itt] == 50){ // for breakpoint
				int boof = 0;		// not used in production
			}

			bytes = sprintf(subBuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", data0[itt], data1[itt], data2[itt], data3[itt], data4[itt], data5[itt], data6[itt], data7[itt], data8[itt], data9[itt], data10[itt]) + itc;
			// prints the floats at the index in each data array to subbuffer

			while((itc < bytes) && (itc < _MAX_SS)){ //copy subbuffer to buffer
				buffer[itc] = subBuf[inc];
				++itc;
				++inc;
			}

			if(subBuf[inc -1] == '\n'){	// if last charachter was a newline reset subbuffer index
				inc = 0;
				++itt;
			}
			offset = inc;	// if last character was not newline this will save the index of the subbuffer

		}

		res = f_write(&SDFile, buffer, itc, (void *)&byteswritten); // write buffer to SD
		if((byteswritten == 0) || (res != FR_OK)){return -3;}	// if fails return error

//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //Toggle the state of pin BLU
	  }

	  res = f_close(&SDFile);	// close the SD file
	  if(res != FR_OK){return -4;}

	  res = f_mount(0, (TCHAR const*)SDPath, 1);	// unmount the SD card
	  if(res != FR_OK){return -5;}

	  return byteswritten;
}

int readNoRowsSD(float fileName){

	/*	ReadNoRowsSD
	 *
	 * 	desc:		takes a filename, and returns the number of rows in the file. The number of rows should be written
	 * 				in the first line of the file.
	 *
	 * 	inputs: 	float filename	:	name of file to write, '.csv' will be appended to it
	 *
	 * 	returns:	int
	 * 				-1	:	returns -1 if SD card cannot be mounted
	 * 				-2	:	returns -2 if the file to be written cannot be opened/created
	 * 				-3 	: 	returns -4 if the file close operation fails
	 * 				-4	: 	returns -5 if the SD card unmount fails
	 * 				else:	returns number of rows in file as read from first line of the file
	 *
	 */

	FRESULT res; /* FatFs function common result code */
	char buffer[_MAX_SS];

	res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1); // mount the SD card
	if(res != FR_OK){return -1;}	// return Error if fails

	sprintf(buffer, "%f.csv", fileName);	// prints the filename into the buffer
	//sprintf(buffer, "%d.txt", fileName);

	res = f_open(&SDFile, buffer, FA_READ);	// open the file
	if(res != FR_OK){return -2;}

	int noRows = 0;
	if(!f_eof(&SDFile)){
		f_gets(buffer, _MAX_SS, &SDFile); // get the first line from the file
		noRows = atoi(buffer);			// turn first line from file into int
	}

	res = f_close(&SDFile);	// close the file
	if(res != FR_OK){return -3;}	// if fails return error

	res = f_mount(0, (TCHAR const*)SDPath, 1);	// unmount SD card
	if(res != FR_OK){return -4;}	// if fails return error

	return noRows;

}

int readSD(float fileName, float data[], int noElements, int column){

	/*	readSD
		 *
		 * 	desc:		takes a csv file on the SD cart and reads one column of the file into an array of floats.
		 *
		 * 	inputs: 	float filename	:	name of file to write, '.csv' will be appended to it
		 * 				float* data  	:	an array containing the floats to be written to file
		 * 				int noElements	:	an int representing the number of elements to write
		 * 				int column		:	the column from from left to be read. Leftmost column is 0
		 *
		 * 	returns:	int
		 * 				-1	:	returns -1 if SD card cannot be mounted
		 * 				-2	:	returns -2 if the file to be written cannot be opened/created
		 * 				-3	:	returns -3 if a read operation fails
		 * 				-4 	: 	returns -4 if the file close operation fails
		 * 				-5	: 	returns -5 if the SD card unmount fails
		 * 				else:	returns number of bytes written to the SD card
		 *
		 */


	FRESULT res; /* FatFs function common result code */
	char buffer[_MAX_SS];
	char subBuf[50];
	int itt = 0;
	int offset = 0;
	int comma = 0;

	res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);	// mount the SD card
	if(res != FR_OK){return -1;}	// if fails return error

	sprintf(buffer, "%f.csv", fileName);	// print filename to buffer
	//sprintf(buffer, "%d.txt", fileName);

	res = f_open(&SDFile, buffer, FA_READ);	// open the file
	if(res != FR_OK){return -2;}	// if fails return error

	//Read text from file and store in data[]

	f_gets(buffer, _MAX_SS, &SDFile); // get rid of header data
	f_gets(buffer, _MAX_SS, &SDFile); // get rid of header data

	while((!f_eof(&SDFile)) && itt <= noElements){

		UINT bytesRead = 0;
		// get the line from the file
		f_read(&SDFile, buffer, _MAX_SS, &bytesRead);	// read 512 bytes from the file
		if(res != FR_OK){return -3;}

/*	This section Probably not useful anymore. Feel bad deleting it.
 *
		while(place < bytesRead){
			int itc = 0;
			int inc = offset;
			while(itc < bytesRead){
				subBuf[inc] = buffer[itc];
				++itc;
				++inc;

				if(subBuf[inc -1] == '\n'){
					float test = atof(subBuf);
					data[itt] = test;
					inc = 0;

					++itt;
			}
		}
*/

		int itc = 0;
		int inc = offset;
		while(itc < bytesRead){

			if(comma == column){	// checks for row of sd file by counting commas
				subBuf[inc] = buffer[itc];	// if in right row, copy chars into subbuffer
				++inc;
				if((subBuf[inc -1] == '\n') || (subBuf[inc -1] == ',')){	// if end of row,
					float test = atof(subBuf);	// turn float in subbuffer into float
					data[itt] = test;	// save float to array
					inc = 0;
					++itt;
				}

			}
			++itc;

			if((buffer[itc -1] == ',')){	// counts commas
				++comma;
			}

			if((buffer[itc -1] == '\n')){	// resets commas at end of line
				comma = 0;
			}

		}

		offset = inc;

//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //Toggle the state of pin RED
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); //Toggle the state of pin BLU

	}

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	res = f_close(&SDFile);	// close the file
	if(res != FR_OK){return -4;}	// if fails return error

	//res = f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
	res = f_mount(0, (TCHAR const*)SDPath, 1);	// unmount sd card
	if(res != FR_OK){return -5;}	// if fails return error

	return itt;
}

#endif /* SRC_SDCARDSTUFF_H_ */
