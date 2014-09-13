/*
 * dinamic_string.h
 *
 *  Created on: 07/03/2014
 *      Author: laso
 */
/*  Copied from http://blog.biicode.com/arduino-parse-json/
 *  No licence mentioned on site - attempting to contact owner
 *
 */

#ifndef STREAM_JSON_READER_H_
#define STREAM_JSON_READER_H_

#define NODE_VALUE 0 // Wait for { or [ or number or "
#define READ_ELEMENT 1 // Wait for
#define READING_ELEMENT_KEY 2
#define WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE 4 // Wait for , or }, something before an element value

#define WAIT_FOR_TWO_POINTS 6 // Readed a complete key and we are in a parent dict. Wait for the ":" before node value

#define TYPE_DICT 0
#define TYPE_STRING 1
#define TYPE_NUMERIC 2
#define TYPE_BOOLEAN 3
#define TYPE_ARRAY 4

#define  JSON_MAX_NAME 30
#define JSON_MAX_VALUE 30
#define JSON_MAX_TRACE 50

#include <HardwareSerial.h>
#include "SignalkModel.h"
#include <PString.h>



class StreamJsonReader {
public:
	// initial_name_size: Initial allocated bytes to field names in json (will be realocated if needed)
	// initial_value_size: Initial allocated bytes to values in json (will be realocated if needed)
	// initial_trace_size: Initial allocated bytes to path in json (will be realocated if needed)
	StreamJsonReader(HardwareSerial* serial, SignalkModel* model);
	virtual ~StreamJsonReader();
	int process_char(char c);
	void reset();


private:
	HardwareSerial* serial;
	SignalkModel* model;
	short int status;
	short int element_type;

	char element_name[JSON_MAX_NAME];
	char element_value[JSON_MAX_VALUE];
	char trace[JSON_MAX_TRACE];
	//char outStr [2500];
	//PString str;
	bool is_numeric_last_trace_element();
	void increment_trace_element();
	int numeric_last_trace_element_value();
	void remove_last_trace_element();
	void add_to_trace(char* element);

	void assign_result(const char* result);
	bool partial_query_match(char* trace);

	bool append_to_trace(char c);
	bool append_to_name(char c);
	bool append_to_value(char c);

	bool ignore_node; // If user don't want current node, don't read child

	void alloc_elements();
	void free_elements();

	//stringutils
	void int_to_string(int value, char* buffer);

	//bool equal_string(const char* string, const char* other);
	bool starts_with(const char* string, const char* other);
	bool append_to_string(char* string, const char c, int max_size);
};

//void append_to(char* string, char c, int max_size);
int atoi(char c);


#endif /* STREAM_JSON_READER_H_ */
