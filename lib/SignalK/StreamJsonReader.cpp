/*
 * dinamic_string.cpp
 *
 *  Created on: 07/03/2014
 *      Author: laso
 */
/*  Originally copied from http://blog.biicode.com/arduino-parse-json/
 *  No licence mentioned on site - attempted to contact owner
 *  Mods to avoid cstringlibs
 *  Mods to suit freeboard data model
 *
 */

#include "StreamJsonReader.h"


	StreamJsonReader::StreamJsonReader(HardwareSerial* serial, SignalkModel* model) {//:str(outStr, sizeof(outStr)){

		this->ignore_node=false;
		this->serial=serial;
		this->model=model;
		alloc_elements();

	}

	StreamJsonReader::~StreamJsonReader(){
		free_elements();
	}

	void StreamJsonReader::free_elements(){


	}

	void StreamJsonReader::alloc_elements(){

		this->status = NODE_VALUE;
		this->element_type = TYPE_DICT;
		this->element_value[0] = '\0';
		this->element_name[0] = '\0';
		this->trace[0] = '\0';

	}

	void StreamJsonReader::reset(){
		//serial->println(str);
		free_elements();
		status = NODE_VALUE;
		alloc_elements();
		//str.begin();

	}

	// MATCH PARTIAL

	void StreamJsonReader::assign_result(const char* result ){

				//put it straight into the Signalk model, truncate vessels.self.
		//TODO: deal with 'null' values, ints, and longs
				unsigned long hashKey = model->hash(&trace[13]);
				switch(element_type){
					case TYPE_STRING:
						model->setValue(hashKey, result);
						break;
					case TYPE_NUMERIC:
						//handle null
						if(strcmp("null", result)==0){
							//TODO: how to handle numeric null??
							//str.println(" #skipping null");
							break;
						}
						switch(model->getNumericType(hashKey)){
							case TYPE_FLOAT:
								model->setValue(hashKey, (float)atof(result));
								break;
							case TYPE_UNSIGNED_LONG:
								model->setValue(hashKey,strtoul(result,NULL,10));
								break;
							case TYPE_INT:
								model->setValue(hashKey, atoi(result));
								break;
							case TYPE_DOUBLE:
								model->setValue(hashKey, atof(result));
								break;
							default:
								break;
						}
						break;
					case TYPE_BOOLEAN:
						model->setValue(hashKey, (strcmp(result,"true")==0));
						break;
				}
				return;

	}

	//only accept vessels.self structure
	bool StreamJsonReader::partial_query_match(char* trace){
		return true;
	}



	bool StreamJsonReader::is_numeric_last_trace_element(){
		/*int size = strlen(trace);
		if(size==0){
			return false;
		}
		for(int i=size-1; i >= 0; i--){
			if(trace[i] == '.'){
				return true;
			}
			if(!(trace[i] >= '0' && trace[i] <= '9')){
				return false;
			}
		}
		return true;*/
		return false;
	}

	int StreamJsonReader::numeric_last_trace_element_value(){
		//cosa.1  => 1
		int size = strlen(trace);
		int value = 0;
		if(!is_numeric_last_trace_element()){
			return -1;
		}
		for(int i=size-1; i >= 0; i--){
			if(trace[i] != '.'){
				value += atoi(trace[i]) * pow(10, (size-1-i));
			}
			else{
				return value;
			}
		}
		return -1;
	}


	void StreamJsonReader::increment_trace_element(){
		//cosa.1 to cosa.2
		int value = numeric_last_trace_element_value();
		if(value == -1){
			return;
		}
		int size = strlen(trace);
		char as_string[5]; // limited to 5 numbers indexes
		int_to_string(value + 1, as_string);

		for(unsigned int i=size-1; i >= 0; i--){
			if(trace[i] == '.'){
				for(unsigned int j=0; j <= strlen(as_string); j++){ //copy \0 too
					if(i+j+2 < JSON_MAX_TRACE){
						trace[i+1+j] = as_string[j];
					}
				}
				return;
			}
		}
		return;
	}

	void StreamJsonReader::remove_last_trace_element(){
		int size = strlen(trace);
		int i;
		for(i=size-1; i >= 0; i--){
			if(trace[i] == '.'){
				trace[i] = '\0';
				ignore_node = !partial_query_match(trace);
				return;
			}
		}
		trace[0] = '\0';
		ignore_node = !partial_query_match(trace);

	}

	void StreamJsonReader::add_to_trace(char* element){

		if(strlen(element) == 0){
			return;
		}
		int size = strlen(trace);
		if(size > 0){
			if(!append_to_trace('.')){
				return;
			}
			size++;
		}
		for(unsigned int i=0; i < strlen(element); i++){ //copy \0
			append_to_trace(element[i]);
		}
		ignore_node = !partial_query_match(trace);

	}

	int StreamJsonReader::process_char(char c){

		//str.print(c);
		switch(status){
			case NODE_VALUE: //At the beginning is a node value of root. Wait for { or [ or number or "
				switch(c){
					case '{': status = READ_ELEMENT; element_type = TYPE_DICT; break;
					case '[': status = NODE_VALUE; add_to_trace((char*) "0"); break;
					case '"': status = READ_ELEMENT; element_type = TYPE_STRING; element_value[0] = '\0'; break;
					case ']': remove_last_trace_element(); status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE; break; //Empty array
					case 't': //true
						status = READ_ELEMENT;
						element_type = TYPE_BOOLEAN;
						append_to_value(c);
						break;
					case 'f': //false
						status = READ_ELEMENT;
						element_type = TYPE_BOOLEAN;
						append_to_value(c);
						break;
					default:
						//consider null here
						if((c >= '0' && c <= '9') || c == '-' || c == 'n'){
							 status = READ_ELEMENT;
							 element_type = TYPE_NUMERIC;
							 element_value[0] = '\0';
							 append_to_value(c); //must be cleared before here
						}
						else{
							return -1;
						}
						break;
				}
				break;
			case READ_ELEMENT: //We are reading an element of type element_type

				switch(element_type){
					case TYPE_DICT:
						// Inside a dict we only expect a element name
						if(c == '}'){
							add_to_trace((char*) "0");
							status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE;
							return process_char(c);
						}
						if(c != '"') return -1;
						status = READING_ELEMENT_KEY;
						element_name[0] = '\0';
						break;
					case TYPE_STRING:
						// Inside a string, concat until "
						if(c != '"'){
							append_to_value(c);
						}
						else{
							assign_result(element_value);
							element_value[0] = '\0';
							status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE;
							return process_char(c); // Advance one status
						}
						break;
					case TYPE_NUMERIC:
						// Inside a numeric, concat until } or ,
						if(c != '}' && c != ',' && c != ']') {
							append_to_value(c);
						}else{
							assign_result(element_value);
							element_value[0] = '\0';
							status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE;
							return process_char(c); // Advance one status
						}
						break;
					case TYPE_BOOLEAN:
							// Inside a boolean, concat until } or ,
							if(c != '}' && c != ',' && c != ']')
								append_to_value(c);
							else{
								assign_result(element_value);
								element_value[0] = '\0';
								status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE;
								return process_char(c); // Advance one status
							}
							break;
				}//end switch
				break;
			case WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE:

				if(c == '}' || c == ']'){
					status = WAIT_FOR_ANOTHER_ELEMENT_OR_CLOSE;
					remove_last_trace_element(); // parent value ended
					//if(is_numeric_last_trace_element()){
					//	element_type = TYPE_ARRAY;
					//}
					//else{
						element_type = TYPE_DICT;
					//}
				}
				else if(c == ',' || c == '"'){
					//if(!is_numeric_last_trace_element()){
						status = READ_ELEMENT;
						remove_last_trace_element(); // value with key ended
						element_type = TYPE_DICT;
					//}
					//else{
					//	status = NODE_VALUE; // Any type of value
					//	increment_trace_element();
					//	element_type = TYPE_ARRAY;
					//}
				}else{
					//end of json?
					status = NODE_VALUE;
				}

				break;
			case READING_ELEMENT_KEY:
				if(c != '"'){
					append_to_name(c);
				}
				else{
					status = WAIT_FOR_TWO_POINTS; // !!!!!! READ FOR MATCH NAME!!
					add_to_trace(element_name);
					element_name[0] = '\0';
				}
				break;

			case WAIT_FOR_TWO_POINTS:
				if(c != ':') return -1;
				else status = NODE_VALUE;
				break;
		}

		return 0;
	}

	bool StreamJsonReader::append_to_trace(char c){
		return append_to_string(this->trace, c, JSON_MAX_TRACE);
	}

	bool StreamJsonReader::append_to_value(char c){
		return append_to_string(this->element_value, c, JSON_MAX_VALUE);

	}

	bool StreamJsonReader::append_to_name(char c){
		return append_to_string(this->element_name, c, JSON_MAX_NAME);

	}


	int atoi(char c){
		return c - '0';
	}


	 void StreamJsonReader::int_to_string(int value, char* buffer){
	        sprintf(buffer, "%d", value);
	    }


	    bool StreamJsonReader::starts_with(const char* string, const char* other){
	        for(unsigned int i=0; i < strlen(other); i++){ //dont compare \0
	            if(string[i] != other[i]){
	                return false;
	            }
	        }
	        return true;
	    }

	    bool StreamJsonReader::append_to_string(char* string, const char c, int max_size){
	        int size = strlen(string);
	        string[size] = c;
	        string[size + 1] = '\0';
	        return true;
	    }

