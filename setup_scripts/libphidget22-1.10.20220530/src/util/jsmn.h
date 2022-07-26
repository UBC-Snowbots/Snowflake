#ifndef __JSMN_H_
#define __JSMN_H_

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define JSMN_STRICT
#define JSMN_PARENT_LINKS

#ifdef __cplusplus
extern "C" {
#endif

/**
 * JSON type identifier. Basic types are:
 * 	o Object
 * 	o Array
 * 	o String
 * 	o Other primitive: number, boolean (true/false) or null
 */
	typedef enum {
		JSMN_PRIMITIVE = 0,
		JSMN_OBJECT = 1,
		JSMN_ARRAY = 2,
		JSMN_STRING = 3
	} pjsmntype_t;

	typedef enum {
		/* Not enough tokens were provided */
		JSMN_ERROR_NOMEM = -1,
		/* Invalid character inside JSON string */
		JSMN_ERROR_INVAL = -2,
		/* The string is not a full JSON packet, more bytes expected */
		JSMN_ERROR_PART = -3
	} pjsmnerr_t;

	/**
	 * JSON token description.
	 * @param		type	type (object, array, string etc.)
	 * @param		start	start position in JSON data string
	 * @param		end		end position in JSON data string
	 */
	typedef struct {
		pjsmntype_t type;
		int start;
		int end;
		int size;
#ifdef JSMN_PARENT_LINKS
		int parent;
#endif
	} pjsmntok_t;

	/**
	 * JSON parser. Contains an array of token blocks available. Also stores
	 * the string being parsed now and current position in that string
	 */
	typedef struct {
		unsigned int pos; /* offset in the JSON string */
		unsigned int toknext; /* next token to allocate */
		int toksuper; /* superior token node, e.g parent object or array */
	} pjsmn_parser;

	/**
	 * Create JSON parser over an array of tokens
	 */
	void pjsmn_init(pjsmn_parser *parser);

	/**
	 * Run JSON parser. It parses a JSON data string into and array of tokens,
	 * each describing a single JSON object.
	 */
	pjsmnerr_t pjsmn_parse(pjsmn_parser *parser, const char *js, size_t len,
	  pjsmntok_t *tokens, unsigned int num_tokens);

	char *pjsmn_string(const char *, pjsmntok_t *, char *, size_t);
	int pjsmn_double(const char *, pjsmntok_t *, double *);
	int pjsmn_uint64(const char *, pjsmntok_t *, uint64_t *);
	int pjsmn_int64(const char *, pjsmntok_t *, int64_t *);
	int pjsmn_boolean(const char *, pjsmntok_t *);

#ifdef __cplusplus
}
#endif

#endif /* __JSMN_H_ */
