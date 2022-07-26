#ifndef _JSON_H_
#define _JSON_H_

#include "phidgetbase.h"
#include "mos/mos_os.h"

/*
 * Should be the same as the max bridge packet string entry size.
 */
#define JSON_STRING_MAX	16384

int parseJSON(const char *, uint32_t, char *, size_t, const char *, ...) SCANF_LIKE(5, 6);
int parseJSONv(const char *, uint32_t, char *, size_t, const char *, va_list);
int mkJSON(char *, uint32_t, const char *, ...) PRINTF_LIKE(3, 4);
int mkJSONv(char *, uint32_t, const char *, va_list);
const char *mkJSONc(char *, uint32_t, const char *, ...) PRINTF_LIKE(3, 4);
API_CRETURN_HDR json_escape(const char *, char *, size_t);
API_CRETURN_HDR json_unescape(char *);

#endif /* _JSON_H_ */

