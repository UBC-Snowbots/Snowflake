#include "mos_os.h"

MOSAPI int MOSCConv
mos_isspace(int c) {

	return (c == ' ' || (c >= '\t' && c <= '\r'));
}

MOSAPI int MOSCConv
mos_isascii(int c) {

	return ((c & ~0x7f) == 0);
}

MOSAPI int MOSCConv
mos_isupper(int c) {

	return (c >= 'A' && c <= 'Z');
}

MOSAPI int MOSCConv
mos_islower(int c) {

	return (c >= 'a' && c <= 'z');
}

MOSAPI int MOSCConv
mos_isalpha(int c) {

	return (mos_isupper(c) || mos_islower(c));
}

MOSAPI int MOSCConv
mos_isdigit(int c) {

	return (c >= '0' && c <= '9');
}

MOSAPI int MOSCConv
mos_isxdigit(int c) {

	return (mos_isdigit(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'));
}

MOSAPI int MOSCConv
mos_isprint(int c) {

	return (c >= ' ' && c <= '~');
}

MOSAPI int MOSCConv
mos_isprint_utf8(int c) {

	return (c >= ' ');
}

MOSAPI int MOSCConv
mos_tolower(int c) {

	return (c + 0x20 * ((c >= 'A') && (c <= 'Z')));
}

MOSAPI int MOSCConv
mos_toupper(int c) {

	return (c - 0x20 * ((c >= 'a') && (c <= 'z')));
}

MOSAPI const char * MOSCConv
mos_uppercase(char *tbuf) {

	if (tbuf == NULL)
		return (NULL);

	while (*tbuf) {
		*tbuf = (char)mos_toupper(*tbuf);
		tbuf++;
	}

	return (tbuf);
}
