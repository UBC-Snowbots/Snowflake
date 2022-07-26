#include "mos_iop.h"

typedef enum {
	FMT_A = 'A',
	FMT_a = 'a',
	FMT_B = 'B',
	FMT_C = 'C',
	FMT_E = 'E',
	FMT_e = 'e',
	FMT_F = 'F',
	FMT_f = 'f',
	FMT_G = 'G',
	FMT_g = 'g',
	FMT_H = 'H',
	FMT_I = 'I',
	FMT_J = 'J',
	FMT_K = 'K',
	FMT_k = 'k',
	FMT_L = 'L',
	FMT_M = 'M',
	FMT_m = 'm',
	FMT_O = 'O',
	FMT_P = 'P',
	FMT_Q = 'Q',
	FMT_q = 'q',
	FMT_R = 'R',
	FMT_U = 'U',
	FMT_V = 'V',
	FMT_v = 'v',
	FMT_W = 'W',
	FMT_w = 'w',
	FMT_Y = 'Y',
	FMT_Z = 'Z'
} mos_fmt_char_t;

/* registerable formatter function for mos_printf() */
typedef const char *(*formatter_t)(void *, char *, size_t *);

MOSAPI int MOSCConv mos_register_formatter(mosiop_t, mos_fmt_char_t, formatter_t);
MOSAPI int MOSCConv mos_unregister_formatter(mosiop_t, mos_fmt_char_t);
