#ifndef _MOS_HEXDUMP_H_
#define _MOS_HEXDUMP_H_

MOSAPI void MOSCConv mos_hexdump(const void *, size_t);
MOSAPI void MOSCConv mos_hexdumpstr(const void *, size_t, char *, size_t);

MOSAPI size_t MOSCConv mos_escape_string(const uint8_t *, ssize_t, uint8_t *, ssize_t);
MOSAPI size_t MOSCConv mos_data2hex(const uint8_t *, size_t, uint8_t *, size_t);
MOSAPI size_t MOSCConv mos_hex2data(const uint8_t *, size_t, uint8_t *, size_t);

#endif /* _MOS_HEXDUMP_H_ */
