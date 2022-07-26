#ifndef _PKCS5_PBKDF2_H_
#define _PKCS5_PBKDF2_H_

/*
 * pass, pass len, salt, salt len, key, key len, rounds
 *
 * Recommend rounds > 10000.
 */
int
pkcs5_pbkdf2(const char *, size_t, const char *, size_t, uint8_t *, size_t,
  unsigned int);

#endif /* _PKCS5_PBKDF2_H_ */
