/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KG_X509_CERT_H__
#define __KG_X509_CERT_H__

#include <crypto/public_key.h>
#include <crypto/hash.h>

typedef struct kg_cert_info {
	void *tbs;
	unsigned int tbs_size;
	struct public_key pub;
	struct public_key_signature sig;
} kg_cert_info_t;

extern int kg_base64_decode(const char *src, int srclen, unsigned char *dst);
extern kg_cert_info_t *kg_x509_cert_parse(unsigned char *src, int srclen);
extern void kg_x509_cert_free(kg_cert_info_t *cert);
int kg_calc_digest(struct public_key_signature *sig);

#endif	/* __KG_X509_CERT_H__ */
