// SPDX-License-Identifier: GPL-2.0
/*
 * kg_x509_cert.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "kg_main.h"
#include "kg_x509_cert.h"

static const char base64_table[65] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int kg_base64_decode(const char *src, int srclen, unsigned char *dst)
{
	unsigned int ac = 0;
	int bits = 0;
	int i;
	unsigned char *bp = dst;

	for (i = 0; i < srclen; i++) {
		const char *p = strchr(base64_table, src[i]);

		if (src[i] == '=') {
			ac = (ac << 6);
			bits += 6;
			if (bits >= 8)
				bits -= 8;
			continue;
		}

		if (src[i] == '\n')
			continue;

		if (p == NULL || src[i] == 0)
			return -1;
		ac = (ac << 6) | (p - base64_table);
		bits += 6;
		if (bits >= 8) {
			bits -= 8;
			*bp++ = (unsigned char)(ac >> bits);
		}
	}
	if (ac & ((1 << bits) - 1))
		return -1;
	return bp - dst;
}

static void *x509_cert_end_addr;

#define CERT_BUF_OVERREAD(src, offset) ((unsigned long)(src + offset) > \
				(unsigned long)x509_cert_end_addr)

static int asn1_check_tag_and_length(const unsigned char *src, unsigned int *length)
{
	int offset = 0, num_bytes, i;
	unsigned int tmp;
	unsigned char tag;

	if (CERT_BUF_OVERREAD(src, offset)) {
		KG_LOG("%s src exceed end addr\n", __func__);
		offset = -1;
		goto out;
	}

	/* get tag */
	tag = src[offset];

	/* check tag */
	if (tag != 0x30 && tag != 0x02 && tag != 0x03 && tag != 0xa0 && tag != 0xa3) {
		KG_LOG("%s tag is wrong 0x%02x\n", __func__, tag);
		offset = -1;
		goto out;
	}

	offset++;
	if (CERT_BUF_OVERREAD(src, offset)) {
		KG_LOG("%s offset(0x%x) exceed end addr\n", __func__, offset);
		offset = -1;
		goto out;
	}

	/* get length */
	tmp = src[offset++];
	if (CERT_BUF_OVERREAD(src, offset)) {
		KG_LOG("%s offset(0x%x) exceed end addr\n", __func__, offset);
		offset = -1;
		goto out;
	}

	if (tmp & 0x80) {
		num_bytes = tmp & 0x7f;
		if (num_bytes > 4) {
			KG_LOG("%s length is wrong 0x%02x\n", __func__, num_bytes);
			offset = -1;
			goto out;
		}
		tmp = 0;
		for (i = 0; i < num_bytes; i++) {
			if (CERT_BUF_OVERREAD(src, offset)) {
				KG_LOG("%s offset(0x%x) exceed end addr\n", __func__, offset);
				offset = -1;
				goto out;
			}
			tmp = ((tmp << 8) | src[offset++]);
		}
	}

	*length = tmp;
	if (CERT_BUF_OVERREAD(src, offset + *length - 1)) {
		KG_LOG("%s length(0x%x), offset(0x%x) exceed end addr\n", __func__, *length, offset);
		offset = -1;
		goto out;
	}
out:
	return offset;
}

static int update_public_key_info(kg_cert_info_t *cert)
{
	int ret, offset, i;
	unsigned int length;
	unsigned char value;
	void *key;

	offset = 0;

	/* check tbs sequence */
	ret = asn1_check_tag_and_length(cert->tbs + offset, &length);
	if (ret < 0) {
		KG_LOG("%s failed to check tbs sequence\n", __func__);
		goto out;
	}

	offset += ret;
	/* find SubjectPublicKeyInfo sequence */
	for (i = 0; i < 7; i++) {
		ret = asn1_check_tag_and_length(cert->tbs + offset, &length);
		if (ret < 0) {
			KG_LOG("%s failed to find SubjectPublicKeyInfo\n", __func__);
			goto out;
		}
		offset += (ret + length);
	}

	offset -= length;
	/* find subjectPublicKey BIT STRING */
	for (i = 0; i < 2; i++) {
		ret = asn1_check_tag_and_length(cert->tbs + offset, &length);
		if (ret < 0) {
			KG_LOG("%s failed to find subjectPublicKey\n", __func__);
			goto out;
		}
		offset += (ret + length);
	}

	offset -= length;
	value = *(const unsigned char *)(cert->tbs + offset);
	if (length < 1 || value != 0) {
		KG_LOG("%s length or value is wrong length: 0x%x value 0x%x\n", __func__, length, value);
		goto out;
	}

	length -= 1;
	key = kzalloc(length, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(key)) {
		KG_LOG("%s failed to kzalloc key\n", __func__);
		goto out;
	}

	memcpy(key, cert->tbs + offset + 1, length);

	/* update public key info */
	cert->pub.key = key;
	cert->pub.keylen = length;
	cert->pub.id_type = "X509";
	cert->pub.pkey_algo = "rsa";

	return 0;

out:
	return -1;
}

static int update_tbs_info(kg_cert_info_t *cert, unsigned char *src, int offset)
{
	int ret;
	unsigned int length;

	/* check tbs tag */
	ret = asn1_check_tag_and_length(src + offset, &length);
	if (ret < 0) {
		KG_LOG("%s failed to tbs Certificate seq\n", __func__);
		goto out;
	}

	cert->tbs = src + offset;
	cert->tbs_size = ret + length;

	ret += length;

	if (update_public_key_info(cert)) {
		KG_LOG("%s failed to update_pub_key_info\n", __func__);
		ret = -1;
	}
out:
	return ret;
}

int kg_calc_digest(struct public_key_signature *sig)
{
	struct crypto_shash *tfm;
	struct shash_desc *desc;
	size_t desc_size;
	int ret = 0;

	tfm = crypto_alloc_shash(sig->hash_algo, 0, 0);
	if (IS_ERR(tfm)) {
		KG_LOG("%s failed to crypto_alloc_shash 0x%x\n", __func__, PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	desc_size = crypto_shash_descsize(tfm) + sizeof(*desc);
	sig->digest_size = crypto_shash_digestsize(tfm);

	ret = -ENOMEM;
	sig->digest = kmalloc(sig->digest_size, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(sig->digest)) {
		KG_LOG("%s failed to kmalloc digest_size: 0x%x\n", __func__, sig->digest_size);
		goto error;
	}

	desc = kzalloc(desc_size, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(desc)) {
		KG_LOG("%s failed to kzalloc desc_size: 0x%x\n", __func__, desc_size);
		goto error;
	}

	desc->tfm = tfm;

	ret = crypto_shash_digest(desc, sig->data, sig->data_size, sig->digest);
	if (ret < 0)
		KG_LOG("%s failed to crypto_shash_digest : 0x%x\n", __func__, ret);

	kfree(desc);
error:
	crypto_free_shash(tfm);

	return ret;
}

static int update_sig_info(kg_cert_info_t *cert, unsigned char *src, int offset)
{
	int ret;
	unsigned int length;
	unsigned char *s;

	/* check signature tag */
	ret = asn1_check_tag_and_length(src + offset, &length);
	if (ret < 0) {
		KG_LOG("%s failed to signature seq\n", __func__);
		goto out;
	}

	s = kmalloc(length - 1, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(s)) {
		KG_LOG("%s failed to kmalloc size: %d\n", __func__, length - 1);
		ret = -ENOMEM;
		goto out;
	}

	/* copy signature for public_key_verify_signature */
	memcpy(s, src + offset + ret + 1, length - 1);

	cert->sig.s = s;
	cert->sig.s_size = length - 1;
	cert->sig.pkey_algo = "rsa";
	cert->sig.hash_algo = "sha256";
	cert->sig.encoding = "pkcs1";
	cert->sig.data = cert->tbs;
	cert->sig.data_size = cert->tbs_size;

	ret += length;
	if (kg_calc_digest(&cert->sig))
		ret = -1;

out:
	return ret;
}

void kg_x509_cert_free(kg_cert_info_t *cert)
{
	if (!ZERO_OR_NULL_PTR(cert)) {
		kfree(cert->pub.key);
		kfree(cert->sig.digest);
		kfree(cert->sig.s);
		kfree(cert);
	}
}

kg_cert_info_t *kg_x509_cert_parse(unsigned char *src, int srclen)
{
	int ret, offset = 0;
	unsigned int length;
	kg_cert_info_t *cert = kzalloc(sizeof(kg_cert_info_t), GFP_KERNEL);

	if (!src || !srclen) {
		KG_LOG("%s src is NULL or srclen is 0\n", __func__);
		goto out;
	}

	if (ZERO_OR_NULL_PTR(cert)) {
		KG_LOG("%s cert is NULL\n", __func__);
		goto out;
	}

	x509_cert_end_addr = src + srclen - 1;
	/* check Certificate sequence */
	ret = asn1_check_tag_and_length(src + offset, &length);
	if (ret < 0) {
		KG_LOG("%s failed to check Certificate seq\n", __func__);
		goto out;
	}

	offset += ret;
	ret = update_tbs_info(cert, src, offset);
	if (ret < 0)
		goto out;

	offset += ret;
	/* skip signatureAlgorithm sequence */
	ret = asn1_check_tag_and_length(src + offset, &length);
	if (ret < 0) {
		KG_LOG("%s failed to signatureAlgorithm seq\n", __func__);
		goto out;
	}

	offset += (ret + length);
	ret = update_sig_info(cert, src, offset);
	if (ret < 0)
		goto out;

	offset += ret;
	if (offset != srclen) {
		KG_LOG("%s 0x%x != 0x%x\n", __func__, offset, srclen);
		goto out;
	}

	return cert;
out:
	kg_x509_cert_free(cert);
	return NULL;
}

