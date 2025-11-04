// SPDX-License-Identifier: GPL-2.0
/*
 * kg_dr.c/
 *
 * Copyright (C) 2025 Samsung Electronics
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "kg_main.h"
#include "kg_dr.h"
#include "kg_dr_cert.h"
#include "kg_x509_cert.h"
#include "kg_key_info.h"

#define BEGIN_CERT_TEXT		"-----BEGIN CERTIFICATE-----\n"
#define END_CERT_TEXT		"-----END CERTIFICATE-----"

#define BEGIN_CERT_TEXT_LEN	(sizeof(BEGIN_CERT_TEXT) - 1)

static bool dr_initialized;
static kg_cert_info_t *interm_cert;
static kg_cert_info_t *leaf_cert;

static kg_cert_info_t *kg_parse_leaf_cert(const char *buf, int src_len)
{
	int cert_size, begin_offset = 0, encoded_len = src_len;
	char *cert_der = kmalloc(src_len, GFP_KERNEL);
	char *end;
	kg_cert_info_t *cert = NULL;

	if (ZERO_OR_NULL_PTR(cert_der)) {
		KG_LOG("%s failed to alloc cert_der %d\n", __func__, src_len);
		goto buf_alloc_fail;
	}

	/* remove begin text if exist */
	if (strstr(buf, BEGIN_CERT_TEXT)) {
		begin_offset = BEGIN_CERT_TEXT_LEN;
		encoded_len -= BEGIN_CERT_TEXT_LEN;
	}

	/* remove end text if exist */
	end = strstr(buf, END_CERT_TEXT);
	if (end)
		encoded_len -= (src_len - (int)(end - buf));

	cert_size = kg_base64_decode(buf + begin_offset, encoded_len, cert_der);
	if (cert_size <= 0) {
		KG_LOG("%s failed to kg_base64_decode %d\n", __func__, cert_size);
		goto base64_decode_fail;
	}

	cert = kg_x509_cert_parse(cert_der, cert_size);
	if (!cert)
		KG_LOG("%s failed to kg_x509_cert_parse\n", __func__);
	else
		KG_LOG("%s success to x509_cert_parse\n", __func__);

base64_decode_fail:
	kfree(cert_der);
buf_alloc_fail:
	return cert;
}

static int kg_dr_cert(const char *encoded_buf, const struct kernel_param *kp)
{
	int encoded_len, decoded_len, ret;
	char *decoded_buf = NULL;

	if (!dr_initialized) {
		KG_LOG("%s dr is not initialized\n", __func__);
		return 0;
	}

	encoded_len = strlen(encoded_buf);
	if (encoded_len > 4096) {
		KG_LOG("%s encoded_len: %d is too big\n", __func__, encoded_len);
		goto out;
	}

	if (!interm_cert) {
		interm_cert = kg_x509_cert_parse(interm_cert_der, interm_cert_der_len);
		if (!interm_cert) {
			KG_LOG("%s failed to x509_cert_parse interm_cert\n", __func__);
			goto out;
		}
		KG_LOG("%s success to x509_cert_parse interm cert\n", __func__);
	}

	decoded_buf = kmalloc(encoded_len, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(decoded_buf)) {
		KG_LOG("%s failed to alloc decoded_buf %d\n", __func__, encoded_len);
		goto out;
	}

	decoded_len = kg_base64_decode(encoded_buf, encoded_len, decoded_buf);
	if (decoded_len <= 0) {
		KG_LOG("%s failed to kg_base64_decode %d\n", __func__, decoded_len);
		goto out;
	}

	if (leaf_cert)
		kg_x509_cert_free(leaf_cert);

	leaf_cert = kg_parse_leaf_cert(decoded_buf, decoded_len);
	if (!leaf_cert)
		goto out;

	ret = public_key_verify_signature(&interm_cert->pub, &leaf_cert->sig);
	if (ret) {
		KG_LOG("%s failed to public_key_verify_signature of leaf cert %d\n", __func__, ret);
		goto verify_fail;
	} else {
		KG_LOG("%s success to public_key_verify_signature of leaf cert\n", __func__);
	}

	kfree(decoded_buf);

	return 0;

verify_fail:
	kg_x509_cert_free(leaf_cert);
	leaf_cert = NULL;
out:
	kfree(decoded_buf);

	return 0;
}
module_param_call(dr_cert, &kg_dr_cert, NULL, NULL, 0220);

#define TOKEN_SIG_SIZE	256
#define TOKEN_AP_SERIAL_SIZE	LEN_AP_SERIAL
#define TOKEN_TIMESTAMP_SIZE 13
#define TOKEN_TYPE_SIZE 8
#define TOKEN_DATA_SIZE (TOKEN_AP_SERIAL_SIZE + (TOKEN_TIMESTAMP_SIZE * 2) + TOKEN_TYPE_SIZE)
#define TOKEN_SIZE	(TOKEN_SIG_SIZE + TOKEN_DATA_SIZE)

typedef struct dr_token_info {
	char ap_serial[LEN_AP_SERIAL + 1];
	unsigned long server_timestamp;
	unsigned long validity_timestamp;
	unsigned int type;
} dr_token_info_t;

static bool dr_token_valid;
static unsigned int dr_type;
static unsigned long dr_server_timestamp;
static unsigned long dr_validity_timestamp;
static unsigned long dr_boottime_expiration;

static unsigned long get_current_time_ms(void)
{
	return ktime_get_real_ns() / NSEC_PER_MSEC;
}

static unsigned long get_boottime_ms(void)
{
	return ktime_get_boottime_ns() / NSEC_PER_MSEC;
}

static unsigned int kg_get_dr_type(void)
{
	unsigned long boottime_ms, cur_time_ms;

	if (!dr_type || !dr_token_valid || !dr_boottime_expiration
			|| !dr_validity_timestamp || !dr_server_timestamp)
		goto out;

	cur_time_ms = get_current_time_ms();
	boottime_ms = get_boottime_ms();
	if (boottime_ms > dr_boottime_expiration ||
			cur_time_ms > dr_validity_timestamp) {
		dr_token_valid = false;
		KG_LOG("%s s: %lu v: %lu t: %u\n", __func__,
			dr_server_timestamp, dr_validity_timestamp, dr_type);
		KG_LOG("%s d: %lu b: %lu be: %lu\n", __func__,
			cur_time_ms, boottime_ms, dr_boottime_expiration);
		goto out;
	}

	return dr_type & KG_DR_TYPE_MASK;
out:
	return 0;
}

bool kg_dr_hl_disabled(void)
{
	return kg_get_dr_type() & KG_DR_TYPE_HL_DISABLE;
}

bool kg_dr_dl_disabled(void)
{
	return kg_get_dr_type() & KG_DR_TYPE_DL_DISABLE;
}

static int kg_dr_token_show(char *buf, const struct kernel_param *kp)
{
	int i;

	i = sprintf(buf, "%u", kg_get_dr_type());

	return i;
}

static void kg_dr_clear_info(void)
{
	KG_LOG("%s s: %lu v: %lu be: %lu t: %u\n", __func__,
			dr_server_timestamp, dr_validity_timestamp,
			dr_boottime_expiration, dr_type);

	dr_token_valid = false;
	dr_type = 0;
	dr_server_timestamp = 0;
	dr_validity_timestamp = 0;
	dr_boottime_expiration = 0;
}

#define MSEC_PER_MIN (60 * MSEC_PER_SEC)

static void kg_dr_set_info(dr_token_info_t *token_info)
{
	unsigned long cur_time_ms, boottime_ms;
	char *ap_serial;

	if (!token_info) {
		KG_LOG("%s token_info is NULL\n", token_info);
		return;
	}

	ap_serial = kg_get_ap_serial();
	if (!ap_serial) {
		KG_LOG("%s ap_serial is not exist\n", __func__);
		return;
	}

	if (strncmp(token_info->ap_serial, ap_serial, TOKEN_AP_SERIAL_SIZE)) {
		KG_LOG("%s wrong ap_serial: %s\n", __func__, token_info->ap_serial);
		return;
	}

	cur_time_ms = get_current_time_ms();
	if (cur_time_ms > token_info->validity_timestamp ||
			cur_time_ms < (token_info->server_timestamp - MSEC_PER_MIN) ||
			token_info->validity_timestamp < token_info->server_timestamp) {
		KG_LOG("%s wrong time d: %lu , s: %lu, v: %lu\n",
			__func__, cur_time_ms, token_info->server_timestamp,
			token_info->validity_timestamp);
		return;
	}

	if (token_info->type & (~KG_DR_TYPE_MASK)) {
		KG_LOG("%s wrong type: 0x%lx\n", __func__, token_info->type);
		return;
	}

	dr_type = token_info->type;
	dr_server_timestamp = token_info->server_timestamp;
	dr_validity_timestamp = token_info->validity_timestamp;
	boottime_ms = get_boottime_ms();
	dr_boottime_expiration = boottime_ms + (dr_validity_timestamp - dr_server_timestamp);
	dr_token_valid = true;

	KG_LOG("%s s: %lu v: %lu t: %u\n", __func__,
			dr_server_timestamp, dr_validity_timestamp, dr_type);
	KG_LOG("%s d: %lu b: %lu be: %lu\n", __func__,
			cur_time_ms, boottime_ms, dr_boottime_expiration);

	kg_hlmd_wakeup(false);
}

#define STR(x) _STR(x)
#define _STR(x) #x

static dr_token_info_t *kg_dr_parse_token_info(const char *buf)
{
	int ret;
	dr_token_info_t *token = kzalloc(sizeof(dr_token_info_t), GFP_KERNEL);

	if (ZERO_OR_NULL_PTR(token)) {
		KG_LOG("%s failed to kzalloc\n", __func__);
		goto out;
	}

	ret = sscanf(buf, "%" STR(TOKEN_AP_SERIAL_SIZE) "s%" STR(TOKEN_TIMESTAMP_SIZE) "lu%"
			STR(TOKEN_TIMESTAMP_SIZE) "lu%" STR(TOKEN_TYPE_SIZE) "u",
		token->ap_serial, &token->server_timestamp, &token->validity_timestamp, &token->type);

	if (ret != 4) {
		KG_LOG("%s err: %s\n", __func__, buf);
		kfree(token);
		token = NULL;
	}
out:
	return token;
}

static bool kg_verify_dr_token(char *buf, int size)
{
	struct public_key_signature sig;
	bool verified = false;
	int ret;

	if (size != TOKEN_SIZE) {
		KG_LOG("%s size: %d != %d\n", __func__, size, TOKEN_SIZE);
		return verified;
	}

	memset(&sig, 0, sizeof(struct public_key_signature));
	sig.s = buf;
	sig.s_size = TOKEN_SIG_SIZE;
	sig.data = buf + TOKEN_SIG_SIZE;
	sig.data_size = TOKEN_DATA_SIZE;
	sig.pkey_algo = "rsa";
	sig.hash_algo = "sha256";
	sig.encoding = "pkcs1";

	if (kg_calc_digest(&sig))
		goto out;

	ret = public_key_verify_signature(&leaf_cert->pub, &sig);
	if (ret) {
		KG_LOG("%s failed to public_key_verify_signature %d\n", __func__, ret);
		goto out;
	}

	KG_LOG("%s success to public_key_verify_signature\n", __func__);
	verified = true;

out:
	kfree(sig.digest);

	return verified;
}

static int kg_dr_token(const char *buf, const struct kernel_param *kp)
{
	int src_len, token_size;
	char *dr_token_buf = NULL;
	dr_token_info_t *dr_token_info = NULL;

	if (!dr_initialized) {
		KG_LOG("%s dr is not initialized\n", __func__);
		return 0;
	}

	kg_dr_clear_info();

	if (!leaf_cert) {
		KG_LOG("%s leaf_cert is not exist\n", __func__);
		goto out;
	}

	src_len = strlen(buf);
	if (src_len > 1024) {
		KG_LOG("%s src_len: %d is too big\n", __func__, src_len);
		goto out;
	}

	dr_token_buf = kmalloc(src_len, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(dr_token_buf)) {
		KG_LOG("%s failed to kmalloc\n", __func__);
		goto out;
	}

	token_size = kg_base64_decode(buf, src_len, dr_token_buf);
	if (token_size <= 0) {
		KG_LOG("%s failed to kg_base64_decode\n", __func__);
		goto out;

	}

	if (!kg_verify_dr_token(dr_token_buf, token_size)) {
		KG_LOG("%s failed to kg_verify_dr_token\n", __func__);
		goto out;
	}

	dr_token_info = kg_dr_parse_token_info(dr_token_buf + TOKEN_SIG_SIZE);
	if (!dr_token_info)
		goto out;

	kg_dr_set_info(dr_token_info);
out:
	kfree(dr_token_info);
	kfree(dr_token_buf);

	return 0;
}

module_param_call(dr_token, &kg_dr_token, &kg_dr_token_show, NULL, 0660);

static int kg_dr_info_notifier(struct notifier_block *nb, unsigned long unused, void *v)
{
	struct seq_file *m = (struct seq_file *)v;

	if (m) {
		seq_puts(m, "== kg dr ==\n");
		seq_printf(m, "\tcurrent_time: %lu\n", get_current_time_ms());
		seq_printf(m, "\tboottime: %lu\n", get_boottime_ms());
		seq_printf(m, "\tdr_boottime_expiration: %lu\n", dr_boottime_expiration);
		seq_printf(m, "\tdr_token(decision): %u\n", kg_get_dr_type());
		seq_puts(m, "\ttoken info\n");
		seq_printf(m, "\t\tvalid: %u , dr_type: %u\n", dr_token_valid, dr_type);
		seq_printf(m, "\t\tdr_server_timestamp  : %lu\n", dr_server_timestamp);
		seq_printf(m, "\t\tdr_validity_timestamp: %lu\n", dr_validity_timestamp);
		seq_puts(m, "\n");
	} else {
		KG_LOG("== kg dr ==\n");
		KG_LOG("\tcurrent_time: %lu\n", get_current_time_ms());
		KG_LOG("\tboottime: %lu\n", get_boottime_ms());
		KG_LOG("\tdr_boottime_expiration: %lu\n", dr_boottime_expiration);
		KG_LOG("\tdr_token(decision): %u\n", kg_get_dr_type());
		KG_LOG("\ttoken info\n");
		KG_LOG("\t\tvalid: %u , dr_type: %u\n", dr_token_valid, dr_type);
		KG_LOG("\t\tdr_server_timestamp  : %lu\n", dr_server_timestamp);
		KG_LOG("\t\tdr_validity_timestamp: %lu\n", dr_validity_timestamp);
		KG_LOG("\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block kg_dr_info_block = {
	.notifier_call = kg_dr_info_notifier,
	.priority = 1
};

void __init kg_dr_init(void)
{
	int ret;

	ret = kg_drv_info_notifier_register(&kg_dr_info_block);
	if (ret)
		KG_LOG("%s failed to kg_drv_info_notifier_register %d\n", __func__, ret);

	dr_initialized = true;
}

#if IS_ENABLED(CONFIG_KG_DRV_TEST)
static int kg_dr_test_handler(const char *buf, const struct kernel_param *kp)
{
	dr_token_info_t *t;
	int src_len;

	if (!dr_initialized) {
		KG_LOG("%s dr is not initialized\n", __func__);
		goto out;
	}

	src_len = strlen(buf);
	if (src_len != TOKEN_DATA_SIZE) {
		KG_LOG("%s len: %d != %d\n", __func__, src_len, TOKEN_DATA_SIZE);
		goto out;
	}

	kg_dr_clear_info();
	t = kg_dr_parse_token_info(buf);
	if (!t)
		goto out;

	kg_dr_set_info(t);
	kfree(t);
out:
	return 0;
}
module_param_call(dr_test, &kg_dr_test_handler, NULL, NULL, 0200);
#endif
