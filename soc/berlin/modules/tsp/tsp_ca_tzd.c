// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2026 Synaptics Incorporate
 *
 */
#include "tsp.h"
#include "tee_client_api.h"

static const TEEC_UUID ta_tsp_uuid = {0x1316a183, 0x894d, 0x43fe, \
	{0x98, 0x93, 0xbb, 0x94, 0x6a, 0xe1, 0x03, 0xe8} };
static TEEC_Context context;
static TEEC_Session session[TSP_FIGO_NUM];

static int tz_tsp_errcode_translate(TEEC_Result result)
{
	int ret;

	switch (result) {
	case TEEC_SUCCESS:
		ret = 0;
		break;
	case TEEC_ERROR_ACCESS_DENIED:
		ret = -ENOTSUPP;
		break;
	case TEEC_ERROR_BAD_PARAMETERS:
		ret = -EINVAL;
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}

int tz_tsp_initialize(void)
{
	TEEC_Result result = TEEC_SUCCESS;
	uint32_t i;

	/* [1] Connect to TEE */
	result = TEEC_InitializeContext(
				NULL,
				&context);
	if (result != TEEC_SUCCESS) {
		pr_err("TEEC_InitializeContext ret=0x%08x\n", result);
		goto fun_ret;
	} else
		pr_info("TEEC_InitializeContext success\n");

	/* [2] Open session with TEE application */
	for (i = 0; i < TSP_FIGO_NUM; i++) {
		TEEC_Operation operation;

		operation.paramTypes = TEEC_PARAM_TYPES(
				TEEC_VALUE_INPUT,
				TEEC_NONE,
				TEEC_NONE,
				TEEC_NONE);
		operation.params[0].value.a = i;

		result = TEEC_OpenSession(
					&context,
					&session[i],
					&ta_tsp_uuid,
					TEEC_LOGIN_USER,
					NULL,
					&operation,
					NULL);
		if (result != TEEC_SUCCESS) {
			while (i--)
				TEEC_CloseSession(&session[i]);
			TEEC_FinalizeContext(&context);
			pr_err("TEEC_OpenSession ret=0x%08x\n", result);
			goto fun_ret;
		}
		pr_info("TEEC_OpenSession %d success\n", i);
	}

fun_ret:
	return tz_tsp_errcode_translate(result);
}

void tz_tsp_finalize(void)
{
	uint32_t i;

	for (i = 0; i < TSP_FIGO_NUM; i++)
		TEEC_CloseSession(&session[i]);
	TEEC_FinalizeContext(&context);
}

int tz_tsp_check_figo_id(uint32_t id)
{
	if (id > (TSP_FIGO_NUM - 1)) {
		pr_err("Invalid figo id:0x%x\n", id);
		return TEEC_ERROR_BAD_PARAMETERS;
	}
	return TEEC_SUCCESS;
}

int tz_tsp_save_hw_context(uint32_t figo_id)
{
	TEEC_Result result = TEEC_SUCCESS;
	TEEC_Operation operation;

	result = tz_tsp_check_figo_id(figo_id);
	if (result != TEEC_SUCCESS)
		goto fun_ret;

	operation.paramTypes = TEEC_PARAM_TYPES(
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE);

	result = TEEC_InvokeCommand(
			&session[figo_id],
			TSP_SAVE_HW_CONTEXT,
			&operation,
			NULL);
	if (result != TEEC_SUCCESS)
		pr_err("figo[%d] save HW context error: 0x%x\n",
			figo_id, result);

fun_ret:
	return tz_tsp_errcode_translate(result);
}

int tz_tsp_restore_hw_context(uint32_t figo_id)
{
	TEEC_Result result = TEEC_SUCCESS;
	TEEC_Operation operation;

	result = tz_tsp_check_figo_id(figo_id);
	if (result != TEEC_SUCCESS)
		goto fun_ret;

	operation.paramTypes = TEEC_PARAM_TYPES(
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE);

	result = TEEC_InvokeCommand(
			&session[figo_id],
			TSP_RESTORE_HW_CONTEXT,
			&operation,
			NULL);
	if (result != TEEC_SUCCESS)
		pr_err("figo[%d] restore HW context error: 0x%x\n",
			figo_id, result);

fun_ret:
	return tz_tsp_errcode_translate(result);
}

int tz_tsp_set_figo_state(uint32_t figo_id, uint32_t state)
{
	TEEC_Result result = TEEC_SUCCESS;
	TEEC_Operation operation;

	result = tz_tsp_check_figo_id(figo_id);
	if (result != TEEC_SUCCESS)
		goto fun_ret;

	operation.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_INPUT,
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE);
	operation.params[0].value.a = state;

	result = TEEC_InvokeCommand(
			&session[figo_id],
			TSP_SET_FIGO_STATE,
			&operation,
			NULL);
	if (result != TEEC_SUCCESS)
		pr_err("figo[%d] set %s failed, error code: 0x%x\n",
			figo_id, (state == Figo_STA_RESET) ? "reset" : "release", result);

fun_ret:
	return tz_tsp_errcode_translate(result);
}

int tz_tsp_get_figo_state(uint32_t figo_id, uint32_t *state)
{
	TEEC_Result result = TEEC_SUCCESS;
	TEEC_Operation operation;

	result = tz_tsp_check_figo_id(figo_id);
	if (result != TEEC_SUCCESS)
		goto fun_ret;

	operation.paramTypes = TEEC_PARAM_TYPES(
			TEEC_VALUE_OUTPUT,
			TEEC_NONE,
			TEEC_NONE,
			TEEC_NONE);

	result = TEEC_InvokeCommand(
			&session[figo_id],
			TSP_GET_FIGO_STATE,
			&operation,
			NULL);
	if (result != TEEC_SUCCESS)
		pr_err("figo[%d] get reset register error: 0x%x\n",
			figo_id, result);
	*state = operation.params[0].value.a;

fun_ret:
	return tz_tsp_errcode_translate(result);
}

