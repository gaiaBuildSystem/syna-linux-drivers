// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2026 Synaptics Incorporate
 *
 */
#include "tsp.h"
#include "tee_client_api.h"
#include <linux/firmware.h>

#define TA_IMG_PATH_TSP      "ta/libtsp.ta"

static const TEEC_UUID ta_tsp_uuid = {0x1316a183, 0x894d, 0x43fe, \
	{0x98, 0x93, 0xbb, 0x94, 0x6a, 0xe1, 0x03, 0xe8} };
static TEEC_Context context;
static TEEC_Session session[TSP_FIGO_NUM];
static bool g_tsp_ta_loaded = false;

bool tz_get_tsp_ta_status(void)
{
	return g_tsp_ta_loaded;
}
EXPORT_SYMBOL(tz_get_tsp_ta_status);

int tz_tsp_load_ta(struct device *dev)
{
	const struct firmware *fw = NULL;
	TEEC_SharedMemory fw_shm = {};
	TEEC_Parameter tee_param = {};
	int ret;

	ret = request_firmware(&fw, TA_IMG_PATH_TSP, dev);
	if (ret) {
		pr_err("faild req fw 0x%x %s\n", ret, TA_IMG_PATH_TSP);
		return ret;
	}

	ret = TEEC_InitializeContext(
				NULL,
				&context);
	if (ret != TEEC_SUCCESS) {
		pr_err("TEEC_InitializeContext ret=0x%08x\n", ret);
		return ret;
	} else {
		pr_info("TEEC_InitializeContext success\n");
	}

	fw_shm.size = ALIGN(fw->size, PAGE_SIZE);
	fw_shm.flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;
	ret = TEEC_AllocateSharedMemory(&context, &fw_shm);
	if (ret || !fw_shm.buffer) {
		pr_err("can't allocate memory(%zu) for firmware loading: 0x%x\n",
			fw_shm.size, ret);
		ret = -ENOMEM;
		goto free_fw;
	}
	memcpy(fw_shm.buffer, fw->data, fw->size);

	tee_param.memref.parent = &fw_shm;
	tee_param.memref.size = fw_shm.size;
	ret = TEEC_RegisterTA(&context, &tee_param, TEEC_MEMREF_PARTIAL_INPUT);
	if (TEEC_ERROR_ACCESS_CONFLICT == ret) {
		pr_warn("%s TA has been loaded\n", TA_IMG_PATH_TSP);
		ret = 0;
	} else if (ret) {
		pr_err("can't register %s TA: 0x%08x\n", TA_IMG_PATH_TSP, ret);
	} else {
		pr_info("TA loaded sucessfully - %s\n", TA_IMG_PATH_TSP);
		g_tsp_ta_loaded = true;
	}

	TEEC_ReleaseSharedMemory(&fw_shm);
free_fw:
	release_firmware(fw);
	return ret;
}

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

	/* [1] Open session with TEE application */
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

