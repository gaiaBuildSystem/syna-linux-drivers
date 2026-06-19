/*
 * Default Platform Dependent File
 *
 * Copyright (C) 2025 Synaptics Incorporated. All rights reserved.
 *
 * This software is licensed to you under the terms of the
 * GNU General Public License version 2 (the "GPL") with Broadcom special exception.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION
 * DOES NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES,
 * SYNAPTICS' TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT
 * EXCEED ONE HUNDRED U.S. DOLLARS
 *
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/skbuff.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wlan_plat.h>
#else
#include <dhd_plat.h>
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#include <dhd_dbg.h>
#include <dhd.h>

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern void dhd_exit_wlan_mem(void);
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#ifndef WLAN_REG_ON_GPIO_DEFAULT
#define WLAN_REG_ON_GPIO_DEFAULT       (-1)
#endif /* WLAN_REG_ON_GPIO_DEFAULT */
#ifndef WLAN_HOST_WAKE_GPIO_DEFAULT
#define WLAN_HOST_WAKE_GPIO_DEFAULT    (-1)
#endif /* WLAN_HOST_WAKE_GPIO_DEFAULT */

#define DHD_GPIO_INVALID               NULL

static struct gpio_desc *wlan_reg_on = NULL;
#ifdef BCMPCIE
#ifndef DHD_DT_COMPAT_ENTRY
#define DHD_DT_COMPAT_ENTRY        "android,bcmdhd_pcie"
#endif /* DHD_DT_COMPAT_ENTRY */
#define DHD_GPIO_REGON_NAME        "WLPCIE_REG_ON"
#define DHD_GPIO_HOSTW_NAME        "WLPCIE_HOST_WAKE"
#elif defined(BCMSDIO)
#ifndef DHD_DT_COMPAT_ENTRY
#define DHD_DT_COMPAT_ENTRY        "android,bcmdhd_sdio"
#endif /* DHD_DT_COMPAT_ENTRY */
#define DHD_GPIO_REGON_NAME        "WLSDIO_REG_ON"
#define DHD_GPIO_HOSTW_NAME        "WLSDIO_HOST_WAKE"
#else /* non-supported */
#error Unsupport BUS type!
#endif /* BCMPCIE */
#define WIFI_WL_REG_ON_PROPNAME    "wl_reg_on"

#ifdef DHD_USE_HOST_WAKE
static struct gpio_desc *wlan_host_wake_up = NULL;
static int wlan_host_wake_irq = 0;
#define WIFI_WLAN_HOST_WAKE_PROPNAME    "wl_host_wake"
#endif /* DHD_USE_HOST_WAKE */

static int
dhd_wifi_init_gpio(void)
{
	int gpio_reg_on_val;
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (root_node) {
		struct fwnode_handle *fwnode = of_fwnode_handle(root_node);

		wlan_reg_on = fwnode_gpiod_get_index(fwnode,
						     WIFI_WL_REG_ON_PROPNAME,
						     0, GPIOD_OUT_HIGH,
						     DHD_GPIO_REGON_NAME);
		if (IS_ERR(wlan_reg_on))
			wlan_reg_on = NULL;
#ifdef DHD_USE_HOST_WAKE
		wlan_host_wake_up = fwnode_gpiod_get_index(fwnode,
							   WIFI_WLAN_HOST_WAKE_PROPNAME,
							   0, GPIOD_IN,
							   DHD_GPIO_HOSTW_NAME);
		if (IS_ERR(wlan_host_wake_up))
			wlan_host_wake_up = NULL;
#endif /* DHD_USE_HOST_WAKE */
	} else {
		DHD_ERROR(("Warning: no device node of BRCM WLAN, use default GPIOs\n"));
	}

	/* add to dump the DTS to confirm in case some platform special changes */
	if (root_node) {
		struct property * pp = root_node->properties;

		while (NULL != pp) {
			char  str[128] = {0};
			memset(str, 0, sizeof(str));
			memcpy(str, pp->value, pp->length);
			DHD_ERROR(("%s: name='%s', len=%d, val='%s'\n",
			           __FUNCTION__, pp->name, pp->length, str));
			pp = pp->next;
		}
	}

	if (!wlan_reg_on) {
		DHD_ERROR(("%s: gpio_wlan_power('%s') not connected - skip\n",
			__FUNCTION__, WIFI_WL_REG_ON_PROPNAME));
	} else {
		/* ========== WLAN_PWR_EN ============ */
		DHD_ERROR(("%s: gpio_wlan_power('%s') acquired\n",
		           __FUNCTION__, WIFI_WL_REG_ON_PROPNAME));

		gpio_reg_on_val = gpiod_get_value_cansleep(wlan_reg_on);
		DHD_INFO(("%s: Initial WL_REG_ON: [%d]\n",
		          __FUNCTION__, gpio_reg_on_val));

		if (gpio_reg_on_val == 0) {
			DHD_INFO(("%s: WL_REG_ON is LOW, drive it HIGH\n", __FUNCTION__));
			if (gpiod_direction_output(wlan_reg_on, 1)) {
				DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
				return -EIO;
			}
		}
	}

	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);

#ifdef DHD_USE_HOST_WAKE
	if (!wlan_host_wake_up) {
		DHD_ERROR(("%s: gpio_wlan_host_wake('%s') not connected, skip\n",
		           __FUNCTION__, WIFI_WLAN_HOST_WAKE_PROPNAME));
		wlan_host_wake_irq = 0;
	} else {
		/* ========== WLAN_HOST_WAKE ============ */
		DHD_ERROR(("%s: gpio_wlan_host_wake('%s') acquired\n",
		           __FUNCTION__, WIFI_WLAN_HOST_WAKE_PROPNAME));

		wlan_host_wake_irq = gpiod_to_irq(wlan_host_wake_up);
	}
#endif /* DHD_USE_HOST_WAKE */
	return 0;
}

static int
dhd_wifi_deinit_gpio(void)
{
	if (wlan_reg_on) {
		gpiod_put(wlan_reg_on);
		wlan_reg_on = NULL;
	}

#ifdef DHD_USE_HOST_WAKE
	if (wlan_host_wake_up) {
		gpiod_put(wlan_host_wake_up);
		wlan_host_wake_up = NULL;
	}
#endif /* DHD_USE_HOST_WAKE */

	return 0;
}

/***********************************************************************/
/*  Customer specific declaration part (can without Marco protection)  */
/***********************************************************************/

static int
dhd_wlan_power(int onoff)
{
	DHD_INFO(("------------------------------------------------\n"));
	DHD_INFO(("%s Enter: wlan_reg_on=%s, power %s\n",
	          __func__, wlan_reg_on ? "ok" : "none", onoff ? "on" : "off"));

#ifdef SKIP_REGON_GPIO
	DHD_ERROR(("%s-%d: ***** skip action for REG_ON *****\n", __FUNCTION__, __LINE__));
	return 0;
#endif /* SKIP_REGON_GPIO */
	if (!wlan_reg_on) {
		DHD_ERROR(("%s-%d: ***** REG_ON hard wired, skip *****\n", __FUNCTION__, __LINE__));
		return BCME_OK;
	}

	if (onoff) {
		if (gpiod_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpiod_get_value_cansleep(wlan_reg_on)) {
			DHD_INFO(("WL_REG_ON on-step-2 : [%d]\n",
			          gpiod_get_value_cansleep(wlan_reg_on)));
		} else {
			DHD_ERROR(("[%s] gpio value is 0. We need reinit.\n", __func__));
			if (gpiod_direction_output(wlan_reg_on, 1)) {
				DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __func__));
			}
		}

		/* Wait for WIFI_TURNON_DELAY due to power stability */
		msleep(WIFI_TURNON_DELAY);
		DHD_ERROR(("%s-%d: ON DELAY=%d\n", __FUNCTION__, __LINE__, WIFI_TURNON_DELAY));

		/***********************************************************************/
		/* START: customer can add some platform related initialization: START */
		/***********************************************************************/

		/***********************************************************************/
		/*   END: customer can add some platform related initialization: END   */
		/***********************************************************************/
	} else {
		/***********************************************************************/
		/* START: customer can add some platform related initialization: START */
		/***********************************************************************/

		/***********************************************************************/
		/*  END: customer can add some platform related deinitialization: END  */
		/***********************************************************************/

		if (gpiod_direction_output(wlan_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
			return -EIO;
		}
		if (gpiod_get_value_cansleep(wlan_reg_on)) {
			DHD_INFO(("WL_REG_ON on-step-2 : [%d]\n",
			          gpiod_get_value_cansleep(wlan_reg_on)));
		}
	}
	return 0;
}

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

static int
dhd_wlan_set_carddetect(int val)
{
	int ret = 0;

	/***********************************************************************/
	/* START: customer can add some platform related initialization: START */
	/***********************************************************************/

	/***********************************************************************/
	/*   END: customer can add some platform related initialization: END   */
	/***********************************************************************/
	return ret;
}

#ifdef DHD_USE_HOST_WAKE
static int dhd_wlan_get_wake_irq(void)
{
	return wlan_host_wake_up ? gpiod_to_irq(wlan_host_wake_up) : -1;
}

static int dhd_get_wlan_oob_gpio_level(void)
{
	return wlan_host_wake_up ?
		gpiod_get_value_cansleep(wlan_host_wake_up) : -1;
}

int dhd_get_wlan_oob_gpio(void)
{
	return dhd_get_wlan_oob_gpio_level();
}
#endif /* DHD_USE_HOST_WAKE */

struct resource dhd_wlan_resources = {
	.name  = "bcmdhd_wlan_irq",
	.start = 0, /* Dummy */
	.end   = 0, /* Dummy */
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
#ifdef BCMPCIE
	IORESOURCE_IRQ_HIGHEDGE,
#else /* non-BCMPCIE */
	IORESOURCE_IRQ_HIGHLEVEL,
#endif /* BCMPCIE */
};

struct wifi_platform_data dhd_wlan_control = {
	.set_power      = dhd_wlan_power,
	.set_reset      = dhd_wlan_reset,
	.set_carddetect = dhd_wlan_set_carddetect,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc   = dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
#ifdef DHD_USE_HOST_WAKE
	.get_wake_irq   = dhd_wlan_get_wake_irq,
	.get_oob_gpio_level   = dhd_get_wlan_oob_gpio_level,
#endif /* DHD_USE_HOST_WAKE */
};

int
dhd_wlan_init(void)
{
	int ret;

	DHD_INFO(("%s: START.......\n", __FUNCTION__));
	ret = dhd_wifi_init_gpio();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to initiate GPIO, ret=%d\n",
		           __FUNCTION__, ret));
		goto fail;
	}

#ifdef DHD_USE_HOST_WAKE
	dhd_wlan_resources.start = wlan_host_wake_irq;
	dhd_wlan_resources.end = wlan_host_wake_irq;
#endif /* DHD_USE_HOST_WAKE */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	ret = dhd_init_wlan_mem();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to alloc reserved memory,"
		           " ret=%d\n", __FUNCTION__, ret));
	} else {
		DHD_ERROR(("%s: Allocate reserved memory sucessfully,"
		           " ret=%d\n", __FUNCTION__, ret));
	}

#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

fail:
	DHD_INFO(("%s: FINISH.......\n", __FUNCTION__));
	/* add to free gpio resource */
	if (0 > ret) {
		dhd_wifi_deinit_gpio();
	}
	return ret;
}

void
dhd_wlan_deinit(void)
{
	dhd_wifi_deinit_gpio();

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	dhd_exit_wlan_mem();
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
}

#ifndef BCMDHD_MODULAR
#if defined(CONFIG_DEFERRED_INITCALLS)
deferred_module_init(dhd_wlan_init);
#elif defined(late_initcall)
late_initcall(dhd_wlan_init);
#else /* default */
module_init(dhd_wlan_init);
#endif /* CONFIG_DEFERRED_INITCALLS */
module_exit(dhd_wlan_deinit);
#endif /* !BCMDHD_MODULAR */
