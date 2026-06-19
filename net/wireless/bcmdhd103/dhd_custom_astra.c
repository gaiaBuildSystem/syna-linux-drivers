/*
 * Platform Dependent file for Khadas VIM3
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

#include <linux/skbuff.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wlan_plat.h>
#else
#include <dhd_plat.h>
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#include <dhd_dbg.h>
#include <dhd.h>
#include <bcmdevs.h>
#include <linux/pci.h>
#include <epivers.h>
#include <bcmdevs_legacy.h>

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern int dhd_init_wlan_mem(void);
extern void dhd_exit_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define WLAN_REG_ON_GPIO		(-1)
#define WLAN_HOST_WAKE_GPIO		(-1)

static struct gpio_desc *wlan_reg_on = NULL;
#define DHD_DT_COMPAT_ENTRY		"synaptics,bcmdhd_wlan"
#define WIFI_WL_REG_ON_PROPNAME		"wl_reg_on"

static struct gpio_desc *wlan_host_wake_up = NULL;
static int wlan_host_wake_irq = 0;
#define WIFI_WLAN_HOST_WAKE_PROPNAME    "wl_host_wake"

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

static int
dhd_wifi_init_reg_gpio(void)
{
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (root_node) {
		DHD_ERROR(("%s: Found device node of BRCM WLAN in DT.\n", __FUNCTION__));
		struct fwnode_handle *fwnode = of_fwnode_handle(root_node);
		wlan_reg_on = fwnode_gpiod_get_index(fwnode, WIFI_WL_REG_ON_PROPNAME,
						    0, GPIOD_OUT_HIGH, "WL_REG_ON");
		if (IS_ERR(wlan_reg_on))
			wlan_reg_on = NULL;
	} else {
		DHD_ERROR(("%s: Failed to get dts of wlan\n", __FUNCTION__));
	}

	// add to dump the DTS to confirm in case some platform special changes
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

	/* ========== WLAN_PWR_EN ============ */
	if (!wlan_reg_on) {
		DHD_ERROR(("%s: wlan_power('%s') not connected\n", __FUNCTION__,
			WIFI_WL_REG_ON_PROPNAME));
		return 0;
	}
	DHD_ERROR(("%s: wlan_power('%s') acquired\n", __FUNCTION__,
		WIFI_WL_REG_ON_PROPNAME));
	if (gpiod_direction_output(wlan_reg_on, 1)) {
		DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __FUNCTION__));
		return -EIO;
	}
	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);

	return 0;
}
int
dhd_wifi_init_gpio(void)
{
	int reg_on_val;
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;

#ifdef HOST_WAKE_IRQ_CPUCORE
	struct cpumask host_wake_cpu_mask;
	cpumask_clear(&host_wake_cpu_mask);
	/* Set on cpu HOST_WAKE_IRQ_CPUCORE */
	cpumask_set_cpu(HOST_WAKE_IRQ_CPUCORE, &host_wake_cpu_mask);
#endif /* HOST_WAKE_IRQ_CPUCORE */

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (root_node) {
		struct fwnode_handle *fwnode = of_fwnode_handle(root_node);
		wlan_reg_on = fwnode_gpiod_get_index(fwnode, WIFI_WL_REG_ON_PROPNAME,
						    0, GPIOD_OUT_HIGH, "WL_REG_ON");
		if (IS_ERR(wlan_reg_on))
			wlan_reg_on = NULL;
		wlan_host_wake_up = fwnode_gpiod_get_index(fwnode, WIFI_WLAN_HOST_WAKE_PROPNAME,
							0, GPIOD_IN, "WLAN_HOST_WAKE");
		if (IS_ERR(wlan_host_wake_up))
			wlan_host_wake_up = NULL;
	} else {
		DHD_ERROR(("failed to get device node of BRCM WLAN\n"));
	}

	// add to dump the DTS to confirm in case some platform special changes
	if (root_node) {
		struct property * pp = root_node->properties;

		while (NULL != pp) {
			char  str[128] = {0};
			memset(str, 0, sizeof(str));
			memcpy(str, pp->value, pp->length);
			DHD_ERROR(("%s: name='%s', len=%d, val='%s'\n",
				__func__, pp->name, pp->length, str));
			pp = pp->next;
		}
	}

	/* ========== WLAN_PWR_EN ============ */
	if (!wlan_reg_on) {
		DHD_ERROR(("%s: wlan_power('%s') not connected\n", __func__,
			WIFI_WL_REG_ON_PROPNAME));
	} else {
		reg_on_val = gpiod_get_value(wlan_reg_on);
		DHD_ERROR(("%s: Initial WL_REG_ON: [%d]\n", __func__, reg_on_val));
		if (reg_on_val == 0) {
			DHD_INFO(("%s: WL_REG_ON is LOW, drive it HIGH\n", __func__));
			if (gpiod_direction_output(wlan_reg_on, 1)) {
				DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __func__));
				return -EIO;
			}
		}
	}

	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);

	/* ========== WLAN_HOST_WAKE ============ */
	if (!wlan_host_wake_up) {
		DHD_ERROR(("%s: wlan_host_wake not connected\n", __func__));
	} else {
		wlan_host_wake_irq = gpiod_to_irq(wlan_host_wake_up);
	}
	DHD_ERROR(("%s: wlan_host_wake_irq %d\n", __func__, wlan_host_wake_irq));

#ifdef HOST_WAKE_IRQ_CPUCORE
	/* Core 0 and 1, A53, are not as fast as 2-5 (A73), to process the SDIO interrupt */
	if (
#ifdef BCMDHD_MODULAR
		irq_set_affinity_hint(wlan_host_wake_irq, &host_wake_cpu_mask) == 0)
#else
		irq_set_affinity(wlan_host_wake_irq, &host_wake_cpu_mask) == 0)
#endif /* BCMDHD_MODULAR */
	{
		DHD_ERROR(("%s: IRQ %d assigned to CPU %d\n", __func__,
			wlan_host_wake_irq, HOST_WAKE_IRQ_CPUCORE));
	} else {
		DHD_ERROR(("%s: Failed to set IRQ affinity for host wake\n", __func__));
	}
#endif /* HOST_WAKE_IRQ_CPUCORE */

	return 0;
}

// add for free GPIO resource
int
dhd_wifi_deinit_gpio(void)
{
	if (wlan_reg_on) {
		gpiod_put(wlan_reg_on);
		wlan_reg_on = NULL;
	}
	if (wlan_host_wake_up) {
		gpiod_put(wlan_host_wake_up);
		wlan_host_wake_up = NULL;
	}

	return 0;
}

int
dhd_wlan_power(int onoff)
{
	DHD_INFO(("------------------------------------------------"));
	DHD_INFO(("------------------------------------------------\n"));
	DHD_ERROR(("%s Enter: power %s\n", __func__, onoff ? "on" : "off"));

	if (!wlan_reg_on) {
		DHD_ERROR(("%s: wlan_reg_on not connected\n", __func__));
		return 0;
	}
	if (onoff) {
		if (gpiod_direction_output(wlan_reg_on, 1)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull up\n", __func__));
			return -EIO;
		}
		/* Wait for WIFI_TURNON_DELAY due to power stability */
		msleep(WIFI_TURNON_DELAY);
	} else {
		if (gpiod_direction_output(wlan_reg_on, 0)) {
			DHD_ERROR(("%s: WL_REG_ON is failed to pull down\n", __func__));
			return -EIO;
		}
	}
	return 0;
}

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

// add for card detect
#if defined(BCMSDIO) && defined(BCMDHD_MODULAR) && defined(ENABLE_INSMOD_NO_FW_LOAD) && \
	!defined(GKI_NO_SDIO_PATCH)

#ifndef EMPTY_CARD_DETECT
extern int wifi_card_detect(void);
#endif /* EMPTY_CARD_DETECT */
#endif // BCMSDIO && BCMDHD_MODULAR && ENABLE_INSMOD_NO_FW_LOAD && !GKI_NO_SDIO_PATCH

static int
dhd_wlan_set_carddetect(int val)
{
// add for card detect
#if defined(BCMSDIO) && defined(BCMDHD_MODULAR) && defined(ENABLE_INSMOD_NO_FW_LOAD) && \
	!defined(GKI_NO_SDIO_PATCH)
#ifndef EMPTY_CARD_DETECT
	int ret = 0;

	ret = wifi_card_detect();
	if (0 > ret) {
		DHD_ERROR(("%s-%d: * error hapen, ret=%d (ignore when remove)\n",
			__func__, __LINE__, ret));
	}
#endif /* EMPTY_CARD_DETECT */
#endif // BCMSDIO && BCMDHD_MODULAR && ENABLE_INSMOD_NO_FW_LOAD && !GKI_NO_SDIO_PATCH
	return 0;
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

#if defined(BCMPCIE) && defined(DHD_ASTRA_CUST_CHIP_SUPPORT)
static int dhd_check_pcie_devices(void)
{
	struct pci_dev *dev;

	dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
	while (dev) {
		printk(KERN_INFO "PCI device: %04x:%04x (class %06x) at %02x:%02x.%d\n",
			   dev->vendor, dev->device,
			   dev->class,
			   dev->bus->number,
			   PCI_SLOT(dev->devfn),
			   PCI_FUNC(dev->devfn));

		if (VENDOR_BROADCOM == dev->vendor) {
			if ((BCM4362_CHIP_ID == dev->device) ||
				(BCM43752_D11AX_ID == dev->device) ||
				(BCM4345_CHIP_ID == dev->device)) {
				return -ENODEV;
			}
		}

		if (VENDOR_SYNAPTICS == dev->vendor) {
			if (BCM43711_D11AX6E_ID == dev->device) {
				return -ENODEV;
			}
		}

		if (VENDOR_BROADCOM == dev->vendor) {
			if ((BCM4381_CHIP_ID == dev->device) ||
				(BCM4382_CHIP_ID == dev->device)) {
					return -ENODEV;
			}

			if ((BCM4384_D11BE_ID == dev->device) ||
			    (BCM4390_D11BE_ID == dev->device)) {
				return 0;
			}
		}

		dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, dev);
	}

	return -ENODEV;
}
#endif /* BCMPCIE */

int
dhd_wlan_init(void)
{
	int ret = 0;
	int ret1 = 0;

#if defined(CONFIG_ARCH_ASTRA) && defined(BCMPCIE) && defined(DHD_ASTRA_CUST_CHIP_SUPPORT)
	msleep(2000);
#endif

#if defined(CONFIG_ARCH_ASTRA) && defined(BCMSDIO) && defined(DHD_ASTRA_CUST_CHIP_SUPPORT)
	msleep(500);
#endif

	DHD_ERROR(("%s: START.......%s\n", __func__, EPI_VERSION_STR));
#ifdef DHD_USE_HOST_WAKE
	ret = dhd_wifi_init_gpio();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to initiate GPIO, ret=%d\n",
			__func__, ret));
		goto fail;
	}

	dhd_wlan_resources.start = wlan_host_wake_irq;
	dhd_wlan_resources.end = wlan_host_wake_irq;

	DHD_ERROR(("%s: WL_HOST_WAKE=%s, oob_irq=%d, oob_irq_flags=%ld\n", __func__,
		wlan_host_wake_up ? "ok" : "none", wlan_host_wake_irq, dhd_wlan_resources.flags));
#else
	ret = dhd_wifi_init_reg_gpio();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to initiate REG ON/OFF GPIO, ret=%d\n",
			__func__, ret));
		goto fail;
	}
#endif /* DHD_USE_HOST_WAKE */

#if defined(BCMPCIE) && defined(DHD_ASTRA_CUST_CHIP_SUPPORT)
	ret1 = dhd_check_pcie_devices();
	if (ret1 < 0) {
		DHD_ERROR(("%s: No supported device, ret=%d\n",
			__func__, ret1));
		goto fail;
	}
#else
	BCM_REFERENCE(ret1);
#endif /* BCMPCIE */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	/* Allocated by kernel */
	ret = dhd_init_wlan_mem();
	if (ret < 0) {
		DHD_ERROR(("%s: failed to alloc reserved memory,"
				" ret=%d\n", __func__, ret));
	}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

fail:
	DHD_INFO(("%s: FINISH.......\n", __func__));
	// add to free gpio resource
	if (0 > ret) {
		dhd_wifi_deinit_gpio();
	}
	return ret;
}

void
dhd_wlan_deinit(void)
{
	if (wlan_host_wake_up) {
		gpiod_put(wlan_host_wake_up);
		wlan_host_wake_up = NULL;
	}

	if (wlan_reg_on) {
		gpiod_put(wlan_reg_on);
		wlan_reg_on = NULL;
	}

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	dhd_exit_wlan_mem();
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
}
#ifdef DHD_COREDUMP
void
dhd_plat_register_coredump(void)
{
	return;
}

void
dhd_plat_unregister_coredump(void)
{
	return;
}
#endif /* DHD_COREDUMP */

#ifndef BCMDHD_MODULAR
/* Required only for Built-in DHD */
device_initcall(dhd_wlan_init);
#endif /* BOARD_HIKEY_MODULAR */
