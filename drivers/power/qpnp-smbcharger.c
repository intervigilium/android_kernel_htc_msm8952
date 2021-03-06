/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef CONFIG_HTC_BATT_8960
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__
#else
#define pr_fmt(fmt)	"[BATT][SMB] %s: " fmt, __func__
#endif

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>

#ifdef CONFIG_HTC_BATT_8960
#include <linux/wakelock.h>
#include <linux/of_fdt.h>
#include <linux/htc_flags.h>
#include <linux/power/htc_gauge.h>
#include <linux/qpnp/qpnp-smbcharger.h>
#include <linux/qpnp/qpnp-fg.h>
#include <linux/power/htc_battery_common.h>
#endif

#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

#ifdef CONFIG_HTC_BATT_8960
#define MIN(a,b)                   (((a)<(b))?(a):(b))
static inline int ABS(int x) { return x >= 0 ? x : -x; }
#endif

struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	int				schg_version;

	
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	
	int				iterm_ma;
	int				usb_max_current_ma;
	int				dc_max_current_ma;
	int				usb_target_current_ma;
	int				usb_tl_current_ma;
	int				dc_target_current_ma;
	int				target_fastchg_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				cfg_chg_led_support;
	bool				cfg_chg_led_sw_ctrl;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				hvdcp3_supported;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;

	
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	
	int				max_vbat_sample;
	int				n_vbat_samples;

	
	int				battchg_disabled;
	int				usb_suspended;
	int				dc_suspended;
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				sw_esr_pulse_en;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	u32				wa_flags;

	
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				safety_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	
	int				chg_error_irq;
	bool				enable_aicl_wake;

	
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
#ifndef CONFIG_HTC_BATT_8960
	struct work_struct		usb_set_online_work;
#else
	struct delayed_work		usb_set_online_work;
#endif
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			battchg_disabled_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			fcc_lock;
	struct mutex			pm_lock;
	
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;

#ifdef CONFIG_HTC_BATT_8960
	int				usbin_chgr_cfg;
	bool				chg_done_batt_full;
	bool				is_ac_safety_timeout;
	bool				wa_adjust_iusb_max_to_prevent_over_chg_from_1A_adaptor;
	struct work_struct		notify_fg_work;
	struct work_struct		batt_soc_work;
	struct work_struct		usb_aicl_limit_current;
	int 				warm_bat_decidegc;
	int                             cool_bat_decidegc;
	int                             hot_bat_decidegc;
	int                             cold_bat_decidegc;
	int				eoc_ibat_thre_ma;
	int				cv_fastchg_cur_ma;
	int				adjust_fastchg_cur_thr_mv;
	int				qc20_usb_max_current_ma;
	int 				kddi_limit1_temp;
	int 				kddi_limit1_current;
	int 				kddi_limit1_current_qc20;
	int 				kddi_limit2_temp;
	int 				kddi_limit2_current;
	int 				kddi_limit2_current_qc20;
	int				otg_output_current_ma;
	spinlock_t			vbus_lock;
	struct wake_lock 		vbus_wlock;
	struct wake_lock		eoc_worker_wlock;
	struct delayed_work 		vbus_detect_work;
	struct delayed_work		eoc_work;
	struct delayed_work		usb_type_polling_work;
	struct delayed_work		retry_aicl_work;
	struct delayed_work		hvdcp_5to9V_work;
	struct delayed_work		usb_limit_max_current;
	struct workqueue_struct 	*cable_detect_wq;
	enum htc_ftm_power_source_type	ftm_src;
	enum power_supply_type 		pre_usb_supply_type;
	enum htc_power_source_type  power_source_type;
#endif
};

#ifdef CONFIG_HTC_BATT_8960
static struct smbchg_chip *the_chip;

bool flag_keep_charge_on;
bool flag_force_ac_chg;
bool flag_pa_fake_batt_temp;
bool flag_disable_safety_timer;
bool flag_disable_temp_protection;
static bool flag_enable_bms_charger_log;

static unsigned int chg_limit_current;
static int smbchg_debug_mask;
static int eoc_count;
static int eoc_count_by_curr;
static bool is_batt_full = false;
static bool is_batt_full_eoc_stop = false;
static int is_device_XA_or_XB;
static int is_safety_timer_disable;
static unsigned long dischg_plus_sr_time;
static unsigned long dischg_last_jiffies;
static unsigned long dischg_suspend_ms;
static bool is_limit_IUSB = false;
static bool gs_is_limit_1A = false;
static bool is_over_fastchg_thr_once = false;
static bool is_qc20_done_flag = false;
static int svbat_mv;
static int retry_aicl_cnt;
static int pre_usb_max_current_ma;
static int pre_current_ma;
static int g_wa_adjust_iusb_max_pre_usb_target_current_ma;
static int g_wa_sdp_chg_suspend_count;

static int is_ac_online(void);

#define PMI8994_CHG_I_MIN_MA          300
#define PMI8994_CHG_I_MIN_MA_L1       2000
#define PMI8994_CHG_I_MIN_MA_L2       300
#define EOC_CHECK_PERIOD_MS           20000
#define CONSECUTIVE_COUNT             3
#define CLEAR_FULL_STATE_BY_LEVEL_THR 97
#define DT_ROOT_VALUE 0

#define CHG_INHIBIT_BIT       BIT(1)
#define P2F_CHG_THRESHOLD_BIT BIT(4)
#define BAT_TAPER_MODE_BIT    BIT(6)
#define BAT_TCC_REACHED_BIT   BIT(7)

#define USB_MA_0       (0)
#define USB_MA_2       (2)
#define USB_MA_100     (100)
#define USB_MA_150     (150)
#define USB_MA_500     (500)
#define USB_MA_900     (900)
#define USB_MA_950     (950)
#define USB_MA_1000     (1000)
#define USB_MA_1100    (1100)
#define USB_MA_1200    (1200)
#define USB_MA_1400 (1400)
#define USB_MA_1450 (1450)
#define USB_MA_1500	(1500)
#define USB_MA_1600	(1600)

#define CFG_SYSMIN 0xF4
#define HVDCP_CHANGE_PERIOD_MS 10000
#endif 

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
	SMBCHG_USB100_WA = BIT(2),
	SMBCHG_BATT_OV_WA = BIT(3),
};

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
};

#ifdef CONFIG_HTC_BATT_8960
enum qpnp_tm_state_smbchg {
	ADC_TM_WARM_STATE_SMBCHG = 0,
	ADC_TM_COOL_STATE_SMBCHG,
	ADC_TM_HOT_STATE_SMBCHG,
	ADC_TM_COLD_STATE_SMBCHG,
	ADC_TM_STATE_NUM_SMBCHG,
};

#define HYSTERISIS_DECIDEGC 20
#endif

static int smbchg_debug_mask;
module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int smbchg_parallel_en = 1;
module_param_named(
	parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
	dynamic_icl_wipower_en, wipower_dyn_icl_en,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
	wipower_dcin_interval, wipower_dcin_interval,
	int, S_IRUSR | S_IWUSR
);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
	wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
	int, S_IRUSR | S_IWUSR
);

#ifdef CONFIG_HTC_BATT_8960
static int
pmi8994_get_usbin_voltage_now(struct smbchg_chip *chip);
static irqreturn_t
cable_detection_vbus_irq_handler(void);

static void smbchg_rerun_aicl(struct smbchg_chip *chip);
static int smbchg_set_otg_output_current(struct smbchg_chip *chip, int otg_ma);
int filter_usb_max_chg_currnt(int current_ma);
static void dump_regs(struct smbchg_chip *chip);
#endif

#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

static int smbchg_write(struct smbchg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#ifndef CONFIG_HTC_BATT_8960
#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}
#endif

#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;

	msleep(20);

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_GND_DET_STS			BIT(2)
static bool is_otg_present_schg_lite(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_GND_DET_STS);
}

static bool is_otg_present(struct smbchg_chip *chip)
{
	if (chip->schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static char *usb_type_str[] = {
	"SDP",		
	"OTHER",	
	"DCP",		
	"CDP",		
	"NONE",		
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		
	POWER_SUPPLY_TYPE_USB_DCP,	
	POWER_SUPPLY_TYPE_USB_DCP,	
	POWER_SUPPLY_TYPE_USB_CDP,	
#ifndef CONFIG_HTC_BATT_8960
	POWER_SUPPLY_TYPE_USB_DCP,	
#else
	POWER_SUPPLY_TYPE_UNKNOWN,	
#endif
};

static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static void read_usb_type(struct smbchg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
#ifdef CONFIG_HTC_BATT_8960
		return;
#endif
	}
	type = get_type(reg);
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

#define CHGR_STS			0x0E
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)
#define CHG_INHIBIT_BIT		BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;

	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & BAT_TCC_REACHED_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	int capacity, rc;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smbchg_chip *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}
	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smbchg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4200000
static int get_prop_batt_voltage_max_design(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}
#endif 

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static const int usb_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int dc_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static const int fcc_comp_table[] = {
	250,
	700,
	900,
	1200,
};

static int calc_thermal_limited_current(struct smbchg_chip *chip,
						int current_ma)
{
	int therm_ma;

	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1)) {
		therm_ma = (int)chip->thermal_mitigation[chip->therm_lvl_sel];
		if (therm_ma < current_ma) {
			pr_smb(PR_STATUS,
				"Limiting current due to thermal: %d mA",
				therm_ma);
			return therm_ma;
		}
	}

	return current_ma;
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
	
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);
}

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	for (i = ARRAY_SIZE(dc_current_table) - 1; i >= 0; i--) {
		if (current_ma >= dc_current_table[i])
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = dc_current_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

enum enable_reason {
	
	REASON_USER = BIT(0),
	REASON_POWER_SUPPLY = BIT(1),
	REASON_USB = BIT(2),
	REASON_WIRELESS = BIT(3),
	REASON_THERMAL = BIT(4),
	REASON_OTG = BIT(5),
	REASON_WEAK_CHARGER = BIT(6),
};

enum battchg_enable_reason {
	
	REASON_BATTCHG_USER		= BIT(0),
	
	REASON_BATTCHG_UNKNOWN_BATTERY	= BIT(1),
};

static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
#ifndef CONFIG_HTC_BATT_8960
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				usb_set_online_work);
#else
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				usb_set_online_work.work);
#endif
	bool user_enabled = (chip->usb_suspended & REASON_USER) == 0;
	int online;

	online = user_enabled && chip->usb_present && !chip->very_weak_charger;

	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		pr_smb(PR_MISC, "setting usb psy online = %d\n", online);
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

static bool smbchg_primary_usb_is_en(struct smbchg_chip *chip,
		enum enable_reason reason)
{
	bool enabled;

	mutex_lock(&chip->usb_en_lock);
	enabled = (chip->usb_suspended & reason) == 0;
	mutex_unlock(&chip->usb_en_lock);

	return enabled;
}

#ifndef CONFIG_HTC_BATT_8960
static bool smcghg_is_battchg_en(struct smbchg_chip *chip,
		enum battchg_enable_reason reason)
{
	bool enabled;

	mutex_lock(&chip->battchg_disabled_lock);
	enabled = !(chip->battchg_disabled & reason);
	mutex_unlock(&chip->battchg_disabled_lock);

	return enabled;
}
#endif

static int smbchg_battchg_en(struct smbchg_chip *chip, bool enable,
		enum battchg_enable_reason reason, bool *changed)
{
	int rc = 0, battchg_disabled;

	pr_smb(PR_STATUS, "battchg %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->battchg_disabled == 0 ? "enabled" : "disabled",
			chip->battchg_disabled, enable, reason);

	mutex_lock(&chip->battchg_disabled_lock);
	if (!enable)
		battchg_disabled = chip->battchg_disabled | reason;
	else
		battchg_disabled = chip->battchg_disabled & (~reason);

	
	if (!!battchg_disabled == !!chip->battchg_disabled) {
		*changed = false;
		goto out;
	}

	rc = smbchg_charging_en(chip, !battchg_disabled);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't configure batt chg: 0x%x rc = %d\n",
			battchg_disabled, rc);
		goto out;
	}
	*changed = true;

	pr_smb(PR_STATUS, "batt charging %s, battchg_disabled = %02x\n",
			battchg_disabled == 0 ? "enabled" : "disabled",
			battchg_disabled);
out:
	chip->battchg_disabled = battchg_disabled;
	mutex_unlock(&chip->battchg_disabled_lock);
	return rc;
}

static int smbchg_primary_usb_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason, bool *changed)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "usb %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->usb_suspended == 0 ? "enabled"
			: "suspended", chip->usb_suspended, enable, reason);
	mutex_lock(&chip->usb_en_lock);
	if (!enable)
		suspended = chip->usb_suspended | reason;
	else
		suspended = chip->usb_suspended & (~reason);

	
	if (!!suspended == !!chip->usb_suspended) {
		*changed = false;
		goto out;
	}

	*changed = true;
	rc = smbchg_usb_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set usb suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	pr_smb(PR_STATUS, "usb charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->usb_suspended = suspended;
	mutex_unlock(&chip->usb_en_lock);
	return rc;
}

static int smbchg_dc_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	int rc = 0, suspended;

	pr_smb(PR_STATUS, "dc %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->dc_suspended == 0 ? "enabled"
			: "suspended", chip->dc_suspended, enable, reason);
	mutex_lock(&chip->dc_en_lock);
	if (!enable)
		suspended = chip->dc_suspended | reason;
	else
		suspended = chip->dc_suspended & ~reason;

	
	if (!!suspended == !!chip->dc_suspended)
		goto out;

	rc = smbchg_dc_suspend(chip, suspended != 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set dc suspend: %d rc = %d\n",
			suspended, rc);
		goto out;
	}

	if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
		power_supply_changed(&chip->dc_psy);
	pr_smb(PR_STATUS, "dc charging %s, suspended = %02x\n",
			suspended == 0 ? "enabled"
			: "suspended", suspended);
out:
	chip->dc_suspended = suspended;
	mutex_unlock(&chip->dc_en_lock);
	return rc;
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = usb_current_table[i];
	return rc;
}

static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;
	bool changed;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
#ifdef CONFIG_HTC_BATT_8960
	static bool first_time = true;
#endif

	if (!chip->batt_present) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}
#ifdef CONFIG_HTC_BATT_8960
	if(is_limit_IUSB && pre_usb_max_current_ma == USB_MA_950
		&& current_ma > USB_MA_1000){
		pr_smb(PR_STATUS, "Charger is bad, force limit current %d -> %dmA\n",current_ma, USB_MA_950);
		current_ma = USB_MA_950;
	}

	
	current_ma = filter_usb_max_chg_currnt(current_ma);

	
	if ((first_time == false) &&
			(pre_current_ma == current_ma)) {
		
		return 0;
	}
	first_time = false;
	pre_current_ma = current_ma;
#endif
	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma == SUSPEND_CURRENT_MA) {
		
		rc = smbchg_primary_usb_en(chip, false, REASON_USB, &changed);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = smbchg_primary_usb_en(chip, true, REASON_USB, &changed);
	}

	read_usb_type(chip, &usb_type_name, &usb_supply_type);

	switch (usb_supply_type) {
	case POWER_SUPPLY_TYPE_USB:
		if ((current_ma < CURRENT_150_MA) &&
				(chip->wa_flags & SMBCHG_USB100_WA))
			current_ma = CURRENT_150_MA;

		if (current_ma < CURRENT_150_MA) {
			
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 100;
		}
		
		if (current_ma == CURRENT_150_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 150;
		}
		if (current_ma == CURRENT_500_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 500;
		}
		if (current_ma == CURRENT_900_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->usb_max_current_ma = 900;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (current_ma < CURRENT_1500_MA) {
			
			rc = smbchg_masked_write(chip,
					chip->usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (rc < 0)
				pr_err("Couldn't set override rc = %d\n", rc);
		}
		
	default:
		rc = smbchg_set_high_usb_chg_current(chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;
	}

out:
	pr_smb(PR_STATUS, "usb type = %s current set to %d mA\n",
			usb_type_name, chip->usb_max_current_ma);
#ifdef CONFIG_HTC_BATT_8960
	if(chip->usb_max_current_ma > pre_usb_max_current_ma) {
		if (delayed_work_pending(&chip->retry_aicl_work))
			cancel_delayed_work_sync(&chip->retry_aicl_work);
		pr_smb(PR_STATUS, "Trigger re-do AICL when usb current goes higher, USB_MAX changed from %d mA -> %d mA\n",
				pre_usb_max_current_ma, current_ma);
		
		smbchg_rerun_aicl(chip);
	}
	pre_usb_max_current_ma = chip->usb_max_current_ma;
#endif
	return rc;
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
static int smbchg_get_min_parallel_current_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->parallel.min_9v_current_thr_ma;
	return chip->parallel.min_current_thr_ma;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	30000
static bool smbchg_is_parallel_usb_ok(struct smbchg_chip *chip)
{
	int min_current_thr_ma, rc, type;
	ktime_t kt_since_last_disable;
	u8 reg;

	if (!smbchg_parallel_en || !chip->parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
					chip->parallel.last_disabled);
	if (chip->parallel.current_max_ma == 0
		&& chip->parallel.enabled_once
		&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	if (!!(reg & USBIN_SUSPEND_STS_BIT) ||
				!(reg & USBIN_ACTIVE_PWR_SRC_BIT)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}
	if (chip->usb_tl_current_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
			chip->usb_tl_current_ma, min_current_thr_ma);
		return false;
	}

	return true;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 cur_val;

	
	for (i = ARRAY_SIZE(usb_current_table) - 1; i >= 0; i--) {
		if (current_ma >= usb_current_table[i])
			break;
	}
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "fastcharge current requested %d, set to %d\n",
			current_ma, usb_current_table[cur_val]);

	chip->fastchg_current_ma = usb_current_table[cur_val];
	return rc;
}

static int smbchg_set_fastchg_current(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

#ifdef CONFIG_HTC_BATT_8960
	pr_smb(PR_STATUS, "sw_esr_pulse_en=%d, current_ma=%d, "
			"chg_limit_current=%d\n",
			chip->sw_esr_pulse_en, current_ma, chg_limit_current);
#endif

	mutex_lock(&chip->fcc_lock);
	if (chip->sw_esr_pulse_en)
		current_ma = 300;
#ifdef CONFIG_HTC_BATT_8960
	if (chg_limit_current != 0 && current_ma >= 0)
		current_ma = MIN(current_ma, chg_limit_current);
#endif
	
	if (current_ma == chip->fastchg_current_ma) {
		pr_smb(PR_STATUS, "not configuring FCC current: %d FCC: %d\n",
			current_ma, chip->fastchg_current_ma);
		goto out;
	}
	rc = smbchg_set_fastchg_current_raw(chip, current_ma);
out:
	mutex_unlock(&chip->fcc_lock);
	return rc;
}

static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	chip->sw_esr_pulse_en = en;
	rc = smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	if (rc)
		return rc;
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
	pr_smb(PR_STATUS, "Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct smbchg_chip *chip, bool en)
{
	mutex_lock(&chip->taper_irq_lock);
	if (en != chip->taper_irq_enabled) {
		if (en) {
			enable_irq(chip->taper_irq);
			enable_irq_wake(chip->taper_irq);
		} else {
			disable_irq_wake(chip->taper_irq);
			disable_irq_nosync(chip->taper_irq);
		}
		chip->taper_irq_enabled = en;
	}
	mutex_unlock(&chip->taper_irq_lock);
}

static void smbchg_parallel_usb_disable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->parallel.initial_aicl_ma = 0;
	chip->parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
				SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	chip->target_fastchg_current_ma = chip->cfg_fastchg_current_ma;
	smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	chip->usb_tl_current_ma =
		calc_thermal_limited_current(chip, chip->usb_target_current_ma);
	smbchg_set_usb_current_max(chip, chip->usb_tl_current_ma);
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
				|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
			* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
		pval.intval);
	
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	mutex_unlock(&chip->parallel.lock);
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

static bool smbchg_is_aicl_complete(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return true;
	}
	return (reg & AICL_STS_BIT) != 0;
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return usb_current_table[reg];
}

#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
static void smbchg_parallel_usb_enable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int current_limit_ma, parallel_cl_ma, total_current_ma;
	int new_parallel_cl_ma, min_current_thr_ma, rc;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");
	
	if (chip->cfg_fastchg_current_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "suspend parallel charger as FCC is %d\n",
			chip->cfg_fastchg_current_ma);
		goto disable_parallel;
	}
	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		goto disable_parallel;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		goto disable_parallel;

	if (chip->parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
				current_limit_ma, min_current_thr_ma);
			goto disable_parallel;
		}
		chip->parallel.initial_aicl_ma = current_limit_ma;
	}

	parallel_cl_ma = chip->parallel.current_max_ma;
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	total_current_ma = current_limit_ma + parallel_cl_ma;

	if (total_current_ma < chip->parallel.initial_aicl_ma
			- chip->parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
			"Too little total current : %d (%d + %d) < %d - %d\n",
			total_current_ma,
			current_limit_ma, parallel_cl_ma,
			chip->parallel.initial_aicl_ma,
			chip->parallel.allowed_lowering_ma);
		goto disable_parallel;
	}

	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc) {
		dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
			rc);
		goto disable_parallel;
	}
	chip->target_fastchg_current_ma = chip->cfg_fastchg_current_ma / 2;
	smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	pval.intval = chip->target_fastchg_current_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);

	chip->parallel.enabled_once = true;
	new_parallel_cl_ma = total_current_ma / 2;

	if (new_parallel_cl_ma == parallel_cl_ma) {
		pr_smb(PR_STATUS,
			"AICL at %d, old ICL: %d new ICL: %d, skipping\n",
			current_limit_ma, parallel_cl_ma, new_parallel_cl_ma);
		return;
	} else {
		pr_smb(PR_STATUS, "AICL at %d, old ICL: %d new ICL: %d\n",
			current_limit_ma, parallel_cl_ma, new_parallel_cl_ma);
	}

	taper_irq_en(chip, true);
	chip->parallel.current_max_ma = new_parallel_cl_ma;
	power_supply_set_present(parallel_psy, true);
	smbchg_set_usb_current_max(chip, chip->parallel.current_max_ma);
	power_supply_set_current_limit(parallel_psy,
				chip->parallel.current_max_ma * 1000);
	return;

disable_parallel:
	if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "disabling parallel charger\n");
		smbchg_parallel_usb_disable(chip);
	} else if (chip->cfg_fastchg_current_ma !=
			chip->target_fastchg_current_ma) {
		rc = smbchg_set_fastchg_current(chip,
				chip->cfg_fastchg_current_ma);
		if (rc)
			pr_err("Couldn't set fastchg current rc: %d\n",
				rc);
		else
			chip->target_fastchg_current_ma =
				chip->cfg_fastchg_current_ma;
	}
}

static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				parallel_en_work.work);

	smbchg_relax(chip, PM_PARALLEL_CHECK);
	mutex_lock(&chip->parallel.lock);
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_parallel_usb_enable(chip);
	} else if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "parallel charging unavailable\n");
		smbchg_parallel_usb_disable(chip);
	}
	mutex_unlock(&chip->parallel.lock);
}

#define PARALLEL_CHARGER_EN_DELAY_MS	3500
static void smbchg_parallel_usb_check_ok(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	mutex_lock(&chip->parallel.lock);
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
		schedule_delayed_work(
			&chip->parallel_en_work,
			msecs_to_jiffies(PARALLEL_CHARGER_EN_DELAY_MS));
	} else if (chip->parallel.current_max_ma != 0) {
		pr_smb(PR_STATUS, "parallel charging unavailable\n");
		smbchg_parallel_usb_disable(chip);
	}
	mutex_unlock(&chip->parallel.lock);
}

static int smbchg_usb_en(struct smbchg_chip *chip, bool enable,
		enum enable_reason reason)
{
	bool changed = false;
	int rc = smbchg_primary_usb_en(chip, enable, reason, &changed);

	if (changed)
		smbchg_parallel_usb_check_ok(chip);
	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int smbchg_set_fastchg_current_user(struct smbchg_chip *chip,
							int current_ma)
{
	int rc = 0;

	mutex_lock(&chip->parallel.lock);
	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);
	chip->cfg_fastchg_current_ma = current_ma;
	if (smbchg_is_parallel_usb_ok(chip)) {
		smbchg_parallel_usb_enable(chip);
	} else {
		if (chip->parallel.current_max_ma != 0) {
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
			goto out;
		}
		rc = smbchg_set_fastchg_current(chip,
				chip->cfg_fastchg_current_ma);
		if (rc)
			pr_err("Couldn't set fastchg current rc: %d\n",
				rc);
	}
out:
	mutex_unlock(&chip->parallel.lock);
	return rc;
}
#endif

static struct ilim_entry *smbchg_wipower_find_entry(struct smbchg_chip *chip,
				struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
			uv))
			ret = &map->entries[i];
	}
	return ret;
}

static int ilim_ma_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct smbchg_chip *chip, int offset, int ma)
{
	int i, rc;

	for (i = ARRAY_SIZE(ilim_ma_table) - 1; i >= 0; i--) {
		if (ma >= ilim_ma_table[i])
			break;
	}

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct smbchg_chip *chip,
						struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->current_ilim.vmax_uv)
		return 0;

	chip->param.channel = DCIN;
	chip->param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->param.timer_interval = wipower_dcin_interval;
	chip->param.threshold_notification = &btm_notify_dcin;
	chip->param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->vadc_dev, &chip->param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->current_ilim.vmin_uv = ilim->vmin_uv;
		chip->current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct smbchg_chip *chip,
						int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->wipower_div2 : &chip->wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct smbchg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->wipower_default.entries[0]);

	if (!chip->wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->wipower_configured = false;
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	chip->current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct smbchg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			 && !usb_present
			&& dc_present
			&& chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		
		rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
			"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
			chg_type, usb_present, dc_present, dcin_uv,
			adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
			"deconfig ICL chg_type = %d usb = %d dc = %d\n",
			chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct smbchg_chip *chip)
{
	if (!chip->wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct smbchg_chip *chip = ctx;

	mutex_lock(&chip->wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct smbchg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

static int smbchg_set_thermal_limited_dc_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	current_ma = calc_thermal_limited_current(chip, current_ma);
	return smbchg_set_dc_current_max(chip, current_ma);
}

static int smbchg_set_thermal_limited_usb_current_max(struct smbchg_chip *chip,
							int current_ma)
{
	int rc, aicl_ma;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	chip->usb_tl_current_ma =
		calc_thermal_limited_current(chip, current_ma);
	rc = smbchg_set_usb_current_max(chip, chip->usb_tl_current_ma);
	if (rc) {
		pr_err("Failed to set usb current max: %d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "AICL = %d, ICL = %d\n",
			aicl_ma, chip->usb_max_current_ma);
	if (chip->usb_max_current_ma > aicl_ma && smbchg_is_aicl_complete(chip))
		smbchg_rerun_aicl(chip);
	smbchg_parallel_usb_check_ok(chip);
	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		rc = smbchg_dc_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, false, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	rc = smbchg_set_thermal_limited_usb_current_max(chip,
					chip->usb_target_current_ma);
	rc = smbchg_set_thermal_limited_dc_current_max(chip,
					chip->dc_target_current_ma);

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		rc = smbchg_dc_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smbchg_usb_en(chip, true, REASON_THERMAL);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}
#endif 

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	
	vin_flash_uv = max((chip->vled_max_uv + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
		"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#ifdef CONFIG_HTC_BATT_8960
int pmi8994_calc_max_flash_current(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return smbchg_calc_max_flash_current(the_chip);
}
#endif 

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct smbchg_chip *chip,
					int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fcc_comp_table); i++)
		if (comp_current == fcc_comp_table[i])
			break;

	if (i >= ARRAY_SIZE(fcc_comp_table))
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
			rc);

	return rc;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
#ifndef CONFIG_HTC_BATT_8960
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
			rc);

	return rc;
}
#endif 

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
				/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
				/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
				/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
				rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->vfloat_mv = vfloat_mv;

	return rc;
}

#ifdef CONFIG_HTC_BATT_8960
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int vfloat_cmp_mv = code;
	u8 temp;

	if ((vfloat_cmp_mv < MIN_FLOAT_MV) || (vfloat_cmp_mv > MAX_FLOAT_MV)) {
		pr_err("bad float comp voltage mv=%d asked to set\n",
					vfloat_cmp_mv);
		return -EINVAL;
	}

	if (vfloat_cmp_mv > chip->vfloat_mv) {
		pr_err("bad float_comp_mv=%d > float_mv=%d, skip setting\n",
                                        vfloat_cmp_mv, chip->vfloat_mv);
		return 0;
	}

	
	if (vfloat_cmp_mv <= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv >= VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = ((HIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
			+ (chip->vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV))
				/ MID_RANGE_FLOAT_STEP_MV
			+ (VHIGH_RANGE_FLOAT_MIN_MV - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv >= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = (chip->vfloat_mv - vfloat_cmp_mv)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv <= HIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv > HIGH_RANGE_FLOAT_MIN_MV) {
		temp = (HIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
				/ MID_RANGE_FLOAT_STEP_MV
			+ (chip->vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_cmp_mv <= VHIGH_RANGE_FLOAT_MIN_MV
			&& chip->vfloat_mv > VHIGH_RANGE_FLOAT_MIN_MV) {
		temp = (VHIGH_RANGE_FLOAT_MIN_MV - vfloat_cmp_mv)
				/ HIGH_RANGE_FLOAT_STEP_MV
			+ (chip->vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	} else {
		temp = (chip->vfloat_mv - vfloat_cmp_mv)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	}

	return smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, temp);
}
#endif 

#ifndef CONFIG_HTC_BATT_8960
static int smbchg_float_voltage_get(struct smbchg_chip *chip)
{
	return chip->vfloat_mv;
}
#endif

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
#ifndef CONFIG_HTC_BATT_8960
static int smbchg_safety_timer_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s safety timer rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->safety_timer_en = enable;
	return 0;
}
#endif 

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
#ifndef CONFIG_HTC_BATT_8960
static int smbchg_force_tlim_en(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg force tlim rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}
#endif 

static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	
	if (chip->schg_version == QPNP_SCHG_LITE)
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}

#ifdef CONFIG_HTC_BATT_8960
static void smbchg_check_and_notify_fg_soc(struct smbchg_chip *chip)
{
	bool charger_present, charger_suspended;

	charger_present = is_usb_present(chip) | is_dc_present(chip);
	charger_suspended = chip->usb_suspended & chip->dc_suspended;

	if (charger_present && !charger_suspended
		&& get_prop_batt_status(chip)
			== POWER_SUPPLY_STATUS_CHARGING) {
		pr_smb(PR_STATUS, "Adjusting battery soc in FG\n");
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
								POWER_SUPPLY_PROP_CHARGE_FULL);
	}
}

static int smbchg_get_prop_voltage_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}

	return 0;
}

static int smbchg_get_prop_current_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}

static int smbchg_get_prop_capacity_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };
	static int fg_notified;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if ((ret.intval == 100) &&
				(!fg_notified) &&
				(get_prop_charge_type(chip) == POWER_SUPPLY_CHARGE_TYPE_TAPER
					|| get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_FULL)) {
			fg_notified = 1;
			smbchg_check_and_notify_fg_soc(chip);
		} else
			fg_notified = 0;

		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}

static int smbchg_get_prop_batt_temp_now(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		if ((ret.intval >= 650) &&
				(flag_keep_charge_on || flag_pa_fake_batt_temp))
			ret.intval = 650;
		return ret.intval;
	} else {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	}
	return 0;
}
#endif 

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_en(struct smbchg_chip *chip, bool en)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, en ? AICL_RERUN_ON : AICL_RERUN_OFF);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);
	return rc;
}

static int smbchg_aicl_config(struct smbchg_chip *chip)
{
	int rc = 0;

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
	if (rc) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	if (!chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Couldn't enable AICL rerun rc= %d\n", rc);
	}
	return rc;
}

static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	if (chip->force_aicl_rerun)
		return;
	if (en && !chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_SHORT);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
		if (!chip->very_weak_charger) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc) {
				pr_err("Couldn't enable AICL rerun rc= %d\n",
						rc);
				return;
			}
		}
		pr_smb(PR_STATUS, "AICL deglitch set to short\n");
	} else if (!en && chip->aicl_deglitch_short) {
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG,
			USB_AICL_DEGLITCH_MASK, USB_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
			return;
		}
		rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + DC_AICL_CFG,
			DC_AICL_DEGLITCH_MASK, DC_AICL_DEGLITCH_LONG);
		if (rc) {
			pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
			return;
		}
#ifndef CONFIG_HTC_BATT_8960 
		rc = smbchg_hw_aicl_rerun_en(chip, false);
		if (rc) {
			pr_err("Couldn't disable AICL rerun rc= %d\n", rc);
			return;
		}
#endif
		pr_smb(PR_STATUS, "AICL deglitch set to normal\n");
	}
	chip->aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	u8 reg;
	bool low_volt_chgr = true;

	if (!(chip->wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->bms_psy)
		return;

	if (is_usb_present(chip)) {
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
		if (rc < 0) {
			pr_err("Couldn't read hvdcp status rc = %d\n", rc);
			return;
		}
		if (reg & USBIN_HVDCP_SEL_BIT)
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->vbat_above_headroom) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->vbat_above_headroom);
}

#ifdef CONFIG_HTC_BATT_8960
void smbchg_aicl_deglitch_wa_ext_check(void)
{
	if (!the_chip) {
		pr_smb(PR_STATUS,
			"smbcharger not initialized, do nothing.\n");
		return;
	}

	if (!the_chip->bms_psy)
		return;

	smbchg_aicl_deglitch_wa_check(the_chip);
}
#endif

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
#ifndef CONFIG_HTC_BATT_8960
static int smbchg_config_chg_battery_type(struct smbchg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
		!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	
	if (chip->battery_type && !strcmp(prop.strval, chip->battery_type))
		return 0;

	batt_node = of_parse_phandle(node, "qcom,battery-data", 0);
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node,
							"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->battery_type = prop.strval;

	
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
						&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
				chip->vfloat_mv, (max_voltage_uv / 1000),
				chip->battery_type);
			rc = smbchg_float_voltage_set(chip,
						(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	if (!of_find_property(chip->spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
				"fastchg-ma changed from %dma to %dma for battery-type %s\n",
				chip->target_fastchg_current_ma, fastchg_ma,
				chip->battery_type);
			chip->target_fastchg_current_ma = fastchg_ma;
			chip->cfg_fastchg_current_ma = fastchg_ma;
			rc = smbchg_set_fastchg_current(chip, fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set fastchg current rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	return ret;
}
#endif 

static void check_battery_type(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;
	bool unused;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		smbchg_battchg_en(chip, en, REASON_BATTCHG_UNKNOWN_BATTERY,
				&unused);
	}
}

#ifndef CONFIG_HTC_BATT_8960
static void smbchg_external_power_changed(struct power_supply *psy)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, soc;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->bms_psy) {
		check_battery_type(chip);
		soc = get_prop_batt_capacity(chip);
		if (chip->previous_soc != soc) {
			chip->previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
				"Couldn't update charger configuration rc=%d\n",
									rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc == 0)
		smbchg_usb_en(chip, prop.intval, REASON_POWER_SUPPLY);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc == 0)
		current_limit = prop.intval / 1000;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type != POWER_SUPPLY_TYPE_USB)
		goto  skip_current_for_non_sdp;

	pr_smb(PR_MISC, "usb type = %s current_limit = %d\n",
			usb_type_name, current_limit);
	mutex_lock(&chip->current_change_lock);
	if (current_limit != chip->usb_target_current_ma) {
		pr_smb(PR_STATUS, "changed current_limit = %d\n",
				current_limit);
		chip->usb_target_current_ma = current_limit;
		rc = smbchg_set_thermal_limited_usb_current_max(chip,
				current_limit);
		if (rc < 0)
			dev_err(chip->dev,
				"Couldn't set usb current rc = %d\n", rc);
	}
	mutex_unlock(&chip->current_change_lock);

skip_current_for_non_sdp:
	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
}
#endif 

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);
	
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_CONT	0x2
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	bool changed;
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_primary_usb_en(chip, false, REASON_OTG, &changed);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	bool changed;
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_primary_usb_en(chip, true, REASON_OTG, &changed);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, chip->original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	return !smbchg_primary_usb_is_en(chip, REASON_OTG);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return 0;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->ext_otg_vreg.rdev = regulator_register(
					&chip->ext_otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->ext_otg_vreg.rdev);
			chip->ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static void smbchg_regulator_deinit(struct smbchg_chip *chip)
{
	if (chip->otg_vreg.rdev)
		regulator_unregister(chip->otg_vreg.rdev);
	if (chip->ext_otg_vreg.rdev)
		regulator_unregister(chip->ext_otg_vreg.rdev);
}

#define CMD_CHG_LED_REG		0x43
#define CHG_LED_CTRL_BIT		BIT(0)
#define LED_SW_CTRL_BIT		0x1
#define LED_CHG_CTRL_BIT		0x0
#define CHG_LED_ON		0x03
#define CHG_LED_OFF		0x00
#define LED_BLINKING_PATTERN1		0x01
#define LED_BLINKING_PATTERN2		0x02
#define LED_BLINKING_CFG_MASK		SMB_MASK(2, 1)
#define CHG_LED_SHIFT		1
static int smbchg_chg_led_controls(struct smbchg_chip *chip)
{
	u8 reg, mask;
	int rc;

	if (chip->cfg_chg_led_sw_ctrl) {
		
		mask = CHG_LED_CTRL_BIT | LED_BLINKING_CFG_MASK;
		reg = LED_SW_CTRL_BIT;
	} else {
		mask = CHG_LED_CTRL_BIT;
		reg = LED_CHG_CTRL_BIT;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_LED_REG,
			mask, reg);
	if (rc < 0)
		dev_err(chip->dev,
				"Couldn't write LED_CTRL_BIT rc=%d\n", rc);
	return rc;
}

static void smbchg_chg_led_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg;
	int rc;

	reg = (value > LED_OFF) ? CHG_LED_ON << CHG_LED_SHIFT :
		CHG_LED_OFF << CHG_LED_SHIFT;

	pr_smb(PR_STATUS,
			"set the charger led brightness to value=%d\n",
			value);
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static enum
led_brightness smbchg_chg_led_brightness_get(struct led_classdev *cdev)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg_val, chg_led_sts;
	int rc;

	rc = smbchg_read(chip, &reg_val, chip->bat_if_base + CMD_CHG_LED_REG,
			1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read CHG_LED_REG sts rc=%d\n",
				rc);
		return rc;
	}

	chg_led_sts = (reg_val & LED_BLINKING_CFG_MASK) >> CHG_LED_SHIFT;

	pr_smb(PR_STATUS, "chg_led_sts = %02x\n", chg_led_sts);

	return (chg_led_sts == CHG_LED_OFF) ? LED_OFF : LED_FULL;
}

static void smbchg_chg_led_blink_set(struct smbchg_chip *chip,
		unsigned long blinking)
{
	u8 reg;
	int rc;

	if (blinking == 0)
		reg = CHG_LED_OFF << CHG_LED_SHIFT;
	else if (blinking == 1)
		reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
	else if (blinking == 2)
		reg = LED_BLINKING_PATTERN2 << CHG_LED_SHIFT;
	else
		reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;

	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static ssize_t smbchg_chg_led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct smbchg_chip *chip = container_of(cdev, struct smbchg_chip,
			led_cdev);
	unsigned long blinking;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &blinking);
	if (rc)
		return rc;

	smbchg_chg_led_blink_set(chip, blinking);

	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, smbchg_chg_led_blink_store);

static struct attribute *led_blink_attributes[] = {
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group smbchg_led_attr_group = {
	.attrs = led_blink_attributes
};

static int smbchg_register_chg_led(struct smbchg_chip *chip)
{
	int rc;

	chip->led_cdev.name = "red";
	chip->led_cdev.brightness_set = smbchg_chg_led_brightness_set;
	chip->led_cdev.brightness_get = smbchg_chg_led_brightness_get;

	rc = led_classdev_register(chip->dev, &chip->led_cdev);
	if (rc) {
		dev_err(chip->dev, "unable to register charger led, rc=%d\n",
				rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->led_cdev.dev->kobj,
			&smbchg_led_attr_group);
	if (rc) {
		dev_err(chip->dev, "led sysfs rc: %d\n", rc);
		return rc;
	}

	return rc;
}

#ifdef CONFIG_HTC_BATT_8960
static void smbchg_load_jeita_setting(struct smbchg_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
				power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
		return;
	}

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_WARM_TEMP, &ret);
	chip->warm_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_COOL_TEMP, &ret);
	chip->cool_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_HOT_TEMP, &ret);
	chip->hot_bat_decidegc = ret.intval;

	chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_COLD_TEMP, &ret);
	chip->cold_bat_decidegc = ret.intval;

	pr_smb(PR_STATUS, "cool_bat_dec=%d, warm_bat_dec=%d, "
					"cold_bat_dec=%d, hot_bat_dec=%d\n",
			chip->cool_bat_decidegc, chip->warm_bat_decidegc,
			chip->cold_bat_decidegc, chip->hot_bat_decidegc);

}

static void smbchg_adc_notification(enum qpnp_tm_state_smbchg state, void *ctx)
{
	union power_supply_propval ret = {0, };
	struct smbchg_chip *chip = ctx;
	int temp;

	if (state >= ADC_TM_STATE_NUM_SMBCHG) {
		pr_smb(PR_STATUS, "invalid notification %d\n", state);
		return;
	}

	
	if (!chip->warm_bat_decidegc)
		smbchg_load_jeita_setting(chip);

	temp = smbchg_get_prop_batt_temp_now(chip);

	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "bms_psy doesn't exist!\n");
	} else {
		if (state == ADC_TM_HOT_STATE_SMBCHG) {
			if (chip->batt_hot) {
				
				ret.intval = chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_HOT_TEMP, &ret);

			} else {
				
				ret.intval = chip->hot_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_HOT_TEMP, &ret);
			}
		} else if (state == ADC_TM_WARM_STATE_SMBCHG) {
			if (chip->batt_warm) {
				
				ret.intval = chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_WARM_TEMP, &ret);
			} else {
				
				ret.intval = chip->warm_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_WARM_TEMP, &ret);
			}
		} else if (state == ADC_TM_COOL_STATE_SMBCHG) {
			if (chip->batt_cool) {
				
				ret.intval = chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COOL_TEMP, &ret);

			} else {
				
				ret.intval = chip->cool_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COOL_TEMP, &ret);
			}
		} else if (state == ADC_TM_COLD_STATE_SMBCHG) {
			if (chip->batt_cold) {
				
				ret.intval = chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COLD_TEMP, &ret);

			} else {
				
				ret.intval = chip->cold_bat_decidegc;
				chip->bms_psy->set_property(chip->bms_psy,
						POWER_SUPPLY_PROP_COLD_TEMP, &ret);
			}
		}
	}
	pr_smb(PR_STATUS, "ret.intval=%d, batt_warm=%d, batt_cool=%d, batt_hot=%d, "
					"bat_cold=%d\n",
					ret.intval, chip->batt_warm, chip->batt_cool, chip->batt_hot,
					chip->batt_cold);
	htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);

}

static void smbchg_notify_fg_chg_status(struct smbchg_chip *chip)
{
       if (get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_CHARGING) {
               pr_smb(PR_STATUS, "notifying FG on charging\n");
               set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
                               POWER_SUPPLY_STATUS_CHARGING);
       } else if (get_prop_batt_status(chip) ==
                       POWER_SUPPLY_STATUS_DISCHARGING) {
               pr_smb(PR_STATUS, "notifying FG on discharging\n");
               set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
                               POWER_SUPPLY_STATUS_DISCHARGING);
       }
}

static void smbchg_notify_fg_work(struct work_struct *work)
{
       struct smbchg_chip *chip = container_of(work,
                               struct smbchg_chip,
                               notify_fg_work);

       smbchg_notify_fg_chg_status(chip);
}
#endif 

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

static bool is_hvdcp_present(struct  smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		pr_err("Couldn't read hvdcp status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	if (chip->schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip))
		return true;

	return false;
}

#define HVDCP_ADAPTER_SEL_MASK	SMB_MASK(5, 4)
#define HVDCP_5V		0x00
#define HVDCP_9V		0x10
#define USB_CMD_HVDCP_1		0x42
#define FORCE_HVDCP_2p0		BIT(3)
static int force_9v_hvdcp(struct smbchg_chip *chip)
{
	int rc;

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc) {
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		return rc;
	}

	
	rc = smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, FORCE_HVDCP_2p0);
	rc |= smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, 0);
	if (rc < 0) {
		pr_err("Couldn't force QC2.0 rc=%d\n", rc);
		return rc;
	}

	
	msleep(500);

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc)
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	return rc;
}

static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				hvdcp_det_work.work);
	int rc;

	if (is_hvdcp_present(chip)) {
		if (!chip->hvdcp3_supported &&
			(chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
			
			rc = force_9v_hvdcp(chip);
			if (rc)
				pr_err("could not force 9V HVDCP continuing rc=%d\n",
						rc);
		}
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_USB_HVDCP);
		power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
	}
}

static int set_usb_psy_dp_dm(struct smbchg_chip *chip, int state)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (!rc && !(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "overwriting state = %d with %d\n",
				state, POWER_SUPPLY_DP_DM_DPF_DMF);
		state = POWER_SUPPLY_DP_DM_DPF_DMF;
	}
	pr_smb(PR_MISC, "setting usb psy dp dm = %d\n", state);
	return power_supply_set_dp_dm(chip->usb_psy, state);
}

#define APSD_CFG		0xF5
#define AUTO_SRC_DETECT_EN_BIT	BIT(0)
#define APSD_TIMEOUT_MS		1500
static void restore_from_hvdcp_detection(struct smbchg_chip *chip)
{
	int rc;

	
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0)
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_9V_CONT);
	if (rc < 0)
		pr_err("Couldn't write usb allowance rc=%d\n", rc);

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable AICL rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	chip->pulse_cnt = 0;
}

#ifdef CONFIG_HTC_BATT_8960
static void smbchg_adjust_batt_soc_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
				struct smbchg_chip,
				batt_soc_work);

	if (smbchg_get_prop_capacity_now(chip) == 100)
		smbchg_check_and_notify_fg_soc(chip);
}

int pmi8994_prepare_suspend(void)
{
	struct timespec xtime;
	unsigned long cur_jiffies;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	xtime = CURRENT_TIME;
	cur_jiffies = jiffies;
	dischg_plus_sr_time += (cur_jiffies -
			dischg_last_jiffies) * MSEC_PER_SEC / HZ;
	dischg_last_jiffies = cur_jiffies;
	dischg_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
					xtime.tv_nsec / NSEC_PER_MSEC;

	pr_smb(PR_STATUS, "total dischg time=%lu ms before suspend.\n",
			dischg_plus_sr_time);

	return 0;
}

int pmi8994_complete_resume(void)
{
	unsigned long resume_ms;
	unsigned long sr_time_period_ms;
	struct timespec xtime;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	xtime = CURRENT_TIME;
	dischg_last_jiffies = jiffies;
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	sr_time_period_ms = resume_ms - dischg_suspend_ms;
	dischg_plus_sr_time += sr_time_period_ms;

	pr_smb(PR_STATUS, "sr_time_period=%lu ms; total dischg time=%lu ms "
			"after suspend.\n", sr_time_period_ms, dischg_plus_sr_time);

	return 0;
}

int pm8994_set_hsml_target_ma(int target_ma)
{
	int rc = 1;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	pr_smb(PR_STATUS, "USB max charging current target_ma = %d\n", target_ma);

	
	if(target_ma != 0 &&
		the_chip->power_source_type == HTC_PWR_SOURCE_TYPE_USB ){
		mutex_lock(&the_chip->current_change_lock);
		the_chip->usb_target_current_ma = target_ma;
		rc = smbchg_set_thermal_limited_usb_current_max(the_chip, target_ma);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&the_chip->current_change_lock);
	}
	return rc;
}
#endif 

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;

	pr_smb(PR_STATUS, "triggered\n");
	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->force_aicl_rerun && !chip->very_weak_charger) {
		rc = smbchg_hw_aicl_rerun_en(chip, true);
		if (rc)
			pr_err("Error enabling AICL rerun rc= %d\n",
				rc);
	}
	
	if (chip->usb_ov_det)
		chip->usb_ov_det = false;
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_UNKNOWN);
#ifndef CONFIG_HTC_BATT_8960
		power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_UNKNOWN);
#endif
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPR_DMR);
#ifndef CONFIG_HTC_BATT_8960
		schedule_work(&chip->usb_set_online_work);
#else
		if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, msecs_to_jiffies(100));
#endif
		pr_smb(PR_MISC, "setting usb psy health UNKNOWN\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNKNOWN);
		if (rc)
			pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_UNKNOWN, rc);
	}
	if (parallel_psy && chip->parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->parallel.avail && chip->aicl_done_irq
			&& chip->enable_aicl_wake) {
		disable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = false;
	}
	chip->parallel.enabled_once = false;
	chip->vbat_above_headroom = false;
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't set override rc = %d\n", rc);

	restore_from_hvdcp_detection(chip);
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

#ifndef CONFIG_HTC_BATT_8960
static bool is_usbin_uv_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_UV_BIT;
}
#endif 

#define HVDCP_NOTIFY_MS		2500
#define DEFAULT_WALL_CHG_MA	1800
#define DEFAULT_SDP_MA		100
#define DEFAULT_CDP_MA		1500
#ifndef CONFIG_HTC_BATT_8960
static void handle_usb_insertion(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";

	pr_smb(PR_STATUS, "triggered\n");
	
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_smb(PR_STATUS,
		"inserted type = %d (%s)", usb_supply_type, usb_type_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy type = %d\n",
				usb_supply_type);
		power_supply_set_supply_type(chip->usb_psy, usb_supply_type);
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		
		if (!chip->usb_ov_det) {
			pr_smb(PR_MISC, "setting usb psy health %s\n",
					chip->very_weak_charger
					? "UNSPEC_FAILURE" : "GOOD");
			rc = power_supply_set_health_state(chip->usb_psy,
					chip->very_weak_charger
					? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
					: POWER_SUPPLY_HEALTH_GOOD);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_GOOD, rc);
		}
		schedule_work(&chip->usb_set_online_work);
	}

	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));

	mutex_lock(&chip->current_change_lock);
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB)
		chip->usb_target_current_ma = DEFAULT_SDP_MA;
	else if (usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP)
		chip->usb_target_current_ma = DEFAULT_CDP_MA;
	else
		chip->usb_target_current_ma = DEFAULT_WALL_CHG_MA;

	pr_smb(PR_STATUS, "%s detected setting mA = %d\n",
		usb_type_name, chip->usb_target_current_ma);
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
				chip->usb_target_current_ma);
	mutex_unlock(&chip->current_change_lock);

	if (parallel_psy) {
		rc = power_supply_set_present(parallel_psy, true);
		chip->parallel_charger_detected = rc ? false : true;
		if (rc)
			pr_debug("parallel-charger absent rc=%d\n", rc);
	}

	if (chip->parallel.avail && chip->aicl_done_irq
			&& !chip->enable_aicl_wake) {
		rc = enable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = true;
	}
}
#endif 

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
#ifndef CONFIG_HTC_BATT_8960
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}

	
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
#else
	cable_detection_vbus_irq_handler();
	schedule_work(&chip->notify_fg_work);
#endif 
	mutex_unlock(&chip->usb_status_lock);
}

static int otg_oc_reset(struct smbchg_chip *chip)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);

	if (!is_otg_present(chip)) {
		pr_smb(PR_STATUS,
			"OTG is not present, not enabling OTG_EN_BIT\n");
		goto out;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

out:
	return rc;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
#ifdef CONFIG_HTC_BATT_8960
#define RETRY_AICL_TOTAL	2
#define RETRY_AICL_INTERVAL_MS	180000
#define INTERVAL_1_MINUTE_MS	60000
#endif
static void increment_aicl_count(struct smbchg_chip *chip)
{
	bool bad_charger = false;
	int rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;
#ifdef CONFIG_HTC_BATT_8960
	int aicl_result = smbchg_get_aicl_level_ma(chip);
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
#endif

#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds);
#else
	read_usb_type(chip, &usb_type_name, &usb_supply_type);

	pr_smb(PR_INTERRUPT, "Aicl triggered icl:%d c:%d dgltch:%d first:%ld, "
			"usb_type:%d\n",
			smbchg_get_aicl_level_ma(chip),
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds,
			usb_supply_type);
#endif 

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->aicl_complete = reg & AICL_STS_BIT;
	else
		chip->aicl_complete = false;

	if (chip->aicl_deglitch_short || chip->force_aicl_rerun) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > AICL_IRQ_LIMIT_COUNT) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->first_aicl_seconds,
				now_seconds, chip->aicl_irq_count);
#ifdef CONFIG_HTC_BATT_8960
			
			rc = smbchg_usb_en(chip, true, REASON_USB);
			rc = smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				USBIN_LIMITED_MODE | USB51_100MA);
			
			chip->usb_max_current_ma = USB_MA_150;
			pre_current_ma = pre_usb_max_current_ma = USB_MA_150;

			pr_smb(PR_INTERRUPT, "Bad cable, set current ma: %d\n", USB_MA_150);

			
			if (delayed_work_pending(&chip->usb_limit_max_current))
				cancel_delayed_work(&chip->usb_limit_max_current);
			schedule_delayed_work(&chip->usb_limit_max_current,
				msecs_to_jiffies(INTERVAL_1_MINUTE_MS));
#else
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			chip->very_weak_charger = true;
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("could not enable aicl reruns: %d", rc);
			bad_charger = true;
#endif
			chip->aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			chip->very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			pr_smb(PR_MISC,
				"setting usb psy health UNSPEC_FAILURE\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
#ifndef CONFIG_HTC_BATT_8960
			schedule_work(&chip->usb_set_online_work);
#else
			if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
			schedule_delayed_work(&chip->usb_set_online_work, 0);
#endif
		}
	}

#ifdef CONFIG_HTC_BATT_8960
	
	if ((reg & AICL_STS_BIT) && (is_qc20_done_flag == false) &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		
		if ((((aicl_result == USB_MA_1100) || (aicl_result == USB_MA_1200)) ||
		
				((is_limit_IUSB == true) && (aicl_result > USB_MA_1000)))) {
			gs_is_limit_1A = true;
			
			retry_aicl_cnt = 0;
		} else if (aicl_result <= USB_MA_500 &&
				retry_aicl_cnt < RETRY_AICL_TOTAL) {
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			pr_smb(PR_STATUS, "Trigger re-do AICL in 3 minutes, cnt=%d\n",
									retry_aicl_cnt);
			schedule_delayed_work(&chip->retry_aicl_work,
								msecs_to_jiffies(RETRY_AICL_INTERVAL_MS));
		}
	}
#endif 
}

#ifndef CONFIG_HTC_BATT_8960
static int wait_for_usbin_uv(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->usbin_uv_lowered;
	bool usbin_uv;

	if (high)
		completion = &chip->usbin_uv_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	usbin_uv = is_usbin_uv_high(chip);

	if (high == usbin_uv)
		return 0;

	pr_err("usbin uv didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			usbin_uv ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int wait_for_src_detect(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->src_det_lowered;
	bool src_detect;

	if (high)
		completion = &chip->src_det_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	src_detect = is_src_detect_high(chip);

	if (high == src_detect)
		return 0;

	pr_err("src detect didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			src_detect ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion)
{
	int rc;
	bool src_detect;
	bool usbin_uv;

	if (insertion) {
		INIT_COMPLETION(chip->src_det_raised);
		INIT_COMPLETION(chip->usbin_uv_lowered);
	} else {
		INIT_COMPLETION(chip->src_det_lowered);
		INIT_COMPLETION(chip->usbin_uv_raised);
	}

	
	usbin_uv = is_usbin_uv_high(chip);
	if (usbin_uv != insertion) {
		pr_err("Skip faking, usbin uv is already %d\n", usbin_uv);
		return -EINVAL;
	}

	
	src_detect = is_src_detect_high(chip);
	if (src_detect == insertion) {
		pr_err("Skip faking, src detect is already %d\n", src_detect);
		return -EINVAL;
	}

	pr_smb(PR_MISC, "Allow only %s charger\n",
			insertion ? "5-9V" : "9V only");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK,
			insertion ?
			USBIN_ADAPTER_5V_9V_CONT : USBIN_ADAPTER_9V);
	if (rc < 0) {
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s usbin uv\n",
			insertion ? "falling" : "rising");
	rc = wait_for_usbin_uv(chip, !insertion);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s src det\n",
			insertion ? "rising" : "falling");
	rc = wait_for_src_detect(chip, insertion);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int smbchg_prepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 reg;

	
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	
	msleep(500);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);
		goto out;
	}

	
	pr_smb(PR_MISC, "Reduce mA = 300\n");
	mutex_lock(&chip->current_change_lock);
	chip->target_fastchg_current_ma = 300;
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
			chip->target_fastchg_current_ma);
	mutex_unlock(&chip->current_change_lock);
	if (rc < 0) {
		pr_err("Couldn't set usb current rc=%d continuing\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;
	
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
		goto handle_removal;
	}

	
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
		goto out;
	}

	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto handle_removal;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DMF);
	msleep(HVDCP_NOTIFY_MS);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if ((reg >> TYPE_BITS_OFFSET) != 0) {
		pr_smb(PR_MISC, "type bits set after 2s sleep - abort\n");
		rc = -EINVAL;
		goto out;
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DM3P3);
	
	msleep(60);

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
handle_removal:
	chip->hvdcp_3_det_ignore_uv = false;
	update_usb_status(chip, 0, 0);
	return rc;
}

static int smbchg_unprepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPF_DMF);

	
	pr_smb(PR_MISC, "Switch to 9V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Enabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable APSD rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Disable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable AICL rc=%d\n", rc);
		return rc;
	}

	
	chip->hvdcp_3_det_ignore_uv = true;
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal rc=%d\n", rc);
		goto out;
	}

	
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto out;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	
	pr_smb(PR_MISC, "Enable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable AICL rc=%d\n", rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Reset ICL\n");
	mutex_lock(&chip->current_change_lock);
	chip->usb_target_current_ma = DEFAULT_WALL_CHG_MA;
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
			chip->usb_target_current_ma);
	mutex_unlock(&chip->current_change_lock);
	if (rc < 0)
		pr_err("Couldn't set usb current rc=%d continuing\n", rc);

out:
	chip->hvdcp_3_det_ignore_uv = false;
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}
	return rc;
}

#define USB_CMD_APSD		0x41
#define APSD_RERUN		BIT(0)
static int rerun_apsd(struct smbchg_chip *chip)
{
	int rc;

	INIT_COMPLETION(chip->src_det_raised);
	INIT_COMPLETION(chip->usbin_uv_lowered);
	INIT_COMPLETION(chip->src_det_lowered);
	INIT_COMPLETION(chip->usbin_uv_raised);

	
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + USB_CMD_APSD,
					APSD_RERUN, APSD_RERUN);
	if (rc) {
		pr_err("Couldn't re-run APSD rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising usbin uv\n");
	rc = wait_for_usbin_uv(chip, true);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling src det\n");
	rc = wait_for_src_detect(chip, false);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling usbin uv\n");
	rc = wait_for_usbin_uv(chip, false);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising src det\n");
	rc = wait_for_src_detect(chip, true);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#define SCHG_LITE_USBIN_HVDCP_5_9V		0x8
#define SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK	0x38
#define SCHG_LITE_USBIN_HVDCP_SEL_IDLE		BIT(3)
static bool is_hvdcp_5v_cont_mode(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
		chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc) {
		pr_err("Unable to read HVDCP status rc=%d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP status = %x\n", reg);

	if (reg & SCHG_LITE_USBIN_HVDCP_SEL_IDLE) {
		rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + INPUT_STS, 1);
		if (rc) {
			pr_err("Unable to read INPUT status rc=%d\n", rc);
			return false;
		}
		pr_smb(PR_STATUS, "INPUT status = %x\n", reg);
		if ((reg & SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK) ==
					SCHG_LITE_USBIN_HVDCP_5_9V)
			return true;
	}
	return false;
}

static int smbchg_prepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	
	if (is_hvdcp_5v_cont_mode(chip)) {
		pr_smb(PR_MISC, "HVDCP by default is in 5V continuous mode\n");
		return 0;
	}

	
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	
	msleep(500);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	
	pr_smb(PR_MISC, "Reduce mA = 300\n");
	mutex_lock(&chip->current_change_lock);
	chip->target_fastchg_current_ma = 300;
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
			chip->target_fastchg_current_ma);
	mutex_unlock(&chip->current_change_lock);
	if (rc < 0) {
		pr_err("Couldn't set usb current rc=%d continuing\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;

	
	rc = rerun_apsd(chip);
	if (rc) {
		pr_err("APSD rerun failed\n");
		goto out;
	}

	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	msleep(HVDCP_NOTIFY_MS);
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	
	if (!is_hvdcp_5v_cont_mode(chip)) {
		pr_err("HVDCP could not be set in 5V continuous mode\n");
		goto out;
	}

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
}

static int smbchg_unprepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
	rc = force_9v_hvdcp(chip);
	if (rc) {
		pr_err("Failed to force 9V HVDCP=%d\n",	rc);
		return rc;
	}

	
	pr_smb(PR_MISC, "Reset ICL\n");
	mutex_lock(&chip->current_change_lock);
	chip->usb_target_current_ma = DEFAULT_WALL_CHG_MA;
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
			chip->usb_target_current_ma);
	mutex_unlock(&chip->current_change_lock);
	if (rc < 0)
		pr_err("Couldn't set usb current rc=%d continuing\n", rc);

	return rc;
}

#define CMD_HVDCP_2		0x43
#define SINGLE_INCREMENT	BIT(0)
#define SINGLE_DECREMENT	BIT(1)
static int smbchg_dp_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Increment DP\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_INCREMENT, SINGLE_INCREMENT);
	if (rc)
		pr_err("Single-increment failed rc=%d\n", rc);

	return rc;
}


static int smbchg_dm_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Decrement DM\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_DECREMENT, SINGLE_DECREMENT);
	if (rc)
		pr_err("Single-decrement failed rc=%d\n", rc);

	return rc;
}

static int smbchg_hvdcp3_confirmed(struct smbchg_chip *chip)
{
	int rc;

	
	pr_smb(PR_MISC, "Reset ICL\n");
	mutex_lock(&chip->current_change_lock);
	chip->usb_target_current_ma = DEFAULT_WALL_CHG_MA;
	rc = smbchg_set_thermal_limited_usb_current_max(chip,
			chip->usb_target_current_ma);
	mutex_unlock(&chip->current_change_lock);
	if (rc < 0)
		pr_err("Couldn't set usb current rc=%d continuing\n", rc);

	pr_smb(PR_MISC, "setting usb psy type = %d\n",
				POWER_SUPPLY_TYPE_USB_HVDCP_3);
	power_supply_set_supply_type(chip->usb_psy,
				POWER_SUPPLY_TYPE_USB_HVDCP_3);
	return 0;
}

static int smbchg_dp_dm(struct smbchg_chip *chip, int val)
{
	int rc = 0;

	switch (val) {
	case POWER_SUPPLY_DP_DM_PREPARE:
		if (!is_hvdcp_present(chip)) {
			pr_err("No pulsing unless HVDCP\n");
			return -ENODEV;
		}
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_prepare_for_pulsing_lite(chip);
		else
			rc = smbchg_prepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_UNPREPARE:
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3:
		rc = smbchg_hvdcp3_confirmed(chip);
		break;
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pulse_cnt++;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pulse_cnt)
			chip->pulse_cnt--;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_HVDCP3_SUPPORTED:
		chip->hvdcp3_supported = true;
		pr_smb(PR_MISC, "HVDCP3 supported\n");
		break;
	default:
		break;
	}

	return rc;
}

static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_RERUN_AICL,
};

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	bool unused;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		smbchg_battchg_en(chip, val->intval,
				REASON_BATTCHG_USER, &unused);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smbchg_usb_en(chip, val->intval, REASON_USER);
		smbchg_dc_en(chip, val->intval, REASON_USER);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smbchg_dp_dm(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		smbchg_rerun_aicl(chip);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = smcghg_is_battchg_en(chip, REASON_BATTCHG_USER);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->aicl_complete;
		break;
	
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = smbchg_is_input_current_limited(chip);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = smbchg_dc_en(chip, val->intval, REASON_POWER_SUPPLY);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smbchg_set_dc_current_max(chip, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
				struct smbchg_chip, dc_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->dc_suspended == 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		
		val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
					== POWER_SUPPLY_STATUS_CHARGING);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->dc_max_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}
#endif 

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#else
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_hot=%d\n",
		reg, chip->batt_hot);

	if (irq != 0)
		smbchg_adc_notification(ADC_TM_HOT_STATE_SMBCHG, chip);
#endif
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#else
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_cold=%d\n",
		reg, chip->batt_cold);

	if (irq != 0)
		smbchg_adc_notification(ADC_TM_COLD_STATE_SMBCHG, chip);
#endif
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#else
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_warm=%d\n",
		reg, chip->batt_warm);

	if (irq != 0)
		smbchg_adc_notification(ADC_TM_WARM_STATE_SMBCHG, chip);
#endif
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
#else
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x, batt_cool=%d\n",
		reg, chip->batt_cool);

	if (irq != 0)
		smbchg_adc_notification(ADC_TM_COOL_STATE_SMBCHG, chip);
#endif
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
#define CHGR_SFT_TIMEOUT_RT_STS BIT(3)
#endif
static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
#ifdef CONFIG_HTC_BATT_8960
	u8 int_rt_sts = 0;
	int rc = 0;

	rc = smbchg_read(chip, &int_rt_sts, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read int_rt_sts (0x1010) rc=%d\n", rc);
	}
	pr_smb(PR_INTERRUPT, "chg-error triggered, int_rt_sts:0x%X\n", int_rt_sts);
#else
	pr_smb(PR_INTERRUPT, "chg-error triggered\n");
#endif
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
#ifdef CONFIG_HTC_BATT_8960
	if (int_rt_sts & CHGR_SFT_TIMEOUT_RT_STS) {
		pr_info("Safety timer (%dmins) timeout with AC cable = %d\n",
				chip->safety_time, is_ac_online());

		chip->is_ac_safety_timeout = true;
		htc_charger_event_notify(HTC_CHARGER_EVENT_SAFETY_TIMEOUT);
	} else
		dump_regs(chip);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "p2f triggered\n");
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
#ifdef CONFIG_HTC_BATT_8960
	wake_lock(&chip->eoc_worker_wlock);
	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#endif
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
static void smbchg_usb_limit_current_WA_work(struct work_struct *work)
{
	int rc = 0;
	if(!the_chip) {
		pr_err("called before init\n");
		return;
	}
	
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	pr_smb(PR_INTERRUPT, "AICL disabled\n");

	mutex_lock(&the_chip->current_change_lock);
	the_chip->usb_target_current_ma = USB_MA_1000;
	rc = smbchg_set_thermal_limited_usb_current_max(the_chip, USB_MA_1000);
	mutex_unlock(&the_chip->current_change_lock);
	if (rc < 0)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	else
		pr_smb(PR_INTERRUPT, "USB max current changed to 1A\n");


	
	msleep(50);
	smbchg_sec_masked_write(the_chip, the_chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	pr_smb(PR_INTERRUPT, "AICL re-enabled\n");
}
#endif 

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
#ifdef CONFIG_HTC_BATT_8960
	if(reg & BAT_TCC_REACHED_BIT) {
		is_batt_full_eoc_stop = true;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
	}
	chip->chg_done_batt_full = !!(reg & BAT_TCC_REACHED_BIT);
#endif
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
#ifdef CONFIG_HTC_BATT_8960
	if (reg & BAT_TCC_REACHED_BIT)
		schedule_work(&chip->batt_soc_work);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t safety_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("safety timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
#define POWER_OK_BIT   BIT(0)
#define OCP_RELEASE_TIME_UPPER_BOUND_MS	180000
static int count_same_dischg = 0;
#endif
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
#ifndef CONFIG_HTC_BATT_8960
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	return IRQ_HANDLED;
#else
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;
	int power_ok;
	long diff;
	unsigned long dischg_time_ms;
	unsigned long cur_jiffies;
	static int first = 1;
	static int prev_power_ok = -1;
	static unsigned long prev_dischg_time_ms = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);

	if (reg & POWER_OK_BIT)
		power_ok = 1;
	else
		power_ok = 0;

	cur_jiffies = jiffies;

	if ((power_ok == 0) && (prev_power_ok == 1)) {
		
		dischg_last_jiffies = cur_jiffies;
		dischg_plus_sr_time = 0;
		dischg_time_ms = 0;
		first = 0;
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x.\n", reg);
	} else if ((power_ok == 1) && (prev_power_ok == 0) && (first == 0)){
		
		dischg_time_ms = dischg_plus_sr_time +
		((cur_jiffies - dischg_last_jiffies) * MSEC_PER_SEC / HZ);

		
		
		diff = dischg_time_ms - prev_dischg_time_ms;

		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. "
				"prev_dischg_time_ms(%lu) dischg_time_ms(%lu) "
				"diff(%ld) count_same_dischg(%d)\n",
				reg,
				prev_dischg_time_ms,
				dischg_time_ms,
				diff,
				count_same_dischg);

		prev_dischg_time_ms = dischg_time_ms;

		if (ABS(diff) < 1200 &&
				dischg_time_ms <= OCP_RELEASE_TIME_UPPER_BOUND_MS) {
			if(++count_same_dischg == 3) {
				is_limit_IUSB = true;
				count_same_dischg = 0;
				pr_smb(PR_INTERRUPT, "Start to limit USB current\n");
				
				schedule_work(&chip->usb_aicl_limit_current);
			}
		} else {
			is_limit_IUSB = false;
			count_same_dischg = 0;
		}
	} else
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x. prev_power_ok(%d) "
				"power_ok(%d) \n", reg, prev_power_ok, power_ok);

	prev_power_ok = power_ok;

	return IRQ_HANDLED;
#endif 
}

#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);
	int rc;

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		
		chip->dc_present = dc_present;
		if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
#ifdef CONFIG_HTC_BATT_8960
		schedule_work(&chip->notify_fg_work);
#endif
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		if (chip->force_aicl_rerun && !dc_present) {
			rc = smbchg_hw_aicl_rerun_en(chip, true);
			if (rc)
				pr_err("Error enabling AICL rerun rc= %d\n",
					rc);
		}
		chip->vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
int pmi8994_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (the_chip->ftm_src != ftm_src) {
		pr_info("%s(%d -> %d)\n", __func__, the_chip->ftm_src, ftm_src);
		the_chip->ftm_src = ftm_src;
	}

	return 0;
}

static u32 htc_fake_charger_for_ftm(enum htc_power_source_type src)
{
	unsigned int new_src = src;

	if((src <= HTC_PWR_SOURCE_TYPE_9VAC) && (src != HTC_PWR_SOURCE_TYPE_BATT)) {
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB)
			new_src = HTC_PWR_SOURCE_TYPE_USB;
		else if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			new_src = HTC_PWR_SOURCE_TYPE_AC;

		if (src != new_src)
			pr_info("%s(%d -> %d)\n", __func__, src , new_src);
	}

	return new_src;
}

static u32 htc_fake_charger_for_testing(enum htc_power_source_type src)
{
	
	enum htc_power_source_type new_src = HTC_PWR_SOURCE_TYPE_AC;

	if((src > HTC_PWR_SOURCE_TYPE_9VAC) || (src == HTC_PWR_SOURCE_TYPE_BATT))
		return src;

	pr_info("%s(%d -> %d)\n", __func__, src , new_src);
	return new_src;
}

static DEFINE_MUTEX(cable_notify_sem);
static void send_cable_connect_notify(int cable_type)
{
	static struct t_cable_status_notifier *notifier;

	mutex_lock(&cable_notify_sem);

	list_for_each_entry(notifier,
		&g_lh_calbe_detect_notifier_list,
		cable_notifier_link) {
			if (notifier->func != NULL) {
				pr_smb(PR_STATUS, "Send to: %s, type %d\n",
						notifier->name, cable_type);
				
				notifier->func(cable_type);
			}
		}
	mutex_unlock(&cable_notify_sem);

	wake_unlock(&the_chip->vbus_wlock);
}

int cable_detect_register_notifier(struct t_cable_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&cable_notify_sem);
	list_add(&notifier->cable_notifier_link,
		&g_lh_calbe_detect_notifier_list);
#if 0
	if (the_cable_info.notify_init == 1)
		notifier->func(cable_get_connect_type());
#endif
	mutex_unlock(&cable_notify_sem);
	return 0;
}

extern int usb_get_connect_type(void);
static void usb_type_polling(struct work_struct *work)
{
	int rc;
	union power_supply_propval prop = {0,};

	the_chip->usb_psy->get_property(the_chip->usb_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
	pr_info("USB present: %d\n", prop.intval);
	if (prop.intval == 0) {
		pr_info("USB cable out\n");
		return;
	}

	rc = usb_get_connect_type();
	switch(rc){
		case CONNECT_TYPE_NOTIFY :
			pr_info("USB type is under checking...\n");
			schedule_delayed_work(&the_chip->usb_type_polling_work, msecs_to_jiffies(2000));
			break;
		case CONNECT_TYPE_UNKNOWN :
		case CONNECT_TYPE_MHL_UNKNOWN :
			pr_info("USB type is unknown or MHL...send notify...%d\n", rc);
			send_cable_connect_notify(CONNECT_TYPE_UNKNOWN);
			break;
		default:
			pr_info("USB type: %d\n", rc);
			break;
	}
}

extern void cable_status_notifier_func(int cable_type);
static void check_cable_type(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip,
				vbus_detect_work);
	int connect_type, vbus_uv;
	char *usb_type_name = "null";
	enum power_supply_type usb_supply_type;
	bool usb_present = is_usb_present(chip);

	
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	vbus_uv = pmi8994_get_usbin_voltage_now(chip);

	pr_smb(PR_STATUS, "inserted %s, usb_type=%d, pre_usb_type=%d, "
					"vbus=%d, pre_usb_present=%d, usb_present=%d\n",
			usb_type_name, usb_supply_type, chip->pre_usb_supply_type,
			vbus_uv, chip->usb_present, usb_present);
	
	if (usb_supply_type == POWER_SUPPLY_TYPE_UNKNOWN && usb_present)
		usb_supply_type = chip->pre_usb_supply_type;
	else
		chip->pre_usb_supply_type = usb_supply_type;

	power_supply_set_supply_type(chip->usb_psy, usb_supply_type);

	if (!usb_present) {
		connect_type = CONNECT_TYPE_NONE;
	} else {
		switch (usb_supply_type) {
		case POWER_SUPPLY_TYPE_UNKNOWN :
			connect_type = CONNECT_TYPE_UNKNOWN;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP :
			connect_type = CONNECT_TYPE_AC;
			break;
		case POWER_SUPPLY_TYPE_USB_CDP :
		default :
			connect_type = CONNECT_TYPE_USB;
			break;
		}
	}
	if((usb_get_connect_type() > CONNECT_TYPE_MHL_UNKNOWN)
		&& (usb_get_connect_type() < CONNECT_TYPE_MHL_100MA)){ 
		cable_status_notifier_func(connect_type);
		send_cable_connect_notify(connect_type);
	}
}
#endif 

static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	
	if (reg & USBIN_OV_BIT) {
		chip->usb_ov_det = true;
		if (chip->usb_psy) {
			pr_smb(PR_MISC, "setting usb psy health OV\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->usb_ov_det = false;
		
		usb_present = is_usb_present(chip);
		if (usb_present)
			update_usb_status(chip, usb_present, false);
	}
#ifdef CONFIG_HTC_BATT_8960
	htc_charger_event_notify(HTC_CHARGER_EVENT_OVP);
#endif
out:
	return IRQ_HANDLED;
}

#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
#ifndef CONFIG_HTC_BATT_8960
	bool unused;
#endif
	u8 reg;
#ifdef CONFIG_HTC_BATT_8960
	bool usb_present = is_usb_present(chip);
	int vbus_uv = 0;
#endif

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}


#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, reg, chip->hvdcp_3_det_ignore_uv,
		aicl_level);
#else
	vbus_uv = pmi8994_get_usbin_voltage_now(chip);
	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d vbus_uv = %d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, reg, chip->hvdcp_3_det_ignore_uv,
		aicl_level, vbus_uv);
#endif

	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	if (reg & USBIN_UV_BIT)
		complete_all(&chip->usbin_uv_raised);
	else
		complete_all(&chip->usbin_uv_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
#ifndef CONFIG_HTC_BATT_8960 
			rc = smbchg_primary_usb_en(chip, false,
					REASON_WEAK_CHARGER, &unused);
			if (rc)
				pr_err("could not disable charger: %d", rc);
		} else if ((chip->aicl_deglitch_short || chip->force_aicl_rerun)
			&& aicl_level == usb_current_table[0]) {
			rc = smbchg_hw_aicl_rerun_en(chip, false);
			if (rc)
				pr_err("could not enable aicl reruns: %d", rc);
#else
			g_wa_sdp_chg_suspend_count++;
			pr_smb(PR_INTERRUPT, "WA_SDP_CHG_SUSPEND: g_wa_sdp_chg_suspend_count=%d\n",
				g_wa_sdp_chg_suspend_count);
#endif
		}
		pr_smb(PR_MISC, "setting usb psy health UNSPEC_FAILURE\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
#ifndef CONFIG_HTC_BATT_8960
		schedule_work(&chip->usb_set_online_work);
#else
		if (delayed_work_pending(&chip->usb_set_online_work))
			cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, 0);
#endif
	}

	smbchg_wipower_check(chip);

#ifdef CONFIG_HTC_BATT_8960
	if ((reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)
			&& chip->usb_present && !usb_present) {
		
		pr_smb(PR_STATUS, "cable-out event\n");
		cable_detection_vbus_irq_handler();
		schedule_work(&chip->notify_fg_work);
	}
#endif

out:
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
static irqreturn_t cable_detection_vbus_irq_handler(void)
{
	unsigned long flags;

	
	g_wa_adjust_iusb_max_pre_usb_target_current_ma = 0;
	g_wa_sdp_chg_suspend_count = 0;

	spin_lock_irqsave(&the_chip->vbus_lock, flags);

	if (delayed_work_pending(&the_chip->vbus_detect_work))
		cancel_delayed_work(&the_chip->vbus_detect_work);
	queue_delayed_work(the_chip->cable_detect_wq,
			&the_chip->vbus_detect_work, HZ/20*7);

	spin_unlock_irqrestore(&the_chip->vbus_lock, flags);

	wake_lock_timeout(&the_chip->vbus_wlock, HZ/2);

	pr_smb(PR_STATUS, "end\n");

	return IRQ_HANDLED;
}
#endif 

static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip), unused;
	bool src_detect = is_src_detect_high(chip);
	int rc;
#ifdef CONFIG_HTC_BATT_8960
	int vbus_uv = 0;
#endif

#ifdef CONFIG_HTC_BATT_8960
	vbus_uv = pmi8994_get_usbin_voltage_now(chip);

	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d vbus_uv=%d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, src_detect,
		chip->hvdcp_3_det_ignore_uv, vbus_uv);
#else
	pr_smb(PR_STATUS,
		"%s chip->usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, src_detect,
		chip->hvdcp_3_det_ignore_uv);
#endif

	if (src_detect)
		complete_all(&chip->src_det_raised);
	else
		complete_all(&chip->src_det_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

#ifdef CONFIG_HTC_BATT_8960
	
	if (flag_force_ac_chg && usb_present) {
		smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USB_AICL_CFG, AICL_EN_BIT, 0);
		pr_smb(PR_STATUS, "ATS: disable AICL\n");
	}
#endif

	chip->very_weak_charger = false;
	rc = smbchg_primary_usb_en(chip, true, REASON_WEAK_CHARGER, &unused);
	if (rc < 0)
		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		update_usb_status(chip, usb_present, 0);
	} else {
		update_usb_status(chip, 0, false);
		chip->aicl_irq_count = 0;
	}
out:
	return IRQ_HANDLED;
}

#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->schg_version == QPNP_SCHG_LITE) {
		pr_warn("OTG OC triggered - OTG disabled\n");
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
static int is_HVDCP_9V_done(void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return FALSE;
	}

	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v)) {
		return TRUE; 
	}

	return FALSE;
}
#endif 

static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);

#ifndef CONFIG_HTC_BATT_8960
	if (chip->aicl_complete)
		power_supply_changed(&chip->batt_psy);
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
static void
retry_aicl_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, retry_aicl_work);
	smbchg_rerun_aicl(chip);
	retry_aicl_cnt++;
}

static void
hvdcp_5to9V_worker(struct work_struct *work)
{
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
			struct smbchg_chip, hvdcp_5to9V_work);

	pr_smb(PR_STATUS, "Set HVDCP 5V to 9V "
			"by setting 0x13F4<5:4>\n");

	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CFG_SYSMIN, 0x30, 0x10);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set 0x13F4 = %d\n",
				rc);
	}
}
#endif 

static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

	pr_smb(PR_INTERRUPT, "triggered\n");

	otg_present = is_otg_present(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy OTG = %d\n",
				otg_present ? 1 : 0);
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	}
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));

	return IRQ_HANDLED;
}

#ifdef CONFIG_HTC_BATT_8960
static void
smbchg_usb_limit_max_current_work(struct work_struct *work)
{
	if(!the_chip)
	{
		pr_err("called before init\n");
		return;
	}

	mutex_lock(&the_chip->current_change_lock);
	pr_smb(PR_STATUS, "USB max current changed to %d\n",USB_MA_1500);
	smbchg_set_thermal_limited_usb_current_max(the_chip,USB_MA_1500);
	mutex_unlock(&the_chip->current_change_lock);
}

static void
smbchg_adjust_fastchg_current(int vbat_mv)
{
	int current_ma = 0;

	if (get_prop_batt_health(the_chip) != POWER_SUPPLY_HEALTH_GOOD) {
		
		is_over_fastchg_thr_once = false;

		if (the_chip->fastchg_current_comp != -EINVAL)
			current_ma = the_chip->fastchg_current_comp;
		else
			current_ma = the_chip->target_fastchg_current_ma;
	} else {
		if (vbat_mv <= the_chip->adjust_fastchg_cur_thr_mv &&
				the_chip->fastchg_current_ma != the_chip->target_fastchg_current_ma &&
				is_over_fastchg_thr_once == false) {
			current_ma = the_chip->target_fastchg_current_ma;
		} else if (vbat_mv > the_chip->adjust_fastchg_cur_thr_mv &&
				the_chip->fastchg_current_ma != the_chip->cv_fastchg_cur_ma) {
			is_over_fastchg_thr_once = true;
			current_ma = the_chip->cv_fastchg_cur_ma;
		}
	}

	if (current_ma != 0)
		smbchg_set_fastchg_current(the_chip, current_ma);
}

static void
smbchg_eoc_work(struct work_struct *work)
{
	int rc, sys_sts, soc = 0;
	u8 chgr_sts = 0, int_rt_sts = 0, chg_type;
	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, eoc_work);
	int vbat_mv, ibat_ma;

	wake_lock(&chip->eoc_worker_wlock);

	rc = smbchg_read(chip, &chgr_sts, chip->chgr_base + CHGR_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read chgr_sts (0x100E) rc=%d\n", rc);
		return;
	}

	rc = smbchg_read(chip, &int_rt_sts, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "failed to read int_rt_sts (0x1010) rc=%d\n", rc);
		return;
	}

	chg_type = (chgr_sts & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	vbat_mv = smbchg_get_prop_voltage_now(chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(chip)/1000;
	rc = pmi8994_fg_get_system_status(&sys_sts);

	if (rc)
		pr_err("failed to read sys_sts (0x4107) rc=%d\n", rc);

	pr_info("vbat_mv=%d, ibat_ma=%d, 0x100E:0x%x, 0x1010:0x%x, chg_type:%d,"
		" is_qc20_done=%d, eoc_count=%d\n", vbat_mv, ibat_ma, chgr_sts, int_rt_sts,
		chg_type, is_qc20_done_flag, eoc_count);

	
	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "no charger connected, stopping\n");
		is_batt_full = false;
		is_batt_full_eoc_stop = false;
		
		g_wa_adjust_iusb_max_pre_usb_target_current_ma = 0;
		g_wa_sdp_chg_suspend_count = 0;
		goto stop_eoc;
	}

	
	if (chip->adjust_fastchg_cur_thr_mv != -EINVAL &&
			chip->cv_fastchg_cur_ma != -EINVAL)
		smbchg_adjust_fastchg_current(vbat_mv);

	
	if (is_HVDCP_9V_done(chip) && (is_qc20_done_flag == false)) {
		is_qc20_done_flag = true;
		mutex_lock(&chip->current_change_lock);
		chip->usb_target_current_ma = chip->qc20_usb_max_current_ma;
		rc = smbchg_set_thermal_limited_usb_current_max(chip,
				chip->qc20_usb_max_current_ma);
		mutex_unlock(&chip->current_change_lock);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		else
			pr_smb(PR_STATUS, "QC2.0 max current changed to %d mA\n",
					chip->qc20_usb_max_current_ma);
	}

	
	if (gs_is_limit_1A == true) {
		gs_is_limit_1A = false;
		mutex_lock(&chip->current_change_lock);
		chip->usb_target_current_ma = USB_MA_1000;
		rc = smbchg_set_thermal_limited_usb_current_max(chip, USB_MA_1000);
		mutex_unlock(&chip->current_change_lock);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		else
			pr_smb(PR_INTERRUPT, "USB max current changed to 1A\n");
	}

	
	if (chip->wa_adjust_iusb_max_to_prevent_over_chg_from_1A_adaptor &&
			!is_HVDCP_9V_done(chip) &&
			chip->usb_target_current_ma >= USB_MA_1000) {
		int mA = chip->usb_target_current_ma;

		if (g_wa_adjust_iusb_max_pre_usb_target_current_ma == 0) {
			if (vbat_mv > 4000) {
				
				g_wa_adjust_iusb_max_pre_usb_target_current_ma = chip->usb_target_current_ma;
				mA = USB_MA_1000;
			} else {
				g_wa_adjust_iusb_max_pre_usb_target_current_ma = -1;
			}
		} else if (g_wa_adjust_iusb_max_pre_usb_target_current_ma != -1){
			
			if (vbat_mv < 3900) {
				
				mA = g_wa_adjust_iusb_max_pre_usb_target_current_ma;
				g_wa_adjust_iusb_max_pre_usb_target_current_ma = -1;
			}
		}

		if (chip->usb_target_current_ma != mA) {
			mutex_lock(&chip->current_change_lock);
			chip->usb_target_current_ma = mA;
			rc = smbchg_set_thermal_limited_usb_current_max(chip, chip->usb_target_current_ma);
			mutex_unlock(&chip->current_change_lock);
			if (rc < 0)
				pr_err("WA: Couldn't set usb current rc = %d\n", rc);
			else
				pr_smb(PR_INTERRUPT, "WA: USB max current changed to %d mA\n", chip->usb_target_current_ma);
		}
	}

	
	if ((int_rt_sts & P2F_CHG_THRESHOLD_BIT) == 0) {
		pr_info("not charging by eoc stop charging\n");
		if (int_rt_sts & CHG_ERROR_BIT &&
				!(int_rt_sts & CHGR_SFT_TIMEOUT_RT_STS)) {
			dump_regs(chip);
			panic("[BATT]Charging stop due to CHGR_ERROR\n");
		}
		goto stop_eoc;
	} else {
		
		if (chg_type != BATT_TAPER_CHG_VAL) {
			pr_debug("Not in CV\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if (ibat_ma > 0) {
			
			pr_smb(PR_DUMP, "Charging but system demand increased\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->eoc_ibat_thre_ma) {
			
			pr_smb(PR_DUMP, "Not at EOC, battery current too high\n");
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->iterm_ma) {
			
			pr_smb(PR_DUMP, "start count for HTC battery full condition\n");
			eoc_count_by_curr = 0;
			eoc_count++;
			if (eoc_count == CONSECUTIVE_COUNT) {
				
				is_batt_full = true;
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		} else {
			eoc_count++;
			
			if (eoc_count_by_curr == CONSECUTIVE_COUNT) {
				
				pr_info("End of Charging\n");
				is_batt_full_eoc_stop = true;
				goto stop_eoc;
			} else {
				if (eoc_count == CONSECUTIVE_COUNT && !is_batt_full) {
					is_batt_full = true;
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
				eoc_count_by_curr += 1;
				pr_info("eoc_count_by_curr = %d, eoc_count=%d\n",
					eoc_count_by_curr, eoc_count);
			}
		}
	}

	if (is_batt_full) {
		soc = smbchg_get_prop_capacity_now(chip);
		if (soc < CLEAR_FULL_STATE_BY_LEVEL_THR) {
			
			if (chip->vfloat_mv && 
				(vbat_mv > (chip->vfloat_mv - 100))) {
				pr_info("Not satisfy overloading battery voltage"
					" critiria (%dmV < %dmV).\n", vbat_mv,
					(chip->vfloat_mv - 100));
			} else {
				is_batt_full = false;
				eoc_count = eoc_count_by_curr = 0;
				pr_info("%s: Clear is_batt_full & eoc_count due to"
					" Overloading happened, soc=%d\n",
					__func__, soc);
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		}
	}

	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	return;

stop_eoc:
	eoc_count_by_curr = eoc_count = 0;
	wake_unlock(&chip->eoc_worker_wlock);
}

int pmi8994_is_batt_temperature_fault(int *result)
{
	if(!the_chip) {
		pr_smb(PR_STATUS, "called before init\n");
		return -EINVAL;
	}
	*result = get_prop_batt_health(the_chip);

	if (*result != POWER_SUPPLY_HEALTH_GOOD &&
			*result != POWER_SUPPLY_HEALTH_COOL)
		*result = 1;
	else
		*result = 0;
	return 0;
}

#define BATT_CHG_WARM_THR_MV	4000
int pmi8994_is_batt_temp_fault_disable_chg(int *result)
{
	int batt_temp_status, vbat_mv, is_vbatt_over_vddmax;
	int is_cold = 0, is_hot = 0, is_warm = 0;
	int warm_thr_mv;

	if (!the_chip) {
		 pr_smb(PR_STATUS, "called before init\n");
		return -EINVAL;
	}

	batt_temp_status = get_prop_batt_health(the_chip);

	vbat_mv = smbchg_get_prop_voltage_now(the_chip) / 1000;

	if (batt_temp_status == POWER_SUPPLY_HEALTH_OVERHEAT)
		is_hot = 1;
	if (batt_temp_status == POWER_SUPPLY_HEALTH_WARM)
		is_warm = 1;
	if (batt_temp_status == POWER_SUPPLY_HEALTH_COLD)
		is_cold = 1;

	if (the_chip->float_voltage_comp &&
			the_chip->float_voltage_comp != -EINVAL)
		warm_thr_mv = the_chip->float_voltage_comp;
	else
		warm_thr_mv = BATT_CHG_WARM_THR_MV;

	if(vbat_mv >= warm_thr_mv)
		is_vbatt_over_vddmax = true;
	else
		is_vbatt_over_vddmax = false;

	pr_smb(PR_STATUS, "is_cold=%d, is_hot=%d, is_warm=%d, "
			"is_vbatt_over_vddmax=%d\n",
			is_cold, is_hot, is_warm, is_vbatt_over_vddmax);
#if 1
	if ((is_cold || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_disable_temp_protection)
#else
	if ((is_cold || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_pa_recharge)
#endif
		*result = 1;
	else
		*result = 0;

	return 0;
}

#define EN_BAT_CHG			BIT(1)
#define CMD_CHG			0x42
static int
smbchg_charge_en(struct smbchg_chip *chip, int enable)
{
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG,
			EN_BAT_CHG, enable ? 0 : EN_BAT_CHG);
}

static int
smbchg_force_run_on_batt(struct smbchg_chip *chip, int disable)
{
#if 0

	
	if (!qpnp_chg_is_batt_present(chip))
		return 0;
#endif
	return smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, disable ? USBIN_SUSPEND_BIT : 0);
}

static DEFINE_SPINLOCK(charger_lock);
static int batt_charging_disabled; 
#define BATT_CHG_DISABLED_BIT_EOC	(1)
#define BATT_CHG_DISABLED_BIT_KDRV	(1<<1)
#define BATT_CHG_DISABLED_BIT_USR1	(1<<2)
#define BATT_CHG_DISABLED_BIT_USR2	(1<<3)
#define BATT_CHG_DISABLED_BIT_BAT	(1<<4)

static int smbchg_disable_auto_enable(struct smbchg_chip *chip,
		int disable, int reason)
{
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&charger_lock, flags);
	if (disable)
		batt_charging_disabled |= reason;	
	else
		batt_charging_disabled &= ~reason;	
	rc = smbchg_charge_en(the_chip, !batt_charging_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	spin_unlock_irqrestore(&charger_lock, flags);

	return rc;
}

static DEFINE_SPINLOCK(pwrsrc_lock);
#define PWRSRC_DISABLED_BIT_KDRV	(1)
#define PWRSRC_DISABLED_BIT_USER	(1<<1)
#define PWRSRC_DISABLED_BIT_AICL		(1<<2)
#define PWRSRC_DISABLED_BIT_FTM		(1<<3)

static int pwrsrc_disabled; 

static int smbchg_disable_pwrsrc(struct smbchg_chip *chip,
		int disable, int reason)
{
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&pwrsrc_lock, flags);
	if (disable)
		pwrsrc_disabled |= reason;	
	else
		pwrsrc_disabled &= ~reason;	
	rc = smbchg_force_run_on_batt(chip, pwrsrc_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	spin_unlock_irqrestore(&pwrsrc_lock, flags);

	return rc;
}

int pmi8994_charger_enable(bool enable)
{
	int rc = 0;

	if (the_chip) {
		enable = !!enable;
		rc = smbchg_disable_auto_enable(the_chip, !enable, BATT_CHG_DISABLED_BIT_KDRV);
	} else {
		pr_err("called before init\n");
		rc = -EINVAL;
	}

	return rc;
}

int pmi8994_pwrsrc_enable(bool enable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

#ifdef CONFIG_HTC_BATT_8960
	if (!enable)
		count_same_dischg = 0;
#endif

	return smbchg_disable_pwrsrc(the_chip, !enable, PWRSRC_DISABLED_BIT_KDRV);
}

static int pmi8952_chg_safety_timer_set(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;

	if (!is_ac_online() || chip->safety_time == 0 ||
			flag_keep_charge_on || flag_disable_safety_timer)
		
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't config safety timer rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static void handle_usb_present_change(struct smbchg_chip *chip,
				int usb_present)
{
	int soc = 0;
	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;
		chip->is_ac_safety_timeout = false;

		
		
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		if (delayed_work_pending(&chip->usb_set_online_work))
				 cancel_delayed_work_sync(&chip->usb_set_online_work);
		schedule_delayed_work(&chip->usb_set_online_work, msecs_to_jiffies(100));

		if (!usb_present) {
			is_batt_full = false;
			is_batt_full_eoc_stop = false;
			is_qc20_done_flag = false;
			is_over_fastchg_thr_once = false;
			gs_is_limit_1A = false;
			
			retry_aicl_cnt = 0;
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			
			if (delayed_work_pending(&chip->usb_limit_max_current))
				cancel_delayed_work_sync(&chip->usb_limit_max_current);
			smbchg_aicl_deglitch_wa_check(chip);
			chip->vbat_above_headroom = false;

			if (flag_force_ac_chg) {
				
				smbchg_sec_masked_write(chip,
					chip->usb_chgpth_base + USB_AICL_CFG,
					AICL_EN_BIT, AICL_EN_BIT);
				pr_smb(PR_STATUS, "ATS: Re-enable AICL\n");
			}
		} else {
			wake_lock(&chip->eoc_worker_wlock);
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

			smbchg_aicl_deglitch_wa_check(chip);

			soc = smbchg_get_prop_capacity_now(the_chip);
			if (chip->previous_soc != soc) {
				chip->previous_soc = soc;
				smbchg_soc_changed(the_chip);
			}
			
			pmi8952_chg_safety_timer_set(chip);
		}
	}
}

int pmi8994_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
		bool chg_enable, bool pwrsrc_enable)
{
	int mA = 0, rc = 0;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	read_usb_type(the_chip, &usb_type_name, &usb_supply_type);

	pr_smb(PR_STATUS, "src=%d, chg_enable=%d, pwrsrc_enable=%d, ftm_src=%d, "
					"usb_type=%d\n",
				src, chg_enable, pwrsrc_enable, the_chip->ftm_src,
				usb_supply_type);

	if (flag_force_ac_chg)
		src = htc_fake_charger_for_testing(src);

	if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB ||
			the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
		src = htc_fake_charger_for_ftm(src);

	if (src < HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN) {
		
		smbchg_disable_pwrsrc(the_chip, !pwrsrc_enable, PWRSRC_DISABLED_BIT_KDRV);

		
		smbchg_disable_auto_enable(the_chip,
				!chg_enable, BATT_CHG_DISABLED_BIT_KDRV);
	}

	
	switch (src) {
	case HTC_PWR_SOURCE_TYPE_BATT:
		mA = USB_MA_2;
		break;
	case HTC_PWR_SOURCE_TYPE_WIRELESS:
		mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_DETECTING:
	case HTC_PWR_SOURCE_TYPE_UNKNOWN_USB:
	case HTC_PWR_SOURCE_TYPE_USB:
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB) {
			mA = USB_MA_500;
		} else if (!is_device_XA_or_XB) {
			if (usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP)
				mA = USB_MA_1000;
			else
				mA = USB_MA_500;
		} else {
			mA = USB_MA_1000;
		}
		break;
	case HTC_PWR_SOURCE_TYPE_AC:
	case HTC_PWR_SOURCE_TYPE_9VAC:
	case HTC_PWR_SOURCE_TYPE_MHL_AC:
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			mA = USB_MA_1000;
		else if (flag_force_ac_chg)
			mA = USB_MA_1000;
		else
			mA = USB_MA_1500;
		break;
    case HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN:
		mA = USB_MA_0;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_100MA:
		mA = USB_MA_100;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_500MA:
		mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_900MA:
		mA = USB_MA_900;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_1500MA:
		mA = USB_MA_1500;
		break;
	case HTC_PWR_SOURCE_TYPE_MHL_2000MA:
		mA = USB_MA_1500;
		break;
	default:
		mA = USB_MA_2;
		break;
	}

	mutex_lock(&the_chip->current_change_lock);
	the_chip->power_source_type = src;
	the_chip->usb_target_current_ma = mA;
	rc = smbchg_set_thermal_limited_usb_current_max(the_chip, mA);
	if (rc < 0)
		pr_err("Couldn't set usb current rc = %d\n", rc);
	mutex_unlock(&the_chip->current_change_lock);

	if (HTC_PWR_SOURCE_TYPE_BATT == src)
		handle_usb_present_change(the_chip, 0);
	else if (HTC_PWR_SOURCE_TYPE_MHL_UNKNOWN <= src) {
		if (is_usb_present(the_chip)) {
			handle_usb_present_change(the_chip, 1);
		} else {
			handle_usb_present_change(the_chip, 0);
		}
	} else
		handle_usb_present_change(the_chip, 1);

	if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_NONE_STOP)
		smbchg_disable_pwrsrc(the_chip, true, PWRSRC_DISABLED_BIT_FTM);
	else
		smbchg_disable_pwrsrc(the_chip, false, PWRSRC_DISABLED_BIT_FTM);

	return 0;
}

static int
pmi8994_get_usbin_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(chip->vadc_dev)) {
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "usbin");
		if (IS_ERR(chip->vadc_dev))
			return PTR_ERR(chip->vadc_dev);
	}

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int is_ac_online(void)
{
	struct smbchg_chip *chip = the_chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return is_usb_present(chip) &&
			(chip->power_source_type == HTC_PWR_SOURCE_TYPE_AC ||
					chip->power_source_type == HTC_PWR_SOURCE_TYPE_9VAC ||
					chip->power_source_type == HTC_PWR_SOURCE_TYPE_MHL_AC ||
					chip->power_source_type == HTC_PWR_SOURCE_TYPE_MHL_900MA ||
					chip->power_source_type == HTC_PWR_SOURCE_TYPE_MHL_1500MA ||
					chip->power_source_type == HTC_PWR_SOURCE_TYPE_MHL_2000MA);
}
#endif 

static int determine_initial_status(struct smbchg_chip *chip)
{

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	chg_term_handler(0, chip);
	usbid_change_handler(0, chip);
#ifndef CONFIG_HTC_BATT_8960
	src_detect_handler(0, chip);

	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present)
		handle_usb_insertion(chip);
	else
		handle_usb_removal(chip);
#endif

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#ifdef CONFIG_HTC_BATT_8960
#define VBUS_VALID_LOWER_BOUND		4150000
#define VBUS_ALLOW_CHANGE_MAX		2000000

int pmi8994_is_power_ok_check(void)
{
	u8 reg = 0;
	int prev_vbus_uv, vbus_uv, i, diff;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	for(i = 0; i < 3; i++) {
		vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
		if (i > 0) {
			diff = vbus_uv - prev_vbus_uv;
			if (ABS(diff) >= VBUS_ALLOW_CHANGE_MAX) {
				pr_info("%s: vbus is vibrating. prev_vbus_uv(%d) "
					"vbus_uv(%d)\n", __func__, prev_vbus_uv, vbus_uv);
				return 0;
			}
		}
		prev_vbus_uv = vbus_uv;
		msleep(50);
	}

	smbchg_read(the_chip, &reg, the_chip->misc_base + RT_STS, 1);

	pr_smb(PR_STATUS, "0x1610: 0x%02x, vbus=%d\n",
			reg, vbus_uv);
	if (((reg & POWER_OK_BIT) && (vbus_uv > VBUS_VALID_LOWER_BOUND)) ||
			(((reg & POWER_OK_BIT) == 0) && (vbus_uv <= VBUS_VALID_LOWER_BOUND)))
		return 0;
	else
		return 1;
}

#define CHGR_ERROR_BIT			BIT(0)

int pmi8994_is_batt_over_charged_check(void)
{
	u8 chgr_rt_sts, bat_if_rt_sts;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	if (svbat_mv > the_chip->vfloat_mv)
		return 0;

	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);

	if ((chgr_rt_sts & CHGR_ERROR_BIT) && (bat_if_rt_sts & BAT_OV_BIT)) {
		pr_smb(PR_STATUS, "batt_ov: 0x1010=0x%02x, 0x1210=0x%02x\n",
				chgr_rt_sts, bat_if_rt_sts);
		return 1;
	} else
		return 0;
}

int pmi8994_is_charger_error_handle(void)
{
	if (pmi8994_is_power_ok_check() == 1)
		return 1;
	else if (pmi8994_is_batt_over_charged_check() == 1)
		return 1;
	else
		return 0;
}

#define ICL_OVERRIDE_BIT	BIT(2)
#define USBIN_MODE_CHG_BIT  BIT(0)

int pmi8994_usbin_mode_charge(void)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	rc = smbchg_masked_write(the_chip, the_chip->usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't set 0x1340 ICL override bit rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_masked_write(the_chip, the_chip->usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't set 0x1340 usbin mode charge bit rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

int pmi8994_set_safety_timer_disable(int disable)
{
	int rc, i;
	u8 reg;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (is_safety_timer_disable != disable) {
		pr_info("%s(%d -> %d)\n", __func__, is_safety_timer_disable, disable);
		is_safety_timer_disable = disable;

		
		if (the_chip->safety_time != -EINVAL) {
			reg = (the_chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
				(the_chip->prechg_safety_time > 0
				? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

			
			if (flag_keep_charge_on || is_safety_timer_disable
					|| flag_disable_safety_timer)
				reg = SFT_TIMER_DISABLE_BIT;

			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (the_chip->safety_time <= chg_time[i]) {
					reg |= i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}
			for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
				if (the_chip->prechg_safety_time <= prechg_time[i]) {
					reg |= i;
					break;
				}
			}

			rc = smbchg_sec_masked_write(the_chip,
					the_chip->chgr_base + SFT_CFG,
					SFT_EN_MASK | SFT_TO_MASK |
					(the_chip->prechg_safety_time > 0
					? PRECHG_SFT_TO_MASK : 0), reg);
			if (rc < 0) {
				dev_err(the_chip->dev,
					"Couldn't set safety timer rc = %d\n",
					rc);
				return rc;
			}
		}
	}

	return 0;
}
#endif 

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_COMMAND_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_BAT_OV_ECC			BIT(4)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define CFG_TCC_REG			0xF9
#define CHG_ITERM_50MA			0x1
#define CHG_ITERM_100MA			0x2
#define CHG_ITERM_150MA			0x3
#define CHG_ITERM_200MA			0x4
#define CHG_ITERM_250MA			0x5
#define CHG_ITERM_300MA			0x0
#define CHG_ITERM_500MA			0x6
#define CHG_ITERM_600MA			0x7
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_45S		0
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
#define OTG_OC_CFG			0xF1
#define HICCUP_ENABLED_BIT		BIT(6)
#ifdef CONFIG_HTC_BATT_8960
#define TEMP_MONITOR_DISABLE_BIT	0
#define HOT_COLD_SL_COMP_BITS		0
#define HARD_TEMP_MASK			SMB_MASK(6, 5)
#define SOFT_TEMP_MASK			SMB_MASK(3, 0)
#define USBIN_CHGR_MASK			SMB_MASK(2, 0)
#endif
static void batt_ov_wa_check(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
				CHG_BAT_OV_ECC, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return;
	}

	rc = smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read Battery RT status rc = %d\n", rc);
		return;
	}

	if (reg & BAT_OV_BIT) {
		rc = smbchg_charging_en(chip, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't disable charging: rc = %d\n", rc);
			return;
		}

		
		msleep(200);

		rc = smbchg_charging_en(chip, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't enable charging: rc = %d\n", rc);
			return;
		}
	}
}

static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	rc = smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			AICL_WL_SEL_MASK, AICL_WL_SEL_45S);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_HTC_BATT_8960
	
	
	if (!chip->hvdcp3_supported && (chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
		
		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_RID_REG,
			HVDCP_AUTH_ALG_EN_BIT, HVDCP_AUTH_ALG_EN_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't enable hvdcp_alg rc=%d\n",
					rc);
			return rc;
		}

		rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_ADAPTER_SEL_9V_BIT);
		if (rc) {
			dev_err(chip->dev, "Couldn't set hvdcp config in chgpath_chg rc=%d\n",
						rc);
			return rc;
		}
		if (is_usb_present(chip)) {
			rc = smbchg_masked_write(chip,
				chip->usb_chgpth_base + CMD_APSD,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
			if (rc)
				pr_err("Unable to re-run APSD rc=%d\n", rc);
		}
	}
#endif 
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_COMMAND_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_COMMAND_BIT
			| (chip->chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}
	chip->battchg_disabled = 0;


	if (chip->chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
			TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	check_battery_type(chip);

#ifdef CONFIG_HTC_BATT_8960
	if (chip->usbin_chgr_cfg != -EINVAL) {
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + USBIN_CHGR_CFG,
				USBIN_CHGR_MASK, chip->usbin_chgr_cfg);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
			return rc;
		}
	}
#endif

	
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->vfloat_mv);
	}

	
	if (chip->fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
			chip->fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
			chip->fastchg_current_comp);
	}

	
	if (chip->float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
			chip->float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
			chip->float_voltage_comp);
	}

	
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->iterm_ma <= 50)
				reg = CHG_ITERM_50MA;
			else if (chip->iterm_ma <= 100)
				reg = CHG_ITERM_100MA;
			else if (chip->iterm_ma <= 150)
				reg = CHG_ITERM_150MA;
			else if (chip->iterm_ma <= 200)
				reg = CHG_ITERM_200MA;
			else if (chip->iterm_ma <= 250)
				reg = CHG_ITERM_250MA;
			else if (chip->iterm_ma <= 300)
				reg = CHG_ITERM_300MA;
			else if (chip->iterm_ma <= 500)
				reg = CHG_ITERM_500MA;
			else
				reg = CHG_ITERM_600MA;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CFG_TCC_REG,
					CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
			pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
					chip->iterm_ma, reg);
		}
	}

	
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

#ifdef CONFIG_HTC_BATT_8960
		
		if (flag_keep_charge_on | flag_disable_safety_timer)
			reg = SFT_TIMER_DISABLE_BIT;
#endif

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
		chip->safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
				rc);
		else if (!(reg & SFT_EN_MASK))
			chip->safety_timer_en = true;
	}

	
	if (chip->jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT,
			chip->jeita_temp_hard_limit
			? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set jeita temp hard limit rc = %d\n",
				rc);
			return rc;
		}
	}

	
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	smbchg_charging_status_change(chip);

	msleep(20);
	smbchg_usb_en(chip, chip->chg_enabled, REASON_USER);
	smbchg_dc_en(chip, chip->chg_enabled, REASON_USER);
	
	if (chip->resume_delta_mv != -EINVAL) {

		if (!chip->chg_inhibit_source_fg) {
			if (chip->resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT, (chip->resume_delta_mv < 200)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT_8960
	
	if (flag_keep_charge_on || flag_disable_temp_protection) {
		reg = TEMP_MONITOR_DISABLE_BIT |
			JEITA_TEMP_HARD_LIMIT_BIT | HOT_COLD_SL_COMP_BITS;
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CCMP_CFG,
			HARD_TEMP_MASK | SOFT_TEMP_MASK, reg);

		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable hw soft/hard temp monitor = %d\n",
				rc);
			return rc;
		}
	}
#endif

	
	if (chip->dc_psy_type != -EINVAL) {
		rc = smbchg_set_thermal_limited_dc_current_max(chip,
						chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev, "can't set dc current: %d\n", rc);
			return rc;
		}
	}


	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_set_fastchg_current(chip, chip->target_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set fastchg current = %d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
				rc);
			return rc;
		}
	}

	if (chip->force_aicl_rerun)
		rc = smbchg_aicl_config(chip);

	if (chip->schg_version == QPNP_SCHG_LITE) {
		
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_OC_CFG,
					HICCUP_ENABLED_BIT, HICCUP_ENABLED_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set OTG OC config rc = %d\n",
				rc);
	}

	if (chip->wa_flags & SMBCHG_BATT_OV_WA)
		batt_ov_wa_check(chip);

#ifdef CONFIG_HTC_BATT_8960
	
	if (chip->otg_output_current_ma != 0) {
		rc = smbchg_set_otg_output_current(chip, chip->otg_output_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set otg output limit current rc = %d\n",
				rc);
			return rc;
		}
	}
#endif

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
		.data           = (void *)ARRAY_SIZE(usb_current_table),
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

#ifdef CONFIG_HTC_BATT_8960
#define HTC_OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval) 						\
		break;							\
	if (optional)							\
		prop = -EINVAL; 					\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"htc," dt_property ,	\
					&prop); 			\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0; 					\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc); 	\
} while (0)
#endif


#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct smbchg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->wipower_dyn_icl_avail = false;

	if (!chip->vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_default,
					"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_pt,
					"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_div2,
					"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->wipower_default.num = 0;
	chip->wipower_pt.num = 0;
	chip->wipower_default.num = 0;
	if (chip->wipower_default.entries)
		devm_kfree(chip->dev, chip->wipower_default.entries);
	if (chip->wipower_pt.entries)
		devm_kfree(chip->dev, chip->wipower_pt.entries);
	if (chip->wipower_div2.entries)
		devm_kfree(chip->dev, chip->wipower_div2.entries);
	chip->wipower_default.entries = NULL;
	chip->wipower_pt.entries = NULL;
	chip->wipower_div2.entries = NULL;
	chip->vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#define DEFAULT_FCC_MA			2000
static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->target_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->target_fastchg_current_ma == -EINVAL)
		chip->target_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->vled_max_uv < 0)
		chip->vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->rpara_uohm < 0)
		chip->rpara_uohm = 0;
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
#ifdef CONFIG_HTC_BATT_8960
	OF_PROP_READ(chip, chip->usb_max_current_ma, "iusb-max-ma", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_temp, "kddi-limit1-temp", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_current, "kddi-limit1-current", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit1_current_qc20, "kddi-limit1-current-qc20", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_temp, "kddi-limit2-temp", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_current, "kddi-limit2-current", rc, 1);
	OF_PROP_READ(chip, chip->kddi_limit2_current_qc20, "kddi-limit2-current-qc20", rc, 1);
	OF_PROP_READ(chip, chip->qc20_usb_max_current_ma, "qc20-iusb-max-ma", rc, 1);
	if (chip->qc20_usb_max_current_ma == -EINVAL) {
		
		dev_err(chip->dev, "qc20-iusb-max-ma missing\n");
		chip->qc20_usb_max_current_ma = USB_MA_1000;
	}

	if (chip->usb_max_current_ma != -EINVAL) {
		pre_usb_max_current_ma = chip->usb_max_current_ma;
	}
#endif
	if (chip->safety_time != -EINVAL &&
		(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->safety_time);
		return -EINVAL;
	}
	if (chip->prechg_safety_time != -EINVAL &&
		(chip->prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
	chip->cfg_fastchg_current_ma = chip->target_fastchg_current_ma;
#ifndef CONFIG_HTC_BATT_8960
	if (chip->parallel.min_current_thr_ma != -EINVAL
			&& chip->parallel.min_9v_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
#else
	chip->parallel.avail = false;
#endif
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->parallel.min_current_thr_ma,
			chip->parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);

	
	chip->use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
						"qcom,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
					"qcom,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
						"qcom,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");
	chip->force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");

	
	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

#ifdef CONFIG_HTC_BATT_8960
	OF_PROP_READ(chip, chip->usbin_chgr_cfg, "usbin-chgr-cfg", rc, 0);
#endif

	
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	
	chip->cfg_chg_led_sw_ctrl =
		of_property_read_bool(node, "qcom,chg-led-sw-controls");
	chip->cfg_chg_led_support =
		of_property_read_bool(node, "qcom,chg-led-support");

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT_8960
	HTC_OF_PROP_READ(chip, chip->eoc_ibat_thre_ma,
			"eoc-ibat-thre-ma", rc, 0);
	HTC_OF_PROP_READ(chip, chip->cv_fastchg_cur_ma,
			"cv-fastchg-cur-ma", rc, 1);
	HTC_OF_PROP_READ(chip, chip->adjust_fastchg_cur_thr_mv,
			"adjust-fastchg-cur-thr-mv", rc, 1);
	HTC_OF_PROP_READ(chip, chip->otg_output_current_ma, "otg-output-current-ma", rc, 1);
	if (chip->otg_output_current_ma == -EINVAL) {
		
		dev_err(chip->dev, "otg-output-current-ma missing\n");
		chip->otg_output_current_ma = 0;
	}

	chip->wa_adjust_iusb_max_to_prevent_over_chg_from_1A_adaptor =
		of_property_read_bool(node, "htc,wa-adjust-iusb-max-to-prevent-over-chg-from-1A-adaptor");
#endif 

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler, flags, rc);
			if (chip->schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->otg_oc_irq);
				enable_irq_wake(chip->otg_fail_irq);
			}
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
			if (chip->parallel.avail && chip->usb_present) {
				rc = enable_irq_wake(chip->aicl_done_irq);
				chip->enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->safety_timeout_irq,
				"safety-timeout",
				safety_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->safety_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			
			enable_irq_wake(chip->otg_oc_irq);
			enable_irq_wake(chip->otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
		case SMBCHG_LITE_OTG_SUBTYPE:
			chip->otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
#ifndef CONFIG_HTC_BATT_8960
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
#else
	pr_smb(PR_STATUS, "%s - %04X = %02X\n", name, addr, reg);
#endif
}

static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

	
	for (addr = 0x4; addr <= 0xE; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xE0; addr <= 0xE7; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR");
	
	for (addr = 0x4; addr <= 0x5; addr++)
		dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xE0; addr <= 0xEF; addr++)
		dump_reg(chip, chip->otg_base + addr, "OTG");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->otg_base + addr, "OTG");
	
	for (addr = 0x4; addr <= 0x8; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0x40; addr <= 0x43; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xE0; addr <= 0xE1; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF");
	
	for (addr = 0x4; addr <= 0xF; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0x40; addr <= 0x43; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xE0; addr <= 0xE5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB");
	
	for (addr = 0x4; addr <= 0x5; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xE0; addr <= 0xE2; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC");
	
	for (addr = 0x0; addr <= 0x8; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0x10; addr <= 0x1B; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0x40; addr <= 0x42; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xD9; addr <= 0xDF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xE0; addr <= 0xEF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC");
}

static int create_debugfs_entries(struct smbchg_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
				  S_IFREG | S_IWUSR | S_IRUGO,
				  chip->debug_root, chip,
				  &force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
			"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}
	return 0;
}

static int smbchg_wa_config(struct smbchg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		pr_err("Unable to get pmic_revid rc=%ld\n",
				PTR_ERR(pmic_rev_id));
		return -EPROBE_DEFER;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA
				| SMBCHG_BATT_OV_WA;
	case PMI8950:
		chip->wa_flags |= SMBCHG_BATT_OV_WA;
		if (pmic_rev_id->rev4 < 2)  {
			chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		} else	{ 
			chip->wa_flags |= SMBCHG_HVDCP_9V_EN_WA
					| SMBCHG_USB100_WA;
		}
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}

#ifndef CONFIG_HTC_BATT_8960
	pr_debug("wa_flags=0x%x\n", chip->wa_flags);
#else
	pr_info("wa_flags=0x%x, pmic_rev_id:%d, pmic_rev_id->rev4:%d\n",
			chip->wa_flags, pmic_rev_id->pmic_subtype, pmic_rev_id->rev4);
#endif

	return 0;
}

static int smbchg_check_chg_version(struct smbchg_chip *chip)
{
	int rc;
	u8 val = 0;

	if (!chip->chgr_base) {
		pr_err("CHG base not specifed, unable to detect chg\n");
		return -EINVAL;
	}

	rc = smbchg_read(chip, &val, chip->chgr_base + SUBTYPE_REG, 1);
	if (rc) {
		pr_err("unable to read subtype reg rc=%d\n", rc);
		return rc;
	}

	switch (val) {
	case SMBCHG_CHGR_SUBTYPE:
		chip->schg_version = QPNP_SCHG;
		break;
	case SMBCHG_LITE_CHGR_SUBTYPE:
		chip->schg_version = QPNP_SCHG_LITE;
		break;
	default:
		pr_err("Invalid charger subtype=%x\n", val);
		break;
	}

	rc = smbchg_wa_config(chip);
	if (rc)
		pr_err("Charger WA flags not configured rc=%d\n", rc);

	return rc;
}

#ifdef CONFIG_HTC_BATT_8960
static void dump_all(int more)
{
	int ibat_ma, soc, batt_temp, vbus_uv;
	int health, present, charger_type, usb_online, dc_online;
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0;
	u8 misc_rt_sts, chgr_cfg2, bat_if_cmd, chgpth_cmd, chgpth_cfg;
	u8 chgpth_input_sts = 0, chgpth_aicl_cfg = 0;
	int ohm_val = 0, aicl_result;

	vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
	svbat_mv = smbchg_get_prop_voltage_now(the_chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(the_chip)/1000;
	soc = smbchg_get_prop_capacity_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	health = get_prop_batt_health(the_chip);
	present = get_prop_batt_present(the_chip);
	charger_type = get_prop_charge_type(the_chip);
	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &chgr_cfg2, the_chip->chgr_base + CHGR_CFG2, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_cmd, the_chip->bat_if_base + CMD_CHG, 1);
	smbchg_read(the_chip, &chgpth_rt_sts,
						the_chip->usb_chgpth_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_cmd,
						the_chip->usb_chgpth_base + CMD_IL, 1);
	smbchg_read(the_chip, &chgpth_aicl_cfg,
						the_chip->usb_chgpth_base + USB_AICL_CFG, 1);
	smbchg_read(the_chip, &chgpth_cfg,
						the_chip->usb_chgpth_base + CHGPTH_CFG, 1);
	smbchg_read(the_chip, &chgpth_input_sts,
						the_chip->usb_chgpth_base + INPUT_STS, 1);
	smbchg_read(the_chip, &misc_rt_sts, the_chip->misc_base + RT_STS, 1);
	usb_online = is_usb_present(the_chip);
	dc_online = is_dc_present(the_chip);

	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg,
								the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	pmi8994_fg_get_batt_id_ohm(&ohm_val);
	aicl_result = smbchg_get_aicl_level_ma(the_chip);

	printk(KERN_ERR "[BATT][SMB] V=%d mV,I=%d mA,T=%d C,id=%d ohm,SoC=%d,vbus=%d,"
			"H=%d,P=%d,CHG=%d,0x100E=%02x,0x1010=%02x,0x10FC=%02x,"
			"0x1210=%02x,0x1242=%02x,0x130D=%02x,0x1310=%02x,0x1340=%02x,0x13F3=%02x,"
			"0x13F4=%02x,0x1610=%02x,USB=%d,DC=%d,IUSB_MAX=%02x,AICL=%d,is_full=%d,"
			"is_limit_IUSB=%d,flag=%d%d%d%d%d,is_ac_ST=%d\n",
			svbat_mv, ibat_ma, batt_temp, ohm_val, soc, vbus_uv, health, present, charger_type,
			chgr_sts, chgr_rt_sts, chgr_cfg2, bat_if_rt_sts, bat_if_cmd,
			chgpth_input_sts, chgpth_rt_sts, chgpth_cmd, chgpth_aicl_cfg, chgpth_cfg, misc_rt_sts, usb_online,
			dc_online, iusb_reg, aicl_result, is_batt_full, is_limit_IUSB, flag_force_ac_chg,
			flag_pa_fake_batt_temp, flag_keep_charge_on, flag_disable_safety_timer,
			flag_disable_temp_protection, the_chip->is_ac_safety_timeout);

	if (smbchg_debug_mask & PR_DUMP)
		dump_regs(the_chip);
}

inline int pmi8994_dump_all(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	dump_all(0);

	return 0;
}

inline int pmi8952_dump_regs(void)
{
	int pr_status_bit = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_status_bit = smbchg_debug_mask & PR_STATUS;

	
	if (pr_status_bit == 0)
		smbchg_debug_mask |= PR_STATUS;

	dump_regs(the_chip);

	
	if (pr_status_bit == 0)
		smbchg_debug_mask &= ~PR_STATUS;

	return 0;
}

int pmi8994_is_batt_full(int *result)
{
    int soc = 0;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if(is_batt_full){
		soc = smbchg_get_prop_capacity_now(the_chip);
		if( soc <= CLEAR_FULL_STATE_BY_LEVEL_THR &&
		(!delayed_work_pending(&the_chip->eoc_work))){
			is_batt_full = false;
			pr_info("EOC not scheduled,clear full flag (is_batt_full = %d)\n", is_batt_full);
		}
	}

	*result = is_batt_full;
	return 0;
}

int pmi8994_is_batt_full_eoc_stop(int *result)
{
	int rc;
	u8 chgr_sts = 0, int_rt_sts = 0, chg_type = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full_eoc_stop;

	if (!delayed_work_pending(&the_chip->eoc_work)) {
		rc = smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
		if (rc) {
			dev_err(the_chip->dev, "failed to read chgr_sts (0x100E) rc=%d\n", rc);
			return -ENXIO;
		}
		rc = smbchg_read(the_chip, &int_rt_sts, the_chip->chgr_base + RT_STS, 1);
		if (rc) {
			dev_err(the_chip->dev, "failed to read int_rt_sts (0x1010) rc=%d\n", rc);
			return -ENXIO;
		}
		chg_type = (chgr_sts & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
		if ((int_rt_sts & P2F_CHG_THRESHOLD_BIT) && 
			((int_rt_sts & BAT_TCC_REACHED_BIT) == 0) && 
			(chg_type != 0)) { 
			wake_lock(&the_chip->eoc_worker_wlock);
			schedule_delayed_work(&the_chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
			pr_info("eoc_work rescheduled from batt_worker\n");
		}
	}
	return 0;
}

int pmi8994_charger_get_attr_text(char *buf, int size)
{
	int len = 0;
	u16 addr;
	u8 reg;
	int vbat_mv, ibat_ma, soc, batt_temp, vbus_uv;
	int health, present, charger_type, usb_online, dc_online;
	u8 chgr_sts = 0, chgr_rt_sts = 0, bat_if_rt_sts = 0;
	u8 chgpth_rt_sts = 0, iusb_reg = 0, aicl_reg = 0;

        if(!the_chip) {
                pr_err("called before init\n");
                return -EINVAL;
        }
	vbat_mv = smbchg_get_prop_voltage_now(the_chip)/1000;
	ibat_ma = smbchg_get_prop_current_now(the_chip)/1000;
	soc = smbchg_get_prop_capacity_now(the_chip);
	vbus_uv = pmi8994_get_usbin_voltage_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	health = get_prop_batt_health(the_chip);
	present = get_prop_batt_present(the_chip);
	charger_type = get_prop_charge_type(the_chip);
	usb_online = is_usb_present(the_chip);
	dc_online = is_dc_present(the_chip);

	smbchg_read(the_chip, &chgr_sts, the_chip->chgr_base + CHGR_STS, 1);
	smbchg_read(the_chip, &chgr_rt_sts, the_chip->chgr_base + RT_STS, 1);
	smbchg_read(the_chip, &bat_if_rt_sts, the_chip->bat_if_base + RT_STS, 1);
	smbchg_read(the_chip, &chgpth_rt_sts,the_chip->usb_chgpth_base + RT_STS, 1);

	smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);
	smbchg_read(the_chip, &aicl_reg, the_chip->usb_chgpth_base + ICL_STS_1_REG, 1);

	len += scnprintf(buf + len, size -len,
			"SOC(%%): %d;\n"
			"VBAT(mV): %d;\n"
			"IBAT(mA): %d;\n"
			"VBUS(uV): %d;\n"
			"BATT_TEMP: %d;\n"
			"HEALTH: %d;\n"
			"BATT_PRESENT(bool): %d;\n"
			"CHARGE_TYPE: %d;\n"
			"USB_ONLINE: %d;\n"
			"DC_ONLINE: %d;\n"
			"CHGR_STS: 0x%02x;\n"
			"CHGR_RT_STS: 0x%02x;\n"
			"IF_RT_STS: 0x%02x;\n"
			"USB_RT_STS: 0x%02x;\n"
			"USB_IL_CFG: 0x%02x;\n"
			"USB_ICL_STS_1_REG: 0x%02x;\n",
			soc, vbat_mv, ibat_ma, vbus_uv, batt_temp, health, present,
			charger_type, usb_online, dc_online,
			chgr_sts, chgr_rt_sts, bat_if_rt_sts, chgpth_rt_sts,
			iusb_reg, aicl_reg);

	
	for (addr = 0xB; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "CHGR_STS_%02x: 0x%02x\n", addr, reg);
	}
	for (addr = 0xF0; addr <= 0xFF; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "CHGR_CF_%02x: 0x%02x\n", addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_STS: 0x%02x\n", reg);
	smbchg_read(the_chip, &reg, the_chip->bat_if_base + CMD_CHG_REG, 1);
	len += scnprintf(buf + len, size -len, "BAT_IF_Command: 0x%02x\n", reg);

	for (addr = 0xF0; addr <= 0xFB; addr++){
		smbchg_read(the_chip, &reg, the_chip->chgr_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "BAT_IF_CF_%02x: 0x%02x\n", addr, reg);
	}
	
	for (addr = 0x7; addr <= 0x10; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "USB_STS_%02x: 0x%02x\n", addr, reg);
	}
	smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + CMD_IL, 1);
	len += scnprintf(buf + len, size -len, "USB_Command: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF5; addr++){
		smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "USB_CF_%02x: 0x%02x\n", addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "DC_Status: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF6; addr++){
		smbchg_read(the_chip, &reg, the_chip->dc_chgpth_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "DC_CF_%02x: 0x%02x\n", addr, reg);
	}
	
	smbchg_read(the_chip, &reg, the_chip->misc_base + IDEV_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_STS: 0x%02x\n", reg);
	smbchg_read(the_chip, &reg, the_chip->misc_base + RT_STS, 1);
	len += scnprintf(buf + len, size -len, "MISC_RT_STS: 0x%02x\n", reg);
	for (addr = 0xF0; addr <= 0xF3; addr++){
		smbchg_read(the_chip, &reg, the_chip->misc_base + addr, 1); 
		len += scnprintf(buf + len, size -len, "MISC_CF_%02x: 0x%02x\n", addr, reg);
	}
	return len;
}

int pmi8994_get_batt_voltage(int *result)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = smbchg_get_prop_voltage_now(the_chip)/1000;

	return 0;
}

int pmi8994_get_charge_type(void)
{
	if(!the_chip) {
		pr_err("called before init\n");
		return 0;
	}
	return get_prop_charge_type(the_chip);
}

int pmi8994_fake_src_detect_irq_handler(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	cable_detection_vbus_irq_handler();

	return 0;
}

int pmi8994_check_cable_status(void)
{
	int usb_present;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	usb_present = (int)is_usb_present(the_chip);
	return usb_present;
}

int pmi8994_is_charger_ovp(int* result)
{
	int ov = 0, uv = 0;
	int rc;
	u8 reg;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = smbchg_read(the_chip, &reg, the_chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(the_chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return 0;
	}

	if (reg & USBIN_UV_BIT) {
		uv = 1;
	} else if (reg & USBIN_OV_BIT) {
		ov = 1;
	}

	*result = ov;
	if (uv || ov)
		pr_smb(PR_STATUS, "ov=%d, uv=%d, reg=0X%2x\n", ov, uv, reg);
	return 0;
}

#define NAVI_LIMITED_CURRENT_UP                 700
#define NAVI_LIMITED_CURRENT_DOWN               1000
#define NAVI_LIMITED_CURRENT_UP_QC20            500
#define NAVI_LIMITED_CURRENT_DOWN_QC20          700

#define NAVI_UPPER_BOUND                        30
#define NAVI_LOWER_BOUND                        30
#define NAVI_LIMIT_TEMP                         390

#ifdef CONFIG_DUTY_CYCLE_LIMIT
int pmi8994_limit_charge_enable(int chg_limit_reason, int chg_limit_timer_sub_mask, int limit_charge_timer_ma)
{
	int ibat_ma;

	pr_info("chg_limit_reason=%d, chg_limit_timer_sub_mask=%d, limit_charge_timer_ma=%d\n",
			chg_limit_reason, chg_limit_timer_sub_mask, limit_charge_timer_ma);

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	if (limit_charge_timer_ma != 0 && !!(chg_limit_reason & chg_limit_timer_sub_mask))
		chg_limit_current = limit_charge_timer_ma;
	else {
		if (!!chg_limit_reason)
			chg_limit_current = PMI8994_CHG_I_MIN_MA;
		else
			chg_limit_current = 0;
	}

	pr_info("%s:chg_limit_current = %d\n", __func__, chg_limit_current);

	if (chg_limit_current == 0) {
		
		is_over_fastchg_thr_once = false;
		ibat_ma = the_chip->target_fastchg_current_ma;
	} else {
		ibat_ma = chg_limit_current;
	}

	smbchg_set_fastchg_current(the_chip, ibat_ma);

	return 0;
}
#else
int pmi8994_limit_charge_enable(bool enable, int reason, int restrict)
{
	int ibat_ma;

	pr_info("limit_charge=%d, reason=0x%x, restrict=%d\n", enable, reason, restrict);

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_TALK))
		chg_limit_current = PMI8994_CHG_I_MIN_MA;
	
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_NET_TALK))
		chg_limit_current = PMI8994_CHG_I_MIN_MA;
	
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_NONE))
		chg_limit_current = 0;
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_SOFT))
		chg_limit_current = PMI8994_CHG_I_MIN_MA_L1;
	else if (enable && (reason & HTC_BATT_CHG_LIMIT_BIT_THRML) && (restrict == RESTRICT_HEAVY))
		chg_limit_current = PMI8994_CHG_I_MIN_MA_L2;
	
	else
		chg_limit_current = 0;

	if (chg_limit_current == 0) {
		
		is_over_fastchg_thr_once = false;
		ibat_ma = the_chip->target_fastchg_current_ma;
	} else {
		ibat_ma = chg_limit_current;
	}

	smbchg_set_fastchg_current(the_chip, ibat_ma);

	return 0;
}
#endif

int pmi8994_get_cable_type_by_usb_detect(int *result)
{
	int rc;

	rc = usb_get_connect_type();
	pr_info("USB type: %d\n", rc);
	*result = rc;
	return 0;
}

int pmi8994_limit_input_current(bool enable, int reason)
{
	int batt_temp = 0;
	int limit1_temp = 0, limit1_current = 0;
	int limit2_temp = 0, limit2_current = 0;
	int limit_thermal_current = 0;
	int limit_navi_current = 0;
	int limit_current = 0;
	int orig_current_ma = 0;
	int soc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	soc = smbchg_get_prop_capacity_now(the_chip);
	batt_temp = smbchg_get_prop_batt_temp_now(the_chip);
	limit1_temp = the_chip->kddi_limit1_temp;
	limit2_temp = the_chip->kddi_limit2_temp;

	
	if (is_qc20_done_flag == true) {
		limit1_current = the_chip->kddi_limit1_current_qc20;
		limit2_current = the_chip->kddi_limit2_current_qc20;
	} else {
		limit1_current = the_chip->kddi_limit1_current;
		limit2_current = the_chip->kddi_limit2_current;
	}

	orig_current_ma = the_chip->usb_target_current_ma;

	if (enable) {
		
		if ((reason & HTC_BATT_CHG_LIMIT_BIT_NAVI) &&
				(soc > NAVI_UPPER_BOUND) &&
				(batt_temp > NAVI_LIMIT_TEMP)) {
			
			if (is_qc20_done_flag == true)
				limit_navi_current = NAVI_LIMITED_CURRENT_UP_QC20;
			else
				limit_navi_current = NAVI_LIMITED_CURRENT_UP;
		} else if ((reason & HTC_BATT_CHG_LIMIT_BIT_NAVI) &&
				(soc <= NAVI_LOWER_BOUND) &&
				(batt_temp > NAVI_LIMIT_TEMP)) {
			
			if (is_qc20_done_flag == true)
				limit_navi_current = NAVI_LIMITED_CURRENT_DOWN_QC20;
			else
				limit_navi_current = NAVI_LIMITED_CURRENT_DOWN;
		} else {
			
		}

		
		if ((reason & HTC_BATT_CHG_LIMIT_BIT_KDDI) &&
			(limit1_temp > 0) && (limit2_temp > 0) &&
			(limit1_temp < limit2_temp) &&
			(limit1_current > 0) && (limit2_current > 0))
		{
			if ((batt_temp > limit1_temp) && (batt_temp <= limit2_temp)) {
				
				limit_thermal_current = limit1_current;
			} else if (batt_temp > limit2_temp) {
				
				limit_thermal_current = limit2_current;
			} else if (batt_temp <= limit1_temp) {
				
				limit_thermal_current = orig_current_ma;
			}
			else {
				
			}
		}

		pr_info("soc:%d, batt_temp:%d, L1_temp:%d, L2_temp:%d, "
				"limit_navi_current:%d, limit_thermal_current:%d, "
				"orig_current_ma:%d\n", soc, batt_temp, limit1_temp,
				limit2_temp, limit_navi_current, limit_thermal_current,
				orig_current_ma);
		
		if (!flag_keep_charge_on && !flag_disable_temp_protection) {
			mutex_lock(&the_chip->current_change_lock);
			
			if (limit_navi_current != 0 && limit_thermal_current != 0) {
				limit_current = MIN(MIN(limit_navi_current, limit_thermal_current), orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else if (limit_navi_current != 0) {
				limit_current = MIN(limit_navi_current, orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else if (limit_thermal_current != 0) {
				limit_current = MIN(limit_thermal_current, orig_current_ma);
				smbchg_set_thermal_limited_usb_current_max(
						the_chip, limit_current);
			} else {
				
			}
			mutex_unlock(&the_chip->current_change_lock);
		}
	}
	else { 
		mutex_lock(&the_chip->current_change_lock);
		smbchg_set_thermal_limited_usb_current_max(
				the_chip, orig_current_ma);
		mutex_unlock(&the_chip->current_change_lock);
	}

	return 0;
}

int pmi8952_set_otg_pulse_skip(int val)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return smbchg_otg_pulse_skip_disable(the_chip,
			REASON_FLASH_ENABLED, val);
}

int pmi8952_get_otg_pulse_skip(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return the_chip->otg_pulse_skip_dis;
}

static const int otg_icfg_table[] = {
	250,
	600,
	750,
	1000,
};

#define OTG_ICFG	0xF3
#define OTG_ICFG_MASK	SMB_MASK(1, 0)
static int smbchg_set_otg_output_current(struct smbchg_chip *chip, int otg_ma)
{
	int rc;
	u8 i;

	for (i = 0; i < ARRAY_SIZE(otg_icfg_table); i++)
		if (otg_ma == otg_icfg_table[i])
			break;

	if (i >= ARRAY_SIZE(otg_icfg_table)) {
		dev_err(chip->dev, "The OTG current (%d mA) is larger than the max value (%d mA) rc = %d\n",
				otg_ma, otg_icfg_table[ARRAY_SIZE(otg_icfg_table) - 1], rc);
		return -EINVAL;
	}

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_ICFG,
			OTG_ICFG_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set OTG output limit current rc = %d\n",
				rc);
	else
		pr_smb(PR_STATUS, "set OTG output limit current to %d mA\n",
				otg_icfg_table[i]);

	return rc;

}


int pmi8952_is_chg_safety_timer_timeout(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = the_chip->is_ac_safety_timeout;
	return 0;
}

int filter_usb_max_chg_currnt(int current_ma)
{
	
	if (current_ma == USB_MA_1000)
		current_ma = USB_MA_950;
	else if (current_ma == USB_MA_1500)
		current_ma = USB_MA_1450;

	return current_ma;
}

int pmi8952_get_vbus(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = pmi8994_get_usbin_voltage_now(the_chip);
	return 0;
}

int pmi8952_get_max_iusb(int *result)
{
	u8 iusb_reg = 0;
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = smbchg_read(the_chip, &iusb_reg, the_chip->usb_chgpth_base + IL_CFG, 1);

	if (rc) {
		dev_err(the_chip->dev, "cannot write to config c rc = %d\n", rc);
		return -EINVAL;
	}

	iusb_reg &= ICL_STS_MASK;
	if (iusb_reg >= ARRAY_SIZE(usb_current_table)) {
		pr_warn("invalid AICL value: %02x\n", iusb_reg);
		return -EINVAL;
	}

	*result = usb_current_table[iusb_reg];
	return 0;
}

int pmi8952_get_AICL(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	*result = smbchg_get_aicl_level_ma(the_chip);
	return 0;
}
#endif 

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct power_supply *usb_psy;
	struct qpnp_vadc_chip *vadc_dev;

#ifdef CONFIG_HTC_BATT_8960
	smbchg_debug_mask = 0x6;
	if (flag_enable_bms_charger_log)
		smbchg_debug_mask = 0x7F;
#endif

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

#ifndef CONFIG_HTC_BATT_8960
	if (of_find_property(spmi->dev.of_node, "qcom,dcin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "dcin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}
#endif

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

#ifndef CONFIG_HTC_BATT_8960
	INIT_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
#else
	INIT_DELAYED_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
#endif
	INIT_DELAYED_WORK(&chip->parallel_en_work,
			smbchg_parallel_usb_en_work);
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
	init_completion(&chip->src_det_lowered);
	init_completion(&chip->src_det_raised);
	init_completion(&chip->usbin_uv_lowered);
	init_completion(&chip->usbin_uv_raised);
	chip->vadc_dev = vadc_dev;
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->fcc_lock);
	mutex_init(&chip->current_change_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->battchg_disabled_lock);
	mutex_init(&chip->usb_en_lock);
	mutex_init(&chip->dc_en_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->taper_irq_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->wipower_config);
	mutex_init(&chip->usb_status_lock);
	device_init_wakeup(chip->dev, true);

#ifdef CONFIG_HTC_BATT_8960
	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy)
		smbchg_load_jeita_setting(chip);

	
	wake_lock_init(&chip->vbus_wlock,
			WAKE_LOCK_SUSPEND, "vbus_wlock");
	spin_lock_init(&chip->vbus_lock);
	wake_lock_init(&chip->eoc_worker_wlock, WAKE_LOCK_SUSPEND,
			"pmi8994_eoc_worker_lock");

	INIT_WORK(&chip->notify_fg_work, smbchg_notify_fg_work);
	INIT_WORK(&chip->batt_soc_work, smbchg_adjust_batt_soc_work);
	INIT_DELAYED_WORK(&chip->vbus_detect_work, check_cable_type);
	INIT_DELAYED_WORK(&chip->eoc_work, smbchg_eoc_work);
	INIT_DELAYED_WORK(&chip->usb_type_polling_work, usb_type_polling);
	INIT_DELAYED_WORK(&chip->retry_aicl_work, retry_aicl_worker);
	INIT_DELAYED_WORK(&chip->hvdcp_5to9V_work, hvdcp_5to9V_worker);
	chip->cable_detect_wq = create_singlethread_workqueue("cable_detect");
	INIT_WORK(&chip->usb_aicl_limit_current, smbchg_usb_limit_current_WA_work);
	INIT_DELAYED_WORK(&chip->usb_limit_max_current, smbchg_usb_limit_max_current_work);
#endif

	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}

	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto free_regulator;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto free_regulator;
	}

#ifdef 	CONFIG_HTC_BATT_8960
	schedule_delayed_work(&chip->hvdcp_5to9V_work,
		msecs_to_jiffies(HVDCP_CHANGE_PERIOD_MS));
#endif

	chip->previous_soc = -EINVAL;
#ifndef CONFIG_HTC_BATT_8960
	chip->batt_psy.name		= chip->battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto free_regulator;
	}
	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;
#else
	chip->psy_registered = false;

	the_chip = chip;
#endif

	if (chip->cfg_chg_led_support &&
			chip->schg_version == QPNP_SCHG_LITE) {
		rc = smbchg_register_chg_led(chip);
		if (rc) {
			dev_err(chip->dev,
					"Unable to register charger led: %d\n",
					rc);
			goto unregister_dc_psy;
		}

		rc = smbchg_chg_led_controls(chip);
		if (rc) {
			dev_err(chip->dev,
					"Failed to set charger led controld bit: %d\n",
					rc);
			goto unregister_led_class;
		}
	}

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_led_class;
	}

	pr_smb(PR_MISC, "setting usb psy present = %d\n", chip->usb_present);
	power_supply_set_present(chip->usb_psy, chip->usb_present);
#ifdef CONFIG_HTC_BATT_8960
	htc_charger_event_notify(HTC_CHARGER_EVENT_READY);
#endif

	dump_regs(chip);
	create_debugfs_entries(chip);
	dev_info(chip->dev,
		"SMBCHG successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d\n",
			version_str[chip->schg_version],
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);
	return 0;

unregister_led_class:
	if (chip->cfg_chg_led_support && chip->schg_version == QPNP_SCHG_LITE)
		led_classdev_unregister(&chip->led_cdev);
unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
#ifndef CONFIG_HTC_BATT_8960
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
#endif
free_regulator:
	smbchg_regulator_deinit(chip);
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(chip->debug_root);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);

	power_supply_unregister(&chip->batt_psy);
	smbchg_regulator_deinit(chip);

#ifdef CONFIG_HTC_BATT_8960
	
	cancel_delayed_work_sync(&chip->eoc_work);
#endif
	return 0;
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
};

static int __init smbchg_init(void)
{
#ifdef CONFIG_HTC_BATT_8960
	flag_keep_charge_on =
		(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ? 1 : 0;
	flag_force_ac_chg =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE) ? 1 : 0;
	flag_pa_fake_batt_temp =
		(get_kernel_flag() & KERNEL_FLAG_FOR_PA_TEST) ? 1 : 0;
	flag_disable_safety_timer =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER) ? 1 : 0;
	flag_disable_temp_protection =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_TBATT_PROTECT) ? 1 : 0;
	flag_enable_bms_charger_log =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? 1 : 0;
#endif

	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
