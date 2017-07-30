/*
 * Copyright (C) 2011 HTC, Inc.
 * Author: Dyson Lee <Dyson@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

static unsigned int usb_project_pid;/*++ 2015/06/30 USB Team, PCN00051 ++*/
/*++ 2015/6/4, USB Team, PCN00022 ++*/
static int manual_serialno_flag = 0;
static char mfg_whiteline_serialno[] = "000000000000";

void android_set_serialno(char *serialno)
{
	strings_dev[STRING_SERIAL_IDX].s = serialno;
}

void init_mfg_serialno(void)
{
	if(!strcmp(htc_get_bootmode(),"factory2") || !strcmp(htc_get_bootmode(),"ftm"))
		android_set_serialno(mfg_whiteline_serialno);
	return;
}

int mfg_check_white_accessory(int accessory_type)
{
	static int previous_type = 0;
	struct android_dev *dev;
	struct usb_composite_dev *cdev;
	static int serialNumber_reset_flag = 0;

	if (manual_serialno_flag || (strcmp(htc_get_bootmode(),"factory2") && strcmp(htc_get_bootmode(),"ftm")))
		return 0;

	if (!_android_dev)
		return -EAGAIN;

	dev = _android_dev;
	cdev = dev->cdev;

	USB_INFO("%s : accessory type %d , previous type %d, connected %d\n", __func__, accessory_type, previous_type, dev->connected);
	switch (accessory_type)
	{
		case DOCK_STATE_CAR:
			mfg_whiteline_serialno[11] = '3';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				serialNumber_reset_flag = 1;
			}
			break;
		case DOCK_STATE_USB_HEADSET:
			mfg_whiteline_serialno[11] = '4';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				serialNumber_reset_flag = 1;
			}
			break;
		case DOCK_STATE_DMB:
			mfg_whiteline_serialno[11] = '5';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != accessory_type) {
				previous_type = accessory_type;
				serialNumber_reset_flag = 1;
			}
			break;
		default:
			mfg_whiteline_serialno[11] = '0';
			android_set_serialno(mfg_whiteline_serialno);
			if (previous_type != 0) {
				previous_type = 0;
				serialNumber_reset_flag = 1;
			}
			break;
	}
	if (serialNumber_reset_flag) {
		if (dev->connected) {
			schedule_delayed_work(&cdev->request_reset,REQUEST_RESET_DELAYED);
		}
		serialNumber_reset_flag = 0;
	}
	return 0;
}
/*-- 2015/6/4, USB Team, PCN00022 --*/

/*++ 2015/6/1, USB Team, PCN00015 ++*/
int htc_usb_enable_function(char *name, int ebl)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	char state_buf[60];
	char name_buf[60];
	char *function[3];
	if (!strcmp(name, "ffs"))
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "adb");
	else
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", name);
	function[0] = state_buf;
	function[2] = NULL;

	if (ebl) {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_on");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_on.dev->kobj, KOBJ_CHANGE,
				function);
	} else {
		snprintf(name_buf, sizeof(name_buf),
				"SWITCH_NAME=%s", "function_switch_off");
		function[1] = name_buf;
		kobject_uevent_env(&cdev->sw_function_switch_off.dev->kobj, KOBJ_CHANGE,
				function);
	}
	return 0;

}
/*-- 2015/6/1, USB Team, PCN00015 --*/

/*++ 2016/3/9, USB Team, PCN00020 ++*/
bool nonCableNotify_flag = false;
int htc_usb_nonCable_notification(int ebl)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	char state_buf[60];
	char name_buf[60];
	char *function[3];

	snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", "usb_nonstandard_cable");
	function[1] = name_buf;
	function[2] = NULL;

	if ( ebl == 1 && !nonCableNotify_flag ) {
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "1");
		function[0] = state_buf;
		kobject_uevent_env(&cdev->usb_nonstandard_cable.dev->kobj, KOBJ_CHANGE, function);
		nonCableNotify_flag = true;
		USB_INFO("[%s] Send uevent to notify framework that non-standard cable is %s\n", __func__, ebl==1 ? "plugged":"unplugged");
		return 0;
	} else if ( ebl == 0 && nonCableNotify_flag ) {
		snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "0");
		function[0] = state_buf;
		kobject_uevent_env(&cdev->usb_nonstandard_cable.dev->kobj, KOBJ_CHANGE, function);
		nonCableNotify_flag = false;
		USB_INFO("[%s] Send uevent to notify framework that non-standard cable is %s\n", __func__, ebl==1 ? "plugged":"unplugged");
		return 0;
	} else {
		USB_INFO("[%s] Skip sending uevent, ebl:%d, nonCableNotify_flag:%d\n", __func__, ebl, nonCableNotify_flag);
		return -1;
	}
}
/*-- 2016/3/9, USB Team, PCN00020 --*/

static int usb_disable;/*++ 2015/6/3 USB Team, PCN00021 ++*/

/*++ 2015/09/09 USB Team, PCN00082 ++*/
const char * change_charging_to_ums(const char *buff) {
	USB_INFO("switch ums function from %s\n", buff);
	if (!strcmp(buff, "charging"))
		return "mass_storage";
	else if (!strcmp(buff, "adb"))
		return "mass_storage,adb";
	else if (!strcmp(buff, "ffs"))
		return "mass_storage,ffs";
	return buff;
}
void change_charging_pid_to_ums(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
		case 0x0f0b:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0ff9;
			break;
		case 0x0c81:
			cdev->desc.idVendor = 0x0bb4;
			cdev->desc.idProduct = 0x0f86;
			break;
		default:
			break;
	}
	return ;
}
/*-- 2015/09/09 USB Team, PCN00082 --*/

/*++ 2015/6/2 USB Team, PCN00019 ++*/
const char * add_usb_radio_debug_function(const char *buff) {
	USB_INFO("switch to radio debug function from %s\n", buff);
	if (!strcmp(buff, "ffs,acm")) /* 0bb4/0f17 ftm mode*/
		return "adb,diag,modem,acm";
/*++ 2015/7/21, USB team, PCN00059 ++*/
	else if (!strcmp(buff, "mass_storage,adb")) /* 0bb4/0fd8 */
		return "mass_storage,adb,diag,modem,rmnet";
	else if (!strcmp(buff, "mass_storage,adb,acm")) /* 0bb4/0fd8 */
		return "mass_storage,adb,diag,modem,rmnet";
	else if (!strcmp(buff, "mass_storage")) /* 0bb4/0fd9 */
		return "mass_storage,diag,modem,rmnet";
	else if (!strcmp(buff, "mass_storage,acm")) /* 0bb4/0fd9 */
		return "mass_storage,diag,modem,rmnet";
/*-- 2015/7/21, USB team, PCN00059 --*/
/*++ 2015/8/26, USB team, PCN00078 ++*/
	else if (!strcmp(buff, "rndis"))/*0bb4 0f82*/
		return "rndis,diag,modem";
	else if (!strcmp(buff, "rndis,adb"))/*0bb4 0f83*/
		return "rndis,adb,diag,modem";
/*-- 2015/8/26, USB team, PCN00078 --*/
/*++ 2015/12/03, USB team, PCN00096 ++*/
	else if (!strcmp(buff, "mtp")) /* 0bb4/0f12 */
		return "mtp,diag,modem,rmnet";
	else if (!strcmp(buff, "mtp,adb")) /* 0bb4/0f11 */
		return "mtp,adb,diag,modem,rmnet";
/*-- 2015/12/03, USB team, PCN00096 --*/
	return buff;
}
/* Change the PID for radio flag 8 20000 */
void check_usb_vid_pid(struct usb_composite_dev *cdev) {
	switch(cdev->desc.idProduct) {
/*++ 2015/12/03, USB team, PCN00096 ++*/
		case 0x0c93:
			cdev->desc.idProduct = 0x0f12;
			break;
		case 0x0f87:
		case 0x0ca8:
			cdev->desc.idProduct = 0x0f11;
			break;
/*-- 2015/12/03, USB team, PCN00096 --*/
/*++ 2015/8/26, USB team, PCN00078 ++*/
		case 0x0ffe:
			cdev->desc.idProduct = 0x0f82;
			break;
		case 0x0ffc:
			cdev->desc.idProduct = 0x0f83;
			break;
/*-- 2015/8/26, USB team, PCN00078 --*/
/*++ 2015/7/21, USB team, PCN00059 ++*/
		case 0x0f86:
		case 0x0ff5:
			cdev->desc.idProduct = 0x0fd8;
			break;
		case 0x0f65:
		case 0x0ff9:
			cdev->desc.idProduct = 0x0fd9;
			break;
/*-- 2015/7/21, USB team, PCN00059 --*/
		case 0x0f15:
			cdev->desc.idProduct = 0x0f17;
		default:
			break;
	}
	cdev->desc.idVendor = 0x0bb4;
	return;
}

/*++ 2015/06/25, USB Team, PCN00045 ++*/
static void setup_usb_denied(int htc_mode)
{
	USB_INFO("%s: htc_mode = %d\n", __func__, htc_mode);
	if (htc_mode)
		_android_dev->autobot_mode = 1;
	else
		_android_dev->autobot_mode = 0;
}

static int usb_autobot_mode(void)
{
	if (_android_dev->autobot_mode)
		return 1;
	else
		return 0;
}

void android_switch_htc_mode(void)
{
	htc_usb_enable_function("adb,mass_storage,serial,projector", 1);
}

/* Check if USB function is available for user process */
static ssize_t show_is_usb_denied(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	int deny = 0;

	if (usb_autobot_mode()) {
		/* In HTC mode, USB function change by
		 * user space should be denied.
		 */
		deny = 1;
	}

	length = snprintf(buf, PAGE_SIZE, "%d\n", deny);
	return length;
}
/*-- 2015/06/25, USB Team, PCN00045 --*/

/*++ 2015/06/30 USB Team, PCN00051 ++*/
/* Change to project default PID */
void check_usb_project_pid(struct usb_composite_dev *cdev) {
    if (cdev->desc.idProduct == 0x0f90 && usb_project_pid != 0x0000) {
        cdev->desc.idVendor = 0x0bb4;
        cdev->desc.idProduct = usb_project_pid;
    }
    return;
}

static int __init get_usb_project_pid(char *str)
{
    int ret = kstrtouint(str, 0, &usb_project_pid);
    USB_INFO("androidusb.pid %d: %08x from %26s\r\n",
            ret, usb_project_pid, str);
    return ret;
} early_param("androidusb.pid", get_usb_project_pid);
/*-- 2015/06/30 USB Team, PCN00051 --*/

/*-- 2015/6/2 USB Team, PCN00019 --*/
static ssize_t iSerial_show(struct device *dev, struct device_attribute *attr,
	char *buf);
/*++ 2015/6/4, USB Team, PCN00023 ++*/
static ssize_t store_dummy_usb_serial_number(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct android_dev *android_dev = _android_dev;
	struct usb_composite_dev *cdev = android_dev->cdev;
	int loop_i;

	if (size >= sizeof(serial_string)) {
		USB_INFO("%s(): input size > %zu\n",
			__func__, sizeof(serial_string));
		return -EINVAL;
	}

	for (loop_i = 0; loop_i < size; loop_i++) {
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) /* 0-9 */
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) /* A-Z */
			continue;
		if (buf[loop_i] == 0x0A) /* Line Feed */
			continue;
		else {
			USB_INFO("%s(): get invaild char (0x%2.2X)\n",
					__func__, buf[loop_i]);
			return -EINVAL;
		}
	}
	strlcpy(serial_string, buf, sizeof(serial_string));
	strim(serial_string);
	android_set_serialno(serial_string);
/*++ 2015/6/26, USB Team, PCN00046 ++*/
	if (android_dev->connected) {
		/*avoid timimg issue that the flag not set*/
		manual_serialno_flag = 1;
		schedule_delayed_work(&cdev->request_reset,REQUEST_RESET_DELAYED);
	} else {
		USB_INFO("%s(); the android_dev not connected, please try againg\n",__func__);
		return -EINVAL;
	}
/*-- 2015/6/26, USB Team, PCN00046 --*/
	return size;
}

/*++ 2015/6/29, USB Team, PCN00050 ++*/
static const char *os_to_string(int os_type)
{
	switch (os_type) {
		case OS_NOT_YET:    return "OS_NOT_YET";
		case OS_MAC:        return "OS_MAC";
		case OS_LINUX:      return "OS_LINUX";
		case OS_WINDOWS:    return "OS_WINDOWS";
		default:        return "UNKNOWN";
	}
}
/*-- 2015/6/29, USB Team, PCN00050 --*/

/*-- 2015/6/4, USB Team, PCN00023 --*/
/* show current os type for mac or non-mac */
static ssize_t show_os_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", os_type);
	USB_INFO("%s: %s, %s", __func__, os_to_string(os_type), buf);
	return length;
}
/*++ 2015/06/29 USB Team, PCN00048 ++*/
static ssize_t store_ats(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &usb_ats);
	return count;
}

static ssize_t show_ats(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", (get_debug_flag() & 0x300) || usb_ats);
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}
/*-- 2015/06/29 USB Team, PCN00048 --*/

/*++ 2015/6/3 USB Team, PCN00021 ++*/
static ssize_t show_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", usb_disable);
	return length;
}

void msm_otg_set_disable_usb(int disable_usb);
static ssize_t store_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int disable_usb_function;

	sscanf(buf, "%d ", &disable_usb_function);
	USB_INFO("USB_disable set %d\n", disable_usb_function);
	usb_disable = disable_usb_function;
	msm_otg_set_disable_usb(disable_usb_function);
	return count;
}
/*-- 2015/6/3 USB Team, PCN00021 --*/

/*++ 2015/6/2, USB Team, PCN00016 ++*/
static ssize_t show_usb_ac_cable_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d",usb_get_connect_type());
	return length;
}

/*++ 2015/7/31, USB Team, PCN00062 ++*/
static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_dev *and_dev = _android_dev;

	length = sprintf(buf, "%d",(and_dev->connected == 1) && !usb_disable ? 1 : 0);
	return length;
}
/*-- 2015/7/31, usb team, pcn00062 --*/

/*++ 2015/08/21, USB Team, PCN00076 ++*/
static ssize_t store_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	if (value == 1 || value == 0) {
		usb_disable = !value ? 1 : 0;
		printk(KERN_WARNING "[USB][%s] set usb_disable = %d, usb_cable_connect = %d\n", __func__, usb_disable, value);
	} else {
		printk(KERN_WARNING "[USB][%s] INVALID value = %d\n", __func__, value);
	}

	return count;
}
/*-- 2015/08/21, usb team, PCN00076 --*/

/*++ 2015/08/04 USB Team, PCN00064 ++*/
static ssize_t store_usb_modem_enable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int usb_modem_enable;

	sscanf(buf, "%d ", &usb_modem_enable);
	USB_INFO("modem: enable %d\n", usb_modem_enable);
	htc_usb_enable_function("modem", usb_modem_enable?1:0);
	return count;
}

/*++ 2016/01/27, USB Team, PCN00015 ++*/
#ifdef CONFIG_USB_FAIRCHILD_FUSB302
extern ssize_t dump_all_register(char *buf);
static ssize_t show_typec_dump_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	USB_INFO("%s : dump ANX7418 register\n", __func__);
	return dump_all_register(buf);
}
#endif
/*-- 2016/01/27, USB Team, PCN00015 --*/

/*-- 2015/08/04, USB Team, PCN00064 --*/

static DEVICE_ATTR(usb_ac_cable_status, 0444, show_usb_ac_cable_status, NULL);
/*-- 2015/6/2, USB Team, PCN00016 --*/

static DEVICE_ATTR(dummy_usb_serial_number, 0644, iSerial_show, store_dummy_usb_serial_number);/*++ 2015/6/4, USB Team, PCN00023 ++*/
static DEVICE_ATTR(os_type, 0444, show_os_type, NULL);
static DEVICE_ATTR(ats, 0664, show_ats, store_ats);/*++ 2015/06/29 USB Team, PCN00048 ++*/
static DEVICE_ATTR(usb_disable, 0664,show_usb_disable_setting, store_usb_disable_setting);/*++ 2015/6/3 USB Team, PCN00021 ++*/
static DEVICE_ATTR(usb_denied, 0444, show_is_usb_denied, NULL);/*++ 2015/6/25 USB Team, PCN00045 ++*/
static DEVICE_ATTR(usb_cable_connect, 0664, show_usb_cable_connect, store_usb_cable_connect);/*++ 2015/7/31, USB Team, PCN00062 ++*/
static DEVICE_ATTR(usb_modem_enable, 0660,NULL, store_usb_modem_enable_setting);/*++ 2015/08/04, USB Team, PCN00064 ++*/
#ifdef CONFIG_USB_FAIRCHILD_FUSB302
static DEVICE_ATTR(typec_dump_reg, 0440, show_typec_dump_reg, NULL); /*++ 2016/01/27, USB Team, PCN00015 ++*/
#endif

static __maybe_unused struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_dummy_usb_serial_number.attr,
	&dev_attr_os_type.attr,
	&dev_attr_ats.attr,/*++ 2015/06/29 USB Team, PCN00048 ++*/
	&dev_attr_usb_ac_cable_status.attr,/*++ 2015/6/2, USB Team, PCN00016 ++*/
	&dev_attr_usb_disable.attr,/*++ 2015/6/3 USB Team, PCN00021 ++*/
	&dev_attr_usb_denied.attr,/*++ 2015/6/25 USB Team, PCN00045 ++*/
	&dev_attr_usb_cable_connect.attr,/*++ 2015/7/31, USB Team, PCN00062 ++*/
	&dev_attr_usb_modem_enable.attr,/*++ 2015/08/04, USB Team, PCN00064 ++*/
#ifdef CONFIG_USB_FAIRCHILD_FUSB302
	&dev_attr_typec_dump_reg.attr, /*++ 2016/01/27, USB Team, PCN00015 ++*/
#endif
	NULL
};

static  __maybe_unused const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

static void setup_vendor_info(struct android_dev *dev)
{
	if (sysfs_create_group(&dev->pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);
	/* Link android_usb to /sys/devices/platform */
	if (sysfs_create_link(&platform_bus.kobj, &dev->pdev->dev.kobj, "android_usb"))
		pr_err("%s: fail to link android_usb to /sys/devices/platform/\n", __func__);
}
