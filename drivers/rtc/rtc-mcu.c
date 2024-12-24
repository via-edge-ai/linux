/* rtc-mcu.c
 *
 * Driver for MCU rtc for VIA platform
 *
 * Author : Bass Huang <basshuang@via.com.tw>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/fcntl.h>
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>
#include <asm/io.h>
#include <asm/fcntl.h>
#include <linux/acpi.h>
#include <linux/serial_8250.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>

#define MCU_UART "/dev/ttyS1"

unsigned int (*io_serial_in)(struct uart_port *p, int offset);
struct uart_8250_port *serial8250_get_port(int line);
struct mcu_dev {
	/* rtc device descriptor */
	struct rtc_device *rtc;

	/* port (/dev/ttyS1 eg.) & speed (115200 eg.)*/
	const char* tty_port;
	u32	baudrate;

	/* locks */
	struct mutex mutex;

	/* tty port info */
	struct file* tty_file;

	struct tty_struct *tty ;
};

static const struct of_device_id mcu_rtc_match_table[] = {
	{	.compatible = "mcu,rtc",
	},
	{}
};


static int file_read(struct file *file, unsigned long long *offset, unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret;

	oldfs = force_uaccess_begin();

	ret = kernel_read(file, data, size, offset);
	
	force_uaccess_end(oldfs);

	return ret;
}

static int mcu_tty_read(struct file *f, char *buf, int size, int timeout)
{
	//unsigned char ch;
	int rc = 0, rx = 0;
	loff_t pos = 0;
	int retries = 0;

	if (IS_ERR(f)) {
		return -EINVAL;
	}

	while (1) {
		//rx = kernel_read(f, buf, size, &pos);
		//pos = 0;
		rx = file_read(f, &pos, buf, size);
		if (rx > 1) {
			rc += rx;
			//printk(KERN_CRIT "rx: %d, buf:%s, total_len:%d, buf[%d] = %x\n", rx, buf, rc, rx, buf[rx-1]);
			vfs_fsync(f, 0);
			if (buf[rx-1] == 0xa)
				break;
			else {
				buf += rx;
				if (rc > size)
					return -EINVAL;
			}
		}

		if (++retries > timeout) {
			return -ETIMEDOUT;
		}
		usleep_range(800, 1200);
	}

	return rc;
}

static int mcu_tty_write(struct file *f, unsigned char *buf, int count)
{
	loff_t pos = 0;
	return kernel_write(f, buf, count, &pos);
}

static int mcu_send_cmd(struct file* tty_file, char *cmd, char *buf, int s_size, int r_size) {
	int rc/*, timeout, i*/;

	/* it's blocked from now on */
	//mcu_tty_block(tty_file);

	if ((rc = mcu_tty_write(tty_file, cmd, s_size)) < 0) {
		//pr_info("write MCU error = %d\n", rc);
		return -EINVAL;
	}

	if ((rc = mcu_tty_read(tty_file, buf, r_size, 1000)) < 0) {
		//pr_info("read MCU error = %d\n", rc);
		return rc;
	}

	return 0;
}

static char covert_int_to_char(int value)
{
	return 0x30 + value;
}

int char_to_date(struct rtc_time *dt, char *src_buf, int mode)
{
	char YY[3], MM[3], DD[3], DoW[3], hh[3], mm[3], ss[3];
	int pass = 1, copy = 0, i;

	if (strlen(src_buf) <= 20)
		return -1;

	// init array
	YY[2] = MM[2] = DD[2] = DoW[2] = hh[2] = mm[2] = ss[2] = '\0';
	YY[1] = MM[1] = DD[1] = DoW[1] = hh[1] = mm[1] = ss[1] = '\0';

	if (mode == 0)
		pass = 1;
	else
		pass = 3;

	for(i=6; i<30; i++) {
		if (src_buf[i] == '\x0a')
			break;

		if (src_buf[i] == ',') {
			pass ++;
			copy = 0;
			continue;
		}

		switch(pass) {
			case 1: // year
				YY[copy] = src_buf[i];
				copy ++;
			break;
			case 2: // Month
				MM[copy] = src_buf[i];
				copy ++;
			break;
			case 3: // Day
				DD[copy] = src_buf[i];
				copy ++;
			break;
			case 4: // Day o Week
				DoW[copy] = src_buf[i];
				copy ++;
			break;
			case 5: // Hour
				hh[copy] = src_buf[i];
				copy ++;
			break;
			case 6: // min
				mm[copy] = src_buf[i];
				copy ++;
			break;
			case 7: // sec
				ss[copy] = src_buf[i];
				copy ++;
			break;
		}
	}
	if (mode == 0) {
		dt->tm_sec = simple_strtoul(ss, NULL, 10);
		dt->tm_min = simple_strtoul(mm, NULL, 10);
		dt->tm_hour = simple_strtoul(hh, NULL, 10);
		dt->tm_mday = simple_strtoul(DD, NULL, 10);
		dt->tm_mon = simple_strtoul(MM, NULL, 10) - 1;
		dt->tm_wday = simple_strtoul(DoW, NULL, 10) - 1;
		dt->tm_year = simple_strtoul(YY, NULL, 10) + 2000 - 1900;
	} else {
		if (!strcmp(src_buf, "$g_al x,x,x,x,x")) {
			dt->tm_sec = -1;
			dt->tm_min = -1;
			dt->tm_hour = -1;
			dt->tm_mday = -1;
			dt->tm_mon = -1;
			dt->tm_year = -1;
			dt->tm_wday = -1;
			dt->tm_yday = -1;
		} else {
			if (DoW[0] == 'x') {
				dt->tm_wday = -1;
				dt->tm_mday = simple_strtoul(DD, NULL, 10);
			} else {
				dt->tm_wday = simple_strtoul(DoW, NULL, 10) - 1;
				dt->tm_mday = -1;
			}
			dt->tm_sec = simple_strtoul(ss, NULL, 10);
			dt->tm_min = simple_strtoul(mm, NULL, 10);
			dt->tm_hour = simple_strtoul(hh, NULL, 10);
		}
	}
	return 0;
}

void date_to_char(struct rtc_time *dt, char *src_buf, int mode)
{
	char YY[3], MM[3], DD[3], DoW[2], hh[3], mm[3], ss[3];
	// init array
	YY[2] = MM[2] = DD[2] = DoW[1] = hh[2] = mm[2] = ss[2] = '\0';
	YY[1] = MM[1] = DD[1] = hh[1] = mm[1] = ss[1] = '\0';

	if ((YY[0] = covert_int_to_char((dt->tm_year + 1900 - 2000) / 10)) > 0) {
		YY[1] = covert_int_to_char(dt->tm_year % 10);
	} else {
		YY[0] = covert_int_to_char(dt->tm_year % 10);
	}

	if ((dt->tm_mon + 1) >= 10) {
		MM[0] = covert_int_to_char(1);
		MM[1] = covert_int_to_char((dt->tm_mon + 1) % 10);
	} else {
		MM[0] = covert_int_to_char((dt->tm_mon + 1) % 10);
	}

	if ((DD[0] = covert_int_to_char(dt->tm_mday / 10)) > 0) {
		DD[1] = covert_int_to_char(dt->tm_mday % 10);
	} else {
		DD[0] = covert_int_to_char(dt->tm_mday % 10);
	}

	DoW[0] = covert_int_to_char(dt->tm_wday % 10);

	if ((hh[0] = covert_int_to_char(dt->tm_hour / 10)) > 0)
		hh[1] = covert_int_to_char(dt->tm_hour % 10);
	else
		hh[0] = covert_int_to_char(dt->tm_hour % 10);

	if ((mm[0] = covert_int_to_char(dt->tm_min / 10)) > 0) {
		mm[1] = covert_int_to_char(dt->tm_min % 10);
	} else {
		mm[0] = covert_int_to_char(dt->tm_min % 10);
	}

	if ((ss[0] = covert_int_to_char(dt->tm_sec / 10)) > 0) {
		ss[1] = covert_int_to_char(dt->tm_sec % 10);
	} else {
		ss[0] = covert_int_to_char(dt->tm_sec % 10);
	}
	if (mode == 0)
		sprintf(src_buf, "s_dt %s,%s,%s,%s,%s,%s,%s\n\r", YY, MM, DD, DoW, hh, mm, ss);
	else {
		if (!strcmp(DD, "0")) {
			sprintf(src_buf, "s_al x,%s,%s,%s,%s\n\r", DoW, hh, mm, ss);
		} else {
			sprintf(src_buf, "s_al %s,x,%s,%s,%s\n\r", DD, hh, mm, ss);
		}
	}
}

static int mcu_read_time(struct device *dev, struct rtc_time *dt)
{
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc;

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg)
		return -EIO;

	if (!mutex_trylock(&priv->mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		//dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		// set default value
		rc = 0;
		dt->tm_sec = 0;
		dt->tm_min = 0;
		dt->tm_hour = 12;
		dt->tm_mday = 1;
		dt->tm_mon = 0;
		dt->tm_wday = 0;
		dt->tm_year = 123;
		goto fail1;
	}

	rc = mcu_send_cmd(priv->tty_file, "g_dt\n\r", recv_msg, 6, 30);
	if (rc < 0) {
		//printk(KERN_ALERT "%s mcu_send_cmd failed\n", __FUNCTION__);
		goto fail;
	}/* else
		printk(KERN_ALERT recv_msg:%s\n", recv_msg);*/

	if (!strncmp(recv_msg, "$g_dt ", 6)) {
		rc = char_to_date(dt, recv_msg, 0);
		//printk(KERN_ALERT "%s YY:%d MM:%d DD:%d DoW:%d hh:%d mm:%d ss:%d\n",
		//							__FUNCTION__, dt->tm_year, dt->tm_mon, dt->tm_mday, dt->tm_wday, dt->tm_hour, dt->tm_min, dt->tm_sec);
		// retry
		//if (rc < 0 && retry < 3) {
		//	retry ++;
		//	goto read;
		//} else if (retry == 3){
		//	pr_debug("Time format failed! recv_msg: %s\n", recv_msg);
		//	return -1;
		//}
	} else {
		dev_err(dev, "RTC data error\n");
	}

fail:
	filp_close(priv->tty_file, NULL);

fail1:
	kfree(recv_msg);

	mutex_unlock(&priv->mutex);

	return rc;
}

static int mcu_set_time(struct device *dev, struct rtc_time *dt)
{
	unsigned char send_buf[30];
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc = 0;

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg)
		return -EIO;

	if (!mutex_trylock(&priv->mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}

	tty_driver_flush_buffer(priv->tty);
	date_to_char(dt, send_buf, 0);

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		//dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		goto fail1;
	}

	rc = mcu_send_cmd(priv->tty_file, send_buf, recv_msg, sizeof(send_buf), 30);
	if (rc < 0) {
		//printk(KERN_ALERT "%s mcu_send_cmd failed\n", __FUNCTION__);
		goto fail;
	} /*else
		printk(KERN_ALERT "%s recv_msg:%s\n", __FUNCTION__, recv_msg);*/

	if (!strncmp(recv_msg, "$s_dt", 5))
		rc = 0;
	else {
		rc = -110;
		//dev_err(dev, "MCU RTC setting failed\n");
	}

fail:
	filp_close(priv->tty_file, NULL);
fail1:

	kfree(recv_msg);
	mutex_unlock(&priv->mutex);

	return rc;
}

static int mcu_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc;

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg)
		return -EIO;

	mutex_lock(&priv->mutex);

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		//dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		goto fail1;
	}

	rc = mcu_send_cmd(priv->tty_file, "g_al\n\r", recv_msg, 6, 30);
	if (rc < 0) {
		//printk(KERN_ALERT "%s mcu_send_cmd failed\n", __FUNCTION__);
		goto fail;
	} else {
		//printk(KERN_ALERT "recv_msg:%s\n", recv_msg);
		rc = 0;
	}

	if (!strncmp(recv_msg, "$g_al ", 6)) {
		char_to_date(&alarm->time, recv_msg, 1);
		if (!strcmp(recv_msg, "$g_al x,x,x,x,x")) {
			alarm->enabled = 0;
			alarm->pending = 0;
		} else {
			alarm->enabled = 1;
		}

		/*printk(KERN_ALERT "%s DD:%d DoW:%d hh:%d mm:%d ss:%d\n", __FUNCTION__
									, alarm->time.tm_mday
									, alarm->time.tm_wday
									, alarm->time.tm_hour
									, alarm->time.tm_min
									, alarm->time.tm_sec);*/
		rc = 0;
	} else {
		dev_err(dev, "%s RTC data error\n", __FUNCTION__);
	}

fail:
	filp_close(priv->tty_file, NULL);
fail1:
	kfree(recv_msg);

	mutex_unlock(&priv->mutex);

	return rc;
}

static int mcu_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	unsigned char send_buf[30];
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc = 0;

	date_to_char(&alarm->time, send_buf, 1);

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg)
		return -EIO;

	tty_driver_flush_buffer(priv->tty);
	mutex_lock(&priv->mutex);

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		//dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		goto fail1;
	}

	rc = mcu_send_cmd(priv->tty_file, send_buf, recv_msg, sizeof(send_buf), 30);
	if (rc < 0) {
		//printk(KERN_ALERT "%s mcu_send_cmd failed\n", __FUNCTION__);
		goto fail;
	} /*else
		printk(KERN_ALERT "%s recv_msg:%s\n", __FUNCTION__, recv_msg);*/

	if (!strncmp(recv_msg, "$s_al", 5)) {
		rc = 0;
		alarm->time.tm_mon = -1;
		alarm->time.tm_year = -1;
		rc = 0;
	} else {
		rc = -110;
		dev_err(dev, "MCU RTC setting failed\n");
	}

fail:
	filp_close(priv->tty_file, NULL);
fail1:
	kfree(recv_msg);
	mutex_unlock(&priv->mutex);

	return rc;
}

/*
static int mcu_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	return 0;
}
*/

static const struct rtc_class_ops mcu_rtc_ops = {
	.read_time = mcu_read_time,
	.set_time = mcu_set_time,
	.read_alarm = mcu_read_alarm,
	.set_alarm = mcu_set_alarm,
	//.alarm_irq_enable = mcu_alarm_irq_enable,
};


static int mcu_rtc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	//struct device *dev;
	struct rtc_device *rtc;
	struct mcu_dev *mcudev;
	dev_t dev;
	int rc;

	/* Allocate memory for our private structure */
	mcudev = kzalloc(sizeof(*mcudev), GFP_KERNEL);
	if (!mcudev) {
		return -ENOMEM;
	}

	rc = of_property_read_string(np, "tty-port", &mcudev->tty_port);
	if (rc) {
		mcudev->tty_port = MCU_UART;
		pr_err("tty-port property not found, use default %s\n",
			mcudev->tty_port);
	}

	rc = of_property_read_u32(np, "baudrate", &mcudev->baudrate);
	if (rc) {
		mcudev->baudrate = 115200;
		pr_err("baudrate property not found, use default %u\n",
			mcudev->baudrate);
	}

	// Other way to open ttyS1 ---------------------------------------------
	rc = tty_dev_name_to_number("ttyS1", &dev);
	if (rc) {
		printk(KERN_ERR "tty_dev_name_to_number failed\n");
		return rc;
	}
	mcudev->tty = tty_kopen_exclusive(dev);
	//mcudev->tty = tty_kopen_shared(dev);
	if (IS_ERR(mcudev->tty) || mcudev->tty == NULL) {
		printk(KERN_ERR "tty_kopen_exclusive failed\n");
		return -1;
	}
	if (mcudev->tty->ops->open)
		rc = mcudev->tty->ops->open(mcudev->tty, NULL);
	else
		rc = -ENODEV;
	if (rc) {
		tty_unlock(mcudev->tty);
		return rc;
	}
	// ttyS1 setting
	mcudev->tty->termios.c_lflag = 0;
	mcudev->tty->termios.c_oflag = 0;
	mcudev->tty->termios.c_cc[VMIN] = 1;
	mcudev->tty->termios.c_cc[VTIME] = 1;
	tty_termios_encode_baud_rate(&mcudev->tty->termios, 115200, 115200);

	tty_driver_flush_buffer(mcudev->tty);

	if (mcudev->tty->ops->close)
		mcudev->tty->ops->close(mcudev->tty, NULL);
	tty_kclose(mcudev->tty);
	// ---------------------------------------------------------------------

	rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	mcudev->rtc = rtc;

	mutex_init(&mcudev->mutex);

	mcudev->rtc->ops = &mcu_rtc_ops;
	mcudev->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	mcudev->rtc->range_max = RTC_TIMESTAMP_END_2099;

	device_set_wakeup_capable(&pdev->dev, true);
	mcudev->rtc->uie_unsupported = 1;
	set_bit(RTC_FEATURE_ALARM, mcudev->rtc->features);

	dev_set_drvdata(&pdev->dev, mcudev);
	return devm_rtc_register_device(mcudev->rtc);
}

static int mcu_rtc_suspend(struct device *dev)
{
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc = 0, retry = 3;

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg) {
		dev_err(dev, "Allocate memory failed\n");
		return -EIO;
	}

	tty_driver_flush_buffer(priv->tty);
	mutex_lock(&priv->mutex);

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		goto fail1;
	}

	mcu_send_cmd(priv->tty_file, "\r\ns_sb 1\r\n", recv_msg, 10, 30);

	filp_close(priv->tty_file, NULL);
fail1:
	kfree(recv_msg);
	mutex_unlock(&priv->mutex);

	return rc;
}

static int mcu_rtc_resume(struct device *dev)
{
	struct mcu_dev *priv = dev_get_drvdata(dev);
	char *recv_msg;
	int rc = 0, retry = 3;

	recv_msg = kmalloc(sizeof(char) * 30, GFP_KERNEL);
	if (!recv_msg)
		return -EIO;

	tty_driver_flush_buffer(priv->tty);
	mutex_lock(&priv->mutex);

	priv->tty_file = filp_open(priv->tty_port, O_RDWR | O_NOCTTY | O_NDELAY, 0);
	if (IS_ERR(priv->tty_file)) {
		rc = (int)PTR_ERR(priv->tty_file);
		//dev_err(dev, "file %s open error = %d\n", priv->tty_port, rc);
		goto fail1;
	}

	mcu_send_cmd(priv->tty_file, "\r\ns_sb 0\r\n", recv_msg, 10, 30);

	filp_close(priv->tty_file, NULL);
fail1:
	kfree(recv_msg);
	mutex_unlock(&priv->mutex);

	return rc;
}

static const struct dev_pm_ops mcu_rtc_pm_ops = {
	.prepare = mcu_rtc_suspend,
	.resume = mcu_rtc_resume,
};

static struct platform_driver mcu_rtc_driver = {
	.driver = {
		.name = "mcu,rtc",
		.of_match_table	= mcu_rtc_match_table,
		.pm = &mcu_rtc_pm_ops,
	},
	.probe = mcu_rtc_probe,
};

static int __init mcu_rtc_module_init(void){
	return platform_driver_register(&mcu_rtc_driver);
}
//module_init(mcu_rtc_module_init);
late_initcall(mcu_rtc_module_init);

static void __exit mcu_rtc_module_exit(void)
{
	platform_driver_unregister(&mcu_rtc_driver);
}
module_exit(mcu_rtc_module_exit);

MODULE_DEVICE_TABLE(of, mcu_rtc_match_table);
//module_platform_driver(mcu_rtc_driver);

MODULE_DESCRIPTION("MCU RTC DRIVER");
MODULE_AUTHOR("Bass Huang <basshuang@via.com.tw>");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);