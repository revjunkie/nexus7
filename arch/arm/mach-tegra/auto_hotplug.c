/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Generic auto hotplug driver for ARM SoCs. Targeted at current generation
 * SoCs with dual and quad core applications processors.
 * Automatically hotplugs online and offline CPUs based on system load.
 *
 * Not recommended for use with OMAP4460 due to the potential for lockups
 * whilst hotplugging.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CPUS_AVAILABLE		num_possible_cpus()
#define SAMPLING_PERIODS 		18	
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)	

#define SHIFT_ALL			500
#define SHIFT_CPU			225
#define DOWN_SHIFT		100
#define MIN_CPU			1
#define MAX_CPU			4
#define SAMPLE_TIME		20

/* Control flags */
unsigned char flags;
#define HOTPLUG_DISABLED	(1 << 0)
#define HOTPLUG_PAUSED		(1 << 1)
#define EARLYSUSPEND_ACTIVE	(1 << 3)

struct rev_tune
{
unsigned int shift_all;
unsigned int shift_cpu;
unsigned int down_shift;
unsigned int min_cpu;
unsigned int max_cpu;
unsigned int sample_time;
unsigned int sampling_period;
} rev = {
	.shift_all = SHIFT_ALL,
	.shift_cpu = SHIFT_CPU,
	.down_shift = DOWN_SHIFT,
	.min_cpu = MIN_CPU,
	.max_cpu = MAX_CPU,
	.sample_time = SAMPLE_TIME,
	.sampling_period = SAMPLING_PERIODS,
};

static unsigned int debug = 0;
module_param(debug, uint, 0644);

#define dprintk(msg...)		\
do { 				\
	if (debug)		\
		pr_info(msg);	\
} while (0)

struct delayed_work hotplug_decision_work;
struct delayed_work hotplug_unpause_work;
struct work_struct hotplug_online_all_work;
struct work_struct hotplug_online_single_work;
struct delayed_work hotplug_offline_work;

static unsigned int history[SAMPLING_PERIODS];
static unsigned int index;

static void hotplug_decision_work_fn(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, enable_load, avg_running = 0;
	unsigned int online_cpus, available_cpus, i, j;

	online_cpus = num_online_cpus();
	available_cpus = rev.max_cpu;
	disable_load = rev.down_shift * online_cpus; 
	enable_load = rev.shift_cpu * online_cpus;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use fp division to get the average.
	 */
	running = nr_running() * 100;
	history[index] = running;

	dprintk("online_cpus is: %d\n", online_cpus);
	dprintk("enable_load is: %d\n", enable_load);
	dprintk("disable_load is: %d\n", disable_load);
	dprintk("index is: %d\n", index);
	dprintk("running is: %d\n", running);	

	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < rev.sampling_period; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;
	
	avg_running = avg_running / rev.sampling_period;
	dprintk("average_running is: %d\n", avg_running);

	if (likely(!(flags & HOTPLUG_DISABLED))) {
		if (unlikely((avg_running >= rev.shift_all) && (online_cpus < available_cpus))) {
			pr_info("auto_hotplug: Onlining all CPUs, avg running: %d\n", avg_running);
			/*
			 * Flush any delayed offlining work from the workqueue.
			 * No point in having expensive unnecessary hotplug transitions.
			 * We still online after flushing, because load is high enough to
			 * warrant it.
			 * We set the paused flag so the sampling can continue but no more
			 * hotplug events will occur.
			 */
			flags |= HOTPLUG_PAUSED;
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_all_work);
			return;
		} else if (flags & HOTPLUG_PAUSED) {
			schedule_delayed_work_on(0, &hotplug_decision_work, msecs_to_jiffies(rev.sample_time));
			return;
		} else if ((avg_running >= enable_load) && (online_cpus < available_cpus)) {
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
			schedule_work(&hotplug_online_single_work);
			return;
		} else if (avg_running <= disable_load) {
			/* Only queue a cpu_down() if there isn't one already pending */
			if (!(delayed_work_pending(&hotplug_offline_work))) {
				pr_info("auto_hotplug: Offlining CPU, avg running: %d\n", avg_running);
				schedule_delayed_work_on(0, &hotplug_offline_work, HZ);
			}
		}
	}

	/*
	 * Reduce the sampling rate dynamically based on online cpus.
	 */
	sampling_rate = msecs_to_jiffies(rev.sample_time) * online_cpus;
	dprintk("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
	schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);

}

static void __cpuinit hotplug_online_all_work_fn(struct work_struct *work)
{
	unsigned int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu))) {
			cpu_up(cpu);
			dprintk("auto_hotplug: CPU%d up.\n", cpu);
		}
	}
	/*
	 * Pause for 1 second before even considering offlining a CPU
	 */
	schedule_delayed_work(&hotplug_unpause_work, HZ);
	schedule_delayed_work_on(0, &hotplug_decision_work, msecs_to_jiffies(rev.sample_time));
}

static void __cpuinit hotplug_online_single_work_fn(struct work_struct *work)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu) {
			if (!cpu_online(cpu)) {
				cpu_up(cpu);
				dprintk("auto_hotplug: CPU%d up.\n", cpu);
				break;
			}
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, msecs_to_jiffies(rev.sample_time));
}

static void hotplug_offline_work_fn(struct work_struct *work)
{
	unsigned int cpu;

	for_each_online_cpu(cpu) {
		if (num_online_cpus() > rev.min_cpu)
			if (cpu) {
				cpu_down(num_online_cpus() - 1);
				dprintk("auto_hotplug: CPU%d down.\n", cpu);
				break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, msecs_to_jiffies(rev.sample_time));
}

static void hotplug_unpause_work_fn(struct work_struct *work)
{
	dprintk("auto_hotplug: Clearing pause flag\n");
	flags &= ~HOTPLUG_PAUSED;
}

void hotplug_disable(bool flag)
{
	if (flags & HOTPLUG_DISABLED && !flag) {
		flags &= ~HOTPLUG_DISABLED;
		flags &= ~HOTPLUG_PAUSED;
		dprintk("auto_hotplug: Clearing disable flag\n");
		schedule_delayed_work_on(0, &hotplug_decision_work, 0);
	} else if (flag && (!(flags & HOTPLUG_DISABLED))) {
		flags |= HOTPLUG_DISABLED;
		dprintk("auto_hotplug: Setting disable flag\n");
		cancel_delayed_work_sync(&hotplug_offline_work);
		cancel_delayed_work_sync(&hotplug_decision_work);
		cancel_delayed_work_sync(&hotplug_unpause_work);
	}
}

/**************SYSFS*******************/

static ssize_t shift_cpu_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.shift_cpu);
}

static ssize_t shift_cpu_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.shift_cpu && val >= 0 && val <= 500)
	{
		rev.shift_cpu = val;
	}

	return size;
}

static ssize_t shift_all_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.shift_all);
}

static ssize_t shift_all_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.shift_all && val >= 0 && val <= 600)
	{
		rev.shift_all = val;
	}

	return size;
}

static ssize_t down_shift_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.down_shift);
}

static ssize_t down_shift_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.down_shift && val >= 0 && val <= 200)
	{
		rev.down_shift = val;
	}

	return size;
}

static ssize_t min_cpu_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.min_cpu);
}

static ssize_t min_cpu_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.min_cpu && val >= 1 && val <= 4)
	{
		rev.min_cpu = val;
	}

	return size;
}

static ssize_t max_cpu_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.max_cpu);
}

static ssize_t max_cpu_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.max_cpu && val >= 1 && val <= 4)
	{
		rev.max_cpu = val;
	}

	return size;
}

static ssize_t sample_time_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.sample_time);
}

static ssize_t sample_time_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.sample_time && val >= 1 && val <= 500)
	{
		rev.sample_time = val;
	}

	return size;
}

static ssize_t sampling_period_show(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", rev.sampling_period);
}

static ssize_t sampling_period_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	if (val != rev.sampling_period && val >= 1 && val <= 500)
	{
		rev.sampling_period = val;
	}

	return size;
}

static DEVICE_ATTR(shift_cpu, 0644, shift_cpu_show, shift_cpu_store);
static DEVICE_ATTR(shift_all, 0644, shift_all_show, shift_all_store);
static DEVICE_ATTR(down_shift, 0644, down_shift_show, down_shift_store);
static DEVICE_ATTR(min_cpu, 0644, min_cpu_show, min_cpu_store);
static DEVICE_ATTR(max_cpu, 0644, max_cpu_show, max_cpu_store);
static DEVICE_ATTR(sample_time, 0644, sample_time_show, sample_time_store);
static DEVICE_ATTR(sampling_period, 0644, sampling_period_show, sampling_period_store);

static struct attribute *revshift_hotplug_attributes[] = 
    {
	&dev_attr_shift_cpu.attr,
	&dev_attr_shift_all.attr,
	&dev_attr_down_shift.attr,
	&dev_attr_min_cpu.attr,
	&dev_attr_max_cpu.attr,
	&dev_attr_sample_time.attr,	
	&dev_attr_sampling_period.attr,
	NULL
    };

static struct attribute_group revshift_hotplug_group = 
    {
	.attrs  = revshift_hotplug_attributes,
    };

static struct miscdevice revshift_hotplug_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "revshift_hotplug",
    };


#ifdef CONFIG_HAS_EARLYSUSPEND
static void auto_hotplug_early_suspend(struct early_suspend *handler)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu)
			cpu_down(cpu);
	dprintk("auto_hotplug: Offlining CPUs for early suspend\n");
	}
	flags |= EARLYSUSPEND_ACTIVE;

	/* Cancel all scheduled delayed work to avoid races */
	cancel_delayed_work_sync(&hotplug_offline_work);
	cancel_delayed_work_sync(&hotplug_decision_work);
}

static void auto_hotplug_late_resume(struct early_suspend *handler)
{
	dprintk("auto_hotplug: late resume handler\n");
	flags &= ~EARLYSUSPEND_ACTIVE;

	schedule_delayed_work_on(0, &hotplug_decision_work, HZ);
}

static struct early_suspend auto_hotplug_suspend = {
	.suspend = auto_hotplug_early_suspend,
	.resume = auto_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

int __init auto_hotplug_init(void)
{
	int ret;

	pr_info("auto_hotplug: v0.220 by _thalamus\n");
	dprintk("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	ret = misc_register(&revshift_hotplug_device);
	if (ret)
	{
		ret = -EINVAL;
		goto err;
	}
	ret = sysfs_create_group(&revshift_hotplug_device.this_device->kobj,
			&revshift_hotplug_group);

	if (ret)
	{
		ret = -EINVAL;
		goto err;
	}

	INIT_DELAYED_WORK(&hotplug_decision_work, hotplug_decision_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_unpause_work, hotplug_unpause_work_fn);
	INIT_WORK(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_WORK(&hotplug_online_single_work, hotplug_online_single_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_work, hotplug_offline_work_fn);

	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	flags |= HOTPLUG_PAUSED;
	schedule_delayed_work_on(0, &hotplug_decision_work, HZ * 10);
	schedule_delayed_work(&hotplug_unpause_work, HZ * 20);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&auto_hotplug_suspend);
#endif
	return 0;
err:
	return ret;
}

late_initcall(auto_hotplug_init);
