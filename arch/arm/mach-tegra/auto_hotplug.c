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
#include <linux/slab.h>
#include <linux/input.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * Enable debug output to dump the average
 * calculations and ring buffer array values
 * WARNING: Enabling this causes a ton of overhead
 *
 * FIXME: Turn it into debugfs stats (somehow)
 * because currently it is a sack of shit.
 */
#define DEBUG 0
#define CPUS_AVAILABLE		num_possible_cpus()
#define SAMPLING_PERIODS 	18	/* SAMPLING_PERIODS * MIN_SAMPLING_RATE is the minimum load history which will be averaged */
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)
#define MIN_SAMPLING_RATE	msecs_to_jiffies(20)	/* MIN_SAMPLING_RATE is scaled based on num_online_cpus() */

/* Control flags */
unsigned char flags;
#define HOTPLUG_DISABLED	(1 << 0)
#define HOTPLUG_PAUSED		(1 << 1)
#define EARLYSUSPEND_ACTIVE	(1 << 3)

/*
 * Load defines:
 * ENABLE_ALL is a high watermark to rapidly online all CPUs
 *
 * DISABLE is the load at which a CPU is disabled
 * These two are scaled based on num_online_cpus()
 */

static unsigned int enable_all_load_threshold = 500;
static unsigned int second_threshold = 450;
static unsigned int disable_load_threshold = 80;
static unsigned int min_cpu_online = 1;
static unsigned int max_cpu_online = 4;
static unsigned int first_threshold = 300;

module_param(enable_all_load_threshold, int, 0755);
module_param(second_threshold, int, 0755);
module_param(disable_load_threshold, int, 0755);
module_param(min_cpu_online, int, 0755);
module_param(max_cpu_online, int, 0755);
module_param(first_threshold, int, 0755);

struct delayed_work hotplug_decision_work;
struct delayed_work hotplug_unpause_work;
struct work_struct hotplug_online_all_work;
struct work_struct hotplug_online_single_work;
struct work_struct touchplug_boost_work;
struct delayed_work hotplug_offline_work;

static unsigned int history[SAMPLING_PERIODS];
static unsigned int index;

static void hotplug_decision_work_fn(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, avg_running = 0;
	unsigned int online_cpus, available_cpus, i, j;
#if DEBUG
	unsigned int k;
#endif

	online_cpus = num_online_cpus();
	available_cpus = 4;
	disable_load = disable_load_threshold * online_cpus; ;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use fp division to get the average.
	 */
	running = nr_running() * 100;
	history[index] = running;

#if DEBUG
	pr_info("online_cpus is: %d\n", online_cpus);
	pr_info("enable_load is: %d\n", enable_load);
	pr_info("disable_load is: %d\n", disable_load);
	pr_info("index is: %d\n", index);
	pr_info("running is: %d\n", running);
#endif

	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < SAMPLING_PERIODS; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;

#if DEBUG
	pr_info("array contents: ");
	for (k = 0; k < SAMPLING_PERIODS; k++) {
		 pr_info("%d: %d\t",k, history[k]);
	}
	pr_info("\n");
	pr_info("avg_running before division: %d\n", avg_running);
#endif

	avg_running = avg_running / SAMPLING_PERIODS;

#if DEBUG
	pr_info("average_running is: %d\n", avg_running);
#endif

	if (likely(!(flags & HOTPLUG_DISABLED))) {
		if (unlikely((avg_running >= enable_all_load_threshold) && (online_cpus < available_cpus))) {
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
			schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE); 
			return;
		} else if ((avg_running >= first_threshold) && (online_cpus < 2)) {
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);			
			schedule_work(&hotplug_online_single_work);
			return;
		} else if ((avg_running >= second_threshold) && (online_cpus < available_cpus)) {
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_single_work);
			return;
		} else if (avg_running <= disable_load) {
			/* Only queue a cpu_down() if there isn't one already pending */
			if (!(delayed_work_pending(&hotplug_offline_work))) {
				pr_debug("auto_hotplug: Offlining CPU, avg running: %d\n", avg_running);
				schedule_delayed_work_on(0, &hotplug_offline_work, HZ);
			return;
			}
		}
	}

	/*
	 * Reduce the sampling rate dynamically based on online cpus.
	 */
	sampling_rate = MIN_SAMPLING_RATE * (online_cpus * online_cpus);
#if DEBUG
	pr_info("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
	schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);

}

static void __init hotplug_online_all_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu))) {
			cpu_up(cpu);
			pr_info("auto_hotplug: CPU%d up.\n", cpu);
		}
	}
	/*
	 * Pause for 1 second before even considering offlining a CPU
	 */
	schedule_delayed_work(&hotplug_unpause_work, HZ * 1);
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void __init hotplug_online_single_work_fn(struct work_struct *work)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu) {
			if (!cpu_online(cpu)) {
				cpu_up(cpu);
				pr_info("auto_hotplug: CPU%d up.\n", cpu);
				break;
			}
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void __init touchplug_boost_work_fn(struct work_struct *work)
{
	if (num_online_cpus() < 2) {
 			cpu_up(1);
	}
	schedule_delayed_work(&hotplug_unpause_work, HZ * 1);
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void hotplug_offline_work_fn(struct work_struct *work)
{
	int cpu;

	for_each_online_cpu(cpu) {
		if (cpu && num_online_cpus() > min_cpu_online) {
			cpu_down(num_online_cpus() - 1);
			pr_info("auto_hotplug: CPU%d down.\n", cpu);
			break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void hotplug_unpause_work_fn(struct work_struct *work)
{
	pr_info("auto_hotplug: Clearing pause flag\n");
	flags &= ~HOTPLUG_PAUSED;
}

void hotplug_disable(bool flag)
{
	if (flags & HOTPLUG_DISABLED && !flag) {
		flags &= ~HOTPLUG_DISABLED;
		flags &= ~HOTPLUG_PAUSED;
		pr_info("auto_hotplug: Clearing disable flag\n");
		schedule_delayed_work_on(0, &hotplug_decision_work, 0);
	} else if (flag && (!(flags & HOTPLUG_DISABLED))) {
		flags |= HOTPLUG_DISABLED;
		pr_info("auto_hotplug: Setting disable flag\n");
		cancel_delayed_work_sync(&hotplug_offline_work);
		cancel_delayed_work_sync(&hotplug_decision_work);
		cancel_delayed_work_sync(&hotplug_unpause_work);
	}
}

static void touchplug_input_event(struct input_handle *handle,
 		unsigned int type, unsigned int code, int value)
{
#ifdef DEBUG
 	pr_info("touchplug touched!\n");
#endif
 	cancel_delayed_work_sync(&hotplug_offline_work);
 
	schedule_work(&touchplug_boost_work);
}
 
static int input_dev_filter(const char *input_dev_name)
{
 	if (strstr(input_dev_name, "touchscreen") ||
 		strstr(input_dev_name, "sec_touchscreen") ||
 		strstr(input_dev_name, "touch_dev") ||
 		strstr(input_dev_name, "-keypad") ||
 		strstr(input_dev_name, "-nav") ||
 		strstr(input_dev_name, "-oj")) {
 		pr_info("touch dev: %s\n", input_dev_name);
 		return 0;
 	} else {
 		pr_info("touch dev: %s\n", input_dev_name);
 		return 1;
 	}
}
 
static int touchplug_input_connect(struct input_handler *handler,
 		struct input_dev *dev, const struct input_device_id *id)
{
 	struct input_handle *handle;
 	int error;
 
 	if (input_dev_filter(dev->name))
 		return -ENODEV;
 
 	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
 	if (!handle)
 		return -ENOMEM;
 
 	handle->dev = dev;
 	handle->handler = handler;
 	handle->name = "auto_hotplug";
 
 	error = input_register_handle(handle);
 	if (error)
 		goto err2;
 
 	error = input_open_device(handle);
 	if (error)
 		goto err1;
 	pr_info("%s found and connected!\n", dev->name);
 	return 0;
 err1:
 	input_unregister_handle(handle);
 err2:
 	kfree(handle);
 	return error;
}
 
static void touchplug_input_disconnect(struct input_handle *handle)
{
 	input_close_device(handle);
 	input_unregister_handle(handle);
 	kfree(handle);
}

static const struct input_device_id touchplug_ids[] = {
 	{
 		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
 			 INPUT_DEVICE_ID_MATCH_ABSBIT,
 		.evbit = { BIT_MASK(EV_ABS) },
 		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
 			    BIT_MASK(ABS_MT_POSITION_X) |
 			    BIT_MASK(ABS_MT_POSITION_Y) },
 	}, /* multi-touch touchscreen */
 	{
 		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
 			 INPUT_DEVICE_ID_MATCH_ABSBIT,
 		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
 		.absbit = { [BIT_WORD(ABS_X)] =
 			    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
 	}, /* touchpad */
 	{ },
};
 
static struct input_handler touchplug_input_handler = {
 	.event          = touchplug_input_event,
 	.connect        = touchplug_input_connect,
 	.disconnect     = touchplug_input_disconnect,
 	.name           = "touchplug_input_handler",
 	.id_table       = touchplug_ids,
};

int __init auto_hotplug_init(void)
{
	int ret;

	pr_info("auto_hotplug: v0.220 by _thalamus\n");
	pr_info("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	ret = input_register_handler(&touchplug_input_handler);

	INIT_DELAYED_WORK(&hotplug_decision_work, hotplug_decision_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_unpause_work, hotplug_unpause_work_fn);
	INIT_WORK(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_WORK(&hotplug_online_single_work, hotplug_online_single_work_fn);
	INIT_WORK(&touchplug_boost_work, touchplug_boost_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_work, hotplug_offline_work_fn);

	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	flags |= HOTPLUG_PAUSED;
	schedule_delayed_work_on(0, &hotplug_decision_work, HZ * 10);
	schedule_delayed_work(&hotplug_unpause_work, HZ * 20);
	return 0;
}
late_initcall(auto_hotplug_init);
