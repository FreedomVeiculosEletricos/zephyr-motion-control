/*
 * Copyright (c) 2026 Freedom Veiculos Eletricos
 * SPDX-License-Identifier: UNLICENSED
 *
 * Motor multi-instance registration (iterable entries + discovery API).
 * Motor group API is implemented on top of the public motor_*() API.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/util.h>

#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_subsys.h>

static motor_t instances[CONFIG_MOTOR_MAX_INSTANCES];
static uint8_t instance_count;
static bool motor_subsys_ready;

struct motor_group_enable_async_ctx {
	struct motor_group *group;
	struct k_work_delayable work;
	int64_t deadline;
	bool in_use;
};

static struct motor_group_enable_async_ctx enable_async_ctx[CONFIG_MOTOR_MAX_GROUPS];

static void motor_group_release_async_slot(struct motor_group *group);

static int64_t deadline_from_timeout(k_timeout_t timeout)
{
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		return INT64_MAX;
	}

	return k_uptime_ticks() + (int64_t)timeout.ticks;
}

static uint32_t member_fault_mask(const struct motor_group *group)
{
	uint32_t mask = 0U;

	for (uint8_t i = 0; i < group->count; i++) {
		enum motor_state st;

		motor_get_status(group->members[i], &st, NULL, NULL);
		if (st == MOTOR_STATE_FAULT) {
			mask |= BIT(i);
		}
	}

	return mask;
}

static bool members_all_in_state(const struct motor_group *group, enum motor_state target)
{
	for (uint8_t i = 0; i < group->count; i++) {
		enum motor_state st;

		motor_get_status(group->members[i], &st, NULL, NULL);
		if (st != target) {
			return false;
		}
	}

	return true;
}

static void apply_fault_policy(struct motor_group *group)
{
	switch (group->fault_policy) {
	case MOTOR_GROUP_FAULT_ESTOP_ALL:
		for (uint8_t i = 0; i < group->count; i++) {
			motor_estop(group->members[i]);
		}
		break;
	case MOTOR_GROUP_FAULT_DISABLE_ALL:
		for (uint8_t i = 0; i < group->count; i++) {
			(void)motor_disable(group->members[i]);
		}
		break;
	case MOTOR_GROUP_FAULT_ISOLATE:
	default:
		break;
	}
}

/*
 * Settle an async-enable slot in the given terminal state, release it, and
 * fire the group callback. Must run from thread context (k_spin_lock).
 */
static void enable_async_finish(struct motor_group_enable_async_ctx *slot,
				enum motor_group_state final_state, uint32_t fault_mask)
{
	struct motor_group *group = slot->group;
	k_spinlock_key_t key = k_spin_lock(&group->lock);

	group->state = final_state;
	k_spin_unlock(&group->lock, key);

	if (final_state == MOTOR_GROUP_FAULT) {
		apply_fault_policy(group);
	}

	slot->in_use = false;
	slot->group = NULL;

	if (group->cb != NULL) {
		group->cb(group, final_state, fault_mask, group->cb_user_data);
	}
}

static void enable_async_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct motor_group_enable_async_ctx *slot =
		CONTAINER_OF(dwork, struct motor_group_enable_async_ctx, work);
	struct motor_group *group = slot->group;
	uint32_t fm;

	if (group == NULL || !slot->in_use) {
		return;
	}

	if (k_uptime_ticks() >= slot->deadline) {
		enable_async_finish(slot, MOTOR_GROUP_FAULT, member_fault_mask(group));
		return;
	}

	fm = member_fault_mask(group);
	if (fm != 0U) {
		enable_async_finish(slot, MOTOR_GROUP_FAULT, fm);
		return;
	}

	if (members_all_in_state(group, MOTOR_STATE_RUN)) {
		enable_async_finish(slot, MOTOR_GROUP_RUN, 0U);
		return;
	}

	(void)k_work_reschedule(dwork, K_MSEC(10));
}

static int motor_group_enable_async_slots_init(void)
{
	for (unsigned i = 0; i < ARRAY_SIZE(enable_async_ctx); i++) {
		k_work_init_delayable(&enable_async_ctx[i].work, enable_async_work_handler);
		enable_async_ctx[i].in_use = false;
		enable_async_ctx[i].group = NULL;
	}

	return 0;
}

SYS_INIT(motor_group_enable_async_slots_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_OBJECTS);

static struct motor_group_enable_async_ctx *alloc_enable_async_slot(void)
{
	for (unsigned i = 0; i < ARRAY_SIZE(enable_async_ctx); i++) {
		if (!enable_async_ctx[i].in_use) {
			enable_async_ctx[i].in_use = true;
			return &enable_async_ctx[i];
		}
	}

	return NULL;
}

static void motor_group_release_async_slot(struct motor_group *group)
{
	for (unsigned i = 0; i < ARRAY_SIZE(enable_async_ctx); i++) {
		if (enable_async_ctx[i].in_use && enable_async_ctx[i].group == group) {
			(void)k_work_cancel_delayable(&enable_async_ctx[i].work);
			enable_async_ctx[i].in_use = false;
			enable_async_ctx[i].group = NULL;
		}
	}
}

int motor_subsys_init(void)
{
	uint8_t i = 0;

	if (motor_subsys_ready) {
		return 0;
	}

	STRUCT_SECTION_FOREACH(motor_subsys_entry, entry) {
		motor_t m;

		if (i >= CONFIG_MOTOR_MAX_INSTANCES) {
			return -ENOMEM;
		}

		m = motor_init(entry->ctrl, entry->sensor, entry->actuator, entry->algo, entry->algo_data,
			       entry->params);
		if (m == NULL) {
			return -EIO;
		}

		instances[i] = m;
		i++;
	}

	instance_count = i;
	motor_subsys_ready = true;
	return 0;
}

motor_t motor_subsys_get_by_label(const char *label)
{
	uint8_t i = 0;

	if (!motor_subsys_ready || (label == NULL)) {
		return NULL;
	}

	STRUCT_SECTION_FOREACH(motor_subsys_entry, entry) {
		if (strcmp(entry->label, label) == 0) {
			return (i < instance_count) ? instances[i] : NULL;
		}
		i++;
	}

	return NULL;
}

motor_t motor_subsys_get_by_index(uint8_t index)
{
	if (!motor_subsys_ready || (index >= instance_count)) {
		return NULL;
	}

	return instances[index];
}

uint8_t motor_subsys_count(void)
{
	return motor_subsys_ready ? instance_count : 0U;
}

#if IS_ENABLED(CONFIG_MOTOR_SUBSYS_AUTO_INIT)
static int motor_subsys_auto_init(void)
{
	return motor_subsys_init();
}

SYS_INIT(motor_subsys_auto_init, APPLICATION, CONFIG_MOTOR_SUBSYS_INIT_PRIORITY);
#endif

int motor_group_add(struct motor_group *group, motor_t *motors, uint8_t count,
		    enum motor_group_fault_policy policy)
{
	if ((group == NULL) || (motors == NULL) || (count == 0U) ||
	    (count > CONFIG_MOTOR_GROUP_MAX_MEMBERS)) {
		return -EINVAL;
	}

	if (group->count != 0U) {
		return -EBUSY;
	}

	for (uint8_t i = 0; i < count; i++) {
		if (motors[i] == NULL) {
			return -EINVAL;
		}
	}

	memcpy(group->members, motors, sizeof(motor_t) * count);
	group->count = count;
	group->fault_policy = policy;
	group->state = MOTOR_GROUP_IDLE;

	(void)k_sem_init(&group->align_done, 0, count);

	return 0;
}

void motor_group_register_cb(struct motor_group *group, motor_group_cb_t cb, void *user_data)
{
	if (group == NULL) {
		return;
	}

	group->cb = cb;
	group->cb_user_data = user_data;
}

int motor_group_self_test(struct motor_group *group, uint32_t *faults)
{
	int ret;

	if ((group == NULL) || (faults == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < group->count; i++) {
		ret = motor_self_test(group->members[i], &faults[i]);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Move the group from IDLE to ALIGNING and call motor_enable() on every
 * member. On failure all members are estopped and the group lands in FAULT.
 */
static int group_enable_preflight(struct motor_group *group)
{
	k_spinlock_key_t key = k_spin_lock(&group->lock);

	if (group->state != MOTOR_GROUP_IDLE) {
		k_spin_unlock(&group->lock, key);
		return -EBUSY;
	}

	k_spin_unlock(&group->lock, key);

	if (!members_all_in_state(group, MOTOR_STATE_IDLE)) {
		return -EBUSY;
	}

	key = k_spin_lock(&group->lock);
	group->state = MOTOR_GROUP_ALIGNING;
	k_spin_unlock(&group->lock, key);

	for (uint8_t i = 0; i < group->count; i++) {
		int ret = motor_enable(group->members[i]);

		if (ret == 0) {
			continue;
		}

		for (uint8_t j = 0; j < group->count; j++) {
			motor_estop(group->members[j]);
		}
		key = k_spin_lock(&group->lock);
		group->state = MOTOR_GROUP_FAULT;
		k_spin_unlock(&group->lock, key);
		return ret;
	}

	return 0;
}

int motor_group_enable(struct motor_group *group, k_timeout_t timeout)
{
	int64_t deadline = deadline_from_timeout(timeout);
	int ret;
	k_spinlock_key_t key;

	if ((group == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	ret = group_enable_preflight(group);
	if (ret != 0) {
		return ret;
	}

	while (k_uptime_ticks() < deadline) {
		uint32_t fm = member_fault_mask(group);

		if (fm != 0U) {
			key = k_spin_lock(&group->lock);
			group->state = MOTOR_GROUP_FAULT;
			k_spin_unlock(&group->lock, key);
			apply_fault_policy(group);
			return -EFAULT;
		}

		if (members_all_in_state(group, MOTOR_STATE_RUN)) {
			key = k_spin_lock(&group->lock);
			group->state = MOTOR_GROUP_RUN;
			k_spin_unlock(&group->lock, key);
			return 0;
		}

		k_sleep(K_MSEC(1));
	}

	key = k_spin_lock(&group->lock);
	group->state = MOTOR_GROUP_FAULT;
	k_spin_unlock(&group->lock, key);

	apply_fault_policy(group);

	return -ETIMEDOUT;
}

int motor_group_enable_async(struct motor_group *group)
{
	struct motor_group_enable_async_ctx *slot;
	int ret;
	k_spinlock_key_t key;

	if ((group == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	ret = group_enable_preflight(group);
	if (ret != 0) {
		return ret;
	}

	slot = alloc_enable_async_slot();
	if (slot == NULL) {
		for (uint8_t j = 0; j < group->count; j++) {
			motor_estop(group->members[j]);
		}
		key = k_spin_lock(&group->lock);
		group->state = MOTOR_GROUP_FAULT;
		k_spin_unlock(&group->lock, key);
		return -EBUSY;
	}

	slot->group = group;
	slot->deadline = k_uptime_ticks() + (int64_t)k_ms_to_ticks_ceil32(30000);

	(void)k_work_reschedule(&slot->work, K_MSEC(1));

	return 0;
}

int motor_group_disable(struct motor_group *group, k_timeout_t timeout)
{
	int64_t deadline = deadline_from_timeout(timeout);

	if ((group == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	motor_group_release_async_slot(group);

	k_spinlock_key_t key = k_spin_lock(&group->lock);

	group->state = MOTOR_GROUP_STOPPING;
	k_spin_unlock(&group->lock, key);

	for (uint8_t i = 0; i < group->count; i++) {
		(void)motor_disable(group->members[i]);
	}

	while (k_uptime_ticks() < deadline) {
		if (members_all_in_state(group, MOTOR_STATE_IDLE)) {
			key = k_spin_lock(&group->lock);
			group->state = MOTOR_GROUP_IDLE;
			k_spin_unlock(&group->lock, key);
			return 0;
		}
		k_sleep(K_MSEC(1));
	}

	return -ETIMEDOUT;
}

void motor_group_estop(struct motor_group *group)
{
	k_spinlock_key_t key;

	if (group == NULL) {
		return;
	}

	motor_group_release_async_slot(group);

	key = k_spin_lock(&group->lock);

	for (uint8_t i = 0; i < group->count; i++) {
		motor_estop(group->members[i]);
	}

	group->state = MOTOR_GROUP_IDLE;
	k_spin_unlock(&group->lock, key);
}

int motor_group_clear_fault(struct motor_group *group)
{
	int ret;

	if ((group == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < group->count; i++) {
		ret = motor_clear_fault(group->members[i]);
		if (ret != 0) {
			return ret;
		}
	}

	k_spinlock_key_t key = k_spin_lock(&group->lock);

	if (group->state == MOTOR_GROUP_FAULT) {
		group->state = MOTOR_GROUP_IDLE;
	}
	k_spin_unlock(&group->lock, key);

	return 0;
}

typedef int (*motor_group_apply_fn)(motor_t m, uint8_t idx, void *ctx);

static int group_require_run(struct motor_group *group)
{
	k_spinlock_key_t key = k_spin_lock(&group->lock);

	if (group->state != MOTOR_GROUP_RUN) {
		k_spin_unlock(&group->lock, key);
		return -ENOEXEC;
	}

	k_spin_unlock(&group->lock, key);

	if (!members_all_in_state(group, MOTOR_STATE_RUN)) {
		return -ENOEXEC;
	}

	return 0;
}

/* Apply fn to every member, return the first non-zero result. */
static int group_for_each(const struct motor_group *group, motor_group_apply_fn fn, void *ctx)
{
	int first_err = 0;

	for (uint8_t i = 0; i < group->count; i++) {
		int r = fn(group->members[i], i, ctx);

		if ((r != 0) && (first_err == 0)) {
			first_err = r;
		}
	}

	return first_err;
}

static int apply_set_torque(motor_t m, uint8_t idx, void *ctx)
{
	const float *torques = ctx;

	return motor_set_torque(m, torques[idx]);
}

int motor_group_set_torque(struct motor_group *group, const float *torques)
{
	int ret;

	if ((group == NULL) || (torques == NULL) || (group->count == 0U)) {
		return -EINVAL;
	}

	ret = group_require_run(group);
	if (ret != 0) {
		return ret;
	}

	return group_for_each(group, apply_set_torque, (void *)torques);
}

static int apply_set_drive_mode(motor_t m, uint8_t idx, void *ctx)
{
	enum motor_drive_mode mode = *(enum motor_drive_mode *)ctx;

	ARG_UNUSED(idx);
	return motor_set_drive_mode(m, mode);
}

int motor_group_set_drive_mode(struct motor_group *group, enum motor_drive_mode mode)
{
	int ret;

	if (group == NULL) {
		return -EINVAL;
	}

	ret = group_require_run(group);
	if (ret != 0) {
		return ret;
	}

	return group_for_each(group, apply_set_drive_mode, &mode);
}

void motor_group_get_status(struct motor_group *group, enum motor_group_state *state,
			    uint32_t *fault_mask)
{
	uint32_t fm;
	enum motor_group_state st;

	if (group == NULL) {
		return;
	}

	fm = member_fault_mask(group);

	k_spinlock_key_t key = k_spin_lock(&group->lock);

	st = group->state;
	k_spin_unlock(&group->lock, key);

	if (fm != 0U) {
		st = MOTOR_GROUP_FAULT;
	}

	if (fault_mask != NULL) {
		*fault_mask = fm;
	}

	if (state != NULL) {
		*state = st;
	}
}
