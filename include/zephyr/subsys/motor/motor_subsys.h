/*
 * Copyright (c) 2026 Zephyr Motor Control Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_SUBSYS_H_
#define ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_SUBSYS_H_

#include <zephyr/subsys/motor/motor.h>
#include <zephyr/subsys/motor/motor_dt.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Control Subsystem — Multi-instance and Group API
 * @defgroup motor_subsys Motor Subsystem
 * @ingroup motor_control
 * @{
 *
 * This header is the top of the motor control stack.  It provides:
 *
 *   1. Subsystem-level init — all motor instances declared via DT
 *      are initialised automatically at SYS_INIT() time.
 *
 *   2. Instance discovery — application retrieves a motor_t handle
 *      by DT node label, index, or device pointer; no manual wiring.
 *
 *   3. Motor groups — a named set of motor instances that must be
 *      commanded, enabled, disabled, and fault-stopped together.
 *      Designed for: quadrotors (4 motors), dual-axis gantry (2),
 *      3-phase spindle + conveyor (mixed), etc.
 *
 *   4. Group enable barrier — all motors in a group start their
 *      alignment sequences in parallel; the group transitions to
 *      RUN only when every member reports MOTOR_STATE_RUN.
 *
 *   5. Atomic group commands — motor_group_set_torque() writes
 *      setpoints to all members before any current loop fires the
 *      next actuation, eliminating inter-motor skew.
 *
 *   6. Fault propagation policy — one member entering FAULT can
 *      trigger group_estop() on all siblings within one supervision
 *      tick (configurable via Kconfig).
 *
 *   7. Shared resource arbitration — if two motor instances share
 *      an ADC or timer peripheral, the subsystem serialises their
 *      update() calls using a per-resource spinlock.
 *
 * Quadrotor usage sketch:
 *
 *   MOTOR_GROUP_DEFINE(rotors, 4);
 *
 *   void app_main(void) {
 *       motor_t m[4];
 *       m[0] = motor_subsys_get_by_label(DT_NODELABEL(motor0));
 *       m[1] = motor_subsys_get_by_label(DT_NODELABEL(motor1));
 *       m[2] = motor_subsys_get_by_label(DT_NODELABEL(motor2));
 *       m[3] = motor_subsys_get_by_label(DT_NODELABEL(motor3));
 *
 *       motor_group_add(&rotors, m, 4);
 *       motor_group_self_test(&rotors, &faults);
 *       motor_group_enable(&rotors);            // parallel align + RUN
 *
 *       float thrust[4] = {T0, T1, T2, T3};
 *       motor_group_set_torque(&rotors, thrust); // every 2.5 ms
 *   }
 */

/* ------------------------------------------------------------------ */
/* Kconfig-driven compile-time limits                                  */
/* ------------------------------------------------------------------ */

/** Maximum number of motor instances the subsystem tracks.
 *  Set by CONFIG_MOTOR_MAX_INSTANCES (default 8). */
#ifndef CONFIG_MOTOR_MAX_INSTANCES
#define CONFIG_MOTOR_MAX_INSTANCES 8
#endif

/** Maximum number of motor groups.
 *  Set by CONFIG_MOTOR_MAX_GROUPS (default 4). */
#ifndef CONFIG_MOTOR_MAX_GROUPS
#define CONFIG_MOTOR_MAX_GROUPS 4
#endif

/** Maximum members per group.
 *  Set by CONFIG_MOTOR_GROUP_MAX_MEMBERS (default 8). */
#ifndef CONFIG_MOTOR_GROUP_MAX_MEMBERS
#define CONFIG_MOTOR_GROUP_MAX_MEMBERS 8
#endif

/* ------------------------------------------------------------------ */
/* Subsystem registration — one entry per DT motor-controller node    */
/* ------------------------------------------------------------------ */

/**
 * @brief Per-instance subsystem entry.
 *
 * Created by MOTOR_SUBSYS_DEFINE() and placed in a linker iterable
 * section so the subsystem init code can walk all instances without
 * a manual registration call.
 */
struct motor_subsys_entry {
	/** Pointer to the controller instance (storage from MOTOR_CTRL_DEFINE). */
	struct motor_ctrl *ctrl;

	/** Sensor backend device (from DT sensor phandle). */
	const struct device *sensor;

	/** Power stage backend device (from DT actuator phandle). */
	const struct device *actuator;

	/** Algorithm vtable pointer (resolved from DT algorithm property). */
	const struct motor_algo_ops *algo;

	/** Algorithm private state buffer. */
	void *algo_data;

	/**
	 * Initial parameters — from @ref MOTOR_SUBSYS_DEFINE_DT (Devicetree) or
	 * supplied explicitly; must not be NULL.
	 */
	const struct motor_ctrl_params *params;

	/** DT node identifier string (e.g. "motor0") — for shell/debug. */
	const char *label;
};

/**
 * @brief Declare and register a motor instance with the subsystem.
 *
 * Places a motor_subsys_entry in the iterable section
 * "motor_subsys_entries", walked by motor_subsys_init().
 *
 * @param _name      C identifier for the entry (unique per TU).
 * @param _ctrl      Storage: struct motor_ctrl instance (from MOTOR_CTRL_DEFINE).
 * @param _sensor    Sensor backend device pointer.
 * @param _actuator  Power stage backend device pointer.
 * @param _algo      Algorithm vtable pointer.
 * @param _algo_data Algorithm private state pointer.
 * @param _params    Pointer to @ref motor_ctrl_params (required).
 * @param _label     DT label string.
 */
#define MOTOR_SUBSYS_DEFINE(_name, _ctrl, _sensor, _actuator, _algo, _algo_data, _params, _label)  \
	static STRUCT_SECTION_ITERABLE(motor_subsys_entry, _name) = {                              \
		.ctrl = (_ctrl),                                                                   \
		.sensor = (_sensor),                                                               \
		.actuator = (_actuator),                                                           \
		.algo = (_algo),                                                                   \
		.algo_data = (_algo_data),                                                         \
		.params = (_params),                                                               \
		.label = (_label),                                                                 \
	}

/**
 * @brief Register a motor-controller instance from Devicetree (dc-torque algorithm).
 *
 * Expands @ref motor_ctrl_params and @ref motor_algo_dc_torque_data from the
 * `zephyr,motor-controller` node using @ref MOTOR_CTRL_PARAMS_INITIALIZER and
 * @ref MOTOR_DC_TORQUE_DATA_INITIALIZER — no application-side parameter structs.
 *
 * @param _nodelabel  DT node label (unquoted), e.g. motor_brushed.
 * @param _ctrl       Pre-declared @ref motor_ctrl instance (MOTOR_CTRL_DEFINE).
 */
#define MOTOR_SUBSYS_DEFINE_DT(_nodelabel, _ctrl)                                                  \
	static const struct motor_ctrl_params UTIL_CAT(_motor_params_, _nodelabel) =                \
		MOTOR_CTRL_PARAMS_INITIALIZER(DT_NODELABEL(_nodelabel));                             \
	static struct motor_algo_dc_torque_data UTIL_CAT(_motor_dc_, _nodelabel) =                   \
		MOTOR_DC_TORQUE_DATA_INITIALIZER(DT_NODELABEL(_nodelabel));                           \
	MOTOR_SUBSYS_DEFINE(motor_entry_##_nodelabel, (_ctrl),                                     \
			    DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(_nodelabel), sensor)),           \
			    DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(_nodelabel), actuator)),         \
			    &motor_algo_dc_torque, &UTIL_CAT(_motor_dc_, _nodelabel),                 \
			    &UTIL_CAT(_motor_params_, _nodelabel), STRINGIFY(_nodelabel))

/* ------------------------------------------------------------------ */
/* Subsystem init and instance discovery                               */
/* ------------------------------------------------------------------ */

/**
 * @brief Initialise the motor subsystem.
 *
 * Walks all MOTOR_SUBSYS_DEFINE entries in the iterable section,
 * calls motor_init() for each, and makes them discoverable.
 *
 * Called automatically by SYS_INIT() at APPLICATION level if
 * CONFIG_MOTOR_SUBSYS_AUTO_INIT=y (default).
 * May also be called explicitly before use.
 *
 * @retval 0 All instances initialised.
 * @retval negative errno if any instance fails to init.
 */
int motor_subsys_init(void);

/**
 * @brief Get a motor handle by DT node label string.
 *
 * @param label  DT node label string, e.g. "motor0".
 * @return motor_t handle, or NULL if not found.
 */
motor_t motor_subsys_get_by_label(const char *label);

/**
 * @brief Get a motor handle by subsystem index.
 *
 * Index order matches the order of MOTOR_SUBSYS_DEFINE declarations
 * in the linker iterable section (typically link order).
 *
 * @param index  Zero-based instance index.
 * @return motor_t handle, or NULL if index out of range.
 */
motor_t motor_subsys_get_by_index(uint8_t index);

/**
 * @brief Return the total number of registered motor instances.
 */
uint8_t motor_subsys_count(void);

/* ------------------------------------------------------------------ */
/* Motor group                                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief Fault propagation policy for a group.
 */
enum motor_group_fault_policy {
	/** Any member fault triggers group_estop() on all members. */
	MOTOR_GROUP_FAULT_ESTOP_ALL = 0,

	/** Any member fault triggers group_disable() (controlled stop). */
	MOTOR_GROUP_FAULT_DISABLE_ALL = 1,

	/** Fault is isolated — other members continue running.
	 *  Application receives fault_cb and decides. */
	MOTOR_GROUP_FAULT_ISOLATE = 2,
};

/**
 * @brief Motor group state.
 */
enum motor_group_state {
	MOTOR_GROUP_IDLE = 0,
	MOTOR_GROUP_ALIGNING = 1, /* one or more members still in ALIGN */
	MOTOR_GROUP_RUN = 2,      /* all members in RUN                 */
	MOTOR_GROUP_STOPPING = 3, /* disable in progress                */
	MOTOR_GROUP_FAULT = 4,    /* one or more members in FAULT       */
};

struct motor_group;

/**
 * @brief Group-level state/fault callback.
 *
 * @param group      Pointer to the group.
 * @param state      New group state.
 * @param fault_mask Bitmask of which member indices have faulted.
 * @param user_data  Registered pointer.
 */
typedef void (*motor_group_cb_t)(struct motor_group *group, enum motor_group_state state,
				 uint32_t fault_mask, void *user_data);

/**
 * @brief Motor group instance.
 *
 * Statically allocated via MOTOR_GROUP_DEFINE().
 */
struct motor_group {
	motor_t members[CONFIG_MOTOR_GROUP_MAX_MEMBERS];
	uint8_t count;

	enum motor_group_state state;
	enum motor_group_fault_policy fault_policy;

	motor_group_cb_t cb;
	void *cb_user_data;

	/** Semaphore used by group_enable barrier:
	 *  given once per member reaching RUN state. */
	struct k_sem align_done;

	/** Spinlock for group-level atomic operations. */
	struct k_spinlock lock;

	const char *name;
};

/**
 * @brief Statically define a motor group.
 *
 * @param _name  C identifier for the group struct.
 * @param _label String name for debug/shell.
 */
#define MOTOR_GROUP_DEFINE(_name, _label)                                                          \
	static struct motor_group _name = {                                                        \
		.count = 0,                                                                        \
		.state = MOTOR_GROUP_IDLE,                                                         \
		.fault_policy = MOTOR_GROUP_FAULT_ESTOP_ALL,                                       \
		.name = (_label),                                                                  \
	}

/* ------------------------------------------------------------------ */
/* Group lifecycle API                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Add motor instances to a group.
 *
 * Must be called before any group operation.
 * All motors must be in IDLE state.
 *
 * @param group    Group instance.
 * @param motors   Array of motor_t handles.
 * @param count    Number of motors to add.
 * @param policy   Fault propagation policy.
 * @retval 0 on success, -ENOMEM if group capacity exceeded.
 */
int motor_group_add(struct motor_group *group, motor_t *motors, uint8_t count,
		    enum motor_group_fault_policy policy);

/**
 * @brief Register a group-level state/fault callback.
 *
 * @param group      Group instance.
 * @param cb         Callback (called from supervision thread context).
 * @param user_data  Passed to cb.
 */
void motor_group_register_cb(struct motor_group *group, motor_group_cb_t cb, void *user_data);

/**
 * @brief Run self-test on all group members sequentially (blocking).
 *
 * @param group   Group instance.
 * @param faults  Output array, one uint32_t per member (fault bitmask).
 *                Caller must provide at least group->count entries.
 * @retval 0 All members pass. Negative errno on hardware failure.
 */
int motor_group_self_test(struct motor_group *group, uint32_t *faults);

/**
 * @brief Enable all group members (parallel alignment, RUN barrier).
 *
 * Calls motor_enable() on all members simultaneously.
 * Blocks until every member has reached MOTOR_STATE_RUN or any
 * member enters MOTOR_STATE_FAULT.
 *
 * Non-blocking variant: motor_group_enable_async() — returns
 * immediately; group_cb fires when all are running.
 *
 * @param group    Group instance.
 * @param timeout  Maximum time to wait for all to reach RUN.
 * @retval 0 All running. -ETIMEDOUT if timeout elapsed.
 *         -EFAULT if any member entered FAULT state.
 */
int motor_group_enable(struct motor_group *group, k_timeout_t timeout);

/**
 * @brief Async variant — returns immediately, group_cb fires on completion.
 */
int motor_group_enable_async(struct motor_group *group);

/**
 * @brief Controlled stop on all group members (parallel deceleration).
 *
 * Calls motor_disable() on all members, waits for all to reach IDLE.
 *
 * @param group    Group instance.
 * @param timeout  Maximum wait time.
 * @retval 0 All stopped. -ETIMEDOUT if timeout elapsed.
 */
int motor_group_disable(struct motor_group *group, k_timeout_t timeout);

/**
 * @brief Immediately cut output on all group members.
 *
 * Safe to call from any context including ISR.
 * Calls motor_estop() on every member atomically under spinlock.
 *
 * @param group  Group instance.
 */
void motor_group_estop(struct motor_group *group);

/**
 * @brief Clear fault on all group members and return to IDLE.
 *
 * @param group  Group instance.
 * @retval 0 All cleared. -EBUSY if any fault cause still present.
 */
int motor_group_clear_fault(struct motor_group *group);

/* ------------------------------------------------------------------ */
/* Group command API — atomic multi-motor setpoint writes             */
/* ------------------------------------------------------------------ */

/**
 * @brief Write torque setpoints to all group members atomically.
 *
 * Writes are batched under the group spinlock and applied before the
 * next current-loop ISR fires on any member.
 *
 * All members must be in MOTOR_STATE_RUN.
 *
 * @param group    Group instance.
 * @param torques  Array of torque values (N·m), length = group->count.
 *                 Positive = forward. Index matches member add order.
 * @retval 0 on success. -ENOEXEC if group not in RUN state.
 */
int motor_group_set_torque(struct motor_group *group, const float *torques);

/**
 * @brief Set drive mode on all group members.
 *
 * @param group  Group instance.
 * @param mode   Drive mode to apply to all members.
 * @retval 0 on success. -ENOTSUP if any member does not support mode.
 */
int motor_group_set_drive_mode(struct motor_group *group, enum motor_drive_mode mode);

/* ------------------------------------------------------------------ */
/* Group status                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Read group state and per-member fault summary.
 *
 * @param group       Group instance.
 * @param state       Output: group state.
 * @param fault_mask  Output: bitmask of which member indices are faulted.
 */
void motor_group_get_status(struct motor_group *group, enum motor_group_state *state,
			    uint32_t *fault_mask);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SUBSYS_MOTOR_MOTOR_SUBSYS_H_ */
