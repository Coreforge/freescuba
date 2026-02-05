// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Wrapper around Mercury's parametric hand code, used by Index and OpenGloves to simulate hand tracking.
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @ingroup aux_util
 */

#pragma once

//#include "xrt/xrt_defines.h"

//#include "util/u_misc.h"
//#include "util/u_hand_tracking.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// structs (and defines) from elsewhere in monado
#define XRT_HAND_JOINT_COUNT 26
struct xrt_space_relation{
	struct pose{
		glm::vec3 position;
		glm::quat orientation;
	} pose;
};

enum xrt_hand{
	XRT_HAND_LEFT,
	XRT_HAND_RIGHT,
};

enum xrt_hand_joint
{
	XRT_HAND_JOINT_PALM = 0,
	XRT_HAND_JOINT_WRIST = 1,
	XRT_HAND_JOINT_THUMB_METACARPAL = 2,
	XRT_HAND_JOINT_THUMB_PROXIMAL = 3,
	XRT_HAND_JOINT_THUMB_DISTAL = 4,
	XRT_HAND_JOINT_THUMB_TIP = 5,
	XRT_HAND_JOINT_INDEX_METACARPAL = 6,
	XRT_HAND_JOINT_INDEX_PROXIMAL = 7,
	XRT_HAND_JOINT_INDEX_INTERMEDIATE = 8,
	XRT_HAND_JOINT_INDEX_DISTAL = 9,
	XRT_HAND_JOINT_INDEX_TIP = 10,
	XRT_HAND_JOINT_MIDDLE_METACARPAL = 11,
	XRT_HAND_JOINT_MIDDLE_PROXIMAL = 12,
	XRT_HAND_JOINT_MIDDLE_INTERMEDIATE = 13,
	XRT_HAND_JOINT_MIDDLE_DISTAL = 14,
	XRT_HAND_JOINT_MIDDLE_TIP = 15,
	XRT_HAND_JOINT_RING_METACARPAL = 16,
	XRT_HAND_JOINT_RING_PROXIMAL = 17,
	XRT_HAND_JOINT_RING_INTERMEDIATE = 18,
	XRT_HAND_JOINT_RING_DISTAL = 19,
	XRT_HAND_JOINT_RING_TIP = 20,
	XRT_HAND_JOINT_LITTLE_METACARPAL = 21,
	XRT_HAND_JOINT_LITTLE_PROXIMAL = 22,
	XRT_HAND_JOINT_LITTLE_INTERMEDIATE = 23,
	XRT_HAND_JOINT_LITTLE_DISTAL = 24,
	XRT_HAND_JOINT_LITTLE_TIP = 25,
	XRT_HAND_JOINT_MAX_ENUM = 0x7FFFFFFF
};

struct xrt_hand_joint_value
{
	struct xrt_space_relation relation;
	float radius;
};

struct xrt_hand_joint_set
{
	union {
		struct xrt_hand_joint_value hand_joint_set_default[XRT_HAND_JOINT_COUNT];
	} values;

	// in driver global space, without tracking_origin offset
	struct xrt_space_relation hand_pose;
	bool is_active;
};

struct u_hand_tracking_finger_value
{
	float splay;

	float joint_curls[4];
	int joint_count;
};

struct u_hand_tracking_values
{
	struct u_hand_tracking_finger_value little;
	struct u_hand_tracking_finger_value ring;
	struct u_hand_tracking_finger_value middle;
	struct u_hand_tracking_finger_value index;
	struct u_hand_tracking_finger_value thumb;
};

// end other structs

struct u_hand_sim_metacarpal
{
	glm::vec2 swing;
	float twist;
};

struct u_hand_sim_finger
{
	struct u_hand_sim_metacarpal metacarpal;
	glm::vec2 proximal_swing;
	// rotation at intermediate joint, then rotation at distal joint
	float rotations[2];
};

struct u_hand_sim_thumb
{
	struct u_hand_sim_metacarpal metacarpal;
	float rotations[2];
};

struct u_hand_sim_hand
{
	// Distance from wrist to middle-proximal.
	bool is_right;
	float hand_size;
	struct xrt_space_relation wrist_pose;
	struct xrt_space_relation hand_pose;

	struct u_hand_sim_thumb thumb;
	struct u_hand_sim_finger finger[4];
};


void
u_hand_sim_simulate(struct u_hand_sim_hand *hand, struct xrt_hand_joint_set *out_set);

void
u_hand_sim_simulate_for_valve_index_knuckles(const struct u_hand_tracking_curl_values *values,
                                             enum xrt_hand xhand,
                                             const struct xrt_space_relation *root_pose,
                                             struct xrt_hand_joint_set *out_set);

void
u_hand_sim_simulate_generic(const struct u_hand_tracking_values *values,
                            enum xrt_hand xhand,
                            const struct xrt_space_relation *root_pose,
                            struct xrt_hand_joint_set *out_set);

#ifdef __cplusplus
}
#endif
