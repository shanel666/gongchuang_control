#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H

/* Compile-time route configuration. Coordinates use the path planner grid. */
#define PATH_START_X        2250u
#define PATH_START_Y        2250u
#define PATH_TARGET_X       1200u
#define PATH_TARGET_Y       150u

/* Initial heading: 1=north (+Y), 2=west (-X), 3=south (-Y), 4=east (+X). */
#define PATH_INITIAL_DIR    1u

/* List obstacle grid points here before building the firmware. */
#define PATH_OBSTACLE_COUNT 4u
#define PATH_OBSTACLE_0_X   675u
#define PATH_OBSTACLE_0_Y   1725u
#define PATH_OBSTACLE_1_X   1725u
#define PATH_OBSTACLE_1_Y   1725u
#define PATH_OBSTACLE_2_X   675u
#define PATH_OBSTACLE_2_Y   625u
#define PATH_OBSTACLE_3_X   1725u
#define PATH_OBSTACLE_3_Y   625u
#define PATH_OBSTACLE_4_X   0u
#define PATH_OBSTACLE_4_Y   0u
#define PATH_OBSTACLE_5_X   0u
#define PATH_OBSTACLE_5_Y   0u
#define PATH_OBSTACLE_6_X   0u
#define PATH_OBSTACLE_6_Y   0u
#define PATH_OBSTACLE_7_X   0u
#define PATH_OBSTACLE_7_Y   0u

#if PATH_INITIAL_DIR < 1 || PATH_INITIAL_DIR > 4
#error "PATH_INITIAL_DIR must be 1, 2, 3 or 4"
#endif
#if PATH_OBSTACLE_COUNT > 8
#error "PATH_OBSTACLE_COUNT must not exceed 8"
#endif

#endif /* PATH_CONFIG_H */
