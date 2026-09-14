#ifndef __HEXA_CONFIG_H
#define __HEXA_CONFIG_H

/* 连杆长度，单位 mm（与 firmware/include/config.h 保持一致） */
#define HEXA_LEG_ROOT_TO_J1      (19.4f)
#define HEXA_LEG_J1_TO_J2        (32.0f)
#define HEXA_LEG_J2_TO_J3        (43.8f)
#define HEXA_LEG_J3_TO_TIP       (90.05f)

/* 安装位置 */
#define HEXA_MOUNT_LR_X          (34.7f)
#define HEXA_MOUNT_OTHER_X       (25.0f)
#define HEXA_MOUNT_OTHER_Y       (58.0f)

/* 时序，单位 ms */
#define HEXA_MOVEMENT_INTERVAL   (20)
#define HEXA_MOVEMENT_SWITCH_DUR (150)

/* 速度 */
#define HEXA_DEFAULT_SPEED       (0.5f)
#define HEXA_MIN_SPEED           (0.25f)
#define HEXA_MAX_SPEED           (1.0f)

#endif