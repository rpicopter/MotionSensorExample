/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu_dmp_motion_driver.h
 *      @brief      DMP image and interface functions.
 *      @details    All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */

#ifndef _INV_MPU_DMP_MOTION_DRIVER_H_
#define _INV_MPU_DMP_MOTION_DRIVER_H_

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

/* Set up functions. */
uint8_t dmp_load_motion_driver_firmware(void);
uint8_t dmp_set_fifo_rate(uint16_t rate);
uint8_t dmp_get_fifo_rate(uint16_t *rate);
uint8_t dmp_enable_feature(uint16_t mask);
uint8_t dmp_get_enabled_features(uint16_t *mask);
uint8_t dmp_set_interrupt_mode(uint8_t mode);
uint8_t dmp_set_orientation(uint16_t orient);
uint8_t dmp_set_gyro_bias(int32_t *bias);
uint8_t dmp_set_accel_bias(int32_t *bias);

/* Tap functions. */
uint8_t dmp_register_tap_cb(void (*func)(uint8_t, uint8_t));
uint8_t dmp_set_tap_thresh(uint8_t axis, uint16_t thresh);
uint8_t dmp_set_tap_axes(uint8_t axis);
uint8_t dmp_set_tap_count(uint8_t min_taps);
uint8_t dmp_set_tap_time(uint16_t time);
uint8_t dmp_set_tap_time_multi(uint16_t time);
uint8_t dmp_set_shake_reject_thresh(int32_t sf, uint16_t thresh);
uint8_t dmp_set_shake_reject_time(uint16_t time);
uint8_t dmp_set_shake_reject_timeout(uint16_t time);

/* Android orientation functions. */
uint8_t dmp_register_android_orient_cb(void (*func)(uint8_t));

/* LP quaternion functions. */
uint8_t dmp_enable_lp_quat(uint8_t enable);
uint8_t dmp_enable_6x_lp_quat(uint8_t enable);

/* Pedometer functions. */
uint8_t dmp_get_pedometer_step_count(uint32_t *count);
uint8_t dmp_set_pedometer_step_count(uint32_t count);
uint8_t dmp_get_pedometer_walk_time(uint32_t *time);
uint8_t dmp_set_pedometer_walk_time(uint32_t time);

/* DMP gyro calibration functions. */
uint8_t dmp_enable_gyro_cal(uint8_t enable);

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */
uint8_t dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, int16_t *sensors, uint8_t *more);

#endif  /* _INV_MPU_DMP_MOTION_DRIVER_H_ */

