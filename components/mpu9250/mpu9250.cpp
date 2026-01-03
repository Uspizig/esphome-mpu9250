#include "mpu9250.h"
#include <cmath>

namespace esphome {
namespace mpu9250 {

static const uint8_t PWR_MGMT_1 = 0x6B;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t GYRO_XOUT_H = 0x43;

static const uint8_t AK8963_ADDR = 0x0C;
static const uint8_t AK8963_ST1 = 0x02;
static const uint8_t AK8963_XOUT_L = 0x03;
static const uint8_t AK8963_CNTL1 = 0x0A;

void MPU9250Component::setup() {
  write_byte(PWR_MGMT_1, 0x00);
  write_byte(0x37, 0x02);
  parent_->write_byte(AK8963_ADDR, AK8963_CNTL1, 0x16);
}

void MPU9250Component::set_calibrate_button(button::Button *b) {
  calib_btn_ = b;
  calib_btn_->add_on_press_callback([this]() { start_calibration(); });
}

void MPU9250Component::start_calibration() {
  calibrating_ = true;
  calib_start_ = millis();
  for (int i = 0; i < 3; i++) {
    mag_min_[i] =  9999;
    mag_max_[i] = -9999;
  }
}

void MPU9250Component::update() {
  uint8_t b[6];

  if (read_bytes(ACCEL_XOUT_H, b, 6) == i2c::ERROR_OK) {
    ax_ = ((b[0]<<8)|b[1]) / 16384.0f;
    ay_ = ((b[2]<<8)|b[3]) / 16384.0f;
    az_ = ((b[4]<<8)|b[5]) / 16384.0f;
    if (ax_s_) ax_s_->publish_state(ax_*9.81f);
    if (ay_s_) ay_s_->publish_state(ay_*9.81f);
    if (az_s_) az_s_->publish_state(az_*9.81f);
  }

  if (read_bytes(GYRO_XOUT_H, b, 6) == i2c::ERROR_OK) {
    gx_ = ((b[0]<<8)|b[1]) / 131.0f;
    gy_ = ((b[2]<<8)|b[3]) / 131.0f;
    gz_ = ((b[4]<<8)|b[5]) / 131.0f;
    if (gx_s_) gx_s_->publish_state(gx_);
    if (gy_s_) gy_s_->publish_state(gy_);
    if (gz_s_) gz_s_->publish_state(gz_);
  }

  uint8_t st;
  if (parent_->read_byte(AK8963_ADDR, AK8963_ST1, &st) == i2c::ERROR_OK && (st & 1)) {
    uint8_t m[7];
    parent_->read_bytes(AK8963_ADDR, AK8963_XOUT_L, m, 7);
    mx_ = ((m[1]<<8)|m[0]) * 0.15f;
    my_ = ((m[3]<<8)|m[2]) * 0.15f;
    mz_ = ((m[5]<<8)|m[4]) * 0.15f;

    if (calibrating_) {
      mag_min_[0]=std::min(mag_min_[0],mx_);
      mag_min_[1]=std::min(mag_min_[1],my_);
      mag_min_[2]=std::min(mag_min_[2],mz_);
      mag_max_[0]=std::max(mag_max_[0],mx_);
      mag_max_[1]=std::max(mag_max_[1],my_);
      mag_max_[2]=std::max(mag_max_[2],mz_);
      if (millis()-calib_start_ > 30000) {
        calibrating_=false;
        for (int i=0;i<3;i++)
          mag_offset_[i]=(mag_max_[i]+mag_min_[i])/2;
      }
    }

    mx_ -= mag_offset_[0];
    my_ -= mag_offset_[1];
    mz_ -= mag_offset_[2];

    if (mx_s_) mx_s_->publish_state(mx_);
    if (my_s_) my_s_->publish_state(my_);
    if (mz_s_) mz_s_->publish_state(mz_);
  }

  uint32_t now = millis();
  float dt = (now-last_update_)/1000.0f;
  last_update_ = now;

  if (use_madgwick_ && heading_s_) {
    filter_.update(gx_,gy_,gz_,ax_,ay_,az_,mx_,my_,mz_,dt);
    float yaw = filter_.get_yaw() + declination_;
    if (yaw<0) yaw+=360;
    if (yaw>=360) yaw-=360;
    heading_s_->publish_state(yaw);
  }
}
}  // namespace mpu9250
}  // namespace esphome
