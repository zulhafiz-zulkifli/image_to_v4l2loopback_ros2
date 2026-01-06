/**
 * Copyright (c) 
 * 2013, Zhiwei Chu
 * 2015, mayfieldrobotics
 */

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <image_to_v4l2loopback/image_converter.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#define ROUND_UP_2(n) (((n) + 1) & ~1)

bool ImageConverter::is_supported(uint32_t fourcc) {
  return (fourcc == V4L2_PIX_FMT_BGR24 ||
          fourcc == V4L2_PIX_FMT_RGB24 ||
          fourcc == V4L2_PIX_FMT_GREY  ||
          fourcc == V4L2_PIX_FMT_YVU420||
          fourcc == V4L2_PIX_FMT_YUYV ||
          fourcc == V4L2_PIX_FMT_UYVY);
}

bool ImageConverter::is_supported(const std::string &fourcc) {
  return fourcc.size() <= 4 && is_supported(_fourcc_code(fourcc));
}

ImageConverter::ImageConverter(uint32_t width, uint32_t height,
                               const std::string &fourcc,
                               const rclcpp::Logger &logger)
  : width_(width),
    height_(height),
    fourcc_(_fourcc_code(fourcc)),
    logger_(logger)
{
  switch (fourcc_) {
  case V4L2_PIX_FMT_BGR24:
    param_bgr24();
    break;
  case V4L2_PIX_FMT_RGB24:
    param_rgb24();
    break;
  case V4L2_PIX_FMT_GREY:
    param_grey();
    break;
  case V4L2_PIX_FMT_YVU420:
    param_yvu420();
    break;
  case V4L2_PIX_FMT_YUYV:
    param_yuyv();
    break;
  case V4L2_PIX_FMT_UYVY:
    param_uyvy();
    break;
  default:
    std::stringstream ss;
    ss << "Unsupported fourcc=" << fourcc_ << ".";
    throw std::invalid_argument(ss.str());
  }
}

v4l2_format ImageConverter::format() const {
  v4l2_format format;
  format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  format.fmt.pix.field = V4L2_FIELD_NONE;
  format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
  format.fmt.pix.width = width_;
  format.fmt.pix.height = height_;
  format.fmt.pix.pixelformat = fourcc_;
  format.fmt.pix.bytesperline = bytes_per_line_;
  format.fmt.pix.sizeimage = size_;
  return format;
}

bool ImageConverter::convert(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg,
    ImageConverter::Buffer &buffer)
{
  // ---- PASS-THROUGH IF ALREADY YUV422 ----
  if (msg->encoding == "yuyv" ||
      msg->encoding == "yuv422" ||
      msg->encoding == "yuv422_yuy2" ||
      msg->encoding == "yuv422_yuyv" ||
      msg->encoding == "yuv422_uyvy" ||
      msg->encoding == "uyvy") {

    buffer.resize(msg->data.size());
    memcpy(&buffer[0], msg->data.data(), msg->data.size());
    return true;
  }

  // ---- OTHERWISE USE OPENCV ----
  cv_bridge::CvImagePtr cv_msg =
    cv_bridge::toCvCopy(msg, cv_copy_encoding_);

  if (!cv_msg) {
    RCLCPP_INFO(logger_,
      "failed to copy sensor image '%s' to cv image '%s'",
      msg->encoding.c_str(), cv_copy_encoding_.c_str());
    return false;
  }

  cv::Mat cv_image;
  cv::swap(cv_msg->image, cv_image);

  if (msg->width != width_ || msg->height != height_) {
    cv::Mat resized;
    cv::resize(cv_image, resized, cv::Size(width_, height_));
    cv::swap(cv_image, resized);
  }

  if (cv_color_) {
    cv::Mat converted;
    cv::cvtColor(cv_image, converted, cv_color_code_,
                 cv_color_channels_);
    cv::swap(cv_image, converted);
  }

  buffer.resize(size_);
  ((*this).*fmt_)(cv_image, buffer);

  return true;
}

bool ImageConverter::operator()(
  const sensor_msgs::msg::Image::ConstSharedPtr &msg,
  ImageConverter::Buffer &buffer)
{
  return convert(msg, buffer);
}

uint32_t ImageConverter::_fourcc_code(const std::string &fourcc) {
  if (fourcc.size() > 4)
    throw std::runtime_error("Invalid fourcc string");

  std::stringstream ss;
  ss << std::setw(4) << fourcc;
  std::string p = ss.str();
  return v4l2_fourcc(p[0], p[1], p[2], p[3]);
}

// -------- BGR24 --------

void ImageConverter::param_bgr24() {
  RCLCPP_INFO(logger_, "bgr24");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 3;
  fmt_ = &ImageConverter::fmt_bgr24;
}

void ImageConverter::fmt_bgr24(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c) {
      cv::Vec3b p = image.at<cv::Vec3b>(r, c);
      *(b++) = p[0];
      *(b++) = p[1];
      *(b++) = p[2];
    }
}

// -------- RGB24 --------

void ImageConverter::param_rgb24() {
  RCLCPP_INFO(logger_, "rgb24");
  cv_copy_encoding_ = sensor_msgs::image_encodings::RGB8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 3;
  fmt_ = &ImageConverter::fmt_rgb24;
}

void ImageConverter::fmt_rgb24(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c) {
      cv::Vec3b p = image.at<cv::Vec3b>(r, c);
      *(b++) = p[0];
      *(b++) = p[1];
      *(b++) = p[2];
    }
}

// -------- GREY --------

void ImageConverter::param_grey() {
  RCLCPP_INFO(logger_, "grey");
  cv_copy_encoding_ = sensor_msgs::image_encodings::MONO8;
  cv_color_ = false;
  bytes_per_line_ = 0;
  size_ = width_ * height_;
  fmt_ = &ImageConverter::fmt_grey;
}

void ImageConverter::fmt_grey(const cv::Mat &image, Buffer &buf) {
  Buffer::value_type *b = &buf[0];
  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c)
      *(b++) = image.at<uchar>(r, c);
}

// -------- YVU420 --------

void ImageConverter::param_yvu420() {
  RCLCPP_INFO(logger_, "yvu420");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = true;
  cv_color_code_ = CV_BGR2YCrCb;
  cv_color_channels_ = 0;
  bytes_per_line_ = 0;
  size_ = (width_ * height_) +
          2 * (ROUND_UP_2(width_) / 2 *
               ROUND_UP_2(height_) / 2);
  fmt_ = &ImageConverter::fmt_yvu420;
}

void ImageConverter::fmt_yvu420(const cv::Mat &image,
                                Buffer &buf) {
  Buffer::value_type *y = &buf[0];
  Buffer::value_type *cr = y + (width_ * height_);
  Buffer::value_type *cb =
      cr + (ROUND_UP_2(width_) / 2 *
            ROUND_UP_2(height_) / 2);

  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c) {
      cv::Vec3b p = image.at<cv::Vec3b>(r, c);
      *(y++) = p[0];
      if ((r % 2 == 0) && (c % 2 == 0)) {
        *(cr++) = p[1];
        *(cb++) = p[2];
      }
    }
}

// -------- YUYV --------

void ImageConverter::param_yuyv() {
  RCLCPP_INFO(logger_, "yuyv");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = true;
  cv_color_code_ = CV_BGR2YCrCb;
  cv_color_channels_ = 0;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 2;
  fmt_ = &ImageConverter::fmt_yuyv;
}

void ImageConverter::fmt_yuyv(const cv::Mat &image,
                              Buffer &buf) {
  Buffer::value_type *b = &buf[0];

  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c) {
      cv::Vec3b p = image.at<cv::Vec3b>(r, c);
      *(b++) = p[0];
      *(b++) = (c % 2 == 0) ? p[2] : p[1];
    }
}

// -------- UYVY --------

void ImageConverter::param_uyvy() {
  RCLCPP_INFO(logger_, "uyvy");
  cv_copy_encoding_ = sensor_msgs::image_encodings::BGR8;
  cv_color_ = true;
  cv_color_code_ = CV_BGR2YCrCb;
  cv_color_channels_ = 0;
  bytes_per_line_ = 0;
  size_ = width_ * height_ * 2;
  fmt_ = &ImageConverter::fmt_uyvy;
}

void ImageConverter::fmt_uyvy(const cv::Mat &image,
                              Buffer &buf) {
  Buffer::value_type *b = &buf[0];

  for (int r = 0; r != image.rows; ++r)
    for (int c = 0; c != image.cols; ++c) {
      cv::Vec3b p = image.at<cv::Vec3b>(r, c);

      if (c % 2 == 0) {
        *(b++) = p[2]; // U (Cb)
        *(b++) = p[0]; // Y0
      } else {
        *(b++) = p[1]; // V (Cr)
        *(b++) = p[0]; // Y1
      }
    }
}
