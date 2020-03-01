#pragma once

#include "defines.h"

class point_projector {
  cv::Mat transform_matrix;
  cv::Mat intrinsics_matrix;
  ;

public:
  point_projector()
      : transform_matrix(cv::Mat()), intrinsics_matrix(cv::Mat()) {}

  point_projector(cv::Mat rotation_matrix, cv::Mat translation, cv::Mat intr) {
    transform_matrix = cv::Mat(3, 4, CV_32F);

    intr.copyTo(intrinsics_matrix);

    assert(rotation_matrix.rows == 3);
    assert(rotation_matrix.cols >= 3 && rotation_matrix.cols <= 4);
    assert(translation.rows == 3);
    assert(translation.cols == 1);
    assert(intr.rows == 3);
    assert(intr.cols >= 3 && intr.cols <= 4);

    if (intrinsics_matrix.cols > 3) {
      intrinsics_matrix =
          intrinsics_matrix(cv::Range(0, intrinsics_matrix.rows),
                            cv::Range(0, intrinsics_matrix.cols - 1));
    }

    for (auto i = 0; i < 3; i++)
      for (auto j = 0; j < 3; j++)
        transform_matrix.at<float>(i, j) = rotation_matrix.at<float>(i, j);

    for (auto i = 0; i < translation.rows; i++)
      transform_matrix.at<float>(i, 3) = translation.at<float>(i);
  }

  std::vector<cv::Vec3f>
  project_points(const std::vector<cv::Vec3f> &point_cloud);
};
