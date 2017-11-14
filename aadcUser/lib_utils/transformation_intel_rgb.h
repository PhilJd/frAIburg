// include opencv, or is it in some adtf stuff?
//#include <opencv2/imgproc/imgproc.hpp>
//#include <vector>

#ifndef AADCUSER_UTILS_TRANSFORMATION_INTEL_RGB_H_
#define AADCUSER_UTILS_TRANSFORMATION_INTEL_RGB_H_

// stdafx.h includes Car.h, which is located at ADTF/src/
// and defines CAR_1 or CAR_2, depending on which car we are
// TODO: create Car.h on Car 1

#ifdef CAR_2
// ----------------------------------------------
// - car 2: transformation calibration 13.09.17 -
// ----------------------------------------------
cv::Point2f my_src[4] = {cv::Point2f(134, 327),  // = y0,43m,   x1,5m
                         cv::Point2f(459, 324),  // = y-0,445m, x1,5m
                         cv::Point2f(562, 375),  // = y-0,445m, x0,8m
                         cv::Point2f(14, 378)};  // = y0,43m,   x0,8m

cv::Point2f my_dst[4] = {cv::Point2f(0.43, 1.50),
                         cv::Point2f(-0.445, 1.50),
                         cv::Point2f(-0.445, 0.8),
                         cv::Point2f(0.43, 0.8)};
#endif  // CAR_2


#ifdef CAR_1
// -----------------------------------------------
//  - car 1: transformation calibration 13.09.17 -
// -----------------------------------------------
cv::Point2f my_src[4] = {cv::Point2f(156, 343),  // = y0,43m,   x1,5m
                         cv::Point2f(486, 343),  // = y-0,445m, x1,5m
                         cv::Point2f(588, 396),  // = y-0,445m, x0,8m
                         cv::Point2f(39, 394)};  // = y0,43m,   x0,8m

cv::Point2f my_dst[4] = {cv::Point2f(0.43, 1.50),
                         cv::Point2f(-0.445, 1.50),
                         cv::Point2f(-0.445, 0.8),
                         cv::Point2f(0.43, 0.8)};
#endif  // CAR_1

#endif  // AADCUSER_UTILS_TRANSFORMATION_INTEL_RGB_H_
