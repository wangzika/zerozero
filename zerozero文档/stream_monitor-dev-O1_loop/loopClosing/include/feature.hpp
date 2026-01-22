

#include "opencv2/features2d.hpp"

#include <opencv2/opencv.hpp>
class CV_EXPORTS_W TEBLID : public cv::Feature2D {
 public:
  /**
   * @brief  Descriptor number of bits, each bit is a box average difference.
   * The user can choose between 256 or 512 bits.
   */
  enum TeblidSize {
    SIZE_256_BITS = 102,
    SIZE_512_BITS = 103,
  };
  /** @brief Creates the TEBLID descriptor.
  @param scale_factor Adjust the sampling window around detected keypoints:
  - <b> 1.00f </b> should be the scale for ORB keypoints
  - <b> 6.75f </b> should be the scale for SIFT detected keypoints
  - <b> 6.25f </b> is default and fits for KAZE, SURF detected keypoints
  - <b> 5.00f </b> should be the scale for AKAZE, MSD, AGAST, FAST, BRISK keypoints
  @param n_bits Determine the number of bits in the descriptor. Should be either
   TEBLID::SIZE_256_BITS or TEBLID::SIZE_512_BITS.
  */
  CV_WRAP static cv::Ptr<TEBLID> create(float scale_factor, int n_bits = TEBLID::SIZE_256_BITS);

  CV_WRAP cv::String getDefaultName() const CV_OVERRIDE;
};

class CV_EXPORTS_W BEBLID : public cv::Feature2D {
 public:
  /**
   * @brief  Descriptor number of bits, each bit is a boosting weak-learner.
   * The user can choose between 512 or 256 bits.
   */
  enum BeblidSize {
    SIZE_512_BITS = 100,
    SIZE_256_BITS = 101,
  };
  /** @brief Creates the BEBLID descriptor.
  @param scale_factor Adjust the sampling window around detected keypoints:
  - <b> 1.00f </b> should be the scale for ORB keypoints
  - <b> 6.75f </b> should be the scale for SIFT detected keypoints
  - <b> 6.25f </b> is default and fits for KAZE, SURF detected keypoints
  - <b> 5.00f </b> should be the scale for AKAZE, MSD, AGAST, FAST, BRISK keypoints
  @param n_bits Determine the number of bits in the descriptor. Should be either
   BEBLID::SIZE_512_BITS or BEBLID::SIZE_256_BITS.
  */
  CV_WRAP static cv::Ptr<BEBLID> create(float scale_factor, int n_bits = BEBLID::SIZE_512_BITS);

  CV_WRAP virtual void setScaleFactor(float scale_factor) = 0;
  CV_WRAP virtual float getScaleFactor() const = 0;

  CV_WRAP cv::String getDefaultName() const CV_OVERRIDE;
};
