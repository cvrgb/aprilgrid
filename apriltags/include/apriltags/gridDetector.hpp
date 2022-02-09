#ifndef APRILTAGS_GRIDDETECTOR_HPP
#define APRILTAGS_GRIDDETECTOR_HPP

#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "TagDetector.h"
#include "Tag36h11.h"

namespace calibration_toolkit
{

  /**
   * @brief Detection of an aprilgrid
   * point ordering: (e.g. 2x2 grid)
   *        12-----13  14-----15
   *        | TAG 3 |  | TAG 4 |
   *        8-------9  10-----11
   *        4-------5  6-------7
   *  y     | TAG 1 |  | TAG 2 |
   * ^      0-------1  2-------3
   * |-->x
   */

  class AprilgridDetector
  {
  public:
    struct AprilgridOptions
    {
      AprilgridOptions() : doSubpixRefinement(true),
                           maxSubpixDisplacement2(1.5),
                           showExtractionVideo(false),
                           minTagsForValidObs(4),
                           minBorderDistance(5.0),
                           blackTagBorder(2){};
      bool doSubpixRefinement;
      double maxSubpixDisplacement2;
      bool showExtractionVideo;
      unsigned int minTagsForValidObs;
      double minBorderDistance;
      unsigned int blackTagBorder;
    };

    AprilgridDetector(double tagSize,
                      double tagSpacing, const AprilgridOptions &options = AprilgridOptions());

    ~AprilgridDetector();

    /**
     * @brief Find corners of an aprilgrid. Total number of feature points N = 6 x 6 x 4
     * @param  image            Input image, must be gray
     * @param  outImagePoints   Output image points, N x 2 matrix,
     * @param  outCornerObserved Output image points flag, a std vector of size N, each entry = true (detected) or false (not detected)
     * @return true
     * @return false
     */
    bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints, std::vector<bool> &outCornerObserved) const;
    /**
     * @brief Remove occluded grid points (flag  = false) from results of computeObservation
     * @param  inImagePoints    outImagePoints of computeObservation
     * @param  inCornerObserved outCornerObserved of computeObservation
     * @param  outObjectP3ds    std vector of cv::Point3d
     * @param  outImageP2ds     std vector of cv::Point2d
     */
    void convertResults_OpenCV(const Eigen::MatrixXd &inImagePoints, const std::vector<bool> &inCornerObserved,
                               std::vector<cv::Point3d> &outObjectP3ds, std::vector<cv::Point2d> &outImageP2ds);

  private:
    void initialize();
    void createGridPoints();

  public:
    inline size_t size() const { return _rows * _cols; };

    /// \brief the number of rows in the calibration target
    inline size_t rows() const { return _rows; };

    /// \brief the number of columns in the calibration target
    inline size_t cols() const { return _cols; };

    /// \brief get a point from the target expressed in the target frame
    Eigen::Vector3d point(size_t i) const;

    /// \brief get all points from the target expressed in the target frame
    Eigen::MatrixXd points() const;

    /// \brief get the grid coordinates for a point
    std::pair<size_t, size_t> pointToGridCoordinates(size_t i) const;

    /// \brief get the point index from the grid coordinates
    size_t gridCoordinatesToPoint(size_t r, size_t c) const;

    /// \brief get a point from the target expressed in the target frame
    ///        by row and column
    Eigen::Vector3d gridPoint(size_t r, size_t c) const;
    double *getPointDataPointer(size_t i);

  private:
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _points;

    /// \brief the number of point rows in the calibration target
    size_t _rows;

    /// \brief the number of point columns in the calibration target
    size_t _cols;

    /// \brief size of a tag [m]
    double _tagSize;

    /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
    double _tagSpacing;

    /// \brief target extraction options
    AprilgridOptions _options;
    AprilTags::TagCodes _tagCodes;
    std::shared_ptr<AprilTags::TagDetector> _tagDetector;
  };

} // namespace cameras

#endif
