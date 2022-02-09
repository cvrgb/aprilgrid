#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "TagDetector.h"
#include "Tag36h11.h"
#include "aprilgrid/detector.hpp"

namespace calibration_toolkit
{
  void AprilgridDetector::convertResults_OpenCV(
      const Eigen::MatrixXd &inImagePoints,
      const std::vector<bool> &inCornerObserved,
      std::vector<cv::Point3d> &outObjectP3ds,
      std::vector<cv::Point2d> &outImageP2ds)
  {
    // std::vector<cv::Point2f> pp2d;
    outImageP2ds.clear();
    outObjectP3ds.clear();
    for (size_t i = 0; i < inImagePoints.rows(); i++)
    {
      if (inCornerObserved[i])
      {
        cv::Point2d p2d(inImagePoints(i, 0), inImagePoints(i, 1));
        cv::Point3d p3d(points()(i, 0), points()(i, 1), points()(i, 2));
        outImageP2ds.push_back(p2d);
        outObjectP3ds.push_back(p3d);
        // cv::circle(frame, p2d, 1, CV_RGB(255, 0, 0), 1);
        // cv::putText(frame, std::to_string(i), p2d, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, 8, false);
      }
    }
  };

  AprilgridDetector::~AprilgridDetector(){};

  /// \brief get all points from the target expressed in the target frame
  Eigen::MatrixXd AprilgridDetector::points() const
  {
    return _points;
  }

  /// \brief get a point from the target expressed in the target frame
  Eigen::Vector3d AprilgridDetector::point(size_t i) const
  {
    return _points.row(i);
  }

  /// \brief get the grid coordinates for a point
  std::pair<size_t, size_t> AprilgridDetector::pointToGridCoordinates(size_t i) const
  {
    return std::pair<size_t, size_t>(i % cols(), (int)i / cols());
  }

  /// \brief get the point index from the grid coordinates
  size_t AprilgridDetector::gridCoordinatesToPoint(size_t r, size_t c) const
  {
    return cols() * r + c;
  }

  /// \brief get a point from the target expressed in the target frame
  ///        by row and column
  Eigen::Vector3d AprilgridDetector::gridPoint(size_t r, size_t c) const
  {
    return _points.row(gridCoordinatesToPoint(r, c));
  }

  double *AprilgridDetector::getPointDataPointer(size_t i)
  {

    assert(size() - 1 < i);
    return &_points(i, 0);
  }

  // AprilgridDetector::~AprilgridDetector(){};
  /// \brief Construct an Aprilgrid calibration target
  ///        tagRows:    number of tags in y-dir (gridRows = 2*tagRows)
  ///        tagCols:    number of tags in x-dir (gridCols = 2*tagCols)
  ///        tagSize:    size of a tag [m]
  ///        tagSpacing: space between tags (in tagSpacing [m] = tagSpacing*tagSize)
  ///
  ///        corner ordering in _points :
  ///          12-----13  14-----15
  ///          | TAG 3 |  | TAG 4 |
  ///          8-------9  10-----11
  ///          4-------5  6-------7
  ///    y     | TAG 1 |  | TAG 2 |
  ///   ^      0-------1  2-------3
  ///   |-->x
  AprilgridDetector::AprilgridDetector(
      double tagSize, double tagSpacing, const AprilgridOptions &options)
      : // GridCalibrationTargetBase(2 * tagRows, 2 * tagCols), // 4 points per tag
        _rows(2 * 6),
        _cols(2 * 6),
        _tagSize(tagSize),
        _tagSpacing(tagSpacing),
        _options(options),
        _tagCodes(AprilTags::tagCodes36h11)
  {

    _points.resize(size(), 3);
    createGridPoints();
    initialize();
  }

  /// \brief initialize the object
  void AprilgridDetector::initialize()
  {
    if (_options.showExtractionVideo)
    {
      cv::namedWindow("Aprilgrid: Tag detection", cv::WINDOW_NORMAL);
      cv::namedWindow("Aprilgrid: Tag corners", cv::WINDOW_NORMAL);
    }

    // create the tag detector
    _tagDetector = std::make_shared<AprilTags::TagDetector>(_tagCodes, _options.blackTagBorder);
  }

  /// \brief initialize an april grid
  ///   point ordering: (e.g. 2x2 grid)
  ///          12-----13  14-----15
  ///          | TAG 3 |  | TAG 4 |
  ///          8-------9  10-----11
  ///          4-------5  6-------7
  ///    y     | TAG 1 |  | TAG 2 |
  ///   ^      0-------1  2-------3
  ///   |-->x
  void AprilgridDetector::createGridPoints()
  {
    // each tag has 4 corners
    // unsigned int numTags = size()/4;
    // unsigned int colsTags = _cols/2;

    for (unsigned r = 0; r < _rows; r++)
    {
      for (unsigned c = 0; c < _cols; c++)
      {
        Eigen::Matrix<double, 1, 3> point;

        point(0) = (int)(c / 2) * (1 + _tagSpacing) * _tagSize + (c % 2) * _tagSize;
        point(1) = (int)(r / 2) * (1 + _tagSpacing) * _tagSize + (r % 2) * _tagSize;
        point(2) = 0.0;
        //  std::cout << _points.rows() << std::endl;

        _points.row(r * _cols + c) = point;
      }
    }
  }

  /// \brief extract the calibration target points from an image and write to an observation
  bool AprilgridDetector::computeObservation(
      const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
      std::vector<bool> &outCornerObserved) const
  {

    bool success = true;

    // detect the tags
    // AprilTags::TagDetector _tagDetector(_tagCodes, 2);
    std::vector<AprilTags::TagDetection> detections = _tagDetector->extractTags(image);

    // tagDetector.extractTags()

    // std::vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
    // std::cout << detections.size() << std::endl;
    // cv::Mat imageClone;
    // cv::cvtColor(image, imageClone, cv::COLOR_RGBA2BGR);
    // //= image.clone();
    // for (int i = 0; i < detections.size(); i++)
    // {
    //   // also highlight in the image
    //   detections[i].draw(imageClone);
    // }
    // cv::imshow("Tag BEFORE", imageClone); // OpenCV call
    // cv::waitKey(0);

    /* handle the case in which a tag is identified but not all tag
     * corners are in the image (all data bits in image but border
     * outside). tagCorners should still be okay as apriltag-lib
     * extrapolates them, only the subpix refinement will fail
     */

    // min. distance [px] of tag corners from image border (tag is not used if violated)
    std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();
    for (iter = detections.begin(); iter != detections.end();)
    {
      // check all four corners for violation
      bool remove = false;

      for (int j = 0; j < 4; j++)
      {
        remove |= iter->p[j].first < _options.minBorderDistance;
        remove |= iter->p[j].first > (float)(image.cols) - _options.minBorderDistance; // width
        remove |= iter->p[j].second < _options.minBorderDistance;
        remove |= iter->p[j].second > (float)(image.rows) - _options.minBorderDistance; // height
      }

      // also remove tags that are flagged as bad
      if (iter->good != 1)
        remove |= true;

      // also remove if the tag ID is out-of-range for this grid (faulty detection)
      if (iter->id >= (int)size() / 4)
        remove |= true;

      // delete flagged tags
      if (remove)
      {
        // SM_DEBUG_STREAM("Tag with ID " << iter->id << " is only partially in image (corners outside) and will be removed from the TargetObservation.\n");

        // delete the tag and advance in list
        iter = detections.erase(iter);
      }
      else
      {
        // advance in list
        ++iter;
      }
    }

    // did we find enough tags?
    if (detections.size() < _options.minTagsForValidObs)
    {
      success = false;

      // immediate exit if we dont need to show video for debugging...
      // if video is shown, exit after drawing video...
      if (!_options.showExtractionVideo)
        return success;
    }

    // sort detections by tagId
    std::sort(detections.begin(), detections.end(),
              AprilTags::TagDetection::sortByIdCompare);

    // check for duplicate tagIds (--> if found: wild Apriltags in image not belonging to calibration target)
    // (only if we have more than 1 tag...)
    if (detections.size() > 1)
    {
      for (unsigned i = 0; i < detections.size() - 1; i++)
        if (detections[i].id == detections[i + 1].id)
        {
          // show the duplicate tags in the image
          cv::destroyAllWindows();
          cv::namedWindow("Wild Apriltag detected. Hide them!");
          cv::startWindowThread();
          // cvStartWindowThread();

          cv::Mat imageCopy = image.clone();
          cv::cvtColor(imageCopy, imageCopy, cv::COLOR_GRAY2BGR);

          // mark all duplicate tags in image
          for (int j = 0; i < detections.size() - 1; i++)
          {
            if (detections[j].id == detections[j + 1].id)
            {
              detections[j].draw(imageCopy);
              detections[j + 1].draw(imageCopy);
            }
          }

          cv::putText(imageCopy, "Duplicate Apriltags detected. Hide them.",
                      cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255, 0, 0), 2, 8, false);
          cv::putText(imageCopy, "Press enter to exit...", cv::Point(50, 80),
                      cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 0, 0), 2, 8, false);
          cv::imshow("Duplicate Apriltags detected. Hide them", imageCopy); // OpenCV call

          // and exit
          //            SM_FATAL_STREAM("\n[ERROR]: Found apriltag not belonging to calibration board. Check the image for the tag and hide it.\n");

          cv::waitKey(0);
          exit(0);
        }
    }

    // convert corners to cv::Mat (4 consecutive corners form one tag)
    /// point ordering here
    ///          11-----10  15-----14
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  12-----13
    ///          3-------2  7-------6
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  4-------5
    ///   |-->x
    cv::Mat tagCorners(4 * detections.size(), 2, CV_32F);

    for (unsigned i = 0; i < detections.size(); i++)
    {
      for (unsigned j = 0; j < 4; j++)
      {
        tagCorners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
        tagCorners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
      }
    }

    // store a copy of the corner list before subpix refinement
    cv::Mat tagCornersRaw = tagCorners.clone();

    // optional subpixel refinement on all tag corners (four corners each tag)
    if (_options.doSubpixRefinement && success)
      cv::cornerSubPix(
          image, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

    if (_options.showExtractionVideo)
    {
      // image with refined (blue) and raw corners (red)
      cv::Mat imageCopy1 = image.clone();
      cv::cvtColor(imageCopy1, imageCopy1, cv::COLOR_GRAY2BGR);
      for (unsigned i = 0; i < detections.size(); i++)
        for (unsigned j = 0; j < 4; j++)
        {
          // raw apriltag corners
          cv::circle(imageCopy1, cv::Point2f(detections[i].p[j].first, detections[i].p[j].second), 2, CV_RGB(255, 0, 0), 1);

          // subpixel refined corners
          cv::circle(
              imageCopy1,
              cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
                          tagCorners.at<float>(4 * i + j, 1)),
              3, CV_RGB(0, 0, 255), 1);

          if (!success)
            cv::putText(imageCopy1, "Detection failed! (frame not used)",
                        cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255, 0, 0), 3, 8, false);
        }

      cv::imshow("Aprilgrid: Tag corners", imageCopy1); // OpenCV call
      // cv::waitKey(0);

      /* copy image for modification */
      cv::Mat imageCopy2 = image.clone();
      cv::cvtColor(imageCopy2, imageCopy2, cv::COLOR_GRAY2BGR);
      /* highlight detected tags in image */
      for (unsigned i = 0; i < detections.size(); i++)
      {
        detections[i].draw(imageCopy2);

        if (!success)
          cv::putText(imageCopy2, "Detection failed! (frame not used)",
                      cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255, 0, 0), 3, 8, false);
      }

      cv::imshow("Aprilgrid: Tag detection", imageCopy2); // OpenCV call
      // printf("asdfasdfsadf\n");
      //  cv::waitKey(0);

      // if success is false exit here (delayed exit if _options.showExtractionVideo=true for debugging)
      if (!success)
        return success;
    }

    // insert the observed points into the correct location of the grid point array
    /// point ordering
    ///          12-----13  14-----15
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  10-----11
    ///          4-------5  6-------7
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  2-------3
    ///   |-->x

    outCornerObserved.resize(size(), false);
    outImagePoints.resize(size(), 2);

    for (unsigned int i = 0; i < detections.size(); i++)
    {
      // get the tag id
      unsigned int tagId = detections[i].id;

      // calculate the grid idx for all four tag corners given the tagId and cols
      unsigned int baseId = (int)(tagId / (_cols / 2)) * _cols * 2 + (tagId % (_cols / 2)) * 2;
      unsigned int pIdx[] = {baseId, baseId + 1, baseId + (unsigned int)_cols + 1, baseId + (unsigned int)_cols};

      // add four points per tag
      for (int j = 0; j < 4; j++)
      {
        // refined corners
        double corner_x = tagCorners.row(4 * i + j).at<float>(0);
        double corner_y = tagCorners.row(4 * i + j).at<float>(1);

        // raw corners
        double cornerRaw_x = tagCornersRaw.row(4 * i + j).at<float>(0);
        double cornerRaw_y = tagCornersRaw.row(4 * i + j).at<float>(1);

        // only add point if the displacement in the subpixel refinement is below a given threshold
        double subpix_displacement_squared = (corner_x - cornerRaw_x) * (corner_x - cornerRaw_x) + (corner_y - cornerRaw_y) * (corner_y - cornerRaw_y);

        // add all points, but only set active if the point has not moved to far in the subpix refinement
        outImagePoints.row(pIdx[j]) = Eigen::Matrix<double, 1, 2>(corner_x,
                                                                  corner_y);

        if (subpix_displacement_squared <= _options.maxSubpixDisplacement2)
        {
          outCornerObserved[pIdx[j]] = true;
          // cv::putText(image, std::to_string(pIdx[j]),
          //             cv::Point(corner_x, corner_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          //             CV_RGB(255, 0, 0), 2, 8, false);
        }
        else
        {
          // std::cout << "Subpix refinement failed for point: " << pIdx[j] << " with displacement: " << sqrt(subpix_displacement_squared) << "(point removed) \n";
          outCornerObserved[pIdx[j]] = false;
        }
      }
    }
    return success;
  }

} // namespace cameras
