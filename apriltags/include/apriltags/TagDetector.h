#ifndef APRILTAGS_TAGDETECTOR_H
#define APRILTAGS_TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "TagDetection.h"
#include "TagFamily.h"
#include "FloatImage.h"

namespace AprilTags
{

	class TagDetector
	{
	public:
		const TagFamily thisTagFamily;

		//! Constructor
		// note: TagFamily is instantiated here from TagCodes
		TagDetector(const TagCodes &tagCodes, const size_t blackBorder = 2) : thisTagFamily(tagCodes, blackBorder) {}

		std::vector<TagDetection> extractTags(const cv::Mat &image);
	};

} // namespace

#endif
