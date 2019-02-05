#include "asn1_bitstream_to_disparity_image.hpp"
#include <opencv2/ximgproc.hpp>

void fromASN1SCCToRosImage(const asn1SccFrame& image, sensor_msgs::Image& msg_image){

    // Fill the sensor_msgs::Image data struct with asn1SccFrame data
    msg_image.header.stamp.fromNSec((uint64_t)image.metadata.timeStamp.microseconds * 1000ull);

    msg_image.width = image.data.cols;
    msg_image.height = image.data.rows;

    msg_image.encoding = "mono8";
    msg_image.is_bigendian = true; //or false?
    msg_image.step = msg_image.width;
    msg_image.data.resize(image.data.data.nCount);

    int i = 0;
    while(i < msg_image.data.size())
    {
        msg_image.data[i] = image.data.data.arr[i];
        i++;
    }
}

namespace infuse_debug_tools
{

    Asn1BitstreamToDisparity::Asn1BitstreamToDisparity() :
        private_nh_{"~"},
        connect_image_srv_{private_nh_.advertiseService("connect_image", &Asn1BitstreamToDisparity::connect_image, this)},
        asn1_image_ptr_{std::make_unique<asn1SccFramePair>()},
        out_raw_disparity_ptr_{std::make_unique<asn1SccFrame>()},
        out_color_disparity_ptr_{std::make_unique<asn1SccFrame>()}
    {
        // Set the differents stereo matching parameters
        private_nh_.param("algorithm", matching_parameters_.stereo_matcher.algorithm, int(1));
        private_nh_.param("min_disparity", matching_parameters_.stereo_matcher.min_disparity, int(0));
        private_nh_.param("num_disparities", matching_parameters_.stereo_matcher.num_disparities, int(64));
        private_nh_.param("block_size", matching_parameters_.stereo_matcher.block_size, int(1));
        private_nh_.param("speckle_window_size", matching_parameters_.stereo_matcher.speckle_window_size, int(50));
        private_nh_.param("speckle_range", matching_parameters_.stereo_matcher.speckle_range, int(1));
        private_nh_.param("disp12_max_diff", matching_parameters_.stereo_matcher.disp12_max_diff, int(-1));
        private_nh_.param("pre_filter_cap", matching_parameters_.stereo_matcher.pre_filter_cap, int(31));
        private_nh_.param("uniqueness_ratio", matching_parameters_.stereo_matcher.uniqueness_ratio, int(15));

        private_nh_.param("pre_filter_type", matching_parameters_.stereo_matcher.bm_params.pre_filter_type, int(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE));
        private_nh_.param("pre_filter_size", matching_parameters_.stereo_matcher.bm_params.pre_filter_size, int(9));
        private_nh_.param("texture_threshold", matching_parameters_.stereo_matcher.bm_params.texture_threshold, int(10));

        private_nh_.param("P1", matching_parameters_.stereo_matcher.sgbm_params.P1, int(0));
        private_nh_.param("P2", matching_parameters_.stereo_matcher.sgbm_params.P2, int(0));
        private_nh_.param("mode", matching_parameters_.stereo_matcher.sgbm_params.mode, int(cv::StereoSGBM::MODE_SGBM));

        #if WITH_XIMGPROC
            private_nh_.param("use_filter", matching_parameters_.filter.use_filter, bool(false));
            private_nh_.param("use_confidence", matching_parameters_.filter.use_confidence, bool(false));
            private_nh_.param("depth_discontinuity_radius", matching_parameters_.filter.depth_discontinuity_radius, int(0));
            private_nh_.param("lambda", matching_parameters_.filter.lambda, double(8000.0));
            private_nh_.param("lrc_thresh", matching_parameters_.filter.lrc_thresh, int(24));
            private_nh_.param("sigma_color", matching_parameters_.filter.sigma_color, double(1.5));
        #endif

        // Set the differents rectification parameters
        private_nh_.param("xratio", rect_parameters_.xratio, int(3));
        private_nh_.param("yratio", rect_parameters_.yratio, int(3));
        private_nh_.param("scaling", rect_parameters_.scaling, double(0));
        private_nh_.param("calibration_file_path", rect_parameters_.calibration_file_path, std::string(" "));

        {
            // Get topic name to connect
            std::string topics_to_connect;
            if (private_nh_.getParam("topics_to_connect", topics_to_connect)) {

                // Split strings into a vector
                std::vector<std::string> topics;
                {
                    std::stringstream ss(topics_to_connect);
                    std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
                    topics.assign(begin, eos);
                }

                // Connect to topics
                for (const auto & topic : topics) {
                    try {
                        bind_subToPubs(topic);
                    } catch (...) {
                        ROS_INFO_STREAM("ERROR: Could not connect to topic " << topic);
                    }
                }
            }
        }
    }

    bool Asn1BitstreamToDisparity::connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
                                       infuse_debug_tools::ConnectTopic::Response &res)
    {
        try
        {
            // Create subscriber and publishers
            bind_subToPubs(req.topic);
            res.success = true;

            return true;
        }
        catch (...)
        {
            std::stringstream ss;
            ss << "Unknown exception when subscribing to topic " << req.topic;
            ROS_INFO_STREAM(ss.str());
            res.success = false;
            res.message = ss.str();

            return false;
        }
    }

    void Asn1BitstreamToDisparity::bind_subToPubs(std::string topic_in){

        std::string output_topic_disparity = topic_in + "_rosDisparity";

        // Create publisher for disparity image
        pub_map_[topic_in] = nh_.advertise<sensor_msgs::Image>(output_topic_disparity, 100);

        // Bind publisher to the callback
        boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
                boost::bind(&Asn1BitstreamToDisparity::image_callback, this, _1, boost::cref(pub_map_[topic_in]));

        // Create subscriber
        sub_map_[topic_in] = nh_.subscribe<sensor_msgs::Image>(topic_in, 100, callback);
        ROS_INFO_STREAM("Connected to topic " << topic_in << ". Publishing disparity image on " << output_topic_disparity);

    }

    void Asn1BitstreamToDisparity::image_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_disparity)
    {
        // Get time at the begining of the callback -- unused now...
        auto cb_time = ros::Time::now();

        // Initialize
        asn1SccFramePair_Initialize(asn1_image_ptr_.get());

        // Decode
        flag res;
        int errorCode;
        BitStream bstream;
        BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
        res = asn1SccFramePair_Decode(asn1_image_ptr_.get(), &bstream, &errorCode);
        if (not res)
        {
            ROS_INFO("Error decoding asn1Image! Error: %d", errorCode);
            return;
        }

        // Process stereo matching
        process_stereo_matching(*asn1_image_ptr_, *out_raw_disparity_ptr_, *out_color_disparity_ptr_);

        // Convert to sensor_msgs/Image
        static sensor_msgs::Image msg_image_disparity;
        fromASN1SCCToRosImage(*out_raw_disparity_ptr_, msg_image_disparity);
        pub_disparity.publish(msg_image_disparity);
    }

    void Asn1BitstreamToDisparity::process_stereo_matching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity, asn1SccFrame& out_color_disparity)
    {
        cv::Mat rect_left, rect_right;
        cv::Mat disparity;

        process_stereo_rectification(in_frame_pair, *asn1_rect_out_frame_pair_ptr_, rect_left, rect_right);

        // Using Algorithm StereoBM
        if(matching_parameters_.stereo_matcher.algorithm == 0)
        {
            if(_bm.empty())
            {
                _bm = cv::StereoBM::create(matching_parameters_.stereo_matcher.num_disparities, matching_parameters_.stereo_matcher.block_size);
            }

            _bm->setBlockSize(matching_parameters_.stereo_matcher.block_size);
            _bm->setDisp12MaxDiff(matching_parameters_.stereo_matcher.disp12_max_diff);
            _bm->setMinDisparity(matching_parameters_.stereo_matcher.min_disparity);
            _bm->setNumDisparities(matching_parameters_.stereo_matcher.num_disparities);
            _bm->setPreFilterCap(matching_parameters_.stereo_matcher.pre_filter_cap);
            _bm->setPreFilterSize(matching_parameters_.stereo_matcher.bm_params.pre_filter_size);
            _bm->setPreFilterType(matching_parameters_.stereo_matcher.bm_params.pre_filter_type);
            _bm->setSpeckleRange(matching_parameters_.stereo_matcher.speckle_range);
            _bm->setSpeckleWindowSize(matching_parameters_.stereo_matcher.speckle_window_size);
            _bm->setTextureThreshold(matching_parameters_.stereo_matcher.bm_params.texture_threshold);
            _bm->setUniquenessRatio(matching_parameters_.stereo_matcher.uniqueness_ratio);

            _bm->compute(rect_left, rect_right, disparity);



        }
        // Using Algorithm StereoSGBM
        else if(matching_parameters_.stereo_matcher.algorithm == 1)
        {
            if(_sgbm.empty())
            {
                _sgbm = cv::StereoSGBM::create(matching_parameters_.stereo_matcher.min_disparity, matching_parameters_.stereo_matcher.num_disparities, matching_parameters_.stereo_matcher.block_size, matching_parameters_.stereo_matcher.sgbm_params.P1, matching_parameters_.stereo_matcher.sgbm_params.P2, matching_parameters_.stereo_matcher.disp12_max_diff, matching_parameters_.stereo_matcher.pre_filter_cap, matching_parameters_.stereo_matcher.uniqueness_ratio, matching_parameters_.stereo_matcher.speckle_window_size, matching_parameters_.stereo_matcher.speckle_range, matching_parameters_.stereo_matcher.sgbm_params.mode);
            }

            _sgbm->setBlockSize(matching_parameters_.stereo_matcher.block_size);
            _sgbm->setDisp12MaxDiff(matching_parameters_.stereo_matcher.disp12_max_diff);
            _sgbm->setMinDisparity(matching_parameters_.stereo_matcher.min_disparity);
            _sgbm->setMode(matching_parameters_.stereo_matcher.sgbm_params.mode);
            _sgbm->setNumDisparities(matching_parameters_.stereo_matcher.num_disparities);
            _sgbm->setP1(matching_parameters_.stereo_matcher.sgbm_params.P1);
            _sgbm->setP2(matching_parameters_.stereo_matcher.sgbm_params.P2);
            _sgbm->setPreFilterCap(matching_parameters_.stereo_matcher.pre_filter_cap);
            _sgbm->setSpeckleRange(matching_parameters_.stereo_matcher.speckle_range);
            _sgbm->setSpeckleWindowSize(matching_parameters_.stereo_matcher.speckle_window_size);
            _sgbm->setUniquenessRatio(matching_parameters_.stereo_matcher.uniqueness_ratio);

            _sgbm->compute(rect_left, rect_right, disparity);

        }

    #if WITH_XIMGPROC
        bool reset_filter = false;
        bool reset_matcher = false;
        if(matching_parameters_.stereo_matcher.algorithm != _algorithm)
        {
            _algorithm = matching_parameters_.stereo_matcher.algorithm;
            reset_filter = true;
            reset_matcher = true;
        }
        else if(matching_parameters_.filter.use_confidence != _use_confidence)
        {
            _use_confidence = matching_parameters_.filter.use_confidence;
            reset_filter = true;
        }

        if(matching_parameters_.filter.use_filter)
        {
            cv::Mat disparity_filtered;

            if(matching_parameters_.filter.use_confidence
                    ){
                cv::Mat disparity_right;
                if(_right_matcher.empty() || reset_matcher)
                {
                    switch(_algorithm)
                    {
                        case 0:
                            _right_matcher = cv::ximgproc::createRightMatcher(_bm);
                            break;
                        case 1:
                            _right_matcher = cv::ximgproc::createRightMatcher(_sgbm);
                            break;
                    }
                }

                _right_matcher->compute(rect_left, rect_right, disparity_right);

                if(_filter.empty() || reset_filter)
                {
                    switch(_algorithm)
                    {
                        case 0:
                            _filter = cv::ximgproc::createDisparityWLSFilter(_bm);
                            break;
                        case 1:
                            _filter = cv::ximgproc::createDisparityWLSFilter(_sgbm);
                            break;
                    }
                }

                _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
                _filter->setLambda(matching_parameters_.filter.lambda);
                _filter->setLRCthresh(matching_parameters_.filter.lrc_thresh);
                _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

                _filter->filter(disparity, rect_left, disparity_filtered, disparity_right);
            }
            else
            {
                if(_filter.empty() || reset_filter)
                {
                    _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
                }

                _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
                _filter->setLambda(matching_parameters_.filter.lambda);
                _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

                _filter->filter(disparity, rect_left, disparity_filtered);
            }

            disparity = disparity_filtered;
        }
    #endif

        // Convert Mat to ASN.1
        out_raw_disparity.metadata.msgVersion = frame_Version;
        out_raw_disparity.metadata = in_frame_pair.left.metadata;
        out_raw_disparity.intrinsic = in_frame_pair.left.intrinsic;
        out_raw_disparity.extrinsic = in_frame_pair.left.extrinsic;

        out_raw_disparity.metadata.mode = asn1Sccmode_UNDEF;
        out_raw_disparity.metadata.pixelModel = asn1Sccpix_DISP;
        out_raw_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
        out_raw_disparity.metadata.errValues.arr[0].value = -16.0;
        out_raw_disparity.metadata.errValues.nCount = 1;

        double min_disp, max_disp;
        cv::minMaxLoc(disparity, &min_disp, &max_disp);
        out_raw_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
        out_raw_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
        out_raw_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
        out_raw_disparity.metadata.pixelCoeffs.arr[3] = max_disp;
        out_raw_disparity.metadata.pixelCoeffs.arr[4] = min_disp;

        out_raw_disparity.data.msgVersion = array3D_Version;
        out_raw_disparity.data.channels = static_cast<asn1SccT_UInt32>(disparity.channels());
        out_raw_disparity.data.rows = static_cast<asn1SccT_UInt32>(disparity.rows);
        out_raw_disparity.data.cols = static_cast<asn1SccT_UInt32>(disparity.cols);
        out_raw_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity.depth());
        out_raw_disparity.data.rowSize = disparity.step[0];
        out_raw_disparity.data.data.nCount =  static_cast<int>(out_raw_disparity.data.rows * out_raw_disparity.data.rowSize);
        memcpy(out_raw_disparity.data.data.arr, disparity.data, static_cast<size_t>(out_raw_disparity.data.data.nCount));

        // Convert the filtered image as a cv::Mat for display
        cv::Mat filtered =  cv::Mat(static_cast<int>(out_raw_disparity.data.rows), static_cast<int>(out_raw_disparity.data.cols),
                                    CV_MAKETYPE(static_cast<int>(out_raw_disparity.data.depth), static_cast<int>(out_raw_disparity.data.channels)),
                                    out_raw_disparity.data.data.arr, out_raw_disparity.data.rowSize);

        // Apply a colormap
        cv::Mat filteredDisparityColor;
        double min,    max;
        cv::minMaxLoc(filtered, &min, &max);
        filtered.convertTo(filtered, CV_8U, 255 / (max - min), -255.0 * min / (max - min));
        cv::Mat mask = filtered > 0;
        cv::applyColorMap(filtered, filtered, 2);
        filtered.copyTo(filteredDisparityColor, mask);

        // Convert Mat to ASN.1
        out_color_disparity.metadata.msgVersion = frame_Version;
        out_color_disparity.metadata = in_frame_pair.left.metadata;
        out_color_disparity.intrinsic = in_frame_pair.left.intrinsic;
        out_color_disparity.extrinsic = in_frame_pair.left.extrinsic;

        out_color_disparity.metadata.mode = asn1Sccmode_UNDEF;
        out_color_disparity.metadata.pixelModel = asn1Sccpix_DISP;
        out_color_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
        out_color_disparity.metadata.errValues.arr[0].value = -16.0;
        out_color_disparity.metadata.errValues.nCount = 1;

        double min_color_disp, max_color_disp;
        cv::minMaxLoc(filteredDisparityColor, &min_color_disp, &max_color_disp);
        out_color_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
        out_color_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
        out_color_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
        out_color_disparity.metadata.pixelCoeffs.arr[3] = max_color_disp;
        out_color_disparity.metadata.pixelCoeffs.arr[4] = min_color_disp;

        out_color_disparity.data.msgVersion = array3D_Version;
        out_color_disparity.data.channels = static_cast<asn1SccT_UInt32>(filteredDisparityColor.channels());
        out_color_disparity.data.rows = static_cast<asn1SccT_UInt32>(filteredDisparityColor.rows);
        out_color_disparity.data.cols = static_cast<asn1SccT_UInt32>(filteredDisparityColor.cols);
        out_color_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(filteredDisparityColor.depth());
        out_color_disparity.data.rowSize = filteredDisparityColor.step[0];
        out_color_disparity.data.data.nCount =  static_cast<int>(out_color_disparity.data.rows * out_color_disparity.data.rowSize);
        memcpy(out_color_disparity.data.data.arr, filteredDisparityColor.data, static_cast<size_t>(out_color_disparity.data.data.nCount));

    }

    void Asn1BitstreamToDisparity::process_stereo_rectification(asn1SccFramePair& in_original_stereo_pair, asn1SccFramePair& out_rectified_stereo_pair, cv::Mat & out_rect_left, cv::Mat & out_rect_right)
    {
        cv::Mat in_left(static_cast<int>(in_original_stereo_pair.left.data.rows), static_cast<int>(in_original_stereo_pair.left.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.left.data.depth), static_cast<int>(in_original_stereo_pair.left.data.channels)), in_original_stereo_pair.left.data.data.arr, in_original_stereo_pair.left.data.rowSize);
        cv::Mat in_right(static_cast<int>(in_original_stereo_pair.right.data.rows), static_cast<int>(in_original_stereo_pair.right.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.right.data.depth), static_cast<int>(in_original_stereo_pair.right.data.channels)), in_original_stereo_pair.right.data.data.arr, in_original_stereo_pair.right.data.rowSize);

        // Generate correction maps if needed
        if( std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr)) != _sensor_id_left ||
                std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr)) != _sensor_id_right ||
                rect_parameters_.calibration_file_path != _calibration_file_path ||
                rect_parameters_.xratio != _xratio ||
                rect_parameters_.yratio != _yratio ||
                rect_parameters_.scaling != _scaling){

            _sensor_id_left = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr));
            _sensor_id_right = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr));
            _calibration_file_path = rect_parameters_.calibration_file_path;
            _xratio = rect_parameters_.xratio;
            _yratio = rect_parameters_.yratio;
            _scaling = rect_parameters_.scaling;

            cv::FileStorage fs( _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml", cv::FileStorage::READ );
            if( fs.isOpened() ){
                cv::Size image_size;
                cv::Mat1d camera_matrix_L;
                cv::Mat1d camera_matrix_R;
                cv::Mat1d dist_coeffs_L;
                cv::Mat1d dist_coeffs_R;
                cv::Mat1d R;
                cv::Mat1d T;

                fs["image_width"]  >> image_size.width;
                fs["image_height"] >> image_size.height;

                fs["camera_matrix_1"] >> camera_matrix_L;
                fs["distortion_coefficients_1"] >> dist_coeffs_L;

                fs["camera_matrix_2"] >> camera_matrix_R;
                fs["distortion_coefficients_2"] >> dist_coeffs_R;

                fs["rotation_matrix"] >> R;
                fs["translation_coefficients"] >> T;

                cv::Size newSize;
                newSize.width = image_size.width / _xratio;
                newSize.height = image_size.height / _yratio;

                cv::Mat1d RLeft;
                cv::Mat1d RRight;
                cv::Mat1d Q;

                cv::stereoRectify(camera_matrix_L, dist_coeffs_L, camera_matrix_R, dist_coeffs_R, image_size, R, T, RLeft, RRight, _PLeft, _PRight, Q, CV_CALIB_ZERO_DISPARITY, _scaling, newSize);

                cv::initUndistortRectifyMap(camera_matrix_L, dist_coeffs_L, RLeft, _PLeft, newSize, CV_32F, _lmapx, _lmapy);
                cv::initUndistortRectifyMap(camera_matrix_R, dist_coeffs_R, RRight, _PRight, newSize, CV_32F, _rmapx, _rmapy);

                _baseline = 1.0 / Q.at<double>(3,2);

                _initialized = true;
            }
            else{
                _initialized = false;
                std::cerr << "Can't open the calibration file: " << _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml" << std::endl;
            }
        }

        if( _initialized ){
            cv::remap(in_left, out_rect_left, _lmapx, _lmapy, cv::INTER_LINEAR);
            cv::remap(in_right, out_rect_right, _rmapx, _rmapy, cv::INTER_LINEAR);

            // Getting image pair
            out_rectified_stereo_pair.msgVersion = frame_Version;
            out_rectified_stereo_pair.baseline = _baseline;

            // Left image
            {
                asn1SccFrame & img = out_rectified_stereo_pair.left;

                // init the structure
                img.msgVersion = frame_Version;

                img.intrinsic = in_original_stereo_pair.left.intrinsic;

                Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PLeft.data), 3, 3, Eigen::OuterStride<>(4));

                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1);
                img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.left.intrinsic.distCoeffs.nCount;

                img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

                img.extrinsic = in_original_stereo_pair.left.extrinsic;
                img.metadata = in_original_stereo_pair.left.metadata;

                // Array3D
                {
                    img.data.msgVersion = array3D_Version;
                    img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_left.rows);
                    img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_left.cols);
                    img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_left.channels());
                    img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_left.depth());
                    img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_left.step[0]);
                    img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                    memcpy(img.data.data.arr, out_rect_left.data, static_cast<size_t>(img.data.data.nCount));
                }
            }

            // Right image
            {
                asn1SccFrame & img = out_rectified_stereo_pair.right;

                // init the structure
                img.msgVersion = frame_Version;

                img.intrinsic = in_original_stereo_pair.right.intrinsic;

                Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PRight.data), 3, 3, Eigen::OuterStride<>(4));

                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1);
                img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.right.intrinsic.distCoeffs.nCount;

                img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

                img.extrinsic = in_original_stereo_pair.right.extrinsic;
                img.metadata = in_original_stereo_pair.right.metadata;

                // Array3D
                {
                    img.data.msgVersion = array3D_Version;
                    img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_right.rows);
                    img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_right.cols);
                    img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_right.channels());
                    img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_right.depth());
                    img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_right.step[0]);
                    img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                    memcpy(img.data.data.arr, out_rect_right.data, static_cast<size_t>(img.data.data.nCount));
                }
            }
        }
    }

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_matching");

    infuse_debug_tools::Asn1BitstreamToDisparity stereo_match;

    ros::spin();

    return 0;
}

