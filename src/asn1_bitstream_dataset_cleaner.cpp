#include "asn1_bitstream_dataset_cleaner.hpp"

static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

namespace infuse_debug_tools
{

    Asn1BitstreamDatasetCleaner::Asn1BitstreamDatasetCleaner() :
        private_nh_{"~"},
        connect_image_srv_{private_nh_.advertiseService("connect_image", &Asn1BitstreamDatasetCleaner::connect_image, this)},
        asn1_image_ptr_{std::make_unique<asn1SccFramePair>()}
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

    bool Asn1BitstreamDatasetCleaner::connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
                                       infuse_debug_tools::ConnectTopic::Response &res)
    {
        try
        {
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

    void Asn1BitstreamDatasetCleaner::bind_subToPubs(std::string topic_in){

        std::string output_topic_cleaned = topic_in + "_Cleaned";

        // Create publisher for disparity image
        pub_map_[topic_in] = nh_.advertise<infuse_msgs::asn1_bitstream>(output_topic_cleaned, 100);

        // Bind publisher to the callback
        boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
                boost::bind(&Asn1BitstreamDatasetCleaner::image_callback, this, _1, boost::cref(pub_map_[topic_in]));

        // Create subscriber
        sub_map_[topic_in] = nh_.subscribe<sensor_msgs::Image>(topic_in, 100, callback);
        ROS_INFO_STREAM("Connected to topic " << topic_in << ". Publishing disparity image on " << output_topic_cleaned);

    }

    void Asn1BitstreamDatasetCleaner::image_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_asn1)
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
        double percentage;
        process_stereo_matching(*asn1_image_ptr_, percentage);

        double roll, pitch, yaw;
        // Process stereo matching
        process_quaternion_convert(*asn1_image_ptr_, roll, pitch, yaw);

        // Check if the pair is well synchronized regarding to disparity
        if(percentage > 70)
        {
            // Check if the pair is not a bad panorama part
            if(yaw >= -100 && yaw <= -80)
            {
                pub_asn1.publish(*msg);
            }
        }
    }

    void Asn1BitstreamDatasetCleaner::process_stereo_matching(asn1SccFramePair& in_frame_pair, double & percentage)
    {
        cv::Mat rect_left, rect_right;
        cv::Mat disparity;

        process_stereo_rectification(in_frame_pair, rect_left, rect_right);

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

        // Get the number of paired pixels and calculate percentage
        double nb_paired = 0;

        for (int x = 0; x<disparity.rows; x++)
        {
            for (int y = 0; y<disparity.cols; y++)
            {
                // Accesssing values of each pixel
                if ((disparity.at<int16_t>(x, y)/16) > 0)
                    nb_paired += 1;
            }
        }

        percentage = (nb_paired/(disparity.rows * disparity.cols))*100;
    }

    void Asn1BitstreamDatasetCleaner::process_stereo_rectification(asn1SccFramePair& in_original_stereo_pair, cv::Mat & out_rect_left, cv::Mat & out_rect_right)
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

        }
    }

    void Asn1BitstreamDatasetCleaner::process_quaternion_convert(asn1SccFramePair& in_frame_pair, double & roll, double & pitch, double & yaw)
    {
        double x = in_frame_pair.left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[0];
        double y = in_frame_pair.left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[1];
        double z = in_frame_pair.left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[2];
        double w = in_frame_pair.left.extrinsic.pose_robotFrame_sensorFrame.data.orientation.arr[3];

        Eigen::Quaterniond q(w, x, y, z);

        toEulerAngle(q, roll, pitch, yaw);

        roll = roll * (180.0/3.141592653589793238463);
        pitch = pitch * (180.0/3.141592653589793238463);
        yaw = yaw * (180.0/3.141592653589793238463);

    }

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
    ros::init(argc, argv, "asn1_bitstream_disparity_cleaner");

    infuse_debug_tools::Asn1BitstreamDatasetCleaner asn1_bitstream_disparity_cleaner;

    ros::spin();

    return 0;
}

