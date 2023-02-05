#include <cstdio>
#include <fstream>
#include <string>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/utils/filesystem.hpp>

void print_calibration(k4a_calibration_t &calib)
{
    printf("RGB Camera Intrinsics:\n");
    for (int i = 0; i < calib.color_camera_calibration.intrinsics.parameter_count; i++)
    {
        printf("%f ", calib.color_camera_calibration.intrinsics.parameters.v[i]);
    }
    printf("\n");
    printf("RGB Camera Extrinsics:\n");
    for (int i = 0; i < 9; i++)
    {
        printf("%10f ", calib.color_camera_calibration.extrinsics.rotation[i]);
        if (i % 3 == 2)
            printf("%10f\n", calib.color_camera_calibration.extrinsics.translation[i / 3]);
    }

    printf("Depth Camera Intrinsics:\n");
    for (int i = 0; i < calib.depth_camera_calibration.intrinsics.parameter_count; i++)
    {
        printf("%f ", calib.depth_camera_calibration.intrinsics.parameters.v[i]);
    }
    printf("\n");
    printf("Depth Camera Extrinsics:\n");
    for (int i = 0; i < 9; i++)
    {
        printf("%10f ", calib.depth_camera_calibration.extrinsics.rotation[i]);
        if (i % 3 == 2)
            printf("%10f\n", calib.depth_camera_calibration.extrinsics.translation[i / 3]);
    }
}

int getRGB(k4a_capture_t &capture, cv::Mat &rgb, uint64_t &timestamp)
{
    int ret = K4A_RESULT_SUCCEEDED;
    k4a_image_t img = nullptr;
    k4a_image_format_t rgb_format;
    uint8_t *rgb_buffer;
    size_t buffer_size;
    img = k4a_capture_get_color_image(capture);
    if (!img)
    {
        ret = K4A_RESULT_FAILED;
        printf("No RGB\n");
        goto error_rgb;
    }
    rgb_format = k4a_image_get_format(img);
    rgb_buffer = k4a_image_get_buffer(img);
    buffer_size = k4a_image_get_size(img);
    rgb = cv::imdecode(std::vector<uint8_t>(rgb_buffer, rgb_buffer + buffer_size), cv::IMREAD_ANYCOLOR);
    timestamp = k4a_image_get_device_timestamp_usec(img);
error_rgb:
    if (img)
        k4a_image_release(img);
    return ret;
}

int getDepth(k4a_capture_t &capture, k4a_transformation_t tf, int rgb_width, int rgb_height, cv::Mat &depth)
{
    int ret = K4A_RESULT_SUCCEEDED;
    k4a_image_t img = nullptr;
    k4a_image_t tf_depth = nullptr;
    img = k4a_capture_get_depth_image(capture);
    if (!img)
    {
        ret = -1;
        printf("No Depth\n");
        goto error_depth;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(
                                    K4A_IMAGE_FORMAT_DEPTH16,
                                    rgb_width, rgb_height,
                                    rgb_width * int(sizeof(uint16_t)),
                                    &tf_depth))
    {
        printf("Create transformed depth failed\n");
        ret = K4A_RESULT_FAILED;
        goto error_tf;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(tf, img, tf_depth))
    {
        printf("Transform failed\n");
        ret = K4A_RESULT_FAILED;
        goto error_tf;
    }

    depth = cv::Mat(rgb_height, rgb_width, CV_16U, (void *)k4a_image_get_buffer(tf_depth), cv::Mat::AUTO_STEP).clone();
error_tf:
    if (tf_depth)
        k4a_image_release(tf_depth);
error_depth:
    if (img)
        k4a_image_release(img);
    return ret;
}

int main(int argc, char *argv[])
{
    int ret = K4A_RESULT_SUCCEEDED;
    k4a_playback_t playback_handle = nullptr;
    k4a_calibration_t calibration;
    uint64_t rec_len;
    k4a_capture_t capture = nullptr;
    k4a_imu_sample_t imu_sample;
    k4a_stream_result_t stream_ret = K4A_STREAM_RESULT_SUCCEEDED;
    k4a_transformation_t tf;
    size_t frame_idx = 0;
    uint64_t timestamp = 0;
    cv::Mat rgb, depth;
    cv::Rect roi;
    std::ofstream of;
    std::string rgbDir, depthDir;

    if (argc != 5)
    {
        printf("input_path output_dir crop_width crop_height\n");
        return -1;
    }
    char *input_path = argv[1];
    char *output_dir = argv[2];
    int crop_width = atoi(argv[3]);
    int crop_height = atoi(argv[4]);
    int crop_left, crop_top;

    if (k4a_playback_open(argv[1], &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Recording Open Failed\n");
        ret = K4A_RESULT_FAILED;
        goto error_open_playback;
    }
    rec_len = k4a_playback_get_recording_length_usec(playback_handle);
    printf("Recording Length : %ld s\n", rec_len / 1000000);
    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback_handle, &calibration))
    {
        printf("Cannot load calibration info\n");
        ret = K4A_RESULT_FAILED;
        goto error_open_playback;
    }
    if (K4A_RESULT_SUCCEEDED != k4a_playback_seek_timestamp(playback_handle, 1000 * 1000, K4A_PLAYBACK_SEEK_BEGIN))
    {
        printf("Cannot skip frames in first %d secs\n", 1000);
        ret = K4A_RESULT_FAILED;
        goto error_open_playback;
    }
    print_calibration(calibration);

    tf = k4a_transformation_create(&calibration);

    rgbDir = cv::utils::fs::join(output_dir, "./rgb/");
    depthDir = cv::utils::fs::join(output_dir, "./depth/");
    if (!cv::utils::fs::exists(rgbDir))
    {
        if (!cv::utils::fs::createDirectories(rgbDir))
        {
            printf("Cannot create output dir\n");
            goto error_open_playback;
        }
        if (!cv::utils::fs::createDirectories(depthDir))
        {
            printf("Cannot create output dir\n");
            goto error_open_playback;
        }
    }

    while (stream_ret == K4A_STREAM_RESULT_SUCCEEDED)
    {

        stream_ret = k4a_playback_get_next_capture(playback_handle, &capture);
        if (stream_ret == K4A_STREAM_RESULT_SUCCEEDED)
        {
            if (K4A_RESULT_SUCCEEDED != getRGB(capture, rgb, timestamp) || timestamp == 0)
            {
                ret = K4A_RESULT_FAILED;
                goto error_capture;
            }
            if (K4A_RESULT_SUCCEEDED != getDepth(capture, tf, rgb.size[1], rgb.size[0], depth))
            {
                ret = K4A_RESULT_FAILED;
                goto error_capture;
            }
            printf("Frame: %ld; Timestamp: %ld\n", frame_idx, timestamp);
            crop_left = (rgb.size[1] - crop_width) / 2;
            crop_top = (rgb.size[0] - crop_height) / 2;
            roi = cv::Rect(crop_left, crop_top, crop_width, crop_height);
            cv::imwrite(cv::utils::fs::join(rgbDir, std::to_string(timestamp) + std::string(".png")), rgb(roi));
            cv::imwrite(cv::utils::fs::join(depthDir, std::to_string(timestamp) + std::string(".png")), depth(roi));
            k4a_capture_release(capture);
        }
        else if (stream_ret == K4A_STREAM_RESULT_EOF)
        {
            break;
        }
        else
        {
            ret = K4A_RESULT_FAILED;
            goto error_capture;
        }
        frame_idx++;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_seek_timestamp(playback_handle, 1000 * 1000, K4A_PLAYBACK_SEEK_BEGIN))
    {
        printf("Cannot skip frames in first %d secs\n", 1000);
        ret = K4A_RESULT_FAILED;
        goto error_open_playback;
    }

    frame_idx = 0;
    of.open(cv::utils::fs::join(output_dir, "./imu.txt"));
    if (!of.is_open())
    {
        printf("Cannot open imu file\n");
        goto error_imu_out;
    }
    stream_ret = K4A_STREAM_RESULT_SUCCEEDED;
    while (stream_ret == K4A_STREAM_RESULT_SUCCEEDED)
    {
        stream_ret = k4a_playback_get_next_imu_sample(playback_handle, &imu_sample);
        if (stream_ret == K4A_STREAM_RESULT_SUCCEEDED)
        {
            of << imu_sample.acc_timestamp_usec << " ";
            for (int i = 0; i < 3; i++)
            {
                of << imu_sample.acc_sample.v[i] << " ";
            }
            of << imu_sample.gyro_timestamp_usec << " ";
            for (int i = 0; i < 3; i++)
            {
                of << imu_sample.gyro_sample.v[i] << " ";
            }
            of << std::endl;
            printf("Sample Idx: %ld; Timestamp: %ld\n", frame_idx, imu_sample.acc_timestamp_usec);
        }
        else if (stream_ret == K4A_STREAM_RESULT_EOF)
        {
            break;
        }
        else
        {
            ret = K4A_RESULT_FAILED;
            break;
        }
        frame_idx++;
    }

error_imu_out:
    of.close();
error_capture:
    if (capture)
        k4a_capture_release(capture);
error_open_playback:
    if (playback_handle)
        k4a_playback_close(playback_handle);
    return ret;
}