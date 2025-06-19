#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <thread>

// FFmpeg C includes
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
}

// Frame queue for buffering
struct FrameData {
    AVFrame* frame;
    int64_t pts;
    int64_t dts;
    int stream_index;
};

class FrameQueue {
private:
    std::queue<FrameData> queue;
    std::mutex mutex;
    std::condition_variable cond;
    size_t max_size;
    std::atomic<bool> running{true};
    std::atomic<int> dropped_frames{0};
    const int DROP_THRESHOLD = 5;  // Number of frames to drop when queue is full
    
    // Drop the oldest frame
    void drop_oldest() {
        if (!queue.empty()) {
            FrameData& oldest = queue.front();
            av_frame_free(&oldest.frame);
            queue.pop();
            dropped_frames++;
            if (dropped_frames % 10 == 0) {
                ROS_WARN("Dropped %d frames so far", dropped_frames.load());
            }
        }
    }

public:
    FrameQueue(size_t size) : max_size(size) {}
    
    ~FrameQueue() {
        clear();
    }
    
    bool push(FrameData&& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        
        // If queue is full, drop oldest frames to make room
        while (queue.size() >= max_size && running) {
            drop_oldest();
        }
        
        if (!running) {
            av_frame_free(&frame.frame);
            return false;
        }
        
        queue.push(std::move(frame));
        cond.notify_one();
        return true;
    }
    
    // Clear all frames from the queue
    void clear() {
        std::unique_lock<std::mutex> lock(mutex);
        while (!queue.empty()) {
            FrameData& data = queue.front();
            av_frame_free(&data.frame);
            queue.pop();
        }
    }
    
    bool pop(FrameData& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        while (queue.empty() && running) {
            if (cond.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::timeout) {
                return false;
            }
        }
        if (!running) return false;
        frame = std::move(queue.front());
        queue.pop();
        return true;
    }
    
    void stop() {
        running = false;
        cond.notify_all();
    }
    
    size_t size() const {
        return queue.size();
    }
};

// Global debug variables
std::atomic<uint64_t> frame_count{0};
std::atomic<uint64_t> dropped_frames{0};
std::chrono::steady_clock::time_point start_time;

void log_debug(const std::string& message) {
    auto now = std::chrono::steady_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    ROS_DEBUG("[%luus] %s", us, message.c_str());
}

int main(int argc, char **argv) {
    start_time = std::chrono::steady_clock::now();
    ros::init(argc, argv, "pluto_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Initialize frame queue with smaller size to reduce latency
    const int QUEUE_SIZE = 10;  // Reduced from 50 to 10
    FrameQueue frame_queue(QUEUE_SIZE);
    
    // Initialize ROS components
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("plutocamera/image_raw", 1);

    std::string camera_info_url;
    private_nh.param<std::string>("camera_info_url", camera_info_url, "");
    camera_info_manager::CameraInfoManager cinfo(nh, "plutocamera", camera_info_url);

    // Camera stream URL
    const char *url = "tcp://192.168.1.1:8080";
    log_debug("Initializing camera stream");

    // FFmpeg variables
    AVFormatContext *pFormatCtx = nullptr;
    AVCodecContext *pCodecCtx = nullptr;
    const AVCodec *pCodec = nullptr;
    AVFrame *pFrame = nullptr;
    AVFrame *pFrameBGR = nullptr;
    AVPacket packet;
    struct SwsContext *sws_ctx = nullptr;
    int videoStream = -1;
    
    // Set HD flag for better quality
    log_debug("HD flag set, starting video stream");

    avformat_network_init();

    if (avformat_open_input(&pFormatCtx, url, nullptr, nullptr) != 0) {
        ROS_ERROR("Couldn't open input stream.");
        return -1;
    }

    if (avformat_find_stream_info(pFormatCtx, nullptr) < 0) {
        ROS_ERROR("Couldn't find stream information.");
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    for (unsigned int i = 0; i < pFormatCtx->nb_streams; i++) {
        if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStream = i;
            break;
        }
    }

    if (videoStream == -1) {
        ROS_ERROR("Didn't find a video stream.");
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    pCodec = avcodec_find_decoder(pFormatCtx->streams[videoStream]->codecpar->codec_id);
    if (pCodec == nullptr) {
        ROS_ERROR("Unsupported codec!");
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    pCodecCtx = avcodec_alloc_context3(pCodec);
    if (avcodec_parameters_to_context(pCodecCtx, pFormatCtx->streams[videoStream]->codecpar) < 0) {
        ROS_ERROR("Couldn't copy codec context.");
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    if (avcodec_open2(pCodecCtx, pCodec, nullptr) < 0) {
        ROS_ERROR("Could not open codec.");
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    pFrame = av_frame_alloc();
    pFrameBGR = av_frame_alloc();
    if (!pFrame || !pFrameBGR) {
        ROS_ERROR("Could not allocate video frame.");
        if(pFrame) av_frame_free(&pFrame);
        if(pFrameBGR) av_frame_free(&pFrameBGR);
        avcodec_close(pCodecCtx);
        avcodec_free_context(&pCodecCtx);
        avformat_close_input(&pFormatCtx);
        return -1;
    }

    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height, 1);
    uint8_t *buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, buffer, AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height, 1);

    sws_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
                             pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_BGR24,
                             SWS_BILINEAR, nullptr, nullptr, nullptr);

    ROS_INFO("Successfully initialized video stream. Starting to publish frames...");

    // Start frame processing thread
    auto processing_thread = std::make_shared<std::thread>([&]() {
        FrameData frame_data;
        while (ros::ok() && frame_queue.pop(frame_data)) {
            try {
                // Convert frame to BGR
                sws_scale(sws_ctx, 
                         (uint8_t const *const *)frame_data.frame->data, 
                         frame_data.frame->linesize, 
                         0, pCodecCtx->height,
                         pFrameBGR->data, pFrameBGR->linesize);

                // Create OpenCV Mat from the frame
                cv::Mat image(pCodecCtx->height, pCodecCtx->width, CV_8UC3, pFrameBGR->data[0], pFrameBGR->linesize[0]);
                
                // Set camera info if not calibrated
                if (!cinfo.isCalibrated()) {
                    sensor_msgs::CameraInfo camera_info;
                    camera_info.width = pCodecCtx->width;
                    camera_info.height = pCodecCtx->height;
                    cinfo.setCameraInfo(camera_info);
                }

                // Create and publish ROS message
                sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo.getCameraInfo()));
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                
                ros::Time now = ros::Time::now();
                msg->header.stamp = now;
                ci->header.stamp = now;
                msg->header.frame_id = "pluto_camera";
                ci->header.frame_id = "pluto_camera";
                
                pub.publish(msg, ci);
                frame_count++;
                
                if (frame_count % 30 == 0) {
                    ROS_DEBUG("Published frame %lu (queue size: %zu)", frame_count.load(), frame_queue.size());
                }
                
                // Free the frame
                av_frame_unref(frame_data.frame);
                av_frame_free(&frame_data.frame);
            } catch (const std::exception& e) {
                ROS_ERROR("Error processing frame: %s", e.what());
            }
        }
    });

    // Main frame reading loop
    log_debug("Starting main frame reading loop");
    int64_t last_frame_time = 0;
    int consecutive_drops = 0;
    const int MAX_CONSECUTIVE_DROPS = 30;  // About 1 second at 30fps
    
    while (ros::ok()) {
        if (av_read_frame(pFormatCtx, &packet) >= 0) {
            if (packet.stream_index == videoStream) {
                // Create a new frame for the queue
                AVFrame* frame = av_frame_alloc();
                if (!frame) {
                    ROS_ERROR("Failed to allocate frame");
                    av_packet_unref(&packet);
                    continue;
                }
                
                // Send packet to decoder
                int ret = avcodec_send_packet(pCodecCtx, &packet);
                if (ret < 0) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    ROS_ERROR("Error sending packet to decoder: %s", errbuf);
                    av_frame_free(&frame);
                    av_packet_unref(&packet);
                    continue;
                }
                
                // Receive frame from decoder
                ret = avcodec_receive_frame(pCodecCtx, frame);
                if (ret == 0) {
                    // Calculate frame timing
                    int64_t current_time = av_gettime_relative();
                    if (last_frame_time > 0) {
                        int64_t frame_interval = current_time - last_frame_time;
                        if (frame_interval > 50000) { // 50ms threshold for frame drops
                            ROS_WARN("Frame interval too large: %ld us", frame_interval);
                        }
                    }
                    last_frame_time = current_time;
                    
                    // Add frame to queue with frame dropping logic
                    FrameData frame_data{frame, frame->pts, frame->pkt_dts, packet.stream_index};
                    if (!frame_queue.push(std::move(frame_data))) {
                        consecutive_drops++;
                        if (consecutive_drops > MAX_CONSECUTIVE_DROPS) {
                            ROS_ERROR("Too many consecutive drops, possible stream issue");
                            break;
                        }
                        av_frame_free(&frame);
                    } else {
                        consecutive_drops = 0;  // Reset counter on successful push
                    }
                } else if (ret != AVERROR(EAGAIN)) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    ROS_ERROR("Error receiving frame from decoder: %s", errbuf);
                    av_frame_free(&frame);
                }
            }
            av_packet_unref(&packet);
        }
        ros::spinOnce();
    }
    
    // Cleanup
    log_debug("Shutting down...");
    frame_queue.stop();
    if (processing_thread && processing_thread->joinable()) {
        processing_thread->join();
    }

    ROS_INFO("Shutting down node.");

    av_free(buffer);
    av_frame_free(&pFrameBGR);
    av_frame_free(&pFrame);
    avcodec_close(pCodecCtx);
    avcodec_free_context(&pCodecCtx);
    avformat_close_input(&pFormatCtx);
    avformat_network_deinit();

    return 0;
}
