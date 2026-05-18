#include <complex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rf_msgs/msg/iq_frame.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/tune_request.hpp>

#include <boost/thread.hpp>
#include <boost/thread/interruption.hpp>

class UsrpDriverNode : public rclcpp::Node
{
public:
    UsrpDriverNode() : Node("usrp_driver_node")
    {
        device_args_ = declare_parameter<std::string>("device_args", "");
        freq_hz_ = declare_parameter<double>("freq_hz", 2.4e9);
        rate_hz_ = declare_parameter<double>("rate_hz", 1.0e6);
        gain_db_ = declare_parameter<double>("gain_db", 10.0);
        channel_ = static_cast<size_t>(declare_parameter<int>("channel", 0));

        pub_ = create_publisher<rf_msgs::msg::IqFrame>("iq", 10);

        worker_ = boost::thread(&UsrpDriverNode::stream_worker, this);
    }

    ~UsrpDriverNode()
    {
        worker_.interrupt();
        if (worker_.joinable())
            worker_.join();
    }

private:
    void stream_worker()
    {
        auto usrp = uhd::usrp::multi_usrp::make(device_args_);
        usrp->set_rx_rate(rate_hz_, channel_);
        usrp->set_rx_freq(uhd::tune_request_t(freq_hz_), channel_);
        usrp->set_rx_gain(gain_db_, channel_);

        const double fs_hz = usrp->get_rx_rate(channel_);
        const double fc_hz = usrp->get_rx_freq(channel_);

        uhd::stream_args_t st_args("fc32", "sc16");
        st_args.channels = {channel_};
        auto streamer = usrp->get_rx_stream(st_args);

        const size_t buffer_size = streamer->get_max_num_samps();
        std::vector<std::complex<float>> recv_buf(buffer_size);

        RCLCPP_INFO(get_logger(),
                    "USRP driver started: fs=%.2f MHz  fc=%.4f GHz  gain=%.1f dB  buffer=%zu samp",
                    fs_hz / 1e6, fc_hz / 1e9, gain_db_, buffer_size);

        uhd::stream_cmd_t start_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        start_cmd.stream_now = true;
        streamer->issue_stream_cmd(start_cmd);

        uhd::rx_metadata_t metadata;

        try
        {
            while (true)
            {
                boost::this_thread::interruption_point();
                size_t nsamples = streamer->recv(recv_buf.data(), buffer_size, metadata);
                if (metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
                    RCLCPP_WARN(get_logger(), "RX error: %s", metadata.strerror().c_str());
                if (nsamples)
                {
                    // reinterpret complex<float> buffer as interleaved float32 [I0,Q0,I1,Q1,…]
                    const float *samples = reinterpret_cast<const float *>(recv_buf.data());
                    publish(samples, nsamples, fs_hz, fc_hz);
                }
            }
        }
        catch (const boost::thread_interrupted&)
        {
            RCLCPP_INFO(this->get_logger(), "streaming interrupted, exiting");
        }

        uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
        streamer->issue_stream_cmd(stop_cmd);
    }

    void publish(const float *samples, size_t nsamples, double fs_hz, double fc_hz)
    {
        auto msg = rf_msgs::msg::IqFrame();
        msg.stamp = now();
        msg.fs_hz = fs_hz;
        msg.fc_hz = fc_hz;
        msg.data.assign(samples, samples + nsamples * 2);

        pub_->publish(msg);
    }

    std::string device_args_;
    double freq_hz_;
    double rate_hz_;
    double gain_db_;
    size_t channel_;

    rclcpp::Publisher<rf_msgs::msg::IqFrame>::SharedPtr pub_;
    boost::thread worker_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsrpDriverNode>());
    rclcpp::shutdown();
    return 0;
}
