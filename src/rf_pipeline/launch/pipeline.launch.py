from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file    = LaunchConfiguration("config_file")
    use_sim_time   = LaunchConfiguration("use_sim_time")
    iq_topic       = LaunchConfiguration("iq_topic")
    stft_win_size  = LaunchConfiguration("stft_win_size")
    stft_nfft      = LaunchConfiguration("stft_nfft")
    stft_hop       = LaunchConfiguration("stft_hop")
    model_path     = LaunchConfiguration("model_path")
    conf_thresh    = LaunchConfiguration("conf_thresh")
    scores_topic   = LaunchConfiguration("scores_topic")
    scoreboard_agg = LaunchConfiguration("scoreboard_agg")

    return LaunchDescription([
        DeclareLaunchArgument("config_file",    default_value=""),
        DeclareLaunchArgument("use_sim_time",   default_value="false"),
        DeclareLaunchArgument("iq_topic",       default_value="iq"),
        DeclareLaunchArgument("scores_topic",   default_value="scores"),
        DeclareLaunchArgument("stft_win_size",  default_value="2048"),
        DeclareLaunchArgument("stft_nfft",      default_value="2048"),
        DeclareLaunchArgument("stft_hop",       default_value="512"),
        DeclareLaunchArgument("model_path",     default_value=str(
            get_package_share_path("rf_pipeline") / "resource" / "best.torchscript"
        )),
        DeclareLaunchArgument("conf_thresh",    default_value="0.5"),
        DeclareLaunchArgument("scoreboard_agg", default_value="max"),

        Node(
            package="rf_pipeline",
            executable="rf_detector",
            name="rf_detector_node",
            output="screen",
            parameters=[{
                "nfft":         stft_nfft,
                "hop":          stft_hop,
                "win_size":     stft_win_size,
                "model_path":   model_path,
                "conf_thresh":  conf_thresh,
                "use_sim_time": use_sim_time,
            }, config_file],
            remappings=[("iq", iq_topic)],
        ),
        Node(
            package="rf_pipeline",
            executable="scoreboard",
            name="scoreboard_node",
            output="screen",
            parameters=[{
                "aggregation":  scoreboard_agg,
                "use_sim_time": use_sim_time,
            }, config_file],
            remappings=[("scores", scores_topic)],
        ),
    ])
