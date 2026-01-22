#include "rerun_loop.hpp"

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: ./rerun <replay_file> <replay_speed> <skip_time> <mode>" << std::endl;
        return -1;
    }

    std::string replay_file = argv[1];
    std::string replay_speed = argv[2];
    std::string replay_skip_time = argv[3];
    std::string mode = argv[4];

    // // 你可以选择运行 ReplayViewer 或 Loopdetection
    if (mode == "0") {
        ReplayViewer viewer(replay_file, replay_speed, replay_skip_time);
        viewer.Play();}
    else if (mode == "1") {
        Loopdetection detector(replay_file, replay_speed, replay_skip_time);
        detector.RunPlay();
    }

    // // // // 或者：
    // Loopdetection detector(replay_file, replay_speed, replay_skip_time);
    // detector.RunPlay();

    return 0;
}