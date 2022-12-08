from pyautd3.extra import Simulator


if __name__ == '__main__':
    Simulator().settings_path("settings.json").vsync(True).gpu_idx(0).run()