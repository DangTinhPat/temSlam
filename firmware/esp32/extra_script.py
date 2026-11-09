import os

Import("env")

# Biên dịch các thư viện dùng chung (control, core) cùng firmware ESP32.
env.BuildSources(
    os.path.join("$BUILD_DIR", "rb1_control"),
    os.path.join("$PROJECT_DIR", "control", "src"),
)
env.BuildSources(
    os.path.join("$BUILD_DIR", "rb1_core"),
    os.path.join("$PROJECT_DIR", "core", "src"),
)
