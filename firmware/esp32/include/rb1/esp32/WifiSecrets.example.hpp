#pragma once
// Copy to WifiSecrets.hpp (git-ignored) and fill in. Without credentials the AP remains disabled.
#define RB1_WIFI_STA_SSID "your-network"
#define RB1_WIFI_STA_PASS "your-password"
// Optional test-site network: preferred when visible during the startup scan.
// Reboot after moving between sites. Passwords stay in the ignored local file.
// #define RB1_WIFI_ALT_SSID "your-test-network"
// #define RB1_WIFI_ALT_PASS "your-test-password"

// Optional highest-priority network, selected ahead of ALT and primary when visible.
// #define RB1_WIFI_ALT2_SSID "your-priority-network"
// #define RB1_WIFI_ALT2_PASS "your-priority-password"
