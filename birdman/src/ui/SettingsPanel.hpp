#pragma once
// 飛行モードの設定パネル(⚙から開く)。タブ: 飛行シミュ / 風・突風
#include "core/Types.hpp"
#include "ui/Widgets.hpp"
#include <functional>

namespace bm {

struct SettingsCallbacks {
    std::function<void()> onStart, onReset, onChange;
    std::function<void(const std::string&)> onMode;   // "platform"/"runway"
    std::function<void(const std::string&)> onSite;   // "biwa"/"fujikawa"
    std::function<bool()> soundOn;                    // サウンド状態取得
    std::function<void()> onToggleSound;
    std::function<bool()> locked;                     // 大会中=true(チート系操作を禁止)
};

class SettingsPanel {
public:
    void build(SimParams* prm, AircraftParams* st,
               std::function<const Analysis*()> getAn, SettingsCallbacks cb);
    bool handleEvent(const sf::Event& ev, sf::Vector2f mouse);
    void draw(sf::RenderTarget& rt, const sf::Font& font, float H);
    bool open = false;

private:
    void syncLabels();
    SimParams* prm_ = nullptr;
    AircraftParams* st_ = nullptr;
    std::function<const Analysis*()> getAn_;
    SettingsCallbacks cb_;
    int tab_ = 0;                       // 0=シミュ, 1=風・突風
    Button tabSim_, tabWind_, bClose_;
    // シミュタブ(発進方式はサイトで自動決定のためボタンなし)
    Button bSiteBiwa_, bSiteFuji_, bGo_, bRst_, bAuto_, bHold_, bGhost_, bPjit_, bSixdof_, bSound_, bStamina_, bFun_;
    std::vector<Slider> simSliders_;
    // 風タブ
    Button bGacha_, bSummer_, bTerrainWind_, bWindVis_, bRealThermal_;
    std::vector<Slider> windSliders_;
    float x_ = 10, y_ = 94, w_ = 340;
    float scroll_ = 0, contentH_ = 600;
};

} // namespace bm
