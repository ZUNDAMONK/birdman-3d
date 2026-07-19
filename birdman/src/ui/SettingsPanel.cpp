#include "ui/SettingsPanel.hpp"
#include "ui/Theme.hpp"
#include "core/Weather.hpp"
#include "core/Aircraft.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace bm {

// テーマ色は関数経由(flight=ライトに追従)
static sf::Color INK()     { return theme::text(); }
static sf::Color INKSOFT() { return theme::textDim(); }
static sf::Color OKC()     { return theme::ok(); }
static sf::Color BADC()    { return theme::bad(); }

static Slider mkS(const std::string& label, double* f, double mn, double mx, double stp,
                  const std::string& unit, std::function<void()> onChange = nullptr) {
    Slider s;
    s.label = label; s.unit = unit; s.minV = mn; s.maxV = mx; s.step = stp;
    s.get = [f] { return *f; };
    s.set = [f](double v) { *f = v; };
    s.onChange = onChange;
    return s;
}

void SettingsPanel::build(SimParams* prm, AircraftParams* st,
                          std::function<const Analysis*()> getAn, SettingsCallbacks cb) {
    prm_ = prm; st_ = st; getAn_ = getAn; cb_ = cb;
    tabSim_.label = u8"飛行シミュ";
    tabWind_.label = u8"風・突風";
    tabSim_.onClick = [this] { tab_ = 0; };
    tabWind_.onClick = [this] { tab_ = 1; };

    bClose_.label = u8"×"; bClose_.style = 3; bClose_.charSize = 14;
    bClose_.onClick = [this] { open = false; };
    // 発進方式はサイトで一意に決まる: 琵琶湖=プラットフォーム発進 / 富士川=滑走路離陸
    auto selectSite = [this](const std::string& s) {
        prm_->site = s;
        prm_->mode = (s == "fujikawa") ? "runway" : "platform";
        if (cb_.onMode) cb_.onMode(prm_->mode);
        if (cb_.onSite) cb_.onSite(prm_->site);
    };
    bSiteBiwa_.label = u8"琵琶湖会場";
    bSiteBiwa_.onClick = [this, selectSite] { selectSite("biwa"); };
    bSiteFuji_.label = u8"富士川飛行場";
    bSiteFuji_.onClick = [this, selectSite] { selectSite("fujikawa"); };
    bGo_.label = u8"▶ 発進"; bGo_.style = 1;
    bGo_.onClick = [this] { if (cb_.onStart) cb_.onStart(); };
    bRst_.label = u8"■ リセット";
    bRst_.onClick = [this] { if (cb_.onReset) cb_.onReset(); };
    bAuto_.onClick = [this] { prm_->auto_ = !prm_->auto_; };
    bHold_.onClick = [this] { if (!prm_->auto_) prm_->hold = !prm_->hold; };
    bGhost_.onClick = [this] { prm_->ghost = !prm_->ghost; };
    bPjit_.onClick = [this] { prm_->pjit = !prm_->pjit; };
    bSixdof_.onClick = [this] { prm_->sixdof = !prm_->sixdof; };
    bAssist_.onClick = [this] { if (prm_->sixdof) prm_->assist = !prm_->assist; };
    bSound_.onClick = [this] { if (cb_.onToggleSound) cb_.onToggleSound(); };
    bTerrainWind_.onClick = [this] { prm_->terrainWind = !prm_->terrainWind; };
    bWindVis_.onClick = [this] { prm_->windVis = !prm_->windVis; };
    bStamina_.onClick = [this] { prm_->stamina = !prm_->stamina; };
    bRealThermal_.onClick = [this] { prm_->realThermal = !prm_->realThermal; };
    bFun_.onClick = [this, selectSite] {
        prm_->funPlane = !prm_->funPlane;
        // お遊び機は滑走路専用 → 富士川飛行場へ自動切替(OFFにしてもサイトは戻さない)
        if (prm_->funPlane && prm_->site != "fujikawa") selectSite("fujikawa");
    };

    simSliders_.clear();
    simSliders_.push_back(mkS(u8"パイロット出力(飛行中も変更可)", &prm->P, 0, 700, 10, " W"));
    simSliders_.push_back(mkS(u8"発進速度(対地・プラットフォーム)", &prm->V0, 0, 10, 0.1, " m/s"));
    simSliders_.push_back(mkS(u8"プッシャー発進速度(滑走路の初速)", &prm->pushV, 0, 8, 0.1, " m/s"));
    simSliders_.push_back(mkS(u8"巡航目標高度(ホールド/オート時)", &prm->hTgt, 0.5, 6, 0.1, " m"));
    {
        Slider spd;
        spd.label = u8"再生速度"; spd.unit = u8" ×";
        spd.minV = 1; spd.maxV = 40; spd.step = 1;
        SimParams* p = prm;
        spd.get = [p] { return (double)p->speed; };
        spd.set = [p](double v) { p->speed = (int)std::lround(v); };
        simSliders_.push_back(spd);
    }
    simSliders_.push_back(mkS(u8"無酸素容量 W'(オート用)", &prm->wcap, 8, 30, 1, " kJ"));
    simSliders_.push_back(mkS(u8"操縦感度", &prm->sens, 0.3, 1.5, 0.05, u8"×"));

    auto wchg = [this] {
        if (prm_->summer) applySummer(*prm_); else applyWindVec(*prm_);
    };
    bGacha_.label = u8"天気ガチャ";
    bGacha_.onClick = [this, wchg] {
        prm_->weather = rollWeather(prm_->site);
        prm_->dirJit = prm_->tempJit = prm_->wspdJit = prm_->gustJit = 0;
        wchg();
    };
    bSummer_.onClick = [this, wchg] { prm_->summer = !prm_->summer; wchg(); };

    windSliders_.clear();
    windSliders_.push_back(mkS(u8"風向(0°=向かい風/180°=追い風)", &prm->wdir, 0, 360, 5, u8"°", [this] { applyWindVec(*prm_); }));
    windSliders_.push_back(mkS(u8"風速", &prm->wspd, 0, 8, 0.1, " m/s", [this] { applyWindVec(*prm_); }));
    windSliders_.push_back(mkS(u8"時刻(早朝〜午前が勝負)", &prm->tod, 5, 18, 0.5, u8"時", [this] { if (prm_->summer) applySummer(*prm_); }));
    windSliders_.push_back(mkS(u8"気温(大気密度に影響)", &prm->temp, -5, 38, 1, u8" ℃"));
    windSliders_.push_back(mkS(u8"乱流の強さ(低空ほど強い)", &prm->turb, 0, 1.5, 0.05, u8"×"));
    windSliders_.push_back(mkS(u8"サーマル(上昇気流の強さ)", &prm->thermal, 0, 1.2, 0.05, u8"×"));
    windSliders_.push_back(mkS(u8"垂直突風", &prm->gust, 0, 3, 0.1, " m/s"));
    windSliders_.push_back(mkS(u8"CLmax(失速揚力係数)", &prm->CLmax, 1.2, 1.6, 0.05, ""));
}

void SettingsPanel::syncLabels() {
    // 飛行中は再構築が必要な設定を固定。出力・再生速度・音・風表示だけ変更可。
    const bool lock = cb_.locked && cb_.locked();
    bAuto_.label = std::string(u8"性能計測オート: ") + (lock ? u8"次回発進から" : prm_->auto_ ? "ON" : "OFF");
    bAuto_.style = prm_->auto_ ? 2 : 0;
    bAuto_.enabled = !lock;
    bHold_.label = std::string(u8"高度ホールド: ") + (lock ? u8"次回発進から" : prm_->auto_ ? u8"(オート)" : prm_->hold ? "ON" : "OFF");
    bHold_.style = (prm_->hold && !prm_->auto_) ? 2 : 0;
    bHold_.enabled = !prm_->auto_ && !lock;
    bSummer_.enabled = !lock;
    bGacha_.enabled = !lock;
    bTerrainWind_.enabled = !lock;
    bRealThermal_.enabled = !lock;
    bSixdof_.enabled = !lock;
    bAssist_.enabled = prm_->sixdof && !lock;
    bStamina_.enabled = !lock;
    bFun_.enabled = !lock;
    bGhost_.enabled = !lock;
    bPjit_.enabled = !lock;
    bFun_.label = std::string(u8"お遊び: 小型プロペラ機(富士川専用) ") + (prm_->funPlane ? "ON" : "OFF");
    bFun_.style = prm_->funPlane ? 2 : 0;
    for (auto& s : windSliders_) s.enabled = !lock;
    for (size_t i = 0; i < simSliders_.size(); ++i)
        simSliders_[i].enabled = !lock || i == 0 || i == 4; // 出力/再生速度のみ飛行中可
    bGhost_.label = std::string(u8"ゴースト: ") + (prm_->ghost ? "ON" : "OFF");
    bGhost_.style = prm_->ghost ? 2 : 0;
    bPjit_.label = std::string(u8"出力ゆらぎ: ") + (prm_->pjit ? "ON" : "OFF");
    bPjit_.style = prm_->pjit ? 2 : 0;
    bSummer_.label = (prm_->site == "fujikawa" ? std::string(u8"リアル気象モード(蒲原): ")
                                                : std::string(u8"夏の大会モード: "))
                     + (prm_->summer ? "ON" : "OFF");
    bSummer_.style = prm_->summer ? 2 : 0;
    bSixdof_.label = std::string(u8"物理: ") + (prm_->sixdof ? u8"拡張物理(回転モデル)" : u8"標準物理(経路モデル)");
    bSixdof_.style = prm_->sixdof ? 2 : 0;
    bAssist_.label = prm_->sixdof
                   ? std::string(u8"操縦補助: ") + (prm_->assist ? "ON" : "OFF")
                   : u8"操縦補助: 常時補助相当(経路モデル)";
    bAssist_.style = prm_->sixdof && prm_->assist ? 2 : 0;
    const bool snd = cb_.soundOn ? cb_.soundOn() : true;
    bSound_.label = std::string(u8"サウンド: ") + (snd ? "ON" : "OFF");
    bSound_.style = snd ? 2 : 0;
    bTerrainWind_.label = (prm_->site == "fujikawa" ? std::string(u8"地形風(谷風・河原サーマル): ")
                                                     : std::string(u8"地形風(比良おろし・岸サーマル): "))
                          + (prm_->terrainWind ? "ON" : "OFF");
    bTerrainWind_.style = prm_->terrainWind ? 2 : 0;
    bWindVis_.label = std::string(u8"風の可視化(粒子): ") + (prm_->windVis ? "ON" : "OFF");
    bWindVis_.style = prm_->windVis ? 2 : 0;
    bStamina_.label = std::string(u8"体力モデル(手動出力に適用): ") + (prm_->stamina ? "ON" : "OFF");
    bStamina_.style = prm_->stamina ? 2 : 0;
    bRealThermal_.label = std::string(u8"サーマル: ") + (prm_->realThermal ? u8"動的セル(リアル)" : u8"固定(JS互換)");
    bRealThermal_.style = prm_->realThermal ? 2 : 0;
    bSiteBiwa_.enabled = !lock;
    bSiteFuji_.enabled = !lock;
    bSiteBiwa_.on = prm_->site == "biwa";
    bSiteFuji_.on = prm_->site == "fujikawa";
    tabSim_.style = tab_ == 0 ? 2 : 0;
    tabWind_.style = tab_ == 1 ? 2 : 0;
}

bool SettingsPanel::handleEvent(const sf::Event& ev, sf::Vector2f m) {
    if (!open) return false;
    const sf::FloatRect panel(x_, y_, w_, contentH_);
    bool consumed = false;
    if (ev.type == sf::Event::MouseWheelScrolled && panel.contains(m)) {
        scroll_ = std::max(0.f, scroll_ - ev.mouseWheelScroll.delta * 40);
        return true;
    }
    consumed |= tabSim_.handle(ev, m);
    consumed |= tabWind_.handle(ev, m);
    consumed |= bClose_.handle(ev, m);
    if (tab_ == 0) {
        for (Button* b : {&bSiteBiwa_, &bSiteFuji_, &bGo_, &bRst_, &bAuto_, &bHold_, &bGhost_, &bPjit_, &bSixdof_, &bAssist_, &bSound_, &bStamina_, &bFun_})
            consumed |= b->handle(ev, m);
        for (auto& s : simSliders_) consumed |= s.handle(ev, m);
    } else {
        consumed |= bGacha_.handle(ev, m);
        consumed |= bSummer_.handle(ev, m);
        consumed |= bTerrainWind_.handle(ev, m);
        consumed |= bWindVis_.handle(ev, m);
        consumed |= bRealThermal_.handle(ev, m);
        for (auto& s : windSliders_) consumed |= s.handle(ev, m);
    }
    if (!consumed && ev.type == sf::Event::MouseButtonPressed && panel.contains(m))
        consumed = true;
    return consumed;
}

void SettingsPanel::draw(sf::RenderTarget& rt, const sf::Font& font, float H) {
    if (!open) return;
    syncLabels();
    const float maxH = H - y_ - 40;
    float y = y_ + 10 - scroll_;
    const float x = x_ + 10, w = w_ - 20;
    // タブ(閉じるボタン分だけ幅を詰める)
    tabSim_.rect = {x, y, w / 2 - 26, 28};
    tabWind_.rect = {x + w / 2 - 22, y, w / 2 - 26, 28};
    bClose_.rect = {x + w - 26, y, 26, 28};
    y += 38;
    float innerTop = y;
    // 見出し等のテキストは背景パネル描画後に描く必要があるため、位置だけ記録して遅延描画
    struct DeferredText { std::string s; float y; unsigned size; bool bold; };
    std::vector<DeferredText> texts;
    auto heading = [&](const char* label) {
        texts.push_back({label, y, 10, true});
        y += 16;
    };
    if (tab_ == 0) {
        heading(u8"発進");
        bSiteBiwa_.rect = {x, y, w / 2 - 3, 30};
        bSiteFuji_.rect = {x + w / 2 + 3, y, w / 2 - 3, 30}; y += 36;
        // 発進方式はサイトで自動決定(琵琶湖=プラットフォーム/富士川=滑走路)なのでボタンなし
        texts.push_back({prm_->site == "fujikawa" ? u8"発進方式: 滑走路離陸(プッシャー)"
                                                   : u8"発進方式: プラットフォーム発進",
                         y + 2, 11, false});
        y += 22;
        bGo_.rect = {x, y, 90, 30};
        bRst_.rect = {x + 96, y, 100, 30}; y += 40;
        heading(u8"操縦アシスト");
        bAuto_.rect = {x, y, w / 2 - 3, 28};
        bHold_.rect = {x + w / 2 + 3, y, w / 2 - 3, 28}; y += 34;
        bGhost_.rect = {x, y, w / 2 - 3, 28};
        bPjit_.rect = {x + w / 2 + 3, y, w / 2 - 3, 28}; y += 38;
        heading(u8"物理・サウンド");
        bSixdof_.rect = {x, y, w * 0.58f - 3, 28};
        bSound_.rect = {x + w * 0.58f + 3, y, w * 0.42f - 3, 28}; y += 34;
        bAssist_.rect = {x, y, w, 28}; y += 34;
        bStamina_.rect = {x, y, w, 28}; y += 34;
        bFun_.rect = {x, y, w, 28}; y += 38;
        heading(u8"パラメータ");
        for (auto& s : simSliders_) { s.rect = {x, y, w, 34}; y += 40; }
        y += 22;
    } else {
        // 天気ガチャ+現在の天候
        y += 58;
        bGacha_.rect = {x + w - 110, innerTop + 6, 100, 26};
        bSummer_.rect = {x, y, w, 28}; y += 34;
        bTerrainWind_.rect = {x, y, w, 28}; y += 34;
        bWindVis_.rect = {x, y, w, 28}; y += 34;
        bRealThermal_.rect = {x, y, w, 28}; y += 40;
        for (auto& s : windSliders_) { s.rect = {x, y, w, 34}; y += 40; }
        y += 130;   // 静解析リードアウト分
    }
    contentH_ = std::min(maxH, y + scroll_ - y_ + 10);
    // 背景
    drawPanelRect(rt, {x_, y_, w_, contentH_}, theme::panel(), theme::line(), 1);
    // 中身
    tabSim_.draw(rt, font);
    tabWind_.draw(rt, font);
    bClose_.draw(rt, font);
    for (const auto& t : texts) drawText(rt, font, t.s, x, t.y, t.size, INKSOFT(), 0, t.bold);
    if (tab_ == 0) {
        for (Button* b : {&bSiteBiwa_, &bSiteFuji_, &bGo_, &bRst_, &bAuto_, &bHold_, &bGhost_, &bPjit_, &bSixdof_, &bAssist_, &bSound_, &bStamina_, &bFun_})
            b->draw(rt, font);
        for (auto& s : simSliders_) s.draw(rt, font);
        drawText(rt, font, u8"操縦: ↑↓=エレベーター ←→=エルロン A/D=ラダー R/F=出力 G=フラップ Z/X=ペラピッチ",
                 x, simSliders_.back().rect.top + 42, 10, INKSOFT());
    } else {
        // 現在の天候(サイト対応)
        const int wi = prm_->weather;
        const auto& wlist = siteWeather(prm_->site);
        const WeatherPreset& wx = wlist[wi >= 0 && wi < (int)wlist.size() ? wi : 1];
        const std::string wtitle = prm_->site == "fujikawa" ? u8"富士川飛行場(蒲原)の天気: " : u8"琵琶湖会場の天気: ";
        drawPanelRect(rt, {x, innerTop, w, 50}, theme::panelSoft(), theme::line(), 1);
        drawText(rt, font, wtitle + wx.name, x + 8, innerTop + 6, 12, INK(), 0, true);
        drawText(rt, font, wx.desc, x + 8, innerTop + 26, 10, INKSOFT());
        bGacha_.draw(rt, font);
        bSummer_.draw(rt, font);
        bTerrainWind_.draw(rt, font);
        bWindVis_.draw(rt, font);
        bRealThermal_.draw(rt, font);
        for (auto& s : windSliders_) s.draw(rt, font);
        // 静解析リードアウト (JS wind.refresh)
        if (getAn_ && getAn_()) {
            GustResult g = gustCalc(*st_, *getAn_(), *prm_);
            float ry = windSliders_.back().rect.top + 44;
            char b[128];
            auto row = [&](const std::string& l, const std::string& v, int ok) {
                drawText(rt, font, l, x, ry, 11, INK());
                drawText(rt, font, v, x + w, ry, 11, ok == 1 ? OKC() : ok == 2 ? BADC() : INK(), 2, true);
                ry += 17;
            };
            std::snprintf(b, sizeof(b), "%.2f m/s", g.Vs);
            row(u8"失速速度 Vs", b, 0);
            std::snprintf(b, sizeof(b), "%.1f m/s / +%.0f%%", g.V, g.margin);
            row(u8"巡航速度 / 失速余裕", b, g.margin > 15 ? 1 : 2);
            if (prm_->summer) {
                std::snprintf(b, sizeof(b), u8"%s / %.0f℃", todLabel(*prm_).c_str(), prm_->temp);
                row(u8"時刻 / 気温", b, 0);
            }
            row(u8"風", windLabel(*prm_), 0);
            std::snprintf(b, sizeof(b), "%.1f m/s", g.launchVa);
            row(u8"発進時の対気速度", b, g.launchOK ? 1 : 2);
            std::snprintf(b, sizeof(b), u8"%.2f", g.n);
            row(u8"突風荷重倍数 n", b, g.n < 2.0 ? 1 : 2);
            std::snprintf(b, sizeof(b), "%.2f", g.sfGust);
            row(u8"突風時の桁安全率", b, g.sfGust >= 1.0 ? 1 : 2);
        }
    }
}

} // namespace bm
