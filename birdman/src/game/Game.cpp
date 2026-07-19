#include "game/Game.hpp"
#include "core/Aircraft.hpp"
#include "core/Physics.hpp"
#include "core/Weather.hpp"
#include "core/SiteConst.hpp"
#include "ui/Theme.hpp"
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

namespace bm {

static const double PI = 3.14159265358979323846;

static const char* editablePartId(BodyPart part) {
    switch (part) {
        case BodyPart::Wing: return "wing.main";
        case BodyPart::Prop: return "prop.main";
        case BodyPart::Cockpit: return "cockpit";
        case BodyPart::HTail: return "tail.h";
        case BodyPart::VTail: return "tail.v";
        default: return nullptr;
    }
}

static bool sameMount(const Mount& a, const Mount& b) {
    return a.parentId == b.parentId && a.hardpointId == b.hardpointId && a.mirror == b.mirror
        && a.offset.pos == b.offset.pos && a.offset.rotDeg == b.offset.rotDeg;
}

static double tailSupportMass(const TailSupportDesign& value) {
    if (value.mounting == "cantilever" || value.supportCount <= 0) return 0.0;
    const double reference = value.mounting == "wire" ? 5.0 : 20.0;
    const double perSupport = value.mounting == "wire" ? 0.025 : 0.12;
    return value.supportCount * perSupport * std::pow(value.supportDiaMm / reference, 2.0);
}

static PilotStationDesign defaultPilotStation(const AircraftParams& st) {
    if (st.posture == "upright") return {0.68, -0.30, 0.80, -0.28, 0.42};
    if (st.posture == "semi") return {0.62, -0.50, 0.85, -0.80, 0.58};
    return {0.60, -0.55, 0.92, -0.95, 0.68};
}

static std::string physicsTag(bool sixdof, bool assist) {
    if (!sixdof) return u8"[標準物理/常時補助相当]";
    return std::string(u8"[拡張物理/補助") + (assist ? "ON]" : "OFF]");
}

int Game::run() {
    sf::ContextSettings ctx;
    ctx.depthBits = 24;
    ctx.antialiasingLevel = 4;
    window_.create(sf::VideoMode(1440, 860), utf8(u8"鳥人間 機体設計シミュレーター 3D (C++)"),
                   sf::Style::Default, ctx);
    window_.setPosition({30, 10});   // 起動毎に位置が変わらないよう固定
    window_.setFramerateLimit(60);
    window_.setActive(true);
    if (!hud_.init())
        std::fprintf(stderr, "warning: no Japanese font found; text may be missing\n");
    r3d_.init();
    r3d_.resize((int)window_.getSize().x, (int)window_.getSize().y);
    r3d_.setSite(prm_.site);
    save_.load();
    career_.load();
    audio_.init();

    // ---- ボタン配線 ----
    bModeDesign_.label = u8"設計";
    bModeDesign_.onClick = [this] { setMode("design"); };
    bModeFlight_.label = u8"テストフライト";
    bModeFlight_.onClick = [this] { setMode("flight"); };
    const char* vlabels[4] = {u8"等角", u8"前", u8"横", u8"上"};
    for (int i = 0; i < 4; i++) {
        bViews_[i].label = vlabels[i];
        bViews_[i].charSize = 11;
        bViews_[i].onClick = [this, i] { if (cam_.fpv) setFPV(false); setView(i); };
    }
    bFpv_.label = u8"パイロット視点"; bFpv_.charSize = 11;
    bFpv_.onClick = [this] { setFPV(!cam_.fpv); };
    bHud_.label = u8"HUD計器"; bHud_.charSize = 11;
    bHud_.onClick = [this] { hudEnabled_ = !hudEnabled_; };
    bSettingsIcon_.label = u8"設定"; bSettingsIcon_.style = 3;
    bSettingsIcon_.onClick = [this] { settings_.open = !settings_.open; };
    bFlightExit_.label = u8"設計に戻る"; bFlightExit_.style = 3;
    bFlightExit_.onClick = [this] { setMode("design"); };
    bStart_.label = u8"▶ 発進"; bStart_.style = 1;
    bStart_.onClick = [this] { startSim(); };
    bReset_.label = u8"リセット"; bReset_.style = 3;
    bReset_.onClick = [this] { stopSim(); };
    bStop_.label = u8"■ 中止"; bStop_.style = 3;
    bStop_.onClick = [this] { stopSim(); };
    bReplay_.label = u8"リプレイ"; bReplay_.style = 3;
    bReplay_.onClick = [this] { startReplay(); };
    bReplayExit_.label = u8"リプレイ終了"; bReplayExit_.style = 3;
    bReplayExit_.onClick = [this] { stopReplay(); };
    bReplaySlow_.label = u8"◀ 遅く"; bReplaySlow_.style = 3; bReplaySlow_.charSize = 11;
    bReplaySlow_.onClick = [this] { adjustReplaySpeed(-1); };
    bReplayFast_.label = u8"速く ▶"; bReplayFast_.style = 3; bReplayFast_.charSize = 11;
    bReplayFast_.onClick = [this] { adjustReplaySpeed(+1); };
    // 再生速度: ×0.25〜×8(◀遅 / 速▶)。負で逆再生
    bReplaySlow_.label = u8"◀ 遅"; bReplaySlow_.charSize = 12;
    bReplaySlow_.onClick = [this] {
        replaySpeedMult_ = clamp(replaySpeedMult_ - 0.5, -4.0, 8.0);
        if (std::abs(replaySpeedMult_) < 0.24) replaySpeedMult_ = -0.25;
    };
    bReplayFast_.label = u8"速 ▶"; bReplayFast_.charSize = 12;
    bReplayFast_.onClick = [this] {
        replaySpeedMult_ = clamp(replaySpeedMult_ + 0.5, -4.0, 8.0);
        if (std::abs(replaySpeedMult_) < 0.24) replaySpeedMult_ = 0.25;
    };
    bDebrief_.label = u8"デブリーフ"; bDebrief_.style = 3;
    bDebrief_.onClick = [this] { debriefOpen_ = !debriefOpen_; };
    bDebriefClose_.label = u8"閉じる";
    bDebriefClose_.onClick = [this] { debriefOpen_ = false; };

    DesignPanel::MountEditorCallbacks mountCallbacks;
    mountCallbacks.get = [this](BodyPart part, Mount& out) { return getPartMount(part, out); };
    mountCallbacks.apply = [this](BodyPart part, const Mount& mount, std::string& error) {
        return applyPartMount(part, mount, error);
    };
    mountCallbacks.reset = [this](BodyPart part, std::string& error) { return resetPartMount(part, error); };
    mountCallbacks.undo = [this] { return undoLayout(); };
    mountCallbacks.redo = [this] { return redoLayout(); };
    mountCallbacks.canUndo = [this] { return !layoutUndo_.empty(); };
    mountCallbacks.canRedo = [this] { return !layoutRedo_.empty(); };
    mountCallbacks.targets = [this](BodyPart part) { return mountTargets(part); };
    mountCallbacks.applyVTailPreset = [this](int preset, std::string& error) {
        return applyVTailPreset(preset, error);
    };
    mountCallbacks.hasComponent = [this](const std::string& key) { return hasComponent(key); };
    mountCallbacks.toggleComponent = [this](const std::string& key, std::string& error) {
        return toggleComponent(key, error);
    };
    mountCallbacks.getDesign = [this](BodyPart part, PartDesign& out) { return getPartDesign(part, out); };
    mountCallbacks.applyDesign = [this](BodyPart part, const PartDesign& value, std::string& error) {
        return applyPartDesign(part, value, error);
    };
    mountCallbacks.estimateDesignMass = [this](BodyPart part, const PartDesign& value) {
        return estimatePartDesignMass(part, value);
    };
    design_.build(&st_, [this] { rebuildAircraft(); },
                  [this]() -> const Analysis* { return &an_; }, std::move(mountCallbacks));
    {
        DesignTools::Callbacks tcb;
        tcb.onLoadDesign = [this](const DesignEntry& entry) {
            st_ = entry.st;
            layoutUndo_.clear(); layoutRedo_.clear();
            customLayout_ = entry.layout.has_value();
            if (entry.layout) graph_ = *entry.layout;
            rebuildAircraft();
        };
        tcb.currentLayout = [this]() -> const AirframeGraph* { return customLayout_ ? &graph_ : nullptr; };
        tcb.onFlexPreview = [this](double n, bool apply) { flexPrevN_ = n; flexPrevApply_ = apply; };
        tcb.lastDist = [this] { return simRes_.dist; };
        tcb.career = &career_;
        tcb.onStartMission = [this](int i) { startMission(i); };
        tcb.onStartContest = [this](double cost) { startContest(cost); };
        tcb.missionCleared = [this](const std::string& id) {
            for (const auto& bg : save_.badges())
                if (bg.first == "msn_" + id) return true;
            return false;
        };
        tools_.build(&st_, &prm_, [this]() -> const Analysis* { return &an_; }, tcb);
    }
    SettingsCallbacks cb;
    cb.onStart = [this] { startSim(); };
    cb.onReset = [this] { stopSim(); };
    cb.onMode = [this](const std::string&) { placeForMode(); };
    cb.onSite = [this](const std::string& s) { r3d_.setSite(s); placeForMode(); };
    cb.soundOn = [this] { return audio_.enabled(); };
    cb.onToggleSound = [this] { audio_.setEnabled(!audio_.enabled()); };
    // 空力定数・気象乱数列を飛行途中で変えない。出力/再生速度など安全な項目は
    // SettingsPanel側で個別に許可する。
    cb.locked = [this] { return simActive_; };
    settings_.build(&prm_, &st_, [this]() -> const Analysis* { return &an_; }, cb);

    // デバッグ/検証用: 環境変数で起動時のサイト・モードを指定できる
    // (例: BIRDMAN_SITE=fujikawa BIRDMAN_MODE=flight で富士川フライト画面を直接起動。
    //  スクリーンショットによる見た目検証の自動化に使う。未設定なら従来どおり)
    if (const char* s = std::getenv("BIRDMAN_SITE")) {
        prm_.site = s;
        if (prm_.site == "fujikawa") prm_.mode = "runway";
        r3d_.setSite(prm_.site);
    }
    applyWindVec(prm_);
    if (prm_.summer) { prm_.weather = rollWeather(prm_.site); applySummer(prm_); }
    rebuildAircraft();
    const char* dbgMode = std::getenv("BIRDMAN_MODE");
    setMode(dbgMode && std::string(dbgMode) == "flight" ? "flight" : "design");

    while (window_.isOpen()) {
        sf::Event ev;
        while (window_.pollEvent(ev)) handleEvent(ev);

        // 設計パネルが今フレームで閉じられた(×/Esc)瞬間を検出→標準ビューへアニメ復帰
        {
            const bool openNow = design_.isOpen();
            if (designPanelWasOpen_ && !openNow && appMode_ == "design") {
                const CamGoal g = designViewGoal();
                animCamTo(g.target, g.theta, g.phi, g.dist);
            }
            designPanelWasOpen_ = openNow;
        }

        // ジョイスティック/パッド対応: X=エルロン Y=エレベーター(反転) U/Z=ラダー
        // スティックが中立のときはキーボード操作を邪魔しない
        if (sf::Joystick::isConnected(0) && simActive_ && !live_.done && !live_.auto_) {
            auto dz = [](float v) { return std::abs(v) > 15 ? clamp(v / 100.0, -1.0, 1.0) : 0.0; };
            const double jx = dz(sf::Joystick::getAxisPosition(0, sf::Joystick::X));
            const double jy = dz(sf::Joystick::getAxisPosition(0, sf::Joystick::Y));
            double jr = 0;
            if (sf::Joystick::hasAxis(0, sf::Joystick::U)) jr = dz(sf::Joystick::getAxisPosition(0, sf::Joystick::U));
            else if (sf::Joystick::hasAxis(0, sf::Joystick::Z)) jr = dz(sf::Joystick::getAxisPosition(0, sf::Joystick::Z));
            static bool stickActive = false;
            if (jx != 0 || jy != 0 || jr != 0) {
                live_.ailTgt = jx;
                live_.eTgt = -jy;
                live_.rudTgt = jr;
                stickActive = true;
            } else if (stickActive) {
                live_.ailTgt = live_.eTgt = live_.rudTgt = 0;
                stickActive = false;
            }
        }

        // 実測フレーム時間(現実1秒=ゲーム内1秒)。ヒッチ時の暴走を防ぐため上限0.1s
        frameDt_ = clamp(frameClock_.restart().asSeconds(), 0.0, 0.1);
        appT_ += frameDt_;
        tickCamAnim();
        simTick();
        stepRivals();
        replayTick();
        r3d_.tickFx(frameDt_);

        // サウンド: ループ音更新+イベント検出
        {
            const bool flying = simActive_ && simWait_ <= 0 && !live_.done;
            audio_.update(flying, live_.V, live_.rpm, st_.propBlades, live_.ground);
            if (simActive_ || wasSparBroken_ || prevTouchdowns_) {
                if (live_.sparBroken && !wasSparBroken_) audio_.onCrack();
                if (live_.touchdowns > prevTouchdowns_) audio_.onTouchdown();
                wasSparBroken_ = live_.sparBroken;
                prevTouchdowns_ = live_.touchdowns;
            }
        }

        // プロペラ回転 (JS: 飛行中はrpm連動、非飛行時はアイドル)
        {
            const bool fly = simActive_ && !live_.done && simWait_ <= 0;
            const double spin = fly ? live_.nps * 2 * PI : 3.6;   // rad/s
            propAngle_ += std::min(spin * prm_.speed, 90.0) * frameDt_;
        }

        // 翼たわみ: 飛行中=荷重n連動 / 発進待機・駐機=自重たれ / 設計=荷重ツールのプレビュー
        {
            double fl = 0, fi = 1;
            if (simActive_) {
                if (simWait_ > 0) { fl = 0; fi = 1; }
                else {
                    fl = live_.sparBroken ? 3.0 : live_.n;
                    fi = live_.ground ? 1.0 : live_.n;
                }
            } else if (replayActive_) {
                fl = fi = replayS_.n;
            } else if (appMode_ == "design" && flexPrevApply_) {
                fl = fi = flexPrevN_;
            }
            // 構造の応答遅れ(一次遅れ~0.2s): 突風荷重でnが毎フレーム震えても
            // 翼は慣性で滑らかに追従する(特に6DOFのぐわんぐわん揺れ対策)
            flexFl_ += (fl - flexFl_) * 0.08;
            flexFi_ += (fi - flexFi_) * 0.08;
            double flDraw = flexFl_;
            // フラッター視覚化: VNE手前で翼が震え始める(疲労破断の前兆警告)。
            // 警告なので平滑化せず直接加算する
            if (simActive_ && simWait_ <= 0 && !live_.ground && !live_.sparBroken) {
                const double vr = live_.V / liveC_.VNE;
                if (vr > 0.85)
                    flDraw += (vr - 0.85) / 0.15 * 0.7 * std::sin(appT_ * 30);
            }
            r3d_.applyFlex(flDraw, flexFi_);
        }

        // ---- 3D描画 ----
        window_.setActive(true);
        glm::dvec3 gpos(0, 0, 0);
        bool gvis = false;
        if (ghostShow_ && simActive_ && ghostIdx_ < save_.best().path.size()) {
            const auto& gp = save_.best().path[ghostIdx_];
            gpos = {gp.yl, gp.h, z0_ - gp.x};
            gvis = ghostIdx_ + 1 < save_.best().path.size();
        }
        r3d_.drawFrame(appMode_, prm_, cam_, acPos_, acRot_, acVisible_, propAngle_,
                       trail_, gvis, gpos, st_.span);
        // 設計モード: 主翼パネル(④)表示中はフラップ区間を緑ハイライト
        if (appMode_ == "design" && design_.isOpen() &&
            design_.currentPart() == BodyPart::Wing && st_.flapSpanFrac > 0.005)
            r3d_.drawFlapHighlight(st_, an_);
        // 風の可視化(実風場で移流する粒子)
        if (appMode_ == "flight" && prm_.windVis)
            r3d_.drawWind(prm_, acPos_, z0_,
                          simActive_ ? live_.t : (replayActive_ ? replayT_ : appT_));
        // 地面効果の可視化(機体の影+低空の水面さざ波)
        if (appMode_ == "flight" && (simActive_ || replayActive_) && acPos_.y < 30)
            r3d_.drawGroundFx(acPos_, st_.span, insideWaterSite(prm_, -acPos_.z, acPos_.x));
        // 富士川の自由発進: 発進前は足元に発進方向の矢印を表示(第5弾)
        if (startPlaceActive())
            r3d_.drawStartArrow(acPos_, prm_.startHdg * PI / 180);
        // 大会ライバル機
        if (appMode_ == "flight" && !rivals_.empty())
            for (const auto& rv : rivals_)
                r3d_.drawSimplePlane({rv.L.yl, rv.L.h, z0_ - rv.L.x}, rv.span, rv.color);

        // ---- 2D UI ----
        window_.pushGLStates();
        drawUI();
        window_.popGLStates();
        window_.display();
    }
    return 0;
}

void Game::rebuildAircraft() {
    an_ = analyze(st_, &prm_);
    if (!customLayout_ || graph_.size() == 0) graph_ = buildDefaultLayout(st_, an_);
    if (!graph_.validate().empty()) {
        std::fprintf(stderr, "warning: invalid aircraft parameters; restoring the default aircraft\n");
        customLayout_ = false;
        layoutUndo_.clear(); layoutRedo_.clear();
        graph_ = buildDefaultLayout(st_, an_);
        if (!graph_.validate().empty()) {
            st_ = AircraftParams{};
            an_ = analyze(st_, &prm_);
            graph_ = buildDefaultLayout(st_, an_);
        }
    }
    r3d_.buildAircraft(st_, an_, graph_);
}

bool Game::getPartMount(BodyPart part, Mount& out) const {
    const char* id = editablePartId(part);
    const Part* value = id ? graph_.find(id) : nullptr;
    if (!value) return false;
    out = value->mount;
    return true;
}

void Game::installLayout(AirframeGraph next, bool custom, bool recordHistory) {
    if (recordHistory) {
        layoutUndo_.push_back({graph_, customLayout_});
        if (layoutUndo_.size() > 64) layoutUndo_.erase(layoutUndo_.begin());
        layoutRedo_.clear();
    }
    graph_ = std::move(next);
    customLayout_ = custom;
    r3d_.buildAircraft(st_, an_, graph_);
}

bool Game::applyPartMount(BodyPart part, const Mount& mount, std::string& error) {
    const char* id = editablePartId(part);
    const Part* current = id ? graph_.find(id) : nullptr;
    if (!current) { error = u8"この部品は取付編集の対象外です"; return false; }
    if (sameMount(current->mount, mount)) return true;
    AirframeGraph candidate = graph_;
    if (!candidate.setMount(id, mount, &error)) return false;
    installLayout(std::move(candidate), true, true);
    return true;
}

bool Game::resetPartMount(BodyPart part, std::string& error) {
    const char* id = editablePartId(part);
    if (!id) { error = u8"この部品は標準位置へ戻せません"; return false; }
    const AirframeGraph defaults = buildDefaultLayout(st_, an_);
    const Part* standard = defaults.find(id);
    if (!standard) { error = u8"現在の構成に標準部品がありません"; return false; }
    return applyPartMount(part, standard->mount, error);
}

bool Game::undoLayout() {
    if (layoutUndo_.empty()) return false;
    LayoutSnapshot previous = std::move(layoutUndo_.back());
    layoutUndo_.pop_back();
    layoutRedo_.push_back({graph_, customLayout_});
    installLayout(std::move(previous.graph), previous.custom, false);
    return true;
}

bool Game::redoLayout() {
    if (layoutRedo_.empty()) return false;
    LayoutSnapshot next = std::move(layoutRedo_.back());
    layoutRedo_.pop_back();
    layoutUndo_.push_back({graph_, customLayout_});
    installLayout(std::move(next.graph), next.custom, false);
    return true;
}

std::vector<DesignPanel::MountTarget> Game::mountTargets(BodyPart part) const {
    std::vector<DesignPanel::MountTarget> result;
    const char* id = editablePartId(part);
    const Part* current = id ? graph_.find(id) : nullptr;
    if (!current) return result;
    auto partName = [](const Part& value) {
        if (value.id == "fuselage") return std::string(u8"胴体");
        if (value.id == "wing.main") return std::string(u8"主翼");
        if (value.id == "tail.h") return std::string(u8"水平尾翼");
        return value.id;
    };
    auto hardpointName = [](const std::string& value) {
        if (value == "hp.wing") return std::string(u8"主翼取付");
        if (value == "hp.tail.h") return std::string(u8"水平尾翼取付");
        if (value == "hp.tail.v") return std::string(u8"垂直尾翼取付");
        if (value == "hp.prop") return std::string(u8"プロペラ取付");
        if (value == "hp.cockpit") return std::string(u8"コックピット取付");
        if (value == "hp.tip") return std::string(u8"翼端");
        return value;
    };
    for (const auto& entry : graph_.parts()) {
        const Part& parent = entry.second;
        for (const auto& hardpoint : parent.hardpoints) {
            Mount trial = current->mount;
            trial.parentId = parent.id;
            trial.hardpointId = hardpoint.id;
            AirframeGraph candidate = graph_;
            if (!candidate.setMount(id, trial)) continue;
            result.push_back({parent.id, hardpoint.id,
                              partName(parent) + " / " + hardpointName(hardpoint.id)});
        }
    }
    return result;
}

bool Game::applyVTailPreset(int preset, std::string& error) {
    const Part* current = graph_.find("tail.v");
    if (!current) { error = u8"垂直尾翼がありません"; return false; }
    Mount mount = current->mount;
    mount.offset = {};
    if (preset == 0) {
        mount.parentId = "fuselage"; mount.hardpointId = "hp.tail.v"; mount.mirror = MirrorMode::None;
    } else if (preset == 1) {
        mount.parentId = "tail.h"; mount.hardpointId = "hp.tip"; mount.mirror = MirrorMode::Pair;
    } else if (preset == 2) {
        mount.parentId = "wing.main"; mount.hardpointId = "hp.tip"; mount.mirror = MirrorMode::Pair;
    } else { error = u8"不明な尾翼配置です"; return false; }
    AirframeGraph candidate = graph_;
    if (preset != 0) {
        const Part* parent = candidate.find(mount.parentId);
        if (!parent) { error = u8"取付先の部品がありません"; return false; }
        bool hasTip = false;
        for (const auto& hardpoint : parent->hardpoints) hasTip = hasTip || hardpoint.id == "hp.tip";
        if (!hasTip) {
            // schema v3 layouts saved before Phase 0B-4 do not yet contain the
            // stable tip hardpoints. Add the current-parameter default point
            // atomically so old custom designs can use the new presets.
            const AirframeGraph defaults = buildDefaultLayout(st_, an_);
            const Part* standardParent = defaults.find(mount.parentId);
            if (!standardParent) { error = u8"標準取付点を生成できません"; return false; }
            Part replacement = *parent;
            for (const auto& hardpoint : standardParent->hardpoints)
                if (hardpoint.id == "hp.tip") replacement.hardpoints.push_back(hardpoint);
            if (!candidate.replacePart(parent->id, std::move(replacement), &error)) return false;
        }
    }
    if (!candidate.setMount("tail.v", mount, &error)) return false;
    installLayout(std::move(candidate), true, true);
    return true;
}

bool Game::hasComponent(const std::string& key) const {
    if (key == "gear") {
        for (const auto& entry : graph_.parts()) if (entry.second.kind == PartKind::Gear) return true;
        return false;
    }
    return graph_.find(key) != nullptr;
}

bool Game::toggleComponent(const std::string& key, std::string& error) {
    if (key != "pilot" && key != "fairing" && key != "gear" && key != "boomwing") {
        error = u8"不明な部品です"; return false;
    }
    AirframeGraph candidate = graph_;
    if (hasComponent(key)) {
        if (key == "gear") {
            std::vector<std::string> ids;
            for (const auto& entry : candidate.parts())
                if (entry.second.kind == PartKind::Gear) ids.push_back(entry.first);
            for (const auto& id : ids)
                if (!candidate.removeSubtree(id, &error)) return false;
        } else if (!candidate.removeSubtree(key, &error)) return false;
    } else {
        const AirframeGraph defaults = buildDefaultLayout(st_, an_);
        auto add = [&](Part part) {
            if (candidate.addPart(part, &error)) return true;
            // A custom graph may already carry the same analysis mass item on
            // another part. Component composition is visual in Phase 0B, so a
            // massless copy remains valid until the dedicated physics phase.
            part.massNodes.clear();
            return candidate.addPart(std::move(part), &error);
        };
        if (key == "pilot" || key == "fairing" || key == "boomwing") {
            if (const Part* standard = defaults.find(key)) {
                if (!add(*standard)) return false;
            } else {
                Part part;
                part.id = key;
                part.kind = key == "pilot" ? PartKind::Pilot
                          : key == "fairing" ? PartKind::Fairing : PartKind::BoomWing;
                part.mount.parentId = "fuselage";
                part.mount.hardpointId = key == "boomwing" ? "hp.boomwing" : "hp.cockpit";
                if (key == "boomwing") part.mount.mirror = MirrorMode::Pair;
                if (key == "pilot") part.design.pilot = PilotStationDesign{};
                if (key == "fairing") part.design.fairing = FairingDesign{};
                if (!add(std::move(part))) return false;
            }
        } else {
            bool added = false;
            for (const auto& entry : defaults.parts()) {
                if (entry.second.kind != PartKind::Gear) continue;
                if (!add(entry.second)) return false;
                added = true;
            }
            if (!added) {
                for (const auto& spec : {std::pair<const char*, const char*>{"gear.front", "hp.gear.front"},
                                         {"gear.rear", "hp.gear.rear"}}) {
                    Part gear;
                    gear.id = spec.first; gear.kind = PartKind::Gear;
                    gear.mount.parentId = "fuselage"; gear.mount.hardpointId = spec.second;
                    if (!add(std::move(gear))) return false;
                }
            }
        }
    }
    installLayout(std::move(candidate), true, true);
    return true;
}

bool Game::getPartDesign(BodyPart part, PartDesign& out) const {
    out = {};
    if (part == BodyPart::Wing) {
        const Part* value = graph_.find("wing.main");
        if (!value) return false;
        out.spar = value->design.spar.value_or(SparDesign{1, 0.30, st_.rootDia, st_.tipDia,
                                                          std::max(0, st_.segments - 1), "tube"});
        return true;
    }
    if (part == BodyPart::Cockpit) {
        if (const Part* fairing = graph_.find("fairing"))
            out.fairing = fairing->design.fairing.value_or(FairingDesign{});
        if (const Part* pilot = graph_.find("pilot"))
            out.pilot = pilot->design.pilot.value_or(defaultPilotStation(st_));
        return out.fairing.has_value() || out.pilot.has_value();
    }
    const char* id = part == BodyPart::HTail ? "tail.h" : part == BodyPart::VTail ? "tail.v" : nullptr;
    const Part* value = id ? graph_.find(id) : nullptr;
    if (!value) return false;
    out.tailSupport = value->design.tailSupport.value_or(TailSupportDesign{});
    return true;
}

double Game::estimatePartDesignMass(BodyPart part, const PartDesign& design) const {
    double total = 0.0;
    if (part == BodyPart::Wing && design.spar) {
        const SparDesign& value = *design.spar;
        const double baseArea = std::max(1.0, 0.5 * (st_.rootDia * st_.rootDia + st_.tipDia * st_.tipDia));
        const double area = 0.5 * (value.rootDiaMm * value.rootDiaMm + value.tipDiaMm * value.tipDiaMm);
        const double sectionFactor = value.section == "box" ? 0.82 : value.section == "i-beam" ? 0.66 : 1.0;
        total += an_.wSpar * value.count * area / baseArea * sectionFactor;
        total += an_.wJoints * (value.jointCount + 1.0) / std::max(1, st_.segments);
    }
    if (design.fairing) {
        const FairingDesign& value = *design.fairing;
        const FairingDesign base;
        const double scale = value.lengthM * (value.widthM + value.heightM)
            / (base.lengthM * (base.widthM + base.heightM));
        total += 1.1 * scale * (0.8 + 0.4 * value.tailRatio);
    }
    if (design.pilot) {
        const PilotStationDesign& value = *design.pilot;
        const double linkage = std::hypot(value.pedalZM - value.crankZM,
                                          value.pedalHeightM - value.crankHeightM);
        total += 0.22 + linkage * 0.12;
    }
    if (design.tailSupport) total += tailSupportMass(*design.tailSupport);
    return total;
}

bool Game::applyPartDesign(BodyPart part, const PartDesign& design, std::string& error) {
    AirframeGraph candidate = graph_;
    auto setMass = [](Part& value, int analysisItem, double kg) {
        for (auto& mass : value.massNodes) if (mass.analysisItem == analysisItem) {
            mass.kg = std::max(0.0, kg); return;
        }
        MassNode node;
        node.kg = std::max(0.0, kg); node.analysisItem = -1;
        value.massNodes.push_back(node);
    };
    auto replace = [&](const char* id, const PartDesign& value, int analysisItem,
                       double massKg, bool absoluteMass) {
        const Part* current = candidate.find(id);
        if (!current) { error = std::string(u8"部品がありません: ") + id; return false; }
        Part replacement = *current;
        replacement.design = value;
        double target = massKg;
        if (!absoluteMass) target += an_.items[(std::size_t)analysisItem].w;
        setMass(replacement, analysisItem, target);
        return candidate.replacePart(id, std::move(replacement), &error);
    };

    if (part == BodyPart::Wing && design.spar) {
        PartDesign baseDesign;
        baseDesign.spar = SparDesign{1, 0.30, st_.rootDia, st_.tipDia,
                                     std::max(0, st_.segments - 1), "tube"};
        const double delta = estimatePartDesignMass(part, design) - estimatePartDesignMass(part, baseDesign);
        if (!replace("wing.main", design, 0, delta, false)) return false;
    } else if (part == BodyPart::Cockpit) {
        if (design.fairing) {
            PartDesign value; value.fairing = design.fairing;
            if (!replace("fairing", value, 8, estimatePartDesignMass(part, value), true)) return false;
        }
        if (design.pilot) {
            PartDesign value; value.pilot = design.pilot;
            PartDesign base; base.pilot = defaultPilotStation(st_);
            const double delta = estimatePartDesignMass(part, value) - estimatePartDesignMass(part, base);
            if (!replace("pilot", value, 10, delta, false)) return false;
        }
    } else if ((part == BodyPart::HTail || part == BodyPart::VTail) && design.tailSupport) {
        const char* id = part == BodyPart::HTail ? "tail.h" : "tail.v";
        const int item = part == BodyPart::HTail ? 1 : 2;
        if (!replace(id, design, item, tailSupportMass(*design.tailSupport), false)) return false;
    } else {
        error = u8"この部位には固有設計がありません"; return false;
    }
    installLayout(std::move(candidate), true, true);
    return true;
}

void Game::setMode(const std::string& mode) {
    appMode_ = mode;
    if (mode == "design") {
        stopReplay();
        if (simActive_) stopSim();
        if (cam_.fpv) setFPV(false);
        settings_.open = false;
        acPos_ = {0, 0, 0}; acRot_ = {0, 0, 0}; acVisible_ = true;
        cam_.target = {0, 1.2, an_.wingLE + an_.MAC * 0.4};
        cam_.theta = -0.7; cam_.phi = 1.05;
        cam_.dist = std::max(12.0, st_.span * 0.85);
        camAnimActive_ = false;
    } else {
        acVisible_ = true;
        placeForMode();
        settings_.open = false;
        design_.close();
        camAnimActive_ = false;
        designPanelWasOpen_ = false;
    }
    bModeDesign_.style = mode == "design" ? 2 : 0;
    bModeFlight_.style = mode == "flight" ? 2 : 0;
}

void Game::placeForMode() {
    if (simActive_ || appMode_ != "flight") return;
    if (prm_.mode == "runway") {
        // 富士川の自由発進(第5弾): startPos(滑走路沿い0〜840m, 0=北端)と
        // startHdg(機首方位°)で待機位置・向きが変わる。ワールドz=850-startPos
        // (滑走路再設計: 850m, 南エンドz=10=RWY36, 北エンドz=860=RWY18)
        const bool fuji = prm_.site == "fujikawa";
        const double rwyStart = fuji ? site::fujikawaStartWorldZ(prm_.startPos) : RWY_START;
        const double hdg = fuji ? prm_.startHdg * PI / 180 : 0.0;
        acPos_ = {0, 0, rwyStart};
        cam_.target = {0, 2, rwyStart};
        acRot_ = {0, -hdg, 0};   // 描画ヨーは-psi(simTickと同じ規約)
        cam_.fpvPose = {0, 1.0, rwyStart, 0, hdg, 0};
        // カメラ方位は機首方位に追従(常に機体後方寄りから見る構図。第7弾:
        // デフォルトが北向き180°になり、固定theta=-0.5だと機体正面からの構図になるため)
        cam_.theta = -0.5 + hdg;
    } else {
        // デッキ後端(x=-10 → ワールドz=+10, 上面高10.6m)で待機
        acPos_ = {0, deckHeightAt(-10), 10};
        cam_.target = {0, 11.3, 10};
        acRot_ = {0, 0, 0};
        cam_.fpvPose = {0, deckHeightAt(-10) + 1.0, 10.0, 0, 0, 0};
        cam_.theta = -0.5;
    }
    cam_.phi = 1.15;
    cam_.dist = std::max(16.0, st_.span * 0.9);
}

void Game::startSim() {
    stopReplay();
    inStartSim_ = true;
    stopSim();
    inStartSim_ = false;
    if (!contestActive_) rivals_.clear();   // 通常発進では前大会のライバルを消す
    debriefOpen_ = false;
    if (prm_.summer) {
        // ミッション中は指定天候を固定(再抽選しない)
        if (missionActive_ < 0) prm_.weather = rollWeather(prm_.site);
        prm_.dirJit = prm_.tempJit = prm_.wspdJit = prm_.gustJit = 0;
        applySummer(prm_);
    }
    // 飛行ごとの環境シード。以後は固定物理刻みごとに更新するためfps/再生速度に依存しない。
    // ライバルはbuildRivalsでこの状態をコピーし、同じ気象系列を受ける。
    const unsigned weatherSeed = (unsigned)(frand() * 4294967295.0);
    resetWeatherState(prm_, weatherSeed, true);
    // 発進時の気温・脚・駆動系を反映した解析値と実飛行定数を同じ入力から作る。
    rebuildAircraft();
    // お遊び機は滑走路専用 → 富士川へ強制(琵琶湖に滑走路は無い)
    if (prm_.funPlane && prm_.site != "fujikawa") {
        prm_.site = "fujikawa";
        r3d_.setSite(prm_.site);
    }
    // 発進方式はサイトで一意: 琵琶湖=プラットフォーム / 富士川=滑走路
    prm_.mode = prm_.site == "fujikawa" ? "runway" : "platform";
    if (prm_.funPlane) {
        liveC_ = funPlaneConstants(prm_);
    } else if (customLayout_) {
        const MassBreakdown customMass = aggregateMass(graph_);
        const MassBreakdown referenceMass = aggregateMass(buildDefaultLayout(st_, an_));
        const AeroLayoutProperties aeroLayout = aggregateAeroLayout(graph_);
        const DesignPhysicsProperties designPhysics = aggregateDesignPhysics(graph_);
        liveC_ = aeroPackCustomDesign(st_, an_, prm_, customMass, referenceMass,
                                      aeroLayout, designPhysics);
    } else {
        liveC_ = aeroPack(st_, an_, prm_);
    }
    const bool runway = prm_.mode == "runway";
    // 富士川: 自由発進位置(startPos)ぶん発進原点をずらす。startHdgはmakeInitialStateがpsiへ反映
    prm_.startPos = clamp(prm_.startPos, site::FUJI_START_POS_MIN, site::FUJI_START_POS_MAX);
    z0_ = runway ? (prm_.site == "fujikawa" ? site::fujikawaStartWorldZ(prm_.startPos) : RWY_START) : 0;
    live_ = makeInitialState(liveC_, prm_, z0_);
    simRes_ = SimResult{};
    simRes_.out.push_back(simSample(live_));
    simRes_.Vs = liveC_.Vs;
    simRes_.runway = runway;
    simRes_.sixdof = prm_.sixdof;
    simRes_.assist = prm_.assist;
    // ゴースト(自己ベスト再生)
    ghostShow_ = prm_.ghost && !runway && save_.best().valid && save_.best().path.size() > 5;
    ghostIdx_ = 0;
    trail_.clear();
    lastX_ = -99;
    simWait_ = 2;
    simActive_ = true;
    wasSparBroken_ = false;
    prevTouchdowns_ = 0;
    // パイロン周回コース(プラットフォーム発進のみ)
    courseLeg_ = runway ? -1 : 0;
    lapTime_ = -1;
    courseMsg_.clear();
    courseMsgTimer_ = 0;
    if (liveC_.a.SM < 0) {
        courseMsg_ = u8"静的不安定 — 操縦補助なしでは発散します";
        courseMsgTimer_ = 6;
    }
    acPos_ = {0, runway ? 0.0 : deckHeightAt(live_.x), z0_ - live_.x};
    acRot_ = {0, -live_.psi, 0};   // 自由発進の機首方位を初期表示から反映(psi=0なら従来通り)
    if (live_.done) {   // 滑走路×脚なし → 即終了 (JS nogearと同じ)
        finishSim();
    }
}

void Game::stopSim() {
    // 飛行中断時のミッション/大会の後始末(startSim内の再スタート時は除く)
    if (!inStartSim_) {
        if (missionActive_ >= 0) {
            st_ = stBackup_;
            prm_ = prmBackup_;
            rebuildAircraft();
            missionActive_ = -1;
            courseMsg_ = u8"ミッション中断";
            courseMsgTimer_ = 4;
        }
        if (contestActive_) {
            // 中止=そこまでの距離が公式記録(実際の大会も着水/中断地点まで)
            const int wi = prm_.weather;
            courseMsg_ = career_.recordContest(wi >= 0 ? siteWeather(prm_.site)[wi].name : "-",
                                               simActive_ ? live_.officialDist : 0.0, contestCost_,
                                               0, 0, prm_.sixdof, prm_.assist);
            courseMsgTimer_ = 8;
            prm_ = prmBackup_;
            contestActive_ = false;
        }
        rivals_.clear();
    }
    r3d_.clearFx();
    simActive_ = false;
    if (!inStartSim_) {
        prm_.atmosphereLocked = false;
        prm_.launchTemp = prm_.temp;
        rebuildAircraft();
    }
    trail_.clear();
    ghostShow_ = false;
    if (appMode_ == "flight") placeForMode();
    acVisible_ = !cam_.fpv;
}

void Game::simTick() {
    if (!simActive_) return;
    if (simWait_ > 0) {
        simWait_ -= frameDt_;
        cam_.target = {0, live_.h + 1.5, z0_ - live_.x};
        return;
    }
    if (!live_.done) {
        live_.acc += prm_.speed * frameDt_;   // 実時間×再生速度ぶんだけsim時間を進める
        int guard = 0;
        while (live_.acc >= 0.02 && guard++ < 480 && !live_.done) {
            live_.acc -= 0.02;
            updateWeatherJitter(prm_, true, 0.02);
            if (prm_.sixdof) stepSim6(live_, liveC_, prm_, 0.02);
            else stepSim(live_, liveC_, prm_, 0.02);
            if (live_.t - live_.lastSample >= 0.2) {
                simRes_.out.push_back(simSample(live_));
                live_.lastSample = live_.t;
            }
        }
        if (live_.done) {
            simRes_.out.push_back(simSample(live_));
            live_.lastSample = live_.t;
        }
        // ハードランディング警告トースト(第6弾: 沈下率2.0〜3.5m/sのバウンド接地)
        if (live_.hardLanding) {
            live_.hardLanding = false;
            courseMsg_ = u8"ハードランディング! 沈下率過大 — バウンドに注意";
            courseMsgTimer_ = 3;
        }
    }
    live_.frame++;
    const double zw = z0_ - live_.x;
    // 6DOFではピッチ姿勢theta・機首方位psi+betaで描画(標準は経路角gam・経路方位psi)
    double pitchAtt = prm_.sixdof && !live_.ground ? live_.theta : live_.gam;
    if (live_.ground && prm_.mode != "runway") pitchAtt = -3.5 * PI / 180;  // デッキ滑走中は斜面に沿う
    const double headAtt = live_.psi + (prm_.sixdof && !live_.ground ? live_.beta : 0.0);
    acPos_ = {live_.yl, live_.h, zw};
    // ロールは-φ: モデルは機首-Z・右翼+Xなので、φ>0(右バンク・右旋回)を
    // 見た目の右翼下げにするには描画回転は負にする(JS版からの符号バグを修正)
    acRot_ = {pitchAtt, -headAtt, -live_.phi};
    bool vis = !cam_.fpv;
    if (live_.done && live_.splash && !live_.crashed) vis = vis && (live_.frame % 6 < 4);
    acVisible_ = vis;
    cam_.fpvPose = {live_.yl, live_.h, zw, pitchAtt, headAtt, live_.phi};
    // ゴースト追従(同じ進行距離の地点)
    if (ghostShow_) {
        const auto& path = save_.best().path;
        while (ghostIdx_ + 1 < path.size() && path[ghostIdx_].x < live_.x) ghostIdx_++;
    }
    // 航跡
    if (live_.x - lastX_ > 2) {
        lastX_ = live_.x;
        trail_.push_back({live_.yl, live_.h + 1.6, zw});
    }
    cam_.target = {live_.yl, live_.h + 1.5, zw};
    // ---- パイロン周回コース判定 ----
    if (!live_.done && !live_.ground) {
        char m[160];
        if (courseLeg_ == 0 && std::hypot(live_.x - 11000, live_.yl) < 250) {
            courseLeg_ = 1;
            std::snprintf(m, sizeof(m), u8"北パイロン通過! %d:%04.1f — PFへ折り返し",
                          (int)(live_.t / 60), std::fmod(live_.t, 60));
            courseMsg_ = m;
            const std::string bg = save_.awardBadge("pylonN", u8"★北パイロン到達");
            if (!bg.empty()) courseMsg_ += "  " + bg;
            courseMsgTimer_ = 6;
        } else if (courseLeg_ == 1 && std::hypot(live_.x, live_.yl) < 300) {
            courseLeg_ = 2;
            lapTime_ = live_.t;
            std::snprintf(m, sizeof(m), u8"北パイロン周回達成!! タイム %d分%04.1f秒",
                          (int)(lapTime_ / 60), std::fmod(lapTime_, 60));
            courseMsg_ = m;
            const std::string bg = save_.awardBadge("lapN", u8"★北パイロン周回(22km)");
            if (!bg.empty()) courseMsg_ += "  " + bg;
            courseMsgTimer_ = 8;
        }
    }
    if (live_.done) finishSim();
}

void Game::finishSim() {
    simRes_.dist = live_.officialDist;
    simRes_.pathAir = live_.pathAir;
    simRes_.time = live_.t;
    simRes_.splash = live_.splash;
    simRes_.overrun = live_.overrun;
    simRes_.groundRoll = live_.groundRoll;
    simRes_.rollDist = live_.rollDist;
    simRes_.landed = live_.landed;
    simRes_.touchdowns = live_.touchdowns;
    simRes_.brkMsg = live_.failureMsg;
    simRes_.failStation = live_.failStation;
    simRes_.failMode = live_.failMode;
    simRes_.sparBroken = live_.sparBroken;
    simRes_.gearBroken = live_.gearBroken;
    simRes_.crashed = live_.crashed;
    simRes_.auto_ = live_.auto_;
    simRes_.sixdof = prm_.sixdof;
    simRes_.assist = prm_.assist;
    simRes_.offcourse = live_.offcourse;
    simRes_.nogear = live_.nogear;
    simRes_.summer = prm_.summer;
    simRes_.tod = prm_.tod;
    simRes_.maxBank = live_.maxBank;
    simRes_.inThermalUsed = live_.inThermalUsed;
    // 記録更新&ゴースト保存
    if (!simRes_.runway && simRes_.dist > 50) {
        BestRun rec;
        rec.dist = simRes_.dist;
        rec.sixdof = simRes_.sixdof;
        rec.assist = simRes_.assist;
        char nm[48];
        std::snprintf(nm, sizeof(nm), "%s/%.0fm", st_.planform.c_str(), st_.span);
        rec.name = nm;
        for (size_t i = 0; i < simRes_.out.size(); i += 3)
            rec.path.push_back({simRes_.out[i].x, simRes_.out[i].h, simRes_.out[i].yl});
        if (!save_.best().valid || simRes_.dist > save_.best().dist) {
            save_.saveBest(rec);
            simRes_.newBest = true;
        }
    }
    simRes_.badges = prm_.funPlane ? std::vector<std::string>{}
                                   : save_.checkMissions(simRes_);   // お遊び機はバッジ対象外
    // ---- チャレンジミッション判定 ----
    if (missionActive_ >= 0) {
        const Mission& ms = missions()[missionActive_];
        if (ms.goal(simRes_, lapTime_)) {
            const std::string bg = save_.awardBadge(std::string("msn_") + ms.id,
                                                    std::string(u8"★") + ms.name);
            courseMsg_ = std::string(u8"ミッション達成! ") + ms.name;
            if (!bg.empty()) courseMsg_ += "  " + bg;
        } else {
            courseMsg_ = std::string(u8"ミッション失敗… ") + ms.name +
                         u8" [" + ms.goalText + u8"]";
        }
        courseMsgTimer_ = 8;
        st_ = stBackup_;
        prm_ = prmBackup_;
        rebuildAircraft();
        missionActive_ = -1;
    }
    // ---- キャリア大会の結果記録(ライバルを完走させて順位確定) ----
    if (contestActive_) {
        for (auto& rv : rivals_)
            while (!rv.L.done && rv.L.t < 3650) {
                updateWeatherJitter(rv.prm, true, 0.02);
                stepSim(rv.L, rv.c, rv.prm, 0.02);
            }
        int rank = 1;
        double bestRivalDist = 0;
        std::string bestRival;
        for (const auto& rv : rivals_) {
            if (rv.L.officialDist > simRes_.dist) rank++;
            if (rv.L.officialDist > bestRivalDist) {
                bestRivalDist = rv.L.officialDist; bestRival = rv.name;
            }
        }
        const int wi = prm_.weather;
        const std::string wx = wi >= 0 ? siteWeather(prm_.site)[wi].name : "-";
        courseMsg_ = career_.recordContest(wx, simRes_.dist, contestCost_,
                                           rank, (int)rivals_.size() + 1,
                                           simRes_.sixdof, simRes_.assist);
        if (!bestRival.empty() && rank > 1) {
            char rb[120];
            std::snprintf(rb, sizeof(rb), u8"  (1位 %s %.0fm)", bestRival.c_str(), bestRivalDist);
            courseMsg_ += rb;
        }
        courseMsgTimer_ = 12;
        prm_ = prmBackup_;
        contestActive_ = false;
    }
    if (live_.splash || live_.crashed || (live_.sparBroken && live_.h <= 0.5)) {
        const double cz = z0_ - live_.x;
        // 湖岸ポリゴンで水面/陸面を判定(東岸・対岸に到達した場合も正しく地面クラッシュに)
        const bool onLand = !insideWaterSite(prm_, live_.x - z0_, live_.yl);
        r3d_.spawnCrashFx({live_.yl, 0.1, cz}, !onLand);
        if (!onLand) audio_.onSplash(); else audio_.onTouchdown();
    }
    simActive_ = false;
    acVisible_ = !cam_.fpv;
}

// ---- リプレイカメラ ----
static double lerpAngle(double a, double b, double t) {
    double d = b - a;
    while (d > PI) d -= 2 * PI;
    while (d < -PI) d += 2 * PI;
    return a + d * t;
}
static SimSample interpSample(const std::vector<SimSample>& out, double t) {
    if (out.empty()) return SimSample{};
    if (t <= out.front().t) return out.front();
    if (t >= out.back().t) return out.back();
    size_t lo = 0, hi = out.size() - 1;
    while (lo + 1 < hi) {
        const size_t mid = (lo + hi) / 2;
        (out[mid].t <= t ? lo : hi) = mid;
    }
    const SimSample& a = out[lo];
    const SimSample& b = out[hi];
    const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
    SimSample s;
    s.t = t;
    s.x = lerp(a.x, b.x, u); s.h = lerp(a.h, b.h, u); s.V = lerp(a.V, b.V, u);
    s.n = lerp(a.n, b.n, u); s.yl = lerp(a.yl, b.yl, u);
    s.officialDist = lerp(a.officialDist, b.officialDist, u);
    s.pathAir = lerp(a.pathAir, b.pathAir, u);
    s.gam = lerp(a.gam, b.gam, u); s.phi = lerp(a.phi, b.phi, u);
    s.psi = lerpAngle(a.psi, b.psi, u);
    return s;
}

void Game::startReplay() {
    if (simActive_ || replayActive_ || simRes_.out.size() < 6) return;
    replayDur_ = simRes_.out.back().t;
    if (replayDur_ < 2) return;
    replayScale_ = std::max(1.0, replayDur_ / 60.0);   // 全体を最長60秒に収める
    replayT_ = 0;
    replaySpeedMult_ = 1.0;
    // 航跡を全経路で再構築(リセット後でもsimRes_から復元できる)
    trail_.clear();
    for (size_t i = 0; i < simRes_.out.size(); i += 2)
        trail_.push_back({simRes_.out[i].yl, simRes_.out[i].h + 1.6, z0_ - simRes_.out[i].x});
    settings_.open = false;
    r3d_.clearFx();
    replayActive_ = true;
}

void Game::stopReplay() {
    if (!replayActive_) return;
    replayActive_ = false;
    trail_.clear();
    acVisible_ = true;
    if (appMode_ == "flight") placeForMode();
}

void Game::adjustReplaySpeed(int dir) {
    // 速度段階: 逆再生(-1)〜×8。dirで隣へ移動
    static const double spd[] = {-1.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0};
    const int N = (int)(sizeof(spd) / sizeof(spd[0]));
    int k = 3;   // ×1
    double best = 1e9;
    for (int i = 0; i < N; i++)
        if (std::abs(spd[i] - replaySpeedMult_) < best) { best = std::abs(spd[i] - replaySpeedMult_); k = i; }
    k = (int)clamp((double)(k + dir), 0.0, (double)(N - 1));
    replaySpeedMult_ = spd[k];
}

void Game::replayTick() {
    if (!replayActive_) return;
    replayT_ += replayScale_ * replaySpeedMult_ * frameDt_;
    if (replayT_ >= replayDur_) replayT_ = 0;          // ループ再生
    if (replayT_ < 0) replayT_ = replayDur_;           // 逆再生でループ
    replayS_ = interpSample(simRes_.out, replayT_);
    const SimSample& s = replayS_;
    const double zw = z0_ - s.x;
    acPos_ = {s.yl, s.h, zw};
    acRot_ = {s.gam, -s.psi, -s.phi};   // ロール符号は simTick と同じ(-φ)
    acVisible_ = !cam_.fpv;
    cam_.fpvPose = {s.yl, s.h, zw, s.gam, s.psi, s.phi};
    propAngle_ += 24 * frameDt_ * std::max(0.2, replaySpeedMult_);
    // シネマティックカメラ: 8秒毎に3種のショットを切替
    if (!cam_.fpv) {
        const int shot = (int)(replayT_ / 8.0) % 3;
        cam_.target = {s.yl, s.h + 1.2, zw};
        if (shot == 0) {          // ゆっくり回り込むオービット
            cam_.theta = -0.7 + replayT_ * 0.06;
            cam_.phi = 1.12;
            cam_.dist = std::max(16.0, st_.span * 0.85);
        } else if (shot == 1) {   // 低い真横(水面すれすれ感)
            cam_.theta = -s.psi + PI / 2;
            cam_.phi = 1.45;
            cam_.dist = std::max(10.0, st_.span * 0.55);
        } else {                  // 引きの俯瞰(景色と航跡)
            cam_.theta = -0.4 + replayT_ * 0.02;
            cam_.phi = 0.85;
            cam_.dist = 150;
        }
    }
}

// ---- キャリア・ミッション ----
void Game::startMission(int i) {
    if (simActive_ || i < 0 || i >= (int)missions().size()) return;
    stBackup_ = st_;
    prmBackup_ = prm_;
    missionActive_ = i;               // startSim前に設定(天候ロックのため)
    prm_.funPlane = false;            // ミッションは人力機のみ(終了時にbackupから復元)
    missions()[i].apply(prm_, st_);
    prm_.site = "biwa";                // ミッションは琵琶湖で実施
    r3d_.setSite(prm_.site);
    if (prm_.summer) applySummer(prm_); else applyWindVec(prm_);
    rebuildAircraft();
    setMode("flight");
    startSim();                       // 内部のstopSimはinStartSim_ガードで安全
}

void Game::startContest(double cost) {
    if (simActive_) return;
    if (an_.SM < 2.0) {
        courseMsg_ = u8"大会出場には静的安定余裕 SM 2%以上が必要です";
        courseMsgTimer_ = 6;
        return;
    }
    prmBackup_ = prm_;           // 大会後に設定を復元
    contestActive_ = true;
    contestCost_ = cost;
    // 大会レギュレーション: 手動のみ・体力モデル適用・チート設定を封印
    prm_.funPlane = false;       // 人力機のみ(エンジン機で大会は出られない)
    prm_.site = "biwa";          // 大会は琵琶湖(鳥コン会場)で実施
    r3d_.setSite(prm_.site);
    prm_.auto_ = false;
    prm_.hold = false;
    prm_.stamina = true;         // 実出力はCP+W'balで制限される
    prm_.pjit = true;            // 人間の出力ゆらぎ
    prm_.summer = true;
    prm_.terrainWind = true;
    prm_.CLmax = 1.4;
    prm_.sens = clamp(prm_.sens, 0.3, 1.5);
    prm_.V0 = std::min(prm_.V0, 7.5);   // 人力押し出しの上限
    prm_.mode = "platform";
    setMode("flight");
    startSim();                  // 天候は当日ガチャ(startSim内で抽選)
    buildRivals();               // 同じ天候でライバル機も発進
}

// ---- ライバル機 ----
void Game::buildRivals() {
    rivals_.clear();
    struct Spec { const char* name; unsigned color; double span, tipC, power, rootDia; double ylOff; };
    // 年数で微妙に強くなる(キャリアの緊張感)
    const double yr = std::min(10, career_.st.year - 1) * 4.0;
    const Spec specs[3] = {
        {u8"青嵐大 鳥人会",   0x2c6fd9, 26.0, 0.60, 235 + yr, 105, -45},
        {u8"湖風テクニカ",     0x1E9F5F, 30.0, 0.50, 275 + yr, 115, 45},
        {u8"なぎさ航空研究会", 0x9b59b6, 24.0, 0.62, 255 + yr, 100, -90},
    };
    for (const auto& s : specs) {
        RivalPlane r;
        r.name = s.name;
        r.color = s.color;
        r.span = s.span;
        AircraftParams rst;               // デフォルト機ベースのバリエーション
        rst.span = s.span;
        rst.tipChord = s.tipC;
        rst.powerMax = s.power;
        rst.rootDia = s.rootDia;
        r.prm = prm_;
        r.prm.auto_ = true;               // ライバルはオート操縦
        // 機体設計とオート操縦以外の環境/物理条件はプレイヤーと同一。
        r.prm.pjit = prm_.pjit;
        r.prm.sens = 1.0;
        r.c = aeroPack(rst, analyze(rst, &r.prm), r.prm);
        r.L = makeInitialState(r.c, r.prm, 0);
        r.L.yl = s.ylOff;                 // 横に並んで発進
        rivals_.push_back(std::move(r));
    }
}

void Game::stepRivals() {
    if (rivals_.empty() || !simActive_ || simWait_ > 0) return;
    for (auto& r : rivals_) {
        if (r.L.done) continue;
        r.L.acc += prm_.speed * frameDt_;
        int guard = 0;
        while (r.L.acc >= 0.02 && guard++ < 480 && !r.L.done) {
            r.L.acc -= 0.02;
            updateWeatherJitter(r.prm, true, 0.02);
            if (r.prm.sixdof) stepSim6(r.L, r.c, r.prm, 0.02);
            else stepSim(r.L, r.c, r.prm, 0.02);
        }
    }
}

void Game::setCtrl(int ax, double v) {
    if (!simActive_ || live_.done || live_.auto_) return;
    v = clamp(v, -1.0, 1.0);
    if (ax == 0) live_.eTgt = v;
    else if (ax == 1) live_.ailTgt = v;
    else live_.rudTgt = v;
}

void Game::adjustPower(double d) {
    if (!simActive_ || prm_.auto_) return;
    prm_.P = clamp(std::round((prm_.P + d) / 10) * 10, 0.0, 700.0);
}

void Game::setView(int which) {
    const double cy = appMode_ == "design" ? 1.2 : cam_.target.y;
    const double cz = appMode_ == "design" ? an_.wingLE + an_.MAC * 0.4 : cam_.target.z;
    cam_.target = {0, cy, cz};
    switch (which) {
        case 0: cam_.theta = -0.7; cam_.phi = 1.05; cam_.dist = std::max(12.0, st_.span * 0.85); break;
        case 1: cam_.theta = PI;   cam_.phi = 1.45; cam_.dist = std::max(12.0, st_.span * 0.8); break;
        case 2: cam_.theta = PI/2; cam_.phi = 1.5;  cam_.dist = std::max(8.0, st_.span * 0.4); break;
        case 3: cam_.theta = 0;    cam_.phi = 0.06; cam_.dist = std::max(14.0, st_.span * 0.95); break;
    }
}

void Game::setFPV(bool on) {
    cam_.fpv = on;
    bFpv_.style = on ? 2 : 0;
    if (!simActive_) acVisible_ = true;
    if (on && !simActive_) {
        cam_.fpvPose = {acPos_.x, acPos_.y + 1.0, acPos_.z, 0, 0, 0};
    }
    if (simActive_) acVisible_ = !on;
    else if (!on) acVisible_ = true;
}

// ---- 部位クリックのアンカー座標（配置はAirframeGraphを唯一の参照元とする） ----
std::array<glm::dvec3, (size_t)BodyPart::Count> Game::bodyAnchors() const {
    std::array<glm::dvec3, (size_t)BodyPart::Count> a{};
    const auto placed = graph_.resolve();
    auto pointOnPart = [&](const char* id, const glm::dvec3& local = glm::dvec3(0.0)) {
        for (const auto& item : placed)
            if (!item.mirrored && item.part->id == id)
                return glm::dvec3(item.world * glm::dvec4(local, 1.0));
        return glm::dvec3(0.0);
    };
    a[(size_t)BodyPart::Wing] = pointOnPart("wing.main", {0.0, 0.0, an_.MAC * 0.5});
    a[(size_t)BodyPart::Prop] = pointOnPart("prop.main");
    a[(size_t)BodyPart::Cockpit] = pointOnPart("cockpit", {0.0, 1.0, 0.0});
    a[(size_t)BodyPart::HTail] = pointOnPart("tail.h", {0.0, -0.02, 0.0});
    a[(size_t)BodyPart::VTail] = pointOnPart("tail.v", {0.0, st_.vHeight * 0.5, 0.0});
    if (const Part* root = graph_.find("fuselage")) {
        for (const auto& hp : root->hardpoints) {
            if (hp.id == "hp.ui.tailbeam") {
                a[(size_t)BodyPart::TailBeam] = glm::dvec3(transformMatrix(hp.t)[3]);
                break;
            }
        }
    }
    return a;
}

bool Game::projectToScreen(const glm::dvec3& local, float W, float H, sf::Vector2f& out) const {
    const glm::mat4 proj = glm::perspective(glm::radians(45.0f), W / H, 0.4f, 14000.0f);
    const glm::mat4 view = cam_.viewMatrix();
    const glm::vec4 clip = proj * view * glm::vec4((float)local.x, (float)local.y, (float)local.z, 1.0f);
    if (clip.w <= 0.001f) return false;
    const glm::vec3 ndc = glm::vec3(clip) / clip.w;
    if (ndc.z < -1 || ndc.z > 1) return false;
    out.x = (ndc.x * 0.5f + 0.5f) * W;
    out.y = (1.f - (ndc.y * 0.5f + 0.5f)) * H;
    return true;
}

// ---- 富士川の自由発進(第5弾) ----
bool Game::startPlaceActive() const {
    return appMode_ == "flight" && !simActive_ && !replayActive_ &&
           prm_.site == "fujikawa" && prm_.mode == "runway";
}

void Game::rotateStartHdg(double d) {
    double h = prm_.startHdg + d;
    while (h > 180) h -= 360;
    while (h <= -180) h += 360;
    prm_.startHdg = h;
    placeForMode();
}

bool Game::placeStartFromClick(float mx, float my, float W, float H) {
    // スクリーン座標→NDC→ワールドレイ→地面(y=0)交点
    const glm::mat4 proj = glm::perspective(glm::radians(45.0f), W / H, 0.4f, 14000.0f);
    const glm::mat4 inv = glm::inverse(proj * cam_.viewMatrix());
    const float nx = mx / W * 2.f - 1.f, ny = 1.f - my / H * 2.f;
    const glm::vec4 pN = inv * glm::vec4(nx, ny, -1.f, 1.f);
    const glm::vec4 pF = inv * glm::vec4(nx, ny, 1.f, 1.f);
    if (std::abs(pN.w) < 1e-9f || std::abs(pF.w) < 1e-9f) return false;
    const glm::dvec3 a = glm::dvec3(pN) / (double)pN.w;
    const glm::dvec3 b = glm::dvec3(pF) / (double)pF.w;
    const double dy = a.y - b.y;
    if (std::abs(dy) < 1e-9) return false;
    const double t = a.y / dy;
    if (t < 0 || t > 1) return false;
    const glm::dvec3 p = a + (b - a) * t;
    // 滑走路矩形(ワールド|x|<=15, z=10..860。850×30m再設計)内なら移動。横位置は中心線にスナップ
    if (std::abs(p.x) > site::FUJI_RWY_HALF_WIDTH
        || p.z < site::FUJI_RWY_SOUTH_Z || p.z > site::FUJI_RWY_NORTH_Z)
        return false;
    prm_.startPos = clamp(site::FUJI_START_Z_BASE - p.z,
                          site::FUJI_START_POS_MIN, site::FUJI_START_POS_MAX);
    placeForMode();
    return true;
}

int Game::hitTestPart(sf::Vector2f m, float W, float H, sf::Vector2f& anchorOut) const {
    const auto anchors = bodyAnchors();
    int best = -1;
    float bestD = 1e9f;
    for (size_t i = 0; i < anchors.size(); i++) {
        sf::Vector2f sp;
        if (!projectToScreen(anchors[i], W, H, sp)) continue;
        const float radius = (i == (size_t)BodyPart::Wing) ? 60.f : 40.f;
        const float dx = sp.x - m.x, dy = sp.y - m.y;
        const float d = std::sqrt(dx * dx + dy * dy);
        if (d <= radius && d < bestD) { bestD = d; best = (int)i; anchorOut = sp; }
    }
    return best;
}

// ---- カメラアニメ: 部位クリック→ズーム / パネルを閉じる→標準ビュー復帰 ----
Game::CamGoal Game::camGoalForPart(BodyPart part) const {
    const auto anchors = bodyAnchors();
    CamGoal g{};
    switch (part) {
        case BodyPart::Wing:
            g.target = anchors[(size_t)BodyPart::Wing];
            g.theta = -0.55; g.phi = 1.0; g.dist = std::max(6.0, st_.span * 0.75);
            break;
        case BodyPart::Prop:
            g.target = anchors[(size_t)BodyPart::Prop];
            g.theta = -0.9; g.phi = 1.2; g.dist = std::max(4.0, st_.propDia * 1.6);
            break;
        case BodyPart::Cockpit:
            g.target = anchors[(size_t)BodyPart::Cockpit];
            g.target.z += 0.4;
            g.theta = -0.8; g.phi = 1.25; g.dist = 5.0;
            break;
        case BodyPart::HTail:
            g.target = anchors[(size_t)BodyPart::HTail];
            g.theta = -0.7; g.phi = 1.1; g.dist = std::max(5.0, st_.hSpan * 1.6);
            break;
        case BodyPart::VTail:
            g.target = anchors[(size_t)BodyPart::VTail];
            g.theta = -1.2; g.phi = 1.35; g.dist = std::max(4.5, st_.vHeight * 3.0);
            break;
        case BodyPart::TailBeam:
        default:
            g.target = anchors[(size_t)BodyPart::TailBeam];
            g.theta = PI / 2 - 0.3; g.phi = 1.15; g.dist = std::max(4.0, st_.tailArm * 1.1);
            break;
    }
    // 左ドック(幅360)が画面左を占めるため、部位が残り領域(x=360..W)の中央に
    // 来るよう狙点をカメラ右方向へ補正する。
    const float H = (float)window_.getSize().y;
    const glm::dvec3 camRight(std::cos(g.theta), 0.0, -std::sin(g.theta));
    const double worldPerPixel = 2.0 * g.dist * std::tan(glm::radians(22.5)) / H;
    g.target -= camRight * (180.0 * worldPerPixel);
    return g;
}

Game::CamGoal Game::designViewGoal() const {
    CamGoal g{};
    g.target = {0, 1.2, an_.wingLE + an_.MAC * 0.4};
    g.theta = -0.7; g.phi = 1.05;
    g.dist = std::max(12.0, st_.span * 0.85);
    return g;
}

void Game::animCamTo(glm::dvec3 target, double theta, double phi, double dist) {
    camFromTheta_ = cam_.theta; camFromPhi_ = cam_.phi; camFromDist_ = cam_.dist;
    camFromTarget_ = cam_.target;
    // 最短角度差でθを補間(何周も回転してきた後でも自然に戻る)
    double dtheta = theta - camFromTheta_;
    while (dtheta > PI) dtheta -= 2 * PI;
    while (dtheta < -PI) dtheta += 2 * PI;
    camToTheta_ = camFromTheta_ + dtheta;
    camToPhi_ = phi; camToDist_ = dist; camToTarget_ = target;
    camAnimT_ = 0;
    camAnimActive_ = true;
}

void Game::tickCamAnim() {
    if (!camAnimActive_) return;
    camAnimT_ += frameDt_ / CAM_ANIM_DUR;
    const double t = std::min(1.0, camAnimT_);
    const double u = t < 0.5 ? 4 * t * t * t : 1 - std::pow(-2 * t + 2, 3) / 2;
    cam_.theta = camFromTheta_ + (camToTheta_ - camFromTheta_) * u;
    cam_.phi = camFromPhi_ + (camToPhi_ - camFromPhi_) * u;
    cam_.dist = camFromDist_ + (camToDist_ - camFromDist_) * u;
    cam_.target = camFromTarget_ + (camToTarget_ - camFromTarget_) * u;
    if (t >= 1.0) camAnimActive_ = false;
}

void Game::selectPart(BodyPart part, sf::Vector2f anchor, float W, float H) {
    const CamGoal g = camGoalForPart(part);
    animCamTo(g.target, g.theta, g.phi, g.dist);
    design_.openFor(part, anchor, W, H);
}

void Game::handleEvent(const sf::Event& ev) {
    if (ev.type == sf::Event::Closed) { window_.close(); return; }
    if (ev.type == sf::Event::Resized) {
        r3d_.resize((int)ev.size.width, (int)ev.size.height);
        sf::FloatRect vr(0, 0, (float)ev.size.width, (float)ev.size.height);
        window_.setView(sf::View(vr));
        return;
    }
    // ---- キーボード(操縦) ----
    if (ev.type == sf::Event::KeyPressed) {
        using K = sf::Keyboard;
        switch (ev.key.code) {
            case K::Up: case K::W: setCtrl(0, 1); return;
            case K::Down: case K::S:
                setCtrl(0, -1);
                // 地上ではブレーキ(S/↓)=キャッチャー相当。空中はエレベーターダウン
                if (simActive_ && !live_.done && !live_.auto_) live_.brake = 1;
                return;
            case K::Comma:   if (replayActive_) { bReplaySlow_.onClick(); return; } break;
            case K::Period:  if (replayActive_) { bReplayFast_.onClick(); return; } break;
            // Q/E: 発進前の富士川は機首方位の回転(15°刻み)。シミュ中は従来のエルロン
            // (setCtrlはsimActive時のみ効くので衝突しないが、発進前に限って回転を有効化)
            case K::Left: case K::Q:
                if (ev.key.code == K::Q && startPlaceActive()) { rotateStartHdg(-15); return; }
                setCtrl(1, -1); return;
            case K::Right: case K::E:
                if (ev.key.code == K::E && startPlaceActive()) { rotateStartHdg(+15); return; }
                setCtrl(1, 1); return;
            case K::A: setCtrl(2, -1); return;
            case K::D: setCtrl(2, 1); return;
            case K::Z: case K::X:   // 可変ピッチペラ(機構装備時のみ)
                if (simActive_ && !live_.done && !live_.auto_ && liveC_.varPitch)
                    live_.pitchOfs = clamp(live_.pitchOfs + (ev.key.code == K::X ? 0.2 : -0.2), -1.2, 1.2);
                return;
            case K::G:              // フラップ 0→50→100→0%
                if (simActive_ && !live_.done && !live_.auto_ && liveC_.hasFlap) {
                    live_.flapTgt = live_.flapTgt < 0.25 ? 0.5 : live_.flapTgt < 0.75 ? 1.0 : 0.0;
                    char fb[48];
                    std::snprintf(fb, sizeof(fb), u8"フラップ %.0f%%", live_.flapTgt * 100);
                    flapToast_ = fb;
                    flapToastT_ = 1.5;
                }
                return;
            case K::R: case K::PageUp: case K::Add: case K::RBracket: adjustPower(20); return;
            case K::F: case K::PageDown: case K::Subtract: case K::LBracket: adjustPower(-20); return;
            case K::Space:
                if (appMode_ == "flight") {
                    if (replayActive_) stopReplay();
                    else if (simActive_) stopSim();
                    else startSim();
                }
                return;
            case K::Escape:
                if (replayActive_) stopReplay();
                if (debriefOpen_) debriefOpen_ = false;
                if (design_.isOpen()) design_.close();
                return;
            // ファンクションキー: F1-F6=ミッション挑戦(設計) F7=大会出場 F8=リプレイ F9=デブリーフ
            case K::F1: case K::F2: case K::F3: case K::F4: case K::F5: case K::F6:
                if (appMode_ == "design" && !simActive_) {
                    const int mi = ev.key.code - K::F1;
                    if (mi < (int)missions().size()) startMission(mi);
                }
                return;
            case K::F7:
                if (appMode_ == "design" && !simActive_) {
                    const double cost = Career::totalCost(st_, an_);
                    if (cost <= career_.budget() && career_.lockViolations(st_).empty())
                        startContest(cost);
                }
                return;
            case K::F8:
                if (appMode_ == "flight") {
                    if (replayActive_) stopReplay(); else startReplay();
                }
                return;
            case K::F9:
                if (appMode_ == "flight" && !simActive_) debriefOpen_ = !debriefOpen_;
                return;
            default: break;
        }
    }
    if (ev.type == sf::Event::KeyReleased) {
        using K = sf::Keyboard;
        switch (ev.key.code) {
            case K::Up: case K::W: case K::Down: case K::S: setCtrl(0, 0); live_.brake = 0; return;
            case K::Left: case K::Q: case K::Right: case K::E: setCtrl(1, 0); return;
            case K::A: case K::D: setCtrl(2, 0); return;
            default: break;
        }
    }

    const sf::Vector2f m((float)sf::Mouse::getPosition(window_).x,
                         (float)sf::Mouse::getPosition(window_).y);
    const float W = (float)window_.getSize().x, H = (float)window_.getSize().y;

    // ---- UIウィジェット(手前から) ----
    bool consumed = false;
    if (appMode_ == "flight") {
        consumed |= settings_.handleEvent(ev, m);
        if (!consumed) {
            consumed |= bSettingsIcon_.handle(ev, m);
            consumed |= bFlightExit_.handle(ev, m);
            consumed |= bStart_.handle(ev, m);
            consumed |= bReset_.handle(ev, m);
            consumed |= bStop_.handle(ev, m);
            consumed |= bReplay_.handle(ev, m);
            consumed |= bReplayExit_.handle(ev, m);
            consumed |= bReplaySlow_.handle(ev, m);
            consumed |= bReplayFast_.handle(ev, m);
            consumed |= bDebrief_.handle(ev, m);
            consumed |= bDebriefClose_.handle(ev, m);
            // デブリーフパネル内のクリックはカメラへ流さない
            if (!consumed && debriefOpen_ && ev.type == sf::Event::MouseButtonPressed) {
                const float Wf = (float)window_.getSize().x, Hf = (float)window_.getSize().y;
                if (sf::FloatRect(Wf * 0.12f, 84, Wf * 0.76f, Hf - 240).contains(m))
                    consumed = true;
            }
        }
    } else {
        consumed |= bModeDesign_.handle(ev, m);
        consumed |= bModeFlight_.handle(ev, m);
    }
    if (!consumed) {
        for (auto& b : bViews_) consumed |= b.handle(ev, m);
        consumed |= bFpv_.handle(ev, m);
        consumed |= bHud_.handle(ev, m);
    }
    if (appMode_ == "design") tools_.setDockOpen(design_.isOpen());
    if (!consumed && appMode_ == "design")
        consumed |= tools_.handleEvent(ev, m, W, H);
    if (!consumed && appMode_ == "design")
        consumed |= design_.handleEvent(ev, m, W, H);
    if (consumed) { dragging_ = false; return; }

    // ---- 部位クリック(設計モード: ラベルピル or アンカー近傍) ----
    // 開いていれば別部位クリックで切替、閉じていれば新規オープン。
    // 空クリック(部位に当たらない)はここでは何もせず、通常のカメラ操作へ流す。
    if (!consumed && appMode_ == "design" && !simActive_ &&
        ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left) {
        // 引き出し線ラベル(ピル)のクリック(矩形はdrawUIで毎フレーム更新)
        for (int i = 0; i < (int)BodyPart::Count; i++) {
            if (!partLabelValid_[i] || !partLabelRects_[i].contains(m)) continue;
            sf::Vector2f anchor(partLabelRects_[i].left + partLabelRects_[i].width / 2,
                                partLabelRects_[i].top + partLabelRects_[i].height / 2);
            selectPart((BodyPart)i, anchor, W, H);
            dragging_ = false;
            return;
        }
        // アンカー近傍の直接クリック(従来通り)
        sf::Vector2f anchor;
        const int hit = hitTestPart(m, W, H, anchor);
        if (hit >= 0) {
            selectPart((BodyPart)hit, anchor, W, H);
            dragging_ = false;
            return;
        }
    }

    // ---- カメラ操作(ユーザーがドラッグ/ホイールしたら自動ズームは即キャンセル=手動優先) ----
    if (ev.type == sf::Event::MouseButtonPressed &&
        (ev.mouseButton.button == sf::Mouse::Left || ev.mouseButton.button == sf::Mouse::Right)) {
        dragging_ = true;
        dragPan_ = ev.mouseButton.button == sf::Mouse::Right ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) ||
                   sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
        lastMouse_ = sf::Mouse::getPosition(window_);
        if (ev.mouseButton.button == sf::Mouse::Left) pressPos_ = lastMouse_;
        camAnimActive_ = false;
    }
    if (ev.type == sf::Event::MouseButtonReleased) {
        // 富士川の発進位置クリック(第5弾): ドラッグ量5px未満のリリース=クリック扱いで
        // 滑走路上へ機体を移動(カメラ回転ドラッグとは押下位置からの移動量で判別)
        if (ev.mouseButton.button == sf::Mouse::Left && startPlaceActive() && !cam_.fpv) {
            const sf::Vector2i cur = sf::Mouse::getPosition(window_);
            const double dpx = std::hypot((double)(cur.x - pressPos_.x), (double)(cur.y - pressPos_.y));
            if (dpx < 5.0) placeStartFromClick((float)cur.x, (float)cur.y, W, H);
        }
        pressPos_ = {-9999, -9999};
        dragging_ = false;
    }
    if (ev.type == sf::Event::MouseMoved && dragging_) {
        sf::Vector2i cur = sf::Mouse::getPosition(window_);
        double dx = cur.x - lastMouse_.x, dy = cur.y - lastMouse_.y;
        if (dragPan_) cam_.pan(dx, dy);
        else cam_.rotate(dx, dy);
        lastMouse_ = cur;
    }
    if (ev.type == sf::Event::MouseWheelScrolled) {
        double minD, maxD;
        if (appMode_ == "design") { minD = std::max(3.0, st_.span * 0.12); maxD = std::max(50.0, st_.span * 2.2); }
        else { minD = 4; maxD = 300; }
        cam_.zoom(-ev.mouseWheelScroll.delta * 40, minD, maxD);
        camAnimActive_ = false;
    }
    (void)W;
}

std::string Game::buildFinMsg() const {
    if (simActive_ || simRes_.out.empty()) return "";
    char b[256];
    std::string fin;
    if (simRes_.sparBroken)
        fin = u8"[主桁破損] " + simRes_.brkMsg + "  ";
    else if (simRes_.gearBroken)
        fin = u8"[着陸装置破損] " + simRes_.brkMsg + "  ";
    if (simRes_.offcourse) {
        std::snprintf(b, sizeof(b), u8"コース逸脱で計測終了(距離 %.0f m)", simRes_.dist);
        fin += b;
    } else if (simRes_.nogear) {
        fin += u8"着陸装置がないため滑走できません(⑥で前後2輪を選択)";
    } else if (simRes_.overrun) {
        // 富士川は方位自由化に伴い「滑走距離1050m」基準(琵琶湖レガシー滑走路は従来表記)
        if (prm_.site == "fujikawa")
            std::snprintf(b, sizeof(b), u8"滑走距離%.0fmで離陸速度に届きません(到達 %.0f m)",
                          site::FUJI_OVERRUN_DISTANCE, simRes_.rollDist);
        else
            std::snprintf(b, sizeof(b), u8"滑走路2000m以内に離陸速度へ届きません(到達 %.0f m)", simRes_.dist);
        fin += b;
    } else if (simRes_.landed) {
        std::snprintf(b, sizeof(b), u8"着陸成功! 飛距離 %.0f m(接地 %d回)", simRes_.dist, simRes_.touchdowns);
        fin += b;
    } else if (simRes_.crashed) {
        std::snprintf(b, sizeof(b), simRes_.splash
                      ? u8"着水時に墜落。公式距離 %.0f m"
                      : u8"墜落。公式距離 %.0f m", simRes_.dist);
        fin += b;
    } else if (simRes_.splash) {
        std::snprintf(b, sizeof(b), u8"着水! 飛距離 %.0f m", simRes_.dist);
        fin += b;
        // 発進直後の失速着水: 原因ヒントを出す(デッキ加速込みの飛び出し速度で判定)
        const double pva = prm_.V0 - prm_.wind;
        const double edgeV = std::sqrt(std::max(0.0, pva * std::abs(pva) + 2 * 9.81 * std::sin(3.5 * PI / 180) * 10));
        if (simRes_.time < 8 && simRes_.Vs > edgeV + 0.3) {
            std::snprintf(b, sizeof(b), u8" ⇒ 飛び出し速度%.1f(助走込)に対し失速速度%.1f m/s。発進速度を上げるか翼を大きく",
                          edgeV, simRes_.Vs);
            fin += b;
        }
    } else if (simRes_.dist > 0) {
        if (simRes_.dist >= 29999) fin += u8"30km到達で打切り — 巡航成立!";
        else {
            std::snprintf(b, sizeof(b), simRes_.auto_ ? u8"性能計測: 飛距離 %.0f m(体力切れ)" : u8"時間切れ(%.0f m)", simRes_.dist);
            fin += b;
        }
    }
    if (simRes_.pathAir > 0) {
        std::snprintf(b, sizeof(b), u8" / 対気経路 %.0f m", simRes_.pathAir);
        fin += b;
    }
    fin += " " + physicsTag(simRes_.sixdof, simRes_.assist);
    if (lapTime_ > 0) {
        std::snprintf(b, sizeof(b), u8" / 北パイロン周回 %d分%04.1f秒", (int)(lapTime_ / 60), std::fmod(lapTime_, 60));
        fin += b;
    }
    if (simRes_.newBest) fin += u8" ★自己ベスト更新!";
    else if (save_.best().valid && simRes_.dist > 0) {
        std::snprintf(b, sizeof(b), u8" (自己ベスト %.0fm ", save_.best().dist);
        fin += b + physicsTag(save_.best().sixdof, save_.best().assist) + ")";
    }
    for (const auto& bg : simRes_.badges) fin += " " + bg;
    return fin;
}

void Game::drawUI() {
    const float W = (float)window_.getSize().x, H = (float)window_.getSize().y;
    const sf::Font& font = hud_.font();
    const bool flight = appMode_ == "flight";
    const bool waiting = simActive_ && simWait_ > 0;
    const bool flying = simActive_;
    // テストフライト=ライトテーマ / 設計=ダークテーマ(UI色は関数経由で追従)
    theme::light = flight;
    // 明るい空に直接描く文字用: 半透明白のピル型背景+濃紺文字
    auto pillText = [&](const std::string& s, float x, float y, unsigned size,
                        sf::Color fg, int align, bool bold) {
        const float tw = textWidth(font, s, size, bold);
        const float px0 = align == 1 ? x - tw / 2 : align == 2 ? x - tw : x;
        drawPanelRect(window_, {px0 - 8, y - 3, tw + 16, size * 1.55f + 6},
                      sf::Color(255, 255, 255, 170), sf::Color::Transparent, 0);
        drawText(window_, font, s, x, y, size, fg, align, bold);
    };

    if (!flight) {
        // ヘッダ(黒透過帯+白太字タイトル+左に青いワンポイントバー)
        drawPanelRect(window_, {0, 0, W, 44}, theme::panelSoft(), theme::line(), 1);
        drawPanelRect(window_, {12, 12, 4, 20}, theme::blue(), sf::Color::Transparent, 0);
        drawText(window_, font, u8"鳥人間 機体設計シミュレーター 3D", 24, 10, 17, theme::text(), 0, true);
        bModeDesign_.rect = {380, 8, 90, 28};
        bModeFlight_.rect = {472, 8, 150, 28};
        bModeDesign_.draw(window_, font);
        bModeFlight_.draw(window_, font);
        drawText(window_, font, u8"drag:回転 / wheel:ズーム / 右drag:平行移動", W - 12, 15, 11,
                 theme::textDim(), 2);
        // ヒント(部位クリックで設定パネルを開く)。中央寄せだが、狭いウィンドウでは
        // モードボタン(x..622)と右の操作ヒントに挟まれた範囲にクランプして重なり防止
        {
            const float hw = textWidth(font, u8"部位をクリックして設定", 13, false);
            const float dw = textWidth(font, u8"drag:回転 / wheel:ズーム / 右drag:平行移動", 11, false);
            const float cx = clamp((double)(W / 2), 632.0 + hw / 2, (double)(W - 24 - dw - hw / 2));
            if (632 + hw <= W - 24 - dw)   // 置き場が無いほど狭い場合は非表示
                drawText(window_, font, u8"部位をクリックして設定", cx, 12, 13, theme::textDim(), 1);
        }
        // ビューボタン列: 左ドック(幅360)展開中はパネルの右へずらす(重なり防止)
        float vx = design_.isOpen() ? 372.f : 10.f;
        for (int i = 0; i < 4; i++) { bViews_[i].rect = {vx, 104, 46, 24}; vx += 50; }
        bFpv_.rect = {vx, 104, 130, 24}; vx += 134;
        bHud_.rect = {vx, 104, 96, 24}; vx += 104;
        // ヒント
        char hint[160];
        std::snprintf(hint, sizeof(hint), u8"b=%.1fm S=%.1fm² 全長≈%.1fm — 簡易推算値",
                      st_.span, an_.S, an_.fusLen);
        drawText(window_, font, hint, vx + 8, 108, 10, theme::textDim());
        // ---- 部位選択: 引き出し線+ピル型ラベル(常時表示。ドラッグ中は薄く) ----
        for (auto& v : partLabelValid_) v = false;
        if (!simActive_) {
            static const char* partNames[] = {u8"主翼", u8"プロペラ", u8"コックピット",
                                               u8"水平尾翼", u8"垂直尾翼", u8"テールビーム"};
            // 部位ごとの機体外側へのオフセット方向(px)
            static const sf::Vector2f offs[] = {{-90, -70}, {-110, -10}, {0, 85},
                                                 {95, 60},   {85, -60},  {0, 95}};
            const sf::Vector2f m((float)sf::Mouse::getPosition(window_).x,
                                 (float)sf::Mouse::getPosition(window_).y);
            const float aMul = dragging_ ? 0.5f : 1.f;   // ドラッグ中は半透明
            auto A = [&](sf::Color c) { c.a = (sf::Uint8)(c.a * aMul); return c; };
            const float leftBound = design_.isOpen() ? 372.f : 10.f;
            struct PartLabel { int part; sf::Vector2f anchor; sf::FloatRect r; };
            std::vector<PartLabel> labels;
            const auto anchors = bodyAnchors();
            for (int i = 0; i < (int)BodyPart::Count; i++) {
                sf::Vector2f sp;
                if (!projectToScreen(anchors[i], W, H, sp)) continue;
                if (sp.x < -50 || sp.x > W + 50 || sp.y < -50 || sp.y > H + 50) continue;
                const float tw = textWidth(font, partNames[i], 11, true);
                const float lw = tw + 20, lh = 22;
                sf::Vector2f c = sp + offs[i];
                c.x = (float)clamp((double)c.x, (double)(leftBound + lw / 2), (double)(W - 10 - lw / 2));
                c.y = (float)clamp((double)c.y, 140.0, (double)(H - 130 - lh));
                labels.push_back({i, sp, {c.x - lw / 2, c.y - lh / 2, lw, lh}});
            }
            // ラベル同士の重なりを縦方向の押し出しで解消(小数なので単純なO(n²)を2周)
            std::sort(labels.begin(), labels.end(),
                      [](const PartLabel& a, const PartLabel& b) { return a.r.top < b.r.top; });
            for (int pass = 0; pass < 2; pass++)
                for (size_t j = 1; j < labels.size(); j++)
                    for (size_t k = 0; k < j; k++)
                        if (labels[j].r.intersects(labels[k].r))
                            labels[j].r.top = labels[k].r.top + labels[k].r.height + 4;
            for (const auto& L : labels) {
                partLabelRects_[L.part] = L.r;
                partLabelValid_[L.part] = true;
                const bool selected = design_.isOpen() && (int)design_.currentPart() == L.part;
                const bool hovered = !dragging_ && L.r.contains(m);
                // 引き出し線(1px): アンカー側TEXT_DIM→ラベル側BLUEのグラデーション
                const sf::Vector2f lc(L.r.left + L.r.width / 2, L.r.top + L.r.height / 2);
                sf::Vertex line[2] = {
                    sf::Vertex(L.anchor, A(sf::Color(theme::TEXT_DIM.r, theme::TEXT_DIM.g, theme::TEXT_DIM.b, 150))),
                    sf::Vertex(lc, A(sf::Color(theme::BLUE.r, theme::BLUE.g, theme::BLUE.b, 190)))};
                window_.draw(line, 2, sf::Lines);
                // アンカー点(小さな点)
                sf::CircleShape dot(2.5f);
                dot.setOrigin(2.5f, 2.5f);
                dot.setPosition(L.anchor);
                dot.setFillColor(A(selected || hovered ? theme::BLUE : theme::TEXT_DIM));
                window_.draw(dot);
                // ピル: ダーク地+細枠 / ホバー=BLUE枠+明るく / 選択中=BLUE塗り
                sf::Color fill = selected ? theme::BLUE
                               : hovered  ? sf::Color(40, 52, 70, 235)
                                          : sf::Color(18, 22, 30, 215);
                sf::Color outline = (selected || hovered) ? theme::BLUE : theme::LINE;
                drawPanelRect(window_, L.r, A(fill), A(outline), 1);
                drawText(window_, font, partNames[L.part], L.r.left + L.r.width / 2,
                         L.r.top + 4, 11, A(selected ? sf::Color::White : theme::TEXT), 1, true);
            }
        }
        // メトリクスバー(下部・ツールバー直上)。ツールパネル展開中は隠す(重なり防止)
        if (!tools_.panelOpen())
            hud_.drawMetricsBar(window_, W, H - DesignTools::BAR_H - 64, st_, an_);
        if (an_.SM < 0)
            pillText(u8"静的不安定 — 操縦補助なしでは発散します", W / 2, 56, 15,
                     theme::bad(), 1, true);
        // 設計パネル(部位クリックで開く左ドック)
        design_.draw(window_, font, W, H, (float)frameDt_);
        // ツールバー+ツールパネル(荷重/ポーラー/保存比較/リブ型紙)。
        // 左ドック設計パネルが開いている間はその右側だけに描画(重なり防止)
        tools_.setDockOpen(design_.isOpen());
        tools_.draw(window_, hud_, W, H);
        bSettingsIcon_.visible = bFlightExit_.visible = false;
        bStart_.visible = bReset_.visible = bStop_.visible = false;
    } else {
        // 飛行モード: 全画面
        float vx = 10;
        for (int i = 0; i < 4; i++) { bViews_[i].rect = {vx, 10, 46, 24}; vx += 50; }
        bFpv_.rect = {vx, 10, 130, 24}; vx += 134;
        bHud_.rect = {vx, 10, 96, 24};
        bSettingsIcon_.visible = true;
        bSettingsIcon_.rect = {10, 44, 80, 30};
        bFlightExit_.visible = true;
        bFlightExit_.rect = {W - 130, 10, 120, 28};   // 右上(下部バーのボタンと重ならない)

        float barTop;
        if (replayActive_) {
            // ---- リプレイバー(重なりゼロに再構成: 左=速度クラスタ / 中央=情報+プログレス / 右=終了) ----
            barTop = H - 58;
            drawPanelRect(window_, {0, barTop, W, 58}, theme::panelSoft(), theme::line(), 1);
            const double eff = replayScale_ * replaySpeedMult_;   // シネマ圧縮×ユーザー倍率
            char rb[192];
            std::snprintf(rb, sizeof(rb), u8"リプレイ ×%.2g   %d:%04.1f / %d:%04.1f    距離 %.0f m  高度 %.1f m  対気 %.1f m/s",
                          eff, (int)(replayT_ / 60), std::fmod(replayT_, 60),
                          (int)(replayDur_ / 60), std::fmod(replayDur_, 60),
                          replayS_.officialDist, replayS_.h, replayS_.V);
            drawText(window_, font, rb, W / 2, barTop + 10, 13, theme::text(), 1, true);
            // 中央プログレスバー(テキスト行と分離してy=barTop+40に)
            drawPanelRect(window_, {W / 2 - 220, barTop + 40, 440, 8}, sf::Color(255, 255, 255, 50), sf::Color::Transparent, 0);
            drawPanelRect(window_, {W / 2 - 220, barTop + 40, 440 * (float)(replayT_ / std::max(1.0, replayDur_)), 8},
                          theme::orange(), sf::Color::Transparent, 0);
            bStart_.visible = bReset_.visible = bStop_.visible = bReplay_.visible = false;
            bReplayExit_.visible = true;
            // 左端クラスタ: [◀遅](64x34) → 倍率表示(60幅) → [速▶](64x34)
            bReplaySlow_.visible = bReplayFast_.visible = true;
            bReplaySlow_.rect = {16, barTop + 12, 64, 34};
            bReplayFast_.rect = {16 + 64 + 8 + 60 + 8, barTop + 12, 64, 34};
            bReplaySlow_.draw(window_, font);
            bReplayFast_.draw(window_, font);
            char sb[24];
            std::snprintf(sb, sizeof(sb), u8"×%.2g", eff);
            drawText(window_, font, sb, 16 + 64 + 8 + 30, barTop + 20, 15, theme::blue(), 1, true);
            // 右端: リプレイ終了
            bReplayExit_.rect = {W - 156, barTop + 12, 140, 34};
            bReplayExit_.draw(window_, font);
        } else {
            // 下部フライトデータバー
            const std::string fin = buildFinMsg();
            barTop = hud_.drawFlightBar(window_, W, H,
                simActive_ ? &live_ : nullptr, simActive_ ? &liveC_ : nullptr,
                prm_, simRes_, flying, waiting, fin);
            // バー内ボタン(発進前: 発進+リセット+リプレイ / 待機中: 中止)
            bStart_.visible = !flying;
            bReset_.visible = !flying;
            bStop_.visible = waiting;
            bReplay_.visible = !flying && simRes_.out.size() > 5;
            bReplayExit_.visible = false;
            bReplaySlow_.visible = bReplayFast_.visible = false;
            bDebrief_.visible = bReplay_.visible;
            bStart_.rect = {W / 2 - 220, barTop + 12, 100, 34};
            bReset_.rect = {W / 2 - 112, barTop + 12, 110, 34};
            bStop_.rect = {W / 2 - 250, barTop + 12, 90, 34};
            bReplay_.rect = {W - 276, barTop + 12, 104, 34};
            bDebrief_.rect = {W - 164, barTop + 12, 130, 34};
            bStart_.draw(window_, font);
            bReset_.draw(window_, font);
            bStop_.draw(window_, font);
            bReplay_.draw(window_, font);
            bDebrief_.draw(window_, font);
        }
        // 疲労警告(フラッター/高荷重の蓄積)
        if (flying && !waiting && !live_.sparBroken && live_.fatigue > 0.2) {
            char fw[96];
            std::snprintf(fw, sizeof(fw), u8"! 主桁疲労 %.0f%% — 速度・荷重を下げろ", live_.fatigue * 100);
            pillText(fw, W / 2, 64, 16,
                     live_.fatigue > 0.6 ? theme::bad() : theme::warn(), 1, true);
        }
        // 体力モデル(手動)のW'表示
        if (flying && !waiting && prm_.stamina && !live_.auto_) {
            char sw[64];
            std::snprintf(sw, sizeof(sw), u8"体力 W' %.1f kJ", live_.wbal / 1000);
            pillText(sw, 10, 120, 12,
                     live_.wbal < 3000 ? theme::bad() : theme::warn(), 0, true);
        }
        // 大会の順位表(ライバルとの途中経過)
        if (!rivals_.empty() && (flying || contestActive_)) {
            struct Row { double d; std::string n; unsigned col; };
            std::vector<Row> rows;
            rows.push_back({simActive_ ? live_.officialDist : simRes_.dist, u8"あなた", 0xE8590Cu});
            for (const auto& rv : rivals_) rows.push_back({rv.L.officialDist, rv.name, rv.color});
            std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.d > b.d; });
            float ry = 140;
            // 半透明白の背景板でまとめて視認性を確保
            drawPanelRect(window_, {6, ry - 4, 190, 24 + rows.size() * 15.f},
                          sf::Color(255, 255, 255, 170), sf::Color::Transparent, 0);
            drawText(window_, font, u8"— 大会順位 —", 10, ry, 11, theme::text(), 0, true);
            ry += 16;
            for (size_t i = 0; i < rows.size(); i++) {
                char rl[96];
                std::snprintf(rl, sizeof(rl), u8"%d. %s  %.2f km", (int)i + 1, rows[i].n.c_str(), rows[i].d / 1000);
                drawText(window_, font, rl, 10, ry, 11,
                         sf::Color((rows[i].col >> 16) & 255, (rows[i].col >> 8) & 255, rows[i].col & 255, 235),
                         0, rows[i].n == u8"あなた");
                ry += 15;
            }
        }

        // コース目標・通過メッセージ
        if (flying && !waiting && courseLeg_ >= 0 && courseLeg_ < 2) {
            char cm[96];
            const double dist = courseLeg_ == 0 ? std::hypot(live_.x - 11000, live_.yl)
                                                : std::hypot(live_.x, live_.yl);
            std::snprintf(cm, sizeof(cm), u8"次: %s まで %.1f km",
                          courseLeg_ == 0 ? u8"北パイロン" : u8"プラットフォーム", dist / 1000);
            pillText(cm, 10, 100, 13, sf::Color(0x18, 0x8A, 0x50), 0, true);
        }
        if (courseMsgTimer_ > 0) {
            courseMsgTimer_ -= 1.0 / 60;
            pillText(courseMsg_, W / 2, H * 0.22f, 22, sf::Color(0xB0, 0x78, 0x00), 1, true);
        }
        // サイドパネル(飛行中・非FPV)
        if (flying && !waiting && !cam_.fpv)
            hud_.drawSidePanels(window_, W, H, simRes_, courseLeg_, prm_);
        // FPV計器
        if (flying && !waiting && cam_.fpv && hudEnabled_)
            hud_.drawInstruments(window_, W, H, live_, liveC_, prm_, simRes_);
        // 富士川の自由発進インフォ(発進前のみ): 位置・方位・滑走路残・操作ヒント
        if (startPlaceActive() && !waiting) {
            // 滑走路残: 現在位置から機首方位に沿って滑走路矩形(|x|<=15, z=10..860)を
            // 出るまでの距離(レイと矩形境界の交差)。850×30mへ再設計
            const double hz = site::fujikawaStartWorldZ(prm_.startPos);
            const double hr = prm_.startHdg * PI / 180;
            const double dx = std::sin(hr), dz = -std::cos(hr);
            double tMax = 1e9;
            if (dx > 1e-9) tMax = std::min(tMax, site::FUJI_RWY_HALF_WIDTH / dx);
            else if (dx < -1e-9) tMax = std::min(tMax, -site::FUJI_RWY_HALF_WIDTH / dx);
            if (dz > 1e-9) tMax = std::min(tMax, (site::FUJI_RWY_NORTH_Z - hz) / dz);
            else if (dz < -1e-9) tMax = std::min(tMax, (site::FUJI_RWY_SOUTH_Z - hz) / dz);
            const double remain = tMax > 1e8 ? 0.0 : std::max(0.0, tMax);
            char pb[160];
            std::snprintf(pb, sizeof(pb),
                          u8"発進位置 %.0f m / 方位 %+.0f°(滑走路残 %.0f m)/ クリック:移動 Q,E:回転",
                          prm_.startPos, prm_.startHdg, remain);
            // y=124: 左上の天候ピル(y=80〜105)との重なりを避けた高さ
            pillText(pb, W / 2, 124, 13, sf::Color(0x1B, 0x2A, 0x4A), 1, true);
        }
        // 天候表示(左上)
        {
            char b[128];
            const int wi = prm_.weather;
            const char* wname = wi >= 0 ? siteWeather(prm_.site)[wi].name.c_str() : "-";
            // 飛行中は機首方位を渡す(旋回すると向かい風⇔追い風が正しく変わる)
            const double windPsi = (flying && !waiting) ? live_.psi : 0.0;
            std::snprintf(b, sizeof(b), u8"%s %s  %s  %.0f℃  %s",
                          prm_.site == "fujikawa" ? u8"[富士川]" : u8"[琵琶湖]",
                          todLabel(prm_).c_str(), wname,
                          prm_.temp, windLabel(prm_, windPsi).c_str());
            pillText(b, 10, 80, 12, theme::text(), 0, true);
        }
        // ---- フライトデブリーフ ----
        bDebriefClose_.visible = false;
        if (debriefOpen_ && !flying && !replayActive_ && simRes_.out.size() > 5) {
            const sf::FloatRect dp(W * 0.12f, 84, W * 0.76f, H - 240);
            drawPanelRect(window_, dp, theme::panel(), theme::line(), 1);
            drawText(window_, font, u8"フライトデブリーフ", dp.left + 14, dp.top + 8, 15, theme::text(), 0, true);
            bDebriefClose_.visible = true;
            bDebriefClose_.rect = {dp.left + dp.width - 92, dp.top + 8, 78, 26};
            bDebriefClose_.draw(window_, font);
            // 集計(間引きしつつ統計)
            const auto& out = simRes_.out;
            const size_t step = std::max<size_t>(1, out.size() / 500);
            HUD::ChartSeries sh{{}, sf::Color(0x2C, 0x5F, 0x9E), u8"高度", false};
            HUD::ChartSeries sv{{}, sf::Color(0xE8, 0x59, 0x0C), u8"対気速度", false};
            double maxH = 0, maxHx = 0, maxV = 0, maxVx = 0, maxN = 0, stallT = 0;
            for (size_t i = 0; i < out.size(); i++) {
                const auto& s = out[i];
                if (i % step == 0) {
                    sh.pts.push_back({s.officialDist, s.h});
                    sv.pts.push_back({s.officialDist, s.V});
                }
                if (s.h > maxH) { maxH = s.h; maxHx = s.officialDist; }
                if (s.V > maxV) { maxV = s.V; maxVx = s.officialDist; }
                maxN = std::max(maxN, s.n);
                if (s.V < simRes_.Vs && i > 0) stallT += out[i].t - out[i - 1].t;
            }
            const float chW = (dp.width - 40) / 2, chH = dp.height - 116;
            hud_.drawChart(window_, {dp.left + 14, dp.top + 40, chW, chH}, {sh},
                           u8"距離 m", u8"高度 m", true,
                           {{maxHx, maxH, u8"最高", sf::Color(0x2C, 0x5F, 0x9E)}});
            hud_.drawChart(window_, {dp.left + 26 + chW, dp.top + 40, chW, chH}, {sv},
                           u8"距離 m", "V m/s", true,
                           {{maxVx, maxV, u8"最速", sf::Color(0xE8, 0x59, 0x0C)}}, simRes_.Vs);
            char sm[300];
            std::snprintf(sm, sizeof(sm),
                          u8"飛行時間 %d:%04.1f   距離 %.0f m   最高高度 %.1f m   最高速度 %.1f m/s   最大荷重 %.2f   失速域滞在 %.1f s   平均速度 %.1f m/s%s",
                          (int)(simRes_.time / 60), std::fmod(simRes_.time, 60), simRes_.dist,
                          maxH, maxV, maxN, stallT,
                          simRes_.time > 0 ? simRes_.dist / simRes_.time : 0.0,
                          lapTime_ > 0 ? u8"   ★周回達成" : "");
            drawText(window_, font, sm, dp.left + 14, dp.top + dp.height - 56, 12, theme::text());
            if (!simRes_.brkMsg.empty())
                drawText(window_, font, u8"破損: " + simRes_.brkMsg, dp.left + 14, dp.top + dp.height - 32, 12, theme::bad());
        }
        // フラップ操作トースト(Gキー時1.5秒、画面中央下)
        if (flapToastT_ > 0) {
            flapToastT_ -= frameDt_;
            const float a = (float)clamp(flapToastT_ / 0.3, 0.0, 1.0);   // 最後0.3秒でフェードアウト
            const float tw2 = textWidth(font, flapToast_, 16, true);
            const float tx = W / 2, ty = H - 150;
            drawPanelRect(window_, {tx - tw2 / 2 - 14, ty - 6, tw2 + 28, 34},
                          sf::Color(255, 255, 255, (sf::Uint8)(200 * a)),
                          sf::Color(0x18, 0x8A, 0x50, (sf::Uint8)(255 * a)), 1);
            drawText(window_, font, flapToast_, tx, ty, 16,
                     sf::Color(0x18, 0x8A, 0x50, (sf::Uint8)(255 * a)), 1, true);
        }
        settings_.draw(window_, font, H);
        bSettingsIcon_.draw(window_, font);
        bFlightExit_.draw(window_, font);
    }
    bHud_.style = hudEnabled_ ? 2 : 0;
    for (auto& b : bViews_) b.draw(window_, font);
    bFpv_.draw(window_, font);
    bHud_.draw(window_, font);
}

} // namespace bm
