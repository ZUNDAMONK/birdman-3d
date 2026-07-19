#include "ui/DesignPanel.hpp"
#include "ui/Theme.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace bm {

using namespace theme;

Slider DesignPanel::mkSlider(const std::string& label, double* field, double mn, double mx,
                             double stp, const std::string& unit, double mul,
                             std::function<bool()> vis) {
    Slider s;
    s.label = label; s.unit = unit;
    s.minV = mn; s.maxV = mx; s.step = stp; s.mul = mul;
    s.get = [field] { return *field; };
    s.set = [field](double v) { *field = v; };
    s.onChange = onChange_;
    s.visibleIf = vis;
    return s;
}

void DesignPanel::build(AircraftParams* st, std::function<void()> onChange,
                        std::function<const Analysis*()> getAnalysis,
                        MountEditorCallbacks mountCallbacks) {
    st_ = st; onChange_ = onChange; getAn_ = getAnalysis; mountCb_ = std::move(mountCallbacks);
    groups_.clear(); sections_.clear();
    AircraftParams* p = st;
    using A = AircraftParams;
    auto grp = [&](const std::string& key, std::vector<TplItem> items) {
        groups_.push_back({key, std::move(items)});
        return (int)groups_.size() - 1;
    };
    auto T = [](const char* id, const char* n, const char* d, std::function<void(A&)> f) {
        TplItem t; t.id = id; t.name = n; t.desc = d; t.apply = f; return t;
    };

    const int gAirfoil = grp("airfoil", {
        T("dae31", "DAE-31", u8"Drela設計。HPA定番・高効率", [](A& s) { s.airfoil = "dae31"; }),
        T("dae21", "DAE-21", u8"やや薄翼で高速向き", [](A& s) { s.airfoil = "dae21"; }),
        T("fx76", "FX 76-MP", u8"高揚力・低速安定。厚翼", [](A& s) { s.airfoil = "fx76"; }),
        T("sd7037", "SD-7037", u8"広い速度域で安定", [](A& s) { s.airfoil = "sd7037"; }),
        T("e387", "E-387", u8"実績豊富な中庸型", [](A& s) { s.airfoil = "e387"; }),
        T("ag18", "AG-18", u8"薄翼・軽量。製作容易", [](A& s) { s.airfoil = "ag18"; })});
    const int gWing = grp("wing", {
        T("rect", u8"矩形翼(製作優先)", u8"リブ共通化で楽。誘導抗力やや大", [](A& s) { s.planform = "rect"; s.span = 24; s.rootChord = 0.95; s.tipChord = 0.95; }),
        T("taper", u8"緩テーパー翼(標準)", u8"上位機の定番。バランス型", [](A& s) { s.planform = "taper"; s.span = 28; s.rootChord = 1.0; s.tipChord = 0.55; }),
        T("strong", u8"強テーパー高AR翼", u8"誘導抗力最小。翼端失速注意", [](A& s) { s.planform = "taper"; s.span = 32; s.rootChord = 1.05; s.tipChord = 0.35; }),
        T("crescent", u8"クレセント翼", u8"楕円分布に近づける翼端形状", [](A& s) { s.planform = "crescent"; s.span = 30; s.rootChord = 1.0; s.tipChord = 0.4; }),
        T("jumbo", u8"超大型機(34m)", u8"滞空特化。桁・制御が難しい", [](A& s) { s.planform = "taper"; s.span = 34; s.rootChord = 1.1; s.tipChord = 0.42; }),
        T("compact", u8"コンパクト機(22m)", u8"初参加向け。製作量少", [](A& s) { s.planform = "taper"; s.span = 22; s.rootChord = 1.0; s.tipChord = 0.6; })});
    const int gJig = grp("jig", {
        T("dihedral", u8"ジグ上反角", u8"治具で根元から上反角を付ける", [](A& s) { s.jig = "dihedral"; s.dihedral = 1.5; }),
        T("flat", u8"フラット翼(自重たれ)", u8"地上では翼端が垂れ、揚力で持ち上がる", [](A& s) { s.jig = "flat"; s.dihedral = 0; })});
    const int gRib = grp("rib", {
        T("dense", u8"密(180mm)", u8"翼型保持◎/重量増", [](A& s) { s.ribPitch = 0.18; }),
        T("std", u8"標準(250mm)", u8"多くのチームの標準", [](A& s) { s.ribPitch = 0.25; }),
        T("sparse", u8"疎(350mm)", u8"軽量/フィルム波打ち注意", [](A& s) { s.ribPitch = 0.35; }),
        T("ultra", u8"超密(120mm)", u8"記録狙い上位校流/重い", [](A& s) { s.ribPitch = 0.12; })});
    const int gPlank = grp("plank", {
        T("le", u8"前縁のみ(15%)", u8"最軽量。上下とも前縁15%", [](A& s) { s.plankTop = 15; s.plankBot = 15; s.plankRearTop = 0; s.plankRearBot = 0; }),
        T("std", u8"標準(30%)", u8"層流維持と剛性の両立", [](A& s) { s.plankTop = 30; s.plankBot = 30; s.plankRearTop = 0; s.plankRearBot = 0; }),
        T("dbox", u8"Dボックス(45%)", u8"ねじり剛性◎/重量増", [](A& s) { s.plankTop = 45; s.plankBot = 45; s.plankRearTop = 0; s.plankRearBot = 0; }),
        T("full", u8"フルプランク(60%)", u8"層流翼狙い。最重", [](A& s) { s.plankTop = 60; s.plankBot = 60; s.plankRearTop = 0; s.plankRearBot = 0; }),
        T("dbox2", u8"前後Dボックス", u8"前縁+後縁でねじり剛性確保", [](A& s) { s.plankTop = 30; s.plankBot = 30; s.plankRearTop = 20; s.plankRearBot = 20; }),
        T("lam", u8"上面層流重視", u8"上面50%・下面前縁20%のみ", [](A& s) { s.plankTop = 50; s.plankBot = 20; s.plankRearTop = 0; s.plankRearBot = 0; })});
    const int gAil = grp("ail", {
        T("none", u8"エルロンなし", u8"上反角+ラダーで旋回", [](A& s) { s.ailMode = "none"; s.ailSpanFrac = 0; s.ailChordFrac = 0; }),
        T("small", u8"翼端エルロン(小)", u8"外翼15%。ロール補助", [](A& s) { s.ailMode = "small"; s.ailSpanFrac = 0.15; s.ailChordFrac = 0.25; }),
        T("large", u8"翼端エルロン(大)", u8"外翼25%。機動性重視", [](A& s) { s.ailMode = "large"; s.ailSpanFrac = 0.25; s.ailChordFrac = 0.3; }),
        T("flaperon", u8"フラッペロン(全幅)", u8"横風着陸◎/機構複雑", [](A& s) { s.ailMode = "large"; s.ailSpanFrac = 0.4; s.ailChordFrac = 0.25; }),
        T("allmove", u8"全遊動エルロン", u8"低速高効率/製作精度が命", [](A& s) { s.ailMode = "allmove"; s.ailSpanFrac = 0.2; s.ailChordFrac = 1.0; })});
    const int gFlap = grp("flap", {
        T("none", u8"フラップなし", u8"標準構成", [](A& s) { s.flapSpanFrac = 0; }),
        T("inner", u8"内翼フラップ(35%)", u8"離着陸で失速速度低下。Gキーで展開", [](A& s) { s.flapSpanFrac = 0.35; }),
        T("wide", u8"広幅フラップ(45%)", u8"効果大/重量・抗力増", [](A& s) { s.flapSpanFrac = 0.45; })});
    const int gVPitch = grp("vpitch", {
        T("fixed", u8"固定ピッチ", u8"軽量・単純", [](A& s) { s.varPitch = false; }),
        T("vari", u8"可変ピッチ機構", u8"+0.6kg。飛行中Z/Xで±1.2m変更", [](A& s) { s.varPitch = true; })});
    const int gProp = grp("prop", {
        T("tractor", u8"機首・牽引式", u8"駆動短い/後流が機体に当たる", [](A& s) { s.propConfig = "tractor"; s.propDia = 2.8; s.propPitch = 3.6; }),
        T("pylon", u8"パイロン・推進式", u8"翼上後方。効率◎", [](A& s) { s.propConfig = "pylon"; s.propDia = 3.0; s.propPitch = 3.9; }),
        T("midboom", u8"ブーム中間・推進式", u8"重心に近く駆動も短め", [](A& s) { s.propConfig = "midboom"; s.propDia = 2.6; s.propPitch = 3.3; }),
        T("pusher", u8"機尾・推進式", u8"シャフトが長く重い", [](A& s) { s.propConfig = "pusher"; s.propDia = 2.6; s.propPitch = 3.3; }),
        T("hp", u8"大径・高効率牽引", u8"低回転大径ペラ。効率最優先", [](A& s) { s.propConfig = "tractor"; s.propDia = 3.4; s.propPitch = 4.4; }),
        T("cr", u8"二重反転ペラ", u8"反トルク相殺・効率◎/複雑", [](A& s) { s.propConfig = "tractor"; s.propDia = 3.0; s.propPitch = 3.9; })});
    const int gPropMat = grp("propMat", {
        T("balsa", u8"バルサ", u8"木製・安価/やや重く剛性中", [](A& s) { s.propMat = "balsa"; }),
        T("carp", u8"CFRP(カーボン)", u8"軽量・高剛性→効率◎/高コスト", [](A& s) { s.propMat = "carbon"; })});
    const int gPilot = grp("pilot", {
        T("upright", u8"アップライト", u8"出力◎/前面投影面積大", [](A& s) { s.posture = "upright"; s.cd0Add = 0.004; s.powerMax = 280; }),
        T("semi", u8"セミリカンベント", u8"バランス型(主流)", [](A& s) { s.posture = "semi"; s.cd0Add = 0.002; s.powerMax = 270; }),
        T("recumbent", u8"フルリカンベント", u8"空気抵抗最小", [](A& s) { s.posture = "recumbent"; s.cd0Add = 0.0; s.powerMax = 255; }),
        T("ace", u8"エースパイロット", u8"持続出力が高い", [](A& s) { s.posture = "semi"; s.cd0Add = 0.002; s.powerMax = 320; }),
        T("rookie", u8"初心者パイロット", u8"出力控えめ。機体効率が重要に", [](A& s) { s.posture = "semi"; s.cd0Add = 0.002; s.powerMax = 200; })});
    const int gDrive = grp("drive", {
        T("chain", u8"ねじりチェーン駆動", u8"軽量・高効率/張力管理が必要", [](A& s) { s.drive = "chain"; s.driveEffPct = 97; }),
        T("shaft", u8"シャフト+ベベルギア", u8"確実・整備性◎/やや重い", [](A& s) { s.drive = "shaft"; s.driveEffPct = 93; })});
    const int gGear = grp("gear", {
        T("none", u8"なし(発進台専用)", u8"地上滑走できない", [](A& s) { s.gear = "none"; }),
        T("tandem", u8"前後2輪(座席前後)", u8"軽量で低抵抗", [](A& s) { s.gear = "tandem"; })});
    const int gFairing = grp("fairing", {
        T("off", u8"フェアリングなし", u8"開放型コックピット。製作容易", [](A& s) { s.fairing = false; }),
        T("on", u8"フェアリングあり", u8"CD0 -0.0012。重量増", [](A& s) { s.fairing = true; })});
    const int gSpar = grp("spar", {
        T("const3", u8"一定径・3分割", u8"製作容易/翼端過剰強度", [](A& s) { s.segments = 3; s.rootDia = 100; s.tipDia = 100; }),
        T("taper4", u8"テーパー桁・4分割", u8"軽量(標準)", [](A& s) { s.segments = 4; s.rootDia = 110; s.tipDia = 60; }),
        T("taper5", u8"強テーパー・5分割", u8"最軽量級。接合精度が命", [](A& s) { s.segments = 5; s.rootDia = 115; s.tipDia = 45; }),
        T("ul", u8"超軽量・6分割", u8"強度マージン僅少(破壊注意)", [](A& s) { s.segments = 6; s.rootDia = 105; s.tipDia = 40; }),
        T("hd", u8"高強度・荒天対応", u8"突風でも折れにくい/重い", [](A& s) { s.segments = 4; s.rootDia = 130; s.tipDia = 75; })});
    const int gSparMat = grp("sparMat", {
        T("t700", u8"T700 標準高強度", u8"E=115GPa・圧縮650MPa・費用×1.0", [](A& s) { s.sparMat = "t700"; }),
        T("t800", u8"T800 中間", u8"E=150GPa・圧縮700MPa・費用×1.4", [](A& s) { s.sparMat = "t800"; }),
        T("m40j", u8"M40J 高弾性", u8"E=210GPa・圧縮450MPa・費用×1.9", [](A& s) { s.sparMat = "m40j"; })});
    const int gHt = grp("ht", {
        T("rect", u8"矩形", u8"製作容易", [](A& s) { s.hShape = "rect"; s.hSpan = 3.0; s.hChord = 0.55; }),
        T("taper", u8"テーパー", u8"軽量・標準", [](A& s) { s.hShape = "taper"; s.hSpan = 3.4; s.hChord = 0.6; }),
        T("ellipse", u8"楕円", u8"美しいが製作難", [](A& s) { s.hShape = "ellipse"; s.hSpan = 3.2; s.hChord = 0.62; }),
        T("swept", u8"後退翼", u8"翼端失速が穏やか", [](A& s) { s.hShape = "swept"; s.hSpan = 3.6; s.hChord = 0.58; }),
        T("delta", u8"デルタ型", u8"短テールアームでも面積確保", [](A& s) { s.hShape = "delta"; s.hSpan = 2.8; s.hChord = 0.75; })});
    const int gElev = grp("elev", {
        T("allmove", u8"全可動式", u8"低速で効きが良い(主流)", [](A& s) { s.elevRatio = 1.0; }),
        T("flap30", u8"舵面式(30%)", u8"後縁のみ可動", [](A& s) { s.elevRatio = 0.3; })});
    const int gVt = grp("vt", {
        T("rect", u8"矩形", u8"製作容易", [](A& s) { s.vShape = "rect"; s.vHeight = 1.2; s.vChord = 0.55; }),
        T("swept", u8"後退", u8"アームを稼げる", [](A& s) { s.vShape = "swept"; s.vHeight = 1.3; s.vChord = 0.6; }),
        T("ellipse", u8"楕円", u8"軽量だが製作難", [](A& s) { s.vShape = "ellipse"; s.vHeight = 1.25; s.vChord = 0.6; }),
        T("delta", u8"デルタ", u8"根元が広く剛性高い", [](A& s) { s.vShape = "delta"; s.vHeight = 1.4; s.vChord = 0.65; }),
        T("dorsal", u8"背びれ型", u8"自然な流線形", [](A& s) { s.vShape = "dorsal"; s.vHeight = 1.35; s.vChord = 0.6; })});
    const int gRud = grp("rud", {
        T("allmove", u8"全可動式", u8"垂直尾翼全体が回る", [](A& s) { s.rudRatio = 1.0; }),
        T("flap35", u8"舵面式(35%)", u8"後縁35%可動", [](A& s) { s.rudRatio = 0.35; })});
    const int gBoomWing = grp("boomWing", {
        T("none", u8"ビーム翼なし", u8"標準構成", [](A& s) { s.boomWing = "none"; }),
        T("LR", u8"左右対称", u8"安定性寄与。左右同時装備", [](A& s) { s.boomWing = "LR"; }),
        T("L", u8"左のみ", u8"片側装備(試験用途)", [](A& s) { s.boomWing = "L"; }),
        T("R", u8"右のみ", u8"片側装備(試験用途)", [](A& s) { s.boomWing = "R"; })});

    tplSel_ = {{"airfoil","dae31"},{"wing","taper"},{"jig","dihedral"},{"rib","std"},{"plank","std"},
               {"ail","none"},{"flap","none"},{"vpitch","fixed"},
               {"prop","tractor"},{"propMat","balsa"},{"pilot","semi"},{"drive","chain"},
               {"gear","tandem"},{"spar","taper4"},{"sparMat","t700"},{"ht","taper"},{"vt","swept"},{"elev","allmove"},
               {"rud","allmove"},{"fairing","off"},{"boomWing","none"}};

    // ボタンのonClick配線
    for (auto& g : groups_)
        for (auto& it : g.items) {
            it.btn.label = it.name;
            it.btn.charSize = 11;
            const std::string key = g.key, id = it.id;
            auto apply = it.apply;
            it.btn.onClick = [this, key, id, apply] {
                tplSel_[key] = id;
                apply(*st_);
                if (onChange_) onChange_();
            };
        }

    auto sec = [&](const char* title, std::vector<int> gs, std::vector<Slider> sl,
                   std::function<std::string()> note = nullptr) {
        Section s; s.title = title; s.groups = std::move(gs); s.sliders = std::move(sl); s.note = note;
        sections_.push_back(std::move(s));
    };
    sec(u8"① 主翼(翼型・平面形)", {gAirfoil, gWing, gJig}, {
        mkSlider(u8"翼幅", &p->span, 18, 38, 0.5, " m"),
        mkSlider(u8"翼根コード", &p->rootChord, 0.6, 1.4, 0.05, " m"),
        mkSlider(u8"翼端コード", &p->tipChord, 0.2, 1.1, 0.05, " m", 1, [p] { return p->planform != "rect"; }),
        mkSlider(u8"主翼前縁位置(機首から)", &p->wingX, 0.8, 3.0, 0.05, " m"),
        mkSlider(u8"上反角", &p->dihedral, 0, 7, 0.25, u8" °"),
        mkSlider(u8"捻り下げ(翼端)", &p->washout, 0, 6, 0.5, u8" °"),
        mkSlider(u8"主翼取付角(地上迎角。離陸距離に直結)", &p->incidence, -1, 8, 0.5, u8" °"),
        mkSlider(u8"後退角", &p->sweep, 0, 12, 1, u8" °")});
    sec(u8"② リブ配置", {gRib}, {mkSlider(u8"リブピッチ", &p->ribPitch, 0.12, 0.4, 0.01, " m")},
        [this] { const Analysis* a = getAn_(); char b[48];
                 std::snprintf(b, sizeof(b), u8"リブ枚数: %d 枚", a ? a->nRibs : 0); return std::string(b); });
    sec(u8"③ プランク(外皮)", {gPlank}, {
        mkSlider(u8"上面 前縁プランク率", &p->plankTop, 0, 70, 5, " %"),
        mkSlider(u8"下面 前縁プランク率", &p->plankBot, 0, 70, 5, " %"),
        mkSlider(u8"上面 後縁プランク率", &p->plankRearTop, 0, 40, 5, " %"),
        mkSlider(u8"下面 後縁プランク率", &p->plankRearBot, 0, 40, 5, " %")});
    sec(u8"④ エルロン・フラップ", {gAil, gFlap}, {
        mkSlider(u8"エルロンスパン(半翼比)", &p->ailSpanFrac, 0.1, 0.4, 0.01, "%", 100, [p] { return p->ailMode != "none"; }),
        mkSlider(u8"エルロンコード比", &p->ailChordFrac, 0.15, 0.4, 0.01, "%", 100, [p] { return p->ailMode != "none"; })});
    {
        Slider blades;
        blades.label = u8"ブレード枚数"; blades.unit = u8" 枚";
        blades.minV = 1; blades.maxV = 4; blades.step = 1;
        blades.get = [p] { return (double)p->propBlades; };
        blades.set = [p](double v) { p->propBlades = (int)std::lround(v); };
        blades.onChange = onChange_;
        sec(u8"⑤ プロペラ配置・素材", {gProp, gPropMat, gVPitch}, {
            mkSlider(u8"プロペラ直径", &p->propDia, 2.0, 3.8, 0.1, " m"),
            mkSlider(u8"ピッチ(1回転の前進距離)", &p->propPitch, 2.5, 7.0, 0.1, " m"), blades});
    }
    sec(u8"⑥ パイロット座席・駆動", {gPilot, gDrive, gGear, gFairing}, {
        mkSlider(u8"座席位置(機首から)", &p->seatX, 0.2, 2.5, 0.05, " m"),
        mkSlider(u8"パイロット体重", &p->pilotW, 45, 80, 1, " kg"),
        mkSlider(u8"持続出力(目安)", &p->powerMax, 160, 360, 5, " W"),
        mkSlider(u8"駆動伝達効率", &p->driveEffPct, 85, 99, 1, " %")},
        [this] { const Analysis* a = getAn_(); char b[80];
                 std::snprintf(b, sizeof(b), u8"駆動距離: %.2f m(長いほど重量・損失増)", a ? a->driveDist : 0.0); return std::string(b); });
    {
        Slider segs;
        segs.label = u8"分割数"; segs.unit = u8" 本";
        segs.minV = 1; segs.maxV = 6; segs.step = 1;
        segs.get = [p] { return (double)p->segments; };
        segs.set = [p](double v) { p->segments = (int)std::lround(v); };
        segs.onChange = onChange_;
        sec(u8"⑦ CFRP主桁", {gSpar, gSparMat}, {
            segs,
            mkSlider(u8"翼根径", &p->rootDia, 70, 150, 5, " mm"),
            mkSlider(u8"翼端径", &p->tipDia, 30, 150, 5, " mm")},
            [this, p] { const Analysis* a = getAn_(); char b[96];
                std::snprintf(b, sizeof(b), u8"桁 %.1fkg + 接合 %.1fkg / 安全率 %.2f", a ? a->wSpar : 0.0,
                              a ? a->wJoints : 0.0, a ? a->sparSF : 0.0); return std::string(b); });
    }
    sec(u8"⑧ 水平尾翼・エレベーター", {gHt, gElev}, {
        mkSlider(u8"尾翼アーム(主翼AC→尾翼)", &p->tailArm, 3.0, 8.0, 0.1, " m"),
        mkSlider(u8"水平尾翼スパン", &p->hSpan, 2.0, 5.0, 0.1, " m"),
        mkSlider(u8"水平尾翼コード", &p->hChord, 0.35, 0.9, 0.05, " m"),
        mkSlider(u8"エレベーター舵面比", &p->elevRatio, 0.2, 0.5, 0.05, "%", 100, [p] { return p->elevRatio < 1; })});
    sec(u8"⑨ 垂直尾翼・ラダー", {gVt, gRud}, {
        mkSlider(u8"垂直尾翼高さ", &p->vHeight, 0.8, 2.2, 0.05, " m"),
        mkSlider(u8"垂直尾翼コード", &p->vChord, 0.35, 0.9, 0.05, " m"),
        mkSlider(u8"ラダー舵面比", &p->rudRatio, 0.2, 0.5, 0.05, "%", 100, [p] { return p->rudRatio < 1; })});
    sec(u8"⑩ テールブーム・ビーム翼", {gBoomWing}, {
        mkSlider(u8"テールブーム径", &p->boomDia, 50, 120, 5, " mm"),
        mkSlider(u8"翼幅(片側)", &p->boomWingSpan, 0.3, 2.0, 0.1, " m", 1, [p] { return p->boomWing != "none"; }),
        mkSlider(u8"翼弦長", &p->boomWingChord, 0.15, 0.6, 0.05, " m", 1, [p] { return p->boomWing != "none"; }),
        mkSlider(u8"取付位置(0=翼付近/1=尾翼付近)", &p->boomWingPos, 0, 1, 0.05, "", 1, [p] { return p->boomWing != "none"; })});

    closeBtn_.label = u8"×";
    closeBtn_.style = 3;
    closeBtn_.charSize = 14;
    closeBtn_.onClick = [this] { close(); };

    const char* mountLabels[6] = {u8"X（右＋）", u8"Y（上＋）", u8"Z（後＋）",
                                  "Pitch", "Yaw", "Roll"};
    for (int i = 0; i < 6; ++i) {
        Slider& slider = mountSliders_[(std::size_t)i];
        slider.label = mountLabels[i];
        slider.unit = i < 3 ? " m" : u8" °";
        slider.minV = i < 3 ? -10.0 : -180.0;
        slider.maxV = i < 3 ? 10.0 : 180.0;
        slider.step = i < 3 ? 0.05 : 1.0;
        slider.get = [this, i] {
            return i < 3 ? pendingMount_.offset.pos[i] : pendingMount_.offset.rotDeg[i - 3];
        };
        slider.set = [this, i](double value) {
            if (i < 3) pendingMount_.offset.pos[i] = value;
            else pendingMount_.offset.rotDeg[i - 3] = value;
            mountMessage_.clear();
        };
    }
    mountMirror_.onClick = [this] {
        pendingMount_.mirror = pendingMount_.mirror == MirrorMode::Pair ? MirrorMode::None : MirrorMode::Pair;
        mountMessage_.clear();
    };
    mountApply_.label = u8"適用"; mountApply_.style = 1;
    mountApply_.onClick = [this] {
        std::string error;
        if (mountEditable_ && mountCb_.apply && mountCb_.apply(part_, pendingMount_, error)) {
            mountMessage_ = u8"適用しました";
            loadMountEditor();
        } else if (!error.empty()) mountMessage_ = u8"適用できません: " + error;
    };
    mountCancel_.label = u8"取消";
    mountCancel_.onClick = [this] { loadMountEditor(); mountMessage_ = u8"変更を取り消しました"; };
    mountReset_.label = u8"標準位置";
    mountReset_.onClick = [this] {
        std::string error;
        if (mountEditable_ && mountCb_.reset && mountCb_.reset(part_, error)) {
            loadMountEditor(); mountMessage_ = u8"標準位置へ戻しました";
        } else if (!error.empty()) mountMessage_ = u8"戻せません: " + error;
    };
    mountUndo_.label = u8"元に戻す";
    mountUndo_.onClick = [this] { if (mountCb_.undo && mountCb_.undo()) loadMountEditor(); };
    mountRedo_.label = u8"やり直す";
    mountRedo_.onClick = [this] { if (mountCb_.redo && mountCb_.redo()) loadMountEditor(); };
}

void DesignPanel::loadMountEditor() {
    Mount value;
    mountEditable_ = mountCb_.get && mountCb_.get(part_, value);
    if (mountEditable_) pendingMount_ = std::move(value);
}

// 部位→関連セクション(sections_のindex)。build()内のsec()呼び出し順と対応:
// 0=①主翼 1=②リブ 2=③プランク 3=④エルロン/フラップ 4=⑤プロペラ 5=⑥パイロット/駆動
// 6=⑦CFRP主桁 7=⑧水平尾翼 8=⑨垂直尾翼 9=⑩テールビーム翼
std::vector<int> DesignPanel::sectionsForPart(BodyPart part) const {
    switch (part) {
        case BodyPart::Wing:     return {0, 1, 2, 3, 6};
        case BodyPart::Prop:     return {4};
        case BodyPart::Cockpit:  return {5};
        case BodyPart::HTail:    return {7};
        case BodyPart::VTail:    return {8};
        case BodyPart::TailBeam: return {9};
        default: return {};
    }
}

// 左ドックの矩形: 幅360、上端=ヘッダ下48px、下端=下部メトリクスバー帯の上
// (メトリクス帯は y = H - DesignTools::BAR_H(34) - 64 - 6 = H - 104 から始まる)
void DesignPanel::updateDockRect(float W, float H) {
    (void)W;
    rect_ = {0, 48, 360, std::max(200.f, (H - 104) - 48)};
}

void DesignPanel::openFor(BodyPart part, sf::Vector2f anchor, float W, float H) {
    (void)anchor;   // 旧ポップアップAPIの名残(位置は左ドック固定になった)
    part_ = part;
    activeSections_ = sectionsForPart(part);
    scroll_ = 0;
    open_ = true;
    animT_ = 0;
    updateDockRect(W, H);
    mountMessage_.clear();
    loadMountEditor();
    relayout();
}

void DesignPanel::close() { open_ = false; }

void DesignPanel::relayout() {
    float y = rect_.top + 40 - scroll_;
    const float x0 = rect_.left + 12, w = rect_.width - 24;
    if (mountEditable_) {
        y += 26;
        for (auto& slider : mountSliders_) { slider.rect = {x0, y, w, 34}; y += 38; }
        mountMirror_.rect = {x0, y, w, 28}; y += 34;
        const float bw = (w - 8) / 3;
        mountApply_.rect = {x0, y, bw, 28};
        mountCancel_.rect = {x0 + bw + 4, y, bw, 28};
        mountReset_.rect = {x0 + (bw + 4) * 2, y, bw, 28}; y += 34;
        mountUndo_.rect = {x0, y, (w - 4) / 2, 26};
        mountRedo_.rect = {x0 + (w - 4) / 2 + 4, y, (w - 4) / 2, 26}; y += 32;
        if (!mountMessage_.empty()) y += 26;
        y += 10;
    } else {
        y += 48;
    }
    for (int si : activeSections_) {
        if (si < 0 || si >= (int)sections_.size()) continue;
        auto& sec = sections_[si];
        y += 26;   // セクションタイトル行
        for (int gi : sec.groups) {
            auto& g = groups_[gi];
            for (size_t i = 0; i < g.items.size(); i++) {
                const float bw = (w - 7) / 2;
                const float bx = x0 + (i % 2) * (bw + 7);
                g.items[i].btn.rect = {bx, y + (float)(i / 2) * 44, bw, 40};
                g.items[i].btn.on = tplSel_[g.key] == g.items[i].id;
            }
            y += ((g.items.size() + 1) / 2) * 44.f + 8;
        }
        for (auto& sl : sec.sliders) {
            if (!sl.visible()) continue;
            sl.rect = {x0, y, w, 34};
            y += 38;
        }
        if (sec.note) y += 18;
        y += 12;
    }
    contentH_ = y + scroll_ - (rect_.top + 40);
}

bool DesignPanel::handleEvent(const sf::Event& ev, sf::Vector2f m, float W, float H) {
    (void)W; (void)H;
    if (!open_) return false;
    if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::Escape) {
        close();
        return true;
    }
    const float innerH = rect_.height - 40;
    if (ev.type == sf::Event::MouseWheelScrolled && rect_.contains(m)) {
        scroll_ = std::max(0.f, std::min(std::max(0.f, contentH_ - innerH),
                                         scroll_ - ev.mouseWheelScroll.delta * 40));
        return true;
    }
    relayout();
    bool consumed = false;
    consumed |= closeBtn_.handle(ev, m);
    if (mountEditable_) {
        for (auto& slider : mountSliders_) consumed |= slider.handle(ev, m);
        consumed |= mountMirror_.handle(ev, m);
        consumed |= mountApply_.handle(ev, m);
        consumed |= mountCancel_.handle(ev, m);
        consumed |= mountReset_.handle(ev, m);
        consumed |= mountUndo_.handle(ev, m);
        consumed |= mountRedo_.handle(ev, m);
    }
    for (int si : activeSections_) {
        if (si < 0 || si >= (int)sections_.size()) continue;
        auto& sec = sections_[si];
        for (int gi : sec.groups)
            for (auto& it : groups_[gi].items)
                consumed |= it.btn.handle(ev, m);
        for (auto& sl : sec.sliders)
            consumed |= sl.handle(ev, m);
    }
    if (!consumed && ev.type == sf::Event::MouseButtonPressed && rect_.contains(m))
        consumed = true;   // パネル内クリックはカメラへ流さない(空クリックでは閉じない)
    return consumed;
}

void DesignPanel::draw(sf::RenderTarget& rt, const sf::Font& font, float W, float H, float dt) {
    (void)W; (void)H;
    if (!open_) return;
    animT_ = std::min(1.f, animT_ + (float)(dt / 0.18));
    // ease-out cubic: t=1-(1-u)^3
    const float u = 1.f - std::pow(1.f - animT_, 3.f);
    const float slideY = (1.f - u) * 14.f;
    const sf::Uint8 alphaMul = (sf::Uint8)std::round(u * 255.f);

    // スライド分だけ矩形を仮に持ち上げてレイアウト(閉時が0オフセットの最終位置)
    const float savedTop = rect_.top;
    rect_.top += slideY;
    relayout();

    auto A = [&](sf::Color c) { c.a = (sf::Uint8)((int)c.a * alphaMul / 255); return c; };
    static const char* names[] = {u8"主翼", u8"プロペラ", u8"コックピット", u8"水平尾翼", u8"垂直尾翼", u8"テールビーム"};

    const float headerH = 40;
    drawPanelRect(rt, rect_, A(PANEL), A(sf::Color(BLUE.r, BLUE.g, BLUE.b, 150)), 1);
    drawPanelRect(rt, {rect_.left, rect_.top, rect_.width, headerH}, A(BLUE_DIM), sf::Color::Transparent, 0);
    drawText(rt, font, names[std::min((int)part_, 5)], rect_.left + 14, rect_.top + 11, 15, A(TEXT), 0, true);
    closeBtn_.rect = {rect_.left + rect_.width - 32, rect_.top + 8, 24, 24};
    closeBtn_.draw(rt, font, alphaMul);

    const float top = rect_.top, panelH = rect_.height;
    float y = rect_.top + headerH - scroll_;
    const float x0 = rect_.left + 12, w = rect_.width - 24;
    if (mountEditable_) {
        if (y + 20 > top + headerH && y < top + panelH) {
            drawPanelRect(rt, {x0 - 4, y, w + 8, 22}, A(BLUE_DIM), sf::Color::Transparent, 0);
            drawText(rt, font, u8"取付位置・角度", x0 + 4, y + 3, 13, A(sf::Color::White), 0, true);
        }
        y += 26;
        for (auto& slider : mountSliders_) { if (slider.rect.top + 34 > top + headerH && slider.rect.top < top + panelH) slider.draw(rt, font, alphaMul); y += 38; }
        mountMirror_.label = pendingMount_.mirror == MirrorMode::Pair ? u8"左右ミラー: ON" : u8"左右ミラー: OFF";
        mountMirror_.style = pendingMount_.mirror == MirrorMode::Pair ? 2 : 0;
        mountUndo_.enabled = mountCb_.canUndo && mountCb_.canUndo();
        mountRedo_.enabled = mountCb_.canRedo && mountCb_.canRedo();
        for (Button* button : {&mountMirror_, &mountApply_, &mountCancel_, &mountReset_, &mountUndo_, &mountRedo_})
            if (button->rect.top + button->rect.height > top + headerH && button->rect.top < top + panelH) button->draw(rt, font, alphaMul);
        y += 100;
        if (!mountMessage_.empty()) {
            const bool bad = mountMessage_.find(u8"ません") != std::string::npos;
            drawText(rt, font, mountMessage_, x0, y, 10, A(bad ? BAD : TEXT_DIM));
            y += 26;
        }
        y += 10;
    } else {
        if (y + 36 > top + headerH && y < top + panelH)
            drawText(rt, font, u8"この基準部品の取付位置は編集できません", x0, y + 14, 11, A(TEXT_DIM));
        y += 48;
    }
    for (int si : activeSections_) {
        if (si < 0 || si >= (int)sections_.size()) continue;
        auto& sec = sections_[si];
        if (y + 20 > top + headerH && y < top + panelH) {
            drawPanelRect(rt, {x0 - 4, y, w + 8, 22}, A(BLUE_DIM), sf::Color::Transparent, 0);
            drawText(rt, font, sec.title, x0 + 4, y + 3, 13, A(sf::Color::White), 0, true);
        }
        y += 26;
        for (int gi : sec.groups) {
            auto& g = groups_[gi];
            for (auto& it : g.items) {
                const auto& r = it.btn.rect;
                if (r.top + r.height < top + headerH || r.top > top + panelH) continue;
                it.btn.draw(rt, font, alphaMul);
                // 説明文(小さく)
                drawText(rt, font, it.desc, r.left + 4, r.top + 26, 8,
                         A(it.btn.on ? BLUE : TEXT_DIM));
            }
            y += ((g.items.size() + 1) / 2) * 44.f + 8;
        }
        for (auto& sl : sec.sliders) {
            if (!sl.visible()) continue;
            if (sl.rect.top + 34 > top + headerH && sl.rect.top < top + panelH)
                sl.draw(rt, font, alphaMul);
            y += 38;
        }
        if (sec.note) {
            if (y > top + headerH && y < top + panelH)
                drawText(rt, font, sec.note(), x0, y, 11, A(TEXT_DIM));
            y += 18;
        }
        y += 12;
    }
    rect_.top = savedTop;
}

} // namespace bm
