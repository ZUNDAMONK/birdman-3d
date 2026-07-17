#include "ui/DesignTools.hpp"
#include "core/Material.hpp"
#include "ui/Theme.hpp"
#include "core/Aircraft.hpp"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <fstream>
#include <algorithm>
#if defined(_WIN32)
#include <direct.h>
#define MKDIR(d) _mkdir(d)
#else
#include <sys/stat.h>
#define MKDIR(d) mkdir(d, 0755)
#endif

namespace bm {

static const double PI = 3.14159265358979323846;
using namespace theme;
static const sf::Color& INK = TEXT;
static const sf::Color& INKSOFT = TEXT_DIM;
static const sf::Color& ACCENT = BLUE;
static const sf::Color& OKC = OK;
static const sf::Color& WARNC = WARN;
static const sf::Color& BADC = BAD;

// リブ型紙用の翼型座標 (Renderer3Dと同じNACA風 camber4% 厚11%)
static void airfoilPoly(std::vector<std::pair<double, double>>& up, std::vector<std::pair<double, double>>& lo) {
    const int n = 16;
    for (int i = 0; i <= n; i++) {
        double x = 0.5 * (1 - std::cos(PI * i / n));
        double t = 0.11, m = 0.04, pp = 0.4;
        double yt = 5 * t * (0.2969 * std::sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
        double yc = x < pp ? m / (pp * pp) * (2 * pp * x - x * x)
                           : m / ((1 - pp) * (1 - pp)) * ((1 - 2 * pp) + 2 * pp * x - x * x);
        up.push_back({x, yc + yt});
        lo.push_back({x, yc - yt});
    }
}

// リブのグルーピング (JS ribGroups)
struct RibGroup { int no; int c, d; double y; int count; };
static std::vector<RibGroup> ribGroups(const AircraftParams& st) {
    const double half = st.span / 2;
    std::vector<RibGroup> groups;
    for (double y = st.ribPitch; y < half * 0.995; y += st.ribPitch) {
        const double t = y / half;
        const int c = (int)std::lround(chordAt(st, t) * 1000 / 5) * 5;
        const int d = (int)std::lround((lerp(st.rootDia, st.tipDia, t) + 2) / 5) * 5;
        bool found = false;
        for (auto& g : groups)
            if (g.c == c && g.d == d) { g.count += 2; found = true; break; }
        if (!found) groups.push_back({(int)groups.size() + 1, c, d, y, 2});
    }
    return groups;
}

void DesignTools::build(AircraftParams* st, SimParams* prm,
                        std::function<const Analysis*()> getAn, Callbacks cb) {
    st_ = st; prm_ = prm; getAn_ = getAn; cb_ = cb;
    const char* labels[6] = {u8"荷重・たわみ", u8"ポーラー", u8"保存・比較", u8"リブ型紙", u8"ミッション", u8"キャリア"};
    for (int i = 0; i < 6; i++) {
        tb_[i].label = labels[i];
        tb_[i].charSize = 12;
        tb_[i].onClick = [this, i] {
            tool_ = tool_ == i ? -1 : i;
            if (tool_ == 2) reloadDesigns();
            if (tool_ != 0 && loadApply_) {   // パネルを閉じたらたわみ解除
                loadApply_ = false;
                if (cb_.onFlexPreview) cb_.onFlexPreview(loadN_, false);
            }
        };
    }
    ldN_.label = u8"荷重倍数 n";
    ldN_.unit = " G";
    ldN_.minV = 1; ldN_.maxV = 2.5; ldN_.step = 0.1;
    ldN_.get = [this] { return loadN_; };
    ldN_.set = [this](double v) { loadN_ = v; };
    ldN_.onChange = [this] { if (loadApply_ && cb_.onFlexPreview) cb_.onFlexPreview(loadN_, true); };
    ldApply_.onClick = [this] {
        loadApply_ = !loadApply_;
        if (cb_.onFlexPreview) cb_.onFlexPreview(loadN_, loadApply_);
    };
    svSave_.label = u8"現在の設計を保存";
    svSave_.style = 1;
    svSave_.onClick = [this] {
        reloadDesigns();
        DesignEntry e;
        e.st = *st_;
        char nm[32];
        std::snprintf(nm, sizeof(nm), u8"設計%d", (int)designs_.size() + 1);
        e.name = nm;
        e.dist = cb_.lastDist ? cb_.lastDist() : -1;
        fillMetrics(e, *prm_);
        designs_.push_back(e);
        saveDesigns(designs_);
        syncDesignButtons();
    };
    rbExport_.label = u8"SVGを書き出す(実寸)";
    rbExport_.style = 1;
    rbExport_.onClick = [this] { exportRibsSVG(); };
    svExportUrl_.label = u8"JS版URL書き出し";
    svExportUrl_.onClick = [this] { exportJsUrl(); };
    // ミッション挑戦ボタン
    msnBtns_.resize(missions().size());
    for (size_t i = 0; i < msnBtns_.size(); i++) {
        msnBtns_[i].label = u8"挑戦";
        msnBtns_[i].style = 1;
        msnBtns_[i].charSize = 11;
        msnBtns_[i].onClick = [this, i] {
            if (cb_.onStartMission) cb_.onStartMission((int)i);
        };
    }
    bContest_.style = 1;
    bContest_.onClick = [this] {
        if (!cb_.career || !getAn_ || !getAn_()) return;
        const double cost = Career::totalCost(*st_, *getAn_());
        if (cost <= cb_.career->budget() && cb_.career->lockViolations(*st_).empty()
            && getAn_()->SM >= 2.0 && cb_.onStartContest)
            cb_.onStartContest(cost);
    };
}

void DesignTools::reloadDesigns() {
    designs_ = loadDesigns();
    designsLoaded_ = true;
    syncDesignButtons();
}

void DesignTools::syncDesignButtons() {
    svLoad_.resize(designs_.size());
    svDel_.resize(designs_.size());
    for (size_t i = 0; i < designs_.size(); i++) {
        svLoad_[i].label = u8"読込"; svLoad_[i].charSize = 11;
        svLoad_[i].onClick = [this, i] {
            if (i < designs_.size() && cb_.onLoadDesign) cb_.onLoadDesign(designs_[i].st);
        };
        svDel_[i].label = u8"削除"; svDel_[i].charSize = 11;
        svDel_[i].onClick = [this, i] {
            if (i < designs_.size()) {
                designs_.erase(designs_.begin() + i);
                saveDesigns(designs_);
                syncDesignButtons();
            }
        };
    }
}

bool DesignTools::handleEvent(const sf::Event& ev, sf::Vector2f m, float W, float H) {
    bool consumed = false;
    for (auto& b : tb_) consumed |= b.handle(ev, m);
    if (tool_ < 0) return consumed;
    const sf::FloatRect panel(DesignPanelLeft(), H - BAR_H - PANEL_H, W - DesignPanelLeft(), PANEL_H);
    if (tool_ == 0) {
        consumed |= ldN_.handle(ev, m);
        consumed |= ldApply_.handle(ev, m);
    } else if (tool_ == 2) {
        if (ev.type == sf::Event::MouseWheelScrolled && panel.contains(m)) {
            svScroll_ = std::max(0.f, svScroll_ - ev.mouseWheelScroll.delta * 30);
            return true;
        }
        consumed |= svSave_.handle(ev, m);
        consumed |= svExportUrl_.handle(ev, m);
        for (auto& b : svLoad_) consumed |= b.handle(ev, m);
        for (auto& b : svDel_) consumed |= b.handle(ev, m);
    } else if (tool_ == 3) {
        consumed |= rbExport_.handle(ev, m);
    } else if (tool_ == 4) {
        for (auto& b : msnBtns_) consumed |= b.handle(ev, m);
    } else if (tool_ == 5) {
        consumed |= bContest_.handle(ev, m);
    }
    if (!consumed && ev.type == sf::Event::MouseButtonPressed && panel.contains(m))
        consumed = true;
    return consumed;
}

// 部位クリックの左ドック設計パネル(幅360)が開いている間は、その右側だけに
// ツールバー/ツールパネルを描画する(重なり防止)。閉時は全幅(機体全画面表示)。
float DesignTools::DesignPanelLeft() const { return dockOpen_ ? 360.f : 0.f; }

void DesignTools::draw(sf::RenderTarget& rt, HUD& hud, float W, float H) {
    const sf::Font& font = hud.font();
    const float left = DesignPanelLeft();
    // ツールバー
    drawPanelRect(rt, {left, H - BAR_H, W - left, BAR_H}, PANEL_SOFT, LINE, 1);
    float bx = left + 10;
    for (int i = 0; i < 6; i++) {
        tb_[i].rect = {bx, H - BAR_H + 4, 102, BAR_H - 8};
        tb_[i].style = tool_ == i ? 2 : 0;
        tb_[i].draw(rt, font);
        bx += 108;
    }
    drawText(rt, font, u8"optimize.exe --power 270 で自動最適化 → 保存・比較 に追加されます",
             W - 12, H - BAR_H + 10, 10, INKSOFT, 2);
    if (tool_ < 0) return;
    const sf::FloatRect p(left, H - BAR_H - PANEL_H, W - left, PANEL_H);
    drawPanelRect(rt, p, PANEL, LINE, 1);
    const Analysis* an = getAn_();
    if (!an) return;
    char b[160];

    if (tool_ == 0) {
        // ---- 荷重・たわみ ----
        LoadsResult r = computeLoads(*st_, *an, loadN_);
        const float colW = 250;
        ldN_.rect = {p.left + 12, p.top + 10, colW - 24, 34};
        ldN_.draw(rt, font);
        ldApply_.label = std::string(u8"たわみを3D表示: ") + (loadApply_ ? "ON" : "OFF");
        ldApply_.style = loadApply_ ? 2 : 0;
        ldApply_.rect = {p.left + 12, p.top + 52, colW - 24, 26};
        ldApply_.draw(rt, font);
        const double half = st_->span / 2;
        std::snprintf(b, sizeof(b), u8"翼端たわみ %.2f m (半翼幅の%.0f%%)", r.tip, r.tip / half * 100);
        drawText(rt, font, b, p.left + 12, p.top + 90, 11,
                 r.tip / half < 0.12 ? OKC : r.tip / half < 0.2 ? WARNC : BADC);
        std::snprintf(b, sizeof(b), u8"実効上反角 %.1f° (据付 %.1f° + たわみ %+.1f°)", r.dihEff, r.baseDih, r.bendDih);
        drawText(rt, font, b, p.left + 12, p.top + 108, 11, INK);
        std::snprintf(b, sizeof(b), u8"根元曲げ %.0f N·m / 許容 %.0f N·m", r.Mroot, r.Mallow);
        drawText(rt, font, b, p.left + 12, p.top + 126, 11, INK);
        std::snprintf(b, sizeof(b), u8"桁安全率 %.2f @%.1fG (%s・半翼%.0f%%)", r.SF, loadN_,
                      r.failMode == "shear" ? u8"せん断" : u8"曲げ", r.failStation * 100.0);
        drawText(rt, font, b, p.left + 12, p.top + 144, 12,
                 r.SF >= 1.5 ? OKC : r.SF >= 1 ? WARNC : BADC, 0, true);
        // チャート
        const float chW = (p.width - colW - 36) / 2;
        HUD::ChartSeries s1{{}, ACCENT, u8"Schrenk分布", false};
        HUD::ChartSeries s2{{}, BLUE, u8"楕円分布", true};
        HUD::ChartSeries s3{{}, INK, u8"曲げモーメント", false};
        for (size_t i = 0; i < r.ys.size(); i++) {
            s1.pts.push_back({r.ys[i], r.Lp[i]});
            s2.pts.push_back({r.ys[i], r.LpE[i]});
            s3.pts.push_back({r.ys[i], r.M[i]});
        }
        hud.drawChart(rt, {p.left + colW, p.top + 8, chW, PANEL_H - 16}, {s1, s2},
                      u8"スパン位置 m", u8"揚力 N/m", true);
        hud.drawChart(rt, {p.left + colW + chW + 12, p.top + 8, chW, PANEL_H - 16}, {s3},
                      u8"スパン位置 m", u8"M N·m", true);
    } else if (tool_ == 1) {
        // ---- ポーラー ----
        PolarResult r = computePolar(*st_, *an, *prm_);
        const float chW = (p.width - 300) / 2;
        HUD::ChartSeries sp{{}, INK, u8"所要パワー", false};
        for (auto& q : r.pP) sp.pts.push_back(q);
        HUD::ChartSeries pl{{{r.pP.front().first, st_->powerMax}, {r.pP.back().first, st_->powerMax}},
                            OKC, u8"持続出力", true};
        hud.drawChart(rt, {p.left + 10, p.top + 8, chW, PANEL_H - 16}, {sp, pl},
                      u8"対気速度 m/s", "P W", false,
                      {{r.mpX, r.mpY, u8"最小P", BLUE}, {an->V, an->Preq, u8"巡航", ACCENT}});
        HUD::ChartSeries ld{{}, ACCENT, u8"揚抗比 L/D", false};
        for (auto& q : r.pLD) ld.pts.push_back(q);
        hud.drawChart(rt, {p.left + chW + 20, p.top + 8, chW, PANEL_H - 16}, {ld},
                      u8"対気速度 m/s", "L/D", true,
                      {{r.bgX, r.bgY, u8"最良滑空", BLUE}});
        // 抗力内訳
        const float tx = p.left + chW * 2 + 34;
        const double tot = r.Dp + r.Di;
        drawText(rt, font, u8"抗力内訳(巡航)", tx, p.top + 12, 12, INK, 0, true);
        std::snprintf(b, sizeof(b), u8"誘導抗力 %.1f N (%.0f%%)", r.Di, r.Di / tot * 100);
        drawText(rt, font, b, tx, p.top + 36, 11, ACCENT);
        std::snprintf(b, sizeof(b), u8"形状抗力 %.1f N (%.0f%%)", r.Dp, r.Dp / tot * 100);
        drawText(rt, font, b, tx, p.top + 54, 11, BLUE);
        drawPanelRect(rt, {tx, p.top + 76, 220, 12}, BLUE, LINE, 1);
        drawPanelRect(rt, {tx, p.top + 76, 220 * (float)(r.Di / tot), 12}, ACCENT, sf::Color::Transparent, 0);
        std::snprintf(b, sizeof(b), u8"最大L/D %.1f @%.1fm/s", r.bgY, r.bgX);
        drawText(rt, font, b, tx, p.top + 100, 11, INK);
        std::snprintf(b, sizeof(b), u8"最小パワー速度 %.1f m/s", r.mpX);
        drawText(rt, font, b, tx, p.top + 118, 11, INK);
        std::snprintf(b, sizeof(b), u8"失速速度 %.2f m/s", r.Vs);
        drawText(rt, font, b, tx, p.top + 136, 11, INK);
        drawText(rt, font, u8"最小パワー速度=滞空、最良滑空速度=距離", tx, p.top + 162, 10, INKSOFT);
    } else if (tool_ == 2) {
        // ---- 保存・比較 ----
        if (!designsLoaded_) reloadDesigns();
        svSave_.rect = {p.left + 12, p.top + 8, 160, 28};
        svSave_.draw(rt, font);
        svExportUrl_.rect = {p.left + 180, p.top + 8, 150, 28};
        svExportUrl_.draw(rt, font);
        if (!svMsg_.empty())
            drawText(rt, font, svMsg_, p.left + 340, p.top + 15, 10, OKC);
        else
            drawText(rt, font, u8"optimize.exe の結果もここに追加されます", p.left + 340, p.top + 15, 10, INKSOFT);
        // ヘッダ行
        const float ty = p.top + 44;
        const float cols[8] = {12, 190, 250, 310, 370, 430, 490, 560};
        const char* heads[7] = {u8"名称", u8"翼幅m", u8"全備kg", "SM%", u8"所要W", "L/D", u8"飛距離m"};
        for (int i = 0; i < 7; i++)
            drawText(rt, font, heads[i], p.left + cols[i], ty, 10, INKSOFT, 0, true);
        // 現在の設計
        float ry = ty + 18 - svScroll_;
        auto row = [&](const DesignEntry& e, bool cur, int idx) {
            if (ry < ty + 12 || ry > p.top + PANEL_H - 22) { ry += 24; return; }
            if (cur) drawPanelRect(rt, {p.left + 8, ry - 2, p.width - 16, 22}, BLUE_DIM, sf::Color::Transparent, 0);
            drawText(rt, font, e.name, p.left + cols[0], ry, 11, INK);
            char v[32];
            std::snprintf(v, sizeof(v), "%.1f", e.b); drawText(rt, font, v, p.left + cols[1], ry, 11, INK);
            std::snprintf(v, sizeof(v), "%.1f", e.W); drawText(rt, font, v, p.left + cols[2], ry, 11, INK);
            std::snprintf(v, sizeof(v), "%.1f", e.SM); drawText(rt, font, v, p.left + cols[3], ry, 11, INK);
            std::snprintf(v, sizeof(v), "%.0f", e.Preq); drawText(rt, font, v, p.left + cols[4], ry, 11, INK);
            std::snprintf(v, sizeof(v), "%.1f", e.LD); drawText(rt, font, v, p.left + cols[5], ry, 11, INK);
            if (e.dist >= 0) { std::snprintf(v, sizeof(v), "%.0f", e.dist); drawText(rt, font, v, p.left + cols[6], ry, 11, INK); }
            else drawText(rt, font, "-", p.left + cols[6], ry, 11, INKSOFT);
            if (idx >= 0) {
                svLoad_[idx].rect = {p.left + cols[7], ry - 3, 48, 20};
                svDel_[idx].rect = {p.left + cols[7] + 54, ry - 3, 48, 20};
                svLoad_[idx].draw(rt, font);
                svDel_[idx].draw(rt, font);
            }
            ry += 24;
        };
        DesignEntry cur;
        cur.name = u8"(現在の設計)";
        cur.st = *st_;
        cur.dist = cb_.lastDist ? cb_.lastDist() : -1;
        fillMetrics(cur, *prm_);
        row(cur, true, -1);
        for (size_t i = 0; i < designs_.size(); i++) row(designs_[i], false, (int)i);
    } else if (tool_ == 3) {
        // ---- リブ型紙 ----
        auto gs = ribGroups(*st_);
        int tot = 0;
        for (auto& g : gs) tot += g.count;
        rbExport_.rect = {p.left + 12, p.top + 8, 190, 28};
        rbExport_.draw(rt, font);
        std::snprintf(b, sizeof(b), u8"型紙 %d 種類 / リブ合計 %d 枚", (int)gs.size(), tot);
        drawText(rt, font, b, p.left + 216, p.top + 15, 12, INK, 0, true);
        if (!rbMsg_.empty())
            drawText(rt, font, rbMsg_, p.left + 12, p.top + 44, 11, OKC, 0, true);
        float ry = p.top + 68;
        for (size_t i = 0; i < gs.size() && ry < p.top + PANEL_H - 34; i++) {
            std::snprintf(b, sizeof(b), u8"R%d: y=%.2fm コード%dmm 桁孔φ%dmm ×%d枚",
                          gs[i].no, gs[i].y, gs[i].c, gs[i].d, gs[i].count);
            drawText(rt, font, b, p.left + 12 + (i % 2) * 330, ry, 10, INKSOFT);
            if (i % 2) ry += 16;
        }
        drawText(rt, font, u8"SVGはmm実寸。100%(拡大縮小なし)で印刷。桁孔=外径+2mm、青破線=プランク後端",
                 p.left + 12, p.top + PANEL_H - 22, 10, INKSOFT);
    } else if (tool_ == 4) {
        // ---- ミッション ----
        drawText(rt, font, u8"チャレンジミッション — 挑戦すると天候・条件が設定され発進します(終了後は元に戻ります)",
                 p.left + 12, p.top + 8, 11, INKSOFT);
        const auto& ms = missions();
        const float rowH = 32;
        for (size_t i = 0; i < ms.size(); i++) {
            const float ry = p.top + 30 + i * rowH;
            if (ry > p.top + PANEL_H - 24) break;
            const bool clr = cb_.missionCleared && cb_.missionCleared(ms[i].id);
            if (clr) drawPanelRect(rt, {p.left + 8, ry - 2, p.width - 16, rowH - 4}, sf::Color(20, 48, 34, 200), sf::Color::Transparent, 0);
            drawText(rt, font, ms[i].name, p.left + 14, ry, 12, INK, 0, true);
            drawText(rt, font, ms[i].desc, p.left + 170, ry + 2, 10, INKSOFT);
            drawText(rt, font, ms[i].goalText, p.left + 560, ry + 2, 10, BLUE);
            drawText(rt, font, clr ? u8"クリア済" : u8"未クリア", p.left + 730, ry + 2, 11,
                     clr ? OKC : INKSOFT, 0, clr);
            msnBtns_[i].rect = {p.left + 810, ry - 2, 60, 24};
            msnBtns_[i].draw(rt, font);
        }
    } else if (tool_ == 5 && cb_.career) {
        // ---- キャリア ----
        Career& cr = *cb_.career;
        const double cost = Career::totalCost(*st_, *an);
        const double bud = cr.budget();
        const auto locks = cr.lockViolations(*st_);
        std::snprintf(b, sizeof(b), u8"第%d年  資金 %.0f万円  評判 ", cr.st.year, cr.st.money);
        std::string head = b;
        for (int i = 0; i < std::min(5, cr.st.rep); i++) head += u8"★";
        if (cr.st.rep == 0) head += u8"—";
        drawText(rt, font, head, p.left + 12, p.top + 8, 14, INK, 0, true);
        std::snprintf(b, sizeof(b), u8"製作予算 %.0f万円 / 今の機体のコスト %.0f万円", bud, cost);
        drawText(rt, font, b, p.left + 12, p.top + 32, 12, cost <= bud ? OKC : BADC, 0, true);
        // コスト内訳(2列)
        const auto items = Career::costBreakdown(*st_, *an);
        for (size_t i = 0; i < items.size(); i++) {
            std::snprintf(b, sizeof(b), "%s %.0f", items[i].name.c_str(), items[i].cost);
            drawText(rt, font, b, p.left + 12 + (i % 2) * 160, p.top + 56 + (float)(i / 2) * 16, 10, INKSOFT);
        }
        // ロック違反
        float ly = p.top + 56 + (float)((items.size() + 1) / 2) * 16 + 6;
        for (const auto& lv : locks) {
            drawText(rt, font, u8"✕ " + lv, p.left + 12, ly, 11, BADC);
            ly += 16;
        }
        if (an->SM < 2.0) {
            drawText(rt, font, u8"✕ 大会出場には静的安定余裕 SM 2%以上が必要", p.left + 12, ly, 11, BADC);
            ly += 16;
        }
        // 出場ボタン
        std::snprintf(b, sizeof(b), u8"第%d回大会に出場する(天候は当日ガチャ・手動一発勝負)", cr.st.year);
        bContest_.label = b;
        bContest_.enabled = cost <= bud && locks.empty() && an->SM >= 2.0;
        bContest_.rect = {p.left + 12, p.top + PANEL_H - 44, 380, 32};
        bContest_.draw(rt, font);
        // 歴代成績
        const float hx = p.left + 470;
        drawText(rt, font, u8"歴代成績", hx, p.top + 32, 12, INK, 0, true);
        drawText(rt, font, u8"年    天候         記録m    賞金", hx, p.top + 52, 10, INKSOFT);
        for (size_t i = 0; i < cr.st.hist.size() && i < 9; i++) {
            const auto& r = cr.st.hist[cr.st.hist.size() - 1 - i];
            std::snprintf(b, sizeof(b), u8"%2d   %-10s  %7.0f   %.0f万円", r.year, r.wx.c_str(), r.dist, r.prize);
            drawText(rt, font, b, hx, p.top + 70 + i * 16, 11, INK);
        }
        if (cr.st.hist.empty())
            drawText(rt, font, u8"まだ出場記録がありません", hx, p.top + 70, 11, INKSOFT);
        drawText(rt, font, u8"賞金: 10km=300 / 5km=150 / 1km=60万円  評判で高級パーツ解禁・予算増",
                 p.left + 12, p.top + PANEL_H - 64, 10, INKSOFT);
    }
}

// JS版(Birdman3Dβ.html)の #d=base64(JSON) 共有ハッシュを書き出す
void DesignTools::exportJsUrl() {
    // JS側のフィールド名(mode/config等)に合わせてJSONを組む
    const AircraftParams& s = *st_;
    char buf[2048];
    std::snprintf(buf, sizeof(buf),
        "{\"st\":{\"planform\":\"%s\",\"span\":%g,\"rootChord\":%g,\"tipChord\":%g,\"wingX\":%g,"
        "\"dihedral\":%g,\"jig\":\"%s\",\"airfoil\":\"%s\",\"washout\":%g,\"incidence\":%g,\"sweep\":%g,"
        "\"ribPitch\":%g,\"plankTop\":%g,\"plankBot\":%g,\"plankRearTop\":%g,\"plankRearBot\":%g,"
        "\"mode\":\"%s\",\"ailSpanFrac\":%g,\"ailChordFrac\":%g,"
        "\"config\":\"%s\",\"propDia\":%g,\"propBlades\":%d,\"propMat\":\"%s\","
        "\"posture\":\"%s\",\"seatX\":%g,\"pilotW\":%g,\"cd0Add\":%g,\"powerMax\":%g,"
        "\"fairing\":%s,\"gear\":\"%s\",\"segments\":%d,\"rootDia\":%g,\"tipDia\":%g,"
        "\"sparMat\":\"%s\",\"sparMod\":%g,\"boomDia\":%g,"
        "\"drive\":\"%s\",\"driveEffPct\":%g,"
        "\"hShape\":\"%s\",\"hSpan\":%g,\"hChord\":%g,\"tailArm\":%g,"
        "\"vShape\":\"%s\",\"vHeight\":%g,\"vChord\":%g,\"elevRatio\":%g,\"rudRatio\":%g}}",
        s.planform.c_str(), s.span, s.rootChord, s.tipChord, s.wingX,
        s.dihedral, s.jig.c_str(), s.airfoil.c_str(), s.washout, s.incidence, s.sweep,
        s.ribPitch, s.plankTop, s.plankBot, s.plankRearTop, s.plankRearBot,
        s.ailMode.c_str(), s.ailSpanFrac, s.ailChordFrac,
        s.propConfig.c_str(), s.propDia, s.propBlades, s.propMat.c_str(),
        s.posture.c_str(), s.seatX, s.pilotW, s.cd0Add, s.powerMax,
        s.fairing ? "true" : "false", s.gear.c_str(), s.segments, s.rootDia, s.tipDia,
        s.sparMat.c_str(), legacyModForMaterial(s.sparMat), s.boomDia,
        s.drive.c_str(), s.driveEffPct,
        s.hShape.c_str(), s.hSpan, s.hChord, s.tailArm,
        s.vShape.c_str(), s.vHeight, s.vChord, s.elevRatio, s.rudRatio);
    // base64
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string b64;
    const unsigned char* d = (const unsigned char*)buf;
    const size_t n = std::strlen(buf);
    for (size_t i = 0; i < n; i += 3) {
        unsigned v = d[i] << 16 | (i + 1 < n ? d[i + 1] << 8 : 0) | (i + 2 < n ? d[i + 2] : 0);
        b64 += T[(v >> 18) & 63];
        b64 += T[(v >> 12) & 63];
        b64 += i + 1 < n ? T[(v >> 6) & 63] : '=';
        b64 += i + 2 < n ? T[v & 63] : '=';
    }
    std::ofstream f(savePath("design_url.txt"));
    if (!f) { svMsg_ = u8"書き出し失敗"; return; }
    f << u8"# この行を Birdman3Dβ.html のURL末尾に付けて開くと設計が読み込まれます\n";
    f << "#d=" << b64 << "\n";
    svMsg_ = savePath("design_url.txt") + u8" に書き出しました";
}

void DesignTools::exportRibsSVG() {
    std::vector<std::pair<double, double>> up, lo;
    airfoilPoly(up, lo);
    auto gs = ribGroups(*st_);
    const AircraftParams& st = *st_;
    // 補間: コード割合x位置の上下面y (JS afY)
    auto afY = [&](double x, double& yu, double& yl) {
        auto f = [&](const std::vector<std::pair<double, double>>& arr) {
            for (size_t i = 1; i < arr.size(); i++)
                if (arr[i].first >= x) {
                    const double u = (x - arr[i - 1].first) / std::max(1e-9, arr[i].first - arr[i - 1].first);
                    return lerp(arr[i - 1].second, arr[i].second, u);
                }
            return arr.back().second;
        };
        yu = f(up); yl = f(lo);
    };
    std::string svg;
    char b[512];
    double yOff = 16, maxW = 0;
    double top = 0, bot = 0;
    for (auto& p : up) top = std::max(top, p.second);
    for (auto& p : lo) bot = std::max(bot, -p.second);
    for (const auto& g : gs) {
        const double c = g.c;
        maxW = std::max(maxW, c + 20);
        yOff += top * c + 8;
        // 翼型外形パス
        std::string path;
        auto pt = [&](double x, double y, bool first) {
            std::snprintf(b, sizeof(b), "%s%.1f %.1f", first ? "M" : "L", 10 + x * c, yOff - y * c);
            path += b;
        };
        bool first = true;
        for (auto it = up.rbegin(); it != up.rend(); ++it) { pt(it->first, it->second, first); first = false; }
        for (auto& q : lo) pt(q.first, q.second, false);
        path += "Z";
        std::snprintf(b, sizeof(b), "<path d=\"%s\" fill=\"none\" stroke=\"#1B2A4A\" stroke-width=\"0.4\"/>", path.c_str());
        svg += b;
        // 桁孔(30%コード)
        double yu, yl;
        afY(0.3, yu, yl);
        std::snprintf(b, sizeof(b), "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"%.1f\" fill=\"none\" stroke=\"#E8590C\" stroke-width=\"0.4\"/>",
                      10 + 0.3 * c, yOff - (yu + yl) / 2 * c, g.d / 2.0);
        svg += b;
        // プランク端マーク
        auto plankMark = [&](double frac, bool isUp, const char* col) {
            if (frac <= 0 || frac >= 1) return;
            double u2, l2;
            afY(frac, u2, l2);
            const double mid = (u2 + l2) / 2;
            const double y1 = yOff - (isUp ? u2 : mid) * c, y2 = yOff - (isUp ? mid : l2) * c;
            std::snprintf(b, sizeof(b), "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"0.3\" stroke-dasharray=\"2 2\"/>",
                          10 + frac * c, y1, 10 + frac * c, y2, col);
            svg += b;
        };
        plankMark(st.plankTop / 100, true, "#2C5F9E");
        plankMark(st.plankBot / 100, false, "#2C5F9E");
        if (st.plankRearTop > 0) plankMark(1 - st.plankRearTop / 100, true, "#1E7F4F");
        if (st.plankRearBot > 0) plankMark(1 - st.plankRearBot / 100, false, "#1E7F4F");
        std::snprintf(b, sizeof(b),
                      u8"<text x=\"10\" y=\"%.1f\" font-size=\"6\" fill=\"#1B2A4A\" font-family=\"monospace\">R%d: y=%.2fm  c=%dmm  桁孔φ%dmm  ×%d枚</text>",
                      yOff + bot * c + 9, g.no, g.y, g.c, g.d, g.count);
        svg += b;
        yOff += bot * c + 24;
    }
    const double Wm = maxW + 10, Hm = yOff + 8;
    std::ofstream f(savePath("birdman_ribs.svg"));
    if (!f) { rbMsg_ = u8"書き出しに失敗しました"; return; }
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    char hdr[300];
    std::snprintf(hdr, sizeof(hdr),
                  "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0fmm\" height=\"%.0fmm\" viewBox=\"0 0 %.0f %.0f\">",
                  Wm, Hm, Wm, Hm);
    f << hdr;
    std::snprintf(hdr, sizeof(hdr),
                  u8"<text x=\"10\" y=\"10\" font-size=\"5\" fill=\"#46587A\" font-family=\"monospace\">リブ型紙(100%%実寸で印刷) 翼幅%.0fm 上面%.0f%%/下面%.0f%% 桁孔は外径+2mm</text>",
                  st.span, st.plankTop, st.plankBot);
    f << hdr << svg << "</svg>\n";
    rbMsg_ = savePath("birdman_ribs.svg") + u8" に書き出しました";
}

} // namespace bm
