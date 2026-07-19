#include "core/Physics.hpp"
#include "core/Aircraft.hpp"
#include "core/Material.hpp"
#include "core/Weather.hpp"
#include "core/SiteConst.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <cstdio>

namespace bm {

static const double PI = 3.14159265358979323846;

// 発進プラットフォームのデッキ(Renderer3Dの発進台描画と一致させること)
// 後端(x=-DECK_LEN)高10.6m → 前縁(x=0)高10.0m、下り傾斜3.5°の助走路
static const double DECK_LEN = 10.0;
static const double DECK_SLOPE = 3.5 * PI / 180;
static const double DECK_EDGE_H = 10.0;
double deckHeightAt(double x) {   // デッキ上面高(x<=0)
    return DECK_EDGE_H - std::min(0.0, x) * std::tan(DECK_SLOPE);
}

double frand() {
    // thread_local: 最適化CLIが物理を並列実行するため
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng);
}

double groundEffect(double h, double span) {
    const double hw = std::max(0.3, h + 2.0);
    const double r = 16.0 * hw / span;
    return std::max(0.2, r * r / (1.0 + r * r));
}

AircraftConstants aeroPack(const AircraftParams& st, const Analysis& a, const SimParams& prm) {
    AeroConsts ac = aeroConsts(st);
    const double rho = airDensity(prm);
    const double g = 9.81, W = a.W * g;
    // 失速CL: スライダー上限と翼(LLT)の3D失速CLの小さい方。
    // さらに失速速度でのRe低下によるclmax悪化を反復補正(小翼弦の「Reの壁」)
    const AirfoilData& af = airfoilOf(st);
    const double ReD0 = rho * a.V * a.MAC / 1.81e-5;
    double CLmax = std::min(prm.CLmax, ac.CLmaxWing);
    double Vs = std::sqrt(2 * W / (rho * a.S * CLmax));
    for (int it = 0; it < 2; it++) {
        const double ReS = rho * Vs * a.MAC / 1.81e-5;
        const double f = clamp(afClmaxAtRe(af, ReS) / afClmaxAtRe(af, ReD0), 0.8, 1.1);
        Vs = std::sqrt(2 * W / (rho * a.S * CLmax * f));
        if (it == 1) CLmax *= f;
    }
    const double Vt = std::max(a.V, 1.15 * Vs);
    const double A = PI * std::pow(st.propDia / 2, 2);
    const double Vdesign = a.V;
    const double chordRef = a.MAC;
    const double driveEff = clamp(st.driveEffPct / 100.0, 0.5, 1.0);
    // 可変ピッチ: ピッチオフセット5点のBEMTテーブルを先に構築
    // (computeBEMTのキャッシュは1entryのため、基準テーブルptは最後に取る)
    std::vector<double> vpOfs;
    std::vector<std::vector<double>> vpCt, vpCp;
    if (st.varPitch) {
        for (double dp : {-1.2, -0.6, 0.0, 0.6, 1.2}) {
            AircraftParams ps = st;
            ps.propPitch = clamp(st.propPitch + dp, 1.0, 12.0);
            const PropTables& pv = computeBEMT(ps);
            vpOfs.push_back(dp);
            vpCt.push_back(pv.Ct);
            vpCp.push_back(pv.Cp);
        }
    }
    // BEMTプロペラテーブルと取付損失
    const PropTables& pt = computeBEMT(st);
    const double propInst = propInstallationEfficiency(st);
    // 最小パワー速度(オートの目標速度)。必要パワーはBEMTで解く
    double Vmp = Vs * 1.1, Pmin = 1e9;
    for (double v = Vs * 1.02; v <= 14; v += 0.1) {
        const double CL = 2 * W / (rho * v * v * a.S);
        if (CL > CLmax) continue;
        const double CD = ac.CD0 + CL * CL / (PI * a.AR * ac.e);
        const double drag = 0.5 * rho * v * v * a.S * CD;
        const double P = propPowerFor(pt.J, pt.Ct, pt.Cp, st.propDia, rho, v, drag, propInst) / driveEff;
        if (P < Pmin) { Pmin = P; Vmp = v; }
    }
    const double nFail = a.sparSF;
    // VNE: 桁のねじり剛性GJから求めるねじり発散/フラッター限界。
    // 一様翼の発散動圧 q_D = GJ·(π/2s)²/(c²·e_ac·a0)、軽量翼のフラッターは
    // その手前で較正した実効限界。太い桁・高弾性率ほどVNEが上がり、
    // 細桁の軽量化はフラッター限界の低下という代償を払う
    double VNE;
    {
        const double d35 = lerp(st.rootDia, st.tipDia, 0.35) / 1000;   // 35%スパンの桁径
        const double tw = d35 / 80;                                    // 肉厚(computeLoadsと同仮定)
        const double Gspar = materialOf(st.sparMat).young * CFRP_G_OVER_E;
        const double GJ = Gspar * 2 * PI * std::pow(d35 / 2, 3) * tw;   // 薄肉円管
        const double halfSpan = st.span / 2;
        const double qDiv = GJ * std::pow(PI / (2 * halfSpan), 2)
                          / (a.MAC * a.MAC * 0.15 * 5.7);              // e_ac=0.15c, a0=5.7/rad
        const double Vdiv = std::sqrt(2 * std::max(1.0, qDiv) / rho);
        // G=E/24の積層有効値に対する較正係数。標準機の16m/sを維持する。
        VNE = clamp(1.38 * Vdiv, 1.25 * a.V, 34.0);
    }
    const double sg = prm.sens;
    const double ailBase = (st.ailMode == "allmove" ? 9 : st.ailMode == "large" ? 7
                         : st.ailMode == "small" ? 4 : 0) * PI / 180;
    const double ailRefSpan = st.ailMode == "allmove" ? 0.20 : st.ailMode == "large" ? 0.25 : 0.15;
    const double ailRefChord = st.ailMode == "allmove" ? 1.00 : st.ailMode == "large" ? 0.30 : 0.25;
    const double ailFac = st.ailMode == "none" ? 0.0
        : clamp((st.ailSpanFrac / ailRefSpan) * (st.ailChordFrac / ailRefChord), 0.4, 1.6);
    const double elevFac = clamp((a.Vh / 0.46) * std::sqrt(std::max(0.0, st.elevRatio)), 0.5, 1.5);
    const double rudFac = clamp((a.Vv / 0.0052) * std::sqrt(std::max(0.0, st.rudRatio)), 0.5, 1.5);
    const double ailRate = ailBase * ailFac * sg;
    const double rudYaw = 2.5 * PI / 180 * rudFac * sg;
    const double rudRoll = 1.2 * PI / 180 * rudFac * sg;
    const double elevAuth = 0.65 * elevFac * sg;
    const bool hasGear = st.gear != "none";
    const double gearCD = gearDragCoefficient(st);

    AircraftConstants c;
    c.a = a;
    c.CD0 = ac.CD0 + gearCD; c.e = ac.e; c.propEff = ac.propEff; c.driveEff = driveEff;
    c.rho = rho; c.g = g; c.W = W; c.m = a.W; c.S = a.S;
    c.CLmax = CLmax; c.Vs = Vs; c.Vt = Vt; c.A = A; c.AR = a.AR;
    // 地上滑走時のCL: 主翼取付角(=地上姿勢での迎角)から揚力線傾斜で算出。
    // α0L≈-5°(キャンバー付きHPA翼型の代表値)。取付角を上げるほど低速で自然に浮き、
    // 下げると滑走が伸びる(従来は0.8固定で取付角が離陸に反映されなかった)
    {
        const double CLa3D = 2 * PI * a.AR / (a.AR + 2);
        c.CLg0 = clamp(CLa3D * (st.incidence + 5.0) * PI / 180.0, 0.2, CLmax * 0.9);
    }
    c.nFail = nFail; c.VNE = VNE;
    c.nFailNeg = -std::max(0.5, 0.6 * nFail);
    c.failStation = a.failStation; c.failMode = a.failMode;
    c.ailRate = ailRate; c.rudYaw = rudYaw; c.rudRoll = rudRoll; c.elevAuth = elevAuth;
    c.hasGear = hasGear; c.CP = st.powerMax;
    // 暑熱derating: 24℃超で0.6%/℃(最大12%)持続出力が落ちる。
    // 「朝凪のうちに飛ぶ」実際の鳥人間戦略がそのままメカニクスになる
    c.heatFac = 1 - clamp((prm.temp - 24) * 0.006, 0.0, 0.12);
    c.Vdesign = Vdesign; c.chordRef = chordRef; c.Vmp = Vmp; c.Pmin = Pmin;
    c.span = st.span; c.propDia = st.propDia;
    // 地上ステアのホイールベース(第6弾): 前輪〜尾輪間を胴体長の1/4で概算(下限1.2m)
    c.wheelBase = std::max(1.2, a.fusLen * 0.25);
    // BEMTテーブルを飛行時参照用に格納。propEffは設計点の実効効率(表示用)
    c.pJ = pt.J; c.pCt = pt.Ct; c.pCp = pt.Cp; c.propInst = propInst;
    c.varPitch = st.varPitch; c.propPitch0 = st.propPitch;
    c.pPitOfs = vpOfs; c.pCtT = vpCt; c.pCpT = vpCp;
    // フラップ(単純フラップの薄翼理論): Δcl=2π·τ·η·δ, τ=√(cf/c), η=大舵角の粘性効率。
    // 部分スパン比と3D補正(AR/(AR+2))を乗じる。最大舵角40°を全開(flap=1)とする
    c.hasFlap = st.flapSpanFrac > 0.01;
    if (c.hasFlap) {
        const double cf = 0.25, tau = std::sqrt(cf), eta = 0.55;
        const double d40 = 40.0 * PI / 180;
        const double dcl2D = 2 * PI * tau * eta * d40;
        c.flapDCL = dcl2D * st.flapSpanFrac * a.AR / (a.AR + 2);
        c.flapDCLmax = 0.8 * c.flapDCL;                 // CLmaxの伸びはCL増分より小さい
        c.flapDCD = 0.9 * cf * std::sin(d40) * std::sin(d40) * st.flapSpanFrac;
        c.flapCm = -0.25 * c.flapDCL;                   // 機首下げモーメント
    }
    // 翼型のRe別極曲線(飛行中のRe効果はこのテーブル差分で計算)
    for (int k = 0; k < 4; k++) { c.afReT[k] = af.reT[k]; c.afCdT[k] = af.cdT[k]; }
    c.cdProfD = afCd0AtRe(af, ReD0);
    {
        const double CLd = 2 * W / (rho * Vdesign * Vdesign * a.S);
        const double CDd = ac.CD0 + CLd * CLd / (PI * a.AR * ac.e);
        const double dragD = 0.5 * rho * Vdesign * Vdesign * a.S * CDd;
        const double PshD = propPowerFor(pt.J, pt.Ct, pt.Cp, st.propDia, rho, Vdesign, dragD, propInst);
        c.propEff = clamp(dragD * Vdesign / std::max(1.0, PshD), 0.3, 0.95);
    }

    // ---- 6DOFモデル用の安定微係数・慣性を推定 ----
    {
        const double b = st.span, S = a.S, MAC = a.MAC;
        c.CLa = 2 * PI * a.AR / (a.AR + 2);
        c.CLcruise = 2 * W / (rho * Vdesign * Vdesign * S);
        c.astall = std::max(0.02, (CLmax - c.CLcruise) / c.CLa);
        // 慣性: 質量分配(analyzeのitems)からピッチ、主翼スパン分布からロール
        double Iyy = 30;   // 集中質量では出ない分布分の下駄
        for (const auto& it : a.items) Iyy += it.w * (it.x - a.xCG) * (it.x - a.xCG);
        const double wWing = a.items[0].w;
        c.Ixx = std::max(200.0, wWing * b * b / 12 + (a.W - wWing) * 0.3);
        c.Iyy = std::max(50.0, Iyy);
        c.Izz = c.Ixx + c.Iyy;
        // ピッチ: 静安定はSM、減衰は水平尾翼容積、舵は3DOFのelevAuthと等価な効き
        const double ARh = st.hSpan * st.hSpan / std::max(0.1, a.Sh);
        const double CLat = 2 * PI * ARh / (ARh + 2);
        c.Cma = -c.CLa * a.SM / 100;
        c.Cmq = -2 * 0.9 * CLat * a.Vh * (a.lh / MAC);
        // 舵効きは静安定余裕とは独立。SM=0や負でも操舵方向が反転しないよう、
        // 尾翼容積・舵面比・感度を含むelevAuthだけから決める。
        c.Cmde = 0.15 * elevAuth;
        // ロール: 減衰・エルロン(3DOFのailRateと定常ロール率が一致するよう換算)・上反角。
        // 実効上反角 = 治具角 + 曲げたわみ分(荷重n比例)。梁解析(computeLoads)から取得し、
        // 飛行中はnに応じてstepSim6が毎ステップ再評価する(たわむ翼はロール安定が増す)
        c.Clp = -c.CLa / 8;
        c.Clda = ailRate * (-c.Clp) * b / (2 * Vdesign);
        c.dihBase = a.baseDih; c.dihBend = a.bendDih;
        c.Clb = c.CLa * clamp(a.baseDih + a.bendDih, -2.0, 15.0) * PI / 180 / 4;   // 1g値
        c.Cldr = rudRoll * (-c.Clp) * b / (2 * Vdesign);
        // ヨー: 垂直尾翼容積による復元・減衰、ラダー(3DOFのrudYawと定常率一致)
        const double ARv = 1.55 * st.vHeight / std::max(0.1, st.vChord);   // 端板効果込み
        const double CLav = 2 * PI * ARv / (ARv + 2);
        const double lv = std::max(0.5, a.xVT - a.xCG);
        c.Cnb = 0.9 * CLav * a.Vv;
        c.Cnr = -2 * 0.9 * CLav * a.Vv * (lv / b) - 0.02;
        c.Cndr = rudYaw * (-c.Cnr) * b / (2 * Vdesign);
        c.CYs = 0.9 * CLav * a.Sv / S;
        c.tipStall = ac.tipStall;   // LLTの失速開始位置(翼端失速ロール崩れ用)
    }
    return c;
}

AircraftConstants funPlaneConstants(const SimParams& prm) {
    // お遊びモード: 小型プロペラ機(超軽量動力機、Rotax級エンジン35kW)。
    // 典型値を直接構成する。人力機の設計解析・体力モデルは通らない
    AircraftParams ps;                       // BEMTテーブル用のプロペラ諸元のみ使用
    ps.propDia = 1.75; ps.propPitch = 1.3; ps.propBlades = 2; ps.propMat = "carbon";
    const double T0 = prm.temp + 273.15;
    const double rho = 101325 * std::pow(1 - 2.25577e-5 * 86, 5.2559) / (287.05 * T0);
    AircraftConstants c;
    c.rho = rho; c.g = 9.81;
    c.m = 280; c.W = c.m * c.g;              // 機体+パイロット
    c.S = 13.0; c.span = 10.0; c.AR = c.span * c.span / c.S;
    c.CD0 = 0.030; c.e = 0.80;               // 固定脚・支柱付き高翼機
    c.CLmax = 1.7;
    c.Vs = std::sqrt(2 * c.W / (rho * c.S * c.CLmax));   // ≈14 m/s
    c.Vt = 1.25 * c.Vs;                      // ローテーション速度 ≈18 m/s
    c.Vdesign = 32; c.chordRef = c.S / c.span;
    c.Vmp = 1.3 * c.Vs; c.Pmin = 9000;
    c.VNE = 58; c.nFail = 6.0;               // 超過禁止速度・終極荷重(制限4G×1.5)
    c.nFailNeg = -2.4;                       // 負荷重終極(-1.6G制限×1.5。押さえ操作で折れない)
    c.hasFlap = true;                        // 軽飛行機のフラップ(Gキーで0/50/100%)
    c.flapDCL = 0.50; c.flapDCLmax = 0.40; c.flapDCD = 0.030; c.flapCm = -0.12;
    c.A = PI * ps.propDia * ps.propDia / 4; c.propDia = ps.propDia;
    c.driveEff = 1.0; c.propEff = 0.78; c.CP = 700;
    c.heatFac = 1.0;                         // エンジンは暑さでバテない
    c.hasGear = true;
    c.wheelBase = 1.6;                       // 小型プロペラ機の前後輪間隔(概算)
    const double sg = prm.sens;
    c.ailRate = 25 * PI / 180 * sg;          // 軽飛行機はロールが機敏
    c.rudYaw = 6 * PI / 180 * sg; c.rudRoll = 2 * PI / 180 * sg;
    c.elevAuth = 0.75 * sg;                  // 舵が良く効く=着陸フレアがしっかり決まる
    const PropTables& pt = computeBEMT(ps);
    c.pJ = pt.J; c.pCt = pt.Ct; c.pCp = pt.Cp; c.propInst = 0.95;
    // GA翼のRe(>2百万)はRe依存ほぼ無し → 平坦テーブル(dProf=0)
    for (int k = 0; k < 4; k++) { c.afReT[k] = 1e5 * std::pow(10, k); c.afCdT[k] = 0.008; }
    c.cdProfD = 0.008;
    // 6DOF: 軽飛行機の代表的な安定微係数
    c.Ixx = 1300; c.Iyy = 1900; c.Izz = 3000;
    c.CLa = 2 * PI * c.AR / (c.AR + 2);
    c.CLcruise = 2 * c.W / (rho * c.Vdesign * c.Vdesign * c.S);
    c.astall = std::max(0.02, (c.CLmax - c.CLcruise) / c.CLa);
    c.Cma = -0.9; c.Cmq = -16; c.Cmde = 1.6 * c.elevAuth;   // GA機は舵が良く効きすぐ引き起こせる
    c.Clp = -0.5; c.Clda = 0.16 * sg; c.Clb = 0.05; c.Cldr = 0.02;
    c.Cnb = 0.07; c.Cnr = -0.10; c.Cndr = 0.05 * sg; c.CYs = 0.30;
    c.tipStall = 0.25;                       // 矩形翼: 翼根から失速(素直)
    c.dihBase = 1.5; c.dihBend = 0;          // 剛体翼: たわみ無し
    c.a = Analysis{};
    c.a.W = c.m; c.a.S = c.S; c.a.AR = c.AR; c.a.MAC = c.chordRef;
    c.a.V = c.Vdesign; c.a.SM = 14;
    return c;
}

FlightState makeInitialState(const AircraftConstants& c, const SimParams& prm, double z0) {
    const bool runway = prm.mode == "runway";
    FlightState L;
    // 滑走路はプッシャー(人力補助)が付けた初速から。プラットフォームは
    // デッキ後端(x=-10, 高10.6m)からの台車滑走(脚なし機も台車で走れる)
    L.ground = runway ? c.hasGear : true;
    // 自由発進(第5弾): 滑走路モードは機首方位startHdg(度)で発進できる。
    // 初期対気速度は「プッシャー初速(対地) − 機首方向の風成分」で一般化
    // (startHdg=0では従来式 pushV - prm.wind と一致=後方互換)
    L.psi = runway ? prm.startHdg * PI / 180 : 0;
    double wind0 = prm.wind, xwind0 = prm.xwind;
    if (runway) horizontalWindAt(prm, 0, 0, 0, wind0, xwind0);
    const double wAlong0 = wind0 * std::cos(L.psi) + xwind0 * std::sin(L.psi);
    L.groundSpeed = runway ? std::max(0.0, prm.pushV) : -1.0;
    L.V = runway ? std::abs(L.groundSpeed - wAlong0) : std::max(0.0, prm.V0 - prm.wind);
    L.gam = 0;
    L.h = runway ? 0 : deckHeightAt(-DECK_LEN);
    L.x = runway ? 0 : -DECK_LEN;
    L.officialX0 = L.x;
    L.officialYl0 = L.yl;
    L.t = 0;
    L.n = 0;   // 滑走開始時は無荷重(デッキ/滑走路とも)
    L.auto_ = prm.auto_ && !prm.funPlane;   // お遊び機は手動スロットルのみ
    L.wbal = prm.wcap * 1000;
    L.Pnow = prm.auto_ ? c.CP : prm.P;
    L.z0 = z0;
    // 気象ジッターとは別ストリームだが、同じweatherSeedなら同じDryden乱流系列になる。
    // FlightStateがRNGを所有するため、プレイヤー/ライバルの更新順にも依存しない。
    L.turbRng = (prm.weatherSeed ^ 0xA511E9B3u) ? (prm.weatherSeed ^ 0xA511E9B3u)
                                                : 0x13579BDFu;
    if (runway && !c.hasGear) {
        L.done = true; L.overrun = true; L.nogear = true; L.officialInvalid = true;
    }
    return L;
}

void refreshOfficialDistance(FlightState& L) {
    L.officialDist = L.officialInvalid
                   ? 0.0
                   : std::hypot(L.x - L.officialX0, L.yl - L.officialYl0);
}

static void invalidateOfficialDistance(FlightState& L) {
    L.officialInvalid = true;
    L.officialDist = 0;
}

SimSample simSample(const FlightState& L) {
    return {L.t, L.x, L.h, L.V, L.gam, L.n, L.yl, L.phi,
            L.officialDist, L.pathAir, L.psi};
}

void stepTurbulence(FlightState& L, const SimParams& prm, double dt) {
    // Dryden型連続乱流: スケール長L_w≈高度(低空ほど小さく細かい)、相関時間τ=L/V。
    // 低空は短いτで「ガタガタ」、上空は長いτの「ゆったりしたうねり」になり、
    // 速度が上がるほど突入周波数が上がる。強度は従来同様地面付近で増幅
    const double sig = 0.76 * prm.turb * std::max(0.25, std::min(1.0, 3.5 / std::max(L.h, 0.5)));
    const double V = std::max(L.V, 2.0);
    const double tauW = clamp(L.h, 3.0, 60.0) / V;          // 鉛直成分
    const double tauV = clamp(2.0 * L.h, 8.0, 120.0) / V;   // 横成分
    auto uniform = [&] {
        unsigned x = L.turbRng ? L.turbRng : 0x13579BDFu;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        L.turbRng = x;
        return (x & 0x00ffffffu) / (double)0x01000000u;
    };
    auto gauss = [&] { return (uniform() + uniform() + uniform() + uniform() - 2.0)
                             * 1.7320508075688772; };
    const double aW = std::min(1.0, dt / tauW), aV = std::min(1.0, dt / tauV);
    L.tz += -L.tz * aW + sig * std::sqrt(2 * aW) * gauss();
    L.ty += -L.ty * aV + 0.78 * sig * std::sqrt(2 * aV) * gauss();
}

static void markSparFailure(FlightState& L, const AircraftConstants& c,
                            const std::string& reason) {
    L.sparBroken = true;
    L.failStation = c.failStation;
    L.failMode = c.failMode;
    const char* mode = c.failMode == "shear" ? u8"せん断" : u8"曲げ";
    char loc[96];
    std::snprintf(loc, sizeof(loc), u8" [%s・半翼%.0f%%位置]", mode, c.failStation * 100.0);
    L.failureMsg = reason + loc;
}

// Phase 4で意図的に残す近似:
//  - 標準物理は速度・経路角・バンクを直接積分する経路モデル
//  - 拡張物理も完全な機体軸u/v/w連成ではなく、姿勢角と角速度を追加した回転モデル
//  - プロペラジャイロ、スピン/ディープストール、負側の非対称翼端失速はPhase 5以降
static double effectiveCLmin(const AircraftConstants& c, const FlightState& L) {
    const double base = -0.50 * c.CLmax;
    return std::min(-0.15, base + 0.60 * c.flapDCL * L.flap);
}

static bool updateStructuralFailure(FlightState& L, const AircraftConstants& c, double dt) {
    const double vr = L.V / c.VNE;
    if (vr > 0.88) L.fatigue += (vr - 0.88) * (vr - 0.88) * 25.0 * dt;
    if (L.n > 0.8 * c.nFail)
        L.fatigue += (L.n / c.nFail - 0.8) * 0.8 * dt;
    if (L.n < 0.8 * c.nFailNeg)
        L.fatigue += (L.n / c.nFailNeg - 0.8) * 0.8 * dt;
    if (L.fatigue >= 1.0) {
        markSparFailure(L, c, u8"フラッター/繰返し荷重による主桁疲労破断");
        return true;
    }

    const bool overPos = L.n > c.nFail;
    const bool overNeg = L.n < c.nFailNeg;
    if (overPos || overNeg) {
        L.overNT += dt;
        const double ratio = overPos ? L.n / c.nFail : L.n / c.nFailNeg;
        if (L.overNT >= 0.15 || ratio > 1.15) {
            char buf[160];
            if (overPos)
                std::snprintf(buf, sizeof(buf), u8"主桁折損(n=%.2f > 限界%.2f)", L.n, c.nFail);
            else
                std::snprintf(buf, sizeof(buf), u8"負荷重で主桁折損(n=%.2f < 限界%.2f)", L.n, c.nFailNeg);
            markSparFailure(L, c, buf);
            return true;
        }
    } else {
        L.overNT = 0;
    }
    if (L.V > c.VNE) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), u8"超過速度でフラッター破壊(V=%.1f > VNE %.1f m/s)", L.V, c.VNE);
        markSparFailure(L, c, buf);
        return true;
    }
    return false;
}

double thermalCellVz(const SimParams& prm, double x, double yl) {
    if (prm.thermal <= 0) return 0;
    const double cell = std::sin(x * 0.0045 + 1.3) * std::cos(x * 0.0021 + yl * 0.008);
    const double core = std::max(0.0, cell - 0.45) / 0.55;
    return prm.thermal * (core > 0 ? 0.25 * core : -0.02);
}

void stepThermal(FlightState& L, const SimParams& prm) {
    L.thrmVz = 0;
    if (prm.thermal <= 0) { L.inThermal = false; return; }
    if (prm.realThermal) {
        // 動的セル: 湧いて消える円形上昇帯。セルが小さいため追従を速めに。
        // 夏の琵琶湖は水温<気温で湖上が安定成層のため、上昇流は高度とともに
        // 減衰する(低空〜数十mが主戦場。際限なく100m超まで上がらない)
        double target = thermalFieldVz(prm, L.x - L.z0, L.yl, L.t);
        if (target > 0) target *= 0.30 + 0.70 * std::exp(-L.h / 60.0);
        L.thrm = L.thrm * 0.96 + target * 0.04;
        L.inThermal = target > 0.15;
    } else {
        // JS互換の固定関数サーマル
        const double cell = std::sin(L.x * 0.0045 + 1.3) * std::cos(L.x * 0.0021 + L.yl * 0.008);
        const double core = std::max(0.0, cell - 0.45) / 0.55;
        const double target = prm.thermal * (core > 0 ? 0.25 * core : -0.02); // 修正済み係数0.25
        L.thrm = L.thrm * 0.985 + target * 0.015;
        L.inThermal = core > 0.3;
    }
    L.thrmVz = L.thrm;
}

// 操舵なまし: 入力時≈0.25秒、中立へ戻すときはさらに速く
static void stepSteering(FlightState& L, const SimParams& prm, double dt) {
    double* cur[3] = {&L.e, &L.ail, &L.rud};
    double tgt[3] = {L.auto_ ? 0 : L.eTgt, L.auto_ ? 0 : L.ailTgt, L.auto_ ? 0 : L.rudTgt};
    // 疲労による操舵ゆらぎ(体力モデルON時): 飛行時間の経過とW'枯渇で
    // エレベーター操作に有色ノイズが乗る(手が震えて精密操舵が難しくなる)
    if (prm.stamina && !L.auto_ && !L.ground) {
        const double wfrac = clamp(L.wbal / std::max(1.0, prm.wcap * 1000), 0.0, 1.0);
        const double ft = clamp(0.20 * L.t / 3600 + 0.35 * (1 - wfrac), 0.0, 0.6);
        L.ctlJit = L.ctlJit * 0.95 + (frand() - 0.5) * 0.05 * ft;
        tgt[0] = clamp(tgt[0] + L.ctlJit, -1.0, 1.0);
    } else {
        L.ctlJit = 0;
    }
    for (int i = 0; i < 3; i++) {
        const double d = tgt[i] - *cur[i];
        const bool returning = std::abs(tgt[i]) < 0.05;
        const double rate = (returning ? 8.0 : 4.5) * (0.6 + 0.4 * prm.sens);
        *cur[i] += clamp(d, -rate * dt, rate * dt);
        if (std::abs(*cur[i]) < 0.02 && std::abs(tgt[i]) < 0.05) *cur[i] = 0;
    }
    // フラップは低速サーボ(全開まで約2.5秒)
    L.flap = clamp(L.flap + clamp(L.flapTgt - L.flap, -0.4 * dt, 0.4 * dt), 0.0, 1.0);
}

// 体力モデル(2階層):
//  - グリコーゲン(gly, 持久力): 基礎代謝+踏んだ出力ぶんだけ消費し、飛行中は回復しない。
//    枯渇するほど持続出力CPeが下がる。低出力でも減るので「絞れば無限回復」を防ぐ。
//  - W'(wbal, 無酸素バッテリー): CPe以下で回復、超過で消費。短時間バーストの原資。
static const double GLY_CAP = 620000.0;   // 総持久エネルギー J(≈620kJ, 全力40分/巡航50分相当)

// 現在の持続可能出力CPe(グリコーゲン残・暑熱・W'残で変化)
static double staminaCPe(const FlightState& L, const AircraftConstants& c, const SimParams& prm) {
    const double wfrac = clamp(L.wbal / std::max(1.0, prm.wcap * 1000), 0.0, 1.0);
    // 満タンで1.0、枯渇で0.55まで低下(時間基準の旧式を仕事量基準のglyに置換)
    return c.CP * c.heatFac * (0.55 + 0.45 * L.gly) * (0.88 + 0.12 * wfrac);
}
// W'とグリコーゲンを1ステップ更新。glyは出力に関わらず必ず減る(基礎代謝+実出力)
static void staminaStores(FlightState& L, const AircraftConstants& c, const SimParams& prm,
                          double P, double CPe, double dt) {
    const double wcap = prm.wcap * 1000;
    if (P > CPe) L.wbal = std::max(0.0, L.wbal - (P - CPe) * dt);
    else         L.wbal = std::min(wcap, L.wbal + (CPe - P) * 0.30 * dt);
    L.wbal = clamp(L.wbal, 0.0, wcap);
    const double basal = 0.12 * c.CP;   // 基礎代謝: 何もしなくても消費する分
    L.gly = std::max(0.0, L.gly - (basal + std::max(0.0, P)) * dt / GLY_CAP);
}

// 手動時: W'/グリコーゲンで要求出力を制限しつつ体力を更新
static double staminaLimit(FlightState& L, const AircraftConstants& c, const SimParams& prm, double P, double dt) {
    const double CPe = staminaCPe(L, c, prm);
    const double Pinst = L.wbal > 0 ? CPe + std::min(220.0, L.wbal / 9) : CPe;
    P = std::min(P, Pinst);
    staminaStores(L, c, prm, P, CPe, dt);
    return P;
}

// 出力決定(手動/オート体力モデル)+プロペラ推力。L.Pnow/wbal/etaP/rpmを更新し推力Tを返す
static double computeThrust(FlightState& L, const AircraftConstants& c, const SimParams& prm, double dt) {
    double P;
    if (L.auto_) {
        const double CPe = staminaCPe(L, c, prm);
        const double Pinst = L.wbal > 0 ? CPe + std::min(220.0, L.wbal / 9) : CPe;
        const double Vtgt = c.Vmp;
        if (L.softHTgt < 0) L.softHTgt = L.h;
        const double rate = 0.5 * dt;
        L.softHTgt += clamp(prm.hTgt - L.softHTgt, -rate, rate);
        const double hErr = L.softHTgt - L.h;
        const double Vz0 = L.V * std::sin(L.gam);
        const double climbP = c.W * std::max(0.0, hErr * 0.4 - Vz0) * 0.9;
        double Ptarget = c.Pmin * 1.04 + climbP;
        Ptarget += (Vtgt - L.V) * 22 * c.m / 10;
        P = std::max(30.0, std::min(Pinst, Ptarget));
        if (L.ground) P = Pinst;
        staminaStores(L, c, prm, P, CPe, dt);
        L.Pnow = P;
        L.autoVtgt = Vtgt;
    } else if (prm.funPlane) {
        // お遊び機: 出力スライダー(0..700)をスロットル0..100%として読み替え。
        // エンジンなので出力ゆらぎ・体力モデルは適用しない
        P = clamp(prm.P / 700.0, 0.0, 1.0) * FUN_ENGINE_W;
        L.pj = 0;
        L.Pnow = P;
    } else {
        P = prm.P;
        if (prm.pjit && prm.P > 5) {
            L.pj = L.pj * 0.94 + (frand() - 0.5) * 3;
            P = std::max(0.0, prm.P + clamp(L.pj, -5.0, 5.0));
        } else { L.pj = 0; }
        // 体力モデル(手動): スライダーの要求出力を実際に出せる範囲に制限
        if (prm.stamina) P = staminaLimit(L, c, prm, P, dt);
        L.Pnow = P;
    }
    // 駆動伝達後の軸出力
    const double Pshaft = std::max(0.0, P) * c.driveEff;
    // BEMT: 軸出力と対気速度から回転数と推力を解く。
    // P=0のときはフリーホイール回転(Cp=0)に落ち着き、軽い風車抗力(T<0)が出る。
    // 可変ピッチ機はオフセット別テーブルを線形補間して使う
    double nps = 0, T;
    if (c.varPitch && c.pPitOfs.size() >= 2) {
        const double po = clamp(L.pitchOfs, c.pPitOfs.front(), c.pPitOfs.back());
        size_t k = 0;
        while (k + 2 < c.pPitOfs.size() && po > c.pPitOfs[k + 1]) k++;
        const double u = (po - c.pPitOfs[k]) / (c.pPitOfs[k + 1] - c.pPitOfs[k]);
        std::vector<double> Ct(c.pJ.size()), Cp(c.pJ.size());
        for (size_t i = 0; i < c.pJ.size(); i++) {
            Ct[i] = lerp(c.pCtT[k][i], c.pCtT[k + 1][i], u);
            Cp[i] = lerp(c.pCpT[k][i], c.pCpT[k + 1][i], u);
        }
        T = propThrust(c.pJ, Ct, Cp, c.propDia, c.rho, std::max(0.0, L.V), Pshaft, nps) * c.propInst;
    } else {
        L.pitchOfs = 0;   // 固定ピッチ機
        T = propThrust(c.pJ, c.pCt, c.pCp, c.propDia, c.rho, std::max(0.0, L.V), Pshaft, nps)
          * c.propInst;
    }
    L.etaP = Pshaft > 1 ? clamp(T * std::max(0.0, L.V) / Pshaft, 0.0, 1.0) : 0.0;
    L.nps = nps; L.rpm = nps * 60;
    return T;
}

// ---- 接地品質・クラブ着陸の共通ヘルパー(第6弾。滑走路モードの陸上接地に適用) ----
// 接地の瞬間の沈下率 sink = -V·sin(gam) で接地品質を分岐:
//   <2.0 m/s      : 通常接地(従来通り)
//   2.0〜3.5 m/s  : ハードランディング=バウンド(gam=+0.03へ跳ね返し・V×0.92・接地扱いにしない)
//   3.5〜5.0 m/s  : 着陸装置破損(接地はするが以後 rollMu=0.15 で滑走停止のみ)
//   >5.0 m/s      : クラッシュ(即終了)
// バンク|phi|>12°での接地は翼端接地として判定を1段階悪化させる。
// クラブ着陸(横風で偏流したままの接地)では、空中の対地速度ベクトルと機首方位の差から
// 残留横対地速度 vLat を保存し、地上滑走側で τ≈0.6s の指数減衰(タイヤの横滑りが止める)を
// かけて (x,yl) に加算する。偏流なし(ψ=進行方向)の接地では vLat=0 で従来と一致。
static void applyTouchdown(FlightState& L, const SimParams& prm, double sink, bool sixdof) {
    int band = sink < 2.0 ? 0 : sink < 3.5 ? 1 : sink < 5.0 ? 2 : 3;
    if (std::abs(L.phi) > 12.0 * PI / 180.0) band = std::min(3, band + 1);   // 翼端接地
    if (band >= 3) {                       // クラッシュ
        L.crashed = true; L.h = 0; L.done = true; L.impact = "crash";
        L.failureMsg = u8"墜落(沈下率または翼端接地が限界超過)";
        return;
    }
    // 残留横対地速度(機首右向き+): 接地直前の対地速度ベクトルの機首直交成分
    const double vh = L.V * std::cos(L.gam);
    double wind = 0, xwind = 0;
    horizontalWindAt(prm, L.x - L.z0, L.yl, 0, wind, xwind);
    const double gvx = vh * std::cos(L.psi) + wind;
    const double gvy = vh * std::sin(L.psi) + xwind;
    L.vLat = -gvx * std::sin(L.psi) + gvy * std::cos(L.psi);
    L.groundSpeed = std::max(0.0, gvx * std::cos(L.psi) + gvy * std::sin(L.psi));
    L.touchdowns++; L.tdT = L.t;
    if (band == 1) {                       // ハードランディング: バウンド
        L.gam = 0.03; L.V *= 0.92; L.h = 0.01;
        if (sixdof) L.theta = L.gam;
        L.hardLanding = true;              // UI警告用(Game側でトースト表示後クリア)
        return;
    }
    if (band == 2) {                       // 着陸装置破損
        L.gearBroken = true;
        L.failureMsg = u8"着陸装置破損(沈下率過大)";
    }
    L.ground = true; L.h = 0; L.gam = 0;   // 通常接地(バウンド時以外はgam即0=従来の簡略化を維持)
    if (sixdof) L.theta = 0;
}

void stepSim(FlightState& L, const AircraftConstants& c, const SimParams& prm, double dt) {
    const double q = 0.5 * c.rho * L.V * L.V;
    stepTurbulence(L, prm, dt);
    // 乱流(連続有色ノイズ)と設定由来の短時間鉛直突風を同じ符号規約で合成。
    // 上向き風+は迎角と揚力を増やし、3DOF/6DOFで同じΔCL=CLa·atan(w/V)を使う。
    const double aeroGustVz = L.tz + prm.verticalGust;
    const double gustAlpha = std::atan2(aeroGustVz, std::max(L.V, 3.0));
    const double gustDCL = c.CLa * gustAlpha;
    stepThermal(L, prm);
    stepSteering(L, prm, dt);
    L.eApplied = L.e; L.ailApplied = L.ail; L.rudApplied = L.rud;
    const double T = computeThrust(L, c, prm, dt);

    if (L.ground && prm.mode != "runway") {
        // ---- プラットフォームのデッキ滑走(台車・下り3.5°・助走10m) ----
        // 後端x=-10(高10.6m)から前縁x=0(高10.0m)へ。重力の斜面成分で加速し、
        // 前縁を越えたら経路角-3.5°で空中へ。Sキーのブレーキで発進中止も可能
        const double CLg = clamp(0.8 + gustDCL + 0.3 * std::max(0.0, L.e)
                               + 0.8 * c.flapDCL * L.flap, 0.05, c.CLmax);
        const double lift = q * c.S * CLg;
        const double CDg = c.CD0 + CLg * CLg / (PI * c.AR * c.e)
                         + c.flapDCD * L.flap * L.flap;   // 水面から10m上: 地面効果なし
        const double normal = std::max(0.0, c.W * std::cos(DECK_SLOPE) - lift);
        const double roll = (0.02 + 0.40 * clamp(L.brake, 0.0, 1.0)) * normal;
        const double dV = (T - q * c.S * CDg - roll) / c.m + c.g * std::sin(DECK_SLOPE);
        L.V = std::max(0.0, L.V + dV * dt);
        L.x += std::max(0.0, L.V + prm.wind) * dt;
        L.t += dt;
        L.yl += prm.xwind * 0.3 * dt;      // 台車上は横流れ小
        refreshOfficialDistance(L);
        L.n = lift / c.W;
        if (L.x >= 0) {                    // 前縁から飛び出し
            L.ground = false;
            L.x = std::min(L.x, 0.5);
            L.h = DECK_EDGE_H;
            L.gam = -DECK_SLOPE;           // 斜面に沿った初期経路角
        } else if (L.V >= 1.02 * c.Vt || (L.e > 0.25 && L.V >= c.Vs)) {
            L.ground = false;              // デッキ上で浮揚(十分な速度)
            L.h = deckHeightAt(L.x);
            L.gam = 0;
        } else {
            L.h = deckHeightAt(L.x);
        }
        return;
    }
    if (L.ground) {
        const bool fuji = prm.site == "fujikawa";
        double groundWind = 0, groundXwind = 0;
        horizontalWindAt(prm, L.x - L.z0, L.yl, 0, groundWind, groundXwind);
        const double wAlong = groundWind * std::cos(L.psi) + groundXwind * std::sin(L.psi);
        const double wCrossRaw = -groundWind * std::sin(L.psi) + groundXwind * std::cos(L.psi);
        if (L.groundSpeed < 0) L.groundSpeed = std::max(0.0, L.V + wAlong);
        const double relativeAlong = L.groundSpeed - wAlong;
        L.V = std::abs(relativeAlong);
        const bool forwardFlow = relativeAlong >= 0;
        const double qGround = 0.5 * c.rho * L.V * L.V;
        // 地上滑走のCL: 主翼取付角由来のCLg0(aeroPackで算出)+エレベーター
        // (負=機首下げで揚力を抑えられる)+フラップ。従来の0.8固定を廃止し、
        // 取付角スライダーが離陸滑走距離に物理的に効くようにする
        const double CLg = clamp(c.CLg0 + gustDCL + 0.35 * L.e + 0.8 * c.flapDCL * L.flap,
                                 0.05, (c.CLmax + c.flapDCLmax * L.flap) * 0.95);
        const double lift = forwardFlow ? qGround * c.S * CLg : 0.0;
        const double CDg = c.CD0 + CLg * CLg / (PI * c.AR * c.e) * groundEffect(0, c.span)
                         + c.flapDCD * L.flap * L.flap;
        // 制動: 転がり摩擦+ブレーキ(Sキー, μ≈0.4のタイヤ制動をキャッチャー相当に)。
        // どちらも接地荷重(W-lift)に比例。
        // 転がり摩擦: 滑走路0.02 / 砂利(富士川の滑走路矩形外)0.05 /
        // 着陸装置破損(第6弾)0.15=引きずり抵抗で滑走停止のみ
        bool onRwy = true;
        bool onSandbar = false;
        if (fuji) {
            const double xAbs = L.x - L.z0;
            onRwy = site::insideFujikawaRunway(xAbs, L.yl);
            onSandbar = site::insideFujikawaSandbar(xAbs, L.yl);
        }
        double rollMu = onRwy ? site::FUJI_RWY_MU
                              : (onSandbar ? site::FUJI_SANDBAR_MU : site::FUJI_RIVERBED_MU);
        if (!fuji) rollMu = onRwy ? 0.02 : 0.05;
        if (L.gearBroken) rollMu = site::FUJI_BROKEN_GEAR_MU;
        const double normal = std::max(0.0, c.W - lift);
        const double roll = (rollMu + 0.40 * clamp(L.brake, 0.0, 1.0)) * normal;
        const double aeroDrag = qGround * c.S * CDg;
        const double dragForce = relativeAlong > 0 ? -aeroDrag : relativeAlong < 0 ? aeroDrag : 0.0;
        const double rollingForce = L.groundSpeed > 0.01 ? roll : 0.0;
        const double dGroundV = (T + dragForce - rollingForce) / c.m;
        L.groundSpeed = std::max(0.0, L.groundSpeed + dGroundV * dt);
        L.V = std::abs(L.groundSpeed - wAlong);
        const double gs = L.groundSpeed;
        // ---- 地上ステア(第6弾: 車輪ステア型に再設計) ----
        // ステア角 δ = rud × δmax(V)。δmax=25°×clamp(1-(V-4)/12, 0.35, 1)
        // (V≤4m/sで25°、高速では絞る=高速でのスピン防止)。
        // 回頭率 dψ/dt = (gs/ホイールベース)·tanδ。静止〜極低速(gs<0.3)では回頭しない。
        // 旧実装(2.0·rudYaw·rud·clamp(V/0.5Vt), 最大≈5°/s)は低速でほぼ回頭できなかった
        {
            double psiDot = 0;
            if (gs >= 0.3) {
                const double dmax = 25.0 * PI / 180.0 * clamp(1.0 - (L.V - 4.0) / 12.0, 0.35, 1.0);
                psiDot = gs / c.wheelBase * std::tan(L.rud * dmax);
            }
            // 風見鶏効果(弱): 横風成分に比例して機首がゆっくり風上を向く(上限0.5°/s)。
            // ラダー中立で横風に置くと徐々に風上へ向く程度の弱い項。
            // 第7弾: 対地速度が上がるほどタイヤの直進グリップが勝るため減衰させる
            // (これが無いと追い風発進の長い滑走中に勝手に回頭して滑走路を逸脱してしまう)
            const double wvGrip = clamp(1.0 - gs / 12.0, 0.0, 1.0);
            psiDot += clamp(-wCrossRaw * 0.006, -0.5 * PI / 180.0, 0.5 * PI / 180.0) * wvGrip;
            psiDot = clamp(psiDot, -30.0 * PI / 180.0, 30.0 * PI / 180.0);   // 上限30°/s
            L.psi += psiDot * dt;
        }
        if (L.psi > PI) L.psi -= 2 * PI; else if (L.psi < -PI) L.psi += 2 * PI;
        // ---- 横風ドリフト(第6弾: タイヤ横グリップで大幅減) ----
        // 接地中の横流れはタイヤの横抵抗でほぼ止まる: 滑走路0.12 / 砂利0.30、
        // ブレーキ中はさらに半減。旧実装の(1-0.7|rud|)は横抵抗を過小評価していた
        const double drift = wCrossRaw * (onRwy ? 0.12 : 0.30) * (1.0 - 0.5 * clamp(L.brake, 0.0, 1.0));
        // クラブ着陸の残留横対地速度: τ≈0.6sの指数減衰(タイヤが横滑りを止める)
        L.vLat *= std::exp(-dt / 0.6);
        const double vSide = drift + L.vLat;
        L.x += (gs * std::cos(L.psi) - vSide * std::sin(L.psi)) * dt; L.t += dt;
        L.yl += (gs * std::sin(L.psi) + vSide * std::cos(L.psi)) * dt;
        L.rollDist += gs * dt;             // 累積滑走距離(自由方位のoverrun判定・表示用)
        refreshOfficialDistance(L);
        // 接地時のバンクを凍結させず速やかに水平へ戻す
        L.phi -= L.phi * std::min(1.0, 5 * dt);
        L.n = lift / c.W;
        // 浮揚判定を物理ベースに変更: 揚力が重量に達したら自然に浮き上がる。
        // CLgは取付角+エレベーター依存なので「速度が十分なのに浮かない」ことはなく、
        // 機首下げ(e<0)でCLgを下げれば地面に留まれる(ホールドダウンも物理で表現)。
        // 旧実装の「1.02×Vt到達で浮揚」という台本処理は廃止。
        // フレアを握ったまま接地してもポーポイズしないよう接地後1秒は再離陸しない。
        // 着陸装置破損(broken)中は再離陸不可
        const bool tdGrace = L.t - L.tdT < 1.0;
        // 引き起こし離陸はVs到達で可能(地面効果内はCLmax実効+数%あり引き剥がせる)。
        // フラップ展開時は失速速度が下がるぶん早くローテーションできる
        const double VsG = c.Vs * std::sqrt(c.CLmax / std::max(0.3, c.CLmax + c.flapDCLmax * L.flap));
        if (!L.gearBroken &&
            ((lift >= c.W && (L.touchdowns == 0 || !tdGrace))
             || (!tdGrace && L.e > 0.25 && L.V >= VsG))) {
            L.ground = false;
            // 第6弾: 離陸直後のgamは0.5秒かけて0→0.05へ滑らかに立ち上げる
            // (メインフローでliftoffTを参照してブレンド。旧実装はgam=0.04固定で唐突だった)
            L.gam = 0.0;
            L.liftoffT = L.t;
            L.vLat = 0;
            if (L.groundRoll < 0) L.groundRoll = L.rollDist;   // 方位自由化: 滑走距離で記録(ψ=0では従来のL.xと一致)
        } else if (fuji && insideWaterSite(prm, L.x - L.z0, L.yl)) {
            // 富士川: 滑走のまま川/海に突っ込んだら着水(方位自由化で全方向がありうる)
            L.splash = true; L.done = true; L.impact = "splash";
        } else if (fuji && !L.touchdowns && L.rollDist > site::FUJI_OVERRUN_DISTANCE) {
            // 富士川の離陸失敗: 方位自由化で「滑走路端x到達」判定は成立しないため、
            // 「滑走距離1050mで離陸速度未達」に置換(滑走路850m+河川敷の余裕分200m)
            L.overrun = true; L.done = true; invalidateOfficialDistance(L);
        } else if (!fuji && (L.z0 - L.x) < 10) {
            // 琵琶湖(レガシー滑走路)従来判定: 滑走路端到達
            if (!L.touchdowns) { L.overrun = true; L.done = true; invalidateOfficialDistance(L); }
            else { L.splash = true; L.done = true; L.impact = "splash"; }   // 滑走で水際を越えた
        }
        else if (dGroundV <= 0.005 && L.t > 20 && !L.touchdowns) {
            L.overrun = true; L.done = true; invalidateOfficialDistance(L);
        }
        // 再着陸後に停止したら着陸成功として終了(推力があればV<=0.3に留まらない)
        else if (L.touchdowns && L.t - L.tdT > 3.0 && L.V <= 0.3) { L.landed = true; L.done = true; }
        return;
    }
    // ---- 破壊後: 揚力を失って落下 ----
    if (L.sparBroken) {
        L.phi += 0.7 * dt;                          // 錐もみ
        L.gam = std::max(-1.2, L.gam - 0.5 * dt);
        const double CDw = 0.08;
        const double dVw = (-q * c.S * CDw) / c.m - c.g * std::sin(L.gam);
        L.V = std::max(2.0, L.V + dVw * dt);
        L.h = std::max(0.0, L.h + L.V * std::sin(L.gam) * dt);
        L.x += std::max(0.0, L.V * std::cos(L.gam) * std::cos(L.psi) + prm.wind) * dt;
        L.yl += (L.V * std::cos(L.gam) * std::sin(L.psi) + prm.xwind) * dt;
        L.t += dt; L.n = 0.2;
        refreshOfficialDistance(L);
        if (L.h <= 0) {
            const bool onWater = insideWaterSite(prm, L.x - L.z0, L.yl);
            L.splash = onWater; L.crashed = true; L.done = true; L.impact = "crash";
        }
        return;
    }
    // 地形風(位置依存の加算風: 比良おろし・岸サーマル)
    double twW = 0, twX = 0, twVz = 0;
    if (prm.terrainWind) localWind(prm, L.x - L.z0, L.yl, L.h, twW, twX, twVz);

    // ---- 横の運動: エルロン/ラダー → バンク・方位 ----
    // エルロンを放したら翼が水平へ戻る(暗黙のウィングレベラー)。
    // 入力中はフェードアウトして操舵を邪魔しない
    const double phiMax = 0.55 + 0.25 * prm.sens;
    const double aFade = std::max(0.0, 1 - std::abs(L.ail) * 2.5);
    const double dphi = c.ailRate * L.ail + c.rudRoll * L.rud - (0.12 + 0.9 * aFade) * L.phi;
    L.phi = clamp(L.phi + dphi * dt, -phiMax, phiMax);
    // ---- 縦+揚力 ----
    // 地面効果の揚力側(鏡像渦の吹き下ろし減で実効CLmax増)+フラップ増分
    const double geL = 1 + 0.10 * (1 - groundEffect(L.h, c.span));
    const double CLmaxE = (c.CLmax + c.flapDCLmax * L.flap) * geL;
    const double VsE = c.Vs * std::sqrt(c.CLmax / std::max(0.3, CLmaxE));
    const double CLl = std::min(CLmaxE, c.W * std::cos(L.gam) / std::max(q * c.S * std::max(0.3, std::cos(L.phi)), 1.0));
    const double Vz = L.V * std::sin(L.gam);
    const bool holdOn = L.auto_ || prm.hold;
    const double holdTgt = L.auto_ ? (L.softHTgt >= 0 ? L.softHTgt : prm.hTgt) : prm.hTgt;
    const double holdGain = holdOn ? (L.auto_ ? 1.0 : std::max(0.0, 1 - std::abs(L.e) * 5)) : 0.0;
    const double hold = clamp(0.03 * (holdTgt - L.h), -0.10, 0.10) * holdGain;
    // 鉛直速度フィードバックを0.10→0.16へ強化: 旧値では位相遅れが大きく、
    // エレベーター引きっぱなしで減衰しないフゴイド振動(V=5.8↔8.2の永久往復)に
    // 入っていた(診断で確認)。実機のフゴイドは弱いながら減衰する
    const double CLminE = effectiveCLmin(c, L);
    const double CLraw = CLl * std::min(1.0, (L.V / c.Vt) * (L.V / c.Vt)) + gustDCL
              + hold - 0.16 * Vz + c.elevAuth * L.e
              + c.flapDCL * L.flap;
    const double negF = clamp((CLminE - CLraw) / 0.25, 0.0, 1.0);
    double CL = clamp(CLraw, CLminE, CLmaxE);
    if (negF > 0) CL *= 1.0 - 0.25 * negF;
    // ---- 失速(ストール)挙動 ----
    // 従来は失速速度以下でも抗力ペナルティのみで「失速寸前で永遠に耐える」人工挙動だった。
    // CL要求が実効CLmaxを超えたまま失速速度を割ると翼の流れが剥がれる:
    //  ・揚力が最大35%崩れる(ポストストールのCL低下)
    //  ・機首落ち(dgへ下向き強制)+バフェット(細かい揺れ)
    //  ・速度が戻れば自然に回復(深さに比例して滑らかに遷移)
    const double CLneed = c.W * std::cos(L.gam) / std::max(q * c.S * std::max(0.3, std::cos(L.phi)), 1.0);
    const double stallF = (CLneed > CLmaxE && L.V < VsE)
                        ? clamp((VsE / std::max(L.V, 1.0) - 1.0) * 5.0, 0.0, 1.0) : 0.0;
    CL *= 1.0 - 0.35 * stallF;
    const double lift = q * c.S * CL;
    L.n = lift / c.W;
    // レイノルズ数効果: 翼型極曲線テーブルの断面cd0差分(設計Re基準)
    const double Re = c.rho * L.V * c.chordRef / 1.81e-5;
    const double dProf = clamp(reTableInterp(c.afReT, c.afCdT, Re) - c.cdProfD, -0.004, 0.012);
    double CD = c.CD0 + dProf + CL * CL / (PI * c.AR * c.e) * groundEffect(L.h, c.span);
    if (L.V < VsE) CD += 0.12 * (VsE / L.V - 1);
    CD += 0.010 * L.rud * L.rud + 0.004 * L.ail * L.ail;
    CD += c.flapDCD * L.flap * L.flap;   // フラップの形状抗力(舵角²に比例)
    L.Re = Re;
    if (updateStructuralFailure(L, c, dt)) return;
    // ---- 運動方程式(3自由度) ----
    const double dV = (T - q * c.S * CD) / c.m - c.g * std::sin(L.gam);
    double dg = (lift * std::cos(L.phi) / c.m - c.g * std::cos(L.gam)) / std::max(L.V, 1.0);
    // 失速中は機首落ちを強制(揚力崩れに加えピッチングモーメントの失速崩れを表現)
    dg -= 0.35 * stallF;
    dg += 0.30 * negF;   // 負側失速では揚力絶対値が崩れ、機首上げ方向へ回復
    // 経路角速度クランプはHPAのCL式パイロット暴走防止の人工制限。
    // 剛体翼のお遊び機は実機並みに機敏な引き起こし(着陸フレア)を許す
    dg = prm.funPlane ? clamp(dg, -1.0, 0.7) : clamp(dg, -0.6, 0.25);
    const double dpsi = lift * std::sin(L.phi) / (c.m * std::max(L.V, 1.0) * std::max(0.3, std::cos(L.gam))) + c.rudYaw * L.rud;
    L.V = std::max(0.5, L.V + dV * dt);
    L.gam = clamp(L.gam + dg * dt, -1.2, 0.35);
    // 第6弾: 離陸直後0.5秒はgamを0→0.05へ線形に立ち上げる(唐突なピッチ跳ねの緩和)。
    // エレベーター入力(|e|>0.05)があれば操舵優先で介入しない(下限としてのみ働く)
    if (L.t - L.liftoffT < 0.5 && std::abs(L.e) <= 0.05)
        L.gam = std::max(L.gam, 0.05 * (L.t - L.liftoffT) / 0.5);
    // 失速バフェット: 剥離流によるロールの細かい震え(失速の体感的な警告にもなる)
    if (stallF > 0) L.phi += std::sin(L.t * 11.0) * 0.5 * stallF * dt;
    L.psi += dpsi * dt;
    L.maxBank = std::max(L.maxBank, std::abs(L.phi));
    if (L.inThermal) L.inThermalUsed = true;
    // 乱流外乱
    const double turbVz = L.tz, turbVy = L.ty;
    const double groundVz = L.V * std::sin(L.gam) + turbVz + prm.verticalGust + L.thrmVz + twVz;
    L.h = std::max(0.0, L.h + groundVz * dt);
    // ---- 接地判定 ----
    if (L.h <= 0 && groundVz < 0) {
        const double sink = -groundVz;
        const bool onLand = !insideWaterSite(prm, L.x - L.z0, L.yl);
        const bool rwMode = prm.mode == "runway";
        if (rwMode && onLand && !L.sparBroken) {
            // 第6弾: 滑走路モードの陸上接地は沈下率で品質分岐(通常/バウンド/装置破損/クラッシュ)
            // +クラブ着陸の残留横速度処理(共通ヘルパー)
            applyTouchdown(L, prm, sink, false);
        } else if (!rwMode && c.hasGear && onLand && sink < 1.5 && std::abs(L.phi) < 0.25 && !L.sparBroken) {
            // 琵琶湖(プラットフォーム系)の陸上接地: 従来判定を維持(第6弾の対象外)
            L.ground = true; L.h = 0; L.gam = 0; L.touchdowns++; L.tdT = L.t;
        } else {
            L.h = 0; L.done = true;
            L.splash = !onLand;
            L.crashed = onLand || L.sparBroken || sink > 1.5;
            L.impact = L.crashed ? "crash" : "splash";
            if (L.crashed && L.failureMsg.empty()) L.failureMsg = u8"墜落";
        }
    }
    // 方位を±πに正規化
    if (L.psi > PI) L.psi -= 2 * PI; else if (L.psi < -PI) L.psi += 2 * PI;
    const double vh = L.V * std::cos(L.gam);
    double wind3 = 0, xwind3 = 0;
    horizontalWindAt(prm, L.x - L.z0, L.yl, L.h, wind3, xwind3);
    L.x += (vh * std::cos(L.psi) + wind3) * dt;
    L.yl += (vh * std::sin(L.psi) + xwind3 + turbVy) * dt;
    L.pathAir += std::max(0.0, L.V) * std::cos(L.gam) * dt;
    refreshOfficialDistance(L);
    L.t += dt;
    if (std::abs(L.yl) > 2500 && !prm.funPlane) { L.offcourse = true; L.done = true; }   // お遊び機は自由飛行
    if (L.t >= 3600 || L.officialDist >= 30000) L.done = true;
}

// ============ 拡張物理(回転モデル) ============
// 標準モデルとの違い:
//  - ピッチ姿勢thetaと迎角alphaを分離し、SM由来の復元モーメント・尾翼減衰で回転動力学を解く
//  - ロールはエルロン/上反角効果のモーメント、ヨーは垂直尾翼の風見安定で駆動
//  - 失速はalpha超過でCL崩れ+機首下げ(ピッチブレーク)として現れる
// 地上滑走・破壊後は標準モデルと同一処理に委譲する。
void stepSim6(FlightState& L, const AircraftConstants& c, const SimParams& prm, double dt) {
    if (L.ground || L.sparBroken) {
        const bool wasGround = L.ground;
        stepSim(L, c, prm, dt);
        if (wasGround && !L.ground) {
            // 離陸: 現在速度の釣り合い迎角でトリムして空中へ(init6と同じ考え方)。
            // theta=gam(迎角0)だと巡航速度未満のローテーションで揚力不足になり
            // 接地バウンドを繰り返す(巡航速度が離陸速度より速い機体で顕著)
            const double q2 = 0.5 * c.rho * L.V * L.V;
            const double CLneed = clamp(c.W / std::max(q2 * c.S, 1.0), 0.2, 0.92 * c.CLmax);
            L.theta = L.gam + clamp((CLneed - c.CLcruise) / c.CLa, -0.05, 0.9 * c.astall + 0.08);
            L.alpha = L.theta - L.gam; L.beta = 0;
            L.pRate = L.qRate = L.rRate = 0;
            L.init6 = true;
        }
        return;
    }
    const double q = 0.5 * c.rho * L.V * L.V;
    // 初回: 現在の対気速度に釣り合う迎角でトリムして開始。
    // 失速速度未満の発進では自然に機首下げのダイブ姿勢から入る
    // (JS/3DOFのCL低減による発進ダイブに相当)
    if (!L.init6) {
        const double CLneed = clamp(c.W / std::max(q * c.S, 1.0), 0.2, 0.92 * c.CLmax);
        L.theta = L.gam + (CLneed - c.CLcruise) / c.CLa;
        L.init6 = true;
    }
    stepTurbulence(L, prm, dt);
    stepThermal(L, prm);
    stepSteering(L, prm, dt);
    const double T = computeThrust(L, c, prm, dt);

    const double b = c.span, S = c.S, MAC = c.chordRef;
    const double Vs2 = std::max(L.V, 2.0);
    const double Vz = L.V * std::sin(L.gam);

    // ---- 迎角と揚力 ----
    L.alpha = L.theta - L.gam;                          // トリム基準の迎角偏差
    const double alphaG = std::atan2(L.tz + prm.verticalGust, std::max(L.V, 3.0));
    const double alphaAero = L.alpha + alphaG;
    // フラップ増分と地面効果の揚力側(実効CLmax・失速迎角が変わる)
    const double geL = 1 + 0.10 * (1 - groundEffect(L.h, c.span));
    const double CLmaxE = (c.CLmax + c.flapDCLmax * L.flap) * geL;
    const double astallE = c.astall
                         + ((c.flapDCLmax - c.flapDCL) * L.flap + (geL - 1) * c.CLmax) / c.CLa;
    const double CLminE = effectiveCLmin(c, L);
    // CL需要がCLminへ到達する迎角。フラップ揚力を差し引かないと、展開時に
    // まだ正揚力の領域でも負側失速が早期発動してしまう。
    const double astallNeg = (CLminE - c.CLcruise - c.flapDCL * L.flap) / c.CLa;
    // 短時間突風の初期荷重は3DOFと同じ線形ΔCLを使う。失速判定は機体姿勢由来の
    // 迎角に適用し、突風分はCLmaxまでの瞬間荷重として加える（ピッチモーメントは
    // 下のalphaAeroで突風を含むため、その後の6DOF応答は維持される）。
    double CL = c.CLcruise + c.CLa * L.alpha + c.flapDCL * L.flap;
    double stallPen = 0, stallNegPen = 0;
    if (L.alpha > astallE) {
        // マッシュ(緩やかな失速): 実翼はCLmax超過後も揚力を大きくは失わない。
        // 急峻なCL崩壊は「揚力減→経路角低下→迎角増」の正帰還で深失速に
        // ロックする非物理挙動を生むため、緩勾配+強い機首下げで回復性を持たせる
        stallPen = L.alpha - astallE;
        CL = std::max(0.55, CLmaxE - 0.25 * c.CLa * stallPen);
    } else if (L.alpha < astallNeg) {
        stallNegPen = astallNeg - L.alpha;
        CL = std::min(-0.25, CLminE + 0.25 * c.CLa * stallNegPen);
    }
    CL += c.CLa * alphaG;
    CL = clamp(CL, CLminE, CLmaxE);
    const double lift = q * S * CL;
    L.n = lift / c.W;

    if (updateStructuralFailure(L, c, dt)) return;

    // ---- 抗力 ----
    const double Re = c.rho * L.V * c.chordRef / 1.81e-5;
    const double dProf = clamp(reTableInterp(c.afReT, c.afCdT, Re) - c.cdProfD, -0.004, 0.012);
    double CD = c.CD0 + dProf + CL * CL / (PI * c.AR * c.e) * groundEffect(L.h, c.span);
    CD += 0.010 * L.rud * L.rud + 0.004 * L.ail * L.ail;
    CD += 1.2 * (stallPen * stallPen + stallNegPen * stallNegPen)
        + ((stallPen > 0 || stallNegPen > 0) ? 0.015 : 0.0);          // 正負失速後の剥離抗力
    CD += 0.4 * L.beta * L.beta;                                     // 横滑り抗力
    CD += c.flapDCD * L.flap * L.flap;                               // フラップの形状抗力
    L.Re = Re;

    // ---- エレベーター実効値 ----
    // JSの3DOFはCL式に「1g維持+沈下率減衰(-0.10·Vz)」の完璧なパイロットが内包されている。
    // 6DOFで同じ操作感にするため、無入力時は沈下率を打ち消す暗黙の操舵を加える
    // (ユーザー入力があるほどフェードアウトし、手動操舵を邪魔しない)
    double eEff = L.e;
    {
        const bool holdOn = L.auto_ || prm.hold;
        if (holdOn) {
            const double holdTgt = L.auto_ ? (L.softHTgt >= 0 ? L.softHTgt : prm.hTgt) : prm.hTgt;
            const double holdGain = L.auto_ ? 1.0 : std::max(0.0, 1 - std::abs(L.e) * 5);
            const double VzT = clamp(0.4 * (holdTgt - L.h), -0.6, 0.8);
            const double eCmd = clamp(0.5 * (VzT - Vz) - 2.0 * L.qRate, -0.8, 0.8);
            eEff = clamp(L.e + holdGain * eCmd, -1.0, 1.0);
        } else if (prm.assist) {
            const double userFade = std::max(0.0, 1 - std::abs(L.e) * 2.5);
            const double ePilot = clamp(-0.12 * Vz - 1.5 * L.qRate, -0.7, 0.7);
            eEff = clamp(L.e + userFade * ePilot, -1.0, 1.0);
        }
    }
    L.eApplied = eEff;

    // ---- ピッチ回転: 静安定(SM)・尾翼減衰・エレベーター・失速ピッチブレーク ----
    // 角速度減衰項(Cmq/Clp/Cnr)は半陰的に積分する: rate_new=(rate+外力·dt)/(1+減衰·dt)。
    // 高動圧(高速機・お遊び機)で減衰が強くてもdt=0.02で無条件安定
    {
        const double K = q * S * MAC / c.Iyy;
        double Cm = c.Cma * alphaAero + c.Cmde * eEff + c.flapCm * L.flap;   // フラップの機首下げ
        if (stallPen > 0) Cm += -0.9 * stallPen;        // 失速で機首下げ(回復性)
        if (stallNegPen > 0) Cm += 0.9 * stallNegPen;    // 負側失速で機首上げ(回復性)
        const double damp = std::max(0.0, -c.Cmq * MAC / (2 * Vs2) * K);
        L.qRate = clamp((L.qRate + Cm * K * dt) / (1 + damp * dt), -1.8, 1.8);
        L.theta += L.qRate * dt;
        // 迎角の暴走防止(クランプ時は角速度も打ち消してワインドアップを防ぐ)
        const double thLo = L.gam - 0.45, thHi = L.gam + 0.7;
        if (L.theta > thHi) { L.theta = thHi; L.qRate = std::min(L.qRate, 0.0); }
        else if (L.theta < thLo) { L.theta = thLo; L.qRate = std::max(L.qRate, 0.0); }
    }
    // 暗黙のラダー協調: アドバースヨーによる横滑りβを打ち消す(ラダー入力でフェード)。
    // HPAは垂直尾翼が小さくβが大きく成長し、上反角効果がエルロンを打ち消して
    // ロールがほぼ効かなくなるため、実機同様の協調旋回を暗黙パイロットが行う
    const double rFade = prm.assist ? std::max(0.0, 1 - std::abs(L.rud) * 2.5) : 0.0;
    const double rudEff = clamp(L.rud + rFade * clamp(-2.5 * L.beta - 0.8 * L.rRate, -0.7, 0.7),
                                -1.0, 1.0);
    L.rudApplied = rudEff;
    // ---- ロール回転: エルロン・ラダー・上反角効果(機首右偏β→右ロール)・減衰 ----
    {
        // たわみ連成: 実効上反角(治具角+曲げ×n)からClbを毎ステップ再評価。
        // 突風や引き起こしで翼が大きくたわむと横安定が強まる(HPA特有の挙動)
        const double Clb = c.CLa * clamp(c.dihBase + c.dihBend * L.n, -2.0, 15.0) * PI / 180 / 4;
        // エルロンを放したら水平へ戻す暗黙の操舵(3DOFのウィングレベラーと同じ操作感。
        // 入力でフェードアウト。エルロンなし機はClda=0のため従来通り上反角頼み)
        const double aFade = prm.assist ? std::max(0.0, 1 - std::abs(L.ail) * 2.5) : 0.0;
        const double ailEff = clamp(L.ail + aFade * clamp(-2.6 * L.phi - 2.0 * L.pRate, -0.85, 0.85),
                                    -1.0, 1.0);
        L.ailApplied = ailEff;
        double Cl = c.Clda * ailEff + c.Cldr * rudEff + Clb * L.beta
                  + 0.02 * L.ty;                        // 乱流ロール外乱(Clp減衰は陰的)
        // 翼端失速のロール崩れ: 失速開始位置(LLT)が翼端側ほど左右差が
        // 大きな非対称失速になり、勝手にロールが入る。引き金は乱流の左右差
        if (stallPen > 0) {
            const double dir = std::abs(L.ty) > 0.02 ? (L.ty > 0 ? 1.0 : -1.0)
                                                     : (L.phi >= 0 ? 1.0 : -1.0);
            Cl += dir * stallPen * (0.2 + 1.2 * c.tipStall);
        }
        const double Kr = q * S * b / c.Ixx;
        const double dampR = std::max(0.0, -c.Clp * b / (2 * Vs2) * Kr);
        L.pRate = clamp((L.pRate + Cl * Kr * dt) / (1 + dampR * dt), -2.0, 2.0);
        const double phiMax = 0.9 + 0.25 * prm.sens;
        L.phi = clamp(L.phi + L.pRate * dt, -phiMax, phiMax);
    }
    // ---- ヨー回転と横滑り: 風見安定・ラダー・アドバースヨー ----
    double psivDot;
    {
        const double Cn = -c.Cnb * L.beta + c.Cndr * rudEff
                        - 0.05 * c.Clda * L.ail;        // アドバースヨー(Cnr減衰は陰的)
        const double Ky = q * S * b / c.Izz;
        const double dampY = std::max(0.0, -c.Cnr * b / (2 * Vs2) * Ky);
        L.rRate = clamp((L.rRate + Cn * Ky * dt) / (1 + dampY * dt), -1.5, 1.5);
        // 進行方向の旋回率: バンクによる求心力 + 横滑りの側力
        psivDot = lift * std::sin(L.phi) / (c.m * std::max(L.V, 1.0) * std::max(0.3, std::cos(L.gam)))
                + q * S * c.CYs * L.beta / (c.m * Vs2);
        L.beta += (L.rRate - psivDot) * dt;
        L.beta = clamp(L.beta, -0.6, 0.6);
    }

    // ---- 並進運動 ----
    // 3DOFのdgクランプ(-0.6..0.25)はCL式パイロットの過大な引き起こしを抑える
    // 人工的制限。6DOFでは回転動力学が引き起こし速度を律するため物理値のまま使う
    // (数値安全のための広い範囲のみ)
    const double dV = (T - q * S * CD) / c.m - c.g * std::sin(L.gam);
    double dg = (lift * std::cos(L.phi) / c.m - c.g * std::cos(L.gam)) / std::max(L.V, 1.0);
    dg = clamp(dg, -1.2, 1.2);
    L.V = std::max(0.5, L.V + dV * dt);
    L.gam = clamp(L.gam + dg * dt, -1.2, 0.35);
    // 第6弾: 離陸直後0.5秒のgam滑らか立ち上げ(stepSimと同一。詳細はそちらのコメント参照)
    if (L.t - L.liftoffT < 0.5 && std::abs(L.e) <= 0.05)
        L.gam = std::max(L.gam, 0.05 * (L.t - L.liftoffT) / 0.5);
    L.psi += psivDot * dt;
    L.maxBank = std::max(L.maxBank, std::abs(L.phi));
    if (L.inThermal) L.inThermalUsed = true;
    // 地形風(位置依存の加算風)
    double twW = 0, twX = 0, twVz = 0;
    if (prm.terrainWind) localWind(prm, L.x - L.z0, L.yl, L.h, twW, twX, twVz);
    const double groundVz = L.V * std::sin(L.gam) + L.tz + prm.verticalGust + L.thrmVz + twVz;
    L.h = std::max(0.0, L.h + groundVz * dt);

    // ---- 接地判定(標準と同一: 第6弾の品質分岐も共通ヘルパーで適用) ----
    if (L.h <= 0 && groundVz < 0) {
        const double sink = -groundVz;
        const bool onLand = !insideWaterSite(prm, L.x - L.z0, L.yl);
        const bool rwMode = prm.mode == "runway";
        if (rwMode && onLand && !L.sparBroken) {
            applyTouchdown(L, prm, sink, true);
        } else if (!rwMode && c.hasGear && onLand && sink < 1.5 && std::abs(L.phi) < 0.25 && !L.sparBroken) {
            // 琵琶湖(プラットフォーム系)の陸上接地: 従来判定を維持(第6弾の対象外)
            L.ground = true; L.h = 0; L.gam = 0; L.theta = 0; L.touchdowns++; L.tdT = L.t;
        } else {
            L.h = 0; L.done = true;
            L.splash = !onLand;
            L.crashed = onLand || L.sparBroken || sink > 1.5;
            L.impact = L.crashed ? "crash" : "splash";
            if (L.crashed && L.failureMsg.empty()) L.failureMsg = u8"墜落";
        }
    }
    if (L.psi > PI) L.psi -= 2 * PI; else if (L.psi < -PI) L.psi += 2 * PI;
    const double vh = L.V * std::cos(L.gam);
    double wind6 = 0, xwind6 = 0;
    horizontalWindAt(prm, L.x - L.z0, L.yl, L.h, wind6, xwind6);
    L.x += (vh * std::cos(L.psi) + wind6) * dt;
    L.yl += (vh * std::sin(L.psi) + xwind6 + L.ty) * dt;
    L.pathAir += std::max(0.0, L.V) * std::cos(L.gam) * dt;
    refreshOfficialDistance(L);
    L.t += dt;
    if (std::abs(L.yl) > 2500 && !prm.funPlane) { L.offcourse = true; L.done = true; }   // お遊び機は自由飛行
    if (L.t >= 3600 || L.officialDist >= 30000) L.done = true;
}

} // namespace bm
