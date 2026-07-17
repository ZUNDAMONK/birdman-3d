#include "core/Aircraft.hpp"
#include "core/Material.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace bm {

double lerp(double a, double b, double t) { return a + (b - a) * t; }
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

static const double PI = 3.14159265358979323846;

double chordAt(const AircraftParams& p, double t) {
    if (p.planform == "rect") return p.rootChord;
    if (p.planform == "taper") return lerp(p.rootChord, p.tipChord, t);
    if (p.planform == "ellipse")
        return std::max(p.tipChord * 0.5, p.rootChord * std::sqrt(std::max(0.0, 1 - t * t)));
    // crescent
    if (t < 0.5) return p.rootChord;
    double u = (t - 0.5) / 0.5;
    return lerp(p.rootChord, p.tipChord, 1 - std::sqrt(std::max(0.0, 1 - u * u)));
}

const std::vector<AirfoilData>& airfoilDB() {
    // 極曲線テーブル: Re = 10万/20万/40万/70万 での cd0(最小抗力) と clmax。
    // XFOIL/風試の公表ポーラーに基づく代表値。厚翼(FX76)ほど低Reで悪化が大きく、
    // 薄翼(AG-18)は低Re耐性が高い — 翼弦長の設計が「Reの壁」に直結する
    static const std::vector<AirfoilData> DB = {
        {"dae31",  "DAE-31",   u8"Drela設計。HPA定番・高効率",   1.00, 0.0095, 1.45, -0.10, 0.85, 0.105,
         {1e5, 2e5, 4e5, 7e5}, {0.0150, 0.0110, 0.0092, 0.0085}, {1.30, 1.38, 1.46, 1.50}},
        {"dae21",  "DAE-21",   u8"DAE系・やや薄翼で高速向き",    0.90, 0.0088, 1.35, -0.09, 0.90, 0.092,
         {1e5, 2e5, 4e5, 7e5}, {0.0135, 0.0102, 0.0086, 0.0080}, {1.22, 1.30, 1.36, 1.40}},
        {"fx76",   "FX 76-MP", u8"Wortmann。高揚力・低速安定",   1.10, 0.0105, 1.55, -0.12, 0.80, 0.135,
         {1e5, 2e5, 4e5, 7e5}, {0.0170, 0.0125, 0.0102, 0.0094}, {1.42, 1.50, 1.57, 1.60}},
        {"ag18",   "AG-18",    u8"薄翼・軽量。製作しやすい",     0.85, 0.0090, 1.30, -0.08, 0.88, 0.082,
         {1e5, 2e5, 4e5, 7e5}, {0.0115, 0.0095, 0.0087, 0.0084}, {1.20, 1.27, 1.32, 1.34}},
        {"sd7037", "SD-7037",  u8"Selig。広い速度域で安定",      0.95, 0.0098, 1.40, -0.09, 0.86, 0.092,
         {1e5, 2e5, 4e5, 7e5}, {0.0125, 0.0100, 0.0092, 0.0089}, {1.28, 1.35, 1.42, 1.45}},
        {"e387",   "E-387",    u8"Eppler。実績豊富な中庸型",     0.92, 0.0100, 1.38, -0.10, 0.84, 0.091,
         {1e5, 2e5, 4e5, 7e5}, {0.0130, 0.0102, 0.0094, 0.0090}, {1.25, 1.33, 1.40, 1.42}},
    };
    return DB;
}

// log(Re)線形補間(端はクランプ)
double reTableInterp(const double reT[4], const double vT[4], double Re) {
    const double lr = std::log(clamp(Re, 2e4, 3e6));
    if (lr <= std::log(reT[0])) return vT[0];
    for (int i = 1; i < 4; i++) {
        const double l1 = std::log(reT[i]);
        if (lr <= l1) {
            const double l0 = std::log(reT[i - 1]);
            return lerp(vT[i - 1], vT[i], (lr - l0) / (l1 - l0));
        }
    }
    return vT[3];
}
double afCd0AtRe(const AirfoilData& af, double Re)   { return reTableInterp(af.reT, af.cdT, Re); }
double afClmaxAtRe(const AirfoilData& af, double Re) { return reTableInterp(af.reT, af.clT, Re); }

const AirfoilData& airfoilOf(const AircraftParams& p) {
    for (const auto& a : airfoilDB()) if (p.airfoil == a.id) return a;
    return airfoilDB()[0];
}

WingGeom wingGeom(const AircraftParams& p) {
    const int N = 40;
    double S = 0, c2 = 0;
    const double half = p.span / 2;
    for (int i = 0; i < N; i++) {
        double t = (i + 0.5) / N;
        double c = chordAt(p, t);
        S += c * (half / N) * 2;
        c2 += c * c * (half / N) * 2;
    }
    return {S, c2 / S, p.span * p.span / S};
}

Analysis analyze(const AircraftParams& st) {
    const double g = 9.81, rho = 1.225;
    WingGeom wgm = wingGeom(st);
    const double S = wgm.S, MAC = wgm.MAC, AR = wgm.AR;
    const double half = st.span / 2;
    const double wingLE = st.wingX, xAC = wingLE + 0.25 * MAC;
    const double xHT = xAC + st.tailArm, xVT = xHT - 0.1;
    const double fusLen = xHT + st.hChord * 0.8;
    const double zBeamF = st.seatX - 0.55;
    double xProp;
    if (st.propConfig == "tractor") xProp = st.seatX - 1.5;
    else if (st.propConfig == "pylon") xProp = wingLE + MAC + 0.6;
    else if (st.propConfig == "midboom") xProp = (zBeamF + fusLen) / 2;
    else xProp = fusLen;
    const double Sh = st.hShape == "ellipse" ? (PI / 4) * st.hSpan * st.hChord
        : st.hSpan * st.hChord * (st.hShape == "taper" ? 0.8 : st.hShape == "swept" ? 0.82 : st.hShape == "delta" ? 0.55 : 1.0);
    const double Sv = st.vShape == "ellipse" ? (PI / 4) * st.vHeight * st.vChord
        : st.vHeight * st.vChord * ((st.vShape == "swept" || st.vShape == "dorsal") ? 0.85 : st.vShape == "delta" ? 0.60 : 1.0);
    const int nRibs = (int)std::floor(half / st.ribPitch) * 2;
    const double plankFrac = ((st.plankTop + st.plankBot + st.plankRearTop + st.plankRearBot) / 2) / 100;
    const double wRibs = nRibs * 0.042, wPlank = S * plankFrac * 0.26, wFilm = S * 2.05 * 0.034;
    const MaterialGrade& mat = materialOf(st.sparMat);
    const double dr = st.rootDia / 1000.0, dt = st.tipDia / 1000.0;
    // Linear-taper thin tube integrated over both half-wings; t=d/80.
    const double wSpar = 2 * SPAR_MASS_CAL * mat.density * (PI / 80.0) * (half / 3.0)
                       * (dr * dr + dr * dt + dt * dt);
    double wJoints = 0;
    for (int k = 1; k < std::max(1, st.segments); k++) {
        const double u = k / (double)std::max(1, st.segments);
        const double d = lerp(dr, dt, u), sleeveT = 1.5 * d / 80.0;
        wJoints += 2 * mat.density * PI * d * sleeveT * 0.18;
    }
    const double wAil = st.ailMode == "none" ? 0 : st.ailSpanFrac * st.span * 0.25 + 0.3;
    const double wFlap = st.flapSpanFrac > 0.01 ? st.flapSpanFrac * st.span * 0.22 + 0.2 : 0;
    const double wBoomWing = (st.boomWing != "none")
        ? ((st.boomWing == "LR" ? 2 : 1) * st.boomWingSpan * st.boomWingChord * 0.55 + 0.12) : 0;
    const double wWing = wRibs + wPlank + wFilm + wSpar + wJoints + wAil + wFlap + wBoomWing;
    const double wHT = Sh * 0.55 + (st.elevRatio < 1 ? 0.15 : 0.05);
    const double wVT = Sv * 0.55 + (st.rudRatio < 1 ? 0.12 : 0.05);
    const double boomLen = fusLen - (wingLE + MAC * 0.6);
    const double boomD = clamp(st.boomDia, 50.0, 120.0) / 1000.0;
    const double wBoom = SPAR_MASS_CAL * mat.density * PI * boomD * (boomD / 80.0)
                       * std::max(0.0, boomLen);
    const double wCockpit = 3.2 + (st.posture == "upright" ? 0.3 : 0.0);
    const double driveDist = std::abs(xProp - st.seatX) + (st.propConfig == "pylon" ? 1.2 : 0.0);
    const double wShaft = st.drive == "shaft" ? 0.25 * driveDist : 0.0;
    const double wDrive = 1.2 + driveDist * 0.35 + wShaft;
    const double wProp = (0.45 + st.propDia * 0.18) * (st.propMat == "carbon" ? 0.78 : 1.0)
                       * (0.8 + 0.1 * std::max(1, st.propBlades))
                       + (st.varPitch ? 0.6 : 0.0);   // 可変ピッチ機構(ハブ+リンク)
    const double wGear = st.gear == "tandem" ? 0.9 : st.gear == "tri" ? 1.4
                       : st.gear == "mono" ? 0.6 : 0.0;
    const double wFairing = st.fairing ? 1.1 : 0.0;
    // 旧miscに含まれていた脚取付金具分0.3kgを明示的なgearへ移し、二重計上を避ける。
    const double wMisc = 0.7;
    const double wEmpty = wWing + wHT + wVT + wBoom + wCockpit + wDrive + wProp
                        + wGear + wFairing + wMisc;
    const double W = (wEmpty + st.pilotW) * g;
    const double pilotCGx = st.seatX + (st.posture == "upright" ? -0.05 : st.posture == "semi" ? -0.15 : -0.25);

    Analysis a;
    a.items = {
        {u8"主翼", wWing, xAC + 0.02 * MAC}, {u8"水平尾翼", wHT, xHT}, {u8"垂直尾翼", wVT, xVT},
        {u8"ブーム", wBoom, (wingLE + MAC * 0.6 + fusLen) / 2}, {u8"コックピット", wCockpit, st.seatX + 0.1},
        {u8"駆動系", wDrive, (st.seatX + xProp) / 2}, {u8"プロペラ", wProp, xProp},
        {u8"着陸装置", wGear, st.seatX}, {u8"フェアリング", wFairing, st.seatX + 0.1},
        {u8"その他", wMisc, st.seatX}, {u8"パイロット", st.pilotW, pilotCGx}};
    double totalM = 0, moment = 0;
    for (const auto& it : a.items) { totalM += it.w; moment += it.w * it.x; }
    const double xCG = moment / totalM;
    const double lh = xHT - xCG, Vh = Sh * lh / (S * MAC), Vv = Sv * (xVT - xCG) / (S * st.span);
    const double tailEff = 0.9 * (st.elevRatio == 1.0 ? 1.0 : 0.85);
    const double xNP = xAC + MAC * Vh * tailEff * 0.72;
    const double SM = (xNP - xCG) / MAC * 100;
    const double CL = 1.0, V = std::sqrt(2 * W / (rho * S * CL));
    const double CD0 = 0.014 + st.cd0Add + (st.propConfig == "pylon" ? 0.0015 : 0.0)
                     + (st.ailMode != "none" ? 0.0008 : 0.0) + (st.fairing ? -0.0012 : 0.0);
    // スパン効率: 揚力線理論(平面形・ねじり反映)+胴体/粘性補正0.95
    const double e = clamp(computeLLT(st, 1.0).e * 0.95, 0.6, 0.97);
    const double CDi = CL * CL / (PI * AR * e);
    const double D = 0.5 * rho * V * V * S * (CD0 + CDi);
    // 必要軸出力: BEMTで推力=抗力となるパワー(プロペラ設計が直接効く)
    const PropTables& pt = computeBEMT(st);
    const double inst = st.propConfig == "pylon" ? 1.0 : st.propConfig == "midboom" ? 0.97
                      : st.propConfig == "pusher" ? 0.92 : 0.96;
    const double Preq = propPowerFor(pt.J, pt.Ct, pt.Cp, st.propDia, rho, V, D, inst);
    const double margin = st.powerMax - Preq;
    a.S = S; a.MAC = MAC; a.AR = AR; a.Sh = Sh; a.Sv = Sv;
    a.wEmpty = wEmpty; a.W = W / g;
    a.xCG = xCG; a.xAC = xAC; a.xNP = xNP; a.SM = SM; a.Vh = Vh; a.Vv = Vv;
    a.V = V; a.Preq = Preq; a.margin = margin;
    a.nRibs = nRibs; a.wSpar = wSpar; a.wJoints = wJoints; a.wBoom = wBoom;
    a.wGear = wGear; a.wFairing = wFairing; a.wShaft = wShaft;
    a.fusLen = fusLen; a.xHT = xHT; a.xVT = xVT; a.xProp = xProp; a.wingLE = wingLE;
    a.pilotCGx = pilotCGx; a.lh = lh; a.driveDist = driveDist;
    const LoadsResult lr = computeLoads(st, a, 1.0);
    a.sparSF = lr.SF; a.failStation = lr.failStation; a.failMode = lr.failMode;
    a.baseDih = lr.baseDih; a.bendDih = lr.bendDih;
    return a;
}

// ---- 揚力線理論 ----
// Γ(θ) = 2bV Σ A_n sin(nθ) (n=1,3,5,...)。モノプレーン方程式
//   α_a(θ) = (4b/(a0·c(θ))) Σ A_n sin(nθ) + Σ n·A_n·sin(nθ)/sin(θ)
// を選点法で解く。α_a はゼロ揚力線からの迎角。対称荷重なので奇数調和のみ。
// 「一様迎角1rad」と「ねじり分布のみ」の2基底を解き、目標CLに合成する。
LLTResult computeLLT(const AircraftParams& st, double CLtarget) {
    const int M = 16;                       // 調和 A_1..A_31 / 選点数
    const AirfoilData& af = airfoilOf(st);
    const double b = st.span, a0 = 2 * PI;
    const double AR = wingGeom(st).AR;
    double th[M], ts[M], ch[M], sn[M][M];   // sn[k][j] = sin(n_j·θ_k)
    double B[M][M * 2 + 2];                 // 拡大係数行列(RHS2本を同時消去)
    for (int k = 0; k < M; k++) {
        th[k] = PI / 2 * (k + 1) / M;       // θ: 翼端寄り→π/2(翼根)
        ts[k] = std::cos(th[k]);            // スパン位置 t(0=根, 1=端)
        ch[k] = chordAt(st, ts[k]);
        for (int j = 0; j < M; j++) {
            const int n = 2 * j + 1;
            sn[k][j] = std::sin(n * th[k]);
            B[k][j] = sn[k][j] * (4 * b / (a0 * ch[k]) + n / std::sin(th[k]));
        }
        B[k][M] = 1.0;                                   // 基底u: 一様迎角1rad
        B[k][M + 1] = -st.washout * ts[k] * PI / 180;    // 基底w: 線形ねじり下げ
    }
    // ガウス消去(部分ピボット)
    for (int col = 0; col < M; col++) {
        int piv = col;
        for (int r = col + 1; r < M; r++)
            if (std::abs(B[r][col]) > std::abs(B[piv][col])) piv = r;
        if (piv != col) for (int c2 = 0; c2 <= M + 1; c2++) std::swap(B[col][c2], B[piv][c2]);
        for (int r = 0; r < M; r++) {
            if (r == col || B[col][col] == 0) continue;
            const double f = B[r][col] / B[col][col];
            for (int c2 = col; c2 <= M + 1; c2++) B[r][c2] -= f * B[col][c2];
        }
    }
    double Au[M], Aw[M];
    for (int j = 0; j < M; j++) {
        Au[j] = B[j][M] / B[j][j];
        Aw[j] = B[j][M + 1] / B[j][j];
    }
    // 目標CLになる翼根迎角: CL = π·AR·A1
    const double aRoot = (CLtarget / (PI * AR) - Aw[0]) / std::max(1e-9, Au[0]);
    double A[M];
    for (int j = 0; j < M; j++) A[j] = aRoot * Au[j] + Aw[j];
    // スパン効率 e = A1² / Σ n·A_n²
    double sum = 0;
    for (int j = 0; j < M; j++) sum += (2 * j + 1) * A[j] * A[j];
    LLTResult r;
    r.e = clamp(A[0] * A[0] / std::max(1e-12, sum), 0.3, 1.0);
    // 局所cl(θ) = 4b·ΣA_n sin(nθ)/c(θ) はCLの一次式 cl_k = P_k + Q_k·CL。
    // 最初に2D clmaxへ達する全機CLが3D失速点、その位置が失速開始スパン
    r.CLmax3D = 1e9;
    for (int k = 0; k < M; k++) {
        double gu = 0, gw = 0;
        for (int j = 0; j < M; j++) { gu += Au[j] * sn[k][j]; gw += Aw[j] * sn[k][j]; }
        const double Qk = (4 * b / ch[k]) * gu / (PI * AR * Au[0]);
        const double Pk = (4 * b / ch[k]) * (gw - Aw[0] / Au[0] * gu);
        if (Qk <= 1e-9) continue;
        const double CLk = (af.clmax - Pk) / Qk;
        if (CLk < r.CLmax3D) { r.CLmax3D = CLk; r.tStall = ts[k]; }
    }
    // 臨界翼素(最初の失速開始)後もCLは伸びる: 失速した翼素はclmax近傍を保ち
    // 残りが増分を担うため、開始位置が翼根側ほど使えるCLmaxが大きい。
    // 翼端失速(tStall→1)はほぼ伸び代なし=即危険、のまま残す
    r.CLmax3D *= 1.02 + 0.08 * (1 - r.tStall);
    r.CLmax3D = clamp(r.CLmax3D, 0.5, af.clmax);
    // 失速CL時のcl分布(根→端の順で格納、表示用)
    const double aStall = (r.CLmax3D / (PI * AR) - Aw[0]) / std::max(1e-9, Au[0]);
    for (int k = M - 1; k >= 0; k--) {
        double gs = 0;
        for (int j = 0; j < M; j++) gs += (aStall * Au[j] + Aw[j]) * sn[k][j];
        r.ts.push_back(ts[k]);
        r.clAtStall.push_back(4 * b * gs / ch[k]);
    }
    return r;
}

AeroConsts aeroConsts(const AircraftParams& st) {
    const AirfoilData& af = airfoilOf(st);
    double CD0 = af.cd0 + 0.004 + st.cd0Add + (st.propConfig == "pylon" ? 0.0015 : 0.0)
               + (st.ailMode != "none" ? 0.0008 : 0.0)
               + (af.thick - 0.092) * 0.02;
    if (st.fairing) CD0 -= 0.0012;  // フェアリング(JSはanalyze側のみだが係数を統一)
    // 揚力線理論: 平面形+ねじり下げからスパン効率と3D失速CLを解く。
    // ×0.95 は胴体干渉・粘性分の一括補正(非粘性LLTのeは実機より高く出る)
    const LLTResult llt = computeLLT(st, 1.0);
    const double e = clamp(llt.e * 0.95, 0.6, 0.97);
    double propEff = st.propConfig == "pylon" ? 0.87 : st.propConfig == "tractor" ? 0.82 : 0.8;
    propEff -= (st.propBlades - 2) * 0.015;
    propEff += st.propMat == "carbon" ? 0.012 : 0.0;
    propEff = std::max(0.7, propEff);
    const double CLmaxWing = clamp(llt.CLmax3D, 0.8, af.clmax);
    return {CD0, e, propEff, CLmaxWing, af.cl0, af.cm, af.reSens, af.thick, llt.tStall};
}

LoadsResult computeLoads(const AircraftParams& st, const Analysis& a, double n) {
    const double g = 9.81, half = st.span / 2;
    const int N = 20;
    const double W = a.W * g * n, wWing = a.items[0].w;
    std::vector<double> ys(N + 1), cA(N + 1), cE(N + 1);
    for (int i = 0; i <= N; i++) {
        double t = (double)i / N;
        ys[i] = t * half;
        cA[i] = chordAt(st, t);
        cE[i] = (4 * a.S / (PI * st.span)) * std::sqrt(std::max(0.0, 1 - t * t));
    }
    const double dy = half / N;
    auto trap = [&](const std::vector<double>& f) {
        double s = 0;
        for (int i = 0; i < N; i++) s += (f[i] + f[i + 1]) / 2 * dy;
        return s;
    };
    std::vector<double> cS(N + 1);
    for (int i = 0; i <= N; i++) cS[i] = (cA[i] + cE[i]) / 2;
    const double kL = W / (2 * trap(cS));
    std::vector<double> Lp(N + 1);
    for (int i = 0; i <= N; i++) Lp[i] = cS[i] * kL;
    const double kE = W / (2 * trap(cE));
    std::vector<double> LpE(N + 1);
    for (int i = 0; i <= N; i++) LpE[i] = cE[i] * kE;
    const double kw = n * wWing * g / (2 * trap(cA));
    std::vector<double> net(N + 1);
    for (int i = 0; i <= N; i++) net[i] = Lp[i] - cA[i] * kw;
    std::vector<double> Sh(N + 1, 0.0), M(N + 1, 0.0);
    for (int i = N - 1; i >= 0; i--) {
        Sh[i] = Sh[i + 1] + (net[i] + net[i + 1]) / 2 * dy;
        M[i] = M[i + 1] + (Sh[i + 1] + Sh[i]) / 2 * dy;
    }
    const MaterialGrade& mat = materialOf(st.sparMat);
    const double E = mat.young;
    std::vector<double> EI(N + 1);
    for (int i = 0; i <= N; i++) {
        double d = lerp(st.rootDia, st.tipDia, ys[i] / half) / 1000, tt = d / 80;
        EI[i] = E * PI * std::pow(d / 2, 3) * tt;
    }
    std::vector<double> th(N + 1, 0.0), de(N + 1, 0.0);
    for (int i = 1; i <= N; i++) {
        th[i] = th[i - 1] + (M[i - 1] / EI[i - 1] + M[i] / EI[i]) / 2 * dy;
        de[i] = de[i - 1] + (th[i - 1] + th[i]) / 2 * dy;
    }
    double minSF = 1e9, failStation = 0, Mallow = 0;
    std::string failMode = "bend";
    for (int i = 0; i < N; i++) {
        const double u = ys[i] / half;
        const double d = lerp(st.rootDia, st.tipDia, u) / 1000.0, tt = d / 80.0;
        const double I = PI * d * d * d * tt / 8.0;
        double jointK = 1.0;
        for (int k = 1; k < std::max(1, st.segments); k++) {
            const double jy = half * k / (double)std::max(1, st.segments);
            if (std::abs(ys[i] - jy) <= 0.15) jointK = 0.80;
        }
        const double bendStress = std::abs(M[i]) * (d / 2.0) / std::max(1e-12, I);
        const double shearStress = 2.0 * std::abs(Sh[i]) / std::max(1e-12, PI * d * tt);
        const double sfB = mat.compressive * CFRP_DESIGN_K * jointK / std::max(1.0, bendStress);
        const double sfS = mat.shear * CFRP_DESIGN_K * jointK / std::max(1.0, shearStress);
        if (sfB < minSF) {
            minSF = sfB; failStation = u; failMode = "bend";
            Mallow = std::abs(M[i]) * sfB;
        }
        if (sfS < minSF) {
            minSF = sfS; failStation = u; failMode = "shear";
            Mallow = std::abs(M[i]) * sfS;
        }
    }
    // 20分割点の間にある接合部も中心位置で必ず評価する（許容値×0.80）。
    for (int k = 1; k < std::max(1, st.segments); k++) {
        const double y = half * k / (double)std::max(1, st.segments);
        const double q = y / dy;
        const int i0 = std::min(N - 1, std::max(0, (int)std::floor(q)));
        const double f = clamp(q - i0, 0.0, 1.0);
        const double Mi = lerp(M[i0], M[i0 + 1], f), Vi = lerp(Sh[i0], Sh[i0 + 1], f);
        const double u = y / half, d = lerp(st.rootDia, st.tipDia, u) / 1000.0, tt = d / 80.0;
        const double I = PI * d * d * d * tt / 8.0;
        const double bendStress = std::abs(Mi) * (d / 2.0) / std::max(1e-12, I);
        const double shearStress = 2.0 * std::abs(Vi) / std::max(1e-12, PI * d * tt);
        const double sfB = mat.compressive * CFRP_DESIGN_K * 0.80 / std::max(1.0, bendStress);
        const double sfS = mat.shear * CFRP_DESIGN_K * 0.80 / std::max(1.0, shearStress);
        if (sfB < minSF) {
            minSF = sfB; failStation = u; failMode = "bend"; Mallow = std::abs(Mi) * sfB;
        }
        if (sfS < minSF) {
            minSF = sfS; failStation = u; failMode = "shear"; Mallow = std::abs(Mi) * sfS;
        }
    }
    const double baseDih = st.jig == "flat" ? 0 : st.dihedral;
    const double bendDih = th[N] * 180 / PI;
    LoadsResult r;
    r.ys = ys; r.Lp = Lp; r.LpE = LpE; r.M = M; r.defl = de;
    r.tip = de[N]; r.Mroot = M[0]; r.Mallow = Mallow; r.SF = minSF;
    r.failStation = failStation; r.failMode = failMode;
    r.baseDih = baseDih; r.bendDih = bendDih; r.dihEff = baseDih + bendDih;
    return r;
}

double interpA(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs[0]) return ys[0];
    for (size_t i = 1; i < xs.size(); i++)
        if (x <= xs[i]) {
            const double u = (x - xs[i - 1]) / std::max(1e-12, xs[i] - xs[i - 1]);
            return lerp(ys[i - 1], ys[i], u);
        }
    return ys.back();
}

FlexBasis computeFlexBasis(const AircraftParams& st, const Analysis& a) {
    const double g = 9.81, half = st.span / 2;
    const int N = 60;
    const double W1 = a.W * g, wWing = a.items[0].w;
    std::vector<double> ys(N + 1), cA(N + 1), cE(N + 1);
    for (int i = 0; i <= N; i++) {
        const double t = (double)i / N;
        ys[i] = t * half;
        cA[i] = chordAt(st, t);
        cE[i] = (4 * a.S / (PI * st.span)) * std::sqrt(std::max(0.0, 1 - t * t));
    }
    const double dy = half / N;
    auto trap = [&](const std::vector<double>& f) {
        double s = 0;
        for (int i = 0; i < N; i++) s += (f[i] + f[i + 1]) / 2 * dy;
        return s;
    };
    std::vector<double> cS(N + 1);
    for (int i = 0; i <= N; i++) cS[i] = (cA[i] + cE[i]) / 2;
    std::vector<double> Lp(N + 1), wp(N + 1);
    const double kL = W1 / (2 * trap(cS)), kw = wWing * g / (2 * trap(cA));
    for (int i = 0; i <= N; i++) { Lp[i] = cS[i] * kL; wp[i] = cA[i] * kw; }
    const double E = materialOf(st.sparMat).young;
    std::vector<double> EI(N + 1);
    for (int i = 0; i <= N; i++) {
        const double d = lerp(st.rootDia, st.tipDia, ys[i] / half) / 1000, tt = d / 80;
        EI[i] = E * PI * std::pow(d / 2, 3) * tt;
    }
    auto beam = [&](const std::vector<double>& load) {
        std::vector<double> Sh(N + 1, 0.0), M(N + 1, 0.0), th(N + 1, 0.0), de(N + 1, 0.0);
        for (int i = N - 1; i >= 0; i--) {
            Sh[i] = Sh[i + 1] + (load[i] + load[i + 1]) / 2 * dy;
            M[i] = M[i + 1] + (Sh[i + 1] + Sh[i]) / 2 * dy;
        }
        for (int i = 1; i <= N; i++) {
            th[i] = th[i - 1] + (M[i - 1] / EI[i - 1] + M[i] / EI[i]) / 2 * dy;
            de[i] = de[i - 1] + (th[i - 1] + th[i]) / 2 * dy;
        }
        return de;
    };
    FlexBasis fb;
    fb.ys = ys;
    fb.dL = beam(Lp);
    fb.dW = beam(wp);
    return fb;
}

// ---- プロペラBEMT ----
// ブレード形状は直径・ピッチから生成する代表的HPAブレード:
//   弦長 c(x) = R(0.085-0.035x)(緩テーパー、低ソリディティ)
//   取付角 θ(x) = atan(pitch/(2πr))(幾何ピッチ一定)
// 断面はキャンバー付き薄翼: cl = 0.5 + 5.7α、cd0はバルサ0.014/CFRP0.011。
// Prandtl翼端損失込み。誘導速度a, a'は固定点反復(クランプで静止推力域も安定)
PropTables computeBEMT(const AircraftParams& st) {
    thread_local char cacheKey[96] = "";
    thread_local PropTables cache;
    char kb[96];
    std::snprintf(kb, sizeof(kb), "%.3f|%d|%.3f|%s",
                  st.propDia, st.propBlades, st.propPitch, st.propMat.c_str());
    if (std::strcmp(kb, cacheKey) == 0) return cache;
    std::strcpy(cacheKey, kb);

    const int B = std::max(1, st.propBlades);
    const double D = st.propDia, R = D / 2;
    const double pitch = clamp(st.propPitch, 1.0, 12.0);
    const double cd0s = st.propMat == "carbon" ? 0.011 : 0.014;
    const double rhoR = 1.225;              // 係数化で消えるため基準値でよい
    const int NE = 12, NJ = 44;
    const double n0 = 2.0, om = 2 * PI * n0;
    cache.J.assign(NJ, 0); cache.Ct.assign(NJ, 0); cache.Cp.assign(NJ, 0);
    for (int ji = 0; ji < NJ; ji++) {
        const double J = 0.02 + ji * (3.4 - 0.02) / (NJ - 1);
        const double V = J * n0 * D;
        double T = 0, Q = 0;
        for (int ei = 0; ei < NE; ei++) {
            const double x = 0.20 + (ei + 0.5) * (0.96 - 0.20) / NE;
            const double r = x * R, dr = (0.96 - 0.20) / NE * R;
            const double ch = R * (0.085 - 0.035 * x);
            const double thb = std::atan2(pitch, 2 * PI * r);
            // 誘導速度viそのものを反復(静止推力域でも運動量整合が保たれる形)
            double vi = 1.0, ap = 0.01, phi = 0, W2 = 0, cl = 0, cd = 0;
            for (int it = 0; it < 24; it++) {
                const double Ua = std::max(0.0, V + vi), Ut = std::max(0.05, om * r * (1 - ap));
                phi = std::atan2(Ua, Ut);
                W2 = Ua * Ua + Ut * Ut;
                const double al = thb - phi;
                // 断面特性: 線形域は cl=0.5+5.7α。失速後は平板特性へ移行
                // (cl≈0.9·sin2α は残るが cd≈1.5·sin²α で効率が大きく落ちる)。
                // 回転翼の失速遅れ(Himmelskamp): 内翼(c/r大)ほど失速角が伸びる
                const double aStl = ((1.25 - 0.5) / 5.7) * (1 + 1.2 * ch / r);
                const double clStl = 0.5 + 5.7 * aStl;
                if (al > aStl) {
                    const double a2 = std::min(al, PI / 2);
                    cl = std::max(0.9 * std::sin(2 * a2), clStl - 2.0 * (al - aStl));
                    cd = cd0s + 0.02 + 1.5 * std::sin(a2) * std::sin(a2);
                } else {
                    cl = clamp(0.5 + 5.7 * al, -0.5, clStl);
                    cd = cd0s + 0.018 * (cl - 0.5) * (cl - 0.5);
                }
                const double sphi = std::max(0.02, std::sin(phi)), cphi = std::cos(phi);
                const double f = B * 0.5 * (1 - x) / std::max(0.02, x * sphi);
                const double F = std::max(0.05, 2 / PI * std::acos(std::exp(-std::min(20.0, f))));
                const double cn = cl * cphi - cd * sphi, ctg = cl * sphi + cd * cphi;
                // 環状運動量 dT = 4πrρ(V+vi)vi F dr と翼素 dT = ½BρW²c·cn dr の釣合いを
                // viについて解く(2次方程式)。cn<0(風車)ではviが負になる
                const double k = 0.5 * B * W2 * ch * cn / (4 * PI * r * F);
                const double disc = std::max(0.0, V * V + 4 * k);
                const double viN = clamp(0.5 * (-V + std::sqrt(disc)), -0.8 * std::max(V, 0.5), 30.0);
                const double apN = clamp(0.5 * B * W2 * ch * ctg /
                                         (4 * PI * r * r * std::max(0.3, Ua) * om * F), 0.0, 0.5);
                vi += (viN - vi) * 0.4;
                ap += (apN - ap) * 0.4;
            }
            const double cphi = std::cos(phi), sphi = std::sin(phi);
            T += B * 0.5 * rhoR * W2 * ch * (cl * cphi - cd * sphi) * dr;
            Q += B * 0.5 * rhoR * W2 * ch * (cl * sphi + cd * cphi) * r * dr;
        }
        cache.J[ji] = J;
        cache.Ct[ji] = T / (rhoR * n0 * n0 * std::pow(D, 4));
        cache.Cp[ji] = Q * om / (rhoR * n0 * n0 * n0 * std::pow(D, 5));
    }
    return cache;
}

double propThrust(const std::vector<double>& J, const std::vector<double>& Ct,
                  const std::vector<double>& Cp, double D, double rho,
                  double V, double Pshaft, double& nps) {
    if (J.empty()) { nps = 0; return 0; }
    // 吸収パワー P(n) = Cp(V/nD)·ρn³D⁵ は n について単調増加 → 二分法
    // (上限45rev/s=2700rpm: お遊びモードの小型エンジン機まで対応。人力機は<4)
    double lo = 0.15, hi = 45.0;
    auto powAt = [&](double n) {
        const double j = V / std::max(0.05, n * D);
        return interpA(J, Cp, j) * rho * n * n * n * std::pow(D, 5);
    };
    if (powAt(lo) >= Pshaft) { nps = lo; }
    else if (powAt(hi) <= Pshaft) { nps = hi; }
    else {
        for (int i = 0; i < 28; i++) {
            const double mid = (lo + hi) / 2;
            (powAt(mid) < Pshaft ? lo : hi) = mid;
        }
        nps = (lo + hi) / 2;
    }
    const double j = V / std::max(0.05, nps * D);
    return interpA(J, Ct, j) * rho * nps * nps * std::pow(D, 4);
}

double propPowerFor(const std::vector<double>& J, const std::vector<double>& Ct,
                    const std::vector<double>& Cp, double D, double rho,
                    double V, double drag, double inst) {
    // T(P) は P について単調増加 → 二分法で T·inst = drag を解く
    double lo = 1.0, hi = 2000.0, n;
    for (int i = 0; i < 26; i++) {
        const double mid = (lo + hi) / 2;
        (propThrust(J, Ct, Cp, D, rho, V, mid, n) * inst < drag ? lo : hi) = mid;
    }
    return (lo + hi) / 2;
}

PolarResult computePolar(const AircraftParams& st, const Analysis& a, const SimParams& prm) {
    const double rho = 1.225, g = 9.81, W = a.W * g;
    AeroConsts ac = aeroConsts(st);
    const double CLmax = std::min(prm.CLmax, ac.CLmaxWing);
    const double Vs = std::sqrt(2 * W / (rho * a.S * CLmax));
    PolarResult r;
    r.Vs = Vs;
    bool first = true;
    const PropTables& pt = computeBEMT(st);
    const double inst = st.propConfig == "pylon" ? 1.0 : st.propConfig == "midboom" ? 0.97
                      : st.propConfig == "pusher" ? 0.92 : 0.96;
    const AirfoilData& af = airfoilOf(st);
    const double cd0D = afCd0AtRe(af, rho * a.V * a.MAC / 1.81e-5);
    for (double V = Vs; V <= 15; V += 0.1) {
        const double CL = 2 * W / (rho * V * V * a.S);
        // 断面抗力はその速度のReで評価(低速側で「Reの壁」が立ち上がる)
        const double dProf = afCd0AtRe(af, rho * V * a.MAC / 1.81e-5) - cd0D;
        const double CD = ac.CD0 + dProf + CL * CL / (PI * a.AR * ac.e);
        // 必要軸出力: BEMTで推力=抗力となるパワーを解く(釣鐘近似を廃止)
        const double drag = 0.5 * rho * V * V * a.S * CD;
        const double P = propPowerFor(pt.J, pt.Ct, pt.Cp, st.propDia, rho, V, drag, inst);
        const double LD = CL / CD;
        r.pP.push_back({V, P});
        r.pLD.push_back({V, LD});
        if (first || P < r.mpY) { r.mpX = V; r.mpY = P; }
        if (first || LD > r.bgY) { r.bgX = V; r.bgY = LD; }
        first = false;
    }
    const double Vc = a.V;
    r.Dp = 0.5 * rho * Vc * Vc * a.S * ac.CD0;
    r.Di = 0.5 * rho * Vc * Vc * a.S * (1 / (PI * a.AR * ac.e));
    return r;
}

GustResult gustCalc(const AircraftParams& st, const Analysis& a, const SimParams& prm) {
    const double rho = 1.225, g = 9.81, W = a.W * g;
    AeroConsts ac = aeroConsts(st);
    const double Vs = std::sqrt(2 * W / (rho * a.S * std::min(prm.CLmax, ac.CLmaxWing)));
    const double V = a.V;
    const double slope = 2 * PI * a.AR / (a.AR + 2);
    const double dA = std::atan2(prm.gust, V);
    const double n = 1 + 0.5 * rho * V * V * a.S * slope * dA / W;
    // デッキ滑走(10m・下り3.5°)の重力加速込みの前縁飛び出し速度で発進可否を判定
    // (推力・抗力は相殺想定の保守見積り)
    const double pushVa = prm.V0 - prm.wind;
    const double launchVa = std::sqrt(std::max(0.0, pushVa * std::abs(pushVa)
                                       + 2 * 9.81 * std::sin(3.5 * PI / 180) * 10));
    return {Vs, V, (V - Vs) / Vs * 100, launchVa, launchVa >= Vs * 1.05,
            dA * 180 / PI, n, a.sparSF / std::max(0.01, n)};
}

} // namespace bm
