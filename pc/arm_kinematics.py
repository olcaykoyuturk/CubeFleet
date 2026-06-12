"""
Kol kinematigi — GERCEK GEOMETRI (v4'un matematik cekirdegi).

Onceki yaklasimlar goruntu-uzayinda kalibrasyonla ugrasti; v4 kolu fiziksel
modeller: uzuv uzunluklari (cetvelle olculur) + servo degerlerinden
trigonometriyle MIKNATISIN cm konumu (ileri kinematik), kameradan KUPUN cm
konumu (isin-zemin kesisimi) ve hedefe gitmek icin gereken servo degerleri
(ters kinematik) HESAPLANIR.

SERVO != DERECE! Servo komutu 0..180 "birim"dir; fiziksel aciya esit DEGILDIR
(PWM bandi + mekanik aktarma -> eklem basina farkli EGIM ve OFSET). Bu yuzden
her eklemin servo->aci haritasi IKI (veya daha cok) referans noktasindan
DOGRU UYDURMA (linear fit) ile cikarilir:
    aci = k * servo + b
Referans noktasi = kullanicinin GOZLE dogrulayabilecegi fiziksel durus
(orn. "L1 tam DIK" -> alpha=90). Iki nokta egimi de verir; tek nokta varsa
egim *_sign * 1.0 varsayilir (kaba). Noktalar config'te listedir:
    sh_pts/el_pts/gr_pts/base_pts = [[servo, aci], ...]
Ayni servo degerine (±2) ikinci nokta eskisini gunceller; iki noktanin servo
araligi < 15 birim ise fit guvenilmez sayilir (missing() raporlar).

KOORDINATLAR
  Dunya: base ekseni orijin, +y ILERI, +x SAG, +z YUKARI (cm).
  Kol duzlemi (base yaw'i ile doner): r = ileri mesafe, z = yukseklik.
  Omuz mafsali duzlemde S = (R0, H0).

UZUVLAR (cetvelle olculur, cm)
  L1: omuz mafsali -> dirsek mafsali
  L2: dirsek mafsali -> bilek mafsali
  L3: bilek mafsali -> MIKNATIS UCU (buton yuzeyi)
  H0: omuz mafsalinin yerden yuksekligi
  R0: omuz mafsalinin base donme eksenine yatay uzakligi

ACILAR (derece; 0 = yatay-ileri, + = yukari)
  alpha     : L1'in mutlak acisi              (sh_pts: DIK=90, YATAY=0)
  beta_rel  : L2'nin L1'e GORE acisi          (el_pts: KOL DUZ=0,
              L2 YATAY iken = -alpha(o anki s1))
  gamma_rel : L3'un L2'ye GORE acisi          (gr_pts: MIKNATIS ASAGI iken
              = -90 - beta_abs, MIKNATIS ILERI iken = -beta_abs)
  yaw       : base donusu (0 = tam ileri, + = SAG; base_pts: ILERI=0, SAG=90)
  Miknatis tam asagi bakar <=> gamma = beta + gamma_rel = -90.

KAMERA (bilege/L3'e rijit):
  cam_dx: L3 ekseni boyunca bilekten ileri ofset (cm)
  cam_dz: L3 eksenine dik, "link yukarisi" ofset (cm)
  cam_pitch: optik eksenin L3 eksenine gore acisi (derece, + yukari)
  cam_f: odak (piksel) — OV3660 HVGA icin ~370; tek olcumle kalibre edilir
  Goruntu: u saga, v asagi artar; merkez (cam_cx, cam_cy).
"""

from __future__ import annotations

import math
from typing import Optional

D2R = math.pi / 180.0


DEFAULTS = {
    "cube_cm": 4.0,
    "L1": 10.0, "L2": 10.0, "L3": 6.0, "H0": 7.0, "R0": 2.0,
    # servo->aci referans noktalari [[servo, aci], ...] — UI 📐 butonlari doldurur
    "sh_pts": [], "el_pts": [], "gr_pts": [], "base_pts": [],
    # PARALELKENAR KOL: dirsek servosu on kolun acisini L1'e gore DEGIL,
    # YERE GORE (mutlak) kontrol ediyorsa True (test: yalniz shoulder oynat —
    # on kol yere gore egimini KORUYORSA paralelkenar). el_pts o zaman
    # [servo, beta_ABS] tutar.
    "el_abs": False,
    # tek-nokta fallback egim isaretleri (kullanici yon semantigi:
    # BASE 0=SOL, SHOULDER 0=ILERI, ELBOW 0=ILERI, GRIPPER 0=GERI)
    "sh_sign": 1, "el_sign": 1, "gr_sign": 1, "base_sign": 1,
    "cam_f": 370.0, "cam_cx": 240.0, "cam_cy": 160.0,
    "cam_dx": 2.0, "cam_dz": 2.5, "cam_pitch": 0.0,
}

_JOINT_LABEL = {"sh": "L1 (omuz)", "el": "dirsek", "gr": "bilek", "base": "base"}
MIN_SPREAD = 15.0    # iki referansin servo araligi en az bu kadar olmali


def fit_line(pts, fallback_slope=1.0):
    """[[servo, aci], ...] -> (k, b) dogru uydurma (aci = k*servo + b).
    0 nokta -> None; 1 nokta -> egim fallback_slope; 2+ -> en kucuk kareler.
    Noktalar servo ekseninde MIN_SPREAD'den dar kumelenmisse tek nokta sayilir."""
    clean = []
    for p in pts or []:
        try:
            clean.append((float(p[0]), float(p[1])))
        except Exception:
            continue
    if not clean:
        return None
    smin = min(p[0] for p in clean)
    smax = max(p[0] for p in clean)
    if len(clean) == 1 or (smax - smin) < MIN_SPREAD:
        s, a = clean[0]
        return (float(fallback_slope), a - float(fallback_slope) * s)
    n = len(clean)
    ms = sum(p[0] for p in clean) / n
    ma = sum(p[1] for p in clean) / n
    den = sum((p[0] - ms) ** 2 for p in clean)
    k = sum((p[0] - ms) * (p[1] - ma) for p in clean) / den
    if abs(k) < 0.2:    # fiziksel olarak anlamsiz egim — kotu referanslar
        return None
    return (k, ma - k * ms)


def point_from_height(model: "ArmModel", joint: str, servos, z_cm: float):
    """CETVELLE OLCULEN YUKSEKLIKTEN referans noktasi uret — "kol tam yatay/dik
    olmuyor" problemi icin: kol HERHANGI bir durusta dururken ilgili mafsalin
    yerden yuksekligi olculur, aci geometriden cikar (sin(aci) = dz / L).

    joint: "sh" -> DIRSEK mafsali yuksekligi  -> [s1, alpha] noktasi
           "el" -> BILEK mafsali yuksekligi   -> [s2, beta_rel] (L1 fiti gerekli)
           "gr" -> MIKNATIS UCU yuksekligi    -> [s3, gamma_rel] (L1+dirsek fiti)
    VARSAYIM: ilgili uzuv ILERI dogru bakar (geri katlanmis degil).
    Donus: (servo, aci) basari; str = hata mesaji."""
    s1, s2, s3 = float(servos[1]), float(servos[2]), float(servos[3])
    H0, L1 = model._f("H0"), model._f("L1")
    L2, L3 = model._f("L2"), model._f("L3")
    if H0 <= 0:
        return "önce H0 ölçüsünü gir (omuz mafsalının yerden yüksekliği)"

    def _asin(dz, L):
        s = dz / L if L > 0 else 99.0
        if abs(s) > 1.03:
            return None
        return math.degrees(math.asin(max(-1.0, min(1.0, s))))

    if joint == "sh":
        a = _asin(z_cm - H0, L1)
        if a is None:
            return f"yükseklik L1 ile uyumsuz: |{z_cm:g}−{H0:g}| > {L1:g}"
        return (s1, round(a, 2))
    if model.maps["sh"] is None:
        return "önce L1 (omuz) referanslarını tamamla"
    alpha = model.alpha(s1)
    zE = H0 + L1 * math.sin(alpha * D2R)
    if joint == "el":
        b = _asin(z_cm - zE, L2)
        if b is None:
            return f"yükseklik L2 ile uyumsuz (dirsek {zE:.1f} cm'de)"
        # paralelkenar kolda el_pts MUTLAK aci tutar, klasikte L1'e gore bagil
        return (s2, round(b if model.c.get("el_abs") else b - alpha, 2))
    if model.maps["el"] is None:
        return "önce dirsek referanslarını tamamla"
    beta = model.beta_abs(s1, s2)
    zW = zE + L2 * math.sin(beta * D2R)
    g = _asin(z_cm - zW, L3)
    if g is None:
        return f"yükseklik L3 ile uyumsuz (bilek {zW:.1f} cm'de)"
    return (s3, round(g - beta, 2))


def _nelder_mead(f, x0, step, iters=400):
    """Bagimsiz Nelder-Mead (scipy YOK) — kucuk param sayisi icin yeterli.
    f: maliyet; x0: baslangic; step: ilk simplex adimlari. Donus: (en_iyi, maliyet)."""
    n = len(x0)
    simplex = [list(x0)]
    for i in range(n):
        p = list(x0)
        p[i] += step[i] if step[i] else 1.0
        simplex.append(p)
    fv = [f(p) for p in simplex]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: fv[i])
        simplex = [simplex[i] for i in order]
        fv = [fv[i] for i in order]
        cen = [sum(simplex[i][j] for i in range(n)) / n for j in range(n)]
        xr = [cen[j] + (cen[j] - simplex[-1][j]) for j in range(n)]
        fr = f(xr)
        if fv[0] <= fr < fv[-2]:
            simplex[-1], fv[-1] = xr, fr
            continue
        if fr < fv[0]:
            xe = [cen[j] + 2.0 * (cen[j] - simplex[-1][j]) for j in range(n)]
            fe = f(xe)
            simplex[-1], fv[-1] = (xe, fe) if fe < fr else (xr, fr)
            continue
        xc = [cen[j] + 0.5 * (simplex[-1][j] - cen[j]) for j in range(n)]
        fc = f(xc)
        if fc < fv[-1]:
            simplex[-1], fv[-1] = xc, fc
            continue
        for i in range(1, n + 1):
            simplex[i] = [simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j])
                          for j in range(n)]
            fv[i] = f(simplex[i])
    order = sorted(range(n + 1), key=lambda i: fv[i])
    return simplex[order[0]], fv[order[0]]


def calibrate_camera(cfg, samples, free=("cam_pitch", "cam_f", "cam_dz")):
    """Kamera parametrelerini OLCULMUS ornekkerden coz — kameranin bilege gore
    egimi/konumu/odagi elle bilinemez, ama kup BILINEN dunya (x, y)'lerine
    konup pikseli kaydedilerek geri cozulur.
      samples: [(servos, u, v, x_cm, y_cm), ...]
      free: optimize edilecek param adlari (ornek sayisina gore kisalir)
    cube_from_pixel tahmini olcumle eslesene dek Nelder-Mead (back-projection
    hatasi). Donus: (yeni_param_sozlugu, rms_cm) | (None, hata_mesaji)."""
    if len(samples) < 2:
        return None, "en az 2 örnek gerekli"
    free = list(free)
    # ornek azsa serbest param sayisini kis (overfit/gozlemlenebilirlik)
    if len(samples) < 4 and "cam_dz" in free:
        free.remove("cam_dz")
    if len(samples) < 3 and "cam_f" in free:
        free.remove("cam_f")
    base = dict(cfg)
    cube = float(base.get("cube_cm") or 4.0)
    zp = cube / 2.0

    def cost(vec):
        c = dict(base)
        for k, val in zip(free, vec):
            c[k] = val
        m = ArmModel(c)
        s = 0.0
        for (servos, u, v, x, y) in samples:
            est = m.cube_from_pixel(servos, u, v, zp)
            if est is None:
                s += 1e4
            else:
                s += (est[0] - x) ** 2 + (est[1] - y) ** 2
        return s / len(samples)

    x0 = [float(base.get(k) or 0.0) for k in free]
    steps = [5.0 if k == "cam_pitch" else (30.0 if k == "cam_f" else 1.5)
             for k in free]
    best, c = _nelder_mead(cost, x0, steps, iters=600)
    out = dict(base)
    for k, val in zip(free, best):
        out[k] = round(val, 3)
    return out, round(c ** 0.5, 2)


class ArmModel:
    """Tum kinematik hesaplar: FK, IK, kamera isini, izdusum."""

    def __init__(self, cfg: Optional[dict] = None):
        self.c = {**DEFAULTS, **(cfg or {})}
        self.maps = {j: fit_line(self.pts(j), self._sgn(j))
                     for j in ("sh", "el", "gr", "base")}

    def pts(self, j: str) -> list:
        v = self.c.get(j + "_pts")
        return v if isinstance(v, list) else []

    def _sgn(self, j: str) -> float:
        try:
            return float(self.c.get(j + "_sign") or 1)   # type: ignore[arg-type]
        except Exception:
            return 1.0

    def _f(self, key: str, default: float = 0.0) -> float:
        """Config degerini float olarak oku (tip-guvenli)."""
        v = self.c.get(key)
        try:
            return default if v is None else float(v)    # type: ignore[arg-type]
        except Exception:
            return default

    # ------------------------------------------------------------- hazirlik
    def missing(self) -> list:
        """Eksik kalibrasyonlarin listesi (bos = model kullanima hazir).
        sh/el/gr icin EGIM de gerekli -> en az 2 ayrik referans istenir."""
        out = []
        for j in ("sh", "el", "gr"):
            pts = self.pts(j)
            spread = (max(float(p[0]) for p in pts)
                      - min(float(p[0]) for p in pts)) if len(pts) >= 2 else 0.0
            if self.maps[j] is None:
                out.append(f"{_JOINT_LABEL[j]} referansı")
            elif len(pts) < 2 or spread < MIN_SPREAD:
                out.append(f"{_JOINT_LABEL[j]} 2. referans (eğim için)")
        if self.maps["base"] is None:
            out.append(f"{_JOINT_LABEL['base']} referansı")
        for k in ("L1", "L2", "L3", "H0"):
            try:
                ok = float(self.c.get(k) or 0) > 0   # type: ignore[arg-type]
            except Exception:
                ok = False
            if not ok:
                out.append(k + " ölçüsü")
        return out

    # ------------------------------------------------------- servo <-> aci
    def _ang(self, j: str, servo: float) -> float:
        m = self.maps[j]
        if m is None:
            raise ValueError(f"{j} kalibre değil")
        return m[0] * servo + m[1]

    def _srv(self, j: str, ang: float) -> float:
        m = self.maps[j]
        if m is None:
            raise ValueError(f"{j} kalibre değil")
        return (ang - m[1]) / m[0]

    def alpha(self, s1):
        return self._ang("sh", s1)

    def beta_abs(self, s1, s2):
        """On kolun MUTLAK acisi: paralelkenar kolda dogrudan dirsek servosundan
        (L1'den bagimsiz), klasik kolda alpha + bagil dirsek acisi."""
        if self.c.get("el_abs"):
            return self._ang("el", s2)
        return self.alpha(s1) + self._ang("el", s2)

    def gamma_rel(self, s3):
        return self._ang("gr", s3)

    def yaw(self, s0):
        return self._ang("base", s0)

    # ------------------------------------------------------------------ FK
    def fk(self, servos) -> dict:
        """Servo degerlerinden duzlem geometrisi (cm) + kamera pozu."""
        s0, s1, s2, s3 = [float(v) for v in servos[:4]]
        L1, L2, L3 = self._f("L1"), self._f("L2"), self._f("L3")
        a = self.alpha(s1)
        b = self.beta_abs(s1, s2)
        g = b + self.gamma_rel(s3)
        ar, br, gr_ = a * D2R, b * D2R, g * D2R
        S = (self._f("R0"), self._f("H0"))
        E = (S[0] + L1 * math.cos(ar), S[1] + L1 * math.sin(ar))
        W = (E[0] + L2 * math.cos(br), E[1] + L2 * math.sin(br))
        M = (W[0] + L3 * math.cos(gr_), W[1] + L3 * math.sin(gr_))
        lx, lz = (math.cos(gr_), math.sin(gr_)), (-math.sin(gr_), math.cos(gr_))
        cdx, cdz = self._f("cam_dx"), self._f("cam_dz")
        C = (W[0] + cdx * lx[0] + cdz * lz[0],
             W[1] + cdx * lx[1] + cdz * lz[1])
        return {"yaw": self.yaw(s0), "alpha": a, "beta": b, "gamma": g,
                "S": S, "E": E, "W": W, "M": M, "C": C,
                "phi": g + self._f("cam_pitch")}

    def magnet_world(self, servos):
        """Miknatis ucunun dunya konumu (x, y, z)."""
        f = self.fk(servos)
        yw = f["yaw"] * D2R
        r, z = f["M"]
        return (r * math.sin(yw), r * math.cos(yw), z)

    # ------------------------------------------------- kamera: isin & izdusum
    def cube_from_pixel(self, servos, u: float, v: float,
                        z_plane: float) -> Optional[tuple]:
        """Piksel (u, v)'deki nesnenin DUNYA (x, y) konumu — kamera isininin
        z = z_plane duzlemiyle kesisimi. bbox boyutuna GEREK YOK."""
        f = self.fk(servos)
        phi = f["phi"] * D2R
        ax = (math.cos(phi), math.sin(phi))
        dn = (math.sin(phi), -math.cos(phi))
        cf = self._f("cam_f", 370.0)
        ky = (v - self._f("cam_cy")) / cf
        kx = (u - self._f("cam_cx")) / cf
        d_r = ax[0] + ky * dn[0]
        d_z = ax[1] + ky * dn[1]
        Cr, Cz = f["C"]
        if d_z >= -1e-6:
            return None
        t = (z_plane - Cz) / d_z
        if t <= 0:
            return None
        r_hit = Cr + t * d_r
        lat = t * kx
        yw = f["yaw"] * D2R
        x = r_hit * math.sin(yw) + lat * math.cos(yw)
        y = r_hit * math.cos(yw) - lat * math.sin(yw)
        return x, y

    def project_point(self, servos, x: float, y: float,
                      z: float) -> Optional[tuple]:
        """Dunya noktasinin goruntudeki pikseli (u, v) — overlay/dogrulama."""
        f = self.fk(servos)
        yw = f["yaw"] * D2R
        rp = x * math.sin(yw) + y * math.cos(yw)
        lat = x * math.cos(yw) - y * math.sin(yw)
        phi = f["phi"] * D2R
        ax = (math.cos(phi), math.sin(phi))
        dn = (math.sin(phi), -math.cos(phi))
        Cr, Cz = f["C"]
        dr, dz = rp - Cr, z - Cz
        depth = dr * ax[0] + dz * ax[1]
        if depth <= 1e-6:
            return None
        down = dr * dn[0] + dz * dn[1]
        cf = self._f("cam_f", 370.0)
        u = self._f("cam_cx") + cf * lat / depth
        v = self._f("cam_cy") + cf * down / depth
        return u, v

    # ------------------------------------------------------------------ IK
    def ik(self, x: float, y: float, tip_z: float,
           current=None) -> Optional[list]:
        """Miknatis ucu (x, y, tip_z)'de ve TAM ASAGI bakacak sekilde
        [s0, s1, s2, s3]. Iki dirsek cozumunden 0..180 servo bandina dusenler
        arasindan mevcuda en yakini secilir. Erisim disi -> None."""
        yaw = math.degrees(math.atan2(x, y))
        rt = math.hypot(x, y)
        Wr, Wz = rt, tip_z + self._f("L3")
        dr, dz = Wr - self._f("R0"), Wz - self._f("H0")
        L1, L2 = self._f("L1"), self._f("L2")
        d2 = dr * dr + dz * dz
        c2 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if not -1.0 <= c2 <= 1.0:
            return None
        sols = []
        for q2 in (math.acos(c2), -math.acos(c2)):
            a = math.atan2(dz, dr) - math.atan2(L2 * math.sin(q2),
                                                L1 + L2 * math.cos(q2))
            a_deg = math.degrees(a)
            b_rel = math.degrees(q2)
            b_abs = a_deg + b_rel
            g_rel = -90.0 - b_abs
            try:
                servos = [self._srv("base", yaw), self._srv("sh", a_deg),
                          self._srv("el", b_abs if self.c.get("el_abs") else b_rel),
                          self._srv("gr", g_rel)]
            except Exception:
                return None
            if all(-0.5 <= s <= 180.5 for s in servos):
                sols.append([min(180, max(0, round(s))) for s in servos])
        if not sols:
            return None
        if current is None or len(sols) == 1:
            return sols[0]
        return min(sols, key=lambda sv: sum(abs(sv[i] - current[i])
                                            for i in range(4)))
