#!/usr/bin/env python3
"""Воспроизводимый наивный baseline: колёсная одометрия без модели, EKF, логики юза и TRN.

Назначение
----------
Точка отсчёта для доказательной базы: что получает аккуратный инженер, который
интегрирует скорость колёс и кладёт пройденный путь на тот же pathgraph, но не
использует тяговую модель (Hammerstein), EKF [v, b, g_tr, g_br, s], детектор
юза/буксования, rate limiter, правила stuck-zero/frozen/spike и уточнение
положения по профилю уклона (TRN).  Заменяет ``results/B0_naive`` (снят со
скелетной версии кода и больше не воспроизводится).

Всё, что НЕ относится к оценке скорости/пути, взято из пакета без изменений,
чтобы разница с ``F3_final`` объяснялась только оценкой скорости и пути:

* порядок сообщений, заголовочные штампы, окно GNSS (``gnss_limit_s``) -- как в
  ``tram_backup_odometry.replay.run_bag`` (используются ``merged_events``,
  ``load_data``, ``default_maps_dir``);
* санация штампов ``core.preprocess.TimeGuard`` (не модель: только консенсус
  заголовочных часов потоков; late-сообщения не подаются в оценку, но на них
  отвечают, как в ``Pipeline``);
* начальная привязка ``core.initializer.Initializer`` (те же правила выбора
  маршрута, G1-мост, sigma0), геометрия ``core.track_map.RunPath`` (мост s<0,
  карта 0..L, хвост 'decay' s>L), выходная СК ``core.frames.OutputFrame``,
  z = z карты + ``output_z_offset_m``, ``output_along_offset_m``;
* контракт публикации: один выход на каждое сообщение колеса/команды со штампом
  этого сообщения, подавление повторных штампов (``pipeline.STAMP_MEMORY``),
  ``publish_on``; без GNSS -- счисление в СК старта ('no_gnss').

Вариант 'naive' (по умолчанию, ``run_bag``)
------------------------------------------
Колесо b: принятый отсчёт v_b = max(raw, 0) / K, K = ``Params.k_for_vehicle()``
(30618: 3.597, 30639: 3.59, иначе 3.595; сырые данные в км/ч).  Отсчёт
отбрасывается, если он не конечен, вне [``wheel_min_kmh``, ``wheel_max_kmh``]
или старше предыдущего отсчёта того же колеса.  Нулевой порядок удержания:
значение колеса действует от момента приёма до следующего отсчёта; колесо
недействительно, если его последний отсчёт старше ``wheel_stale_s`` (0.5 с).

    v(t) = среднее действительных колёс;  нет ни одного -> последнее значение v
           (удержание без ограничения по времени);
    s(t) = интеграл v(t) dt  (кусочно-постоянный, точные моменты устаревания
           колёс учтены как точки излома).

Новый отсчёт действует с max(штамп, текущее время интегратора): штампы колёс
отстают от штампов команд на ~0.05 с, ретроспективной коррекции нет.  На
выходном штампе: v = текущее v (без экстраполяции), s_odo = s + v * dt, dt =
штамп - время интегратора, |dt| <= ``extrap_max_s``.  Позиция: s_route =
s_offset + s_odo + ``output_along_offset_m`` на ``RunPath`` (без поправки TRN).
Ковариация: var_along = sigma0^2 + (``odo_scale_sigma`` * s_odo)^2 плюс те же
надбавки вне карты, что в ``Pipeline._output`` (``tail_sigma_frac``,
``tail_cross_frac``, ``bridge_cross_frac``, ``offmap_yaw_rad_per_m``);
var_cross = ``pose_sigma_cross_m``^2 (+ надбавки); var_v = ``meas_sigma``^2
(оба колеса), ``meas_sigma_single``^2 (одно), при удержании растёт как
(``meas_sigma_single`` + ``acc_max`` * возраст)^2.  Флаг юза всегда False.

Вариант 'kf' (``run_bag_kf``; безмодельный кинематический фильтр Калмана)
--------------------------------------------------------------------------
Проверка «хватает ли обычного фильтра без тяговой модели».  Состояние
[s, v, a], модель постоянного ускорения с белым рывком (спектральная плотность
q_j^2, ``baseline_kf_jerk``, по умолчанию 0.3 м/с^3/sqrt(Гц); выбран по
score_loss на всех 60 eval-мешках из {0.03, 0.1, 0.3, 1, 3, 10}, т.е. настроен
в пользу baseline, без кросс-валидации).  Каждый принятый
отсчёт каждого колеса -- измерение скорости в момент его штампа:
h = [0, 1, -(t_filter - t_b)], R = ``meas_sigma``^2; нет ни проверки
согласия колёс, ни юза, ни ограничителя.  Если оба колеса устарели, a := 0
(удержание v).  Выход: экстраполяция [s, v, a] к штампу (|dt| <=
``extrap_max_s``, v >= 0).  Прочее -- как у 'naive'.

Результаты (60 eval-мешков, эталон master, окно GNSS 10 с; медиана / среднее)
------------------------------------------------------------------------------
                  B1_naive        B1_naive_kf     AF_trn_off      F3_final
    v_rmse, м/с   0.0541/0.0728   0.0296/0.0527   0.0295/0.0511   0.0295/0.0511
    v_bias_trans  0.0501/0.0640   0.0121/0.0230   0.0121/0.0229   0.0121/0.0229
    al_rmse, м    2.81 / 5.91     3.13 / 5.97     3.08 / 5.91     3.03 / 4.03
    al_max, м     10.2 / 14.9     10.9 / 15.1     9.94 / 14.9     9.51 / 12.7
    e3d_mean, м   5.23 / 8.57     5.14 / 8.63     5.17 / 8.59     4.74 / 6.73
    drift_pct     0.471 / 1.17    0.473 / 1.17    0.473 / 1.17    0.318 / 1.10
    score_loss    2.00 / 2.13     1.61 / 1.70     1.47 / 1.68     1.25 / 1.49
Скорость: основной выигрыш над 'naive' -- компенсация запаздывания (штамп
команды впереди штампа колеса на ~0.05 с, экстраполяция), его даёт уже 'kf';
модель и логика юза видны в хвосте (p90 v_rmse 0.093 -> 0.083) и в событиях
юза: max|dv| naive 1.1-2.6 м/с против F3 0.16-0.8 м/с (2050d396, 33bec73f,
50956d6e, 68d1748a).  Положение: без TRN (AF_trn_off) модель не меняет
положение относительно 'naive'; TRN снижает среднее/p90 al_rmse (5.9 -> 4.0 /
12.7 -> 8.1 м) за счёт мешков с плохой начальной привязкой, но на медианном
мешке паритет (F3 лучше в 30 из 60).  Выпадения одного колеса 30639
(927002c2, d927f360, 584b6e32, 4285f2bc, 9f0b519f, 3b3d9eb8) 'naive' проходит
за счёт правила устаревания 0.5 с (путь в окне выпадения в пределах 2.5 % от
доплеровского).  Скрипты разбора: .work/scratch_deliv/b1_*.py.

Использование
-------------
    PY=python3
    $PY tools/run_cv.py --tag B1_naive    --bags all --antenna master --jobs 4 --estimator baseline_naive:run_bag
    $PY tools/run_cv.py --tag B1_naive_kf --bags all --antenna master --jobs 4 --estimator baseline_naive:run_bag_kf
    $PY tools/run_cv.py --compare B1_naive F3_final
    $PY tools/baseline_naive.py 30618_0e41eac3 [--variant kf]      # один мешок + метрики

``run_bag(bag, params=None, data=None, gnss_limit_s=10.0, **kw)`` возвращает тот
же словарь массивов, что ``tram_backup_odometry.replay.run_bag`` (t, v, a, x, y,
z, yaw, has_pose, var_v, var_along, var_cross, var_yaw, slip, mode, cpu_s,
n_out, diag_*).  Класс ``Pipeline`` модуля имеет интерфейс on_wheel/on_cmd/on_fix
пакета, поэтому ``run_cv`` измеряет и его задержку на вызов.
"""
import argparse
import copy
import math
import os
import sys
import time
from collections import deque

import numpy as np

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(os.path.dirname(_TOOLS), 'src', 'tram_backup_odometry')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from tram_backup_odometry.core.config import Params  # noqa: E402
from tram_backup_odometry.core.frames import OutputFrame  # noqa: E402
from tram_backup_odometry.core.initializer import Initializer  # noqa: E402
from tram_backup_odometry.core.pipeline import Output, STAMP_MEMORY  # noqa: E402
from tram_backup_odometry.core.preprocess import TimeGuard, FLAG_INVALID, FLAG_LATE  # noqa: E402
from tram_backup_odometry.core.track_map import RunPath, load_routes  # noqa: E402
from tram_backup_odometry.replay import merged_events, load_data, default_maps_dir  # noqa: E402

BOGIES = ('front', 'rear')
KF_JERK_DEFAULT = 0.3       # m/s^3/sqrt(Hz), in-sample best of {0.03..10} (module docstring)


class NaivePipeline:
    """Same interface as core.pipeline.Pipeline; speed/distance from plain wheel averaging."""

    def __init__(self, params: Params, maps_dir: str, variant: str = 'naive'):
        p = params.validate()
        if variant not in ('naive', 'kf'):
            raise ValueError('variant must be naive|kf')
        self.p = p
        self.variant = variant
        self.routes = load_routes(maps_dir, p.grade_window_m)
        self.frame = OutputFrame(p)
        self.k = p.k_for_vehicle()
        self.tg = TimeGuard(p)
        self.init = Initializer(p, self.routes)
        self.stale_s = float(p.wheel_stale_s)
        self.extrap = float(p.extrap_max_s)
        # per bogie: [t_last, v_last] of the last accepted sample
        self.bog = {b: [None, 0.0] for b in BOGIES}
        self.started = False
        self.t = None           # integrator clock (monotonic effective time)
        self.s = 0.0            # odometer at self.t
        self.v = 0.0            # naive: piecewise-constant speed valid from self.t on
        self.v_hold = 0.0
        self.state = 'model_only'
        # kf variant: x = [s, v, a], P 3x3
        self.q_j = float(getattr(p, 'baseline_kf_jerk', KF_JERK_DEFAULT))
        self.x = [0.0, 0.0, 0.0]
        self.P = [[0.0] * 3 for _ in range(3)]
        self.init_res = None
        self.route = None
        self.path = None
        self.direction = ''
        self.s_offset = 0.0
        self.sigma0 = 0.0
        self.odo_init = 0.0
        self.fallback = False
        self._stamps = deque(maxlen=STAMP_MEMORY)
        self._stamp_set = set()
        self.n_late = 0
        self.n_out = 0

    # ------------------------------------------------------------ properties
    @property
    def init_done(self):
        return self.init.done or self.fallback

    # ------------------------------------------------------------ wheel state
    def _valid(self, b, tau):
        t_b = self.bog[b][0]
        return t_b is not None and tau - t_b < self.stale_s

    def _fused(self, tau):
        vals = [self.bog[b][1] for b in BOGIES if self._valid(b, tau)]
        if len(vals) == 2:
            self.state = 'both'
        elif vals:
            self.state = 'single_' + ('front' if self._valid('front', tau) else 'rear')
        else:
            self.state = 'hold'
            return self.v_hold
        v = sum(vals) / len(vals)
        self.v_hold = v
        return v

    def _add_sample(self, b, t, raw):
        """Store a raw sample (km/h); False if rejected (non-finite, out of range, out of order)."""
        try:
            raw = float(raw)
        except (TypeError, ValueError):
            return False
        p = self.p
        if not math.isfinite(raw) or raw > p.wheel_max_kmh or raw < p.wheel_min_kmh:
            return False
        st = self.bog.get(b)
        if st is None or (st[0] is not None and t < st[0]):
            return False
        st[0] = t
        st[1] = raw / self.k if raw > 0.0 else 0.0
        return True

    # ------------------------------------------------------------ integration
    def _advance(self, t):
        """Integrate the odometer from self.t to t (no-op for t <= self.t)."""
        if t <= self.t:
            return
        if self.variant == 'kf':
            self._kf_predict(t)
            return
        # piecewise-constant speed; a bogie turning stale inside (self.t, t) is a breakpoint
        while self.t < t:
            t_next = t
            for b in BOGIES:
                t_b = self.bog[b][0]
                if t_b is not None:
                    ts = t_b + self.stale_s
                    if self.t < ts < t_next:
                        t_next = ts
            self.s += self.v * (t_next - self.t)
            self.t = t_next
            self.v = self._fused(self.t)

    # ------------------------------------------------------------ kf variant
    def _kf_reset(self, t, v):
        self.t = t
        self.x = [0.0, v, 0.0]
        self.P = [[0.0, 0.0, 0.0], [0.0, self.p.meas_sigma ** 2, 0.0], [0.0, 0.0, 1.0]]

    def _kf_predict(self, t):
        dt = t - self.t
        x, P = self.x, self.P
        if not (self._valid('front', t) or self._valid('rear', t)):
            x[2] = 0.0                       # no wheel at all: hold the speed
            self.state = 'hold'
        s, v, a = x
        v1 = v + a * dt
        if v1 < 0.0:                         # stop inside the step: exact stop distance
            tz = -v / a if a < 0.0 else 0.0
            s += 0.5 * v * tz
            x[:] = [s, 0.0, 0.0]
        else:
            x[:] = [s + v * dt + 0.5 * a * dt * dt, v1, a]
        F = ((1.0, dt, 0.5 * dt * dt), (0.0, 1.0, dt), (0.0, 0.0, 1.0))
        FP = [[sum(F[i][k] * P[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        FPF = [[sum(FP[i][k] * F[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        q = self.q_j ** 2
        d2, d3, d4, d5 = dt * dt, dt ** 3, dt ** 4, dt ** 5
        Q = ((d5 / 20, d4 / 8, d3 / 6), (d4 / 8, d3 / 3, d2 / 2), (d3 / 6, d2 / 2, dt))
        self.P = [[FPF[i][j] + q * Q[i][j] for j in range(3)] for i in range(3)]
        self.t = t

    def _kf_update(self, z, t_b):
        """Speed measurement z taken at t_b <= self.t: h = [0, 1, -(self.t - t_b)]."""
        lag = self.t - t_b
        if lag < 0.0:
            lag = 0.0
        if lag > self.extrap:
            lag = self.extrap
        h = (0.0, 1.0, -lag)
        x, P = self.x, self.P
        Ph = [sum(P[i][k] * h[k] for k in range(3)) for i in range(3)]
        S = sum(h[i] * Ph[i] for i in range(3)) + self.p.meas_sigma ** 2
        y = z - (x[1] - lag * x[2])
        K = [Ph[i] / S for i in range(3)]
        for i in range(3):
            x[i] += K[i] * y
        self.P = [[P[i][j] - K[i] * Ph[j] for j in range(3)] for i in range(3)]
        if x[1] < 0.0:
            x[1] = 0.0

    # ------------------------------------------------------------ odometer at a stamp
    def _v_s_at(self, stamp):
        dt = stamp - self.t
        if dt > self.extrap:
            dt = self.extrap
        elif dt < -self.extrap:
            dt = -self.extrap
        if self.variant == 'kf':
            s, v, a = self.x
            ve = v + a * dt
            if ve >= 0.0:
                return ve, s + 0.5 * (v + ve) * dt
            tz = -v / a
            return 0.0, s + 0.5 * v * tz
        return self.v, self.s + self.v * dt

    def _odo_at(self, t):
        if not self.started:
            return 0.0
        return self._v_s_at(t)[1]

    # ------------------------------------------------------------ inputs
    def on_wheel(self, bogie, t_hdr, raw_kmh):
        t, flag = self.tg.correct(bogie, t_hdr)
        if flag == FLAG_INVALID:
            return None
        if flag == FLAG_LATE:
            self.n_late += 1            # not fed, but still answered
        else:
            if not self.started:
                if not self._add_sample(bogie, t, raw_kmh):
                    return None
                v0 = self.bog[bogie][1]
                self.t, self.s, self.v, self.v_hold = t, 0.0, v0, v0
                self.state = 'single_' + bogie
                if self.variant == 'kf':
                    self._kf_reset(t, v0)
                self.started = True
            else:
                self._advance(t)
                if self._add_sample(bogie, t, raw_kmh):
                    if self.variant == 'kf':
                        self._kf_update(self.bog[bogie][1], t)
                        self._fused(self.t)           # only for the state label
                    else:
                        self.v = self._fused(self.t)  # new value effective from the integrator clock
        if not self.started:
            return None
        self._check_init(t)
        if self.p.publish_on == 'cmd':
            return None
        return self._output(t_hdr)

    def on_cmd(self, t_hdr, notch):
        t, flag = self.tg.correct('cmd', t_hdr)
        if flag == FLAG_INVALID:
            return None
        try:
            int(notch)                  # the notch itself is not used; same validity rule as Pipeline
        except (TypeError, ValueError):
            return None
        if flag == FLAG_LATE:
            self.n_late += 1
        if not self.started:
            return None
        if flag != FLAG_LATE:
            self._advance(t)
        self._check_init(t)
        if self.p.publish_on == 'wheels':
            return None
        return self._output(t_hdr)

    def on_fix(self, antenna, t_hdr, lat, lon, alt, status):
        if self.init_done:
            return None
        try:
            t_hdr = float(t_hdr)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(t_hdr):
            return None
        self.init.add_fix(antenna, t_hdr, lat, lon, alt, status, self._odo_at(t_hdr))
        res = self.init.result()
        if res is not None and res is not self.init_res and res.kind != 'none':
            self._apply_init(res)
        return None

    # ------------------------------------------------------------ alignment (as Pipeline, no TRN)
    def _check_init(self, t):
        if self.init_done:
            return
        if self.init.maybe_finalize(t):
            res = self.init.result()
            if res is None or res.kind == 'none':
                self._start_fallback()
            else:
                self._apply_init(res)

    def _apply_init(self, res):
        p = self.p
        self.init_res = res
        self.direction = res.direction
        self.route = self.routes[res.direction]
        self.path = RunPath(self.route, res.head, p.tail_mode, p.tail_curv_decay_m, p.tail_curv_window_m)
        self.s_offset = res.s_offset
        self.sigma0 = res.sigma0
        self.odo_init = self._odo_now()
        x0, y0, z0 = res.start_xyz
        if not math.isfinite(z0):
            z0 = 0.0
        self.frame.set_origin(x0, y0, z0)

    def _odo_now(self):
        if not self.started:
            return 0.0
        return self.x[0] if self.variant == 'kf' else self.s

    def _start_fallback(self):
        if self.fallback:
            return
        self.fallback = True
        self.path = None
        self.frame.mode = 'start'
        self.frame.frame_id = self.p.frame_id_start
        self.frame.set_origin(0.0, 0.0, 0.0)
        self.odo_init = self._odo_now()

    def _mode(self):
        if self.fallback:
            base = 'no_gnss'
        elif self.init_res is None:
            base = 'init'
        elif not self.init.done:
            base = 'init_provisional'
        else:
            base = 'nav'
        return 'naive_' + base + ':' + self.state

    # ------------------------------------------------------------ output
    def _output(self, stamp):
        if stamp in self._stamp_set:
            return None
        if len(self._stamps) == self._stamps.maxlen:
            self._stamp_set.discard(self._stamps[0])
        self._stamps.append(stamp)
        self._stamp_set.add(stamp)
        p = self.p
        v, s_odo = self._v_s_at(stamp)
        if not v > 0.0:
            v = 0.0
        if self.variant == 'kf':
            var_v = self.P[1][1]
            a = self.x[2]
        else:
            if self.state == 'both':
                var_v = p.meas_sigma ** 2
            elif self.state == 'hold':
                ages = [self.t - self.bog[b][0] for b in BOGIES if self.bog[b][0] is not None]
                age = min(ages) if ages else 0.0
                var_v = (p.meas_sigma_single + p.acc_max * age) ** 2
            else:
                var_v = p.meas_sigma_single ** 2
            a = 0.0
        out = Output(stamp=stamp, v=v, a=a, var_v=var_v, mode=self._mode(), slip=False,
                     frame_id=self.frame.frame_id)
        var_s = (p.odo_scale_sigma * abs(s_odo)) ** 2
        if self.path is not None:
            s_route = self.s_offset + s_odo + p.output_along_offset_m
            x, y, z, yaw = self.path.pose(s_route)
            out.x, out.y, out.z = self.frame.apply(x, y, z + p.output_z_offset_m)
            yaw = self.frame.apply_yaw(x, y, yaw)
            out.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            out.has_pose = True
            var = self.sigma0 ** 2 + var_s
            L = self.route.L
            off = s_route - L if s_route > L else (-s_route if s_route < 0.0 else 0.0)
            var_c = p.pose_sigma_cross_m ** 2
            if s_route > L:
                geo = (p.tail_cross_frac * off) ** 2
                var += (p.tail_sigma_frac * off) ** 2 + geo
                var_c += geo
            elif s_route < 0.0:
                var_c += (p.bridge_cross_frac * off) ** 2
            out.var_along = var
            out.var_cross = var_c
            sy = min(math.pi, math.hypot(p.pose_sigma_yaw_rad, p.offmap_yaw_rad_per_m * off))
            out.var_yaw = sy * sy
            out.diag['s_route'] = s_route
            out.diag['off_map_m'] = off
        elif self.fallback:
            d = s_odo - self.odo_init
            out.x, out.y, out.z = self.frame.apply(d, 0.0, 0.0)
            out.has_pose = True
            out.var_along = var_s
        vf = self.bog['front'][1] if self._valid('front', self.t) else float('nan')
        vr = self.bog['rear'][1] if self._valid('rear', self.t) else float('nan')
        out.diag.update(s_odo=s_odo, v_front=vf, v_rear=vr)
        self.n_out += 1
        return out


Pipeline = NaivePipeline        # run_cv times every on_* call of a module-level 'Pipeline'


def _run(bag, params, maps_dir, data, use_gnss, gnss_limit_s, variant):
    p = params or Params()
    if not p.vehicle_id:
        base = os.path.basename(os.path.normpath(bag)) if isinstance(bag, str) else ''
        if base[:5] in ('30618', '30639'):
            p = copy.copy(p)
            p.vehicle_id = base[:5]
    D = data if data is not None else load_data(bag)
    pipe = Pipeline(p, maps_dir or default_maps_dir(), variant)   # module global: may be run_cv's timed subclass
    rows = []
    t_first = None
    cpu0 = time.process_time()
    for t_rec, kind, pl in merged_events(D, use_gnss):
        if kind in ('front', 'rear'):
            out = pipe.on_wheel(kind, float(pl[0]), float(pl[1]))
        elif kind == 'cmd':
            out = pipe.on_cmd(float(pl[0]), int(pl[1]))
        else:
            if t_first is None:
                t_first = float(pl[0])
            if gnss_limit_s is not None and float(pl[0]) > t_first + gnss_limit_s:
                continue
            pipe.on_fix(kind, float(pl[0]), float(pl[1]), float(pl[2]), float(pl[3]), int(pl[4]))
            continue
        if out is not None:
            rows.append(out)
    cpu = time.process_time() - cpu0
    res = {
        't': np.array([o.stamp for o in rows]),
        'v': np.array([o.v for o in rows]),
        'a': np.array([o.a for o in rows]),
        'x': np.array([o.x for o in rows]),
        'y': np.array([o.y for o in rows]),
        'z': np.array([o.z for o in rows]),
        'yaw': np.array([o.yaw for o in rows]),
        'has_pose': np.array([o.has_pose for o in rows], bool),
        'var_v': np.array([o.var_v for o in rows]),
        'var_along': np.array([o.var_along for o in rows]),
        'var_cross': np.array([o.var_cross for o in rows]),
        'var_yaw': np.array([o.var_yaw for o in rows]),
        'slip': np.array([o.slip for o in rows], bool),
        'mode': np.array([o.mode for o in rows]),
        'cpu_s': cpu,
        'n_out': len(rows),
    }
    diag_keys = set()
    for o in rows[-1:]:
        diag_keys.update(k for k, v in o.diag.items() if isinstance(v, (int, float)))
    for k in sorted(diag_keys):
        res['diag_' + k] = np.array([float(o.diag.get(k, np.nan)) for o in rows])
    return res


def run_bag(bag, params=None, data=None, gnss_limit_s=10.0, maps_dir=None, use_gnss=True, **kw):
    """Naive wheel-average baseline; same return contract as tram_backup_odometry.replay.run_bag."""
    return _run(bag, params, maps_dir, data, use_gnss, gnss_limit_s, 'naive')


def run_bag_kf(bag, params=None, data=None, gnss_limit_s=10.0, maps_dir=None, use_gnss=True, **kw):
    """Model-free constant-acceleration Kalman filter on the raw bogie samples (no slip logic)."""
    return _run(bag, params, maps_dir, data, use_gnss, gnss_limit_s, 'kf')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag id (cache .work/cache/<bag>.pkl), bag directory or .pkl')
    ap.add_argument('--variant', default='naive', choices=('naive', 'kf'))
    ap.add_argument('--gnss-limit', type=float, default=10.0)
    ap.add_argument('--antenna', default='master')
    a = ap.parse_args(argv)
    sys.path.insert(0, _TOOLS)
    import evaluate as ev
    path = a.bag if os.path.exists(a.bag) else ev.cache_path(a.bag)
    if not os.path.exists(path):        # hash suffix only: 927002c2 -> 30639_927002c2
        cand = sorted(f for f in os.listdir(ev.CACHE_DIR) if f.endswith('_' + a.bag + '.pkl'))
        if len(cand) != 1:
            raise SystemExit('bag %r not found (or ambiguous) in %s' % (a.bag, ev.CACHE_DIR))
        path = os.path.join(ev.CACHE_DIR, cand[0])
    bag = os.path.splitext(os.path.basename(os.path.normpath(path)))[0]
    D = load_data(path)
    fn = run_bag_kf if a.variant == 'kf' else run_bag
    est = fn(path, Params(), data=D, gnss_limit_s=a.gnss_limit)
    print('outputs %d, cpu %.2f s, pose %.1f%%' % (est['n_out'], est['cpu_s'],
                                                   100 * est['has_pose'].mean() if est['n_out'] else 0))
    R = ev.evaluate(bag, est, a.antenna, D=D)
    for k in ('v_rmse', 'v_bias_trans', 'al_rmse', 'al_max', 'ct_rmse', 'e3d_mean', 'drift_pct', 'p_in95', 'score_loss'):
        if k in R:
            print('%-14s %s' % (k, R[k]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
