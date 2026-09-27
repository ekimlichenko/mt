# Офлайн-инструменты

Запуск из корня решения офлайн-интерпретатором Python (numpy, scipy, matplotlib, PyYAML):

    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
    PY=python3          # любой python 3.10+ с numpy, scipy, matplotlib, pyyaml

Входные данные: разобранные bag `.work/cache/<bag>.pkl`, индекс `.work/bag_index.json`, карты
`src/tram_backup_odometry/maps/{t2s,s2t}.json`. Сам датасет (`../dataset/data/<bag>/`) нужен
только `build_cache.py` и `bag_index.py`; `bag_index.py` читает `metadata.yaml`, а если его нет,
берёт время первой и последней записи из кэша. ROS-пакет эти инструменты не импортирует, и для
работы ноды они не нужны. Исключения:

- `latency_probe.py` и `resource_monitor.py` запускаются рядом с живой нодой (см. ниже);
- `checker_sim.py` читает тестовый bag организаторов
  `../check_code/check-code/bags/30618_88aea4d9` напрямую, без кэша;
- `build_loops.py` дополнительно читает тот же bag для проверки, если он есть.

Сам чекер организаторов в Docker запускает `scripts/run_checker.sh` (раздел в конце).

Где что описано: итоговые числа — [docs/RESULTS.md](../docs/RESULTS.md), воспроизведение
оценки для жюри — [docs/JURY_GUIDE.md](../docs/JURY_GUIDE.md), раздел 7, скрипты калибровки
параметров — [calibration/README.md](calibration/README.md).

## build_cache.py: разбор bag в кэш

    $PY tools/build_cache.py                    # все bag из ../dataset/data -> .work/cache/<bag>.pkl
    $PY tools/build_cache.py 0e41eac3 --force   # выбранные bag (полное имя или уникальный суффикс), перезаписать
    $PY tools/build_cache.py --list             # какие bag уже в кэше

Разбирает sqlite + CDR без ROS (`tram_backup_odometry.bagio.load_bag`) параллельно (`--jobs`,
по умолчанию min(8, число CPU − 2)) и записывает каждый pickle атомарно. Существующие файлы
пропускаются, если не задан `--force`. `--data` и `--out` меняют пути по умолчанию. Полный кэш:
122 файла, около 0.5 ГБ. После пересборки кэша обновите индекс (`bag_index.py`).

## bag_index.py: список bag, группы, фолды

    $PY tools/bag_index.py            # -> .work/bag_index.json (--out; --jobs, по умолчанию 8)

Для каждого bag: вагон, день по московскому времени, `group = <vehicle>_<day>` (единица
leave-one-group-out), `fold`, длительность, покрытие GNSS/RTK, направление, доля на карте,
дубли (`dup_of`, по md5 скорости передней тележки) и `eval_ok` (уникальный, не меньше 60 с
RTK-эталона). Текущие данные: 122 bag, 97 уникальных, 60 `eval_ok`, 6 групп (они же фолды).

## build_loops.py: геометрия разворотных петель (`maps/loops.json`)

    OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry $PY tools/build_loops.py
    $PY tools/build_loops.py --no-test-bag --no-plot      # без проверки на тестовом bag и без рисунка

Каждый прогон начинается на конечной перед разворотной петлёй и заканчивается в петле другой
конечной, а в pathgraph петель нет. Скрипт восстанавливает осевые линии петель офлайн по
RTK-фиксам master (status 2) уникальных обучающих bag обоих вагонов. Полное описание в
docstring. Шаги:

1. **Отбор фиксов.** Выбрасываются глитчи stamp и позиции (`evaluate.position_glitch_mask`)
   и дубли. Геометрию строят только прогоны с качеством на карте q ≥ 0.45.
2. **Сегменты.** Голова берётся до въезда на карту (только прогоны со стартом в пределах 20 м
   от s = 0), хвост после съезда с неё (в пределах 5 м от s = L).
3. **Кластеры.** Для каждой пары (конечная, голова/хвост) прогоны группируются по совпадению
   трека: p90 расстояния < 2 м.
4. **Шаблон кластера.** Медиана по 1 м колёсного пути и 3 итерации уточнения по боковому
   смещению; z = медиана (alt GNSS − 3.10). Путь антенны сдвигается на ось пути
   (`--antenna-path` оставляет путь антенны).
5. **Ветви.** Ветви сшиваются, сглаживаются, перепрорежаются с шагом ~1 м и плавно
   стыкуются с концами pathgraph.
6. **Проверка.** Для каждого прогона и тестового bag печатаются поперечное отклонение
   фиксов от лучшей ветви и расхождение длины дуги с колёсным путём.

Результат: `src/tram_backup_odometry/maps/loops.json` (`--out`) и `docs/plots/loops.png`,
время работы около 5 с. Состав текущего файла:

| петля (from → to) | ветвь | длина, м | прогонов (хвост/голова) | не наблюдалось, м дуги |
|---|---|---|---|---|
| S, Щукинская (T2S → S2T) | A | 1108.67 | 51 (26/25) | — |
| T, Таллинская (S2T → T2S) | A | 519.98 | 39 (24/15) | — |
| T | B | 586.14 | 6 (4/2) | 227.9–365.1: разворот не записан, вместо него заглушка-«баллон» R 18 м |
| T | C | 521.06 | 6 (0/6) | 296.7–321.7: A со смещением на параллельный путь у платформы |

Нода читает файл из `maps_dir`. Если его нет, `load_loops` возвращает None, и нода работает
как без петель (`loops_enable` ничего не меняет). Пересборка нужна только при изменении
обучающих данных.

## evaluate.py: оценщик в духе жюри (модуль и CLI)

    import evaluate as ev
    R = ev.evaluate(bag, est)            # est: словарь run_bag() или путь к .npz
    R = ev.evaluate(bag, est, antenna='rover', series=True)   # series: ряды для рисунков

Эталон: фиксы master со status 2 в системе карты (UTM 37N − (300000, 6100000)), z = высота
GNSS; `antenna='rover'` или `'mid'` берёт rover или середину базы. Скорость эталона —
`hypot(vx, vy)` из `/sensing/gnss/master/vel` там, где в пределах 0.05 с есть фикс со status 2.
Сопоставление: ближайший по stamp выход в пределах 0.05 с (`--tol`); выход с
`has_pose=False` считается пропуском. Метрики:

- скорость: rmse, mae, bias, p99, max, по режимам, покрытие;
- положение: e2d/e3d, along/cross-track на карте с воротами 5 м (`--gate`), e2d вне карты,
  дрейф в конце в % пути, дрейф на карте, рост along-track;
- `score_loss`.

Полные определения — в docstring модуля. `clean=True` / `--clean` убирает сбои эталона: скачки
stamp, скачки RTK-позиции и скорость GNSS, замёрзшую на 0 при движущихся фиксах.

    $PY tools/evaluate.py 30618_0e41eac3 --est est.npz [--antenna master|rover|mid] [--clean]
    $PY tools/evaluate.py 30618_0e41eac3 --self-test          # эталон как оценка -> все ошибки 0
    $PY tools/evaluate.py 30618_0e41eac3 --self-test interp   # эталон на stamp входов (нижняя граница ошибки)

`.npz` оценки содержит ключи `t, v, x, y, z, has_pose` (такие файлы пишет
`run_cv.py --save-est`). Без имени bag `--self-test` проходит по всем bag кэша. Имя bag
ставится до `--self-test`: следующий за ним аргумент argparse читает как режим
(`exact` | `interp`), и `--self-test 30618_0e41eac3` завершается ошибкой `invalid choice`.

Тесты: `$PY src/tram_backup_odometry/test/test_evaluate.py` (или pytest).

## run_cv.py: параллельный реплей и оценка

    $PY tools/run_cv.py --tag F4_final --bags all --antenna all --jobs 6    # итоговый прогон (22 с на 6 процессах)
    $PY tools/run_cv.py --tag F4_final_burst --bags all --antenna all --gnss-limit 30 --gnss-burst 120 1.5
    $PY tools/run_cv.py --tag B1_naive --bags all --jobs 4 --estimator baseline_naive:run_bag
    $PY tools/run_cv.py --tag x --bags 'eval&dir:S2T' --set trn_enable=false --plots worst
    $PY tools/run_cv.py --tag cal --bags eval --fold-params .work/foldp   # fold_<k>.yaml, вне фолда
    $PY tools/run_cv.py --tag zoh --bags eval --estimator path/to/file.py:func
    $PY tools/run_cv.py --compare F4_final x

Выбор bag (`--bags`, по умолчанию `all`): `all`, `eval`, `unique`, `nogps` (нет топика
GNSS-фиксов), `noeval` (не `eval_ok`), `vehicle:`, `group:`, `fold:`, `dir:`, `@file`, имена bag
(достаточно суффикса-хэша); `,` — объединение, `&` — пересечение.

Основные опции:

- `--jobs` — число процессов, по умолчанию число CPU − 2;
- `--antenna master|rover|mid|all` — эталон, первая антенна основная (по умолчанию `master`);
- `--estimator module:function` или `file.py:function` — оценщик вместо
  `tram_backup_odometry.replay:run_bag`, вызывается как `f(bag_path, params, data=..[, gnss_limit_s=..])`;
- `--gnss-limit` — GNSS подаётся только первые N с после первого фикса (по умолчанию 10);
- `--gnss-burst PERIOD DUR` — после окна `--gnss-limit` подавать ещё пачку GNSS длительностью
  DUR с каждые PERIOD с. Так имитируется редкий GNSS в середине маршрута для коррекции
  положения (`gnss_aiding`). Передаётся оценщику, у которого есть аргумент `gnss_burst`
  (`replay.run_bag`);
- `--tol` (0.05 с), `--gate` (5 м), `--clean-ref` — как в `evaluate.py`;
- `--plots none|all|worst|<bags>` — рисунки `results/<tag>/plots/<bag>.png`;
- `--save-est` — сохранить оценки в `results/<tag>/est/<bag>.npz`;
- `--compare TAG_A TAG_B` — сравнить два готовых прогона без реплея.

Результат: `results/<tag>/per_bag.json`, `summary.json` (с `argv` и `wall_s`) и `summary.md`.
В сводке медиана, среднее, p90 и max по `eval_ok` bag, разбивка по направлению, вагону и группе,
метрики против rover- и mid-эталона (при `--antenna all`), худшие bag, а также робастность и
время по всем bag. Bag без GNSS тоже прогоняются; для них учитываются исключения, нечисловые
и отрицательные выходы и stamp, не взятые из входов. Порядок наложения параметров: значения
`Params` по умолчанию, затем yaml из `--params`, затем `--fold-params`, затем `--set`.
Ключи, которых нет в `Params`, становятся дополнительными атрибутами и перечисляются в сводке.

Эталон CV — фиксы master-антенны. Поэтому `build_params` перед `--set` принудительно
выставляет выходную точку master-антенны: `output_along_offset_m=0`, `output_z_offset_m` =
`z_offset_m` (3.10) и `output_velocity_delay_s=0`. У пакета по умолчанию другое: `base_link`,
z pathgraph и задержка 0.09 с под чекер организаторов. Вернуть значения пакета можно через
`--set output_along_offset_m=9.873 ...`. `vehicle_id` берётся из префикса имени bag, если не
задан.

## checker_sim.py: офлайн-эмуляция чекера организаторов

    OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry $PY tools/checker_sim.py \
        --gnss-start-only 35 --out results/checker_30618_88aea4d9
    $PY tools/checker_sim.py --variants final,no_gnss_aiding --set gnss_sigma_rtk_m=2.0

Эмулирует `hackathon_solution_checker` на тестовом bag организаторов с эталоном
`/localization/kinematic_state` (base_link на pathgraph, 50 Гц):

- bag идёт через тот же `Pipeline`, что в ноде, в порядке времени записи;
- выход считается пришедшим через 1 мс после записи своего входа;
- `ApproximateTimeSynchronizer` эмулируется с очередью 100 и slop 0.05 с;
- метрики те же, что у чекера: v RMSE по `twist.linear.x`, x/y/z и 3D RMSE и max. Сверх
  этого считаются медиана, p95 и 3D RMSE по минутам.

Опции:

- `--bag` — по умолчанию `../check_code/check-code/bags/30618_88aea4d9`;
- `--vehicle-id` — по умолчанию `30618`;
- `--variants all|final|<список>` — по умолчанию `all`;
- `--gnss-start-only N` — добавляет вариант `final_gnss_first_Ns_only`, где GNSS подаётся
  только первые N с;
- `--set k=v ...` — переопределения для всех вариантов;
- `--out DIR` — пишет `summary.json` и `summary.md`.

Варианты:

| вариант | что меняется |
|---|---|
| `final` | ничего, значения пакета по умолчанию |
| `no_gnss_aiding` | `gnss_aiding=False` |
| `no_loops` | `loops_enable=False` |
| `no_velocity_delay` | `output_velocity_delay_s=0` |
| `master_antenna_point` | `output_along_offset_m=0`, `output_z_offset_m=3.10` |
| `pre_checker_version` | все четыре изменения сразу |

Результат для `30618_88aea4d9` лежит в `results/checker_30618_88aea4d9/summary.md` (снят первой
командой выше, с `--gnss-start-only 35`; строка «Command:» в заголовке файла эту опцию не
показывает). Для `final` v RMSE 0.0335 м/с и 3D RMSE 1.11 м, для `pre_checker_version`
0.0566 м/с и 24.24 м.
Полная таблица и разбор — [docs/RESULTS.md](../docs/RESULTS.md), разделы 2.1 и 2.3.

## plots.py: рисунок по одному bag

    $PY tools/plots.py --tag mytag --bags 30618_0e41eac3,40ffd323 [--set k=v ...]

`--tag` и `--bags` обязательны; `--params`, `--set`, `--estimator`, `--antenna`, `--gnss-limit`
как в `run_cv.py` (оценщик прогоняется заново). Пишет `results/<tag>/plots/<bag>.png`. Левая
колонка: скорость против эталона, ошибка скорости, along-track и 2D-ошибка. Справа карта xy с
обоими маршрутами и увеличенные первые и последние 600 м пути. Те же рисунки строит
`run_cv.py --plots worst|all|<bags>`.

## baseline_naive.py: воспроизводимый наивный baseline (`results/B1_naive`)

    $PY tools/baseline_naive.py 0e41eac3 [--variant naive|kf] [--gnss-limit 10] [--antenna master]
    $PY tools/run_cv.py --tag B1_naive    --bags all --antenna master --jobs 4 --estimator baseline_naive:run_bag
    $PY tools/run_cv.py --tag B1_naive_kf --bags all --antenna master --jobs 4 --estimator baseline_naive:run_bag_kf

`naive`: средняя скорость тележек / K, путь интегрируется по тому же pathgraph, с той же
привязкой по GNSS и той же выходной СК, но без EKF, модели тяги, детектора юза и TRN. `kf`:
безмодельный кинематический фильтр Калмана по `[s, v, a]`. Разница с `F4_final` объясняется
только оценкой скорости и пути. CLI печатает основные метрики одного bag.

## robustness.py: набор тестов робастности (`results/robustness`)

    $PY tools/robustness.py --jobs 4              # все части, 4 процесса (по умолчанию --jobs 4)
    $PY tools/robustness.py --part faults --bags 30618_073f08d1 --scenarios drop_both_30 --jobs 1
    $PY tools/robustness.py --part plots          # только рисунки (нужен summary.json)
    $PY tools/robustness.py --list                # список сценариев (28)

Части (`--part`, список через запятую; по умолчанию `all`):

- `faults` — синтетические отказы на фиксированных 12 `eval_ok` bag (6 T2S / 6 S2T, оба вагона);
  эталон всегда строится по чистым данным;
- `clean` — чистые прогоны 60 `eval_ok` bag: частота флага юза, сцепление, адаптация усилений;
- `real` — реальные bag с аномалиями без инъекций;
- `plots` — рисунки `docs/plots/robust_*.png` (каталог задаёт `--plots`);
- `md` — переписать `summary.md` из `summary.json`.

Результат: `results/robustness/{summary.md, summary.json, per_bag.json}` (каталог `--out`).
Прочие опции: `--seed` (0), `--gnss-limit` (10). Определения метрик — в docstring (`--help`).

## inject_faults.py: синтетические отказы датчиков

    import inject_faults as fi                    # sys.path: tools/, src/tram_backup_odometry/
    D2 = fi.apply(D, [{'kind': 'drop_both', 't0': 300, 'dur': 30}], seed=0)
    p = Params(output_along_offset_m=0.0, output_z_offset_m=3.10, output_velocity_delay_s=0.0)
    est = replay.run_bag(pkl_path, p, data=D2, gnss_limit_s=10.0)

`apply(D, specs, seed=0)` возвращает изменённую глубокую копию разобранного bag (`D` — pickle
из кэша); `t0` — секунды от первого stamp колёс. У `replay.run_bag` по умолчанию
`gnss_limit_s=None` (GNSS весь прогон), поэтому окно 10 с, как в `run_cv.py`, задаётся явно.
Точка выхода тоже задаётся явно: `evaluate.py` сравнивает с master-антенной, а `Params()` по
умолчанию публикует `base_link` (+9.873 м вдоль пути) и скорость с задержкой 0.09 с. Те же три
переопределения ставит `run_cv.build_params`. `robustness.py` (`params_for`) их пока не ставит,
поэтому абсолютные al_rmse и v_rmse в его таблице 10 с CV не сравниваются; парные Δ-метрики
от точки выхода не зависят.

Виды: `drop_both`, `drop_front` / `drop_rear`, `zero_front` / `zero_rear`, `freeze_front` /
`freeze_rear`, `spikes`, `noise`, `scale_front`, `slip_front`, `stamp_jump`, `cmd_drop`,
`cmd_freeze`, `nan_front`. Параметры каждого вида — в docstring.
Используется в `robustness.py`, который добавляет свои виды (`slip_rear`, `slip_both`,
`noise_moving`).

## report_plots.py: рисунки отчёта (`docs/plots/*.png`)

    $PY tools/report_plots.py                                    # все рисунки
    $PY tools/report_plots.py --only realtime,ablations --jobs 4
    $PY tools/report_plots.py --only realtime --rt-short results/rt/0e41eac3_120s_final --rt-full results/rt/27e994fc_full_final

Рисунки (`--only`): `speed_example`, `speed_error`, `along_track`, `trajectory`, `per_bag`,
`ablations`, `realtime`, `covariance`. Рисунок абляций берёт `B1_naive`, `F4_final`,
`AF_*` (включая `AF_noloops`, `AF_noaiding`) и `G_limit*` из `results/`. Реплей 60 `eval_ok` bag тем же `Pipeline`, таблицы из
`results/<tag>/` (`--tag`, по умолчанию `F4_final`), замеры реального времени из
`--rt-short` / `--rt-full`. По умолчанию эти два аргумента указывают на замеры ранней сборки
(`results/rt/0e41eac3_120s`, `results/rt/27e994fc_full`); для замеров образа версии F3
передайте `--rt-short results/rt/0e41eac3_120s_final --rt-full results/rt/27e994fc_full_final`.
Прочие опции: `--out`, `--jobs`
(по умолчанию число CPU − 2), `--gnss-limit`, `--t2s-bag`, `--s2t-bag`, `--speed-bag`,
`--speed-t0`, `--cov-bag`, `--cache`.

## gen_params_yaml.py: `config/params.yaml` из `Params`

    $PY tools/gen_params_yaml.py            # переписать src/tram_backup_odometry/config/params.yaml
    $PY tools/gen_params_yaml.py --check    # "... params.yaml is up to date (199 parameters)", иначе код 1

Источник имён, типов и значений по умолчанию — `core/config.py`. После записи файл читается
обратно и сверяется с `Params()` по значению и типу. `scripts/build.sh` вызывает генератор
перед сборкой образа.

## latency_probe.py, resource_monitor.py: замеры живой ноды

Работают рядом с запущенной нодой, а не с кэшем: `latency_probe.py` требует `rclpy` и
`tram_vehicle_msgs`, `resource_monitor.py` — `psutil`. Обычно их запускает
`scripts/run_bag.sh --measure` (или `scripts/measure.sh`) внутри Docker-контейнера, результат
лежит в `<out_dir>/latency.*` и `<out_dir>/resources.*`.

- `latency_probe.py`: задержка «приход входа → приход выхода» по совпадению header stamp,
  частота выхода. Опции `--duration`, `--idle-exit`, `--warmup` (по умолчанию 2 с, исключаются
  из установившихся оценок), `--json`, `--csv`.
- `resource_monitor.py`: CPU в ядрах и RSS процесса ноды (psutil, шаг `--interval` 0.5 с).
  Опции `--pattern` (по умолчанию `backup_odometry_node`), `--pid`, `--duration`, `--wait`,
  `--csv`, `--json`.

## scripts/run_checker.sh: настоящий чекер организаторов в Docker

    ./scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk --measure
    ./scripts/run_checker.sh <bag_dir> <out_dir> [--rate R] [--duration S] [--vehicle ID] [--measure] \
        [--checker DIR] [--image NAME] [-- аргументы launch]

Это не инструмент `tools/`, но он использует `latency_probe.py` и `resource_monitor.py`
(`--measure`). Все прогоны идут в одном контейнере образа `tram_backup_odometry:humble` с
`--network none`. Порядок:

1. Собирается пакет `hackathon_solution_checker` из `--checker` (по умолчанию
   `../check_code/check-code/src/checker_ros`) поверх нашего `tram_vehicle_msgs`.
2. Запускаются нода и `ros2 run hackathon_solution_checker metrics`, `ros2 bag record` и пробы.
3. Проигрывается `ros2 bag play -d 1 -r R`.
4. Чекер останавливается по SIGINT, и итоговый отчёт пишется в
   `<out_dir>/checker_final.{txt,json}`.

Остальные файлы в `<out_dir>`: `checker.log`, `checker_build.log`, `node.log`, `play.log`,
`record.log`, `result_bag/`, а с `--measure` ещё `latency.*` и `resources.*`.

Ограничения:

- `vehicle_id` берётся из префикса имени bag (`--vehicle`).
- `<out_dir>/result_bag` не должен существовать.
- После `--` действуют только аргументы, объявленные в launch: `params_file`,
  `output_frame`, `vehicle_id`, `use_sim_time`, `maps_dir`, `log_level`. Остальные
  (`gnss_aiding:=false` и т. п.) `ros2 launch` молча игнорирует. Их задают частичным YAML в
  `<out_dir>` с `-- params_file:=/out/<file>.yaml`.

Подробности в docs/JURY_GUIDE.md, раздел 7.1.

## calibration/: откуда взяты значения параметров

Скрипты калибровки (масштаб колёс, модель тяги, TRN, мост, согласованность ковариации и др.)
описаны в [calibration/README.md](calibration/README.md).
