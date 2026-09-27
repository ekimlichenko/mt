# Заметки по ROS 2: tram_backup_odometry

Краткая памятка по сборке и запуску в ROS 2 Humble (Docker Desktop, arm64, образ `ros:humble`,
workspace в `/ws`). Полная инструкция с командами, интерфейсом и проверкой —
[JURY_GUIDE.md](JURY_GUIDE.md); справочник всех параметров — [PARAMETERS.md](PARAMETERS.md).
Актуальные замеры — [RESULTS.md](RESULTS.md), раздел 7; как их снять — JURY_GUIDE, раздел 6.

## Сборка

```bash
./scripts/build.sh                 # params.yaml -> docker build -> офлайн colcon build (--network none)
RUN_TESTS=1 ./scripts/build.sh     # ... + pytest пакета внутри образа
```

- Сначала скрипт перегенерирует `src/tram_backup_odometry/config/params.yaml` из
  `core/config.py` (`tools/gen_params_yaml.py`: 199 параметров, с обратной сверкой значений и
  типов). Только проверить: `python3 tools/gen_params_yaml.py --check`
  (`.../params.yaml is up to date (199 parameters)`, код 1, если файл устарел).
- `docker build` получает только `src/ tools/ docker/` tar-потоком: `.work` с кэшами занимает
  сотни МБ. Обычный `docker build -f docker/Dockerfile -t tram_backup_odometry:humble .` из
  корня тоже работает, `docker/Dockerfile.dockerignore` ограничивает контекст теми же тремя
  каталогами.
- Офлайн-проверка: чистый `colcon build` смонтированных исходников в контейнере
  `docker run --network none`. Ожидается `2 packages finished`, `import OK`, оба исполняемых
  файла (`backup_odometry_node`, `replay`) и `share/tram_backup_odometry/{maps,config,launch}`.
- `src/tram_vehicle_msgs` совпадает с пакетом из датасета (`diff -r` с
  `../dataset/tram_vehicle_msgs` пуст). Организаторы подтвердили, что при проверке берутся
  именно эти сообщения.
- В `tram_vehicle_msgs/package.xml` (пакет организаторов) нет `<maintainer>`, и colcon в
  Humble (catkin_pkg 1.x) его отклоняет. Dockerfile и скрипты добавляют заглушку только в
  копии для сборки и только если тега нет; исходный файл не меняется. Так же
  `scripts/run_checker.sh` собирает пакет чекера `hackathon_solution_checker`.
- Тесты в образе: `120 passed, 11 skipped` (`results/docker_build.log`). Пропускается
  `test_evaluate.py`: ему нужен scipy для `tools/evaluate.py`, а scipy не является
  зависимостью времени выполнения. На хосте без ROS: `106 passed, 12 skipped` (дополнительно
  пропускается `test_ros_node.py`: нет `rclpy`). В `test_ros_node.py` есть проверки, что по
  умолчанию подписки на GNSS остаются, а с `gnss_aiding=False` уничтожаются после выставки.
  `test_gnss_aiding.py` проверяет, что коррекция по GNSS не меняет скорость.

## Запуск ноды

```bash
docker run --rm -it --network host tram_backup_odometry:humble          # по умолчанию: ros2 launch ...
docker run --rm -it --network host tram_backup_odometry:humble \
    ros2 launch tram_backup_odometry backup_odometry.launch.py vehicle_id:=30618 output_frame:=map
```

Аргументы launch (пустое значение оставляет значение из `params_file`):

| аргумент | по умолчанию | примечание |
|---|---|---|
| `params_file` | `share/tram_backup_odometry/config/params.yaml` | `backup_odometry: ros__parameters:` |
| `vehicle_id` | `30618` (launch); `''` в `params.yaml` | `30618` / `30639` выбирают масштаб колёс K = 3.597 / 3.59; пусто — 3.595. Скрытая проверка идёт на 30618, это умолчание launch; `vehicle_id:=''` берёт значение из файла параметров |
| `output_frame` | `''` (yaml: `map`) | `map` / `utm` (фрейм `utm_37n`) / `start` (фрейм `odom`) / `enu` (фрейм `enu`) |
| `use_sim_time` | `false` | результат считается по header stamp, подходят оба значения |
| `maps_dir` | установленные `share/.../maps` | `t2s.json`, `s2t.json`, `loops.json` (разворотные петли, `tools/build_loops.py`) |
| `log_level` | `info` | |

Launch передаёт ноде только эти аргументы. Другие (`gnss_aiding:=false` и т. п.)
`ros2 launch` молча игнорирует. Остальные параметры задаются через `params_file` (можно
частичный: нода объявляет все параметры со значениями по умолчанию) или через
`ros2 run tram_backup_odometry backup_odometry_node --ros-args -p name:=value`. Параметры под
чекер организаторов (по умолчанию):

| параметр | по умолчанию | смысл |
|---|---|---|
| `output_along_offset_m` | 9.873 | публикуется точка `base_link` на pathgraph в 9.873 м впереди master-антенны, как в эталоне `/localization/kinematic_state`; 0 = master-антенна |
| `output_z_offset_m` | 0.0 | z = z pathgraph; 3.10 = высота master-антенны (прежнее соглашение) |
| `output_velocity_delay_s` | 0.09 | в `/result/velocity` публикуется оценка на момент stamp − 0.09 с: эталонный `twist.linear.x` отстаёт от своего stamp; 0 = без задержки |
| `gnss_aiding` | `true` | подписки на GNSS остаются после выставки, редкие фиксы корректируют положение вдоль пути (скорость не меняется); `false` = только выставка |
| `loops_enable` | `true` | геометрия разворотных петель из `maps/loops.json` вне pathgraph |

Прочие параметры под систему оценки (`enu_origin_auto`, `enu_origin_lat/lon/alt`,
`publish_on`) описаны в JURY_GUIDE, раздел 4.1. Пример частичного файла:

```yaml
backup_odometry:
  ros__parameters:
    gnss_aiding: false
```

`docker stop` завершает контейнер штатно: в образе задан `STOPSIGNAL SIGINT`, нода печатает
итоговую строку `shutdown: ...`.

Все параметры объявлены read-only: менять их через yaml или `-p name:=value`, изменение на
лету отклоняется. Неизвестное значение перечислимого параметра (`publish_on`, `tail_mode`,
`output_frame`) останавливает ноду при старте с `ValueError`. Ключ в файле — имя узла
`backup_odometry`, поэтому нода, запущенная в namespace, файл не подхватит.

Робастность: NaN и inf не попадают в `/result/*`. Выход с нечисловой скоростью отбрасывается,
с нечисловой позой публикуется только как скорость, нечисловые σ²_along и σ²_v публикуются
как 1e6. Все такие случаи считаются (`msgs_nonfinite`) и поднимают WARN в диагностике.
Исключения локализованы в каждом callback, включая таймер диагностики, поэтому одно плохое
значение не останавливает `rclpy.spin`.

## Топики

| направление | топик | тип | QoS |
|---|---|---|---|
| вход | `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` (км/ч) | best effort, KEEP_LAST 50 |
| вход | `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` | best effort, KEEP_LAST 50 |
| вход (выставка и коррекция положения) | `/sensing/gnss/{master,rover}/fix` | `sensor_msgs/NavSatFix` | best effort, 50. По умолчанию (`gnss_aiding: true`) подписки остаются на весь прогон, фиксы идут только в положение. С `gnss_aiding:=false` или при переходе в режим без GNSS подписки уничтожаются после окончательной выставки (строка лога `GNSS subscriptions destroyed`, диагностика `gnss_subscribed=False`) |
| выход | `/result/velocity` | `VelocitySensor` (м/с), фрейм `velocity_frame_id` = `base_link` | reliable, volatile, 50 |
| выход | `/result/position` | `nav_msgs/Odometry`, фрейм = выходной (`map` по умолчанию), child `child_frame_id` = `base_link`; точка `base_link` на pathgraph, z = z pathgraph | reliable, volatile, 50 |
| выход | `/backup_odometry/diagnostics` | `diagnostic_msgs/DiagnosticArray`, 1 Гц (steady clock) | reliable, 10 |
| выход | `/backup_odometry/slip` | `std_msgs/Bool`, при изменении | reliable, transient local, 1 |

Stamp выхода — исходный объект `Time` входного сообщения, вызвавшего публикацию, поэтому
совпадение побитовое. Выходы идут в порядке прихода входов, и их stamp не монотонны:
метки колёс отстают от меток команды на десятки миллисекунд. Для монотонных меток —
`publish_on:=cmd` (20 Гц).

## Прогон bag, проверка и замер

```bash
scripts/run_bag.sh <bag_dir> <out_dir> [--rate R] [--duration S] [--start S] [--vehicle ID]
                   [--sim-time] [--dev | --src DIR] [--measure] [--image NAME] [-- launch_arg:=value ...]
scripts/measure.sh <bag_dir> [out_dir] [опции run_bag.sh]     # = run_bag.sh --measure --duration 120
```

- `--dev` пересобирает текущий `src/` внутри контейнера, `--src DIR` — то же для другого
  дерева исходников.
- `vehicle_id` по умолчанию берётся из префикса имени bag (`30618_`, `30639_`).
- Всё выполняется в одном контейнере с `--network none`.
- В `<out_dir>` появляются:
  - `result_bag/`;
  - `node.log`, `play.log`, `record.log`, `check.txt` (сверка stamp, фреймов, частоты,
    покрытия сетки 0.1 с, дублей и убываний stamp с входным bag);
  - с `--measure`: `latency.{txt,json,csv}` от `tools/latency_probe.py` и
    `resources.{txt,json,csv}` от `tools/resource_monitor.py`.

Замеры (стартовая пачка bag, дубли stamp, длинный прогон 31 мин) — [RESULTS.md](RESULTS.md),
раздел 7.2; команды — JURY_GUIDE, раздел 6; файлы лежат в `results/rt/`.

**Текущая версия** (петли, коррекция по GNSS, выход `base_link`). Прогон снят под чекером
организаторов на всём тестовом bag 30618_88aea4d9, 1309.6 с
(`results/checker_ros_30618_88aea4d9_final/`, итоговый образ, `run_checker.sh --measure`,
хост без параллельной нагрузки):

- выходов скорости 38 476, частота 29.3 Гц по stamp, дублей stamp 0;
- задержка p50 / p99 1.17 / 3.03 мс (скорость) и 1.90 / 3.78 мс (положение);
- максимум 32.1 / 32.4 мс (стартовая пачка bag), после первых 2 с 22.8 / 24.9 мс; выше
  100 мс — 0 из 76 949 выходов;
- callback ноды mean 0.693 мс, max 6.32 мс;
- CPU 0.071 ядра в среднем, RSS 62.2–64.1 МиБ без роста.

Предварительный прогон (`results/checker_ros_30618_88aea4d9/`) шёл одновременно с офлайн-CV
на том же хосте и дал пики 202 / 204 мс (10 выходов) при callback не больше 13.4 мс
([RESULTS.md](RESULTS.md), раздел 7.1).

**Версия F3** (образ 26.09, до петель и коррекции по GNSS), 30618_0e41eac3, первые 120 с
(`results/rt/0e41eac3_120s_final`): 3645 выходов скорости,
29.9 Гц по stamp, 100 % stamp равны входным, покрытие сетки 0.1 с 100 %, 0 дублей; задержка
после первых 2 с p50 / p99 / max 0.74 / 1.73 / 5.5 мс (скорость) и 1.41 / 2.73 / 7.2 мс
(положение); CPU 0.06 ядра в среднем, RSS 62–63.4 МиБ. Весь bag 30618_27e994fc, 1900 с
(`results/rt/27e994fc_full_final`): 55 948 выходов скорости, 29.4 Гц, 0 дублей, покрытие
100 %; после первых 2 с p50 / p99 / max 0.45 / 1.53 / 28.0 мс (скорость) и 0.88 / 2.37 /
28.2 мс (положение), максимум — единичный пик на 1550-й секунде; CPU 0.044 ядра в среднем,
RSS 62.1–63.8 МиБ без роста.

- **Стартовая пачка.** Каждый bag начинается с пачки устаревших сообщений, записанных за
  доли секунды. `ros2 bag play` выдаёт их разом, поэтому в первые ~0.2 с задержки из-за
  очереди достигают десятков миллисекунд (максимум 62 мс в замере 0e41eac3 версии F3). `latency_probe`
  печатает статистику и по всем выходам, и без первых `--warmup` секунд (по умолчанию 2 с).
- **`--sim-time`.** CPU выше из-за подписки на `/clock` 100 Гц; на результат не влияет.

## Проверка чекером организаторов

```bash
scripts/run_checker.sh <bag_dir> <out_dir> [--rate R] [--duration S] [--vehicle ID] [--measure]
                       [--checker DIR] [--image NAME] [-- launch_arg:=value ...]
scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 /tmp/chk --measure   # ~22 мин
OMP_NUM_THREADS=1 PYTHONPATH=src/tram_backup_odometry python3 tools/checker_sim.py   # то же офлайн, без ROS
```

Как работает `run_checker.sh`:

- Всё идёт в одном контейнере с `--network none`.
- Пакет `hackathon_solution_checker` (по умолчанию `../check_code/check-code/src/checker_ros`)
  собирается colcon поверх `/ws/install`, то есть против нашего `tram_vehicle_msgs`.
- Затем запускаются нода (`vehicle_id` из имени bag), `ros2 run hackathon_solution_checker
  metrics`, `ros2 bag record` и пробы (`--measure`), после чего `ros2 bag play -d 1 -r R`.
- Чекер останавливается по SIGINT, его итоговый отчёт пишется в
  `<out_dir>/checker_final.{txt,json}`.
- Остальные файлы: `checker.log` (отчёты каждые 5 с), `checker_build.log`, `node.log`,
  `play.log`, `record.log`, `result_bag/`, а с `--measure` ещё `latency.*` и `resources.*`.
- `<out_dir>/result_bag` не должен существовать.

Launch принимает только объявленные аргументы (`params_file`, `output_frame`, `vehicle_id`,
`use_sim_time`, `maps_dir`, `log_level`), и `-- gnss_aiding:=false` молча игнорируется.
Такой параметр задаётся частичным YAML в `<out_dir>` и `-- params_file:=/out/<file>.yaml`.

Результат на `30618_88aea4d9`, значения по умолчанию
(`results/checker_ros_30618_88aea4d9_final/checker_final.txt`):

| метрика | RMSE | max | n |
|---|---|---|---|
| скорость, м/с | 0.0324 | 0.302 | 38 437 |
| 3D (distance), м | 1.111 | 14.02 | 38 437 |
| z, м | 0.037 | 0.288 | 38 437 |

Офлайн-эмуляция `tools/checker_sim.py` на текущем коде даёт 0.0335 м/с и 1.11 м
(`results/checker_30618_88aea4d9/summary.md`). Варианты с выключенными механизмами разобраны в
[RESULTS.md](RESULTS.md), разделы 2.1 и 2.3.
