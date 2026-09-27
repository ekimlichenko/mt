# Чекер организаторов в ROS 2 (Docker) на 30618_88aea4d9 — итоговый образ

Итоговый прогон: образ `tram_backup_odometry:humble` пересобран 27.09 после правки `pipeline.py`
(anchor_delta при неактивном TRN) и смены умолчания `vehicle_id` в launch на 30618
(`RUN_TESTS=1 scripts/build.sh`, `.work/build_final3.log`: offline colcon build + pytest в образе 120 passed, 11 skipped).
Оценка — **собственный чекер организаторов** `hackathon_solution_checker`, собранный из их исходников;
проигрывание в реальном времени (rate 1, полный bag, 1311 с). Во время прогона на хосте не запускались
тяжёлые офлайн-расчёты (в отличие от предварительного прогона `results/checker_ros_30618_88aea4d9/`).

```bash
RUN_TESTS=1 scripts/build.sh
scripts/run_checker.sh ../check_code/check-code/bags/30618_88aea4d9 results/checker_ros_30618_88aea4d9_final --measure
```

Узел запущен с параметрами по умолчанию: `ros2 launch tram_backup_odometry backup_odometry.launch.py
use_sim_time:=false vehicle_id:=30618` (точка вывода base_link, кадр map, GNSS-коррекция включена).
Что делает скрипт — см. `results/checker_ros_30618_88aea4d9/README.md` и шапку `scripts/run_checker.sh`.

## Итог чекера (финальная сводка при остановке, exit code 0; `checker_final.txt`)

| метрика | RMSE | max | n пар |
|---|---|---|---|
| скорость, м/с | **0.0324** | 0.302 | 38437 |
| позиция x, м | 0.904 | 11.02 | 38437 |
| позиция y, м | 0.645 | 8.67 | 38437 |
| позиция z, м | 0.037 | 0.288 | 38437 |
| позиция 3D, м | **1.111** | 14.02 | 38437 |

Офлайн-эмуляция того же bag (`results/checker_30618_88aea4d9/summary.md`, вариант final): скорость 0.0335 м/с,
3D 1.11 м (max 14.0). Разница в скорости — порядок обработки почти одновременных сообщений тележек в executor rclpy
(разбор в `results/checker_ros_30618_88aea4d9/README.md`).

Узел (`node.log`, сводка при остановке): входы front 12216 / rear 12225 / cmd 26249 / fix master 438, rover 435;
выходы velocity 38476, position 38473; колбэк mean 0.693 мс, p50 0.668, p95 1.585, p99 1.995, max 6.32 мс;
ошибок 0, подмен стампа 0, отброшенных нечисловых выходов 0.

## Реальное время (`latency.txt/json`, `resources.txt/json`)

| | p50 | p95 | p99 | max (после 2 с) | max (всё, включая стартовую пачку bag) |
|---|---|---|---|---|---|
| задержка вход→`/result/velocity`, мс | 1.17 | 2.45 | 3.03 | 22.8 | 32.1 |
| задержка вход→`/result/position`, мс | 1.90 | 3.23 | 3.78 | 24.9 | 32.4 |

- Частота выхода 29.38 Гц (wall) / 29.29 Гц (по стампам); все стампы выходов совпадают со стампами входов
  (38476 / 38473, unmatched 0, повторов 0); покрытие сетки 0.1 с 99.99 % / 99.98 %; frame_id base_link / map.
- CPU узла: среднее 0.071 ядра, p95 0.100, max 0.259 (93.8 с CPU за 1317 с); RSS 62.2 МиБ в начале,
  среднее 64.0, max 64.1 МиБ (роста нет); 11 потоков.
- Требования ТЗ: задержка ≤ 100 мс — выполнено с запасом (max 32 мс), частота ≥ 10 Гц, CPU ≤ 2 ядер, RSS ≤ 0.5 ГБ.
