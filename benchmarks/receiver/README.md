# Независимые проверки приёмников

Эти программы — воспроизведение конкретного контрольного сценария 2025-06-05
10:05:30 UTC. Они не передают RF. Сборки и журналы держать в `~/storage`, IQ —
на USB 2 ТБ. Описание результатов: `docs/GNSS_RESIDUAL_REVIEW.md`.

## Galileo: неполная пара не является ошибкой CRC

```sh
python3 benchmarks/receiver/inav_startup_repro.py /path/to/galileo_inav_message.cc
```

Нужен исходный файл GNSS-SDR **v0.0.20**; SHA проверяется скриптом. Он извлекает
реальный `split_page`, компилирует его с наблюдаемым счётчиком вызовов CRC и
проверяет три случая: одиночная odd-половина, полная верная пара, полная неверная
пара. В первом случае CRC не вызывается. Малый патч рядом исправляет логирование
в `galileo_telemetry_decoder_gs.cc`; к системному GNSS-SDR он не применялся.

## B1C: декодер LDPC/CRC

PocketSDR revision: `03787da53c65a8eb291eec571b6412ec470aeb17`.
Собрать его `lib/build` и `app/pocket_trk` с `USE_SOAPY=0 USE_FFTW=1`.
Локальный `make install` в `lib/build` копирует библиотеки в `lib/linux` самого
checkout, а не в системные каталоги.

`check_b1c.c` связывается с `libsdr.a`/`librtk.a` и библиотеками fftw3f, usb-1.0,
m, pthread. Вход: `tests/fixtures/b1c_reference_symbols.txt`. Ожидается
`LDPC ... SF2=0 SF3=0; CRC SF2=1 SF3=1`. Полярность soft symbols в PocketSDR
специфична для BCNAV1; здесь 1 → 0, 0 → 255.

В `pocket_trk` необходимо `max_acq = 10` в `-opt` файле: дефолт 4 ms не
разрешает самостоятельный поиск 10-ms B1C-кода. Формат IQ этого проекта —
`-fmt CS8`, а не Q-инвертированный `INT8X2`. Для быстрого чтения `-tscale 10`;
значение 0 не означает «максимальная скорость».

Контроль B1C: `-sig B1CD -prn 21,22,34,36,38,39,44,45 -f 4.2 -fo 0`.

## PVT B1C без исходного RINEX

Текущая проверенная версия PocketSDR декодирует BCNAV1, но не передаёт эти
эфемериды в собственный PVT так же, как D1/D2. Поэтому используется адаптер:

```sh
python3 benchmarks/receiver/b1c_to_observations.py /path/to/receiver_folder
# compile b1c_pvt.c against PocketSDR lib/RTKLIB/src and lib/linux/librtk.a
/path/to/b1c_pvt /path/to/receiver_folder/rtklib_input.txt
```

В папке нужен `nav.log` с `$NAV` и `$OBS`. Python разбирает SF2 по независимой
разметке GNSS-SDR/ICD, C вызывает RTKLIB `pntpos` только с принятыми параметрами
и псевдодальностями. Он не читает исходный RINEX. Эпоха в C зафиксирована для
этого теста: GPST week 2369 / 381948 s. Рассматриваются эпохи с 46 s, когда
контрольные кадры уже поступили. Адаптер намеренно отклоняет ненулевые Adot/
Δn-dot: выбранные legacy RINEX-эфемериды имеют нулевые производные; перенос
этого теста на другие данные требует расширить модель RTKLIB, а не молча
отбросить эти поля. Iono/trop corrections в данном сравнении OFF.
