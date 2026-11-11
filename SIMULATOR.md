# WRO Future Engineers — simulator with the AGSG firmware in the loop

The browser UI owns the world (arena, walls, signs, parking bay, physics). The
Python process is the brain. By default the brain is the **real firmware** of the
AGSG car (`ImSimao/FE-AGSG-S2026`, MicroPython, base revision `74893b4`),
vendored in `src/robot/` and run under CPython through a hardware shim.

A commit on the firmware repository changes the next simulated round with no
porting step: the same `obstacle_challenge()` state machine drives the virtual car
that will drive the real one.

**Repository:** https://git.severino.xyz/Assistant/wro-sim-agsg (privado). Cada
alteração ao simulador *e* ao firmware vai aqui, com a razão no corpo do commit;
`git log -p src/robot/` mostra só o que é nosso, e a secção
[Firmware: o que é nosso](#firmware-o-que-é-nosso-e-o-que-é-da-equipa) diz o que
cada patch resolve. Um hook `post-commit` faz `git push` best-effort para
`origin` (desligar com `WRO_NO_PUSH=1`).

```

```
browser (physics + scoring)  <--ws-->  src/simulator/  <--json lines-->  src/robot/
   pose, sonar, signs, encoder            controller                 MicroPython code
   timer, laps, score                     shim: machine, bno08x      unmodified
```

## Run it

```bash
cd wro-fe-simulator
./start.sh                 # UI on :8781, websocket on :8780
# open http://localhost:8781/
```

Ports 8765/8000 are commonly taken by other local services, hence 8780/8781
(`UI_PORT`, `SIM_PORT` env vars override).

`./start.sh` installs `websockets` into `.venv` on first run. The brain can be
switched: `python -m simulator.main --controller placeholder` restores the old
stand-in loop that just drives forward.

## What a round does

1. Page connects → the server resets the car into the parking bay and starts a
   fresh firmware interpreter (its state lives in module globals, so each round
   needs a new process).
2. The UI streams heading (50 Hz), encoder delta (100 Hz), the four ranges (20 Hz)
   and every sign in the camera FOV (10 Hz) over the websocket.
3. The controller converts units, feeds the runner process, and turns the
   firmware's `state.target_speed` + `Servo.current_angle` back into a world
   command (`speed`, `steeringRadius = L / tan(delta)`).
4. `src/simulator/ui/lib/race.js` times the round, counts sections and laps, tracks sign and
   wall contact, and scores it with the official WRO 2026 table.

Buttons: **Nova pista** (new seed), **Repetir pista** (same seed, fresh firmware),
**Ruído** (sensor noise + real sonar cones), **1×/2×** (simulation speed).
The seed fixes both the sign layout and the starting direction, so two firmware
versions can be compared on the same track. The bay itself is fixed at the
right-hand end of the starting section (1.55–2.00 m) for both directions — the
firmware assumes it mirrors with the direction, which is issue 4 in
`FIRMWARE_NOTES.md`.

## Units and frames (where these integrations usually die)

| quantity | world (browser) | firmware |
| --- | --- | --- |
| length | metres | cm |
| heading | degrees CCW from +x, absolute | degrees, sign set by `--imu-sign`, zero at boot |
| sonar | metres, labels Front/Rear/Left/Right | cm ×10 in a `0xA5` + XOR frame, `0xFFFF` = no echo, saturates at 250 cm |
| camera | `{lane, state(1 green / 2 red), distance, angleDeg}` | `CAM:cx,cy,COLOR` lines, QQVGA 160×120 with vflip+hmirror |
| encoder | metres travelled this tick (projected on heading) | pulses injected into `encoder._count` (33.2 counts/cm, from PPR 200, ratio 3/5, Ø 32 mm) |
| position | metres, origin bottom-left | absolute cm, mirrored in x when `clockwise == 1` (`303 - odom_x`) |

The sign table in `src/simulator/ui/lib/traffic.js` matches `camera.py:get_traffic_lane_inside_pov()`
exactly (bottom 100/150/200 @ y=60 plus the 240,100 corner of the next lane; left
and right at x=40/60 and 240/260; top at y=240/260), so the mapping between the
world and the firmware's zones is derived, not guessed.

## Knobs (all on `src/simulator/main.py`)

| flag | default | meaning |
| --- | --- | --- |
| `--controller` | `agsg` | `agsg` = real firmware, `placeholder` = old loop |
| `--imu-sign` | `-1` | firmware heading = sign × world heading |
| `--servo-sign` | `1` | which way a positive servo angle steers |
| `--wheelbase-mm` | `70` | AGSG wheelbase |
| `--servo-gain` / `--servo-offset-deg` | `1.0` / `0` | `delta_eff = gain × angle + offset` |
| `--actuator` | `target_speed` | or `pwm`, with `--pwm-cm-per-duty` |

UI query parameters: `?ws=ws://host:port`, `?start=centre|nose|nose_in`,
`?dir=cw|ccw`.

## Bench calibration on the real car (30 min, closes the loop)

`R = L / tan(delta)` assumes the servo at 0° really points straight. It rarely does.

1. Command a fixed servo angle (29°), let the car drive 2 full circles on the
   floor, read total encoder distance `D` and total IMU heading `theta` (≈720°).
   `R = D / theta_rad`. Repeat the other way round and average.
2. Repeat at 15°.
3. Fit `delta_eff = k * delta + delta0` from the two radii
   (`delta_eff = atan(L / R)`), then run with `--servo-gain k --servo-offset-deg delta0`.

Worked example: `R(29°) = 13.0 cm` and `R(15°) = 26.0 cm` give `k = 0.945`,
`delta0 = 0.89°` — a servo that is nearly perfect with a 0.9° offset.
If the measured radii are much larger than 12.6 cm, the real lock is smaller than
the firmware's ±29° and `--servo-gain` around 0.8–0.9 is the honest setting.

`python -m simulator.calibrate` sweeps `imu-sign`, `servo-sign`, start pose and
servo gain in the sim and ranks them by sections completed, wall hits and odometry
error. Its best row is the baseline until the bench numbers exist.

## Headless runs

```bash
.venv/bin/pip install playwright          # one-off; needs the chromium build
.venv/bin/python -m simulator.race_run --runs 3 --seconds 180   # → race-results.json
.venv/bin/python -m simulator.race_run --runs 2 --seconds 60 --shot /tmp/run
```

Each run prints the score breakdown as JSON and leaves a screenshot, so a change
to the firmware can be compared by score and by odometry error (the "fidelity is
a number" metric in the UI).

## Scoring (WRO FE 2026, Obstacle Challenge, max 62)

`1.1` 1 pt per section in the right direction (24) · `1.2` 1 pt per full lap (3) ·
`1.3` 3 pts stopped in the finish section after three laps · `1.4/1.5` 2/4 pts
signs moved or not, only if fewer than three laps · `1.6/1.7` 8/10 pts after three
laps · `1.8.1` 7 pts started inside the parking lot (needs one full lap) ·
`1.8.2` 15 pts parked inside and parallel · `1.8.3` 7 pts parked partly or not
parallel. Round cap 180 s, ties broken by time.

Contact rules enforced by the world: **9.18** moving a wall zeroes the round (the
car stops on contact, and a contact above 8 cm/s counts as moving it);
**9.20** touching or knocking a sign is allowed and only costs 1.4/1.6 — signs do
not block the car; **9.22** signs only count during the three official laps.

Passing a traffic sign on the wrong side **ends the round there and then**: the
firmware documents its line (green on its left, red on its right), the camera
decides which one it drives, and a pass against that line is not a note in a log.
The points earned up to that moment stand — the round is over, it is not zeroed;
`result.signWrongSide` records why it ended.

## Current state — read this before trusting a score

Medido a 1×, 2× e 4× (o multiplicador tem de ser transparente; se não for, é bug):

| seed | voltas | secções | pontos | tempo | estacionamento |
|---|---|---|---|---|---|
| 4111204647 | 3 | 24 | **62** | 112–122 s | 1.8.2 (completo e paralelo) |
| 2026 | 3 | 24 | **62** | 128 s a 2× | 1.8.2 |
| «Repetir pista» a 4× | 3 | 24 | **62/62/62** | 142.7 / 153.3 / 138.2 s |
| varredura de sinais (53 layouts) | 3 | 24 | **40 de 53** | mediana 133 s, máx 151 s | 1.8.2 em todas as que fecham |

A varredura (`src/simulator/tools/sign_sweep.py` + `src/simulator/tools/sweep_report.py`) corre uma ronda por
cada layout de sinais em sentido horário — uma faixa de cada vez, 12 padrões por
faixa (mesma cor nas duas linhas, cores opostas, linhas diferentes, slots
adjacentes, sinal único) mais cinco casos com as quatro faixas iguais.

* 21 passagens de sinais por ronda, **0 do lado errado** nos dois seeds.
* O estacionamento está numa linha de navalha (1.8.2 / 1.8.3 / zero): uma ronda
  isolada não prova nada, são precisas **N≥3**.
* **Aberto:** 1 em cada 3 rondas do seed 2026 a 4× sai da faixa depois da paragem
  de 0,5 s e acaba em "parede movida" (0 pontos) aos 67 s — é margem de
  trajectória, não a escolha da linha.
* **Aberto (o maior):** a faixa de baixo falha 10 dos 12 layouts. A secção tem
  100 cm, o carro arranca a 2 cm do primeiro sinal e cobre-a em ~2 s: quando as
  duas cores são diferentes não há tempo para trocar de linha. Falta um passo de
  "sair da baía já na linha do primeiro sinal" (a cor desse sinal está a 96 cm,
  fora do alcance de 63 cm da câmara).
* **Aberto:** a ronda passou de ~100 s para ~133 s (mediana) quando o modelo da
  câmara ficou fiel: o firmware gasta tempo a olhar para sinais que só vê a 63 cm.
  A 151 s o limite de 180 s está perto.
* **Odometria medida contra a verdade, no mesmo instante simulado:** mediana
  **2,8 cm**, máximo por ronda **6,9 cm** (pior caso 9,5), rumo 9–14°. Parado — sem
  latência de amostra — mediana 2,4 cm e máximo 4,8 cm. O "máximo de 12–19 cm" que
  aparecia antes era a idade da amostra da telemetria, não deriva.
* **Instável:** `bottom_gg_a13` deu 62 / 54 / 0 / 47 em quatro rondas — o que
  falha é sempre o estacionamento (1.8.3 ou parede). É a baía numa linha de
  navalha, não a linha da pista.
* Um layout pode passar numa ronda e falhar na seguinte: os casos que mudaram com
  as últimas correcções deram 62 em 10 de 10 repetições, mas a varredura é uma
  ronda por caso. Antes de acreditar numa falha isolada, repetir.
* **Aberto:** a odometria do firmware em algumas manobras lê ~30 cm a menos do que
  a verdade (POV do 3.º sinal a 83 cm quando o sinal está a 114 cm).
* Nada aqui substitui o robô: o modelo de direcção e a manobra de estacionamento
  continuam a precisar das medidas de bancada.

## Firmware: o que é nosso e o que é da equipa

`src/robot/` é o repositório da equipa (`ImSimao/FE-AGSG-S2026`, base
`74893b4`) com **quatro patches documentados**; tudo o resto é deles. `git log -p
src/robot/` mostra exactamente o que mudámos.

| patch | ficheiro | o que resolve | commit |
|---|---|---|---|
| Linha travada por secção + atribuição de blobs por rumo | `obstacle_challenge.py`, `camera.py` | a linha mudava a meio da secção quando o verde seguinte entrava no FOV (64 cm de desvio) | `9935804` |
| `Field.prepare_lane()` — zona 2 relida a cada entrada na faixa | `field.py`, `obstacle_challenge.py` | a cor do canto do fim da volta 1 escolhia a linha da volta 2: o vermelho de top passado 27 cm por fora | `6b47f6b` |
| Espera pelos quatro sonares no arranque | `obstacle_challenge.py` | a 4× o primeiro sleep é 250 ms e o sonar traseiro ainda vinha a `-1`; a geometria inicial saía inválida | `61cca72` |
| Olhar o centro da faixa à espera da cor + usar a cor encontrada | `obstacle_challenge.py` | 3,0 s parado em cada secção a olhar ao longo da linha de entrada, com o 3.º sinal a 49° fora do FOV; agora 0–600 ms | `5019a3d` |

Instrumentação nossa no firmware (só aparece no export com `WRO_TEL_ECHO` no
controlador): `call`/`end … t=`, `busca:`, `espera cor de saida` / `espera
terminou apos N ms`, `line … odom=`.

Tudo o que o firmware escreve (`telemetry.py`) sai pelo **UART0 que ele partilha
com o frame dos sonares**: 115200, ~11,5 kB/s, e cada escrita bloqueia quando o
anel de transmissão enche. Com a `update_odom()` a escrever 2 linhas longas a
40 Hz, isso é o ciclo de controlo — e a leitura dos sonares, que corre na mesma
thread — à espera do link. Por isso `Telemetry.ENABLED` nasce **desligado na
placa** e **ligado no simulador**, decidido por `sys.implementation.name`
(`"micropython"` vs `"cpython"`). Para o trazer de volta na bancada:
`Telemetry.ENABLED = True` em `src/robot/telemetry.py`, ou `WRO_TELEMETRY=1`
(`=0` força o silêncio) a arrancar o simulador — a variável é lida no arranque de
cada processo de firmware, e há um por ronda. Em qualquer dos casos perdem-se as
linhas do firmware no log e no export; física, sensores e `odom` não mudam.

Uma direcção foi tentada e revertida: pôr a filtragem por tamanho do blob dentro
do firmware (`8e89af2`). O sítio certo é o **emulador da câmara** — `hardware.py`
calcula a largura aparente (`f·5 cm/distância`) e não reporta blobs abaixo de
3 px, como um `find_blobs` real; o frame continua `CAM:cx,cy,COR` e o firmware vê
exactamente o que vê no robô (`1b719d1`).

## Repository layout

```
src/robot/            firmware da equipa + os nossos 4 patches documentados
src/simulator/agsg_shim/      machine.py, bno08x.py, hardware.py, runner.py
src/simulator/agsg_controller.py  websocket <-> firmware bridge and unit conversion
src/simulator/agsg_process.py subprocess plumbing (JSON lines)
src/simulator/main.py          entry point, --controller switch, knobs
src/simulator/race_run.py      headless round runner
src/simulator/calibrate.py     sign/pose/gain sweep
src/simulator/ui/lib/race.js             sections, laps, contact rules, WRO scoring
src/simulator/ui/lib/simulator.js        world physics (Ackermann, braking, collisions)
FIRMWARE_NOTES.md         what the firmware does wrong, with evidence
```

## Exportar uma prova

`Exportar` (JSON) descarrega tudo o que é preciso para discutir a corrida mais
tarde:

- `result` — pontuação, decomposição, voltas, **tempos por volta** (`lapTimesMs`),
  secções, toques na parede, estacionamento, erro do ghost;
- `sim` — seed, sentido, modo de arranque, multiplicador, ruído e a revisão do
  firmware que correu a ronda;
- `fieldSigns` — o layout verdadeiro dos sinais (o sorteio depende do seed);
- `signSightings` — cada vez que um sinal entrou no campo de visão da câmara,
  com a distância e o ângulo que o firmware recebeu e a pose do carro;
- `signPassages` — cada sinal efetivamente passado e o **lado do sinal** por onde
  o carro passou (`side`, medido pela direção de deslocamento — portanto também
  válido em marcha-atrás). `intended_side` e `on_intended_side` dizem se seguiu a
  linha que o firmware documenta (verde pela esquerda do sinal, vermelho pela
  direita); **o lado não é critério de pontuação** — só custa pontos tocar no
  sinal;
- `signHits` — cada toque num sinal, com o instante, o sinal e a pose do carro.
  É isto que troca a 1.5/1.7 pela 1.4/1.6;
- `raceEndedAtMs` — instante em que a ronda terminou. As amostras da trajetória
  depois disto são o carro a continuar a andar até se carregar em Exportar;
- `trajectory` — (t, x, y, heading, velocidade) por passo da física;
- `log` — a telemetria da sessão.

`CSV` descarrega só a trajetória, para folha de cálculo (XY scatter).

## Relatório de margem (`src/simulator/tools/margin_report.py`)

O JSON exportado diz o que aconteceu; este relatório diz **quanto espaço
sobrava**. Constrói a caixa do corpo (12,7 × 6,1 cm, a 3,8 cm do eixo traseiro)
em cada passo da trajetória e mede a folga até às 8 paredes rígidas, às paredes
da baía e a cada sinal do campo:

```bash
.venv/bin/python src/simulator/tools/margin_report.py prova-2026-....json
.venv/bin/python src/simulator/tools/margin_report.py /tmp/*.json --top 5   # compara rondas
```

Imprime, por volta, o tempo, o **caminho percorrido** (comprimento real, não
deslocamento), a velocidade média, a percentagem de tempo abaixo de 15 cm/s e o
tempo em marcha-atrás; por secção, a folga mínima com o instante e o ponto onde
aconteceu; o sinal mais próximo; os contactos com parede; e cada passagem fora
da linha do firmware com o desvio em cm.

Duas leituras que valem mais do que o total: `caminho` maior que o perímetro
(≈9,1 m por volta nesta pista) é linha a zig-zaguear, não velocidade; e um
`marcha-atrás` de segundos na primeira volta é a rotina de correção a pagar-se
em tempo de prova.

## Portabilidade MicroPython (`src/simulator/tools/mpy_audit.py`)

O simulador corre o firmware em **CPython**; a placa corre **MicroPython**, cuja
biblioteca padrão é um **subconjunto**. Uma chamada que só existe no CPython
importa e corre na simulação e só rebenta no carro, a meio de uma ronda:
`os.environ` no arranque e `math.hypot` na primeira rotação já custaram duas
rondas (história em `FIRMWARE_NOTES.md`, Issue 6).

```bash
.venv/bin/python src/simulator/tools/mpy_audit.py              # src/robot/ (30 ficheiros)
.venv/bin/python src/simulator/tools/mpy_audit.py --self-test  # confirma que as regras disparam
```

Lê o firmware e lista cada nome da biblioteca padrão que não exista no
MicroPython, com a substituição a usar (`math.hypot` → `math.sqrt(x*x + y*y)`),
e sai com código 1 — dá para o pôr à frente do flash. O `src/robot/Camera/`
(OpenMV) fica de fora porque corre noutro interpretador; uma utilização
deliberada de uma API só de CPython marca-se na linha com `# mpy-ok`.
