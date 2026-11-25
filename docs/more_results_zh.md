# 更多结果

**[English](more_results.md) | 简体中文**

RoboColiseum 第一赛季、RoboTwin 2.0、LIBERO-Pro 2.0 和 LIBERO-PRO 的运行方法、协议和我们的结果。其他基准和真实机器人见
[README](../README_zh.md#results)。命令均在仓库根目录下运行；环境见 [scripts/setup/README.md](../scripts/setup/README.md)。

成功率（%），使用 `gpt-6-astra`，推理强度 `high`，时间为 2026 年 10 月至 11 月。RoboTwin 2.0、LIBERO-Pro 2.0 和
LIBERO-PRO 为自行评测；RoboColiseum 由其平台运行和评分。无技能时不为任何任务编写任何内容；有技能时把该任务的一页指导加入提示词。
“其他结果”为各基准排行榜上的最好成绩，或论文报告的结果，各自基于其自身的回合与协议。

| 基准 | 回合数 | 无技能 | 有技能 | 其他结果 |
|---|---|---|---|---|
| [RoboColiseum](https://robocoliseum.ai/robocoliseum/leaderboard?season=season1&model_type=all) 第一赛季（智元 G2，Genie Sim 3.5）：四个榜，由平台运行和评分；总分 × 100（[详情](#robocoliseum1)） | – | – | **第一**，总分 **98.45** | MXI SimWorld v0 Preview 96.56（排行榜） |
| [RoboTwin 2.0](https://robotwin-platform.github.io/leaderboard)：50 个任务 × 15 个种子，随机化场景，未见指令（[详情](#robotwin)） | 750 | 84.9 | **96.8** | HumanWAM-Goal\* 86.5（排行榜）；OmniBrain-0 报告 90.4 |
| [LIBERO-Pro 2.0](https://racingemperor.github.io/LIBERO-Pro2.0/leaderboard.html)：339 个测试实例，测试种子 152–161（[详情](#liberopro2)） | 3,390 | 75.6 | **90.8** | CloudRobo-WM 81.7（排行榜） |
| [LIBERO-PRO](https://github.com/Zxy-MLlab/LIBERO-PRO)：156 个任务 × 10 个初始状态（[详情](#liberopro)） | 1,560 | 78.3（验证状态 10–19） | **96.7**（测试状态 20–29） | HarnessPAI 96.5；ASPIRE 使用 GPT-6-Astra 为 96.0（论文，各自的子集） |

- 在 RoboTwin 2.0 和 LIBERO-Pro 2.0 上，两列只在技能上不同，回合相同；在 LIBERO-PRO 上，两列的回合也不同。
- 在 RoboTwin 2.0 上，我们运行随机化设置，每个任务 15 个回合；官方评测有两种设置，每种设置每个任务 100 个回合。在 LIBERO-Pro 2.0
  上，我们运行了基准示例评测 30 个种子中的十个。
- 排行榜日期：LIBERO-Pro 2.0 为 2026 年 11 月 17 日，RoboColiseum 和 RoboTwin 2.0 为 11 月 25 日。HumanWAM-Goal 的星号表示使用 GPT
  Image 2.5 生成的目标图像。

<a id="robocoliseum1"></a>

## RoboColiseum 第一赛季

Genie Sim 3.5，智元 G2 与 omnipicker 夹爪；四个榜，`instruction`（指令）、`spatial`（空间）、`manip`（操作）和 `robust`（鲁棒）；
12 份技能（[skills/robocoliseum1/](../skills/robocoliseum1/)）。与[第二赛季](../README_zh.md#robocoliseum-eval)一样，基准是客户端，
由 HexaShell 在本地或在比赛平台上应答它的请求。本地回合按布局分为学习、验证和测试三部分，只有指定了该部分（`--part`）时，一次运行
才会打开验证或测试布局。用 `HEXASHELL_SKILLS=1` 打开技能。

```bash
source config/env.sh
# 本地，需要 Genie Sim 3.5：一个布局及其全部指令
HEXASHELL_SKILLS=1 $HEXASHELL_PYTHON -m hexashell.benches.robocoliseum1.run --config g2op_if_pick_block_color --layout 0
# 线上，不需要仿真器：在某个榜上提交任务，再为它启动 K 个智能体
$HEXASHELL_PYTHON -m hexashell.benches.robocoliseum1.challenge login
$HEXASHELL_PYTHON -m hexashell.benches.robocoliseum1.challenge submit --board spatial --name <name>    # 加 --yes 才会提交
bash hexashell/benches/robocoliseum1/online.sh spatial 4 results/online_spatial
```

| [排行榜](https://robocoliseum.ai/robocoliseum/leaderboard?season=season1&model_type=all)，分数 × 100，2026 年 11 月 25 日 | 总分 | 指令 | 空间 | 操作 | 鲁棒 |
|---|---|---|---|---|---|
| **HexaShell** | **98.45** | **99.80** | 99.58 | **94.53** | **99.87** |
| MXI SimWorld v0 Preview | 96.56 | 99.65 | **99.79** | 86.97 | 99.85 |
| FF Omnimind | 94.55 | 98.61 | 98.54 | 82.55 | 98.51 |
| PyRUA-Lean | 82.77 | 99.25 | 97.92 | 48.97 | 84.92 |

HexaShell 于 2026 年 11 月 10 日以总分 98.45 位列第一。

`python -m hexashell.benches.robocoliseum1.batch` 在多台机器上规划、运行并汇总本地批量运行；`challenge` 还可以查看账号、配额、
已提交的任务及其分数，以及失败任务的日志。两个模块的文件开头都写有用法。请使用你自己的账号和模型密钥。

<a id="robotwin"></a>

## RoboTwin 2.0

运行器和评分与 [RoboCasa365](../README_zh.md#robocasa-eval) 相同，列表每行的第一个词为 `robotwin`：

```bash
source config/env.sh
$HEXASHELL_PYTHON -m hexashell.benches.run --bench robotwin --task adjust_bottle --seed 100000 --set skills=yes \
    --out results/demo
# 测试列表：750 个回合
mkdir -p logs && $HEXASHELL_PYTHON -c 'from hexashell.benches.robotwin.protocol import test_episodes
for task, seed in test_episodes(): print("robotwin", task, seed)' > logs/twin_test.todo
SETS="skills=yes" python3 scripts/bench_pool.py twin_test 0,0,1,1 4
python3 scripts/bench_score.py results/twin_test
```

测试遵循 RoboTwin 自身对 clean2random 设置的评测：`demo_randomized` 场景，各任务的步数上限（400–1700 步；一步是一次
`take_action`，即 aloha-agilex 机器人双臂的关节目标），取自任务未见描述模板的指令，以及每个任务从 100000 起由
RoboTwin 的专家策略能够完成的前若干个种子，按 RoboTwin 评测的方式查找（`scripts/setup/scan_robotwin_seeds.py`；
结果为 `hexashell/benches/robotwin/seeds.json`）。我们为每个任务取前 15 个：共 750 个回合，每个回合回复最多 40 次，
时间最多两小时。100000 以下的种子用于开发，每个任务 10 个用于学习、10 个用于验证，使用已见模板中的指令；技能只
使用这些措辞。任务为 RPent 评测列表（`eval_seeds.json`，见 NOTICE）中的 50 个。场景构建时对 RoboTwin 的运行时
做了三处修正（见 `hexashell/benches/robotwin/protocol.py`）。感知只使用彩色图像和相机模型，与 RoboDojo 相同
（`--set depth=true` 时也读取深度图）。两次测试都使用 `config/benches.yml` 的设置，图片窗口为
20/14。

- 在相同的回合上，**有技能 726/750（96.8%），无技能 637/750（84.9%）**：101 个回合由失败变为成功，12 个由成功变为失败。有技能时
  36 个任务为 15/15；有 29 个任务两次运行都是 15/15。
- 在验证轮之后修订过技能的 15 个任务，有技能 205/225，无技能 142/225；其余 35 个任务为 521/525 和 495/525。无技能时，两臂各拿起
  一个瓶子的两个任务（pick_diverse_bottles、pick_dual_bottles）15 个回合全部失败，place_dual_shoes 失败 14 个；有技能时这三个任务
  各通过 14 个。
- 有技能的 24 次失败中，14 次由模型自行结束，7 次用完回复，3 次用完步数；无技能的 113 次失败中，三者分别为 52、6 和 55。每个回合
  平均回复次数为有技能 8.4，无技能 12.0。
- 我们没有运行 clean2clean（`demo_clean`，已见指令）。

<details>
<summary>有技能时的逐任务结果（15 次中的成功数）</summary>

| 任务 | | 任务 | | 任务 | |
|---|---|---|---|---|---|
| adjust_bottle | 15 | open_microwave | 15 | place_object_stand | 15 |
| beat_block_hammer | 15 | pick_diverse_bottles | 14 | place_phone_stand | 14 |
| blocks_ranking_rgb | 15 | pick_dual_bottles | 14 | place_shoe | 15 |
| blocks_ranking_size | 15 | place_a2b_left | 15 | press_stapler | 15 |
| click_alarmclock | 15 | place_a2b_right | 15 | put_bottles_dustbin | 12 |
| click_bell | 15 | place_bread_basket | 14 | put_object_cabinet | 13 |
| dump_bin_bigbin | 14 | place_bread_skillet | 14 | rotate_qrcode | 14 |
| grab_roller | 15 | place_burger_fries | 15 | scan_object | 12 |
| handover_block | 15 | place_can_basket | 15 | shake_bottle | 15 |
| handover_mic | 15 | place_cans_plasticbox | 15 | shake_bottle_horizontally | 15 |
| hanging_mug | 12 | place_container_plate | 15 | stack_blocks_three | 15 |
| lift_pot | 15 | place_dual_shoes | 14 | stack_blocks_two | 15 |
| move_can_pot | 15 | place_empty_cup | 15 | stack_bowls_three | 11 |
| move_pillbottle_pad | 15 | place_fan | 15 | stack_bowls_two | 15 |
| move_playingcard_away | 15 | place_mouse_pad | 15 | stamp_seal | 15 |
| move_stapler_pad | 15 | place_object_basket | 14 | turn_switch | 15 |
| open_laptop | 15 | place_object_scale | 15 |  |  |

</details>

<a id="liberopro2"></a>

## LIBERO-Pro 2.0

[LIBERO-Pro 2.0](https://github.com/LIBERO-Pro-Team/LIBERO-Pro2.0)（[论文](https://arxiv.org/abs/2610.24587)）每次对 LIBERO 的任务
施加一种扰动：22 种静态扰动在开始时就存在，20 种动态扰动是回合中发生的事件，共分六类。它的测试集有 339 个实例，来自八个任务，
每个套件两个。每个实例都从初始状态 0 开始，种子决定扰动的抽取。在基准示例评测的 30 个种子（142–171）中，我们用 142–146 学习，
147–151 验证，152–171 测试。报告的测试为前十个种子 152–161：3,390 个回合。

适配层遵循官方评测：相同的重置方式；官方动作（手的相对位姿和夹爪），经由基准自身的 step 函数执行；其步数上限（Spatial、Object、
Goal 和 LIBERO-10 分别为 20 Hz 下的 220、280、300 和 520 步）；以及每一步之后的成功检查。此外，每个回合最多 40 次回复、两小时。
智能体看到前方相机和腕部相机的 512 像素 RGB 图片，没有深度，还有观测中的各项读数。它的相机模型是标称的，相机扰动会使其失准。它
不知道当前施加的是哪种扰动：任务卡写明套件，当前的指令随每次回复给出。

```bash
source config/env.sh
$HEXASHELL_PYTHON -m hexashell.benches.run --bench liberopro2 --task libero_goal_t3_S15 --seed 142 --set skills=yes \
    --out results/demo
mkdir -p logs && python3 scripts/liberopro2_lists.py test --seeds 152-161 > logs/libero2_test.todo   # 3390 个回合
SETS="skills=yes" python3 scripts/bench_pool.py libero2_test 0,0,1,1 4
python3 scripts/bench_score.py results/libero2_test      # 排行榜的各项分数：Base、Static、Dynamic、Overall
```

任务的命名为 `<suite>_t<i>_<case>`；测试任务为 spatial 的 0 和 8、object 的 1 和 8、goal 的 3 和 6、libero_10 的 5 和 8。
`skills/liberopro2/` 中每个任务有一份技能，用于它的所有扰动。一个格子（cell）是一种扰动下的一个套件；与排行榜相同，Base 为四个
无扰动格子的平均，Static 和 Dynamic 分别为 87 个和 80 个格子的平均，各类别为其格子的平均，Overall 为全部 167 个有扰动格子的平均
（成功率，%；排行榜条目截至 2026 年 11 月 17 日，各自基于其自身的回合）：

| | HexaShell，无技能 | HexaShell，有技能 | CloudRobo-WM | Lingbot-VA | MolmoAct2 |
|---|---|---|---|---|---|
| Base | 83.8 | **100.0** | 99.6 | 98.5 | – |
| **Static** | 74.7 | **90.3** | 77.8 | 68.7 | 71.2 |
| – Condition | 68.5 | **85.0** | 73.3 | 52.2 | 63.8 |
| – Environment | 77.9 | **94.1** | 88.5 | 86.1 | 85.0 |
| – Observation | 78.8 | **97.1** | 60.7 | 35.7 | 65.1 |
| – Execution | 77.5 | **88.8** | 30.4 | 86.7 | 0.0 |
| – Robot | 73.8 | 83.1 | 96.9 | **98.3** | 85.0 |
| – Language | 86.2 | **100.0** | 98.8 | 97.1 | 94.6 |
| **Dynamic** | 76.6 | **91.4** | 86.0 | 81.8 | 76.9 |
| – Condition | 67.1 | **81.0** | 67.8 | 77.3 | 58.6 |
| – Environment | 82.1 | **97.9** | 86.9 | 81.5 | 74.2 |
| – Observation | 81.9 | **99.1** | 94.2 | 90.5 | 90.3 |
| – Execution | 75.0 | 86.2 | **99.6** | 96.8 | 90.4 |
| – Robot | 82.1 | **97.5** | 92.5 | 59.3 | 79.0 |
| – Language | 85.0 | **100.0** | 98.8 | 96.7 | 94.2 |
| **Overall** | 75.6 | **90.8** | 81.7 | 75.0 | 73.9 |

- **成功数。** 3,390 个回合中，无技能成功 2,572 个，有技能成功 3,090 个；技能使 573 个回合由失败变为成功，55 个由成功变为失败。
  在全部 20 个测试种子上（只有有技能的版本运行过），总成功率为 90.9（6,780 个回合中成功 6,191 个）。
- 差距的一半以上来自一个任务：在 `libero_goal_t3`（打开最上层抽屉并把碗放进去）中，无技能的智能体 420 个回合全部失败，有技能时
  成功 322 个。
- 有技能时每个回合平均 6.1 次回复、95 秒，无技能时 8.5 次回复、119 秒。有技能的 300 次失败中，226 次用完步数，74 次由智能体自行
  结束。
- 测试之后，对失败最多的格子修订了技能。修订只在新的种子（172–191）上检验过，不计入上述结果；`skills/liberopro2/` 中是修订后的
  技能。

<a id="liberopro"></a>

## LIBERO-PRO

运行器和评分与 [RoboCasa365](../README_zh.md#robocasa-eval) 相同，列表每行的第一个词为 `libero`：

```bash
source config/env.sh
$HEXASHELL_PYTHON -m hexashell.benches.run --bench libero --task libero_goal_swap_t3 --seed 20 --set skills=yes \
    --out results/demo
# 测试列表：1560 个回合
mkdir -p logs && $HEXASHELL_PYTHON -c 'from hexashell.benches.libero.protocol import TEST_SEEDS, test_tasks
for task in test_tasks():
    for seed in TEST_SEEDS: print("libero", task, seed)' > logs/libero_test.todo
SETS="skills=yes" python3 scripts/bench_pool.py libero_test 0,0,1,1 4
python3 scripts/bench_score.py results/libero_test
```

LIBERO-PRO 对 LIBERO 的四个套件（spatial、object、goal、10）分别施加单一类型的扰动。我们测试其中四种扰动：物体
起始位置互换（`_swap`）、物体外观改变（`_object`）、指令改写（`_lan`），以及同一场景中的另一个任务（`_task`）：
共 16 个套件，每个十个任务（`<suite>_t<i>`，例如 `libero_goal_swap_t3`），其中四个任务在 LIBERO-PRO 的发布版本中
没有初始状态，因此共 156 个任务。种子 `s` 从该任务的初始状态 `s` 开始；种子 0–9 用于学习，10–19 用于验证，20–29
用于测试：共 1560 个回合。场景先稳定 10 个控制步，与 LIBERO-PRO 自身的评测一致；之后适用该套件的步数上限（20 Hz
控制步：spatial 220，object 280，goal 300，libero_10 520），回复最多 40 次，时间最多两小时。成功以 LIBERO 的目标
检查为准。有技能的 71 个任务是在学习轮（无技能）或验证轮中失败过的任务。技能在开发种子上学习和检验；测试种子的记录没有用于编写
技能。测试使用 benches.yml 的设置，打开技能，图片窗口为 20/14：

| 套件 | `_swap` | `_object` | `_lan` | `_task` | 全部 |
|---|---|---|---|---|---|
| libero_spatial | 97/100 | 90/100 | 96/100 | 77/80 | 360/380 |
| libero_object | 96/100 | 100/100 | 100/100 | 100/100 | 396/400 |
| libero_goal | 99/100 | 100/100 | 98/100 | 100/100 | 397/400 |
| libero_10 | 94/100 | 79/90 | 96/100 | 87/90 | 356/380 |
| **全部** | 386/400 | 369/390 | 390/400 | 364/370 | **1509/1560 (96.7%)** |

- 有技能的任务为 696/710，其余任务为 813/850（这些任务在学习中从未失败，因此在新的初始状态上有所下降在意料之中）。
- **无技能**时，在验证用初始状态（10–19）上为 1222/1560（78.3%），由两部分组成：当时没有技能的 102 个任务取自验证
  轮，其余 54 个任务另以关闭技能的方式运行。这 54 个任务即第一批技能所针对的任务（编写这些技能时未见过验证状态），
  无技能时为 252/540，有技能时为 531/540（逐回合 +286/−7）。

  | 套件，无技能 | `_swap` | `_object` | `_lan` | `_task` | 全部 |
  |---|---|---|---|---|---|
  | libero_spatial | 87/100 | 83/100 | 82/100 | 74/80 | 326/380 |
  | libero_object | 92/100 | 70/100 | 82/100 | 84/100 | 328/400 |
  | libero_goal | 76/100 | 78/100 | 84/100 | 68/100 | 306/400 |
  | libero_10 | 70/100 | 56/90 | 80/100 | 56/90 | 262/380 |
  | **全部** | 325/400 | 287/390 | 328/400 | 282/370 | **1222/1560 (78.3%)** |

- 全部 51 次失败都是用完步数或结束时剩余步数不足。libero_10_object_t3 十次全部失败（每一轮都如此；它的碗似乎太宽，
  抽屉无法在碗上合拢）。
