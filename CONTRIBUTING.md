# 贡献指南

## 分支模型

| 分支 | 用途 | 保护规则 |
|---|---|---|
| `main` | 发布分支，只接受已经过完整构建的代码 | 必须走 PR；`Lint` + `Humble build + test` 都必须通过；禁止直接 push、禁止 force push |
| `dev` | 集成分支，日常 PR 的目标 | 必须走 PR；`Lint` 必须通过；禁止 force push |
| `feat/<topic>`、`fix/<topic>` | 功能 / 修复 | 无保护，随用随删 |

仓库的**默认分支保持 `main`**：这样别人 `git clone` 拿到的是稳定可用的版本，
也符合"clone 下来直接能用"的定位。代价是开 PR 时要手动把目标分支选成 `dev`
（`gh pr create --base dev`，网页里改下拉框）。

## 日常工作流

```bash
git switch dev && git pull
git switch -c feat/whatever

# ... 改代码 ...

python -m pip install -r requirements-dev.txt   # 一次性
python -m flake8 .                              # 本地先过一遍风格
python -m pytest test -q                        # 112 passed, 2 skipped

git push -u origin feat/whatever
gh pr create --base dev --fill                  # 或网页操作
```

PR 上的 `Lint`（约 30 秒）通过后即可 squash 合并。合并后删除功能分支。

合并到 `dev` 会触发一次完整的 ROS 构建（push 事件），不阻塞合并，但红了要尽快修 ——
`main` 的 PR 会强制要求同一次构建通过。

## 发布流程

```bash
# 1. dev 上的内容通过 PR 合入 main
gh pr create --base main --head dev --title "release: 0.2.0"

# 2. 合并后打标签
git switch main && git pull
git tag -a v0.2.0 -m "ik_pressure_sensor 0.2.0"
git push origin v0.2.0
```

发版前记得同步改 `package.xml` 与 `setup.py` 里的 `<version>`，并在 GitHub 上把标签做成 Release。

## 本地检查

不需要 ROS（纯 Python 部分）：

```bash
python -m pip install -r requirements-dev.txt
python -m flake8 .
python -m pytest test -q
```

需要 ROS 2 Humble（完整构建 + ament lint）：

```bash
colcon build --packages-select ik_pressure_sensor
colcon test --packages-select ik_pressure_sensor
colcon test-result --verbose
```

## 提交信息

用 Conventional Commits：

```
feat: 支持一次轮询多台从机
fix: 修正断线后重连不重置统计的问题
docs: 补充 udev 配置说明
test: 增加 CRC 异常帧用例
ci: 新增 lint workflow
```

## 不要提交的东西

- 厂家的说明书 PDF、例程工程、HEX、调试软件（第三方版权资料，不随仓库分发）
- `build/`、`install/`、`log/`、`__pycache__/`、`.pytest_cache/`（`.gitignore` 已覆盖，别用 `git add -f` 绕过）
- 任何临时调试脚本
