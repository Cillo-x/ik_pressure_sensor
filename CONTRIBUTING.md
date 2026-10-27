# 贡献指南

## 分支与合并

| 分支 | 用途 | 保护 | 合并方式 |
|---|---|---|---|
| `main` | 发布分支 | PR + `Python style` + `Humble build + test`，禁 force push | merge commit |
| `dev` | 集成分支，日常 PR 的目标 | PR + `Python style`，禁 force push | squash |
| `feat/<topic>`、`fix/<topic>` | 功能与修复，随用随删 | 无 | — |

功能分支合入 `dev` 用 squash，让集成分支上一个提交对应一个改动；  
`dev` 合入 `main`
用 merge commit，保留祖先关系，使下次发版的 diff 只含新增改动。

默认分支是 `main`，开 PR 时需显式指定 `--base dev`。

## 日常流程

```bash
git switch dev && git pull
git switch -c feat/whatever

# ... 修改 ...

git push -u origin feat/whatever
gh pr create --base dev --fill
```

`Python style` 检查通过后即可 squash 合并，合并后删除功能分支。  
合入 `dev`会触发一次完整构建（不阻塞合并）；发往 `main` 的 PR 则必须构建通过才能合并。

## 本地检查

不需要 ROS：

```bash
python -m pip install -r requirements-dev.txt   # 一次性
python -m flake8 .
python -m pytest test -q
```

需要 ROS 2 Humble：

```bash
colcon build --packages-select ik_pressure_sensor
colcon test --packages-select ik_pressure_sensor
colcon test-result --verbose
```

## 提交信息

采用 Conventional Commits, 示例：

```
feat: 支持一次轮询多台从机
fix: 断线重连后重置统计
docs: 补充 udev 配置说明
test: 增加 CRC 异常帧用例
ci: 新增 lint workflow
```

## `main` 发布流程

1. 在 `dev` 上更新 `package.xml` 与 `setup.py` 中的版本号，按普通 PR 合并
2. 发起发版 PR：`gh pr create --base main --head dev --title "release: X.Y.Z"`
3. 两个检查通过后以 merge commit 合并
4. 打标签并创建 Release：

```bash
git switch main && git pull
git tag -a vX.Y.Z -m "ik_pressure_sensor X.Y.Z"
git push origin vX.Y.Z
```

## 不要提交

1. `.gitignore` 中指定的文件与目录
2. 临时调试脚本等
