<!--
短小、可执行的 Copilot 指南，目标是让 AI 代理在该仓库里尽快开始有价值的工作。
如果仓库随后添加了代码或已有内容，请先合并已有文件中有用的片段而不是覆盖。
-->

# Copilot / AI 代理 使用说明（仓库扫描模式）

说明：我当前在仓库中未发现可供直接分析的源代码文件（例如 [README.md](README.md)、[package.json](package.json)、[pyproject.toml](pyproject.toml)、`src/` 等）。因此本文件偏重于“如何在空/稀疏仓库里展开有效探索并与开发者协作”。

1. 初始检查（优先级高）
- **确认仓库状态**：运行 `git status`、`git rev-parse --show-toplevel`、`git ls-files` 来确认分支与跟踪文件。
- **快速列举潜在入口文件**：寻找 `README.md`, `package.json`, `pyproject.toml`, `requirements.txt`, `setup.py`, `go.mod`, `Cargo.toml`, `Dockerfile`, `docker-compose.yml`。
- **如果没有以上文件**：向人类请求仓库目的、首要语言/框架、以及希望 AI 完成的具体任务（例如“实现 API”、“修复测试”或“添加 CI”）。

2. 当发现代码/文件时的第一组动作（可运行命令示例）
- **识别语言/工具链**：
  - Node.js: `cat package.json | jq -r '.name,.scripts'`
  - Python: 查找 `pyproject.toml` / `requirements.txt` / `setup.py`
  - Go/Rust: 查找 `go.mod` / `Cargo.toml`
- **构建与测试（只读/侦查）**：在提出修改前，先运行项目推荐的测试/构建命令（如存在）：`npm test` / `npm run build` / `pytest -q` / `go test ./...`。

3. 代码库结构与要关注的模式
- 如果出现 `services/`, `cmd/`, `internal/` 或 `pkg/`，优先把这些目录视为边界（service / cli / library）。
- 如果存在 `docker-compose.yml` 或 `k8s/`，说明项目以多服务部署为主，优先辨别服务间的接口（REST/gRPC/消息队列）。
- 如果存在 `infra/`、`terraform/`、`charts/`，将其标记为部署/运维相关并避免在无明确权限下修改。

4. 合并原则（当仓库已有 `.github/copilot-instructions.md`）
- 不要覆盖已有内容。把新增发现（本文件的条目）合入已有章节，保留原作者的“为什么/约定”说明。

5. 与开发者互动的建议（具体问题模板）
- 项目目标："这个仓库的主要功能或预期产物是什么？"
- 语言/工具链："首选语言、测试与构建命令是什么？"
- 优先级："要我先做哪三件事？（例如：修 CI、补单元测试、实现某个接口）"

6. 安全与边界
- 不要在没有授权的情况下推送含凭据的更改或修改 infra 脚本（`terraform/`, `cloud/`）。

7. 若需我继续：
- 请告知仓库目标或上传/打开主要源码文件；我会：
  1) 自动生成项目概览（架构 + 关键文件清单），
  2) 推荐最小可行任务清单，
  3) 起草 PR/补丁并运行相关本地测试（如果你允许我运行命令）。

---
请告诉我是否要把我刚生成的检查脚本或自动扫描命令添加为 CI 步骤，或把这份说明合并到现有的 README 中。需要我合并或修改哪一部分请指出。
