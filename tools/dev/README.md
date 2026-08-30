# 开发环境说明与本地自检工具（Windows 实测）

> 2026-08 在本机（Windows 11 + git bash + PowerShell 7）实测结论。目的：下次改代码后
> 不装编译器也能做基础自检，并避免重复踩环境坑。

## 一、本机工具链现状

| 工具 | 状态 | 说明 |
|------|------|------|
| git bash | ✅ | 日常 shell；注意 `/tmp` 是 MSYS 虚拟路径 |
| node / npm | ✅ v24.x | `D:\Program Files\nodejs`；本目录脚本依赖它 |
| PowerShell 7 (`pwsh`) | ✅ 7.6.5 | `powershell`（5.1）也可用 |
| Python | ❌ | `python`/`python3` 是 WindowsApps 占位 stub：**退出码 49、零输出**，`LOCALAPPDATA\Programs\Python` 为空，WSL 未安装 → `tools/protocol_test.py` 本机跑不了 |
| gcc / clang / make | ❌ | PATH 中均无 → 无法本地编译或 gcc 语法检查 |
| Keil MDK + ARMCLANG | ✅（IDE） | 固件的最终编译/验收门 |

**补装建议**（按需）：真 Python 用 `winget install Python.Python.3.12`；gcc 用
MSYS2/MinGW-w64，装好后即可恢复返修报告里 `gcc -fsyntax-only` 的老检查法。

## 二、踩坑记录与规避（原因 → 对策）

1. **Python Store stub 静默失败**：`python -c "..."` 无输出且退出码 49。对策：判断
   Python 可用性要看"有输出"，不能只看无报错；或干脆装真 Python。
2. **node 不认 git-bash 的 `/tmp`**：node.exe 是 Windows 程序，`require('/tmp/...')`
   实际解析到 `C:\tmp\...` 不存在；`cygpath -w` 还可能给出 `CYBERS~1` 短名。对策：
   **脚本内一律用 `__dirname` 相对定位或正斜杠 Windows 绝对路径**（本目录脚本即此写法）。
3. **bash 内联脚本的转义黑洞**：`node -e "...${var}..."` 的 `$` 会被 bash 先展开，
   `\r\n`、`\\` 等也会被吃掉/改写。对策：复杂逻辑写成 .js 文件再 `node 文件` 执行，
   不要 `-e` 内联。
4. **pwsh -Command 的 `$` 同理被 bash 展开**：整段命令用**单引号**包住 bash 层。
5. **npm 装原生模块的 allow-scripts 警告**：`tree-sitter` 系列的 node-gyp 安装脚本被
   拦截，但本工具用的是 **WASM**（`web-tree-sitter` + `tree-sitter-c.wasm`），
   不需要原生构建，警告可忽略。
6. **git CRLF 警告 / git log 中文乱码**：`LF will be replaced by CRLF` 是
   autocrlf 自动转换提示；log 乱码只是控制台显示编码问题（GBK 终端显示 UTF-8），
   仓库内数据均为 UTF-8 无损。
7. **tree-sitter 解析 C 的三类已知误报**（对原始文本做语法分析、不做预处理所致）：
   - `#if ... #endif` 包裹的悬空 `else`（CtrlSeqTick 的 DRY_RUN 分支）
   - 字符串 + 宏 + 字符串的拼接行（自检 STestOut）
   - 带 `extern "C" {` 的头文件 hasError（aimotor.h/mwmotor.h 同样报）
   对策：`syntax_check.js` **自动与 git HEAD 基线对比**，只报告"基线没有的新错误"，
   上述误报在基线里已存在，不会被误报为回归。

## 三、本目录工具

| 命令 | 作用 | 依赖 |
|------|------|------|
| `node tools/dev/balance_check.js` | 括号平衡粗检（App/Core 全部 C/H） | 无 |
| `node tools/dev/syntax_check.js`  | tree-sitter 语法解析，**对比 HEAD 基线只报新错误** | 首次：`cd tools/dev && npm install --no-audit --no-fund` |
| `python tools/protocol_test.py`   | 上位机协议回归（60 项） | 真 Python（见上） |

退出码约定：`0` 通过，`1` 发现回归，`2` 环境问题（缺依赖等）。
新文件未提交时无基线可比，请先 commit 再检查（README 已注明）。

## 四、能力边界（实测教训）

`balance_check.js`/`syntax_check.js` 都只做**语法结构**层检查，查不出语义错误——
首次 Keil 编译暴露的正是这类：调用先于定义的隐式声明、static/非 static 重声明、
链接期符号缺失。拆分/搬迁代码后，除跑本目录脚本外，**必须以 Keil 编译为最终门**。
跨文件新符号统一收敛在 `App/Inc/aimotor_internal.h` 声明，新加符号时先声明再使用。
