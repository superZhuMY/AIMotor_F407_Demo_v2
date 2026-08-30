// tree-sitter 语法自检：解析 App/Core 的 C/H 文件，并与 git HEAD 基线对比，
// 只报告"基线中不存在的新错误"——规避 tree-sitter 对预处理文本的三类已知误报
//（#if 悬空 else / 字符串宏拼接 / extern "C" 头文件，详见同目录 README.md）。
// 退出码：0=无回归，1=发现回归，2=环境问题。
// 首次使用：cd tools/dev && npm install --no-audit --no-fund
'use strict';
const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

let ts;
try {
  ts = require('web-tree-sitter');
} catch (e) {
  console.error('缺少依赖。请先执行：cd tools/dev && npm install --no-audit --no-fund');
  process.exit(2);
}

const ROOT = path.resolve(__dirname, '..', '..');

// 收集检查目标：App 全部 + Core/Src + Core/Inc 的 .c/.h（不含 Drivers/HAL）
function listSources(dir, out) {
  for (const name of fs.readdirSync(dir)) {
    const p = path.join(dir, name);
    const st = fs.statSync(p);
    if (st.isDirectory()) {
      if (name === 'node_modules') continue;
      listSources(p, out);
    } else if (/\.(c|h)$/i.test(name)) {
      out.push(p);
    }
  }
  return out;
}
const files = [
  ...listSources(path.join(ROOT, 'App'), []),
  ...listSources(path.join(ROOT, 'Core', 'Src'), []),
  ...listSources(path.join(ROOT, 'Core', 'Inc'), []),
].map((p) => path.relative(ROOT, p).replace(/\\/g, '/'));

// 错误签名：不含行号（改动后行号会平移），用节点类型 + 上下文文本区分
function collectErrors(tree) {
  const sigs = [];
  const walk = (node) => {
    if (node.type === 'ERROR') {
      const txt = String(node.text || '').slice(0, 60).replace(/\s+/g, ' ').trim();
      sigs.push(`ERROR: ${txt}`);
    }
    for (let i = 0; i < node.childCount; i++) walk(node.child(i));
  };
  walk(tree.rootNode);
  if (tree.rootNode.hasError) sigs.push('HAS_ERROR');   // 覆盖缺失符号等无 ERROR 节点的情况
  return sigs;
}

function baselineSource(rel) {
  try {
    return execSync(`git show HEAD:${rel}`, { cwd: ROOT, maxBuffer: 32 * 1024 * 1024 }).toString('utf8');
  } catch (e) {
    return null;   // 新文件尚未提交：无基线
  }
}

(async () => {
  await ts.Parser.init();
  const wasm = path.join(__dirname, 'node_modules', 'tree-sitter-c', 'tree-sitter-c.wasm');
  const C = await ts.Language.load(wasm);
  const parser = new ts.Parser();
  parser.setLanguage(C);

  let regressions = 0;
  for (const rel of files) {
    const src = fs.readFileSync(path.join(ROOT, rel), 'utf8');
    const now = collectErrors(parser.parse(src));
    const baseSrc = baselineSource(rel);
    const base = new Set(baseSrc === null ? [] : collectErrors(parser.parse(baseSrc)));
    const news = now.filter((s) => !base.has(s));

    if (baseSrc === null) {
      console.log(`NEW  ${rel}  (无基线：${now.includes('HAS_ERROR') ? 'hasError，含 extern "C"/预处理误报时属正常' : '干净'})`);
      continue;
    }
    if (news.length === 0) {
      const n = now.filter((s) => s !== 'HAS_ERROR').length;
      console.log(`OK   ${rel}  (${n === 0 ? '干净' : `${n} 处已知误报，与基线一致`})`);
    } else {
      regressions += news.length;
      console.log(`FAIL ${rel}  新增语法错误:`);
      news.forEach((s) => console.log(`     ${s}`));
    }
  }
  console.log(regressions === 0 ? '\n语法自检通过（无相对 HEAD 的新错误）' : `\n发现 ${regressions} 处新语法错误`);
  process.exit(regressions === 0 ? 0 : 1);
})().catch((e) => {
  console.error('解析器初始化失败:', e.message);
  process.exit(2);
});
