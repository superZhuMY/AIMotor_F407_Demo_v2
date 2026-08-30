// 括号平衡粗检：剥离注释与字符串/字符字面量后计数 {} 与 ()。
// 快速烟囱检查，捕捉"少半个括号"类低级错误；语义级检查见 syntax_check.js。
// 退出码：0=全部平衡，1=有不平衡文件。
'use strict';
const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..', '..');

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
];

let fail = 0;
for (const p of files) {
  const t = fs.readFileSync(p, 'utf8');
  const b = t
    .replace(/\/\*[\s\S]*?\*\//g, '')
    .replace(/\/\/[^\n]*/g, '')
    .replace(/"(?:[^"\\]|\\.)*"/g, '""')
    .replace(/'(?:[^'\\]|\\.)*'/g, "''");
  const ob = (b.match(/{/g) || []).length, cb = (b.match(/}/g) || []).length;
  const op = (b.match(/\(/g) || []).length, cp = (b.match(/\)/g) || []).length;
  const rel = path.relative(ROOT, p);
  if (ob === cb && op === cp) {
    console.log(`OK   ${rel}`);
  } else {
    fail++;
    console.log(`FAIL ${rel}  {} ${ob}/${cb}   () ${op}/${cp}`);
  }
}
process.exit(fail ? 1 : 0);
