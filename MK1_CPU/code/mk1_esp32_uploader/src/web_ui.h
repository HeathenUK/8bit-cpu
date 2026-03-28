// MK1 Web IDE — HTML/CSS/JS served from flash

#pragma once

static const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MK1 CPU</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: -apple-system, 'Segoe UI', sans-serif; background: #1a1a2e; color: #e0e0e0; height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
header { background: #16213e; padding: 6px 12px; display: flex; align-items: center; gap: 6px; border-bottom: 1px solid #0f3460; flex-shrink: 0; overflow-x: auto; }
header h1 { font-size: 16px; color: #e94560; font-weight: 700; letter-spacing: 1px; }
header select { background: #0f3460; color: #e0e0e0; border: 1px solid #e94560; border-radius: 4px; padding: 4px 8px; font-size: 13px; }
.btn { border: none; border-radius: 4px; padding: 6px 12px; font-size: 13px; font-weight: 600; cursor: pointer; }
.btn:active { transform: scale(0.97); }
.btn-run { background: #e94560; color: white; }
.btn-run:hover { background: #c73550; }
.btn-sec { background: #0f3460; color: #e0e0e0; border: 1px solid #335; }
.btn-sec:hover { background: #1a4a7a; }
.btn-warn { background: #6a3000; color: #ffa040; border: 1px solid #804000; }
.btn-warn:hover { background: #804000; }
.spacer { flex: 1; }
#status { font-size: 12px; color: #888; }
.btn-step { background: #1a5c3a; color: #4ecca3; border: 1px solid #2a7a4a; }
.btn-step:hover { background: #2a7a4a; }
#cpubar { background: #0d0d1a; border-bottom: 1px solid #0f3460; padding: 4px 12px; font-family: monospace; font-size: 11px; display: flex; gap: 16px; align-items: center; flex-shrink: 0; }
#cpubar .lbl { color: #666; }
#cpubar .val { color: #4ecca3; }
#cpubar .val.off { color: #e94560; }
main { flex: 1; display: flex; flex-direction: column; overflow: hidden; min-height: 0; }
.editor-wrap { flex: 1; display: flex; overflow: hidden; min-height: 0; }
#lines { width: 38px; background: #0d0d1a; color: #555; border: none; padding: 12px 4px 12px 0; font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace; font-size: 14px; line-height: 1.6; text-align: right; overflow: hidden; user-select: none; -webkit-user-select: none; }
#editor { flex: 1; background: #0a0a1a; color: #a8d8a8; border: none; outline: none; padding: 12px 12px 12px 8px; font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace; font-size: 14px; line-height: 1.6; resize: none; tab-size: 2; white-space: pre; overflow-wrap: normal; overflow: auto; }
#output { background: #0d0d1a; border-top: 1px solid #0f3460; padding: 6px 12px; font-family: monospace; font-size: 11px; min-height: 24px; max-height: 120px; overflow-y: auto; white-space: pre-wrap; flex-shrink: 0; }
.err { color: #e94560; }
.ok { color: #4ecca3; }
.info { color: #888; }
</style>
</head>
<body>
<header>
  <h1>MK1 CPU</h1>
  <select id="examples" onchange="loadExample()">
    <option value="">Examples...</option>
    <option value="counter">Counter (inc)</option>
    <option value="countold">Counter (addi)</option>
    <option value="fibonacci">Fibonacci</option>
    <option value="xortest">XOR test</option>
    <option value="swaptest">SWAP + XOR test</option>
    <option value="negtest">NEG test</option>
    <option value="page3">Page 3 test</option>
    <option value="incdec">INC/DEC + JNZ</option>
    <option value="stackrel">Stack-relative (C-style calls)</option>
  </select>
  <button class="btn btn-sec" onclick="save()">Save</button>
  <div class="spacer"></div>
  <span id="status"></span>
  <button class="btn btn-step" onclick="mk1step()">Step</button>
  <button class="btn btn-sec" onclick="mk1resume()">Resume</button>
  <button class="btn btn-warn" onclick="mk1reset()">Reset</button>
  <button class="btn btn-warn" onclick="mk1halt()">Halt</button>
  <button class="btn btn-run" onclick="asmRun()">Assemble &amp; Run</button>
</header>
<div id="cpubar">
  <span><span class="lbl">STATE </span><span class="val" id="cpu-state">---</span></span>
  <span><span class="lbl">CLK </span><span class="val" id="cpu-clk">---</span></span>
  <span><span class="lbl">BUS </span><span class="val" id="cpu-bus">---</span></span>
  <span><span class="lbl">CYCLES </span><span class="val" id="cpu-cycles">---</span></span>
</div>
<main>
  <div class="editor-wrap">
    <pre id="lines">1</pre>
    <textarea id="editor" spellcheck="false" placeholder="; Enter MK1 assembly here..."></textarea>
  </div>
  <div id="output"><span class="info">Ready. Connect to MK1 and press Assemble &amp; Run.</span></div>
</main>
<script>
const examples = {
  counter: '; Simple counter (uses inc)\nmain:\n  inc\n  out\n  j main\n',
  countold: '; Simple counter (original addi)\nmain:\n  addi 1 $a\n  out\n  j main\n',
  fibonacci: '; Fibonacci sequence\ninit:\n  ldi $a 1\n  ldi $b 1\n\nloop:\n  out $a\n  mov $a $c\n  add $b $a\n  jc end\n  mov $c $b\n  j loop\n\nend:\n  hlt\n',
  xortest: '; XOR test (no swap, isolates XOR)\n; Should show: 255, then 240, then halt\n  ldi $a 0xFF\n  ldi $b 0x0F\n  out $a       ; display 255\n  xor          ; A = 0xFF XOR 0x0F = 0xF0 = 240\n  out $a       ; display 240\n  hlt\n',
  swaptest: '; SWAP + XOR test\n; Should cycle: 255, 15, 240, 15...\n  ldi $a 0xFF\n  ldi $b 0x0F\n\nloop:\n  out $a\n  xor\n  swap\n  j loop\n',
  negtest: '; NEG test: negate counter\n; Counts 1,255,2,254,3,253...\n  ldi $a 0\n\nloop:\n  addi 1 $a\n  out $a\n  neg\n  out $a\n  neg\n  j loop\n',
  page3: '; Page 3 test: store/load from page 3\n; Stores 42 to page 3 addr 0, reads it back\n  ldi $a 42\n  stp3 0\n  ldi $a 0\n  out $a       ; display 0\n  ldp3 0\n  out $a       ; should display 42\n  hlt\n',
  incdec: '; INC/DEC test\n; Count up to 5, then down to 0, repeat\n  ldi $a 0\n\nup:\n  inc\n  out\n  cmp 5\n  jnz up         ; keep going if A != 5\n\ndown:\n  dec\n  out\n  cmp 0\n  jnz down       ; keep going if A != 0\n  j up\n',
  stackrel: '; Stack-relative load demo\n; Shows C-style function calls with arguments\n; max(a,b) accesses args via ldsp\n;\n; Expected output: 25, 200\n\n  ; max(10, 25)\n  ldi $a 10\n  push $a          ; arg1\n  ldi $a 25\n  push $a          ; arg2\n  jal max\n  pop $d           ; clean arg2\n  pop $d           ; clean arg1\n  out              ; display 25\n\n  ; max(200, 150)\n  ldi $a 200\n  push $a          ; arg1\n  ldi $a 150\n  push $a          ; arg2\n  jal max\n  pop $d           ; clean arg2\n  pop $d           ; clean arg1\n  out              ; display 200\n  hlt\n\n; max(arg1, arg2) -> $a\n; Stack: [SP+1]=ret, [SP+2]=arg2, [SP+3]=arg1\nmax:\n  ldsp 3           ; A = arg1\n  mov $a $b        ; B = arg1\n  ldsp 2           ; A = arg2\n  cmp $b           ; arg2 vs arg1\n  jc .done         ; CF=1: arg2 >= arg1, keep A\n  mov $b $a        ; arg1 was bigger\n.done:\n  ret\n',
};

const ed = document.getElementById('editor');
const ln = document.getElementById('lines');

function updateLines() {
  const count = ed.value.split('\n').length;
  let s = '';
  for (let i = 1; i <= count; i++) s += i + '\n';
  ln.textContent = s;
}

function syncScroll() { ln.scrollTop = ed.scrollTop; }

ed.addEventListener('input', updateLines);
ed.addEventListener('scroll', syncScroll);

function loadExample() {
  const sel = document.getElementById('examples');
  if (sel.value && examples[sel.value]) { ed.value = examples[sel.value]; updateLines(); }
  sel.value = '';
}

function log(msg, cls) {
  const o = document.getElementById('output');
  o.innerHTML += '<span class="' + (cls||'info') + '">' + esc(msg) + '</span>\n';
  o.scrollTop = o.scrollHeight;
}

function esc(s) { return s.replace(/&/g,'&amp;').replace(/</g,'&lt;'); }

async function asmRun() {
  const o = document.getElementById('output');
  o.innerHTML = '';
  document.getElementById('status').textContent = 'Assembling...';
  try {
    const resp = await fetch('/assemble', {
      method: 'POST', headers: {'Content-Type': 'text/plain'}, body: ed.value
    });
    const result = await resp.json();
    if (result.errors && result.errors.length > 0) {
      result.errors.forEach(e => log('Line ' + e.line + ': ' + e.message, 'err'));
      document.getElementById('status').textContent = 'Errors';
      return;
    }
    const codePct = Math.round(result.code_size / 256 * 100);
    const dataPct = Math.round(result.data_size / 256 * 100);
    log('Code: ' + result.code_size + '/256 bytes (' + codePct + '%)  Data: ' + result.data_size + '/256 bytes (' + dataPct + '%)', 'ok');
    if (result.code_hex) {
      // Format hex dump with address column: 00: XX XX XX XX ...
      const bytes = result.code_hex.split(' ');
      let hexLines = '';
      for (let i = 0; i < bytes.length; i += 16) {
        const addr = i.toString(16).toUpperCase().padStart(3, '0');
        const chunk = bytes.slice(i, i + 16).join(' ');
        hexLines += addr + ': ' + chunk + '\n';
      }
      log('Code:', 'info');
      log(hexLines.trimEnd(), 'info');
      if (result.data_hex) {
        const dbytes = result.data_hex.split(' ');
        let dLines = '';
        for (let i = 0; i < dbytes.length; i += 16) {
          const addr = i.toString(16).toUpperCase().padStart(3, '0');
          const chunk = dbytes.slice(i, i + 16).join(' ');
          dLines += addr + ': ' + chunk + '\n';
        }
        log('Data:', 'info');
        log(dLines.trimEnd(), 'info');
      }
    }
    document.getElementById('status').textContent = 'Uploading...';

    const resp2 = await fetch('/upload', { method: 'POST' });
    const r2 = await resp2.json();
    if (r2.ok) {
      log('Uploaded and running!', 'ok');
      document.getElementById('status').textContent = 'Running';
    } else {
      log('Upload failed: ' + (r2.error || 'unknown'), 'err');
      document.getElementById('status').textContent = 'Error';
    }
  } catch(e) {
    log('Error: ' + e.message, 'err');
    document.getElementById('status').textContent = 'Error';
  }
}

async function mk1reset() {
  await fetch('/reset', { method: 'POST' });
  document.getElementById('status').textContent = 'Reset';
}

async function mk1halt() {
  await fetch('/halt', { method: 'POST' });
  document.getElementById('status').textContent = 'Halted';
}

async function mk1step() {
  try {
    const r = await fetch('/step', { method: 'POST' });
    const j = await r.json();
    if (j.ok) {
      document.getElementById('cpu-bus').textContent = j.bus + ' (0x' + j.bus.toString(16).toUpperCase().padStart(2,'0') + ')';
      document.getElementById('cpu-cycles').textContent = j.cycles;
    }
  } catch(e) {}
}

async function mk1resume() {
  await fetch('/resume', { method: 'POST' });
  document.getElementById('status').textContent = 'Running';
}

// Status polling
function formatClk(hz) {
  if (hz < 1) return 'stopped';
  if (hz < 1000) return hz.toFixed(1) + ' Hz';
  if (hz < 1000000) return (hz/1000).toFixed(1) + ' kHz';
  return (hz/1000000).toFixed(2) + ' MHz';
}

async function pollStatus() {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    const stEl = document.getElementById('cpu-state');
    stEl.textContent = s.state.toUpperCase();
    stEl.className = 'val' + (s.state === 'halted' ? ' off' : '');
    document.getElementById('cpu-clk').textContent = formatClk(s.clock_hz);
    document.getElementById('cpu-bus').textContent = s.bus + ' (0x' + s.bus.toString(16).toUpperCase().padStart(2,'0') + ')';
    document.getElementById('cpu-cycles').textContent = s.cycles;
  } catch(e) {}
}

setInterval(pollStatus, 1000);

async function save() {
  try {
    await fetch('/save', {
      method: 'POST', headers: {'Content-Type': 'text/plain'}, body: ed.value
    });
    log('Saved to flash', 'ok');
  } catch(e) { log('Save failed: ' + e.message, 'err'); }
}

async function loadSaved() {
  try {
    const resp = await fetch('/load');
    if (resp.ok) {
      const text = await resp.text();
      if (text.length > 0) { ed.value = text; updateLines(); }
    }
  } catch(e) {}
}

// Tab key inserts spaces
ed.addEventListener('keydown', function(e) {
  if (e.key === 'Tab') {
    e.preventDefault();
    const s = this.selectionStart;
    this.value = this.value.substring(0, s) + '  ' + this.value.substring(this.selectionEnd);
    this.selectionStart = this.selectionEnd = s + 2;
    updateLines();
  }
});

// Load saved program on startup
loadSaved();
updateLines();
</script>
</body>
</html>
)rawliteral";
