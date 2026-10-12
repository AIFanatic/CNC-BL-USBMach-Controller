const $ = id => document.getElementById(id);
let state = {}, preview = null;
async function api(action, data = {}) {
  const response = await fetch('/api/' + action, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-CNC-UI': '1'}, body: JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  return result;
}
async function perform(action, data) {
  try { await api(action, data); await refresh(); }
  catch (error) { $('message').textContent = error.message; }
}
function buttons() {
  const enabled = state.allow_write && state.connected && !state.busy;
  $('connect').disabled = !state.allow_write || state.busy;
  $('refresh').disabled = !enabled;
  $('run').disabled = !enabled || !preview;
  $('stop').disabled = !state.allow_write || (!state.connected && !state.busy);
  document.querySelectorAll('#jogs button').forEach(button => button.disabled = !enabled);
}
async function refresh() {
  try {
    state = await (await fetch('/api/state')).json();
    $('mode').textContent = state.allow_write ? (state.connected ? 'Connected' : 'Disconnected') : 'Preview only';
    $('message').textContent = state.message + (state.busy && state.progress ? ` · move ${state.progress.move}/${state.progress.total}` : '');
    if (state.position_mm) $('positions').textContent = 'XYZ'.split('').map(axis => `${axis} ${state.position_mm[axis].toFixed(3)}`).join('   ');
    if (state.limits) $('limits').textContent = 'XYZ'.split('').map(axis => {
      const limit = state.limits[axis];
      return `${axis}: ${limit.feed_mm_min.toFixed(1)} mm/min · ${limit.acceleration_mm_s2.toFixed(1)} mm/s² · ${limit.line_distance_mm.toFixed(2)} mm per line`;
    }).join(' | ');
    if (preview) draw(preview.moves, state.program_id === preview.id ? state.tool_mm : null);
    buttons();
  } catch (error) { $('message').textContent = 'Server unavailable'; }
}
for (const axis of 'XYZ') {
  const row = document.createElement('div'); row.className = 'row';
  const title = document.createElement('strong'); title.textContent = axis; row.append(title);
  for (const sign of [-1, 1]) {
    const button = document.createElement('button'); button.textContent = `${axis} ${sign < 0 ? '−' : '+'}`;
    button.onclick = () => {
      const distance = Number($('distance').value), feed = Number($('feed').value);
      if (!(distance > 0 && feed > 0)) { $('message').textContent = 'Distance and feed must be positive'; return; }
      perform('jog', {axis, distance: sign * distance, feed});
    };
    row.append(button);
  }
  $('jogs').append(row);
}
function draw(moves, tool = null) {
  const canvas = $('path'), ctx = canvas.getContext('2d'), points = [[0,0], ...moves.flatMap(m => m.path_xy || [[m.target_mm.X,m.target_mm.Y]])];
  const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  const scale = Math.min((canvas.width-100)/Math.max(maxX-minX, .1), (canvas.height-100)/Math.max(maxY-minY, .1));
  const project = p => [canvas.width/2+(p[0]-(minX+maxX)/2)*scale, canvas.height/2-(p[1]-(minY+maxY)/2)*scale];
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle='#e2ebe7'; ctx.lineWidth=1; ctx.setLineDash([]);
  for(let x=0;x<canvas.width;x+=38){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke();}
  for(let y=0;y<canvas.height;y+=38){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke();}
  let previous = [0,0];
  moves.forEach(m=>{ctx.strokeStyle=m.mode==='G0'?'#b88435':'#16765c';ctx.lineWidth=3;ctx.setLineDash(m.mode==='G0'?[6,5]:[]);ctx.beginPath();ctx.moveTo(...project(previous));const path=m.path_xy || [[m.target_mm.X,m.target_mm.Y]];path.forEach(p=>ctx.lineTo(...project(p)));ctx.stroke();previous=path[path.length-1];});
  const start=project(points[0]);ctx.fillStyle='#263b44';ctx.beginPath();ctx.arc(...start,5,0,Math.PI*2);ctx.fill();ctx.font='13px system-ui';ctx.fillText('Start / zero',start[0]+10,start[1]-10);
  if (tool) {
    const p=project([tool.X,tool.Y]);ctx.setLineDash([]);ctx.strokeStyle='#d35b27';ctx.lineWidth=2;
    ctx.beginPath();ctx.arc(...p,9,0,Math.PI*2);ctx.moveTo(p[0]-14,p[1]);ctx.lineTo(p[0]+14,p[1]);ctx.moveTo(p[0],p[1]-14);ctx.lineTo(p[0],p[1]+14);ctx.stroke();
    ctx.fillStyle='#a43b13';ctx.fillText(`Tool X ${tool.X.toFixed(3)} Y ${tool.Y.toFixed(3)} Z ${tool.Z.toFixed(3)} mm`,16,22);
  }
  ctx.fillText(`X ${(maxX-minX).toFixed(3)} mm × Y ${(maxY-minY).toFixed(3)} mm`,16,canvas.height-16);
}
$('preview').onclick = async () => {
  preview=null; buttons();
  try { preview=await api('preview',{text:$('code').value}); draw(preview.moves); const feeds=preview.moves.map(m=>m.feed_mm_min); $('summary').textContent=`${preview.moves.length} moves · ${preview.seconds.toFixed(2)} s · planned peak feed ${Math.min(...feeds).toFixed(1)}–${Math.max(...feeds).toFixed(1)} mm/min`; buttons(); }
  catch(error){$('summary').textContent=error.message; draw([]);}
};
$('code').oninput = () => { preview=null; $('summary').textContent='Changed — preview again before running'; buttons(); };
$('file').onchange = async event => { const file=event.target.files[0]; if(!file)return; if(file.size>900000){$('summary').textContent='File too large (maximum 900 KB)';return;} $('code').value=await file.text(); $('filename').textContent=file.name; $('preview').click(); };
$('connect').onclick=()=>perform('connect');
$('refresh').onclick=()=>perform('status');
$('stop').onclick=()=>perform('stop');
$('run').onclick=()=>{if(preview)perform('run',{id:preview.id});};
draw([]); refresh(); setInterval(refresh,250);
