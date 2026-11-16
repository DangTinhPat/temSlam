import {api, escapeHTML} from './api.js';
const normalize = text => text.toLocaleLowerCase('vi').normalize('NFD').replace(/[\u0300-\u036f]/g,'').replace(/đ/g,'d');
const main = document.querySelector('#main');
const route = location.pathname.split('/')[1] || 'home';
const arrow = '<span class="arrow" aria-hidden="true">↗</span>';
const img = name => `/Images_attachments/${name}`;
const searchIcon = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" aria-hidden="true"><circle cx="10.8" cy="10.8" r="6.5"/><path d="m16 16 4.5 4.5"/></svg>';
const breadcrumb = name => `<div class="breadcrumb"><a href="/">RB Lab</a> &nbsp; / &nbsp; ${name}</div>`;
const reportBack = '<div class="breadcrumb"><a href="/">Báo cáo</a> &nbsp; / &nbsp; <a href="/library/report">Về báo cáo SLAM 3D</a></div>';
const heading = (overline,title,text,extra='') => `<div class="page-heading"><div><div class="eyebrow">${overline}</div><h1>${title}</h1><p>${text}</p></div>${extra}</div>`;
const card = doc => `<a class="document-card" href="/library/${encodeURIComponent(doc.id)}"><div class="document-top"><span class="file-icon">${escapeHTML(doc.kind.toUpperCase())}</span><span class="pill outline">${escapeHTML(doc.category)}</span></div><h3>${escapeHTML(doc.title)}</h3><p>${escapeHTML(doc.summary)}</p><div class="document-bottom"><span>${escapeHTML(doc.tags.slice(0,2).join(' · '))}</span><span>${escapeHTML(doc.created.split('-').reverse().join('.'))} &nbsp; ↗</span></div></a>`;

function enableImageZoom() {
  main.querySelectorAll('[data-zoom],.article-content img').forEach(image => {
    image.tabIndex=0; image.setAttribute('role','button');
    image.setAttribute('aria-label', 'Phóng lớn: '+(image.alt || 'Ảnh tài liệu'));
  });
}
async function home() {
  document.title = 'Báo cáo — RB Lab';
  const query = new URLSearchParams(location.search);
  main.innerHTML = `<div class="container report-home">
    ${heading('Không gian báo cáo','Báo cáo.','Chọn một báo cáo để đọc, khám phá kết quả và xem tài liệu liên quan.','<a class="btn primary" href="/admin?new=report"><span aria-hidden="true">＋</span> Thêm báo cáo</a>')}
    <div class="library-tools report-tools"><div class="search-field">${searchIcon}<input id="report-search" type="search" aria-label="Tìm báo cáo" placeholder="Tìm theo tiêu đề, mô tả hoặc chủ đề…" value="${escapeHTML(query.get('q') || '')}"></div><select id="report-topic" aria-label="Lọc chủ đề"><option value="">Tất cả chủ đề</option></select><select id="report-sort" aria-label="Sắp xếp báo cáo"><option value="newest">Ngày đăng mới nhất</option><option value="title">Tên báo cáo A–Z</option></select></div>
    <div class="library-count"><span id="report-count" role="status">Đang tải báo cáo…</span><a href="/library">Xem toàn bộ tài liệu ↗</a></div>
    <div id="report-grid" class="document-grid report-grid section"></div>
    <div class="report-help"><span>Báo cáo mới xuất bản sẽ được cập nhật tại đây.</span><a class="text-link" href="/admin">Quản lý bài đăng ↗</a></div>
  </div>`;
  const reports = (await api('/api/documents')).filter(doc => doc.category === 'Báo cáo');
  const search = document.querySelector('#report-search'), topic = document.querySelector('#report-topic'), sort = document.querySelector('#report-sort');
  const tags = [...new Set(reports.flatMap(doc => doc.tags))].sort((a,b) => a.localeCompare(b,'vi'));
  topic.insertAdjacentHTML('beforeend', tags.map(tag => `<option value="${escapeHTML(tag)}">${escapeHTML(tag)}</option>`).join(''));
  topic.value = tags.includes(query.get('topic')) ? query.get('topic') : '';
  sort.value = query.get('sort') === 'title' ? 'title' : 'newest';
  const addCard = `<a class="add-report-card" href="/admin?new=report" aria-label="Thêm báo cáo mới"><span class="add-report-icon" aria-hidden="true">＋</span><h3>Thêm báo cáo mới</h3><p>Tải tệp, thêm tiêu đề và mô tả.<br>Lưu nháp hoặc xuất bản khi sẵn sàng.</p><span class="text-link">Tạo bài đăng <span aria-hidden="true">→</span></span><small>Dành cho quản trị viên</small></a>`;
  function filter() {
    const value = normalize(search.value.trim());
    const matches = reports.filter(doc => (!topic.value || doc.tags.includes(topic.value)) && normalize([doc.title,doc.summary,...doc.tags].join(' ')).includes(value));
    matches.sort((a,b) => sort.value === 'title' ? a.title.localeCompare(b.title,'vi') : b.created.localeCompare(a.created) || a.title.localeCompare(b.title,'vi'));
    document.querySelector('#report-count').textContent = `${matches.length} báo cáo${matches.length !== reports.length ? ' / '+reports.length+' đã xuất bản' : ' đã xuất bản'}`;
    const empty = `<div class="empty-state"><h3>${reports.length ? 'Chưa tìm thấy báo cáo phù hợp.' : 'Chưa có báo cáo nào được xuất bản.'}</h3><p>${reports.length ? 'Thử một từ khóa hoặc chủ đề khác.' : 'Bắt đầu bằng thẻ Thêm báo cáo mới bên cạnh.'}</p>${reports.length ? '<button class="btn" id="clear-reports">Xóa bộ lọc</button>' : ''}</div>`;
    document.querySelector('#report-grid').innerHTML = addCard + (matches.length ? matches.map(card).join('') : empty);
    const params = new URLSearchParams();
    if(search.value)params.set('q',search.value);if(topic.value)params.set('topic',topic.value);if(sort.value==='title')params.set('sort','title');
    history.replaceState(null,'',`/${params.size?'?'+params:''}`);
    const clear = document.querySelector('#clear-reports');
    if(clear)clear.onclick=()=>{search.value='';topic.value='';filter();search.focus();};
  }
  search.oninput=filter;topic.onchange=filter;sort.onchange=filter;filter();
}
function viewerImportError() {
  document.querySelector('.viewer-host').innerHTML='<div class="viewer-error" role="alert">Chưa tải được thư viện 3D. <a class="btn" href="/library/report">Xem hình ảnh trong báo cáo</a></div>';
}
function explore() {
  document.title = 'Khám phá bản scan 3D — RB Lab';
  main.innerHTML = `<div class="container">${reportBack}${heading('Không gian tương tác','Khám phá bản scan 3D.','Một căn phòng, tái tạo từ RGB và 64 vùng đo ToF. Tự chọn góc nhìn và theo dõi quỹ đạo camera đã ghi.','<span class="pill"><span class="status-dot"></span>Dữ liệu ghi sẵn</span>')}
    <section class="workbench" aria-label="Trình xem 3D"><div class="workbench-stage"><div id="scan-viewer" class="viewer-host"></div><div class="viewer-label">ROOM / FLOOR PRESERVED · 12.11.2026</div><div class="workbench-toolbar"><button id="top-view" class="tool-button" data-viewer-control disabled>Nhìn từ trên</button><button id="reset-view" class="tool-button" data-viewer-control disabled>↺ Đặt lại góc</button></div><div class="viewer-bottom"><span>KÉO: XOAY · CUỘN: ZOOM · CHUỘT PHẢI: DỊCH</span></div></div>
    <aside class="inspector"><section><h3>Lớp hiển thị</h3><label class="check"><span><i class="layer-dot"></i>Mesh bề mặt</span><input id="layer-mesh" type="checkbox" checked data-viewer-control disabled></label><label class="check"><span><i class="layer-dot route"></i>Quỹ đạo camera</span><input id="layer-route" type="checkbox" checked data-viewer-control disabled></label><label class="check"><span>Lưới tham chiếu</span><input id="layer-grid" type="checkbox" checked data-viewer-control disabled></label><label class="check"><span>Khung tam giác</span><input id="wireframe" type="checkbox" data-viewer-control disabled></label></section>
    <section><h3>Thông tin bản scan</h3><dl><dt>Khung fusion</dt><dd>3.198</dd><dt>Voxel</dt><dd>4 cm</dd><dt>Nguồn depth</dt><dd>RGB–ToF</dd><dt>Bề mặt</dt><dd>TSDF</dd><dt>Hệ tọa độ</dt><dd>Map · mét</dd></dl></section><section><p class="note">Mesh đã dựng hoàn tất. Thanh thời gian phát lại pose camera; không mô phỏng quá trình dựng map trực tiếp.</p></section></aside>
    <div class="playback"><button class="btn small" id="play" aria-pressed="false" data-viewer-control disabled>▷ Phát</button><div class="playback-track"><label class="sr-only" for="seek">Vị trí phát lại</label><input id="seek" type="range" min="0" max="3197" value="0" data-viewer-control disabled><div class="playback-meta"><span id="frame-label">Đang tải quỹ đạo…</span><span id="time-label">—</span></div></div><select id="speed" aria-label="Tốc độ phát lại" data-viewer-control disabled><option value="1">×1</option><option value="10">×10</option><option value="30" selected>×30</option><option value="60">×60</option></select></div></section>
    <div class="provenance"><p>Pose: wheel/IMU + correction visual có gate + surface matching sau quét. Khung bổ sung dùng pose truyền từ anchor; đây không phải lượt tracking mới cho toàn bộ 3.198 khung.</p><a class="text-link" href="/library/status">Nguồn & giới hạn ↗</a></div>
    <div class="info-grid"><div><h3>01 / Khám phá hình học</h3><p>Xoay mô hình để xem các bề mặt đã quan sát. Những vùng thiếu bằng chứng vẫn để mở.</p></div><div><h3>02 / Theo dấu camera</h3><p>Chấm màu cam là vị trí camera ước lượng; mũi tên chỉ hướng nhìn. Đầu quay có thể đổi hướng camera khi thân đứng yên.</p></div><div><h3>03 / Đọc đúng kết quả</h3><p>Mesh có lọc và làm mịn. Hình ảnh liền mạch không thay thế đánh giá độ chính xác.</p></div></div></div>`;
  import('./viewer.js').then(m => m.mountViewer(document.querySelector('#scan-viewer'),true)).catch(viewerImportError);
}
const stages = [
  {name:'Cảm biến',sub:'RGB · ToF · encoder · IMU',title:'Quan sát thế giới từ robot.',tag:'01 / THU NHẬN & ĐỒNG BỘ',text:'Camera cung cấp chi tiết hình ảnh, ToF đo khoảng cách theo 64 vùng. Encoder, IMU và góc cổ mô tả chuyển động của thân và cụm cảm biến.',input:'Cảnh vật lý trong MuJoCo',output:'RGB 640×480 · ToF 8×8 · odometry',image:'Hinh03_vung_phu_ToF.png',alt:'Vùng phủ ToF được chiếu lên ảnh RGB',note:'ToF hiện không thêm nhiễu ngẫu nhiên. Calibration là của mô phỏng; cần hiệu chuẩn lại khi chuyển sang robot thật.'},
  {name:'Depth fusion',sub:'DELTAR & kiểm tra ToF',title:'Từ phép đo thưa đến depth dày.',tag:'02 / ƯỚC LƯỢNG KHOẢNG CÁCH',text:'DELTAR dự đoán độ sâu theo pixel từ RGB và thống kê ToF. Bộ xử lý căn bias theo vùng đo, kiểm tra tính phù hợp và loại giá trị mâu thuẫn.',input:'Ảnh RGB + mean/sigma ToF + calibration',output:'Depth fusion cùng hệ pixel với RGB',image:'Hinh08_lia_dau_truoc_sau.png',alt:'Quan sát các góc lia đầu và dữ liệu fusion',note:'Độ sâu dày là suy luận của mạng, không phải mỗi pixel đều được ToF đo độc lập. Giá trị không hợp lệ vẫn để rỗng.'},
  {name:'Định vị',sub:'cuVSLAM & wheel–IMU',title:'Đặt mỗi góc nhìn vào đúng chỗ.',tag:'03 / POSE CAMERA',text:'cuVSLAM theo dõi RGB-D. Odometry bánh và IMU cung cấp chuyển động cục bộ; correction visual đi qua gate trước khi hiệu chỉnh hệ map. Động học bù vị trí camera khi đầu quay.',input:'RGB-D + wheel/IMU + encoder cổ',output:'Pose camera trong hệ tọa độ map',image:'Hinh04_tuyen_quet.png',alt:'Tuyến quét cố định và quỹ đạo trong phòng',note:'Giả thiết nền ngang; correction chủ yếu X/Y/yaw. Fusion nằm ngoài cuVSLAM, chưa phải VIO tích hợp. Tuyến quét dùng scene mô phỏng đã biết.'},
  {name:'Dựng bề mặt',sub:'TSDF & Marching Cubes',title:'Tích lũy quan sát thành không gian.',tag:'04 / TÁI TẠO HÌNH HỌC',text:'Mỗi depth được tích phân vào TSDF theo pose camera. Khi có đủ hỗ trợ từ nhiều góc nhìn, Marching Cubes trích bề mặt tam giác; bộ lọc loại thành phần nhỏ và làm mịn.',input:'Depth fusion + pose các keyframe',output:'Mesh tam giác · STL / PLY',image:'20261113-102624.jpg',alt:'Mesh 3D phòng khách đã được tái tạo',note:'Live/finalize dùng voxel tối thiểu 4 cm. Vùng không có dữ liệu vẫn có thể thủng; bộ lọc cũng có thể làm mất chi tiết.'},
  {name:'Hoàn tất',sub:'Surface matching & đối chiếu',title:'Tinh chỉnh rồi đo lại kết quả.',tag:'05 / SAU PHIÊN QUÉT',text:'Surface matching trên các mặt gần đứng tinh chỉnh X/Y/yaw theo từng frame, sau đó dựng lại TSDF. Kết quả được lưu kèm nguồn dữ liệu để đánh giá riêng depth, pose và reconstruction.',input:'Keyframe fusion + pose đã lưu',output:'Mesh cuối & báo cáo đánh giá',image:'Hinh06_do_phu_san_toan_phong.png',alt:'So sánh hỗ trợ mesh trước và sau bổ sung quan sát',note:'Đây là tinh chỉnh tuần tự sau quét, chưa phải tối ưu pose graph toàn cục hoặc matching 6-DoF realtime.'}
];
function technology() {
  document.title='Công nghệ RGB–ToF — RB Lab';
  main.innerHTML=`<div class="container">${reportBack}${heading('Bên trong hệ thống','Theo dấu một quan sát.','Chọn từng khối để khám phá dữ liệu đi từ cảm biến đến bản đồ bề mặt 3D.')}<div class="pipeline-layout"><div class="pipeline-nav" aria-label="Các khối công nghệ">${stages.map((s,i)=>`<button class="pipeline-node" data-stage="${i}" aria-pressed="${i===0}"><span>0${i+1}</span><div><strong>${s.name}</strong><small>${s.sub}</small></div></button>`).join('')}</div><section id="pipeline-detail" class="pipeline-detail" aria-live="polite"></section></div><div class="library-preview section"><div><div class="eyebrow">Ranh giới dữ liệu</div><h2>Hiểu nguồn trước khi đọc kết quả.</h2></div><div><p>Map sử dụng RGB, ToF, calibration và pose ước lượng. Ground truth phục vụ đánh giá ngoại tuyến. ROS/RViz là lớp hiển thị; lõi điều khiển và mapping chạy native.</p><a href="/library/architecture" class="text-link">Đọc kiến trúc đầy đủ ${arrow}</a></div></div></div>`;
  function show(index) {
    const s=stages[index];
    document.querySelectorAll('[data-stage]').forEach(b=>b.setAttribute('aria-pressed',String(Number(b.dataset.stage)===index)));
    document.querySelector('#pipeline-detail').innerHTML=`<span class="pill outline">${s.tag}</span><h2>${s.title}</h2><p>${s.text}</p><div class="io-grid"><div class="io-box"><span>ĐẦU VÀO</span>${s.input}</div><div class="io-box"><span>ĐẦU RA</span>${s.output}</div></div><img class="tech-image" data-zoom src="${img(s.image)}" alt="${s.alt}"><p class="tech-foot">Ảnh từ dữ liệu dự án · Bấm để phóng lớn</p><div class="note">${s.note}</div>`;
    enableImageZoom();
  }
  document.querySelectorAll('[data-stage]').forEach(b=>b.onclick=()=>show(Number(b.dataset.stage)));show(0);
}
const experiments = [
  {title:'Thêm góc nhìn toàn phòng',date:'12.11.2026',caption:'912 → 3.198 khung',source:'room-results',image:'Hinh06_do_phu_san_toan_phong.png',text:'Bổ sung 2.286 góc nhìn từ dữ liệu đã ghi. Giữ voxel 4 cm và min_views = 3.',note:'Tỷ lệ ô XY có mesh gần sàn trong ROI, không phải độ phủ sàn thật. RMSE chỉ tính trên các tam giác vượt bộ chọn gần sàn. Đây là replay, chưa xác nhận tracking toàn tuyến.',type:'coverage'},
  {title:'Phân rã sai số gần sàn',date:'12.11.2026',caption:'32 khung · vùng bên phải',source:'floor-analysis',image:'Hinh07_phan_ra_sai_so.png',text:'Tách ảnh hưởng của depth và pose trên mẫu sát tường phải bằng evaluator có ground truth.',note:'32 frame đã tham gia map, không phải tập kiểm tra độc lập. Hai thành phần có tương quan, không cộng tuyến tính; không đại diện sai số toàn phòng.',type:'error'}
];
function experimentsPage() {
  document.title='Thí nghiệm & đối chiếu — RB Lab';
  main.innerHTML=`<div class="container">${reportBack}${heading('Đo lường & đối chiếu','Để kết quả tự lên tiếng.','Chọn thí nghiệm để xem điều kiện, số liệu và bằng chứng. Mỗi chỉ số đi cùng phạm vi áp dụng.')}<div class="experiment-layout section"><aside class="experiment-nav" aria-label="Danh sách thí nghiệm">${experiments.map((e,i)=>`<button data-experiment="${i}" aria-pressed="${i===0}"><small class="mono">${e.date} · OFFLINE</small><strong>${e.title}</strong><small>${e.caption}</small></button>`).join('')}</aside><section id="experiment-detail" class="experiment-main" aria-live="polite"></section></div></div>`;
  function show(index) {
    const e=experiments[index];
    document.querySelectorAll('[data-experiment]').forEach(b=>b.setAttribute('aria-pressed',String(Number(b.dataset.experiment)===index)));
    const metrics=e.type==='coverage'?`<div class="metric-panel"><h3>Ô có mesh gần sàn trong ROI</h3><div class="bar-row old"><span>Trước</span><div class="bar-track"><i style="width:74.96%"></i></div><strong>74,96%</strong></div><div class="bar-row"><span>Sau</span><div class="bar-track"><i style="width:82.91%"></i></div><strong>82,91%</strong></div><p>Tăng 7,95 điểm phần trăm trong khung đo.</p></div><div class="metric-panel"><h3>RMSE cao độ tam giác gần sàn</h3><div class="metric-pair"><div class="old"><strong>2,30 <small>cm</small></strong><small>TRƯỚC BỔ SUNG</small></div><span>→</span><div><strong>1,84 <small>cm</small></strong><small>SAU BỔ SUNG</small></div></div><p>Trên các tam giác được chọn theo điều kiện gần sàn.</p></div>`:`<div class="metric-panel"><h3>RMS cao độ do depth</h3><div class="metric-pair"><div><strong>2,60 <small>cm</small></strong></div></div><p>Dùng pose chuẩn để cô lập ảnh hưởng của depth fusion.</p></div><div class="metric-panel"><h3>RMS cao độ do pose</h3><div class="metric-pair"><div><strong>0,17 <small>cm</small></strong></div></div><p>Dùng depth chuẩn để cô lập ảnh hưởng của pose ước lượng.</p></div>`;
    document.querySelector('#experiment-detail').innerHTML=`<div class="experiment-top"><h2>${e.title}</h2><span class="pill outline">Đã có số liệu</span></div><p class="muted">${e.text}</p><div class="metric-grid">${metrics}</div><div class="spec-row"><span class="pill outline">Dữ liệu mô phỏng</span><span class="pill outline">Đánh giá offline</span><span class="pill outline">${e.caption}</span></div><figure class="plot"><img src="${img(e.image)}" alt="${e.title}" data-zoom><figcaption>Biểu đồ từ evaluator của dự án · Bấm để xem lớn.</figcaption></figure><div class="note">${e.note}</div><a class="text-link" href="/library/${e.source}">Đọc phương pháp và báo cáo nguồn ${arrow}</a>`;
    enableImageZoom();
  }
  document.querySelectorAll('[data-experiment]').forEach(b=>b.onclick=()=>show(Number(b.dataset.experiment)));show(0);
}
async function library() {
  document.title='Thư viện nghiên cứu — RB Lab';
  const query=new URLSearchParams(location.search);
  main.innerHTML=`<div class="container">${heading('Tri thức của dự án','Thư viện nghiên cứu.','Báo cáo, hướng dẫn và ghi chú kỹ thuật. Tìm điều bạn cần, đi sâu vào điều bạn quan tâm.')}<div class="library-tools"><div class="search-field">${searchIcon}<input id="library-search" type="search" placeholder="Tìm tài liệu, chủ đề, từ khóa…" aria-label="Tìm tài liệu" value="${escapeHTML(query.get('q')||'')}"></div><div class="filter-tabs" aria-label="Lọc loại tài liệu">${['Tất cả','Báo cáo','Công nghệ','Hướng dẫn','Dữ liệu'].map(c=>`<button data-category="${c}" aria-pressed="${c===(query.get('category')||'Tất cả')}">${c}</button>`).join('')}</div></div><div class="library-count"><span id="result-count" role="status">Đang tải…</span><span>MỚI NHẤT TRƯỚC</span></div><div id="documents" class="document-grid section"></div></div>`;
  const docs=await api('/api/documents');
  let category=query.get('category')||'Tất cả';
  if (!['Tất cả','Báo cáo','Công nghệ','Hướng dẫn','Dữ liệu'].includes(category)) category='Tất cả';
  const search=document.querySelector('#library-search');
  function filter() {
    const value=normalize(search.value.trim());
    const matches=docs.filter(d=>(category==='Tất cả'||d.category===category)&&normalize([d.title,d.summary,...d.tags].join(' ')).includes(value));
    document.querySelector('#result-count').textContent=`${matches.length} / ${docs.length} tài liệu`;
    document.querySelector('#documents').innerHTML=matches.map(card).join('')||'<div class="empty-state"><h3>Chưa tìm thấy tài liệu phù hợp.</h3><p>Thử từ khóa ngắn hơn hoặc chọn danh mục khác.</p><button class="btn" id="clear-search">Xóa bộ lọc</button></div>';
    document.querySelectorAll('[data-category]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.category===category)));
    const params=new URLSearchParams();if(search.value)params.set('q',search.value);if(category!=='Tất cả')params.set('category',category);
    history.replaceState(null,'',`/library${params.size?'?'+params:''}`);
    const clear=document.querySelector('#clear-search');if(clear)clear.onclick=()=>{category='Tất cả';search.value='';filter();search.focus();};
  }
  search.oninput=filter;document.querySelectorAll('[data-category]').forEach(b=>b.onclick=()=>{category=b.dataset.category;filter();});filter();
  if(query.get('focus')==='search')search.focus();
}
async function article(id) {
  const doc=await api('/api/documents/'+encodeURIComponent(id));
  document.title=doc.title+' — RB Lab';
  let body=doc.html;
  if(doc.kind==='pdf')body=`<span class="pill">PDF</span><h1>${escapeHTML(doc.title)}</h1><p>${escapeHTML(doc.summary)}</p><p>Tải bản PDF để đọc đầy đủ, in hoặc lưu lại.</p><a class="btn primary" href="${doc.file_url}">Tải tài liệu PDF ↓</a>`;
  else if(['png','jpg','jpeg','webp'].includes(doc.kind))body=`<h1>${escapeHTML(doc.title)}</h1><p>${escapeHTML(doc.summary)}</p><img src="${doc.file_url}" alt="${escapeHTML(doc.title)}" data-zoom>`;
  else if(['mp4','webm'].includes(doc.kind))body=`<h1>${escapeHTML(doc.title)}</h1><p>${escapeHTML(doc.summary)}</p><video controls preload="metadata" src="${doc.file_url}">Trình duyệt chưa hỗ trợ video này.</video>`;
  const isReport = doc.category === 'Báo cáo';
  document.querySelectorAll('[data-nav]').forEach(a => {
    if(a.dataset.nav === (isReport?'home':'library'))a.setAttribute('aria-current','page');
    else a.removeAttribute('aria-current');
  });
  const resources = doc.id === 'report' && doc.source === 'curated' ? `<section class="report-resources" aria-label="Nội dung tương tác của báo cáo"><div><h2>Khám phá cùng báo cáo</h2><p>Demo và dữ liệu minh họa cho bài triển khai SLAM 3D này.</p></div><div class="report-resource-links"><a class="btn" href="/explore">Khám phá 3D</a><a class="btn" href="/technology">Pipeline công nghệ</a><a class="btn" href="/experiments">Đối chiếu thí nghiệm</a></div></section>` : '';
  main.innerHTML=`<div class="container">${breadcrumb(`<a href="${isReport?'/':'/library'}">${isReport?'Báo cáo':'Thư viện'}</a> &nbsp; / &nbsp; ${escapeHTML(doc.title)}`)}${resources}<div class="article-layout"><article class="article-content">${body}</article><aside class="article-meta"><h3>Về tài liệu này</h3><p><span class="pill">${escapeHTML(doc.category)}</span> ${!doc.published?'<span class="pill warm">Bản nháp</span>':''}</p><p>${escapeHTML(doc.created.split('-').reverse().join('.'))}</p><p>${escapeHTML(doc.tags.join(' · '))}</p><p>${doc.source==='curated'?'Tài liệu từ repository RB.':'Tài liệu do quản trị viên đăng.'}</p><a class="btn small" href="${doc.file_url}?download=1">Tải bản gốc ↓</a><a class="text-link" href="${isReport?'/':'/library'}">← ${isReport?'Danh sách báo cáo':'Về thư viện'}</a></aside></div></div>`;
  enableImageZoom();
}

function showError(error) {
  main.innerHTML=`<div class="container"><div class="error-box" role="alert"><h2>Chưa tải được nội dung.</h2><p>${escapeHTML(error.message)}</p><a class="btn" href="${escapeHTML(location.pathname)}">Thử lại</a> <a class="text-link" href="/">Về trang chủ</a></div></div>`;
}
const navSection = ['explore','technology','experiments'].includes(route) ? 'home' : route;
document.querySelectorAll('[data-nav]').forEach(a=>{if(a.dataset.nav===navSection)a.setAttribute('aria-current','page');});
const menu=document.querySelector('#menu-toggle');menu.onclick=()=>{const open=menu.getAttribute('aria-expanded')!=='true';menu.setAttribute('aria-expanded',String(open));document.querySelector('#main-nav').classList.toggle('open',open);};
const dialog=document.querySelector('#image-dialog');
dialog.querySelector('button').onclick=()=>dialog.close();
dialog.addEventListener('click',e=>{if(e.target===dialog)dialog.close();});
main.addEventListener('keydown',e=>{if((e.key==='Enter'||e.key===' ')&&e.target.matches('[data-zoom],.article-content img')){e.preventDefault();e.target.click();}});
main.addEventListener('click',e=>{const target=e.target.closest('[data-zoom],.article-content img');if(target){dialog.querySelector('img').src=target.src;dialog.querySelector('img').alt=target.alt;dialog.querySelector('p').textContent=target.alt;dialog.showModal();}});
try {
  if(route==='home')await home();
  else if(route==='explore')explore();
  else if(route==='technology')technology();
  else if(route==='experiments')experimentsPage();
  else if(route==='library'&&location.pathname.split('/')[2])await article(decodeURIComponent(location.pathname.split('/')[2]));
  else if(route==='library')await library();
  else if(route==='admin'){document.title='Quản trị nội dung — RB Lab';const {admin}=await import('./admin.js');await admin(main);}
} catch(error){showError(error);}
