import {api,escapeHTML as esc} from './api.js';
const categories=['Báo cáo','Công nghệ','Hướng dẫn','Dữ liệu'];
export async function admin(main) {
  const newReport = new URLSearchParams(location.search).get('new') === 'report';
  let session=await api('/api/session');
  const headers=()=>({'X-CSRF-Token':session.csrf});
  main.innerHTML=`<div class="container"><div class="page-heading"><div><div class="eyebrow">Không gian biên tập</div><h1>${newReport?'Thêm báo cáo mới.':'Quản lý bài đăng.'}</h1><p>Tải lên nội dung, lưu bản nháp và xuất bản. Báo cáo đã xuất bản sẽ xuất hiện trong danh sách ở trang chủ.</p></div>${session.authenticated?'<button id="logout" class="btn small">Đăng xuất</button>':''}</div><div id="admin-body"></div></div>`;
  const body=document.querySelector('#admin-body');
  if(!session.authenticated){
    body.innerHTML=`<form id="login-form" class="admin-login"><span class="pill outline">RB LAB / QUẢN TRỊ</span><h2 style="margin-top:20px">Chào mừng trở lại.</h2><p>Đăng nhập để tải tài liệu, chỉnh thông tin và quản lý xuất bản.</p>${!session.configured?'<div class="note">Chưa thiết lập mật khẩu quản trị. Xem phần thiết lập tài khoản trong README của website.</div>':''}<div class="field"><label for="password">Mật khẩu quản trị</label><input id="password" type="password" autocomplete="current-password" maxlength="256" required ${!session.configured?'disabled':''}></div><button class="btn primary full-width" ${!session.configured?'disabled':''}>Đăng nhập <span>→</span></button><p id="login-message" class="form-message" role="status"></p></form>`;
    document.querySelector('#login-form').onsubmit=async e=>{
      e.preventDefault();const button=e.target.querySelector('button');button.disabled=true;
      try{session=await api('/api/login',{method:'POST',headers:{...headers(),'Content-Type':'application/json'},body:JSON.stringify({password:document.querySelector('#password').value})});await admin(main);}
      catch(error){document.querySelector('#login-message').textContent=error.message;button.disabled=false;}
    };return;
  }
  document.querySelector('#logout').onclick=async()=>{await api('/api/logout',{method:'POST',headers:headers()});await admin(main);};
  body.innerHTML=`<div class="admin-stats" id="admin-stats"></div><div class="admin-layout section"><section class="admin-panel"><h2>${newReport?'Thông tin báo cáo':'Đăng tài liệu mới'}</h2><form id="upload-form"><div class="field"><label for="upload-file">Tệp tài liệu</label><input id="upload-file" name="file" type="file" accept=".md,.pdf,.png,.jpg,.jpeg,.webp,.mp4,.webm" required><small>Markdown, PDF, ảnh, video · Tối đa 64 MB<br>Markdown UTF-8 tối đa 2 MB.</small></div><div class="field"><label for="upload-title">Tiêu đề</label><input id="upload-title" name="title" maxlength="160" placeholder="Ví dụ: Báo cáo tiến độ tháng 11" required></div><div class="field"><label for="upload-summary">Mô tả ngắn</label><textarea id="upload-summary" name="summary" maxlength="600" rows="3" placeholder="Tài liệu này giúp người đọc hiểu điều gì?"></textarea></div><div class="field"><label for="upload-category">Danh mục</label><select id="upload-category" name="category">${categories.map(c=>`<option>${c}</option>`).join('')}</select><small>Báo cáo xuất hiện trên trang chủ; các loại khác nằm trong thư viện.</small></div><div class="field"><label for="upload-tags">Nhãn chủ đề</label><input id="upload-tags" name="tags" maxlength="200" placeholder="Điều khiển, thử nghiệm, tiến độ"><small>Phân cách bằng dấu phẩy, tối đa 8 nhãn.</small></div><div class="field"><label for="upload-status">Trạng thái</label><select id="upload-status" name="published"><option value="false">Lưu bản nháp</option><option value="true">Xuất bản công khai</option></select></div><button class="btn primary full-width" id="upload-submit">Lưu tài liệu <span>↑</span></button><div id="upload-message" class="form-message" role="status"></div></form></section><section><div class="section-head"><h2 style="font-size:20px">Tài liệu của dự án</h2></div><div id="admin-list" class="admin-list"></div></section></div>`;
  let docs=[];
  async function refresh(){
    docs=await api('/api/documents?admin=1');
    document.querySelector('#admin-stats').innerHTML=`<span><strong>${docs.length}</strong> tài liệu</span><span><strong>${docs.filter(d=>d.published).length}</strong> đã xuất bản</span><span><strong>${docs.filter(d=>!d.published).length}</strong> bản nháp</span>`;
    document.querySelector('#admin-list').innerHTML=docs.map(d=>`<article class="admin-entry" data-id="${d.id}"><div class="admin-entry-head"><h3>${esc(d.title)}</h3><span class="pill ${d.published?'':'warm'}">${d.published?'Đã xuất bản':'Bản nháp'}</span></div><p>${esc(d.category)} · ${esc(d.kind.toUpperCase())} · ${esc(d.created)}</p><div class="admin-entry-actions"><button class="btn small" data-action="edit">Chỉnh thông tin</button><button class="btn small" data-action="toggle">${d.published?'Thu hồi xuất bản':'Xuất bản'}</button><a class="text-link" href="/library/${d.id}">Xem tài liệu ↗</a></div><div class="entry-message form-message" role="status"></div></article>`).join('')||'<div class="empty-state">Chưa có tài liệu. Tải lên tệp đầu tiên để bắt đầu.</div>';
  }
  document.querySelector('#upload-form').onsubmit=async e=>{
    e.preventDefault();const button=document.querySelector('#upload-submit'),message=document.querySelector('#upload-message');
    const file=document.querySelector('#upload-file').files[0];
    if(file?.size>64*1024*1024){message.textContent='Tệp vượt giới hạn 64 MB.';return;}
    button.disabled=true;message.classList.remove('success');message.textContent='Đang tải và lưu tài liệu…';
    try{const result=await api('/api/documents',{method:'POST',headers:headers(),body:new FormData(e.target)});e.target.reset();await refresh();message.classList.add('success');message.innerHTML=`Đã lưu tài liệu. <a class="text-link" href="/library/${result.id}">Xem ngay ↗</a> · <a class="text-link" href="/">Danh sách báo cáo</a>`;}
    catch(error){message.textContent=error.message;}finally{button.disabled=false;}
  };
  document.querySelector('#admin-list').onclick=async e=>{
    const button=e.target.closest('[data-action]');if(!button)return;
    const entry=button.closest('[data-id]'),doc=docs.find(d=>d.id===entry.dataset.id),message=entry.querySelector('.entry-message');
    if(button.dataset.action==='toggle'){
      button.disabled=true;
      try{await api('/api/documents/'+doc.id,{method:'PATCH',headers:{...headers(),'Content-Type':'application/json'},body:JSON.stringify({published:!doc.published})});await refresh();}
      catch(error){message.textContent=error.message;button.disabled=false;}
      return;
    }
    if(entry.querySelector('form')){entry.querySelector('form').remove();return;}
    const form=document.createElement('form');form.className='admin-edit';
    form.innerHTML=`<div class="field"><label for="edit-title-${doc.id}">Tiêu đề</label><input id="edit-title-${doc.id}" name="title" maxlength="160" required value="${esc(doc.title)}"></div><div class="field"><label for="edit-summary-${doc.id}">Mô tả</label><textarea id="edit-summary-${doc.id}" name="summary" maxlength="600" rows="2">${esc(doc.summary)}</textarea></div><div class="field-row"><div class="field"><label for="edit-category-${doc.id}">Danh mục</label><select id="edit-category-${doc.id}" name="category">${categories.map(c=>`<option ${c===doc.category?'selected':''}>${c}</option>`).join('')}</select></div><div class="field"><label for="edit-tags-${doc.id}">Nhãn</label><input id="edit-tags-${doc.id}" name="tags" maxlength="200" value="${esc(doc.tags.join(', '))}"></div></div><button class="btn primary small">Lưu thay đổi</button>`;
    form.onsubmit=async event=>{
      event.preventDefault();const save=form.querySelector('button');save.disabled=true;
      try{await api('/api/documents/'+doc.id,{method:'PATCH',headers:{...headers(),'Content-Type':'application/json'},body:JSON.stringify(Object.fromEntries(new FormData(form)))});await refresh();}
      catch(error){message.textContent=error.message;save.disabled=false;}
    };entry.append(form);form.querySelector('input').focus();
  };
  await refresh();
}
