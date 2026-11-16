import {test,expect} from '@playwright/test';

test('desktop: actual 3D, playback, layers and camera controls',async({page})=>{
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 page.on('console',e=>{if(e.type()==='error')errors.push(e.text());});
 await page.goto('/library/report');
 await page.getByRole('link',{name:'Khám phá 3D',exact:true}).click();
 await expect(page.locator('#scan-viewer')).toHaveAttribute('data-ready','true');
 await page.getByRole('button',{name:'▷ Phát',exact:true}).click();
 await expect(page.locator('#scan-viewer')).not.toHaveAttribute('data-frame','0');
 await page.getByRole('button',{name:'Ⅱ Dừng',exact:true}).click();
 await page.locator('#seek').fill('1500');
 await expect(page.locator('#frame-label')).toContainText('1.501');
 await page.getByLabel('Quỹ đạo camera',{exact:true}).uncheck();
 await page.getByLabel('Khung tam giác',{exact:true}).check();
 await page.getByRole('button',{name:'Nhìn từ trên'}).click();
 await page.screenshot({path:'test-results/explore-wireframe.png',fullPage:true});
 await page.getByLabel('Khung tam giác',{exact:true}).uncheck();
 await page.getByLabel('Quỹ đạo camera',{exact:true}).check();
 await page.getByRole('button',{name:'Đặt lại góc',exact:false}).click();
 await page.screenshot({path:'test-results/explore-desktop.png',fullPage:true});
 expect(errors).toEqual([]);
});

test('technology, experiments and image inspection',async({page})=>{
 await page.goto('/technology');
 await page.getByRole('button',{name:'Depth fusion',exact:false}).click();
 await expect(page.locator('#pipeline-detail')).toContainText('Từ phép đo thưa đến depth dày.');
 await page.locator('.tech-image').focus();await page.keyboard.press('Enter');await expect(page.locator('dialog')).toBeVisible();
 await page.keyboard.press('Escape');await expect(page.locator('dialog')).not.toBeVisible();
 await page.getByRole('button',{name:'Định vị',exact:false}).click();
 await page.screenshot({path:'test-results/technology-desktop.png',fullPage:true});
 await page.goto('/experiments');await expect(page.locator('#experiment-detail')).toContainText('82,91%');
 await page.getByRole('button',{name:'Phân rã sai số',exact:false}).click();
 await expect(page.locator('#experiment-detail')).toContainText('2,60');
 await expect(page.locator('#experiment-detail')).toContainText('0,17');
 await page.screenshot({path:'test-results/experiments-desktop.png',fullPage:true});
});

test('library search, shareable filter, empty state and article',async({page})=>{
 await page.goto('/library');await expect(page.locator('.document-card')).toHaveCount(7);
 await page.getByRole('button',{name:'Hướng dẫn',exact:true}).click();await expect(page.locator('.document-card')).toHaveCount(1);
 await page.reload();await expect(page.locator('.document-card')).toHaveCount(1);
 await page.getByRole('button',{name:'Tất cả',exact:true}).click();
 await page.getByRole('searchbox').fill('loi lom');await expect(page.locator('.document-card')).toHaveCount(1);
 await page.locator('.document-card').click();await expect(page.locator('.article-content')).toContainText('sàn');
 await page.goto('/library');await page.getByRole('searchbox').fill('zzzznotfound');
 await expect(page.getByText('Chưa tìm thấy tài liệu phù hợp.')).toBeVisible();
 await page.getByRole('button',{name:'Xóa bộ lọc'}).click();await expect(page.locator('.document-card')).toHaveCount(7);
 await page.screenshot({path:'test-results/library-desktop.png',fullPage:true});
});

test('admin upload, publish, edit, persistence and unpublish',async({page,browser})=>{
 await page.goto('/');await page.getByRole('link',{name:'Thêm báo cáo mới',exact:true}).click();await page.getByLabel('Mật khẩu quản trị').fill('browser-test-only-password');
 await page.getByRole('button',{name:'Đăng nhập',exact:false}).click();await expect(page.locator('#upload-form')).toBeVisible();await expect(page.locator('#upload-category')).toHaveValue('Báo cáo');
 await page.locator('#upload-file').setInputFiles({name:'qa-note.md',mimeType:'text/markdown',buffer:Buffer.from('# Kiểm tra xuất bản\n\nTài liệu QA **thực tế**.')});
 await page.getByLabel('Tiêu đề',{exact:true}).fill('Báo cáo hiệu chuẩn động cơ QA');
 await page.getByLabel('Mô tả ngắn').fill('Tệp thử nghiệm trong phiên kiểm tra riêng.');
 await page.getByRole('button',{name:'Lưu tài liệu',exact:false}).click();
 await expect(page.locator('#upload-message')).toContainText('Đã lưu tài liệu.');
 let entry=page.locator('.admin-entry').filter({has:page.getByRole('heading',{name:'Báo cáo hiệu chuẩn động cơ QA',exact:true})});
 await expect(entry).toContainText('Bản nháp');
 const context=await browser.newContext();const publicPage=await context.newPage();
 await publicPage.goto('http://127.0.0.1:8081/');await expect(publicPage.locator('.document-card')).toHaveCount(3);
 await entry.getByRole('button',{name:'Xuất bản',exact:true}).click();
 await expect(entry).toContainText('Đã xuất bản');await publicPage.reload();await expect(publicPage.locator('.document-card')).toHaveCount(4);
 await publicPage.getByRole('searchbox',{name:'Tìm báo cáo'}).fill('HIEU CHUAN DONG CO');
 await expect(publicPage.locator('.document-card')).toHaveCount(1);
 await publicPage.getByRole('link',{name:'Báo cáo hiệu chuẩn động cơ QA',exact:false}).click();
 await expect(publicPage.locator('.article-content strong')).toHaveText('thực tế');
 await expect(publicPage.locator('.report-resources')).toHaveCount(0);
 await entry.getByRole('button',{name:'Chỉnh thông tin'}).click();
 await entry.getByLabel('Tiêu đề',{exact:true}).fill('Báo cáo động cơ QA đã chỉnh');
 await entry.getByRole('button',{name:'Lưu thay đổi'}).click();
 await page.reload();entry=page.locator('.admin-entry').filter({has:page.getByRole('heading',{name:'Báo cáo động cơ QA đã chỉnh',exact:true})});
 await expect(entry).toBeVisible();
 await page.screenshot({path:'test-results/admin-desktop.png',fullPage:true});
 await entry.getByRole('button',{name:'Thu hồi xuất bản'}).click();
 await expect(entry).toContainText('Bản nháp');
 await publicPage.goto('http://127.0.0.1:8081/');await expect(publicPage.locator('.document-card')).toHaveCount(3);
 await page.getByRole('button',{name:'Đăng xuất',exact:true}).click();await expect(page.locator('#login-form')).toBeVisible();
 await context.close();
});

test('mobile layouts and navigation without horizontal overflow',async({page})=>{
 await page.setViewportSize({width:390,height:844});
 for(const route of ['/','/explore','/technology','/experiments','/library','/library/room-results','/admin']){
  await page.goto(route);
  if(route==='/')await expect(page.locator('#report-count')).toContainText('3 báo cáo');
  if(route==='/explore')await expect(page.locator('#scan-viewer')).toHaveAttribute('data-ready','true');
  await expect(page.locator('main h1')).toBeVisible();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth)).toBeTruthy();
  await page.screenshot({path:`test-results/mobile-${route.replaceAll('/','-')||'home'}.png`,fullPage:true});
 }
 await page.getByRole('button',{name:'Mở menu',exact:true}).click();
 await page.getByRole('link',{name:'Báo cáo',exact:true}).click();
 await expect(page).toHaveURL('http://127.0.0.1:8081/');
});

test('3D unavailable state is clear and controls stay disabled',async({page})=>{
 await page.route('**/static/demo/room.stl',route=>route.fulfill({status:404,body:'Missing dataset'}));
 await page.goto('/explore');await expect(page.locator('#scan-viewer')).toHaveAttribute('data-error','true');
 await expect(page.getByRole('button',{name:'▷ Phát'})).toBeDisabled();
 await expect(page.getByText('Chưa mở được bản scan 3D')).toBeVisible();
});


test('home is a report catalog with add card, search and report-specific demos',async({page})=>{
 const meshRequests=[];page.on('request',r=>{if(r.url().includes('/static/demo/'))meshRequests.push(r.url());});
 await page.goto('/');
 await expect(page.getByRole('heading',{name:'Báo cáo.',exact:true})).toBeVisible();
 await expect(page.locator('#report-grid .document-card')).toHaveCount(3);
 await expect(page.getByRole('link',{name:'Thêm báo cáo mới',exact:true})).toBeVisible();
 await expect(page.locator('#main-nav')).not.toContainText('Khám phá 3D');
 await page.getByRole('searchbox',{name:'Tìm báo cáo'}).fill('loi lom');
 await expect(page.locator('#report-grid .document-card')).toHaveCount(1);
 await page.reload();await expect(page.locator('#report-grid .document-card')).toHaveCount(1);
 await page.getByRole('searchbox',{name:'Tìm báo cáo'}).fill('khongtimthayzz');
 await expect(page.getByText('Chưa tìm thấy báo cáo phù hợp.')).toBeVisible();
 await expect(page.getByRole('link',{name:'Thêm báo cáo mới',exact:true})).toBeVisible();
 await page.getByRole('button',{name:'Xóa bộ lọc',exact:true}).click();
 await expect(page.locator('#report-grid .document-card')).toHaveCount(3);
 await page.getByLabel('Lọc chủ đề').selectOption('SLAM');
 await expect(page.locator('#report-grid .document-card')).toHaveCount(1);
 await page.getByLabel('Lọc chủ đề').selectOption('');
 await page.screenshot({path:'test-results/home-desktop.png',fullPage:true});
 expect(meshRequests).toEqual([]);
 await page.locator('#report-grid').getByRole('link',{name:'Triển khai SLAM 3D',exact:false}).click();
 await expect(page.locator('.report-resources')).toBeVisible();
 await expect(page.locator('#main-nav a[aria-current=page]')).toHaveText('Báo cáo');
 await page.getByRole('link',{name:'Khám phá 3D',exact:true}).click();
 await expect(page.locator('#scan-viewer')).toHaveAttribute('data-ready','true');
 await page.getByRole('link',{name:'Về báo cáo SLAM 3D',exact:true}).click();
 await expect(page).toHaveURL(/library\/report$/);
});
