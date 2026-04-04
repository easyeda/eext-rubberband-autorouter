const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

async function importPlugin() {
    // 自动查找最新的.eext文件
    const distDir = path.join(__dirname, 'build', 'dist');
    const files = fs.readdirSync(distDir).filter(f => f.endsWith('.eext'));

    if (files.length === 0) {
        throw new Error('未找到.eext文件');
    }

    // 按修改时间排序，取最新的
    const latestFile = files
        .map(f => ({ name: f, time: fs.statSync(path.join(distDir, f)).mtime }))
        .sort((a, b) => b.time - a.time)[0].name;

    const pluginPath = path.join(distDir, latestFile);
    console.log(`找到最新插件: ${latestFile}`);

    const browser = await chromium.connectOverCDP('http://localhost:9222');
    const context = browser.contexts()[0];
    const page = context.pages()[0];

    console.log('已连接到现有浏览器...');

    console.log('等待页面加载...');
    await page.waitForTimeout(3000);

    console.log('打开扩展管理器...');
    // 使用快捷键打开扩展管理器
    await page.keyboard.press('Alt+E');
    await page.waitForTimeout(1000);

    console.log('上传插件文件...');
    const fileInput = await page.locator('input[type="file"]').first();
    await fileInput.setInputFiles(pluginPath);

    await page.waitForTimeout(3000);
    console.log('插件导入完成！');
}

importPlugin().catch(console.error);
