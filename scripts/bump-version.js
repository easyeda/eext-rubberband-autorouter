const fs = require('fs');
const path = require('path');

// 读取 package.json
const packagePath = path.join(__dirname, '..', 'package.json');
const packageJson = JSON.parse(fs.readFileSync(packagePath, 'utf8'));

// 解析版本号
const oldVersion = packageJson.version;
const [major, minor, patch] = packageJson.version.split('.').map(Number);
const newVersion = `${major}.${minor}.${patch + 1}`;

// 更新 package.json
packageJson.version = newVersion;
fs.writeFileSync(packagePath, JSON.stringify(packageJson, null, 2) + '\n');

// 更新 package-lock.json（root 与 packages[""] 两处，保持三处版本一致）
const lockPath = path.join(__dirname, '..', 'package-lock.json');
if (fs.existsSync(lockPath)) {
    const lockJson = JSON.parse(fs.readFileSync(lockPath, 'utf8'));
    lockJson.version = newVersion;
    if (lockJson.packages && lockJson.packages['']) lockJson.packages[''].version = newVersion;
    fs.writeFileSync(lockPath, JSON.stringify(lockJson, null, 2) + '\n');
}

// 更新 extension.json
const extensionPath = path.join(__dirname, '..', 'extension.json');
const extensionJson = JSON.parse(fs.readFileSync(extensionPath, 'utf8'));
extensionJson.version = newVersion;

// 菜单不显示版本号（版本信息保留在“关于”弹窗中）
if (extensionJson.headerMenus?.pcb?.[0]?.menuItems) {
    const aboutItem = extensionJson.headerMenus.pcb[0].menuItems.find(item => item.id === 'About');
    if (aboutItem) {
        aboutItem.title = '关于';
    }
}

fs.writeFileSync(extensionPath, JSON.stringify(extensionJson, null, '\t') + '\n');

console.log(`版本号已更新: ${oldVersion} -> ${newVersion}`);
