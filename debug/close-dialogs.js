/**
 * 关闭可能打开的对话框
 */
(async () => {
    // 尝试关闭运行脚本对话框
    const runScriptDlg = document.getElementById('runScriptDlg');
    if (runScriptDlg) {
        const closeBtn = runScriptDlg.querySelector('.lc_modal_close') || runScriptDlg.querySelector('[data-test="close"]');
        if (closeBtn) {
            closeBtn.click();
            console.log('已关闭对话框');
        }
    }
    
    // 关闭任何打开的 toast 或消息框
    const modals = document.querySelectorAll('.lc_modal_RVdjQ');
    modals.forEach(modal => {
        const closeBtn = modal.querySelector('.lc_modal_close');
        if (closeBtn) closeBtn.click();
    });
    
    console.log('清理完成');
})();
