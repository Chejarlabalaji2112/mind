const UI = {
    setDock(isOpen) {
        app.state.dockOpen = isOpen;
        const dock = document.getElementById('dock');
        const arrowBtn = document.getElementById('dock-arrow');

        if (isOpen) {
            dock.classList.remove('hidden');
            arrowBtn.classList.remove('up-mode');
        } else {
            dock.classList.add('hidden');
            arrowBtn.classList.add('up-mode');
        }
    },

    manualToggle() {
        UI.setDock(!app.state.dockOpen);
    },

    handleMouseEnter() {
        if (app.state.autoHideEnabled && !app.state.dockOpen) {
            UI.setDock(true);
        }
    },

    handleMouseLeave() {
        if (app.state.autoHideEnabled && app.state.dockOpen) {
            UI.setDock(false);
        }
    },

    toggleAutoMode() {
        app.state.autoHideEnabled = !app.state.autoHideEnabled;
        const btn = document.getElementById('auto-mode-btn');

        if (app.state.autoHideEnabled) {
            btn.classList.add('active-mode');
            btn.setAttribute('tooltip', 'Auto-Hide: ON');
        } else {
            btn.classList.remove('active-mode');
            btn.setAttribute('tooltip', 'Auto-Hide: OFF');
        }
    }
};
