const app = {
    state: {
        view: 'home',
        activeContext: null,
        autoHideEnabled: true,
        dockOpen: false
    },

    router: Router,
    ui: UI,
    chat: Chat,
};

setInterval(updateClock, 1000);
updateClock();
