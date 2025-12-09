const app = {
    state: { view: 'home', activeContext: null, autoHideEnabled: true, dockOpen: false },
    router: {
        navigate: (viewId) => {
            // Switch Views
            document.querySelectorAll('.view-section').forEach(el => el.classList.remove('active'));
            document.getElementById(`view-${viewId}`).classList.add('active');
            app.state.view = viewId;
            // If going to Chat, hide dock automatically
            if(viewId === 'chat') {
                app.ui.setDock(false);
            }
        }
    },
    ui: {
        // --- ARROW & VISIBILITY LOGIC ---
        setDock: (isOpen) => {
            app.state.dockOpen = isOpen;
            const dock = document.getElementById('dock');
            const arrowBtn = document.getElementById('dock-arrow');
            if(isOpen) {
                dock.classList.remove('hidden');
                arrowBtn.classList.remove('up-mode'); // Arrow points DOWN
            } else {
                dock.classList.add('hidden');
                arrowBtn.classList.add('up-mode'); // Arrow points UP
            }
        },
        // Manual Arrow Click
        manualToggle: () => {
            app.ui.setDock(!app.state.dockOpen);
        },
        // Mouse Enter (Bottom Trigger)
        handleMouseEnter: () => {
            if(app.state.autoHideEnabled && !app.state.dockOpen) {
                app.ui.setDock(true);
            }
        },
        // Mouse Leave (Dock)
        handleMouseLeave: () => {
            if(app.state.autoHideEnabled && app.state.dockOpen) {
                app.ui.setDock(false);
            }
        },
        // --- AUTO MODE TOGGLE (The "Magnet" Button) ---
        toggleAutoMode: () => {
            app.state.autoHideEnabled = !app.state.autoHideEnabled;
            const btn = document.getElementById('auto-mode-btn');
            if(app.state.autoHideEnabled) {
                btn.classList.add('active-mode');
                btn.setAttribute('tooltip', 'Auto-Hide: ON');
            } else {
                btn.classList.remove('active-mode');
                btn.setAttribute('tooltip', 'Auto-Hide: OFF');
            }
        }
    },
    chat: {
        sendUserMessage: () => {
            const input = document.getElementById('chat-input');
            const text = input.value.trim();
            if(!text) return;
            app.chat.appendMessage('user', text);
            input.value = '';
            setTimeout(() => { app.chat.appendMessage('ai', "This is a simulated AI response."); }, 1000);
        },
        appendMessage: (role, text) => {
            const container = document.getElementById('message-container');
            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            let inner = role === 'ai'
                ? `<div class="avatar ai">H</div><div class="bubble"><p>${text}</p><div class="doubt-trigger-wrapper"><button class="doubt-btn" onclick="app.chat.openDoubt('${text}')"><span>?</span> Ask Doubt</button></div></div>`
                : `<div class="bubble"><p>${text}</p></div><div class="avatar">U</div>`;
            row.innerHTML = inner;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        openDoubt: (ctx) => {
            app.state.activeContext = ctx;
            document.getElementById('doubt-overlay').classList.add('active');
            document.getElementById('doubt-messages').innerHTML = `<div style="font-size:0.8rem;color:#888;margin-bottom:10px;"><strong>Ctx:</strong> ${ctx}</div>`;
        },
        closeDoubt: () => document.getElementById('doubt-overlay').classList.remove('active'),
        sendDoubt: () => { /* Logic hidden for brevity, same as previous */ },
        clearChat: () => { document.getElementById('message-container').innerHTML = ''; }
    }
};
// Clock & Init
function updateClock() {
    const now = new Date();
    document.getElementById('clock').textContent = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}`;
}
setInterval(updateClock, 1000); updateClock();
// Start Logic
// app.router.navigate('home'); // Defaults in HTML