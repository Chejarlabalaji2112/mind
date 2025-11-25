const app = {
    state: {
        theme: 'dark',
        currentView: 'home',
        isProcessing: false
    },
    roboEyes: null,

    // --- 1. THE ROUTER ---
    router: {
        navigate: async (viewName) => {
            const container = document.getElementById('main-stage');
            
            // Handle Eyes Mode (Global Style)
            if(viewName === 'eyes') {
                document.body.classList.add('eyes-mode');
            } else {
                document.body.classList.remove('eyes-mode');
                // If we leave eyes mode, stop the animation loop to save resources
                if (app.roboEyes) {
                    app.roboEyes.stop();
                    app.roboEyes = null; // Cleanup
                }
            }

            // DOCK Logic
            const dock = document.getElementById('dock');
            const icon = document.getElementById('dock-toggle-icon');
            if (viewName === 'home') {
                dock.classList.remove('minimized');
                icon.innerHTML = '<path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/>';
            } else {
                dock.classList.add('minimized');
                icon.innerHTML = '<path d="M7.41 15.41L12 10.83l4.59 4.58L18 14l-6-6-6 6z"/>';
            }

            // LOAD CONTENT
            try {
                // Fetch HTML
                const res = await fetch(`views/${viewName}.html`);
                if(!res.ok) throw new Error("404");
                const html = await res.text();
                container.innerHTML = html;
                app.state.currentView = viewName;

                // POST-LOAD INITIALIZATION
                if(viewName === 'eyes') {
                    // Re-init canvas logic
                    app.roboEyes = new RoboEyes('eyes-canvas');
                    app.roboEyes.start();
                }
                if(viewName === 'chat') {
                    setTimeout(() => {
                        const input = document.getElementById('chat-input');
                        if(input) input.focus();
                    }, 100);
                }

            } catch(e) {
                console.error(e);
                container.innerHTML = "<h1>Error Loading View</h1>";
            }
        }
    },

    // --- 2. THE CHAT LOGIC ---
    chat: {
        resizeInput: (el) => {
            el.style.height = 'auto';
            el.style.height = Math.min(el.scrollHeight, 150) + 'px';
        },
        handleEnter: (e) => {
            if(e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); app.chat.sendMessage(); }
        },
        newSession: () => {
            const container = document.getElementById('message-container');
            if(container) container.innerHTML = `<div class="message-row ai"><div class="avatar ai">H</div><div class="bubble"><p>New session started.</p></div></div>`;
        },
        appendMessage: (role, text) => {
            const container = document.getElementById('message-container');
            if(!container) return; // Guard if not on chat view
            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            let html = (role === 'ai') ? `<div class="avatar ai">H</div>` : '';
            html += `<div class="bubble"><p>${text}</p></div>`;
            if(role === 'user') html += `<div class="avatar">U</div>`;
            row.innerHTML = html;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        showLoading: () => {
            const container = document.getElementById('message-container');
            if(!container) return;
            const row = document.createElement('div');
            row.id = 'loading-indicator';
            row.className = 'message-row ai';
            row.innerHTML = `<div class="avatar ai">H</div><div class="bubble"><div class="typing-indicator"><div class="dot"></div><div class="dot"></div><div class="dot"></div></div></div>`;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        removeLoading: () => { const el = document.getElementById('loading-indicator'); if(el) el.remove(); },
        sendMessage: () => {
            const input = document.getElementById('chat-input');
            const text = input.value.trim();
            if(!text || app.state.isProcessing) return;
            app.chat.appendMessage('user', text);
            input.value = '';
            app.chat.resizeInput(input);
            app.state.isProcessing = true;
            app.chat.showLoading();
            app.bridge.send('chat_query', { prompt: text });
        }
    },

    // --- 3. UI UTILS ---
    ui: {
        toggleDock: () => {
            const dock = document.getElementById('dock');
            const icon = document.getElementById('dock-toggle-icon');
            dock.classList.toggle('minimized');
            if(dock.classList.contains('minimized')) {
                icon.innerHTML = '<path d="M7.41 15.41L12 10.83l4.59 4.58L18 14l-6-6-6 6z"/>';
            } else {
                icon.innerHTML = '<path d="M7.41 8.59L12 13.17l4.59-4.58L18 10l-6 6-6-6 1.41-1.41z"/>';
            }
        },
        toggleTheme: () => {
            const newTheme = app.state.theme === 'light' ? 'dark' : 'light';
            app.state.theme = newTheme;
            document.documentElement.setAttribute('data-theme', newTheme);
        },
        triggerEyesAction: (action) => {
            app.router.navigate('eyes').then(() => {
                setTimeout(() => {
                    if(!app.roboEyes) return;
                    if(action === 'laugh') app.roboEyes.triggerLaugh();
                    if(action === 'confused') app.roboEyes.triggerConfused();
                }, 100); // Small delay to ensure canvas is ready
            });
        }
    },

    modules: {
        stopwatch: {
            update: (time) => {
                const el = document.getElementById('sw-display');
                if(el) el.innerText = time;
            },
            addLap: (lapTime) => {
                const list = document.getElementById('sw-laps');
                if(list) {
                    const item = document.createElement('div');
                    item.innerText = `Lap: ${lapTime}`;
                    list.prepend(item);
                }
            }
        },
        pomodoro: {
            update: (time, phase) => {
                const elTime = document.getElementById('pomo-display');
                const elPhase = document.getElementById('pomo-phase');
                if(elTime) elTime.innerText = time;
                if(elPhase) elPhase.innerText = phase.replace('_', ' ');
            }
        }
    },

    // --- 4. PYTHON BRIDGE ---
    bridge: {
        send: (command, payload = {}) => {
            if (window.pywebview) {
                window.pywebview.api.receive_from_ui({ command, payload });
            }
        },
        receive: (event, data) => {
            console.log("Event:", event, data);
            
            // ROUTING EVENTS TO MODULES
            if (event === 'stopwatch_tick') {
                if(app.modules.stopwatch) app.modules.stopwatch.update(data.formatted_time);
            }
            if (event === 'stopwatch_lap') {
                if(app.modules.stopwatch) app.modules.stopwatch.addLap(data.formatted_time);
            }
            if (event === 'pomodoro_tick') {
                if(app.modules.pomodoro) app.modules.pomodoro.update(data.formatted_time, data.phase);
            }
            if (event === 'ai_response') {
                python_execute('ai_response', data); // Use existing logic
            }
        }
    }
};

// --- PUBLIC API FOR PYTHON ---
function python_execute(action, data) {
    if (action === 'navigate') { app.router.navigate(data.view); }
    if (action === 'ai_response') {
        app.chat.removeLoading();
        app.state.isProcessing = false;
        // If we aren't in chat view, maybe navigate there? 
        // For now, only append if chat exists or is active.
        if (document.getElementById('message-container')) {
            app.chat.appendMessage('ai', data.text);
        }
    }
    // Remote Eyes Control
    if (app.roboEyes) {
        if (action === 'eyes_mood') app.roboEyes.setMood(data.mood);
        if (action === 'eyes_blink') app.roboEyes.blink();
        if (action === 'eyes_anim') {
            if (data.type === 'laugh') app.roboEyes.triggerLaugh();
            if (data.type === 'confused') app.roboEyes.triggerConfused();
        }
    }
}

// --- INIT ---
document.addEventListener('DOMContentLoaded', () => {
    // Clock
    const updateClock = () => {
        const now = new Date();
        const hours = String(now.getHours()).padStart(2, '0');
        const minutes = String(now.getMinutes()).padStart(2, '0');
        document.getElementById('clock').textContent = `${hours}:${minutes}`;
    };
    updateClock(); setInterval(updateClock, 1000);

    // Event Listeners
    document.getElementById('theme-toggle').addEventListener('click', app.ui.toggleTheme);
    
    // Start at Home
    app.router.navigate('home');
});