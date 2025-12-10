const app = {
    state: { view: 'home', activeContext: null, autoHideEnabled: true, dockOpen: false, status: 'shutdown', ws: null },
    router: {
        navigate: (viewId) => {
            // Switch Views
            document.querySelectorAll('.view-section').forEach(el => el.classList.remove('active'));
            document.getElementById(`view-${viewId}`).classList.add('active');
            app.state.view = viewId;
            // If going to Chat, ensure WS connected
            if(viewId === 'chat') {
                if (!app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) {
                    app.connect();
                }
                app.ui.setDock(false);
            }
        }
    },
    connect: () => {
        app.state.ws = new WebSocket('ws://localhost:8000/ws');  // Adjust host/port if needed
        app.state.ws.onopen = () => {
            console.log('WebSocket connected');
            // Request initial status if not received
        };
        app.state.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'chunk') {
                app.chat.appendChunk(data.text);
            } else if (data.type === 'status') {
                app.power.updateStatus(data.data);
            } else if (data.type === 'audio_response') {
                // Placeholder: Play audio (e.g., new Audio(`data:audio/wav;base64,${data.data}`).play())
                console.log('Audio response:', data.data);
            } else if (data.type === 'error') {
                console.error('Error:', data.data);
            }
        };
        app.state.ws.onerror = (error) => console.error('WS Error:', error);
        app.state.ws.onclose = () => {
            console.log('WS Closed; reconnecting in 3s...');
            setTimeout(app.connect, 3000);  // Simple reconnect
        };
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
    power: {
        showMenu: () => {
            const menu = document.getElementById('power-menu');
            const status = app.state.status;
            // Hide all buttons, show relevant
            menu.querySelectorAll('button').forEach(btn => btn.style.display = 'none');
            if (status === 'shutdown') {
                menu.querySelector('[onclick="app.power.powerOn()"]').style.display = 'block';
            } else {
                menu.querySelector('[onclick="app.power.shutdown()"]').style.display = 'block';
                menu.querySelector('[onclick="app.power.sleep()"]').style.display = 'block';
            }
            menu.classList.toggle('hidden');
        },
        updateStatus: (status) => {
            app.state.status = status;
            const dot = document.getElementById('robot-status');
            dot.className = `status-dot status-${status}`;
        },
        async powerOn() { 
            if (app.state.ws && app.state.ws.readyState === WebSocket.OPEN) {
                app.state.ws.send(JSON.stringify({ type: 'power', data: { action: 'on' } }));
            }
            app.power.showMenu();
        },
        async shutdown() { 
            if (app.state.ws && app.state.ws.readyState === WebSocket.OPEN) {
                app.state.ws.send(JSON.stringify({ type: 'power', data: { action: 'shutdown' } }));
            }
            app.power.showMenu();
        },
        async sleep() { 
            if (app.state.ws && app.state.ws.readyState === WebSocket.OPEN) {
                app.state.ws.send(JSON.stringify({ type: 'power', data: { action: 'sleep' } }));
            }
            app.power.showMenu();
        }
    },
    chat: {
        sendUserMessage: () => {
            const input = document.getElementById('chat-input');
            const text = input.value.trim();
            if(!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) return;
            app.chat.appendMessage('user', text);
            input.value = '';
            app.state.ws.send(JSON.stringify({ type: 'text', data: text }));
            // Start a placeholder AI row for streaming
            app.chat.appendMessage('ai', '');  // Empty for incremental fill
        },
        appendMessage: (role, text) => {
            const container = document.getElementById('message-container');
            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            let inner = role === 'ai'
                ? `<div class="avatar ai">H</div><div class="bubble"><p class="streaming-text">${text}</p><div class="doubt-trigger-wrapper"><button class="doubt-btn" onclick="app.chat.openDoubt('${text}')"><span>?</span> Ask Doubt</button></div></div>`
                : `<div class="bubble"><p>${text}</p></div><div class="avatar">U</div>`;
            row.innerHTML = inner;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        appendChunk: (chunk) => {
            // Append to last AI message incrementally
            const lastAiBubble = document.querySelector('.message-row.ai:last-child .bubble p');
            if (lastAiBubble) {
                lastAiBubble.textContent += chunk;
                // Update doubt onclick with full text
                const doubtBtn = lastAiBubble.parentElement.querySelector('.doubt-btn');
                if (doubtBtn) doubtBtn.onclick = () => app.chat.openDoubt(lastAiBubble.textContent);
            } else {
                app.chat.appendMessage('ai', chunk);
            }
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        openDoubt: (ctx) => {
            app.state.activeContext = ctx;
            document.getElementById('doubt-overlay').classList.add('active');
            document.getElementById('doubt-messages').innerHTML = `<div style="font-size:0.8rem;color:#888;margin-bottom:10px;"><strong>Ctx:</strong> ${ctx}</div>`;
        },
        closeDoubt: () => document.getElementById('doubt-overlay').classList.remove('active'),
        sendDoubt: () => {
            const input = document.getElementById('doubt-input');
            const text = input.value.trim();
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) return;
            app.state.ws.send(JSON.stringify({ type: 'text', data: text }));
            input.value = '';
            app.chat.closeDoubt();  // Close modal after send
        },
        clearChat: () => { document.getElementById('message-container').innerHTML = ''; }
    },
    mic: {
        isRecording: false,
        start: async () => {
            if (!app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) {
                app.connect();
                setTimeout(app.mic.start, 1000);  // Retry after connect
                return;
            }
            if (app.mic.isRecording) return;  // Prevent multiple
            try {
                const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
                const recorder = new MediaRecorder(stream);
                app.mic.isRecording = true;
                const chunks = [];
                recorder.ondataavailable = (e) => chunks.push(e.data);
                recorder.onstop = async () => {
                    const blob = new Blob(chunks, { type: 'audio/wav' });
                    app.state.ws.send(blob);  // Binary over WS
                    stream.getTracks().forEach(track => track.stop());
                    app.mic.isRecording = false;
                    document.getElementById('mic-btn').classList.remove('recording');
                };
                recorder.start();
                document.getElementById('mic-btn').classList.add('recording');  // Visual feedback (add CSS for this)
                setTimeout(() => recorder.stop(), 5000);  // 5s recording
            } catch (err) {
                console.error('Mic access denied:', err);
            }
        }
    },
    init: () => {
        app.connect();  // Connect WS on load
        // Event listeners
        document.getElementById('power-button').onclick = app.power.showMenu;
        document.getElementById('mic-btn').onclick = app.mic.start;
        document.getElementById('chat-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') app.chat.sendUserMessage();
        });
        // Close menu on outside click
        document.addEventListener('click', (e) => {
            if (!e.target.closest('#power-button') && !e.target.closest('#power-menu')) {
                document.getElementById('power-menu').classList.add('hidden');
            }
        });
        // Defaults
        app.router.navigate('home');
    }
};

// Clock & Init
function updateClock() {
    const now = new Date();
    document.getElementById('clock').textContent = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}`;
}
setInterval(updateClock, 1000); updateClock();

document.addEventListener('DOMContentLoaded', app.init);