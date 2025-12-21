const app = {
    state: { 
        view: 'home', 
        activeContext: null, 
        autoHideEnabled: true, 
        dockOpen: false, 
        status: 'shutdown', 
        ws: null, 
        inDoubtMode: false,
        reconnectDelay: 1000,       
        maxReconnectDelay: 30000,   
        reconnectTimer: null,
        isGeneratingMain: false,    
        isGeneratingDoubt: false,   
        currentSession: null,
        directChatActive: false  
    },
    router: {
        navigate: (viewId) => {
            document.querySelectorAll('.view-section').forEach(el => el.classList.remove('active'));
            document.getElementById(`view-${viewId}`).classList.add('active');
            app.state.view = viewId;
            
            if(viewId === 'chat') {
                if (!app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) {
                    app.connect();
                }
                app.ui.setDock(false);
                if(app.state.directChatActive) app.home.toggleDirectInput(); 
            }
        }
    },
    
    // --- HOME SCREEN & DIRECT CHAT LOGIC ---
    home: {
        // Display AI response on LEFT side
        displayResponse: (text, append = true) => {
            const el = document.getElementById('home-ai-display');
            const container = el ? el.parentElement : null;
            
            if (el && container) {
                if (append) {
                    el.innerHTML += text;
                } else {
                    el.innerHTML = text;
                }
                // Auto-scroll to bottom
                container.scrollTop = container.scrollHeight; 
            }
        },
        
        // Display User request on RIGHT side
        displayRequest: (text) => {
            const userEl = document.getElementById('home-user-display');
            const aiEl = document.getElementById('home-ai-display');
            const userContainer = userEl ? userEl.parentElement : null;

            if (userEl && userContainer) {
                // Append new user block
                const newBlock = `<div style="margin-top:15px;">${text}</div>`;
                userEl.innerHTML += newBlock;
                
                // Clear AI text for a "fresh" turn logic (optional, requested style)
                if(aiEl) aiEl.innerHTML = ''; 

                // Auto-scroll
                userContainer.scrollTop = userContainer.scrollHeight;
            }
        },

        // Toggle the Input Bar, Hide Dock, Clear on Close
        toggleDirectInput: () => {
            const bar = document.getElementById('direct-chat-bar');
            const input = document.getElementById('direct-input');
            const dock = document.getElementById('dock');
            
            const aiDisplay = document.getElementById('home-ai-display');
            const userDisplay = document.getElementById('home-user-display');

            app.state.directChatActive = !app.state.directChatActive;

            if (app.state.directChatActive) {
                // OPEN
                bar.classList.add('active');
                dock.classList.add('hidden'); 
                app.ui.setDock(false); 
                setTimeout(() => input.focus(), 100);
            } else {
                // CLOSE
                app.chat.stopGeneration();
                bar.classList.remove('active');
                dock.classList.remove('hidden'); 
                app.ui.setDock(true); 
                input.value = ''; 
                
                // CLEAR OVERLAY TEXT (Return to pristine state)
                if(aiDisplay) aiDisplay.innerHTML = '';
                if(userDisplay) userDisplay.innerHTML = '';
            }
        },

        sendDirectMessage: () => {
            const input = document.getElementById('direct-input');
            const text = input.value.trim();
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) return;

            // 1. Display User Text on Home Screen (Right side)
            app.home.displayRequest(text);

            // 2. Send to backend
            app.state.currentSession = 'main';
            app.state.isGeneratingMain = true; 
            app.state.ws.send(JSON.stringify({ type: 'text', data: text, session: 'main' }));
            
            // 3. Update Chat View history
            app.chat.appendMessage('user', text);
            app.chat.appendMessage('ai', ''); 

            // 4. Clear input
            input.value = '';
        }
    },

    connect: () => {
       if (app.state.ws && (app.state.ws.readyState === WebSocket.CONNECTING || app.state.ws.readyState === WebSocket.OPEN)) return;

        const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        const host = window.location.host; 
        app.state.ws = new WebSocket(`${protocol}${host}/ws`);
        
        app.state.ws.onopen = () => {
            console.log('WebSocket connected');
            app.state.reconnectDelay = 1000;
        };
        app.state.ws.onmessage = (event) => {
            let data;
            try { data = JSON.parse(event.data); } catch (e) { return; }

            if (data.type === 'chunk') {
                if (data.session === 'doubt' && app.state.inDoubtMode) {
                    const messagesDiv = document.getElementById('doubt-messages');
                    const lastBubble = messagesDiv.lastElementChild;
                    if (lastBubble && lastBubble.style.alignSelf === 'flex-start') {
                        lastBubble.textContent += data.text;
                    } else {
                        const aiBubble = document.createElement('div');
                        aiBubble.style = 'align-self: flex-start; background: var(--surface); border: 1px solid var(--border); padding: 10px 14px; border-radius: 16px; max-width: 85%; margin: 8px 0; word-wrap: break-word;';
                        aiBubble.textContent = data.text;
                        messagesDiv.appendChild(aiBubble);
                    }
                    messagesDiv.scrollTop = messagesDiv.scrollHeight;

                } else if (data.session === 'main') {
                    app.chat.appendChunk(data.text);
                    
                    // Update Home Overlay if on Home View
                    if (app.state.view === 'home') {
                        app.home.displayResponse(data.text, true);
                    }
                }

            } else if (data.type === 'status') {
                app.power.updateStatus(data.data);

            } else if (data.type === 'screen_update') {
                if (data.data.active === false) {
                    app.screen.reset();
                } else {
                    app.screen.render(data.data.content);
                }

            } else if (data.type === 'stream_end') {
                const session = data.session;
                if (session === 'main') {
                    app.state.isGeneratingMain = false;
                    app.chat.updateSendButton();
                } else if (session === 'doubt') {
                    app.state.isGeneratingDoubt = false;
                    app.chat.updateDoubtButton();
                }
                app.state.currentSession = null;
            }
        };
        app.state.ws.onerror = (error) => console.error('WS Error:', error);
        app.state.ws.onclose = () => {
            app.power.updateStatus('shutdown'); 
            app.state.isGeneratingMain = false;
            app.state.isGeneratingDoubt = false;
            app.state.currentSession = null;
            app.chat.updateSendButton();
            app.chat.updateDoubtButton();
            
            const nextDelay = app.state.reconnectDelay;
            app.state.reconnectTimer = setTimeout(() => {
                app.connect();
            }, nextDelay);

            app.state.reconnectDelay = Math.min(app.state.reconnectDelay * 2, app.state.maxReconnectDelay);
        };
    },

    ui: {
        setDock: (isOpen) => {
            app.state.dockOpen = isOpen;
            const dock = document.getElementById('dock');
            const arrowBtn = document.getElementById('dock-arrow');
            
            if(app.state.directChatActive) return;

            if(isOpen) {
                dock.classList.remove('hidden');
                arrowBtn.classList.remove('up-mode'); 
            } else {
                dock.classList.add('hidden');
                arrowBtn.classList.add('up-mode'); 
            }
        },
        manualToggle: () => {
            app.ui.setDock(!app.state.dockOpen);
        },
        handleMouseEnter: () => {
            if(app.state.autoHideEnabled && !app.state.dockOpen && !app.state.directChatActive) {
                app.ui.setDock(true);
            }
        },
        handleMouseLeave: () => {
            if(app.state.autoHideEnabled && app.state.dockOpen) {
                app.ui.setDock(false);
            }
        },
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
            
            const btnOn = menu.querySelector('[onclick="app.power.powerOn()"]');
            const btnOff = menu.querySelector('[onclick="app.power.shutdown()"]');
            const btnSleep = menu.querySelector('[onclick="app.power.sleep()"]');
            
            [btnOn, btnOff, btnSleep].forEach(b => b.style.display = 'none');

            if (status === 'shutdown') {
                btnOn.style.display = 'block';     
                btnOn.innerText = "Power On";      
            } else if (status === 'sleep') {
                btnOn.style.display = 'block';     
                btnOn.innerText = "Wake Up";       
                btnOff.style.display = 'block';    
            } else { 
                btnOff.style.display = 'block';    
                btnSleep.style.display = 'block';  
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
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN || app.state.isGeneratingMain) return;
            
            app.state.currentSession = 'main';
            app.state.isGeneratingMain = true;
            app.chat.updateSendButton();
            
            app.home.displayRequest(text); // Sync with home
            
            app.chat.appendMessage('user', text);
            input.value = '';
            app.state.ws.send(JSON.stringify({ type: 'text', data: text, session: 'main' }));
            app.chat.appendMessage('ai', ''); 
        },
        stopGeneration: () => {
            const session = app.state.currentSession;
            if (session && app.state.ws && app.state.ws.readyState === WebSocket.OPEN) {
                app.state.ws.send(JSON.stringify({ type: 'stop', session: session }));
            }
            if (session === 'main') {
                app.state.isGeneratingMain = false;
                app.chat.updateSendButton();
            } else if (session === 'doubt') {
                app.state.isGeneratingDoubt = false;
                app.chat.updateDoubtButton();
            }
            app.state.currentSession = null;
        },
        updateSendButton: () => {
            const btn = document.querySelector('.input-area .btn-icon');
            if (!btn) return;
            const svg = btn.querySelector('svg');
            
            if (app.state.isGeneratingMain) {
                btn.classList.add('stop-mode');
                btn.onclick = app.chat.stopGeneration;
                btn.setAttribute('tooltip', 'Stop Generation');
                svg.innerHTML = '<path d="M6 6h12v12H6z" fill="currentColor"/>';
            } else {
                btn.classList.remove('stop-mode');
                btn.onclick = app.chat.sendUserMessage;
                btn.setAttribute('tooltip', 'Send');
                svg.innerHTML = '<path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" fill="currentColor"/>';
            }
            btn.offsetHeight;
        },
        updateDoubtButton: () => {
            const btn = document.querySelector('.doubt-input-row button');
            if (!btn) return;
            
            if (app.state.isGeneratingDoubt) {
                btn.classList.add('stop-mode');
                btn.textContent = 'Stop';
                btn.onclick = app.chat.stopGeneration;
            } else {
                btn.classList.remove('stop-mode');
                btn.textContent = 'Send';
                btn.onclick = app.chat.sendDoubt;
            }
            btn.offsetHeight;
        },
        appendMessage: (role, text) => {
            const container = document.getElementById('message-container');
            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            let inner = role === 'ai'
                ? `<div class="avatar ai">H</div><div class="bubble"><p class="streaming-text">${text}</p><div class="doubt-trigger-wrapper"><button class="doubt-btn" onclick="app.chat.openDoubt('${text.replace(/'/g, "\\'")}')"><span>?</span> Ask Doubt</button></div></div>`
                : `<div class="bubble"><p>${text}</p></div><div class="avatar">U</div>`;
            row.innerHTML = inner;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        appendChunk: (chunk) => {
            const lastAiBubble = document.querySelector('.message-row.ai:last-child .bubble p');
            if (lastAiBubble) {
                lastAiBubble.textContent += chunk;
                const doubtBtn = lastAiBubble.parentElement.querySelector('.doubt-btn');
                if (doubtBtn) {
                    const fullText = lastAiBubble.textContent;
                    doubtBtn.onclick = () => app.chat.openDoubt(fullText.replace(/'/g, "\\'"));
                }
            } else {
                app.chat.appendMessage('ai', chunk);
            }
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        openDoubt: (ctx) => {
            app.state.activeContext = ctx;
            app.state.inDoubtMode = true;
            const overlay = document.getElementById('doubt-overlay');
            overlay.classList.add('active');
            
            const messages = document.getElementById('doubt-messages');
            messages.innerHTML = `
                <div style="font-size:0.82rem; color:#888; margin-bottom:12px; line-height:1.4;">
                    <strong>Context:</strong><br>${ctx.replace(/</g, '&lt;').replace(/>/g, '&gt;')}
                </div>`;
            
            setTimeout(() => document.getElementById('doubt-input').focus(), 100);
            app.chat.updateDoubtButton();
        },
        closeDoubt: () => {
            app.state.inDoubtMode = false;
            if (app.state.isGeneratingDoubt) {
                app.state.isGeneratingDoubt = false;
                app.chat.updateDoubtButton();
                app.chat.stopGeneration();
            }
            document.getElementById('doubt-overlay').classList.remove('active');
            app.state.activeContext = null;
        },
        sendDoubt: () => {
            const input = document.getElementById('doubt-input');
            const text = input.value.trim();
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN || app.state.isGeneratingDoubt) return;

            app.state.currentSession = 'doubt';
            app.state.isGeneratingDoubt = true;
            app.chat.updateDoubtButton();

            const messagesDiv = document.getElementById('doubt-messages');
            messagesDiv.innerHTML += `
                <div style="align-self: flex-end; background: var(--chat-user-bg); padding: 9px 14px; border-radius: 16px; max-width: 80%; margin: 8px 0 8px auto; font-size:0.95rem;">
                    ${text.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</div>`;

            app.state.ws.send(JSON.stringify({ type: 'text', data: text, session: 'doubt' }));

            const aiBubble = document.createElement('div');
            aiBubble.style = 'align-self: flex-start; background: var(--surface); border: 1px solid var(--border); padding: 10px 14px; border-radius: 16px; max-width: 85%; margin: 8px 0; word-wrap: break-word;';
            messagesDiv.appendChild(aiBubble);
            messagesDiv.scrollTop = messagesDiv.scrollHeight;

            input.value = '';
        },
        clearChat: () => { document.getElementById('message-container').innerHTML = ''; }
    },
    
    screen: {
        render: (htmlContent) => {
            const identity = document.getElementById('home-identity');
            const dynamic = document.getElementById('home-dynamic-content');
            dynamic.innerHTML = htmlContent;
            identity.classList.add('hidden');
            dynamic.classList.remove('hidden');
        },
        reset: () => {
            const identity = document.getElementById('home-identity');
            const dynamic = document.getElementById('home-dynamic-content');
            dynamic.classList.add('hidden');
            identity.classList.remove('hidden');
            setTimeout(() => dynamic.innerHTML = '', 300); 
        }
    },
    
    mic: {
        isRecording: false,
        start: async () => {
            if (!app.state.ws || app.state.ws.readyState !== WebSocket.OPEN) {
                app.connect();
                setTimeout(app.mic.start, 1000);
                return;
            }
            if (app.mic.isRecording) return;
            try {
                const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
                const recorder = new MediaRecorder(stream);
                app.mic.isRecording = true;
                const chunks = [];
                recorder.ondataavailable = (e) => chunks.push(e.data);
                recorder.onstop = async () => {
                    const blob = new Blob(chunks, { type: 'audio/wav' });
                    app.state.ws.send(blob);
                    stream.getTracks().forEach(track => track.stop());
                    app.mic.isRecording = false;
                    document.getElementById('mic-btn').classList.remove('recording');
                };
                recorder.start();
                document.getElementById('mic-btn').classList.add('recording');
                setTimeout(() => recorder.stop(), 5000);
            } catch (err) {
                console.error('Mic access denied:', err);
            }
        }
    },

    init: () => {
        app.connect();
        
        document.getElementById('power-button').onclick = app.power.showMenu;
        document.getElementById('mic-btn').onclick = app.mic.start;
        
        document.getElementById('chat-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !app.state.isGeneratingMain) app.chat.sendUserMessage();
        });
        
        document.getElementById('doubt-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !app.state.isGeneratingDoubt) app.chat.sendDoubt();
        });
        
        const directInput = document.getElementById('direct-input');
        if(directInput) {
            directInput.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') app.home.sendDirectMessage();
            });
        }

        document.addEventListener('click', (e) => {
            if (!e.target.closest('#power-button') && !e.target.closest('#power-menu')) {
                document.getElementById('power-menu').classList.add('hidden');
            }
        });
        
        app.router.navigate('home');
        app.chat.updateSendButton();
        app.chat.updateDoubtButton();
    }
};

function updateClock() {
    const now = new Date();
    document.getElementById('clock').textContent = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}`;
}
setInterval(updateClock, 1000); updateClock();

document.addEventListener('DOMContentLoaded', app.init);