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
        currentSession: null        
    },
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
       if (app.state.ws && (app.state.ws.readyState === WebSocket.CONNECTING || app.state.ws.readyState === WebSocket.OPEN)) {
            return; 
        }

        const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        const host = window.location.host; 
        app.state.ws = new WebSocket(`${protocol}${host}/ws`);
        
        app.state.ws.onopen = () => {
            console.log('WebSocket connected');
            app.state.reconnectDelay = 1000;
            // Request initial status if not received
        };
        app.state.ws.onmessage = (event) => {
            let data;
            try {
                data = JSON.parse(event.data);
            } catch (e) {
                // Handle binary audio if needed
                return;
            }

            if (data.type === 'chunk') {
                console.log('Received chunk for session:', data.session, 'Doubt mode:', app.state.inDoubtMode);  // DEBUG: Confirm routing
                // Route based on session
                if (data.session === 'doubt' && app.state.inDoubtMode) {
                    // Append to doubt modal
                    const messagesDiv = document.getElementById('doubt-messages');
                    const lastBubble = messagesDiv.lastElementChild;
                    if (lastBubble && lastBubble.style.alignSelf === 'flex-start') {
                        lastBubble.textContent += data.text;
                    } else {
                        // Fallback: Create new AI bubble if none
                        const aiBubble = document.createElement('div');
                        aiBubble.style = 'align-self: flex-start; background: var(--surface); border: 1px solid var(--border); padding: 10px 14px; border-radius: 16px; max-width: 85%; margin: 8px 0; word-wrap: break-word;';
                        aiBubble.textContent = data.text;
                        messagesDiv.appendChild(aiBubble);
                    }
                    messagesDiv.scrollTop = messagesDiv.scrollHeight;
                } else if (data.session === 'main') {
                    // Append to main chat
                    app.chat.appendChunk(data.text);
                }
                // Ignore chunks for closed doubt sessions

            } else if (data.type === 'status') {
                app.power.updateStatus(data.data);

            } else if (data.type === 'screen_update') {
                // FIX: Ensure screen update logic is present and robust
                if (data.data.active === false) {
                    app.screen.reset();
                } else {
                    app.screen.render(data.data.content);
                }

            } else if (data.type === 'audio_response') {
                console.log('Audio:', data.data);
            } else if (data.type === 'error') {
                console.error('Error:', data.data);
            } else if (data.type === 'stream_end') {
                const session = data.session;
                
                // UPDATED: More detailed log
                console.log('Stream ended via WS:', session, 'Full data:', data);
                
                if (session === 'main') {
                    app.state.isGeneratingMain = false;
                    app.chat.updateSendButton();  // Reset button
                } else if (session === 'doubt') {
                    app.state.isGeneratingDoubt = false;
                    app.chat.updateDoubtButton();  // Reset button
                }
                app.state.currentSession = null;
            }
        };
        app.state.ws.onerror = (error) => console.error('WS Error:', error);
        app.state.ws.onclose = () => {
            // 1. Visually indicate offline
            app.power.updateStatus('shutdown'); 
            // NEW: Reset gen states on close
            app.state.isGeneratingMain = false;
            app.state.isGeneratingDoubt = false;
            app.state.currentSession = null;
            app.chat.updateSendButton();
            app.chat.updateDoubtButton();
            
            // 2. Calculate next delay (Exponential Backoff)
            const nextDelay = app.state.reconnectDelay;
            console.log(`WS Closed. Reconnecting in ${nextDelay / 1000}s...`);

            // 3. Set the timer
            app.state.reconnectTimer = setTimeout(() => {
                app.connect();
            }, nextDelay);

            // 4. Increase delay for next time (Double it, cap at max)
            app.state.reconnectDelay = Math.min(
                app.state.reconnectDelay * 2, 
                app.state.maxReconnectDelay
            );
            if (app.state.reconnectDelay >= app.state.maxReconnectDelay) {
                console.log("Server seems permanently offline. Stopping retries.");
                return; 
            }
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
            
            // 1. Hide all buttons first
            const btnOn = menu.querySelector('[onclick="app.power.powerOn()"]');
            const btnOff = menu.querySelector('[onclick="app.power.shutdown()"]');
            const btnSleep = menu.querySelector('[onclick="app.power.sleep()"]');
            
            [btnOn, btnOff, btnSleep].forEach(b => b.style.display = 'none');

            // 2. Show buttons based on specific state
            if (status === 'shutdown') {
                btnOn.style.display = 'block';     // Show Power On
                btnOn.innerText = "Power On";      // Ensure text is Power On
            } 
            else if (status === 'sleep') {
                btnOn.style.display = 'block';     // Show Power On (as Wake Up)
                btnOn.innerText = "Wake Up";       // Change text to Wake Up
                btnOff.style.display = 'block';    // Show Shutdown
            } 
            else { 
                // Status is 'active'
                btnOff.style.display = 'block';    // Show Shutdown
                btnSleep.style.display = 'block';  // Show Sleep
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
            console.log('Send button clicked!');  // DEBUG: Confirm click fires
            const input = document.getElementById('chat-input');
            const text = input.value.trim();
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN || app.state.isGeneratingMain) return;  // Safeguard: No dup sends
            
            console.log('Starting main generation');  // DEBUG
            app.state.currentSession = 'main';
            app.state.isGeneratingMain = true;
            
            // ADD THIS: Confirm state before toggle
            console.log('State before button update:', { isGeneratingMain: app.state.isGeneratingMain, currentSession: app.state.currentSession });
            
            app.chat.updateSendButton();  // This triggers the visual change
            
            app.chat.appendMessage('user', text);
            input.value = '';
            app.state.ws.send(JSON.stringify({ type: 'text', data: text, session: 'main' }));
            app.chat.appendMessage('ai', '');  // Placeholder
        },
        stopGeneration: () => {
            const session = app.state.currentSession;
            
            // ADD THIS: Log entry
            console.log('stopGeneration called - session:', session, 'isGeneratingMain:', app.state.isGeneratingMain);
            
            if (session && app.state.ws && app.state.ws.readyState === WebSocket.OPEN) {
                app.state.ws.send(JSON.stringify({ type: 'stop', session: session }));
            }
            if (session === 'main') {
                app.state.isGeneratingMain = false;
                
                // ADD THIS: Confirm reset
                console.log('Resetting main gen state');
                
                app.chat.updateSendButton();
            } else if (session === 'doubt') {
                app.state.isGeneratingDoubt = false;
                app.chat.updateDoubtButton();
            }
            app.state.currentSession = null;
            
            // ADD THIS: Log exit
            console.log('stopGeneration done - final states:', { isGeneratingMain: app.state.isGeneratingMain, currentSession: app.state.currentSession });
        },
        // FIXED: Use innerHTML for SVG (more reliable than replaceChild) + force reflow
        updateSendButton: () => {
            const btn = document.querySelector('.input-area .btn-icon');
            if (!btn) {
                console.error('Send button not found!');  // DEBUG
                return;
            }
            const svg = btn.querySelector('svg');
            if (!svg) {
                console.error('SVG not found in send button!');  // DEBUG
                return;
            }
            
            // ADD THIS: Log entry and current state
            console.log('updateSendButton called - isGeneratingMain:', app.state.isGeneratingMain);
            console.log('Button classes before:', btn.className);  // Check if 'stop-mode' is added/removed
            console.log('SVG innerHTML before:', svg.innerHTML);  // Check path before swap
            
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
            // Force reflow for visual update
            btn.offsetHeight;
            
            // ADD THIS: Log exit and final state
            console.log('updateSendButton done - Button classes after:', btn.className);
            console.log('SVG innerHTML after:', svg.innerHTML);  // Should show square path during gen, arrow otherwise
        },
        // UPDATED: Text button for doubt (simpler, but with class)
        updateDoubtButton: () => {
            const btn = document.querySelector('.doubt-input-row button');
            if (!btn) {
                console.error('Doubt button not found!');  // DEBUG
                return;
            }
            
            console.log('Updating doubt button to:', app.state.isGeneratingDoubt ? 'stop' : 'send');  // DEBUG
            
            if (app.state.isGeneratingDoubt) {
                btn.classList.add('stop-mode');
                btn.textContent = 'Stop';
                btn.onclick = app.chat.stopGeneration;
            } else {
                btn.classList.remove('stop-mode');
                btn.textContent = 'Send';
                btn.onclick = app.chat.sendDoubt;
            }
            // Force reflow
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
                // Update doubt onclick with full text (escaped)
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
                    <strong>Context:</strong><br>${ctx.replace(/</g, '&lt;').replace(/>/g, '&gt;')}  <!-- XSS safe -->
                </div>`;
            
            setTimeout(() => document.getElementById('doubt-input').focus(), 100);
            app.chat.updateDoubtButton();  // Ensure initial state
        },
        closeDoubt: () => {
            console.log('Closing doubt modal');  // DEBUG
            // FIXED: Reset states FIRST to ignore incoming chunks immediately
            app.state.inDoubtMode = false;
            if (app.state.isGeneratingDoubt) {
                app.state.isGeneratingDoubt = false;
                app.chat.updateDoubtButton();
                app.chat.stopGeneration();  // Then send stop
            }
            document.getElementById('doubt-overlay').classList.remove('active');
            app.state.activeContext = null;
        },
        sendDoubt: () => {
            const input = document.getElementById('doubt-input');
            const text = input.value.trim();
            if (!text || !app.state.ws || app.state.ws.readyState !== WebSocket.OPEN || app.state.isGeneratingDoubt) return;  // Safeguard

            console.log('Starting doubt generation');  // DEBUG
            app.state.currentSession = 'doubt';
            app.state.isGeneratingDoubt = true;
            app.chat.updateDoubtButton();

            const messagesDiv = document.getElementById('doubt-messages');
            messagesDiv.innerHTML += `
                <div style="align-self: flex-end; background: var(--chat-user-bg); padding: 9px 14px; border-radius: 16px; max-width: 80%; margin: 8px 0 8px auto; font-size:0.95rem;">
                    ${text.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</div>`;  // XSS safe

            app.state.ws.send(JSON.stringify({
                type: 'text',
                data: text,
                session: 'doubt'
            }));

            const aiBubble = document.createElement('div');
            aiBubble.style = 'align-self: flex-start; background: var(--surface); border: 1px solid var(--border); padding: 10px 14px; border-radius: 16px; max-width: 85%; margin: 8px 0; word-wrap: break-word;';
            messagesDiv.appendChild(aiBubble);
            messagesDiv.scrollTop = messagesDiv.scrollHeight;

            input.value = '';
        },
        clearChat: () => { document.getElementById('message-container').innerHTML = ''; }
    },
    
    screen: {
        // Render ANY html string sent from backend
        render: (htmlContent) => {
            const identity = document.getElementById('home-identity');
            const dynamic = document.getElementById('home-dynamic-content');
            
            // 1. Inject content
            dynamic.innerHTML = htmlContent;

            // 2. Swap views
            identity.classList.add('hidden');
            dynamic.classList.remove('hidden');
        },

        // Revert to default HITOMI text
        reset: () => {
            const identity = document.getElementById('home-identity');
            const dynamic = document.getElementById('home-dynamic-content');

            dynamic.classList.add('hidden');
            identity.classList.remove('hidden');
            
            // Optional: Clear content after transition for cleanliness
            setTimeout(() => dynamic.innerHTML = '', 300); 
        }
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
                document.getElementById('mic-btn').classList.add('recording');
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
            if (e.key === 'Enter' && !app.state.isGeneratingMain) app.chat.sendUserMessage();
        });
        document.getElementById('doubt-input').addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && !app.state.isGeneratingDoubt) app.chat.sendDoubt();
        });
        // Close menu on outside click
        document.addEventListener('click', (e) => {
            if (!e.target.closest('#power-button') && !e.target.closest('#power-menu')) {
                document.getElementById('power-menu').classList.add('hidden');
            }
        });
        // Defaults
        app.router.navigate('home');
        app.chat.updateSendButton();  // Initial setup
        app.chat.updateDoubtButton();  // Initial for doubt (harmless if hidden)
    }
};

// Clock & Init
function updateClock() {
    const now = new Date();
    document.getElementById('clock').textContent = `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}`;
}
setInterval(updateClock, 1000); updateClock();

document.addEventListener('DOMContentLoaded', app.init);