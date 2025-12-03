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
        activeContext: null,
        editingRow: null,

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
            app.chat.editingRow = null;
        },

        // --- APPEND MESSAGE (Standard) ---
        appendMessage: (role, text) => {
            const container = document.getElementById('message-container');
            if(!container) return;
            
            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            
            // 1. Create the Bubble Structure
            const bubble = document.createElement('div');
            bubble.className = 'bubble';
            
            const p = document.createElement('p');
            p.innerText = text; // Safely set text
            bubble.appendChild(p);

            // 2. Add "Ask Doubt" Button (AI Only) - THE FIX FOR "BUTTON NOT WORKING"
            if (role === 'ai') {
                const wrapper = document.createElement('div');
                wrapper.className = 'doubt-trigger-wrapper';
                
                const btn = document.createElement('button');
                btn.className = 'doubt-btn';
                btn.innerHTML = '<span>?</span> Ask Doubt Here';
                
                // CRITICAL FIX: Attach listener via JS, not HTML string
                // This allows texts with quotes, newlines, etc. to work perfectly.
                btn.addEventListener('click', () => {
                    app.chat.openDoubtWindow(text);
                });

                wrapper.appendChild(btn);
                bubble.appendChild(wrapper);
            }

            // 3. Assemble Row
            if(role === 'ai') {
                row.innerHTML = `<div class="avatar ai">H</div>`;
                row.appendChild(bubble);
            } else {
                row.appendChild(bubble);
                row.innerHTML += `<div class="avatar">U</div>`;
            }

            // 4. Add User Actions (Edit/Copy)
            if (role === 'user') {
                const safeText = text.replace(/"/g, '&quot;'); 
                const actions = document.createElement('div');
                actions.className = 'message-actions';
                actions.innerHTML = `
                    <div class="action-btn" onclick='app.chat.copyMessage(this)' title="Copy">
                        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path></svg>
                    </div>
                    <div class="action-btn" onclick='app.chat.editMessage(this)' title="Edit">
                        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M11 4H4a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-7"></path><path d="M18.5 2.5a2.121 2.121 0 0 1 3 3L12 15l-4 1 1-4 9.5-9.5z"></path></svg>
                    </div>
                `;
                row.appendChild(actions);
            }

            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },

        // --- ACTIONS ---
        copyMessage: (btn) => {
            const row = btn.closest('.message-row');
            const text = row.querySelector('.bubble p').innerText;
            navigator.clipboard.writeText(text);
        },

        editMessage: (btn) => {
            const row = btn.closest('.message-row');
            const text = row.querySelector('.bubble p').innerText;

            const input = document.getElementById('chat-input');
            input.value = text;
            input.focus();
            app.chat.resizeInput(input);

            document.querySelectorAll('.message-row.editing').forEach(el => el.classList.remove('editing'));
            row.classList.add('editing');
            app.chat.editingRow = row; 
        },

        sendMessage: () => {
            const input = document.getElementById('chat-input');
            const text = input.value.trim();
            if(!text || app.state.isProcessing) return;

            if (app.chat.editingRow) {
                // In-place edit logic
                const pTag = app.chat.editingRow.querySelector('.bubble p');
                if(pTag) pTag.innerText = text;
                
                app.chat.editingRow.classList.remove('editing');
                
                // Remove future messages
                let nextSibling = app.chat.editingRow.nextElementSibling;
                while(nextSibling) {
                    nextSibling.remove();
                    nextSibling = app.chat.editingRow.nextElementSibling;
                }
                app.chat.editingRow = null;
            } else {
                app.chat.appendMessage('user', text);
            }

            input.value = '';
            app.chat.resizeInput(input);
            app.state.isProcessing = true;
            app.chat.showLoading(); 
            
            // Standard Send
            app.bridge.send('chat_query', { prompt: text });
        },

        // --- DOUBT WINDOW LOGIC ---
        openDoubtWindow: (contextText) => {
            app.chat.activeContext = contextText;
            const modal = document.getElementById('doubt-modal');
            const list = document.getElementById('doubt-messages');
            
            // Initialize with the context
            list.innerHTML = `<div class="message-row ai" style="font-size:0.8rem; opacity:0.7"><div class="bubble"><strong>Context:</strong> "${contextText.substring(0, 100)}..."</div></div>`;
            
            modal.classList.add('active');
            setTimeout(() => document.getElementById('doubt-input').focus(), 100);
        },

        closeDoubtWindow: () => {
            document.getElementById('doubt-modal').classList.remove('active');
            app.chat.activeContext = null;
        },

        // NEW: Helper to append specifically to the modal
        appendDoubtMessage: (role, text) => {
            const list = document.getElementById('doubt-messages');
            if(!list) return;

            const row = document.createElement('div');
            row.className = `message-row ${role}`;
            
            let html = `<div class="bubble" ${role==='user' ? 'style="background:var(--primary); color:black"' : ''}>${text}</div>`;
            if(role === 'ai') html = `<div class="avatar ai">H</div>` + html;
            
            row.innerHTML = html;
            list.appendChild(row);
            list.scrollTop = list.scrollHeight;
        },

        showDoubtLoading: () => {
            const list = document.getElementById('doubt-messages');
            const row = document.createElement('div');
            row.id = 'doubt-loading';
            row.className = 'message-row ai';
            row.innerHTML = `<div class="avatar ai">H</div><div class="bubble"><div class="typing-indicator"><div class="dot"></div><div class="dot"></div><div class="dot"></div></div></div>`;
            list.appendChild(row);
            list.scrollTop = list.scrollHeight;
        },

        sendDoubtMessage: () => {
            const input = document.getElementById('doubt-input');
            const text = input.value.trim();
            if(!text) return;
            
            // 1. Show User Message in Modal
            app.chat.appendDoubtMessage('user', text);
            
            input.value = '';
            
            // 2. Show Loading in Modal
            app.chat.showDoubtLoading();

            // 3. SEND TO PYTHON
            // We send the 'prompt' (question) and the 'context' (what they clicked on)
            app.bridge.send('chat_query', { 
                prompt: text, 
                context: app.chat.activeContext 
            });
        },

        showLoading: () => {
            const container = document.getElementById('message-container');
            const row = document.createElement('div');
            row.id = 'loading-indicator';
            row.className = 'message-row ai';
            row.innerHTML = `<div class="avatar ai">H</div><div class="bubble"><div class="typing-indicator"><div class="dot"></div><div class="dot"></div><div class="dot"></div></div></div>`;
            container.appendChild(row);
            document.getElementById('scroller').scrollTop = document.getElementById('scroller').scrollHeight;
        },
        removeLoading: () => { 
            const el = document.getElementById('loading-indicator'); if(el) el.remove(); 
            const el2 = document.getElementById('doubt-loading'); if(el2) el2.remove(); 
        },
    },    // --- 3. UI UTILS ---
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

        // CHECK: Is the user currently in the Doubt Window?
        const doubtModal = document.getElementById('doubt-modal');
        const isDoubtActive = doubtModal && doubtModal.classList.contains('active');

        if (isDoubtActive) {
            // Append to Doubt Modal
            app.chat.appendDoubtMessage('ai', data.text);
        } else {
            // Append to Main Chat
            if (document.getElementById('message-container')) {
                app.chat.appendMessage('ai', data.text);
            }
        }
    }

    // Remote Eyes Control logic remains here...
    if (app.roboEyes) {
        if (action === 'eyes_mood') app.roboEyes.setMood(data.mood);
        if (action === 'eyes_blink') app.roboEyes.blink();
        if (action === 'eyes_anim') {
            if (data.type === 'laugh') app.roboEyes.triggerLaugh();
            if (data.type === 'confused') app.roboEyes.triggerConfused();
        }
    }
}
window.app = app;

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

