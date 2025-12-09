const Chat = {
    sendUserMessage() {
        const input = document.getElementById('chat-input');
        const text = input.value.trim();
        if (!text) return;

        Chat.appendMessage('user', text);
        input.value = '';

        setTimeout(() => {
            Chat.appendMessage('ai', "This is a simulated AI response.");
        }, 1000);
    },

    appendMessage(role, text) {
        const container = document.getElementById('message-container');
        const row = document.createElement('div');
        row.className = `message-row ${role}`;

        row.innerHTML = role === 'ai'
            ? `<div class="avatar ai">H</div>
               <div class="bubble">
                    <p>${text}</p>
                    <div class="doubt-trigger-wrapper">
                        <button class="doubt-btn" onclick="Chat.openDoubt('${text}')">
                            <span>?</span> Ask Doubt
                        </button>
                    </div>
               </div>`
            : `<div class="bubble"><p>${text}</p></div><div class="avatar">U</div>`;

        container.appendChild(row);

        const scroller = document.getElementById('scroller');
        scroller.scrollTop = scroller.scrollHeight;
    },

    openDoubt(ctx) {
        app.state.activeContext = ctx;
        document.getElementById('doubt-overlay').classList.add('active');
        document.getElementById('doubt-messages').innerHTML =
            `<div style="font-size:0.8rem;color:#888;margin-bottom:10px;"><strong>Ctx:</strong> ${ctx}</div>`;
    },

    closeDoubt() {
        document.getElementById('doubt-overlay').classList.remove('active');
    },

    clearChat() {
        document.getElementById('message-container').innerHTML = '';
    }
};
