from flask import Flask, render_template, request, jsonify
import time

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/api/chat', methods=['POST'])
def chat():
    data = request.json
    user_text = data.get('prompt', '')
    
    # Simulate AI "Thinking" time
    time.sleep(1) 

    response_data = {}

    # --- LOGIC PORTED FROM ADAPTER API ---
    
    # 1. Check for "Angry" command
    if "angry" in user_text.lower():
        return jsonify({
            'action': 'command_sequence',
            'commands': [
                {'cmd': 'navigate', 'data': {'view': 'eyes'}},
                {'cmd': 'eyes_mood', 'data': {'mood': 'ANGRY'}}
            ]
        })

    # 2. Check for "Happy" command
    elif "happy" in user_text.lower():
        return jsonify({
            'action': 'command_sequence',
            'commands': [
                {'cmd': 'navigate', 'data': {'view': 'eyes'}},
                {'cmd': 'eyes_mood', 'data': {'mood': 'HAPPY'}}
            ]
        })
    
    elif "all" in user_text.lower():
        return jsonify({
            'action': 'command_sequence',
            'commands': [
                {'cmd': 'navigate', 'data': {'view': 'eyes'}},
                {'cmd': 'eyes_mood', 'data': {'mood': 'HAPPY'}},
                {'cmd': 'wait', 'data': {'duration': 2}},
                {'cmd': 'eyes_mood', 'data': {'mood': 'ANGRY'}}
            ]
        })

    # 3. Standard Response
    else:
        return jsonify({
            'action': 'ai_response',
            'data': {
                'text': f"Flask Backend: I heard you say '{user_text}'. (Try saying 'happy' or 'angry')"
            }
        })

if __name__ == '__main__':
    app.run(debug=True, port=5000)