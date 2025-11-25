class RoboEyes {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return; // Guard clause
        this.ctx = this.canvas.getContext('2d');
        this.running = false;
        
        // Config
        this.mood = 'DEFAULT'; 
        this.screenWidth = 0;
        this.screenHeight = 0;
        this.cyclops = false;
        this.curious = false;
        
        // Eye Geometry
        this.eyeL = { width: 200, height: 200, x: 0, y: 0, open: true };
        this.eyeR = { width: 200, height: 200, x: 0, y: 0, open: true };
        
        // Targets
        this.target = {
            eyeL: { w: 200, h: 200, x: 0, y: 0, r: 20 },
            eyeR: { w: 200, h: 200, x: 0, y: 0, r: 20 },
            space: 50,
            eyelids: { tired: 0, angry: 0, happy: 0 }
        };
        
        // Current
        this.current = {
            eyeL: { w: 200, h: 1, x: 0, y: 0, r: 20 },
            eyeR: { w: 200, h: 1, x: 0, y: 0, r: 20 },
            space: 50,
            eyelids: { tired: 0, angry: 0, happy: 0 }
        };

        // Timers
        this.lastUpdate = 0;
        this.blinkTimer = 0;
        this.blinkInterval = 3000;
        this.autoBlink = true;
        this.idleTimer = 0;
        this.idleInterval = 2000;
        this.idleMode = true;

        // Shake
        this.shaking = { active: false, type: null, endTime: 0, amplitude: 0 };

        this.resizeHandler = () => this.resize();
        window.addEventListener('resize', this.resizeHandler);
        this.resize();
    }

    destroy() {
        this.running = false;
        window.removeEventListener('resize', this.resizeHandler);
    }

    resize() {
        if (!this.canvas) return;
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
        this.screenWidth = this.canvas.width;
        this.screenHeight = this.canvas.height;
        
        const baseSize = Math.min(this.screenWidth, this.screenHeight) * 0.35;
        this.target.eyeL.w = baseSize; this.target.eyeL.h = baseSize;
        this.target.eyeR.w = baseSize; this.target.eyeR.h = baseSize;
        this.centerEyes();
    }
    
    centerEyes() {
        const totalW = this.target.eyeL.w + this.target.space + this.target.eyeR.w;
        const startX = (this.screenWidth - totalW) / 2;
        const startY = (this.screenHeight - this.target.eyeL.h) / 2;
        
        this.target.eyeL.x = startX;
        this.target.eyeL.y = startY;
        this.target.eyeR.x = startX + this.target.eyeL.w + this.target.space;
        this.target.eyeR.y = startY;
    }

    start() {
        if(!this.running) {
            this.running = true;
            this.loop();
        }
        this.target.eyeL.h = this.target.eyeL.w; 
        this.target.eyeR.h = this.target.eyeR.w;
    }

    stop() { this.running = false; }
    
    setMood(mood) { this.mood = mood; }
    
    blink() {
        this.current.eyeL.h = 1;
        this.current.eyeR.h = 1;
    }

    triggerLaugh() {
        this.shaking = { active: true, type: 'laugh', endTime: Date.now() + 1000, amplitude: 10 };
    }

    triggerConfused() {
        this.shaking = { active: true, type: 'confused', endTime: Date.now() + 1000, amplitude: 10 };
    }
    
    loop() {
        if(!this.running) return;
        const now = Date.now();
        const ease = 0.2; 
        
        // Tweening Logic
        ['eyeL', 'eyeR'].forEach(e => {
            ['w', 'h', 'x', 'y'].forEach(p => {
                this.current[e][p] += (this.target[e][p] - this.current[e][p]) * ease;
            });
        });

        // Eyelids
        let tT = 0, tA = 0, tH = 0;
        const hh = this.current.eyeL.h / 2;
        if (this.mood === 'TIRED') tT = hh;
        if (this.mood === 'ANGRY') tA = hh;
        if (this.mood === 'HAPPY') tH = hh;
        
        this.current.eyelids.tired += (tT - this.current.eyelids.tired) * ease;
        this.current.eyelids.angry += (tA - this.current.eyelids.angry) * ease;
        this.current.eyelids.happy += (tH - this.current.eyelids.happy) * ease;

        // Shaking
        let sx = 0, sy = 0;
        if (this.shaking.active) {
            if (now > this.shaking.endTime) this.shaking.active = false;
            else {
                const v = Math.sin(now * 0.5) * this.shaking.amplitude; 
                if (this.shaking.type === 'laugh') sy = v;
                if (this.shaking.type === 'confused') sx = v;
            }
        }

        // Auto Blink & Idle
        if(this.autoBlink && now > this.blinkTimer) {
            this.blink();
            this.blinkTimer = now + this.blinkInterval + (Math.random() * 2000);
        }
        if(this.idleMode && now > this.idleTimer) {
            this.centerEyes(); 
            const ox = (Math.random() - 0.5) * 100;
            const oy = (Math.random() - 0.5) * 50;
            this.target.eyeL.x += ox; this.target.eyeL.y += oy;
            this.target.eyeR.x += ox; this.target.eyeR.y += oy;
            this.idleTimer = now + this.idleInterval + (Math.random() * 2000);
        }
        
        this.draw(sx, sy);
        requestAnimationFrame(() => this.loop());
    }

    draw(ox, oy) {
        if (!this.ctx) return;
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        const primary = getComputedStyle(document.body).getPropertyValue('--primary').trim();
        
        const lx = this.current.eyeL.x + ox, ly = this.current.eyeL.y + oy;
        const rx = this.current.eyeR.x + ox, ry = this.current.eyeR.y + oy;

        this.ctx.fillStyle = primary;
        this.drawRR(lx, ly, this.current.eyeL.w, this.current.eyeL.h, 20);
        this.drawRR(rx, ry, this.current.eyeR.w, this.current.eyeR.h, 20);
        
        this.ctx.fillStyle = '#000000';
        // Tired
        if(this.current.eyelids.tired > 1) {
            this.ctx.fillRect(lx, ly - 10, this.current.eyeL.w, this.current.eyelids.tired + 10);
            this.ctx.fillRect(rx, ry - 10, this.current.eyeR.w, this.current.eyelids.tired + 10);
        }
        // Angry
        if(this.current.eyelids.angry > 1) {
           this.drawAngry(lx, ly, this.current.eyeL.w, this.current.eyelids.angry, true); 
           this.drawAngry(rx, ry, this.current.eyeR.w, this.current.eyelids.angry, false); 
        }
        // Happy
        if(this.current.eyelids.happy > 1) {
            const h = this.current.eyelids.happy;
            this.drawRR(lx - 2, (ly + this.current.eyeL.h) - h, this.current.eyeL.w + 4, h + 10, 10);
            this.drawRR(rx - 2, (ry + this.current.eyeR.h) - h, this.current.eyeR.w + 4, h + 10, 10);
        }
    }
    
    drawRR(x, y, w, h, r) {
        if (w < 2 * r) r = w / 2; if (h < 2 * r) r = h / 2;
        this.ctx.beginPath();
        this.ctx.moveTo(x + r, y);
        this.ctx.arcTo(x + w, y, x + w, y + h, r);
        this.ctx.arcTo(x + w, y + h, x, y + h, r);
        this.ctx.arcTo(x, y + h, x, y, r);
        this.ctx.arcTo(x, y, x + w, y, r);
        this.ctx.fill();
    }
    
    drawAngry(x, y, w, h, flip) {
        this.ctx.beginPath();
        this.ctx.moveTo(x, y - 10); 
        this.ctx.lineTo(x + w, y - 10); 
        this.ctx.lineTo(flip ? x + w : x, y + h); 
        this.ctx.fill();
    }
}