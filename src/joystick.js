function circle(pos, radius, color) {
    context.beginPath();
    context.fillStyle = color;
    context.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
    context.fill();
    context.closePath();
}
 
class Joystick {
    constructor(x, y, radius, handleRadius) {
        this.pos = new Vector2(x, y);
        this.origin = new Vector2(x, y);
        this.radius = radius;
        this.handleRadius = handleRadius;
        this.handleFriction = 0.25;
        this.ondrag = false;
        this.touchPos = new Vector2(0, 0);
        this.listener();
        this.tempX = 10;
        this.tempY = 10;
    }
    listener() {
	// Touch Events
        addEventListener('touchstart', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('touchend', () => {
            this.ondrag = false;
        });
        addEventListener('touchmove', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
        });
	// Mouse Events
	addEventListener('mousedown', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('mouseup', () => {
            this.ondrag = false;
        });
        addEventListener('mousemove', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
        });
    }
    reposition() {
        if (this.ondrag == false) {
            this.pos = this.pos.add(this.origin.sub(this.pos).mul(this.handleFriction));
        } else {
            const diff = this.touchPos.sub(this.origin);
            const maxDist = Math.min(diff.mag(), this.radius);
            this.pos = this.origin.add(diff.normalize().mul(maxDist));
        }
    }
    draw() {
        // Draw Joystick
        circle(this.origin, this.radius, '#e57ceb');
        // Draw Handle
        circle(this.pos, this.handleRadius, '#3d3d3d');
        
    }
    update() {
        this.reposition();
        this.draw();  
        let xVel = 0;
        let yVel = 0;
        
        if (this.ondrag) {
            xVel = 2 * (this.pos.x - this.origin.x) / this.radius;
            yVel = -2 * (this.pos.y - this.origin.y) / this.radius;
        }
        //console.log("Joystick Velocities:", xVel, yVel);
        
        let twist = new ROSLIB.Message({
            linear : {
                x : yVel,
            },
            angular : {
                z : xVel
            }
        });

        if (xVel != this.tempX || yVel != this.tempY)
            {
                cmdVel.publish(twist);
                this.tempX = xVel; 
                this.tempY = yVel;
                //console.log(this.tempX, this.tempY, xVel, yVel);
            }
        //cmdVel.publish(twist);

        // Update the displayed velocity values
        document.getElementById("x-vel").innerHTML = xVel.toFixed(2);
        document.getElementById("y-vel").innerHTML = yVel.toFixed(2);
    }
}


class ArmJoystick {
    constructor(x, y, radius, handleRadius) {
        this.pos = new Vector2(x, y);
        this.origin = new Vector2(x, y);
        this.radius = radius;
        this.handleRadius = handleRadius;
        this.handleFriction = 0.25;
        this.ondrag = false;
        this.touchPos = new Vector2(0, 0);
        this.listener();
    }
    listener() {
    // Touch Events
        addEventListener('touchstart', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('touchend', () => {
            this.ondrag = false;
        });
        addEventListener('touchmove', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
        });
    // Mouse Events
    addEventListener('mousedown', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('mouseup', () => {
            this.ondrag = false;
        });
        addEventListener('mousemove', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
        });
    }
    reposition() {
        if (this.ondrag == false) {
            this.pos = this.pos.add(this.origin.sub(this.pos).mul(this.handleFriction));
        } else {
            const diff = this.touchPos.sub(this.origin);
            const maxDist = Math.min(diff.mag(), this.radius);
            this.pos = this.origin.add(diff.normalize().mul(maxDist));
        }
    }
    draw() {
        // Draw Joystick
        circle(this.origin, this.radius, '#000333');
        // Draw Handle
        circle(this.pos, this.handleRadius, '#3d3d3d');
        
    }
    update() {
        this.reposition();
        this.draw();    
        let xVel = 0;
        let yVel = 0;
        
        if (this.ondrag) {
            xVel = 2 * (this.pos.x - this.origin.x) / this.radius;
            yVel = -2 * (this.pos.y - this.origin.y) / this.radius;
        }
        //console.log("Joystick Velocities:", xVel, yVel);

        // Update the displayed velocity values
        document.getElementById("x-velA").innerHTML = xVel.toFixed(2);
        document.getElementById("y-velA").innerHTML = yVel.toFixed(2);

        }
}
      
