const canvas = document.createElement('canvas'), context = canvas.getContext('2d');
document.body.append(canvas);

let width = canvas.width = innerWidth;
let height = canvas.height = innerHeight;

const FPS = 120;

function background() {
    context.fillStyle = '#000';
    context.fillRect(0, 0, width, height);
}

let joysticks = [
    new Joystick(width/2 + 250, height/2, width/6, width/12), //Main
    new Joystick(width/3 - 80, height/3 + 30, 50, 25),//2nd Up
    new Joystick(width/3 - 80, height/3 + 140, 50, 25),//2nd down
];

setInterval(() => {
    background();

    for (let joystick of joysticks) {
        joystick.update();
    }

    

    

}, 1000 / FPS);