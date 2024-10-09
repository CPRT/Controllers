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
    new Joystick(width/1.5, height/2, width/6, width/12), //Main
    new ArmJoystick(width/2.5, height/2, width/8, width/16),
    //new ArmJoystick(width/2 - 80, height/3 - 10, 50, 25),

];



setInterval(() => {
    background();

    for (let joystick of joysticks ) {
        joystick.update();
    }
     

},1000 / FPS);