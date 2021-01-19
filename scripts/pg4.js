const roomWidth = 3.3167;
const roomHeight = 8;

const canvasWidth = 199;
const canvasHeight = 480;

const graphWidth = 500;
const graphHeight = 250;

const canvasAdjust = 10;
const m = 1;//mass
const desired_speed = 1.34*2;//mean desired speed
const tau = 0.5//relaxation time
const expon = 9 * Math.E;
const dt = 1 / 60;
//Potentials
const V0 = 2.1;
const sigma = 0.3;
const U0 = 0;
const R = 0.2;
const neutral_distance = 2;
const private_distance = 0.5;
const private_force = 20;
const Apower = 2;
const Bpower = 1.9999;
const A = -(private_force * private_distance ** Apower) / ((neutral_distance ** (Bpower - Apower)) * ((private_distance ** (Apower - Bpower))) - 1);
const B = A * neutral_distance ** (Bpower - Apower);

function pedForceFunction(distance){
    return -A / (distance ** Apower) + B / (distance ** Bpower);
}

function findDistanceToWall(pointX, pointY, wall) {//Finding shortest path between a point and a line segment (used for exits, walls)
    let dx = wall.endX - wall.startX;
    let dy = wall.endY - wall.startY;
    let closest;
    if ((dx == 0) && (dy == 0)) {
        // It's a point not a line segment.
        closest = [wall.startX, wall.startY];
        dx = pointX - wall.startX;
        dy = pointY - wall.startY;
        return [closest, Math.sqrt(dx * dx + dy * dy)];
    }
    let t = ((pointX - wall.startX) * dx + (pointY - wall.startY) * dy) / (dx * dx + dy * dy);
    if (t < 0) {
        closest = [wall.startX, wall.startY];
        dx = pointX - wall.startX;
        dy = pointY - wall.startY;
    }
    else if (t > 1) {
        closest = [wall.endX, wall.endY];
        dx = pointX - wall.endX;
        dy = pointY - wall.endY;
    }
    else {
        closest = [wall.startX + t * dx, wall.startY + t * dy]
        dx = pointX - closest[0];
        dy = pointY - closest[1];
    }

    return [closest, Math.sqrt(dx * dx + dy * dy)]
}
function findDistanceToExit(pointX, pointY, exit) {//Finding shortest path between a point and a line segment (used for exits, exits)
    let dx = exit.bufferedEndX - exit.bufferedStartX;
    let dy = exit.bufferedEndY - exit.bufferedStartY;
    let closest;
    if ((dx == 0) && (dy == 0)) {
        // It's a point not a line segment.
        closest = [exit.bufferedStartX, exit.bufferedStartY];
        dx = pointX - exit.bufferedStartX;
        dy = pointY - exit.bufferedStartY;
        return [closest, Math.sqrt(dx * dx + dy * dy)];
    }
    let t = ((pointX - exit.bufferedStartX) * dx + (pointY - exit.bufferedStartY) * dy) / (dx * dx + dy * dy);
    if (t < 0) {
        closest = [exit.bufferedStartX, exit.bufferedStartY];
        dx = pointX - exit.bufferedStartX;
        dy = pointY - exit.bufferedStartY;
    }
    else if (t > 1) {
        closest = [exit.bufferedEndX, exit.bufferedEndY];
        dx = pointX - exit.bufferedEndX;
        dy = pointY - exit.bufferedEndY;
    }
    else {
        closest = [exit.bufferedStartX + t * dx, exit.bufferedStartY + t * dy]
        dx = pointX - closest[0];
        dy = pointY - closest[1];
    }

    return [closest, Math.sqrt(dx * dx + dy * dy)]
}
class Pedestrian {
    constructor(x, y, room) {
        this.pos = [x, y]
        this.mass = m;
        //this.desired_speed = desired_speed + 0.804*(Math.random()+Math.random()+Math.random()+Math.random()+Math.random()+Math.random()-3)/6;
        this.desired_speed = desired_speed;
        this.vel = [0, 0];
        this.acc = [0, 0];
        this.diameterX = 0.25 * canvasWidth / room.width;
        this.diameterY = 0.25 * canvasHeight / room.height
        this.max_speed = 1.3 * this.desired_speed;
        //this.x and this.y is for canvas display, actual positions are in this.pos
        this.x = x * canvasWidth / room.width + canvasAdjust;
        this.y = y * canvasHeight / room.height + canvasAdjust;
        this.room = room
        this.exit_force = [0, 0];
        this.border_force = [0, 0];
        this.pedestrian_force = [0, 0];
    }
    display(p) {
        p.fill(0);
        p.stroke(0);
        p.ellipse(this.x, this.y, this.diameterX, this.diameterY);
    }
    calcTargetAttractiveForce(target) {//Calculating attractive force to a target
        let desiredVX;
        let desiredVY;
        let sX = Math.abs(this.pos[0] - target[0]);
        let sY = Math.abs(this.pos[1] - target[1]);
        desiredVY = Math.sqrt((this.desired_speed ** 2) / (1 + (sX / sY) ** 2))//Y component of desired velocity
        desiredVX = Math.sqrt((this.desired_speed ** 2) / (1 + (sY / sX) ** 2))//^ x component
        if (this.pos[0] > target[0]) {//desiedVX and VY always positive from previous calculation, so need to adjust here
            desiredVX *= -1;
        }
        if (this.pos[1] > target[1]) {
            desiredVY *= -1;
        }
        let Fx = m * (desiredVX - this.vel[0]) / tau;
        let Fy = m * (desiredVY - this.vel[1]) / tau;
        return [Fx, Fy];
    }
    pedpedForce(other_ped) {
        let dx = other_ped.pos[0] - this.pos[0];
        let dy = other_ped.pos[1] - this.pos[1];
        let rab = Math.sqrt(dx ** 2 + dy ** 2);
        let Ft = pedForceFunction(rab);
        let Fx = Ft * dx / rab;
        let Fy = Ft * dy / rab;
        return [Fx, Fy];
    }
    calcPedPedForces() {
        let totalFx = 0;
        let totalFy = 0;
        let Force;
        for (let i = 0; i < this.room.pedestrians.length; i++) {
            if (this !== this.room.pedestrians[i]) {
                Force = this.pedpedForce(this.room.pedestrians[i]);
                totalFx += Force[0];
                totalFy += Force[1];
            }
        }
        return [totalFx, totalFy];
    }
    pedBorderRepulsivePotential(raB) {//UaB in paper
        return U0 * expon ** (-1 * raB / R);//Changed to 18 from e, now can actually reach exit (smallest door tested is 0.5m wide, any smaller unrealistic)
    }
    pedBorderRepulsiveForce(wall, delta = 0.001) {//delta is the small step for finite difference differentiation 
        let closest = findDistanceToWall(this.pos[0], this.pos[1], wall);
        let raB = closest[1];
        let UaB = this.pedBorderRepulsivePotential(raB);
        let raBdx = findDistanceToWall(this.pos[0] - delta, this.pos[1], wall)[1];
        let raBdy = findDistanceToWall(this.pos[0], this.pos[1] - delta, wall)[1];
        let UaBdx = this.pedBorderRepulsivePotential(raBdx);
        let UaBdy = this.pedBorderRepulsivePotential(raBdy);
        let Fx = -1 * (UaB - UaBdx) / delta;//This is grad calculation, using finite difference partial differentiation
        let Fy = -1 * (UaB - UaBdy) / delta;

        return ([Fx, Fy]);
    }
    calcBorderForces() {//Calculating the total border forces on the pedestrian
        let totalFx = 0;
        let totalFy = 0;
        let currentForce;
        for (let i = 0; i < this.room.walls.length; i++) {
            currentForce = this.pedBorderRepulsiveForce(this.room.walls[i]);
            totalFx += currentForce[0];
            totalFy += currentForce[1];
        }

        return [totalFx, totalFy];
    }
    move() {//Moving a pedestrian
        let target = 0;
        let targForce = [0, 0];
        let borderForce = [0, 0];
        let pedForces = [0, 0];
        let minDist = 9999;
        let targX;
        let targY;
        let targDist;
        let potentialTarget;
        let desiredExit;
        for (let i = 0; i < this.room.exits.length; i++) {//Finding closest target
            potentialTarget = findDistanceToExit(this.pos[0], this.pos[1], this.room.exits[i]);
            targX = potentialTarget[0][0];
            targY = potentialTarget[0][1];
            targDist = Math.sqrt((targX - this.pos[0]) ** 2 + (targY - this.pos[1]) ** 2);
            if (targDist < minDist) {
                target = potentialTarget[0];
                desiredExit = this.room.exits[i];
                minDist = targDist;
            }
        }
        if (target !== 0) {
            targForce = this.calcTargetAttractiveForce(target);
        }
        //console.log(targForce);
        borderForce = this.calcBorderForces();
        pedForces = this.calcPedPedForces();
        this.desiredExit = desiredExit;
        this.exit_force = targForce;
        this.border_force = borderForce;
        this.pedestrian_force = pedForces;
        this.acc[0] = (targForce[0] + borderForce[0] + pedForces[0]) / m;
        this.acc[1] = (targForce[1] + borderForce[1] + pedForces[1]) / m;
        //this.acc[0] += this.acc[0]*(0.2*(Math.random()-0.5))//Fluctuations(+-10%)
        //this.acc[1] += this.acc[1]*(0.2*(Math.random()-0.5))
        this.vel[0] += this.acc[0] * dt;
        this.vel[1] += this.acc[1] * dt;
        let speed = Math.sqrt(this.vel[0] ** 2 + this.vel[1] ** 2)
        if (speed > this.max_speed) {
            this.vel[0] = this.vel[0] * this.max_speed / speed;
            this.vel[1] = this.vel[1] * this.max_speed / speed;
        }
        this.pos[0] += this.vel[0] * dt;
        this.pos[1] += this.vel[1] * dt
        this.x = this.pos[0] * canvasWidth / this.room.width + canvasAdjust;//x coordinate on canvas
        this.y = this.pos[1] * canvasHeight / this.room.height + canvasAdjust;

    }
}
class Wall {//Wall class
    constructor(startX, endX, startY, endY) {
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.type = 'wall';
    }
}
class Exit {//Exit
    constructor(startX, endX, startY, endY) {
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.centre = [(this.startX + this.endX) / 2, (this.startY + this.endY) / 2];//Used initally for target finding, now using findDistanceToWall().
        this.type = 'exit';
        this.bufferedStartX = this.startX;
        this.bufferedEndX = this.endX;
        this.bufferedStartY = this.startY;
        this.bufferedEndY = this.endY;
        if (this.startX === this.endX && this.endY - this.startY >= 0.5) {
            this.bufferedStartY = this.startY + 0.25;
            this.bufferedEndY = this.endY - 0.25;
        }
        else if (this.startY === this.endY && this.endX - this.startX >= 0.5) {
            this.bufferedStartX = this.startX + 0.25;
            this.bufferedEndX = this.endX - 0.25;
        }
    }
}
class Room {//Room
    constructor(width, height) {
        this.width = width;
        this.height = height;
        this.pedestrians = [];
        this.exits = [];
        this.walls = [];
    }
    create1Ped(x = Math.random() * (this.width - 0.05 * this.width) + 0.025 * this.width, y = Math.random() * (this.height - 0.05 * this.height) + 0.025 * this.height) {
        this.pedestrians.push(new Pedestrian(x, y, this));
    }
    create1Exit(startX = this.width, endX = this.width, startY = 0.45 * this.height, endY = 0.55 * this.height) {
        this.exits.push(new Exit(startX, endX, startY, endY));
    }
    create1Wall(startX, endX, startY, endY) {
        this.walls.push(new Wall(startX, endX, startY, endY));
    }
    generateWall() {
        this.create1Wall(0, 0, 0, this.height);//Left Wall
        this.create1Wall(this.width, this.width, 0, this.height);//Right Wall
        this.create1Wall(0, this.width, 0, 0);//Bottom Wall
        this.create1Wall(0, this.width, this.height, this.height);//Top Wall
    }
    breaksInWalls() {//Function o create 'breaks' in walls due to exits. only works for rectangular room (pretty sure)
        let exitStartX, exitStartY, exitEndX, exitEndY;
        for (let e = 0; e < this.exits.length; e++) {
            exitStartX = this.exits[e].startX;
            exitStartY = this.exits[e].startY;
            exitEndX = this.exits[e].endX;
            exitEndY = this.exits[e].endY;
            let len = this.walls.length;
            for (let w = 0; w < len; w++) {
                let wallStartX = this.walls[w].startX;
                let wallEndX = this.walls[w].endX;
                let wallStartY = this.walls[w].startY;
                let wallEndY = this.walls[w].endY;
                if (wallStartX === wallEndX && exitStartX === exitEndX && wallStartX === exitStartX) {//vertical wall
                    this.walls[w].endY = exitStartY;//Adjusts end coordinate
                    this.walls.push(new Wall(wallStartX, wallEndX, exitEndY, this.height));//Creates new wall for the remainder after the exit
                }
                else if (wallStartY === wallEndY && exitStartY === exitEndY && wallStartY === exitStartY) {//horizontal wall
                    this.walls[w].endX = exitStartX;
                    this.walls.push(new Wall(exitEndX, this.width, wallStartY, wallEndY));
                }
            }
        }
    }
    setupDemoRoom() {//Function to create a basic rom
        this.generateWall();//Wall is generated
        this.create1Exit(0, 0);//Exits are placed
        this.create1Exit();
        this.breaksInWalls();//Parts of the wall with exits are "broken"
    }
    generatePed(n) {
        for (let i = 0; i < n; i++) {
            this.create1Ped();
        }
    }
    step(p) {
        let pedLen = this.pedestrians.length;

        for (let i = 0; i < pedLen; i++) {
            this.pedestrians[i].move();
            //console.log(this.pedestrians[i].pos)
        }
        for (let i = 0; i < this.pedestrians.length; i++) {
            this.pedestrians[i].display(p);
        }
        let currExit;
        let newPeds = [];
        if (this.exits.length > 0) {
            for (let i = 0; i < pedLen; i++) {//Removing pedestrians upon reaching exit
                newPeds.push(this.pedestrians[i]);
                currExit = this.pedestrians[i].desiredExit;
                if (currExit.startX === currExit.endX) {//Vertical Exit
                    if (this.pedestrians[i].pos[1] < currExit.endY && this.pedestrians[i].pos[1] > currExit.startY) {
                        if (currExit.startX === 0 && this.pedestrians[i].pos[0] < currExit.startX + 0.0001) {
                            newPeds.pop()
                        }
                        else if (currExit.startX === this.width && this.pedestrians[i].pos[0] > currExit.startX - 0.0001) {
                            newPeds.pop();
                        }

                    }
                }
                else if (currExit.startY === currExit.endY) {//Horizontal Exit
                    if (this.pedestrians[i].pos[0] < currExit.endX && this.pedestrians[i].pos[0] > currExit.startX && this.pedestrians[i].pos[1] > currExit.startY - 0.01 && this.pedestrians[i].pos[1] < currExit.startY + 0.01) {
                        if (currExit.startY === 0 && this.pedestrians[i].pos[1] < currExit.startY + 0.0001) {
                            newPeds.pop()
                        }
                        else if (currExit.startY === this.height && this.pedestrians[i].pos[1] > currExit.startY - 0.0001) {
                            newPeds.pop();
                        }
                    }
                }
            }
            this.pedestrians = newPeds;
        }




    }
}

let r = new Room(roomWidth, roomHeight);
//r.setupDemoRoom();
//r.generatePed(10);

r.generateWall();
//console.log(r.walls[0]);
let points = [];
for (let i = 100; i < roomHeight*100; i++) {
    d = i / 100;
    Ft = pedForceFunction(d);
    pointX = d ;
    pointY = Ft ;

    points.push(new GPoint(pointX, pointY));
};
let slider;
let buttonWidth = 150;
let buttonHeight = 50;
let run = false;
let distance;
let distPoint = [];
let linePoints = [new GPoint(0,0),new GPoint(roomHeight + 1,0)];
var sketch = function (p) {
    
    p.setup = function () {
        
        p.createCanvas(canvasWidth + 4 * canvasAdjust + graphWidth + 50, canvasHeight + 2 * canvasAdjust);
        p.frameRate(1 / dt);
        p.background(255);
        p.textSize(28);
        p.strokeWeight(6);

        plot = new GPlot(p);
        plot.setPos(canvasWidth + 25, 0);
        plot.setDim(graphWidth, graphHeight);
        plot.setTitleText("Inter-Agent Force");
        plot.getXAxis().setAxisLabelText("Distance");
        plot.getYAxis().setAxisLabelText("Force");
        plot.setXLim(0, roomHeight+1);
        plot.setPoints(points);
        plot.setLineColor(p.color(0,0,255));
        plot.setLineWidth(3);
        plot.setAxesOffset(0);
        plot.addLayer("current", distPoint);
        plot.getLayer("current").setPointColor(p.color(255,0,0));
        plot.getLayer("current").setPointSize(15);
        plot.addLayer("line", linePoints);
        plot.getLayer("line").setLineColor(p.color(50));
        plot.getLayer("line").setLineWidth(0.5);
        //plot.defaultDraw();
        slider = p.createSlider(105,750,700);
        slider.position(canvasWidth + 2*canvasAdjust + 20 + graphWidth/5,graphHeight+75);
        slider.style("color","#0E406F");
        slider.size(0.75*graphWidth,100);
        reset = p.createButton("Reset");
        reset.mousePressed(p.resetClick);
        reset.style("background", "#0E406F");
        reset.style("color", "#FFFFFF");
        reset.style("font-size", "28px");
        reset.style("opacity", "0.9");
        reset.style("transition", "0.05s");
        reset.size(buttonWidth, buttonHeight);
        reset.position(canvasWidth + 2*canvasAdjust + 10 +graphWidth/5, graphHeight + 175);
        start = p.createButton("Start");
        start.mousePressed(p.startClick);
        start.style("background", "#0E406F");
        start.size(buttonWidth, buttonHeight);
        start.style("color", "#FFFFFF");
        start.style("font-size", "28px");
        start.style("opacity", "0.9");
        start.style("transition", "0.05s");
        start.position(canvasWidth + 2*canvasAdjust + 10 + 3.5*graphWidth/5, graphHeight + 175);
        
    }
    p.startClick = function(){
        run = true;
    }
    p.resetClick = function(){
        run = false;
        r = new Room(roomWidth, roomHeight);
        r.generateWall();
        
    }
    p.drawArrow = function(vec0, vec1, myColor) { //code adapted from https://p5js.org/reference/#/p5.Vector/magSq and https://stackoverflow.com/questions/44874243/drawing-arrows-in-p5js
        let offset = 4;
        p.stroke(myColor);
        p.strokeWeight(3);
        p.fill(myColor);
        p.push();
        p.line(vec0.x, vec0.y, vec0.x + vec1.x, vec0.y + vec1.y);
        let angle = p.atan2(vec1.y, vec1.x);
        p.translate(vec0.x + vec1.x, vec0.y + vec1.y);
        p.rotate(angle + p.HALF_PI);
        p.triangle(-offset * 0.5, offset, offset * 0.5, offset, 0, -offset / 2);
        p.pop();
    
    }
    p.showPedForceVect=function(Fx, Fy, posx, posy) {
        let i = Fx;
        let j = Fy;
        let x = posx;
        let y = posy;
        let v0 = p.createVector(x, y);
        let v1 = p.createVector(i, j);
        p.drawArrow(v0, v1, 'green');
    }
    p.draw = function () {
        p.background(255);//Whiting out canvas
        plot.beginDraw();
        plot.drawBackground();
        plot.drawBox();
        plot.drawXAxis();
        plot.drawYAxis();

        plot.drawTitle();
        plot.drawLines();
        plot.getLayer("current").drawPoints();
        plot.endDraw();
        
        p.stroke(100);
        for (let i = 0; i < r.walls.length; i++) {//Drawing walls
            p.line(r.walls[i].startX * canvasWidth / r.width + canvasAdjust, r.walls[i].startY * canvasHeight / r.height + canvasAdjust, r.walls[i].endX * canvasWidth / r.width + canvasAdjust, r.walls[i].endY * canvasHeight / r.height + canvasAdjust);

        }
        p.stroke(255, 0, 0);
        for (let i = 0; i < r.exits.length; i++) {//Drawing exits
            p.line(r.exits[i].startX * canvasWidth / r.width + canvasAdjust, r.exits[i].startY * canvasHeight / r.height + canvasAdjust, r.exits[i].endX * canvasWidth / r.width + canvasAdjust, r.exits[i].endY * canvasHeight / r.height + canvasAdjust);
        }
        for (let i = 0; i < r.pedestrians.length; i++) {
            //let Fx = (r.pedestrians[i].exit_force[0])*((canvasWidth+2*canvasAdjust)/r.width);
            //let Fy = (r.pedestrians[i].exit_force[1])*((canvasHeight+2*canvasAdjust)/r.height);
            let Fx = 1.5*r.pedestrians[i].pedestrian_force[0] * canvasWidth / r.width;
            let Fy = 1.5*r.pedestrians[i].pedestrian_force[1] * canvasHeight / r.height;
            let posx = r.pedestrians[i].x;
            let posy = r.pedestrians[i].y;
            let mag = (Fx**2 + Fy**2)**0.5;
            if (mag>0.95*canvasWidth){
                Fx = Fx*0.95*canvasWidth/mag;
                Fy = Fy*0.95*canvasHeight/mag;
            }
            p.showPedForceVect(Fx, Fy, posx, posy);
    
        }
        if (run===false && r.pedestrians.length === 0){
            distance = slider.value()/100;
            r.create1Ped(0.5 * roomWidth, 0.5*roomHeight - distance/2);
            r.create1Ped(0.5 * roomWidth, 0.5*roomHeight + distance/2);
            for (let i = 0; i<r.pedestrians.length; i++){
                r.pedestrians[i].display(p);
            }
            r.pedestrians = [];
            plot.getLayer("current").removePoint(0);
            distPoint = new GPoint(distance, pedForceFunction(distance));
            plot.getLayer("current").addPoint(distPoint);
            
            
        }
        if (run === true && r.pedestrians.length === 0){
            distance = slider.value()/100;
            r.create1Ped(0.5 * roomWidth, 0.5*roomHeight - distance/2);
            r.create1Ped(0.5 * roomWidth, 0.5*roomHeight + distance/2);
        }
        if (run === true && r.pedestrians.length===2){
            r.step(p);
            plot.getLayer("current").removePoint(0);
            distance = Math.sqrt((r.pedestrians[0].pos[0]-r.pedestrians[1].pos[0])**2 + (r.pedestrians[0].pos[1]-r.pedestrians[1].pos[1])**2);
            distPoint = new GPoint(distance, pedForceFunction(distance));
            plot.getLayer("current").addPoint(distPoint);
        }
        
        
    }
}

var s = new p5(sketch);