//Global variables
const canvasWidth = 500;
const canvasHeight = 500;
const canvasAdjust = 10;
const m = 1;//mass
const desired_speed = 1.34;//mean desired speed
const tau = 0.5//relaxation time
const dt = 1/60;
//Potentials
const V0 = 2.1;
const sigma = 0.3;
const U0 = 10;
const R = 0.2;

function findDistanceToSegment(pointX, pointY, wall){
    let dx = wall.endX - wall.startX;
    let dy = wall.endY - wall.startY;
    let closest;
    if ((dx == 0) && (dy == 0)){
        // It's a point not a line segment.
        closest = [wall.startX, wall.startY];
        dx = pointX - wall.startX;
        dy = pointY - wall.startY;
        return [closest,Math.sqrt(dx * dx + dy * dy)];
    }
    let t = ((pointX-wall.startX)*dx + (pointY-wall.startY)*dy)/(dx*dx + dy*dy);
    if(t<0){
        closest = [wall.startX, wall.startY];
        dx = pointX-wall.startX;
        dy = pointY-wall.startY;
    }
    else if(t>1){
        closest = [wall.endX, wall.endY];
        dx = pointX-wall.endX;
        dy = pointY-wall.endY;
    }
    else{
        closest = [wall.startX + t*dx, wall.startY + t*dy]
        dx = pointX-closest[0];
        dy = pointY-closest[1];
    }
    return [closest,Math.sqrt(dx * dx + dy * dy)]
}

class Pedestrian{
    constructor(x,y,room){
        this.pos = [x,y]
        this.mass = m;
        this.desired_speed = desired_speed + 0.804*(Math.random()+Math.random()+Math.random()+Math.random()+Math.random()+Math.random()-3)/6;
        this.vel = [0,0];
        this.acc = [0,0];
        this.diameterX = 0.3* canvasWidth/room.width;
        this.diameterY = 0.3* canvasHeight/room.height
        this.x = x * canvasWidth/room.width + canvasAdjust;
        this.y = y * canvasHeight/room.height + canvasAdjust;
        this.room = room
        
    }
    display(){
        fill(0);
        stroke(0);
        ellipse(this.x,this.y,this.diameterX, this.diameterY);
        console.log(this.pos);
    }
    calcTargetAttractiveForce(target){
        let desiredVX;
        let desiredVY;
        let sX = Math.abs(this.pos[0]-target[0]);
        let sY = Math.abs(this.pos[1]-target[1]);
        desiredVY = Math.sqrt((this.desired_speed**2)/(1+(sX/sY)**2))
        desiredVX = Math.sqrt((this.desired_speed**2)/(1+(sY/sX)**2))
        if (this.pos[0]>target[0]){
            desiredVX *= -1;
        }
        if (this.pos[1]>target[1]){
            desiredVY *= -1;
        }
        //console.log(desiredVX, desiredVY, this.desired_speed, Math.sqrt(desiredVX**2 + desiredVY **2));
        let Fx = m*(desiredVX - this.vel[0])/tau;
        let Fy = m*(desiredVY - this.vel[1])/tau;
        return [Fx,Fy];
    }
    pedBorderRepulsivePotential(raB){//UaB
        return U0 * Math.exp(-1 * raB/R);
    }
    pedBorderRepulsiveForce(wall, delta = 0.001){
        let closest = findDistanceToSegment(this.pos[0], this.pos[1], wall);
        let raB = closest[1];
        let UaB = this.pedBorderRepulsivePotential(raB);
        let raBdx = findDistanceToSegment(this.pos[0]+delta, this.pos[1], wall)[1];
        let raBdy = findDistanceToSegment(this.pos[0], this.pos[1]+delta, wall)[1];
        let UaBdx = this.pedBorderRepulsivePotential(raBdx);
        let UaBdy = this.pedBorderRepulsivePotential(raBdy);
        let Fx = -1 * (UaBdx - UaB)/delta;
        let Fy = -1 * (UaBdy - UaB)/delta;
        return ([Fx,Fy]);
    }
    calcBorderForces(){
        let totalFx=0;
        let totalFy=0;
        let currentForce;
        for (let i = 0; i < this.room.walls.length; i++){
            currentForce = this.pedBorderRepulsiveForce(this.room.walls[i]);
            totalFx += currentForce[0];
            totalFy += currentForce[1];
        }
        return [totalFx, totalFy];
    }
    move(){
        let target;
        let minDist = 9999;
        let targX;
        let targY;
        let targDist;
        let potentialTarget;
        for (let i = 0; i < this.room.exits.length; i++){
            potentialTarget = findDistanceToSegment(this.pos[0], this.pos[1], this.room.exits[i]);
            targX = potentialTarget[0][0];
            targY = potentialTarget[0][1];
            targDist = Math.sqrt((targX - this.pos[0])**2 + (targY - this.pos[1])**2);
            if (targDist<minDist){
                target = potentialTarget[0];
                minDist = targDist;
            }
        }
        
        let targForce = this.calcTargetAttractiveForce(target);
        let borderForce = this.calcBorderForces();
        this.acc[0] = (targForce[0]+borderForce[0])/m;
        this.acc[1] = (targForce[1]+borderForce[1])/m;
        this.vel[0] += this.acc[0]*dt;
        this.vel[1] += this.acc[1]*dt;
        //console.log(this.acc);
        this.pos[0] += this.vel[0] * dt;
        this.pos[1] += this.vel[1] * dt
        this.x = this.pos[0] * canvasWidth/this.room.width + canvasAdjust;
        this.y = this.pos[1] * canvasHeight/this.room.height + canvasAdjust;

    }
    randomMove(){
        this.x += random(-this.desired_speed*(dt) *canvasWidth/this.room.width, this.desired_speed* (dt) *canvasWidth/this.room.width); 
        this.y += random(-this.desired_speed*(dt) * canvasHeight/this.room.height, this.desired_speed* (dt) *canvasHeight/this.room.height); 
    }
    

}
class Wall{
    constructor(startX, endX, startY, endY){
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        
    }
}
class Exit{
    constructor(startX, endX, startY, endY){
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.centre = [(this.startX+this.endX)/2,(this.startY+this.endY)/2];
    }
}
class Room{
    constructor(width, height){
        this.width = width;
        this.height = height;
        this.pedestrians=[];
        this.exits=[];
        this.walls=[];
    }
    create1Ped(x=Math.random()*(this.width-0.05 * this.width)+0.025*this.width,y=Math.random()*(this.height-0.05 * this.height)+0.025*this.height){
        this.pedestrians.push(new Pedestrian(x,y,this));
    }
    create1Exit(startX = this.width, endX = this.width, startY = 0.475*this.height, endY = 0.525*this.height){
        this.exits.push(new Exit(startX, endX, startY, endY));
    }
    create1Wall(startX, endX, startY, endY){
        this.walls.push(new Wall(startX, endX, startY, endY));
    }
    generateWall(){
        this.create1Wall(0,0,0,this.height);//Left Wall
        this.create1Wall(this.width,this.width,0,this.height);//Right Wall
        this.create1Wall(0,this.width,0,0);//Bottom Wall
        this.create1Wall(0,this.width,this.height,this.height);//Top Wall
    }
    breaksInWalls(){//only if rectangular room 
        let exitStartX, exitStartY, exitEndX, exitEndY;
        for (let e = 0; e<this.exits.length; e++){
            exitStartX = this.exits[e].startX;
            exitStartY = this.exits[e].startY;
            exitEndX = this.exits[e].endX;
            exitEndY = this.exits[e].endY;
            let len = this.walls.length;
            for (let w = 0; w<len; w++){
                let wallStartX = this.walls[w].startX;
                let wallEndX = this.walls[w].endX;
                let wallStartY = this.walls[w].startY;
                let wallEndY = this.walls[w].endY;
                if(wallStartX === wallEndX && exitStartX === exitEndX && wallStartX === exitStartX){//vertical wall
                    this.walls[w].endY = exitStartY;
                    this.walls.push(new Wall(wallStartX, wallEndX, exitEndY, this.height));
                }
                else if (wallStartY === wallEndY && exitStartY === exitEndY && wallStartY === exitStartY){//horizontal wall
                    this.walls[w].endX = exitStartX;
                    this.walls.push(new Wall(exitEndX, this.width, wallStartY, wallEndY));
                }
            } 
        }
    }
    setupDemoRoom(){
        this.generateWall();
        this.create1Exit(0,0);
        this.create1Exit();
        this.breaksInWalls();
    }

}

let r = new Room(10,10);
r.setupDemoRoom();
r.create1Ped();
r.create1Ped();
r.create1Ped();
r.create1Ped();
r.create1Ped();

//console.log(r.walls[0]);



function setup(){
    createCanvas(canvasWidth+2*canvasAdjust, canvasHeight+2*canvasAdjust);
    frameRate(1/dt);
    background(255);
    textSize(28);
    strokeWeight(6);

}
function draw(){
    background(255);
    stroke(100);
    for (let i = 0; i<r.walls.length; i++){
        line(r.walls[i].startX*canvasWidth/r.width + canvasAdjust,r.walls[i].startY*canvasHeight/r.height+ canvasAdjust,r.walls[i].endX*canvasWidth/r.width+ canvasAdjust,r.walls[i].endY*canvasHeight/r.height+ canvasAdjust);

    }
    stroke(255,0,0);
    for (let i = 0; i<r.exits.length; i++){
        line(r.exits[i].startX*canvasWidth/r.width+ canvasAdjust,r.exits[i].startY*canvasHeight/r.height+ canvasAdjust,r.exits[i].endX*canvasWidth/r.width+ canvasAdjust,r.exits[i].endY*canvasHeight/r.height+ canvasAdjust);
    }
    for (let i = 0; i<r.pedestrians.length; i++){
        r.pedestrians[i].move();
        r.pedestrians[i].display();
    }
}