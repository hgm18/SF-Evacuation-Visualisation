//Global variables
const canvasWidth = 500;
const canvasHeight = 500;
const canvasAdjust = 10;
const m = 1;//mass
const desired_speed = 1.34;//mean desired speed
const tau = 0.5//relaxation time
const expon = 9 * Math.E;
const dt = 1/45;
//Potentials
const V0 = 2.1;
const sigma = 0.3;
const U0 = 10;
const R = 0.2;
const neutral_distance = 3;
const private_distance = 0.5;
const private_force = 4;
const Apower = 12;
const Bpower = -6;
const A = -(private_force*private_distance**Apower)/((neutral_distance**(Bpower-Apower))*((private_distance**(Apower-Bpower)))-1);
const B = A * neutral_distance ** (Bpower-Apower);



function findDistanceToWall(pointX, pointY, wall){//Finding shortest path between a point and a line segment (used for exits, walls)
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
function findDistanceToExit(pointX, pointY, exit){//Finding shortest path between a point and a line segment (used for exits, exits)
    let dx = exit.bufferedEndX - exit.bufferedStartX;
    let dy = exit.bufferedEndY - exit.bufferedStartY;
    let closest;
    if ((dx == 0) && (dy == 0)){
        // It's a point not a line segment.
        closest = [exit.bufferedStartX, exit.bufferedStartY];
        dx = pointX - exit.bufferedStartX;
        dy = pointY - exit.bufferedStartY;
        return [closest,Math.sqrt(dx * dx + dy * dy)];
    }
    let t = ((pointX-exit.bufferedStartX)*dx + (pointY-exit.bufferedStartY)*dy)/(dx*dx + dy*dy);
    if(t<0){
        closest = [exit.bufferedStartX, exit.bufferedStartY];
        dx = pointX-exit.bufferedStartX;
        dy = pointY-exit.bufferedStartY;
    }
    else if(t>1){
        closest = [exit.bufferedEndX, exit.bufferedEndY];
        dx = pointX-exit.bufferedEndX;
        dy = pointY-exit.bufferedEndY;
    }
    else{
        closest = [exit.bufferedStartX + t*dx, exit.bufferedStartY + t*dy]
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
        this.diameterX = 0.25* canvasWidth/room.width;
        this.diameterY = 0.25* canvasHeight/room.height
        this.max_speed = 1.3 * this.desired_speed;
        //this.x and this.y is for canvas display, actual positions are in this.pos
        this.x = x * canvasWidth/room.width + canvasAdjust;
        this.y = y * canvasHeight/room.height + canvasAdjust;
        this.room = room
        this.exit_force = [0,0];
        this.border_force = [0,0];
        this.pedestrian_force = [0,0];
    }
    display(){
        fill(0);
        stroke(0);
        ellipse(this.x,this.y,this.diameterX, this.diameterY);
    }
    calcTargetAttractiveForce(target){//Calculating attractive force to a target
        let desiredVX;
        let desiredVY;
        let sX = Math.abs(this.pos[0]-target[0]);
        let sY = Math.abs(this.pos[1]-target[1]);
        desiredVY = Math.sqrt((this.desired_speed**2)/(1+(sX/sY)**2))//Y component of desired velocity
        desiredVX = Math.sqrt((this.desired_speed**2)/(1+(sY/sX)**2))//^ x component
        if (this.pos[0]>target[0]){//desiedVX and VY always positive from previous calculation, so need to adjust here
            desiredVX *= -1;
        }
        if (this.pos[1]>target[1]){
            desiredVY *= -1;
        }
        let Fx = m*(desiredVX - this.vel[0])/tau;
        let Fy = m*(desiredVY - this.vel[1])/tau;
        return [Fx,Fy];
    }
    pedpedForce(other_ped){
        let dx = other_ped.pos[0]-this.pos[0];
        let dy = other_ped.pos[1]-this.pos[1];
        let rab = Math.sqrt (dx**2 + dy**2);
        let Ft = -A/(rab**Apower)  +  B/(rab**Bpower);
        let Fx = Ft * dx/rab;
        let Fy = Ft * dy/rab;
        return [Fx,Fy];
    }
    calcPedPedForces(){
        let totalFx = 0;
        let totalFy = 0;
        let Force;
        for(let i = 0; i<this.room.pedestrians.length; i++){
            if (this!==this.room.pedestrians[i]){
                Force = this.pedpedForce(this.room.pedestrians[i]);
                totalFx+=Force[0];
                totalFy+=Force[1];
            }
        }
        return [totalFx,totalFy];
    }
    pedBorderRepulsivePotential(raB){//UaB in paper
        return U0 * expon**(-1 * raB/R);//Changed to 18 from e, now can actually reach exit (smallest door tested is 0.5m wide, any smaller unrealistic)
    }
    pedBorderRepulsiveForce(wall, delta = 0.001){//delta is the small step for finite difference differentiation 
        let closest = findDistanceToWall(this.pos[0], this.pos[1], wall);
        let raB = closest[1];
        let UaB = this.pedBorderRepulsivePotential(raB);
        let raBdx = findDistanceToWall(this.pos[0]-delta, this.pos[1], wall)[1];
        let raBdy = findDistanceToWall(this.pos[0], this.pos[1]-delta, wall)[1];
        let UaBdx = this.pedBorderRepulsivePotential(raBdx);
        let UaBdy = this.pedBorderRepulsivePotential(raBdy);
        let Fx = -1 * (UaB-UaBdx)/delta;//This is grad calculation, using finite difference partial differentiation
        let Fy = -1 * (UaB - UaBdy)/delta;
        
        return ([Fx,Fy]);
    }
    calcBorderForces(){//Calculating the total border forces on the pedestrian
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
    move(){//Moving a pedestrian
        let target=0;
        let targForce = [0,0];
        let borderForce = [0,0];
        let pedForces = [0,0];
        let minDist = 9999;
        let targX;
        let targY;
        let targDist;
        let potentialTarget;
        let desiredExit;
        for (let i = 0; i < this.room.exits.length; i++){//Finding closest target
            potentialTarget = findDistanceToExit(this.pos[0], this.pos[1], this.room.exits[i]);
            targX = potentialTarget[0][0];
            targY = potentialTarget[0][1];
            targDist = Math.sqrt((targX - this.pos[0])**2 + (targY - this.pos[1])**2);
            if (targDist<minDist){
                target = potentialTarget[0];
                desiredExit = this.room.exits[i];
                minDist = targDist;
            }
        }
        if (target!==0){
            targForce = this.calcTargetAttractiveForce(target);
        }
        console.log(target);
        borderForce = this.calcBorderForces();
        pedForces = this.calcPedPedForces();
        this.desiredExit = desiredExit;
        this.exit_force = targForce;
        this.border_force = borderForce;
        this.pedestrian_force = pedForces;
        this.acc[0] = (targForce[0]+borderForce[0]+pedForces[0])/m;
        this.acc[1] = (targForce[1]+borderForce[1]+pedForces[1])/m;
        this.acc[0] += this.acc[0]*(0.2*(Math.random()-0.5))//Fluctuations(+-10%)
        this.acc[1] += this.acc[1]*(0.2*(Math.random()-0.5))
        this.vel[0] += this.acc[0]*dt;
        this.vel[1] += this.acc[1]*dt;
        let speed = Math.sqrt(this.vel[0]**2 + this.vel[1]**2)
        if (speed>this.max_speed){
            this.vel[0] = this.vel[0] * this.max_speed/speed;
            this.vel[1] = this.vel[1] * this.max_speed/speed;
        }
        this.pos[0] += this.vel[0] * dt;
        this.pos[1] += this.vel[1] * dt
        this.x = this.pos[0] * canvasWidth/this.room.width + canvasAdjust;//x coordinate on canvas
        this.y = this.pos[1] * canvasHeight/this.room.height + canvasAdjust;

    }
}
class Wall{//Wall class
    constructor(startX, endX, startY, endY){
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.type = 'wall';
    }
}
class Exit{//Exit
    constructor(startX, endX, startY, endY){
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
        this.centre = [(this.startX+this.endX)/2,(this.startY+this.endY)/2];//Used initally for target finding, now using findDistanceToWall().
        this.type = 'exit';
        this.bufferedStartX = this.startX;
        this.bufferedEndX = this.endX;
        this.bufferedStartY = this.startY;
        this.bufferedEndY = this.endY;
        if (this.startX === this.endX && this.endY - this.startY >= 0.5){
            this.bufferedStartY = this.startY + 0.25;
            this.bufferedEndY = this.endY - 0.25;
        }
        else if (this.startY === this.endY && this.endX - this.startX >=0.5){
            this.bufferedStartX = this.startX + 0.25;
            this.bufferedEndX = this.endX - 0.25;
        }
    }
}
class Room{//Room
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
    create1Exit(startX = this.width, endX = this.width, startY = 0.45*this.height, endY = 0.55*this.height){
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
    breaksInWalls(){//Function o create 'breaks' in walls due to exits. only works for rectangular room (pretty sure)
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
                    this.walls[w].endY = exitStartY;//Adjusts end coordinate
                    this.walls.push(new Wall(wallStartX, wallEndX, exitEndY, this.height));//Creates new wall for the remainder after the exit
                }
                else if (wallStartY === wallEndY && exitStartY === exitEndY && wallStartY === exitStartY){//horizontal wall
                    this.walls[w].endX = exitStartX;
                    this.walls.push(new Wall(exitEndX, this.width, wallStartY, wallEndY));
                }
            } 
        }
    }
    setupDemoRoom(){//Function to create a basic rom
        this.generateWall();//Wall is generated
        this.create1Exit(0,0);//Exits are placed
        this.create1Exit();
        this.breaksInWalls();//Parts of the wall with exits are "broken"
    }
    generatePed(n){
        for (let i = 0; i<n ; i++){
            this.create1Ped();
        }
    }
    step(){
        let pedLen = this.pedestrians.length;
        
        for(let i = 0; i<pedLen; i++){
            this.pedestrians[i].move();
            //console.log(this.pedestrians[i].pos)
        }
        for (let i = 0; i<this.pedestrians.length; i++){
            this.pedestrians[i].display();
        }
        let currExit;
        let newPeds=[];
        if (this.exits.length>0){
            for(let i = 0; i<pedLen; i++){//Removing pedestrians upon reaching exit
                newPeds.push(this.pedestrians[i]);
                currExit = this.pedestrians[i].desiredExit;
                if(currExit.startX === currExit.endX){//Vertical Exit
                    if (this.pedestrians[i].pos[1]<currExit.endY && this.pedestrians[i].pos[1]>currExit.startY ){
                        if (currExit.startX === 0 && this.pedestrians[i].pos[0]<currExit.startX + 0.0001){
                            newPeds.pop()
                        }
                        else if(currExit.startX === this.width && this.pedestrians[i].pos[0]>currExit.startX -0.0001){
                            newPeds.pop();
                        }
    
                    }
                }
                else if(currExit.startY === currExit.endY){//Horizontal Exit
                    if (this.pedestrians[i].pos[0]<currExit.endX && this.pedestrians[i].pos[0]>currExit.startX && this.pedestrians[i].pos[1]>currExit.startY - 0.01 && this.pedestrians[i].pos[1]<currExit.startY + 0.01){
                        if (currExit.startY === 0 && this.pedestrians[i].pos[1]<currExit.startY + 0.0001){
                            newPeds.pop()
                        }
                        else if(currExit.startY === this.height && this.pedestrians[i].pos[1]>currExit.startY -0.0001){
                            newPeds.pop();
                        }
                    }
                }
            }
            this.pedestrians = newPeds;
        }
        
        
            
        
    }
}

let r = new Room(20,20)
r.setupDemoRoom();
r.generatePed(1);
for (let i=0; i<10; i++){

r.step();
console.log(r.pedestrians[0].x);
console.log(r.pedestrians[0].y);
console.log(r.pedestrians[0].exit_force[0]);
console.log(r.pedestrians[0].exit_force[1]);
}

//console.log(r.walls[0]);

function drawArrow(vec0, vec1, myColor){ //code adapted from https://p5js.org/reference/#/p5.Vector/magSq and https://stackoverflow.com/questions/44874243/drawing-arrows-in-p5js
    let offset = 4;
    stroke(myColor);
    strokeWeight(3);
    fill(myColor);
    push();
    line(vec0.x, vec0.y, vec0.x+vec1.x, vec0.y+vec1.y);
    let angle = atan2(vec1.y, vec1.x);
    translate(vec0.x+vec1.x, vec0.y+vec1.y);
    rotate(angle + HALF_PI);
    triangle(-offset*0.5, offset, offset*0.5, offset, 0, -offset/2); 
    pop();
     
}



function showExitForceVect(Fx, Fy, posx, posy){
    let i = Fx;
    let j = Fy;
    let x = posx;
    let y = posy;
    let v0 = createVector(x, y) //pedestrian centre of sphere
    let v1 = createVector(i, j); //force vector
    drawArrow(v0, v1, 'black');
    
}

function showBorderForceVect(Fx, Fy, posx, posy){
    let i = Fx;
    let j = Fy;
    let x = posx;
    let y = posy;
    let v0 = createVector(x, y);
    let v1 = createVector(i, j);
    drawArrow(v0, v1, 'red');

}

function showPedForceVect(){


}




function setup(){
    createCanvas(canvasWidth+2*canvasAdjust, canvasHeight+2*canvasAdjust);
    frameRate(1/dt);
    background(255);
    textSize(28);
    strokeWeight(6);

}
function draw(){
    background(255);//Whiting out canvas
    stroke(100);
    for (let i = 0; i<r.walls.length; i++){//Drawing walls
        line(r.walls[i].startX*canvasWidth/r.width + canvasAdjust,r.walls[i].startY*canvasHeight/r.height+ canvasAdjust,r.walls[i].endX*canvasWidth/r.width+ canvasAdjust,r.walls[i].endY*canvasHeight/r.height+ canvasAdjust);

    }

    //SHOWING EXIT FORCE
    for (let i=0; i<r.pedestrians.length; i++){
        //let Fx = (r.pedestrians[i].exit_force[0])*((canvasWidth+2*canvasAdjust)/r.width);
        //let Fy = (r.pedestrians[i].exit_force[1])*((canvasHeight+2*canvasAdjust)/r.height);
        let Fx = r.pedestrians[i].exit_force[0]*canvasWidth/r.width;
        let Fy = r.pedestrians[i].exit_force[1]*canvasHeight/r.height;
        let posx = r.pedestrians[i].x;
        let posy = r.pedestrians[i].y;
        
        showExitForceVect(Fx,Fy,posx,posy);
        
    }

    //SHOWING BORDER FORCE
    /*for (let i=0; i<r.pedestrians.length; i++){
        let Fx = (r.pedestrians[i].border_force[0])*((canvasWidth+2*canvasAdjust)/r.width);
        let Fy = (r.pedestrians[i].border_force[1])*((canvasHeight+2*canvasAdjust)/r.height);
        let posx = r.pedestrians[i].x;
        let posy = r.pedestrians[i].y;
        
        showBorderForceVect(Fx,Fy,posx,posy);
    }*/

    



   
    stroke(255,0,0);
    for (let i = 0; i<r.exits.length; i++){//Drawing exits
        line(r.exits[i].startX*canvasWidth/r.width+ canvasAdjust,r.exits[i].startY*canvasHeight/r.height+ canvasAdjust,r.exits[i].endX*canvasWidth/r.width+ canvasAdjust,r.exits[i].endY*canvasHeight/r.height+ canvasAdjust);
    }


    r.step()
   
}

