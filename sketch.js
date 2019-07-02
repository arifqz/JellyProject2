

let video;
let poseNet;
let poses = [];

var yoff1 = 0.0;
let proto = []

var theta = 1;
var yoff = 0.0;

let shape = [];
let shapeys = [];
const segLen = 4;
const numSegs = 100;

let size = 10;

let minSize = 40;

let maxSize = 50;

let sizeSpeed = 0.0025;
var flowfield;



function setup() {
  createCanvas(windowWidth, windowHeight);
  
  flowfield = new FlowField(20);
  for (let i = 0; i < 25; i++){
    proto.push(new protozoa());
  }
  for (let i = 0; i < 1; i++) {
    shape.push(new Blobbytent(random(1, width), random(1, height)));
  }
}

function mousePressed() {
 shape.push(new Blobbytent(random(1, width), random(1, height)));

}

function modelReady() {
  console.log('model ready');
}


function draw() {
  background(0);
//     push();
//  textSize(40);
//   noStroke();
//   fill(255, 0,0);
//   text(int(mouseX), 200, 40);
//   text(int(mouseY), 300, 40)
//   textAlign(LEFT);
//   if (poses) {
// text(int(shapeys.length),120,40);
//      text(int(poses.length),90,40);
//      text(int(shape.length),90,100);
//   }
//   text(int(frameRate()),30,40);
//   pop();

 
  

  for (let i = 0; i < shape.length; i++) {
    
    shape[i].show();
    
    shape[i].follow(flowfield);
    shape[i].edges();
    shape[i].applyBehaviorsSeparate(shape);
    shape[i].makeLegs();
    for (let k = 0; k < shapeys.length; k++){
    shape[i].applyBehaviorsFlee(shapeys[k]);
    // shape[i].intersects(shapeys[k]);

  }
  
}

  for (var i = 0; i < proto.length; i++) {
  proto[i].render();
  }
}





class Segment {
  constructor(x, y, len, angle, id) {
    this.a = createVector(x, y);
    this.len = len;
    this.angle = angle;
    this.id = id;
    this.parent = null;
    this.reCalculate();
  }

  createParent(len, angle, id) {
    let parent = new Segment(0, 0, len, angle, id);
    this.parent = parent;
    parent.follow(this.a.x, this.a.y);
    return parent;
  }

  reCalculate() {
    let dx = cos(this.angle) * this.len;
    let dy = sin(this.angle) * this.len;
    this.b = createVector(this.a.x + dx, this.a.y + dy);
  }

  follow(tx, ty) {
    let target = createVector(tx, ty);
    let dir = p5.Vector.sub(target, this.a);
    this.angle = dir.heading();
    dir.setMag(this.len);
    dir.mult(-1);
    this.a = p5.Vector.add(target, dir);
  }

  update() {
    this.reCalculate();
  }

  show() {
    stroke(255);
    line(this.a.x, this.a.y, this.b.x, this.b.y);
  }
}
class RobotArm {
  constructor(x, y, numSegs, segLen, angle) {
    this.base = createVector(x, y);
    this.segs = [];
    this.segs[0] = new Segment(x, y, segLen, angle, 0);
    for (let i = 1; i < numSegs; i++) {
      this.addSegment(segLen, 0, i + 1);
    }
  }

  addSegment(len, angle) {
    let c = this.segs[this.segs.length - 1];
    let s = new Segment(0, 0, len, angle, this.segs.length);
    c.parent = s;
    this.segs.push(s);
    s.follow(c.a.x, c.a.y);
    return s;
  }

  update2(x, y) {
    for (let i = 0; i < this.segs.length; i++) {
      const seg = this.segs[i];
      // seg.update();
      if (i === 0) {
        seg.follow(x, y);
      } else {
        const previous = this.segs[i - 1];
        seg.follow(previous.a.x, previous.a.y);
      }
    }

    const last = this.segs.length - 1;
    const s = this.segs[last];
  
    s.reCalculate();
    for (let i = last - 1; i >= 0; i--) {
      const seg = this.segs[i];
      const next = this.segs[i + 1];
    
      seg.reCalculate();
    }
  }

  show() {
    this.segs.forEach(s => s.show());
  }
}


class Blobbytent {
  constructor(x, y) {
    this.pos = createVector(x, y);
    this.vel = createVector(0, 0);
    this.acc = createVector(0, 0);
    this.rot = random(360);
    this.radius = random(50, 70);
    this.mass = 20;
    this.G = 1;
    this.maxspeed = 5;
    this.maxforce = .2;
    this.legs = [];
    this.newSegs = numSegs * this.radius * .0092;
    for (let i = 0; i < 8; i++) {

      this.legs.push(new RobotArm(width, height, this.newSegs, segLen, 0));
    }
  }
  flee(target){
    var desired = p5.Vector.sub(target.pos, this.pos);
    var d = desired.mag();
    if (d <250) {
    desired.setMag(this.maxspeed);
    desired.mult(-1);
    var steer = p5.Vector.sub(desired, this.vel);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0,0);
  }

}


  applyBehaviorsSeparate(shape) {
    var separateForce = this.separate(shape);
    separateForce.mult(5);
    this.applyForce(separateForce);
  }
  applyBehaviorsFlee(shape) {
    var fleeForce = this.flee(shape);
    fleeForce.mult(5);
    this.applyForce(fleeForce);
  }

 
  move() {
    this.vel.add(this.acc);
    this.vel.limit(this.maxspeed);
    this.pos.add(this.vel);
    this.acc.mult(0);
  }
  makeLegs() {
    this.radius2 = this.radius;

    for (let i = 0; i < this.legs.length; i++) {

      this.legs[0].update2(this.pos.x + (this.radius2 * -.3), this.pos.y + (this.radius2 * .4));
      this.legs[1].update2(this.pos.x + (this.radius2 * .2), this.pos.y + (this.radius2 * -.4));
      this.legs[2].update2(this.pos.x + (this.radius2 * .4), this.pos.y + (this.radius2 * -.5));
      this.legs[3].update2(this.pos.x + (this.radius2 * -.13), this.pos.y + (this.radius2 * -.7));
      this.legs[4].update2(this.pos.x + (this.radius2 * .42), this.pos.y + (this.radius2 * .7));
      this.legs[5].update2(this.pos.x + (this.radius2 * .71), this.pos.y + (this.radius2 * .5));
      this.legs[6].update2(this.pos.x + (this.radius2 * -.71), this.pos.y + (this.radius2 * -.3));
      this.legs[7].update2(this.pos.x + (this.radius2 * -.4), this.pos.y + (this.radius2 * -.4));
      
    
      this.legs[i].show();
    }

  }
  show() {
var theta = this.vel.heading() + PI;
    push();
    var radius = 30;
    translate(this.pos.x, this.pos.y);
    rotate(theta);
    this.r = this.pos.x + this.pos.y;
    fill(255);
    stroke(255);
    beginShape();
    let xoff = 0;
    for (var a = 0; a < TWO_PI + .04; a += 0.2) {
      var offset1 = map(noise(xoff, yoff), 0, 1, -15, 15);
      var offset2 = map(noise(xoff, yoff), 0, 1, -15, 15);
      var r = this.radius + random(offset1, offset2);
      var x = r * cos(a);
      var y = r * sin(a);
      vertex(x, y);
      // this.tents[0].update(x,y);
      // this.tents[0].show();
      xoff += 0.1;
    }
    endShape(CLOSE);
    yoff += 0.01;

    pop();
  }

  applyForce(force) {
    this.acc.add(force);
    }
  separate(shape) {
            var desiredseparation = this.radius * 2;
            var sum = createVector();
            var count = 0;
            for (var i = 0; i < shape.length; i++) {
              var d = p5.Vector.dist(this.pos, shape[i].pos);
              if ((d > 0) && (d < desiredseparation)) {
                var diff = p5.Vector.sub(this.pos, shape[i].pos);
                diff.normalize();
                diff.div(d);
                sum.add(diff);
                count++;
              }
            }
            if (count > 0) {
              sum.div(count);
              sum.normalize();
              sum.mult(this.maxspeed);
              sum.sub(this.velocity);
              sum.limit(this.maxforce);
            }
            return sum;
          }



follow(flow) {
var desired = flow.lookup(this.pos);
desired.mult(this.maxspeed);
var steer = p5.Vector.sub(desired, this.vel);
steer.limit(this.maxforce);
this.applyForce(steer);
  this.vel.add(this.acc);
    this.vel.limit(this.maxspeed);
    this.pos.add(this.vel);
    this.acc.mult(0);

}

  edges() {
     
    if (this.pos.x < 0 - 100) {this.pos.x = windowWidth; this.pos.y = random(windowHeight);};
    if (this.pos.y < 0 - 100) {this.pos.y = windowHeight;  this.pos.y = random(windowHeight);};
    if (this.pos.x > windowWidth + 100) {this.pos.x = 0 };
    if (this.pos.y > windowHeight + 100 ) {this.pos.y = 0 - 100; this.pos.x = random(windowHeight);};
   
  }
}



class protozoa {
  constructor() {
  this.pos = createVector(random(width), random(height));
  this.vel = createVector(random(width), random(height));
  this.acc = createVector(0, 0);
  this.mass = 1;
    this.rot = random(360);
    this.r = 20;
   
  
  }
render(){
  this.pos = createVector(this.pos.x +random(-1,1), this.pos.y + random(-5,5));
  
  push();
  stroke(255);
  noFill();
  this.r = random(9, 10);
  beginShape();
  let xoff1 = 0;
  translate(this.pos.x, this.pos.y);
  for (var i = 0; i< TWO_PI; i+=0.5){
      var offset1 = map(noise(xoff1, yoff1), 0, 1, 2, 10);
      var offset2 = map(noise(xoff1, yoff1), 0, 1, 2, 20);
      var r = this.r + random(offset1, offset2);

    var x = r *cos(i);
    var y = r *sin(i);
    ellipse(x, y, 1, 2);
    rect(3,3,2,1);
    vertex(x,y);
    xoff1 += 0.1;
    yoff1 += 0.1;
    this.rot += 0.1;
    rotate(noise(this.rot));
  }
  endShape(CLOSE);
  pop();
}
update(){
   this.pos.add(this.vel); 
  }
edges() {
    if (this.pos.x < -this.r) {this.pos.x = width + this.r}
    if (this.pos.y < -this.r) {this.pos.y = height + this.r}
    if (this.pos.x > width + this.r) {this.pos.x = -this.r}
    if (this.pos.y > height + this.r) {this.pos.y = -this.r}
  }
intersects(object2) {
    var ds = p5.Vector.dist(this.pos, object2.pos);
    var rd = this.radius + object2.radius;
    if (ds > rd) {
stopsound();
return false;
    } 
    if (ds < rd) {
playsound();

return true;
    }
  }
}