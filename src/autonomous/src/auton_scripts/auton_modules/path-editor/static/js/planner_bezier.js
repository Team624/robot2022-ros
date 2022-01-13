// Start position of the robot
var lastStartingBezierX = 10;
var lastStartingBezierY = 10;

var bezierArray = [] 
var interval = 20;

function createNewBezier(){
    var curve = new Bezier(lastStartingBezierX,lastStartingBezierY,lastStartingBezierX + 30, lastStartingBezierY + 30, lastStartingBezierX + 70, lastStartingBezierY + 70, lastStartingBezierX + 100, lastStartingBezierY + 100);
    lastStartingBezierX = curve.points[3].x;
    lastStartingBezierY = curve.points[3].y;

    let x = new CodeExample();
    x.setColor("#FF0000");
    x.drawSkeleton(curve);
    //x.drawCurve(curve);
    var LUT = curve.getLUT(interval);
    //console.log(LUT);
    autonData.paths[autonData.paths.length - 1].control_points = curve.points
    LUT.forEach(p =>
        autonData.paths[autonData.paths.length - 1].goals.push({"x": p.x*widthRatio, "y": -p.y*heightRatio, "t": p.t}) && x.drawCircle(p,2)
    );

    bezierArray.push([curve, x]);
    addOnUpdateEvent(bezierArray.length - 1);
}

function createSavedBezier(e, index){
    var curve = new Bezier(e.control_points[0].x, e.control_points[0].y, e.control_points[1].x, e.control_points[1].y, e.control_points[2].x, e.control_points[2].y, e.control_points[3].x, e.control_points[3].y);
    lastStartingBezierX = curve.points[3].x;
    lastStartingBezierY = curve.points[3].y;

    let x = new CodeExample();
    x.setColor("#FF0000");
    x.drawSkeleton(curve);
    //x.drawCurve(curve);
    var LUT = curve.getLUT(interval);
    LUT.forEach(p => x.drawCircle(p,2));

    bezierArray.push([curve, x]);
    addOnUpdateEvent(bezierArray.length - 1);
}

function loadBeziers(){
    console.log("Loaded beziers")
    autonData.paths.forEach(createSavedBezier);
}

function deleteBezier(index)
{
    bezierArray[index][1].reset()
    bezierArray.splice(index, 1)
    console.log(bezierArray)

    if (bezierArray.length == 0)
    {
        lastStartingBezierX = autonData.start_pose[0]/widthRatio
        lastStartingBezierY = -autonData.start_pose[1]/heightRatio
    }
    else
    {
        lastStartingBezierX = bezierArray[bezierArray.length - 1][0].points[3].x;
        lastStartingBezierY = bezierArray[bezierArray.length - 1][0].points[3].y;
    }
}

function addOnUpdateEvent(index)
{
    var element = bezierArray[index];
    handleInteraction(element[1].getCanvas(), element[0]).onupdate = evt => {
        if (bezierArray.indexOf(element) != -1 && autonData.paths[index] != null)
        {
            if (bezierArray.indexOf(element) == 0)
            {
                element[1].reset();
                element[0].points[0].x = autonData.start_pose[0]/widthRatio
                element[0].points[0].y = -autonData.start_pose[1]/heightRatio

            }
            if (bezierArray.indexOf(element) == bezierArray.length -1)
            {
                lastStartingBezierX = element[0].points[3].x;
                lastStartingBezierY = element[0].points[3].y;
            }
                
            element[1].drawSkeleton(element[0]);
            //element[1].drawCurve(element[0]);
            var LUT = element[0].getLUT(interval);
            autonData.paths[index].control_points = element[0].points
            autonData.paths[index].goals = []
            LUT.forEach(p =>
                autonData.paths[index].goals.push({"x": p.x*widthRatio, "y": -p.y*heightRatio, "t": p.t}) && element[1].drawCircle(p,2)
            );
        }
        else
        {
            
        }
    };
}
