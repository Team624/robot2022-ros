// GOALS

// This adds a goal to the data set
function addGoal(){
  var goalsData = autonData.paths;
  var goallength =goalsData.length + 1;
  autonData.paths.push({"id":autonData.paths.length, "name":"Path " + autonData.paths.length + 1, "constants_kP":5.0, "constants_kA":0.0, "constants_kB":0.0, "max_linear_speed":20.0, "min_linear_speed":0.1, "max_linear_acceleration":1E9, "linear_tolerance_outer":0.3, "linear_tolerance_inner":0.1, "max_angular_speed":2.0, "min_angular_speed":1.0, "max_angular_acceleration":1E9, "angular_tolerance_outer":0.2, "angular_tolerance_inner":0.1, "ignore_angular_tolerance":false, "forward_movement_only":false, "end_of_path_stop":true, "control_points":[], "goals": []})
  console.log("Added Goal to Path")
  console.log(autonData.paths)
  
  // Updates view
  //displayPathTable()
  displayGoalDropDown()
  createNewBezier();
  postJSON()
}

// This deletes a goal from the data
function deleteGoal(id){
  // autonData.paths.splice(id.slice(10,id.length),1);
  // var index = id.slice(10,id.length);
  // console.log("Deleted Path Index : " + index)
  // while (index < autonData.paths.length){
  //   autonData.paths[index].id -= 1;
  //   autonData.paths[index].name = "Path " + (autonData.paths[index].id + 1);
  //   // console.log(autonData.paths[index])
  //   index++;
  // }
  autonData.paths.splice(autonData.paths.length - 1,1);

  displayGoalDropDown()

  // Known error deletes the last index of the bezier
  deleteBezier(autonData.paths.length)
  postJSON()
}

// This displays the goal data for the selected path
function displayGoalDropDown(){
  var myContent = document.getElementById("dropdown-content");

  var elements = document.getElementsByClassName("GoalOption");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
    console.log(myContent)

    for (var x in autonData.paths){

      var myGoal = document.createElement('BUTTON');
      myGoal.className = "GoalOption";
      myGoal.id = "Goal " + autonData.paths[x].id;
      var value = autonData.paths[x].id + 1;
      myGoal.textContent = "Path " + value;
      myGoal.onclick = function() { 
        openRight(event, this.id + "Content");
        document.getElementById("dropdown-content").style.display = "none";  
        document.getElementById(this.id).style.background = ''
    }
      myContent.insertBefore(myGoal, myContent.firstChild);

    }

    displayGoalEditor()

}

// This displays the UI for editing values of the goal
function displayGoalEditor(){

  var myRightSideBar = document.getElementById("right_sidebar");

  var elements = document.getElementsByClassName("GoalContent");
    while(elements.length > 0){
        elements[0].parentNode.removeChild(elements[0]);
    }
  
    for (var x in autonData.paths){

      var myDiv = document.createElement('DIV');
      myDiv.className = "GoalContent";
      myDiv.id = "Goal " + autonData.paths[x].id + "Content";
      var value = autonData.paths[x].id + 1;
      var myH1 = document.createElement('H1');
      myH1.textContent = "Path " + value;

      var myDivData = document.createElement('DIV');
      myDivData.className = "GoalContentData";
      myDivData.id = "GoalContentData" + autonData.paths[x].id;

      var myGoalTrash = document.createElement('H4');
      var trash = '\u2716'
      myGoalTrash.textContent = trash
      myGoalTrash.id = "GoalTrash " + autonData.paths[x].id
      myGoalTrash.className = "GoalTrash"

      myGoalTrash.onclick = function() { 
        deleteGoal(this.id); 
      }

      myDiv.insertBefore(myGoalTrash, myDiv.firstChild);

      myDiv.insertBefore(myDivData, myDiv.firstChild);
      myDiv.insertBefore(myH1, myDiv.firstChild);
      myDiv.insertBefore(myGoalTrash, myDiv.firstChild);

      myRightSideBar.appendChild(myDiv);

      //displayGoalPose(autonData.paths[currentSelctedPathIndex].id);
      displayGoalConstants(autonData.paths[x].id)
      displayGoalLinear(autonData.paths[x].id);
      displayGoalAngular(autonData.paths[x].id);

    }
}