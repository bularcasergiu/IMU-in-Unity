#pragma strict

// Draws a horizontal slider control that goes from -10 to 10.
var robotBaseSliderValue : float = 0.0;

// Draws a horizontal slider control that goes from -10 to 10.
var robotUpperArmSliderValue : float = 0.0;

//These slots are where you will plug in the appropriate robot parts into the inspector.
var RobotBase : Transform;
var RobotUpperArm : Transform;

//These allow us to have numbers to adjust in the inspector for the speed of each part's rotation.
var baseTurnRate : float = 5;
var upperArmTurnRate : float = 5;

function Update(){

//rotating our base of the robot here around the Y axis and multiplying
//the rotation by the slider's value and the turn rate for the base.
RobotBase.Rotate (0, robotBaseSliderValue * baseTurnRate, 0);

//rotating our upper arm of the robot here around the X axis and multiplying
//the rotation by the slider's value and the turn rate for the upper arm.
RobotUpperArm.Rotate (robotUpperArmSliderValue * upperArmTurnRate, 0 , 0);

   if (Input.GetMouseButtonUp (0)) {
   
        //resets the sliders back to 0 when you lift up on the mouse click down.
        robotBaseSliderValue = 0;
        robotUpperArmSliderValue = 0;
    }

}

function OnGUI () {

//creates the slider and sets it 25 pixels in x, 25 in y, 100 wide and 30 tall.
robotBaseSliderValue = GUI.HorizontalSlider (Rect (25, 25, 100, 30), robotBaseSliderValue, -10.0, 10.0);

//creates the slider and sets it 25 pixels in x, 80 in y, 100 wide and 30 tall.
robotUpperArmSliderValue = GUI.HorizontalSlider (Rect (25, 80, 100, 30), robotUpperArmSliderValue, -10.0, 10.0);

}