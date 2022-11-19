# Team 2's code for CC6

## FieldSim setup
- Select Field2d in NetworkTables -> SmartDashboard -> Field
- Right-click on blue section of the Field window
- Edit name to "Cruise Control"
- Set units to "meters"
- Click Choose Image and select image in assets/field.png
- Set Field Width to 7.620 and Field Height to 6.096
- Set Style to "Box/Image"
- Set Width to 0.559 and Length to 0.686
- Click Close

## KeyboardSim setup
- Drag Keyboard0 from System Joysticks window to Joystick[0] in Joysticks window
- Right-click Keyboard0 in System Joysticks window and click Keyboard 0 Settings
- Increase Axes count to 5
- Under Axis 1, set key for Increase to "s" and Decrease to "w"
- Under Axis 4, set key for Increase to "d" and Decrease to "a"

## AutoSim setup
- Open Auto Chooser by clicking on NetworkTables -> Shuffleboard -> Auto -> Auto Chooser
- Select desired auto in dropdown

## PathPlanner setup
- Select Robot Project and select the "team-2-971" folder
- Create field called Cruise Control, select field.png from assets/, and set Pixels per Meter to 185.433071
- Set Robot Width to 0.56 and Robot Length to 0.69