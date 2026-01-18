
# **RED**     

**Constants**   
GoalX = 132     
GoalY = 135     
TICKS_PER_REV = ???     
TICKS_PER_DEGREE = ???  
StartHeading = 0; (Ideally) *Can be set*

**Updated Variables**   
CurrentX = x    
CurrentY = y    
CurrentHeading = heading    
CorrectedHeading = atan(GoalY - CurrentY, GoalX - CurrentX)     
HeadingError = CurrentHeading - CorrectedHeading        
CurrentPos = servo.getCurrentPosition    
***Servo.setPoint(CurrentPos - HeadingError * TICKS_PER_DEGREE)***
