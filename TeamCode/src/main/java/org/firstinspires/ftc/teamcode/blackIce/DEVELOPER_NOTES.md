Note: all this code is written by Jacob Ophoven

Definitions:
Parametric = a function that describes a path using a single variable (t)
Designed for modularity, readability, expandability, minimize redundancy, easily chaining methods, and performance.

Side-effects of having chained methods but Minimizing Redundancy is have to make long generics
to allow classes to extend PathConfig so they can use it's chain methods to configure the parameters.

Overuse of inheritance but it is used to reduce redundancy and allow for chaining methods without
remaking getters and setters for every class.

Only why comments. no what comments. that is a sign of bad and unreadable code.

@apiNote - API note for common users.
@implNote - Implementation note for developers.


Follow style guidelines: https://www.oracle.com/java/technologies/javase/codeconventions-fileorganization.html



Heading 0 is the front side of the robot facing away from the starting wall
positive X-axis is forward for the robot with 0 heading (facing the center of the field)
positive Y-axis is strafing the robot to the left from 0 heading

This is the axis with the robot's HEADING AT 0 DEGREES:
^ +Y Axis
│      
│ ┌──────────┐
│ │          │
│ │          │--> FRONT OF ROBOT
│ │          │
│ └──────────┘
└──────────────> +X Axis

