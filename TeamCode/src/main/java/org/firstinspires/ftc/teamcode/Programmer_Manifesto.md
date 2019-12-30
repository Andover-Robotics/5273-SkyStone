# Code Structure and Management (The Programmers' Manifesto)

The manifesto for all of ARC Thunder's code. Follow these practices and life will be easier for all of us.

## **Autonomous** OpMode naming conventions

* Begin the name of each OpMode with the alliance it should be run on, if it matters (`Red` or `Blue`). This may be shortened to `R` and `B`.
    * If the OpMode is not impacted by alliance (it is from `alliance_insignificant`), do not specify an alliance in the OpMode's name
* Provide a brief name that distinguishes the actions of this OpMode from the others (eg `Park` makes it clear that this OpMode only parks under the bridges)
* End the name with the intended starting position, in terms of tiles
    * Tiles are numbered from 1 to 6, with 1 being the depot and 6 being the building site
    * The robot is to be placed against the edge of the tile closer to the depot unless otherwise specified in the OpMode's name (`Top` for the edge closer to the building zone `Middle` for the center of the tile (not recommended) and `Bottom` for the edge closer to the depot)
* Examples of proper names:
    * `Blue Skystone (Tile 2)`
    * `Red Foundation (Top Tile 5)`
    * `Park (Middle Tile 3)`
    * `R Foundation (Bottom Tile 4)`
    
## OpMode grouping conventions
* If the OpMode is purely test code and not to be used in competition, set the group to `Test`
* If the OpMode has a purpose but is not a competition OpMode (such as a tuning class), set the group to `AB`
* If the OpMode is to be used in competition:
    * If the alliance is insignificant to the function of the OpMode (it is a `TeleOp` or in `alliance_insignificant`), set the group to `AA`
    * If the OpMode must be run for a specific alliance:
        * Set the group to `AAR` if the OpMode is intended to be run on the **Red** alliance
        * Set the group to `AAB` if the OpMode is intended to be run on the **Blue** Alliance

## `teleop` Package Specifications
`TeleOp` OpModes should be placed in this package. Be sure to follow the grouping conventions. Code intended to tune or reset the robot (such as an OpMode to reset the slides or tune the skystone detector with the controller) should be included in this package.

## `test` Package Specifications
OpModes intended to test a part or function of the robot (not intended for tuning or competition use) should be placed here. Be sure to follow the grouping conventions.

## `vision` Package Specifications
Classes related to computer vision software, such as skystone detection, should be placed here. Do not put OpModes in this package.

## Autonomous Package Specifications
What to put in each subpackage of the package `org.firstinspires.ftc.teamcode.autonomous`. 

In general, **all autonomous OpModes should extend `AutonomousMaster` at some point in the inheritance hierarchy.** Utilize the inherited fields to simplify code and reduce repetetive code.

### `alliance_insignificant`
Autonomous OpModes in this package should function **exactly the same** in the specified starting position on **both sides of the field**. In other words, the function of the OpMode is not dependent on the current alliance.

### `base_classes`
**Abstract** base classes of OpModes should be put in this package. They should all `extend AutonomousBase` and include `super.runOpMode()` in their overridden `runOpMode()`. This allows them to use the field `RobotAlliance currentAlliance` during autonomous commands.

**When should a class be made a base class?** 

An OpMode should be turned into a base class when its actions will **slightly differ with alliance** for example, this could be turning counterclockwise in some locations instead of clockwise. The majority of the movements should be equivalent in magnitude and only differ in direction. 

**How should base classes be used?**

A provided base class should have an implementation in both the `red` and `blue` packages. The class name should be the same as that of the base without the `Base` suffix. For example, there should be 2 classes named `SampleTile2`, one in `blue` and one in `red`, both of which `extend SampleTile2Base`. The classes are simple, consisting of the following:

`org.firstinspires.ftc.teamcode.autonomous.red.SampleTile2`:
```java
@Autonomous(name = "Red Sample (Tile 2)", group = "AAR")
public class SampleTile2 extends SampleTile2Base {
    @Override
    protected RobotAlliance getCurrentAlliance() {
        return RobotAlliance.RED;
    }
}
```

`org.firstinspires.ftc.teamcode.autonomous.blue.SampleTile2`:
```java
@Autonomous(name = "Blue Sample (Tile 2)", group = "AAB")
public class SampleTile2 extends SampleTile2Base {
    @Override
    protected RobotAlliance getCurrentAlliance() {
        return RobotAlliance.BLUE;
    }
}
``` 

They simply provide a name for the OpMode and a group, and implement the abstract method `getCurrentAlliance()`. The `runOpMode()` method is not changed, as `SampleTile2Base`'s `runOpMode()` adapts to the given implementation of `getCurrentAlliance()` to move properly.

Ideally, the `blue` and `red` packages should mostly contain implementations of `base_classes` OpModes. 

### `blue`
OpModes intended to be run from the specified starting position on the **blue alliance** go here.

### `red`
OpModes intended to be run from the specified starting position on the **red alliance** go here.
