This is a modification of the [RevRobotics MAXSwerve-Java-Template](https://github.com/REVrobotics/MAXSwerve-Java-Template).


## Modifications from the base template:
* Switched gyro to NavX
* Switch the `DriveSubsystem` to use `SwerveDrivePoseEstimator` for odometry collection.
* Log odometry for [AdvantageScope](https://docs.advantagescope.org/) use.
* Display [odometry on dashboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html) using `Field2d`.
* Added [PathPlanner](https://pathplanner.dev/) code for autonomous commands.
* Added [Elastic Dashboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/elastic.html) support.
* Added `VisionSubsystem` which uses [PhotonVision](https://photonvision.org/).
* Added code to support simulation.


## How to use this template:
Make as many robot programs as you like.  Code is as impermanent as scratch paper. Don't be afraid to throw it away and start over.
To create a new robot program:
* Visit our 'Java_Template' repository.  Click the green "Use this template" button in the upper right corner of the page.  Select "Create a new repository".
* Give the project a new repository name in your own account.  Turn off the "Include all branches" toggle.  Create the repository.
* In your new repository page:
    * Click the green "Code" button.
    * Copy the URL.
* Clone the code back to your laptop:
    * Start up [Visual Studio Code](https://code.visualstudio.com/docs/getstarted/getting-started).
    * Hit Control-Shift-P to bring up the command palette.
    * Type "git clone".
    * Paste in the URL.
* Make changes.
* Push your repository back to Github.
    * As you develop your code, commit and publish your work frequently.  Try to always make your commit messages meaningful, so the commit history will show how the project is evolving.
    

# TODO:
* Test on 2025 robot with RPi and PhotonVision
    * Test with the current Arducam_OV9281 and Raspberry Pi.  Verify that the `VisionSubsystem` returns an appropriate `Pose2d` whenever it sees an AprilTag.  The `Pose2d` should be pushed to the `DriveSubsystem` to update its odometry.   Watch that the `Field2d` widget on the dashboard always displays an appropriate robot location.
    * Retest with another USB camera, such as a [Microsoft LifeCam](https://en.wikipedia.org/wiki/LifeCam) or [Arducam 1080P HDR](https://www.amazon.com/dp/B0FX47YSJQ).   Determine if cheaper USB cameras are as effective as the global-shutter Arducams.
    * Retest with a [Raspberry Pi](https://www.raspberrypi.com/documentation/accessories/camera.html) camera.  Determine if this camera is a workable choice.
    * Retest with multiple cameras connected and configured.  It should be able to get `Pose2d` data from multiple cameras.  Estimate performance of multiple cameras on a Pi 5 based on pose accuracy and FPS.
* VisionSubsystem
    * Write a `DriveToPoseCommand` that will drive robot to a specified pose based on PIDs.
    * Write mechanism that can optionally always point towards some specific `Pose3d`.  Probably a variation of `DriveSubsystem`.
* Test [Elastic Dashboard](https://frc-elastic.gitbook.io/docs) integration.
    * Build larger dashboards.  Test Field widget.  Add camera view widgets.
    * Add notification if RIO can't connect to photonvision.  Or also maybe connection to lights.
* Test [AdvantageScope](https://docs.advantagescope.org/) logging.  Is there anything else we should add for this?
* Test that [PathPlanner](https://pathplanner.dev/) works well with this template.  It should be easy to just add Paths and Autonomous routines and get them to work.  Also, verify that PathPlanner makes good use of the updated odometry from the `VisionSubsystem`
* Support simulation
* Add a checklist in the README for how to apply this template to a new Robot

