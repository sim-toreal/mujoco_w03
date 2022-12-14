# MuJoCo Bootcamp - W03 - Pendulum Control
[Here](https://www.youtube.com/watch?v=JNgO_OPVb5k&list=PLc7bpbeTIk758Ad3fkSywdxHWpBh9PM0G&index=8) is the video.

Again, rather than using the provided template, I used the `basic.cc` from the MuJoCo distribution.

This section is about setting up actuators and sensors and then accessing them through the code.

#### Simulator Print Model and Print Data
The simulator (under `mujoco-xxx/bin/simulate`) has a very useful function that allows to print the 
details of the model ([MJMODEL.TXT](MJMODEL.TXT)) and well as data ([MJDATA.TXT](MJDATA.TXT)).

This is very useful for designing and debugging. 

### Setting up Actuators
The actuators are defined in the XML file in the `<actuator>` block ([see the docs for the available types](https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator)):

	<actuator>
		<motor joint="pendulum_mount" name="torque" gear="1" ctrllimited="true" ctrlrange="-5 5" />
		<position joint="pendulum_mount" name="position_servo" kp="0" />
		<velocity joint="pendulum_mount" name="velocity_servo" kv="0" />
	</actuator>

The `position` (btw this actuator behaves like a spring - NOT a position servo) is controlled by `Kp`.
The `velocity` sets angular (does it set forward velocity for sliding joints?) velocity and the 'strength' is controlled through `Kv`

### Controlling the Actuators
Actuators are controlled by setting values to `d->ctrl[...]` where each index corresponds with one actuator (e.g. in the example above torque is 0, position is 1 and velocity is 2).
This can be done directly before calling `mj_step(m, d);` but a better way is to define a "controller" method and assign it to `mjcb_control`.
This method will be called at each `mj_step` but very late in the process, once all the intermediate results dependent on the state but not on the control have been computed.

I.e. setting the `d-ctrl` before calling mj_step and not from the callback means that many of the state observations will not have been calculated yet.

Here's an example of the controller method (here it's also accessing sensor data):

    void pendulumController(const mjModel* m, mjData* d) {
        // the first actuator is torque so this is setting PD control
        // 1) this is using qpos[0] i.e. the ground truth for position data
        // d->ctrl[0] = -10 * (d->qpos[0] - 0) - 1 * d->qvel[0];

        // 2) this is using sensor readings with noise (sensor[0] is position sensor and sensor[1] is velocity sensor)
        d->ctrl[0] = -10 * (d->sensordata[0] - 0) - 1 * d->sensordata[1];
    }

Don't forget to assign it before the loop calling `mj_step` (and before the glfw loop):

    mjcb_control = pendulumController;


### Updating Actuator Settings in the Code
Based on the [docs](https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator) the actuators have multiple settings, but the important ones here are `dyntype, gaintype and biastype`:

<table class="docutils align-default"><colgroup><col style="width: 28%"><col style="width: 22%"><col style="width: 28%"><col style="width: 22%"></colgroup><thead><tr class="row-odd"><th class="head"><p>Attribute</p></th><th class="head"><p>Setting</p></th><th class="head"><p>Attribute</p></th><th class="head"><p>Setting</p></th></tr></thead><tbody><tr class="row-even"><td><p>dyntype</p></td><td><p>none</p></td><td><p>dynprm</p></td><td><p>1 0 0</p></td></tr><tr class="row-odd"><td><p>gaintype</p></td><td><p>fixed</p></td><td><p>gainprm</p></td><td><p>kp 0 0</p></td></tr><tr class="row-even"><td><p>biastype</p></td><td><p>affine</p></td><td><p>biasprm</p></td><td><p>0 -kp 0</p></td></tr></tbody></table>

E.g. for `position`, the Kp is set in the gaintype and biastype. You can see this settings in the [MJMODEL.TXT](MJMODEL.TXT) printed from the `simulation` (Print Model button) under `ACTUATOR` 1 and 2.
You can see that each has 10 values, but only 2 are set: first in `actuator_gainprm` and 2nd in `actuator_biasprm`.
`velocity` has Kv set as the first value in `actuator_gainprm` and 3rd in `actuator_biasprm`.

The actuators can be turned on and off by setting these values using:

    // position actuator
    actuator_no = 1;
    m->actuator_gainprm[10*actuator_no + 0] = 100;
    m->actuator_biasprm[10*actuator_no + 1] = -100;

    // velocity actuator
    actuator_no = 2;
    m->actuator_gainprm[10*actuator_no + 0] = 100;
    m->actuator_biasprm[10*actuator_no + 2] = -100;

See this in the `void pendulumController(const mjModel* m, mjData* d)` method.

For the torque actuator the first value in `actuator_gainprm` needs to be set to `1` (again see the docs).

### Simulating Positional Servo - a normal hobby grade servo
This can be done by combining both `position` and `velocity` using a PD control:

    m->actuator_gainprm[10] = 100;
    m->actuator_biasprm[11] = -100;
    m->actuator_gainprm[20] = 10;
    m->actuator_biasprm[22] = -10;
    d->ctrl[1] = 0.5;
    d->ctrl[2] = 0;

This will stop the pendulum at position 0.5 (looks like 0.5 rad angle).

Seems that a good ratio between them is 10:1 and higher values seem to be causing higher speed and less oscillation.

### Setting up Sensors
Sensors are defined in the XML file in the `<sensor>` block ([see the docs](https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor) for available sensors and settings).

    <sensor>
		<jointpos name="position_sensor" joint="pendulum_mount" noise="0.2" />
		<jointvel name="velocity_sensor" joint="pendulum_mount" noise="0.2" />
	</sensor>

In the code, they can be accessed through `d->sensordata[...]`, again each index corresponding to one sensor (i.e. 0 would be position and 1 velocity).
They can be accessed in the controller to calculate the actuation values, e.g.:

    d->ctrl[0] = -10 * (d->sensordata[0] - 0) - 1 * d->sensordata[1];