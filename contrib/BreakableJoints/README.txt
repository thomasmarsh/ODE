Breakable Joints by Roel van Dijk

Description:
This is a small addition to ODE that makes joints breakable. Breakable means that if a force
on a joint is to high it wil break. I have included a modified version of test_buggy.cpp so
you can see it for your self. Just drive your buggy in to a wall and enjoy!

Installation instructions:
- copy joint.h, joint.cpp, ode.cpp and step.cpp to the ode/src/ directory
- copy common.h and object.h to the include/ directory
- copy test_buggy.cpp to the ode/test/ directory
- make ode-lib
- make ode-test
enjoy!

Functions:
dJointSetBreakable (dJointID joint, int b)
	If b is 1 the joint is made breakable. If b is 0 the joint is made
	unbreakable.

dJointSetBreakMode (dJointID joint, int m)
	Use this functions to set some flags. These flags can be ORred ( | )
	together; ie. dJointSetBreakMode (someJoint, dJOINT_BREAK_AT_FORCE|dJOINT_DELETE_ON_BREAK)
	dJOINT_BREAK_AT_FORCE  - The joint can break at a certain amount of force.
	dJOINT_BREAK_AT_TORQUE - The joint can break at a certain amount of torque.
	dJOINT_DELETE_ON_BREAK - If the joint breaks it wil be deleted.

dJointSetBreakForce (dJointID joint, dReal force)
	If the force on the joint is greater then the force you set with this functions the joint
	will break.

dJointSetBreakTorque (dJointID joint, dReal torque)
	If the torque on the joint is greater then the torque you set with this functions the joint
	will break.

int dJointIsBreakable (dJointID joint)
	Returns 1 if this joint is breakable, 0 otherwise.

dReal dJointGetBreakForce (dJointID joint)
	Returns the force at what this joint will break.

dReal dJointGetBreakTorque (dJointID joint)
	Returns the torque at what this joint will break.

Problems, known bugs & other issues:
- I'm thinking about including a callback function if a joint breaks. Because if a user
  sets the dJOINT_DELETE_ON_BREAK flag the joint wil be deleted when it breaks. This can
  cause troubles for the user because suddenly their dJointID becomes invalid. With a
  callback the user can take apropriate steps to avoid this.
- If the timestep is very small then joints get a lot weaker. They can even fall apart!
- The force and torque are stored squared. You don't have to supply squared values, the
  dJointSetBreakForce and dJointSetBreakTorque will do this for you. I do this to avoid
  calculating the square root of a force every step.
- The values that are returned by dJointGetBreakForce and dJointGetBreakTorque are
  squared. I have to put a sqrt() function in there.
- I haven't tested the torque breaking yet.
- I have tested all this with the latest checkout from CVS. I haven't tested it with earlier
  versions of ODE.
- I have modified the code that fills the jointfeedback struct. I haven't tested if it still
  works.

Send me an e-mail if you have any suggestions, ideas, bugs, bug-fixes, anything!
e-mail: roelvandijk@home.nl
