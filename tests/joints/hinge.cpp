/*************************************************************************
  *                                                                       *
  * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
  * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
  *                                                                       *
  * This library is free software; you can redistribute it and/or         *
  * modify it under the terms of EITHER:                                  *
  *   (1) The GNU Lesser General Public License as published by the Free  *
  *       Software Foundation; either version 2.1 of the License, or (at  *
  *       your option) any later version. The text of the GNU Lesser      *
  *       General Public License is included with this library in the     *
  *       file LICENSE.TXT.                                               *
  *   (2) The BSD-style license that is included with this library in     *
  *       the file LICENSE-BSD.TXT.                                       *
  *                                                                       *
  * This library is distributed in the hope that it will be useful,       *
  * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
  * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
  *                                                                       *
  *************************************************************************/
//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/hinge.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/joints/hinge.h"

SUITE (TestdxJointHinge)
{
  // The 2 bodies are positionned at (0, 0, 0), with no rotation
  // The joint is an Hinge Joint
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  //         ^Y
  //         |
  //         |
  //         |
  //         |
  //         |
  // Z <---- . (X going out of the page)
  struct dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X {
    dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
    {
      wId = dWorldCreate();

      bId1 = dBodyCreate (wId);
      dBodySetPosition (bId1, 0, 0, 0);

      bId2 = dBodyCreate (wId);
      dBodySetPosition (bId2, 0, 0, 0);

      jId   = dJointCreateHinge (wId, 0);
      joint = (dxJointHinge*) jId;


      dJointAttach (jId, bId1, bId2);
      dJointSetHingeAnchor (jId, 0, 0, 0);

      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 0;
    }

    ~dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId1;
    dBodyID bId2;


    dJointID jId;
    dxJointHinge* joint;

    dVector3 axis;
  };

  // Rotate 2nd body 90deg around X then back to original position
  //
  //   ^  ^       ^
  //   |  |  =>   |  <---
  //   |  |       |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 90deg
  //   ^           ^   ^
  //   | <---  =>  |   |
  //   |           |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_B2_90deg) {
    dMatrix3 R;

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 2nd body -90deg around X then back to original position
  //
  //   ^  ^       ^
  //   |  |  =>   |  --->
  //   |  |       |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 90deg
  //   ^           ^   ^
  //   | --->  =>  |   |
  //   |           |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_B2_Minus90deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 1st body 0.23rad around X then back to original position
  //
  //   ^  ^     ^      ^
  //   |  |  =>  \     |
  //   |  |       \    |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 0.23rad
  // ^    ^        ^   ^
  //  \   | =>     |   |
  //   \  |        |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_B1_0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, REAL(0.23) );
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 1st body -0.23rad around Z then back to original position
  //
  //   ^  ^         ^  ^
  //   |  |  =>    /   |
  //   |  |       /    |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 0.23rad
  //     ^ ^        ^   ^
  //    /  | =>     |   |
  //   /   |        |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_B1_Minus0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // The 2 bodies are positionned at (0, 0, 0), with no rotation
  // The joint is an Hinge Joint.
  // Axis in the inverse direction of the X axis
  // Anchor at (0, 0, 0)
  //         ^Y
  //         |
  //         |
  //         |
  //         |
  //         |
  // Z <---- x (X going out of the page)
  struct dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X {
    dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
    {
      wId = dWorldCreate();

      bId1 = dBodyCreate (wId);
      dBodySetPosition (bId1, 0, -1, 0);

      bId2 = dBodyCreate (wId);
      dBodySetPosition (bId2, 0, 1, 0);

      jId   = dJointCreateHinge (wId, 0);
      joint = (dxJointHinge*) jId;


      dJointAttach (jId, bId1, bId2);
      dJointSetHingeAnchor (jId, 0, 0, 0);

      axis[0] = -1;
      axis[1] = 0;
      axis[2] = 0;
    }

    ~dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId1;
    dBodyID bId2;


    dJointID jId;
    dxJointHinge* joint;

    dVector3 axis;
  };

  // Rotate 2nd body 90deg around X then back to original position
  //
  //   ^  ^       ^
  //   |  |  =>   |  <---
  //   |  |       |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 90deg
  //   ^           ^   ^
  //   | <---  =>  |   |
  //   |           |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_B2_90Deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 2nd body -90deg around X then back to original position
  //
  //   ^  ^       ^
  //   |  |  =>   |  --->
  //   |  |       |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 90deg
  //   ^           ^   ^
  //   | --->  =>  |   |
  //   |           |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_B2_Minus90Deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 1st body 0.23rad around X then back to original position
  //
  //   ^  ^     ^      ^
  //   |  |  =>  \     |
  //   |  |       \    |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 0.23rad
  // ^    ^        ^   ^
  //  \   | =>     |   |
  //   \  |        |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_B1_0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, REAL(0.23));
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Rotate 2nd body -0.23rad around Z then back to original position
  //
  //   ^  ^         ^  ^
  //   |  |  =>    /   |
  //   |  |       /    |
  //  B1  B2     B1   B2
  //
  // Start with a Delta of 0.23rad
  //     ^ ^        ^   ^
  //    /  | =>     |   |
  //   /   |        |   |
  //  B1  B2      B1   B2
  TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_B1_Minus0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
  }


  // Only one body body1 at (0,0,0)
  // The joint is an Hinge Joint.
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  //
  //       ^Y
  //       |
  //       |
  //       |
  //       |
  //       |
  // Z <-- X
  struct dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X {
    dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
    {
      wId = dWorldCreate();

      bId1 = dBodyCreate (wId);
      dBodySetPosition (bId1, 0, 0, 0);

      jId   = dJointCreateHinge (wId, 0);
      joint = (dxJointHinge*) jId;


      dJointAttach (jId, bId1, NULL);
      dJointSetHingeAnchor (jId, 0, 0, 0);

      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 0;
    }

    ~dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId1;


    dJointID jId;
    dxJointHinge* joint;

    dVector3 axis;
  };

  // Rotate B1 by 90deg around X then back to original position
  //
  //   ^
  //   |  => <---
  //   |
  //  B1      B1
  //
  // Start with a Delta of 90deg
  //            ^
  //  <---  =>  |
  //            |
  //   B1      B1
  TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_1Body_B1_90Deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
    CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }

  // Rotate B1 by -0.23rad around X then back to original position
  //
  //   ^         ^
  //   |  =>    /
  //   |       /
  //  B1      B1
  //
  // Start with a Delta of -0.23rad
  //     ^     ^
  //    /  =>  |
  //   /       |
  //   B1     B1
  TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
    CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }



  // Only one body body1 at (0,0,0)
  // The joint is an Hinge Joint.
  // Axis the inverse of the X axis
  // Anchor at (0, 0, 0)
  //
  //       ^Y
  //       |
  //       |
  //       |
  //       |
  //       |
  // Z <-- X
  struct dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X {
    dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
    {
      wId = dWorldCreate();

      bId1 = dBodyCreate (wId);
      dBodySetPosition (bId1, 0, 0, 0);

      jId   = dJointCreateHinge (wId, 0);
      joint = (dxJointHinge*) jId;


      dJointAttach (jId, bId1, NULL);
      dJointSetHingeAnchor (jId, 0, 0, 0);

      axis[0] = -1;
      axis[1] = 0;
      axis[2] = 0;
    }

    ~dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId1;


    dJointID jId;
    dxJointHinge* joint;

    dVector3 axis;
  };

  // Rotate B1 by 90deg around X then back to original position
  //
  //   ^
  //   |  => <---
  //   |
  //  B1      B1
  //
  // Start with a Delta of 90deg
  //            ^
  //  <---  =>  |
  //            |
  //   B1      B1
  TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_1Body_B1_90Deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }

  // Rotate B1 by -0.23rad around X then back to original position
  //
  //   ^         ^
  //   |  =>    /
  //   |       /
  //  B1      B1
  //
  // Start with a Delta of -0.23rad
  //     ^     ^
  //    /  =>  |
  //   /       |
  //   B1     B1
  TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
                test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId1, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }





  // Only one body body2 at (0,0,0)
  // The joint is an Hinge Joint.
  // Axis is along the X axis
  // Anchor at (0, 0, 0)
  //
  //       ^Y
  //       |
  //       |
  //       |
  //       |
  //       |
  // Z <-- X
  struct dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X {
    dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
    {
      wId = dWorldCreate();

      bId2 = dBodyCreate (wId);
      dBodySetPosition (bId2, 0, 0, 0);

      jId   = dJointCreateHinge (wId, 0);
      joint = (dxJointHinge*) jId;


      dJointAttach (jId, NULL, bId2);
      dJointSetHingeAnchor (jId, 0, 0, 0);

      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 0;
    }

    ~dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId2;


    dJointID jId;
    dxJointHinge* joint;

    dVector3 axis;
  };

  // Rotate B2 by 90deg around X then back to original position
  //
  //   ^
  //   |  => <---
  //   |
  //  B2      B2
  //
  // Start with a Delta of 90deg
  //            ^
  //  <---  =>  |
  //            |
  //   B2      B2
  TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_1Body_B2_90Deg) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
    CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }

  // Rotate B2 by -0.23rad around X then back to original position
  //
  //   ^         ^
  //   |  =>    /
  //   |       /
  //  B2      B2
  //
  // Start with a Delta of -0.23rad
  //     ^     ^
  //    /  =>  |
  //   /       |
  //   B2     B2
  TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
                test_dJointSetHingeAxisOffset_1Body_B2_Minus0_23rad) {
    dMatrix3 R;

    dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

    CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
    CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

    dRFromAxisAndAngle (R, 1, 0, 0, 0);
    dBodySetRotation (bId2, R);

    CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
  }


} // End of SUITE TestdxJointHinge


