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
// ode/src/joinst/piston.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/joints/piston.h"

SUITE (TestdxJointPiston)
{
    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::axis =
        {
            1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        // Only here to test a deprecated warning
        dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B1            =>   B1
    //      B2             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //     B2           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of -offset unit
    //
    //     X------->    X---------> Axis -->
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }



    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is the opposite of the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, bId2);


            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;
        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::axis =
        {
            -1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->   <-- Axis
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <-- Axis
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->  <-- Axis
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //     X------->  X--------->      <-- Axis
    //  B1       =>   B1
    //     B2         B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X--------->  <-- Axis
    //  B1          =>  B1
    //  B2                 B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <-- Axis
    //  B1          =>  B1
    //     B2           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X--------->  <-- Axis
    //  B1          =>     B1
    //  B2              B2
    //
    // Start with a Offset of -offset unit
    //
    //     X------->    X--------->     <-- Axis
    //     B1       =>  B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_3Unit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }


    // Only body 1
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, NULL);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::axis =
        {
            1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B1            =>   B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Only body 1
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is in the oppsite X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, bId1, NULL);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::axis =
        {
            -1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 1st body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B1          =>     B1
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B1       =>  B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B1          =>  B1
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B1            =>   B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B1_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId1, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }









    // Only body 2
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, NULL, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::axis =
        {
            1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X::offset = REAL (3.1);

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X---------> Axis -->
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X---------> Axis -->
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Move 2nd body offset unit in the opposite X direction
    //
    //  X------->          X---------> Axis -->
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X---------> Axis -->
    //  B2            =>   B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // Only body 2
    // The body are positionned at (0, 0, 0), with no rotation
    // The joint is a Piston Joint
    // Axis is in the opposite X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X
    {
        Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreatePiston (wId, 0);
            joint = (dxJointPiston*) jId;


            dJointAttach (jId, NULL, bId2);

            dJointSetPistonAxis (jId, axis[0], axis[1], axis[2]);
        }

        ~Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;

        dJointID jId;
        dxJointPiston* joint;

        static const dVector3 axis;

        static const dReal offset;
    };
    const dVector3 Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::axis =
        {
            -1, 0, 0
        };
    const dReal    Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X::offset = REAL (3.1);

    // Move 2nd body offset unit in the X direction
    //
    //  X------->       X--------->  <--- Axis
    //  B2          =>     B2
    //
    // Start with a Offset of offset unit
    //
    //  X------->       X--------->  <--- Axis
    //     B2       =>  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, offset, 0, 0);

        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     offset*axis[0],offset*axis[1],offset*axis[2]);
        CHECK_CLOSE (offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        dJointSetPistonAxisDelta (jId, 1, 0, 0, 0, 0, 0);
    }

    // Move 1st body offset unit in the opposite X direction
    //
    //  X------->          X--------->   <--- Axis
    //  B2          =>  B2
    //
    // Start with a Offset of -offset unit
    //
    //      X------->      X--------->   <--- Axis
    //  B2            =>   B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonAxisOffset_B2_Minus_OffsetUnit)
    {
        dJointSetPistonAnchor (jId, 0, 0, 0);

        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, -offset, 0, 0);

        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dJointSetPistonAnchorOffset (jId, 0, 0, 0,
                                     -offset*axis[0],-offset*axis[1],-offset*axis[2]);
        CHECK_CLOSE (-offset, dJointGetPistonPosition (jId), 1e-4);

        dBodySetPosition (bId2, 0, 0, 0);
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
    }

    // ==========================================================================
    // Test Position Rate
    // ==========================================================================

    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1  F->      =>     B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1  <-F      => B1
    //  B2                  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1  F->      =>     B1
    //  B2              B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1  <-F      => B1
    //  B2                  B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1          =>  B1
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1           =>     B1
    //  B2  <-F          B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1          =>  B1
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1          =>      B1
    //  B2 <-F           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }



    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B1  F->      =>     B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B1  <-F      => B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on 1st body in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B1  F->      =>     B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on 1st body in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B1  <-F      => B1
    TEST_FIXTURE (Fixture_dxJointPiston_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B1)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId1, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on body 2 in the X direction also the Axis direction
    //
    //  X------->       X---------> Axis -->
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Along_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on body 2 in the inverse X direction
    //
    //  X------->           X---------> Axis -->
    //  B2  <-F          B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Along_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }


    // Apply force on body 2 in the X direction also the Axis direction
    //
    //  X------->       X---------> <-- Axis
    //  B2 F->             B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Inverse_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, 1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (1, dJointGetPistonPositionRate (jId), 1e-4);
    }

    // Apply force on body 2 in the inverse X direction
    //
    //  X------->           X---------> <-- Axis
    //  B2 <-F           B2
    TEST_FIXTURE (Fixture_dxJointPiston_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetPistonPositionRate_Force_Along_of_Axis_on_B2)
    {
        CHECK_CLOSE (0.0, dJointGetPistonPosition (jId), 1e-4);
        CHECK_CLOSE (0.0, dJointGetPistonPositionRate (jId), 1e-4);

        dBodyAddForce (bId2, -1.0, 0, 0);
        dWorldQuickStep (wId, 1.0);

        CHECK_CLOSE (-1, dJointGetPistonPositionRate (jId), 1e-4);
    }

} // End of SUITE TestdxJointPiston
