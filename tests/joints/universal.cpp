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
// ode/src/joinst/universal.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/joints/universal.h"

SUITE (TestdxJointUniversal)
{
    // The 2 bodies are positionned at (-1, -2, -3),  and (11, 22, 33)
    // The bodis have rotation of 27deg around some axis.
    // The joint is a Universal Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    struct Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis_Along_X
    {
        Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, -1, -2, -3);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 11, 22, 33);

            dMatrix3 R;

            dVector3 axis; // Random axis

            axis[0] = 0.53;
            axis[1] = -0.71;
            axis[2] = 0.43;
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                REAL(0.47123)); // 27deg
            dBodySetRotation (bId1, R);


            axis[0] = 1.2;
            axis[1] = 0.87;
            axis[2] = -0.33;
            dNormalize3(axis);
            dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                REAL(0.47123)); // 27deg
            dBodySetRotation (bId2, R);

            jId   = dJointCreateUniversal (wId, 0);
            joint = (dxJointUniversal*) jId;


            dJointAttach (jId, bId1, bId2);
        }

        ~Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointUniversal* joint;
    };

    // Test is dJointSetUniversalAxis and dJointGetUniversalAxis return same value
    TEST_FIXTURE (Fixture_dxJointUniversal_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetGetUniversalAxis)
    {
        dVector3 axisOrig, axis;


        dJointGetUniversalAxis1 (jId, axisOrig);
        dJointGetUniversalAxis1 (jId, axis);
        dJointSetUniversalAxis1 (jId, axis[0], axis[1], axis[2]);
        dJointGetUniversalAxis1 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dJointGetUniversalAxis2 (jId, axisOrig);
        dJointGetUniversalAxis2(jId, axis);
        dJointSetUniversalAxis2 (jId, axis[0], axis[1], axis[2]);
        dJointGetUniversalAxis2 (jId, axis);
        CHECK_CLOSE (axis[0], axisOrig[0] , 1e-4);
        CHECK_CLOSE (axis[1], axisOrig[1] , 1e-4);
        CHECK_CLOSE (axis[2], axisOrig[2] , 1e-4);


        dVector3 anchor1, anchor2, anchorOrig1, anchorOrig2;
        dJointGetUniversalAnchor (jId, anchorOrig1);
        dJointGetUniversalAnchor (jId, anchor1);
        dJointGetUniversalAnchor2 (jId, anchorOrig2);
        dJointGetUniversalAnchor2 (jId, anchor2);

        dJointSetUniversalAnchor (jId, anchor1[0], anchor1[1], anchor1[2]);
        dJointGetUniversalAnchor (jId, anchor1);
        dJointGetUniversalAnchor2 (jId, anchor2);
        CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
        CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);
        CHECK_CLOSE (anchor1[0], anchorOrig1[0] , 1e-4);

        CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
        CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
        CHECK_CLOSE (anchor2[0], anchorOrig2[0] , 1e-4);
    }



  // Create 2 bodies attached by a Universal joint
  // Axis is along the X axis (Default value
  // Anchor at (0, 0, 0)      (Default value)
  //
  //       ^Y
  //       |
  //       * Body2
  //       |
  //       |
  // Body1 |
  // *     Z-------->
  struct dxJointUniversal_Test_Initialization {
    dxJointUniversal_Test_Initialization()
    {
      wId = dWorldCreate();

      // Remove gravity to have the only force be the force of the joint
      dWorldSetGravity(wId, 0,0,0);

      for (int j=0; j<2; ++j) {
        bId[j][0] = dBodyCreate (wId);
        dBodySetPosition (bId[j][0], -1, -2, -3);

        bId[j][1] = dBodyCreate (wId);
        dBodySetPosition (bId[j][1], 11, 22, 33);


        dMatrix3 R;
        dVector3 axis; // Random axis

        axis[0] = 0.53;
        axis[1] = -0.71;
        axis[2] = 0.43;
        dNormalize3(axis);
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                            REAL(0.47123)); // 27deg
        dBodySetRotation (bId[j][0], R);


        axis[0] = 1.2;
        axis[1] = 0.87;
        axis[2] = -0.33;
        dNormalize3(axis);
        dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                            REAL(0.47123)); // 27deg
        dBodySetRotation (bId[j][1], R);

        jId[j]   = dJointCreateUniversal (wId, 0);
        dJointAttach (jId[j], bId[j][0], bId[j][1]);
        dJointSetUniversalParam(jId[j], dParamLoStop, 1);
        dJointSetUniversalParam(jId[j], dParamHiStop, 2);
        dJointSetUniversalParam(jId[j], dParamFMax, 200);
      }
    }

    ~dxJointUniversal_Test_Initialization()
    {
      dWorldDestroy (wId);
    }

    dWorldID wId;

    dBodyID bId[2][2];


    dJointID jId[2];

  };


  // Test if setting a Universal with its default values
  // will behave the same as a default Universal joint
  TEST_FIXTURE (dxJointUniversal_Test_Initialization,
                test_Universal_Initialization) {
    using namespace std;

    dVector3 axis;
    dJointGetUniversalAxis1(jId[1], axis);
    dJointSetUniversalAxis1(jId[1], axis[0], axis[1], axis[2]);

    dJointGetUniversalAxis2(jId[1], axis);
    dJointSetUniversalAxis2(jId[1], axis[0], axis[1], axis[2]);

    dVector3 anchor;
    dJointGetUniversalAnchor(jId[1], anchor);
    dJointSetUniversalAnchor(jId[1], anchor[0], anchor[1], anchor[2]);


    for (int b=0; b<2; ++b) {
      // Compare body b of the first joint with its equivalent on the
      // second joint
      const dReal *qA = dBodyGetQuaternion(bId[0][b]);
      const dReal *qB = dBodyGetQuaternion(bId[1][b]);
      CHECK_CLOSE (qA[0], qB[0], 1e-4);
      CHECK_CLOSE (qA[1], qB[1], 1e-4);
      CHECK_CLOSE (qA[2], qB[2], 1e-4);
      CHECK_CLOSE (qA[3], qB[3], 1e-4);
    }

    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);
    dWorldStep (wId,0.5);

    for (int b=0; b<2; ++b) {
      // Compare body b of the first joint with its equivalent on the
      // second joint
      const dReal *qA = dBodyGetQuaternion(bId[0][b]);
      const dReal *qB = dBodyGetQuaternion(bId[1][b]);
      CHECK_CLOSE (qA[0], qB[0], 1e-4);
      CHECK_CLOSE (qA[1], qB[1], 1e-4);
      CHECK_CLOSE (qA[2], qB[2], 1e-4);
      CHECK_CLOSE (qA[3], qB[3], 1e-4);


      const dReal *posA = dBodyGetPosition(bId[0][b]);
      const dReal *posB = dBodyGetPosition(bId[1][b]);
      CHECK_CLOSE (posA[0], posB[0], 1e-4);
      CHECK_CLOSE (posA[1], posB[1], 1e-4);
      CHECK_CLOSE (posA[2], posB[2], 1e-4);
      CHECK_CLOSE (posA[3], posB[3], 1e-4);
    }
  }

} // End of SUITE TestdxJointUniversal

