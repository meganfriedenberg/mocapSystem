#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "quaternion.h"
#include "types.h"
#include "transform.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // need to first create the rotation matrices from the given euler angles
    double xRotation[4][4];
    double yRotation[4][4];
    double zRotation[4][4];

    rotationX(xRotation, angles[0]);
    rotationY(yRotation, angles[1]);
    rotationZ(zRotation, angles[2]);

    // these are 3 separate 4x4 matrices, so now they need to be combined into R
    
    // ordering is z axis * y axis * x axis
    double zTimesY[4][4];
    matrix_mult(zRotation, yRotation, zTimesY);
    double zyTimesX[4][4];
    matrix_mult(zTimesY, xRotation, zyTimesX);

    R[0] = zyTimesX[0][0];
    R[1] = zyTimesX[0][1];
    R[2] = zyTimesX[0][2];

    R[3] = zyTimesX[1][0];
    R[4] = zyTimesX[1][1];
    R[5] = zyTimesX[1][2];

    R[6] = zyTimesX[2][0];
    R[7] = zyTimesX[2][1];
    R[8] = zyTimesX[2][2];
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int totalFrames = pInputMotion->GetNumFrames();
    int keyFrameStart = 0;

    // need to walk thru each frame, it is N+1 because of the example given in the assignment description
    do {
        Posture* postureStart = pInputMotion->GetPosture(keyFrameStart);
        int keyFrameEnd = keyFrameStart + N + 1;
        Posture* postureEnd = pInputMotion->GetPosture(keyFrameEnd);

        // copy over initial and ending frames to output motion (since it doesnt change)
        pOutputMotion->SetPosture(keyFrameStart, *postureStart);
        pOutputMotion->SetPosture(keyFrameEnd, *postureEnd);

        // interpolate postures in between till the next N
        for (int currFrame = 1; currFrame <= N; currFrame++)
        {
            Posture interpPosture;
            double t = 1.0 * currFrame / (N + 1);

            // interpolate root position
            interpPosture.root_pos = postureStart->root_pos * (1 - t) + postureEnd->root_pos * t;

            // interpolate bones
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                vector q1;
                vector q2;
                vector q3;
                vector qNm1;
                vector qNp1;
                vector aN;
                vector bN;

                q1 = postureStart->bone_rotation[bone];
                q2 = postureEnd->bone_rotation[bone];

                // aN formula: slerp(qN,aBar,1/3)
                // aBar=Slerp(Slerp(qN-1,qN,2),qN+1,0.5), but a1's formula is a1=slerp(q1,slerp(q3,q2,2),1/3)
                // bN formula: slerp(qN,aBar,-1/3)
                // aN's formula requires qN-1, qN, qN+1 - this is an issue if we are the first keyframe or the last

                if (keyFrameStart == 0) // is this the first frame
                {
                    // there's no N-1, so use N+1, so then N-1 is just N, but get the next posture since there isnt a current one
                    q3 = (pInputMotion->GetPosture(keyFrameEnd + N + 1))->bone_rotation[bone];
                    aN = Lerp(q1,Lerp(q3, q2, 2.0), (1.0/3.0)); // calculates a1 (spline 1)

                    // now set B, use spline 2 formula
                    bN = Lerp(q2, (Lerp(Lerp(q1, q2, 2.0), (q3), 0.5)), (-1.0 / 3.0)); // qN is the Nth pose
                }
                else if ((keyFrameEnd + N + 1) > totalFrames) // is this the last frame
                {
                    // if so, then there is a previous posture, because must be more than one frame long
                    qNm1 = (pInputMotion->GetPosture(keyFrameStart - (N + 1)))->bone_rotation[bone];
                    aN = Lerp(q1, Lerp(Lerp(qNm1, q1, 2.0), q2, 0.5), (1.0 / 3.0)); // since qN-1 exists, qN+1 doesn't, so use the last posture

                    // but this is the LAST frame, so no aN+1, but we can use the previous frame and the first frame for qN-2 and qN-1
                    bN = Lerp(q2, Lerp(qNm1, q1, 2.0), (1.0 / 3.0));

                }
                else // not the first frame and not the last frame, "normal" case, both qN-1 and qN+1 exist
                {
                    qNm1 = (pInputMotion->GetPosture(keyFrameStart - (N + 1)))->bone_rotation[bone]; // get the previous pose
                    aN = Lerp(q1, Lerp(Lerp(qNm1, q1, 2.0), q2, 0.5), (1.0 / 3.0));

                    // then there is a next posture to use for b
                    qNp1 = (pInputMotion->GetPosture(keyFrameEnd + N + 1))->bone_rotation[bone]; // get the next pose
                    bN = Lerp(q2, Lerp(Lerp(q1, q2, 2.0), (qNp1), 0.5), (-1.0 / 3.0));

                }


                interpPosture.bone_rotation[bone] = DeCasteljauEuler(t, q1, aN, bN, q2); // from lecture, use to evaluate the spline
            }

            pOutputMotion->SetPosture(keyFrameStart + currFrame, interpPosture);

        }

        keyFrameStart = keyFrameEnd;

    } while ((keyFrameStart + N + 1) < totalFrames);

    for (int frame = keyFrameStart + 1; frame < totalFrames; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));


}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int totalFrames = pInputMotion->GetNumFrames();
    int keyFrameStart = 0;

    // need to walk thru each frame, it is N+1 because of the example given in the assignment description
    do {
        Posture* postureStart = pInputMotion->GetPosture(keyFrameStart);
        int keyFrameEnd = keyFrameStart + N + 1;
        Posture* postureEnd = pInputMotion->GetPosture(keyFrameEnd);

        // copy over initial and ending frames to output motion (since it doesnt change)
        pOutputMotion->SetPosture(keyFrameStart, *postureStart);
        pOutputMotion->SetPosture(keyFrameEnd, *postureEnd);

        // interpolate postures in between till the next N
        for (int currFrame = 1; currFrame <= N; currFrame++)
        {
            Posture interpPosture;
            double t = 1.0 * currFrame / (N + 1);

            // interpolate root position
            interpPosture.root_pos = postureStart->root_pos * (1 - t) + postureEnd->root_pos * t;

            // interpolate bones
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> quatStart;
                Quaternion<double> quatEnd;
                
                Euler2Quaternion(postureStart->bone_rotation[bone].p, quatStart);
                Euler2Quaternion(postureEnd->bone_rotation[bone].p, quatEnd);
                Quaternion<double> boneSlerped = Slerp(t, quatStart, quatEnd);
                if (!isnan(boneSlerped.Gets()))
                {
                    Quaternion2Euler(boneSlerped, interpPosture.bone_rotation[bone].p);
                }
                else // if it is nan, linearly interpolate
                {
                    interpPosture.bone_rotation[bone] = postureStart->bone_rotation[bone] * (1 - t) + postureEnd->bone_rotation[bone] * t;
                }

            }

            pOutputMotion->SetPosture(keyFrameStart + currFrame, interpPosture);

        }

        keyFrameStart = keyFrameEnd;

    } while ((keyFrameStart + N + 1) < totalFrames);

    for (int frame = keyFrameStart + 1; frame < totalFrames; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{

    int totalFrames = pInputMotion->GetNumFrames();
    int keyFrameStart = 0;

    // need to walk thru each frame, it is N+1 because of the example given in the assignment description
    do {
        Posture* postureStart = pInputMotion->GetPosture(keyFrameStart);
        int keyFrameEnd = keyFrameStart + N + 1;
        Posture* postureEnd = pInputMotion->GetPosture(keyFrameEnd);

        // copy over initial and ending frames to output motion (since it doesnt change)
        pOutputMotion->SetPosture(keyFrameStart, *postureStart);
        pOutputMotion->SetPosture(keyFrameEnd, *postureEnd);

        // interpolate postures in between till the next N
        for (int currFrame = 1; currFrame <= N; currFrame++)
        {
            Posture interpPosture;
            double t = 1.0 * currFrame / (N + 1);

            // interpolate root position
            interpPosture.root_pos = postureStart->root_pos * (1 - t) + postureEnd->root_pos * t;

            // interpolate bones
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> q1;
                Quaternion<double> q2;
                Quaternion<double> q3;
                Quaternion<double> qNm1;
                Quaternion<double> qNp1;
                Quaternion<double> aN;
                Quaternion<double> bN;

                Euler2Quaternion(postureStart->bone_rotation[bone].p, q1);
                Euler2Quaternion(postureEnd->bone_rotation[bone].p, q2);

                // aN formula: slerp(qN,aBar,1/3)
                // aBar=Slerp(Slerp(qN-1,qN,2),qN+1,0.5), but a1's formula is a1=slerp(q1,slerp(q3,q2,2),1/3)
                // bN formula: slerp(qN,aBar,-1/3)
                // aN's formula requires qN-1, qN, qN+1 - this is an issue if we are the first keyframe or the last

                if (keyFrameStart == 0) // is this the first frame
                {
                    // there's no N-1, so use N+1, so then N-1 is just N, but get the next posture since there isnt a current one
                    Euler2Quaternion((pInputMotion->GetPosture(keyFrameEnd + N + 1))->bone_rotation[bone].p, q3);
                    aN = Slerp((1.0 / 3.0), q1, Double(q3, q2)); // calculates a1 (spline 1)

                    // now set B, use spline 2 formula
                    bN = Slerp((-1.0 / 3.0), q2, (Slerp(0.5, Double(q1, q2), q3))); // qN is the Nth pose
                }
                else if ((keyFrameEnd + N + 1) > totalFrames) // is this the last frame
                {
                    // if so, then there is a previous posture, because must be more than one frame long
                    Euler2Quaternion((pInputMotion->GetPosture(keyFrameStart - (N + 1)))->bone_rotation[bone].p, qNm1);
                    aN = Slerp((1.0 / 3.0), q1, Slerp(0.5, Double(qNm1, q1), q2)); // since qN-1 exists, qN+1 doesn't, so use the last posture

                    // but this is the LAST frame, so no aN+1, but we can use the previous frame and the first frame for qN-2 and qN-1
                    bN = Slerp((1.0 / 3.0), q2, Double(qNm1, q1));

                }
                else // not the first frame and not the last frame, "normal" case, both qN-1 and qN+1 exist
                {
                    Euler2Quaternion((pInputMotion->GetPosture(keyFrameStart - (N + 1)))->bone_rotation[bone].p, qNm1); // get the previous pose
                    aN = Slerp((1.0 / 3.0), q1, Slerp(0.5, Double(qNm1, q1), q2));

                    // then there is a next posture to use for b
                    Euler2Quaternion((pInputMotion->GetPosture(keyFrameEnd + N + 1))->bone_rotation[bone].p, qNp1); // get the next pose
                    bN = Slerp((-1.0 / 3.0), q2, Slerp(0.5, Double(q1, q2), (qNp1)));

                }


                Quaternion2Euler(DeCasteljauQuaternion(t, q1, aN, bN, q2), interpPosture.bone_rotation[bone].p);  // evaluate the spline
                if (isnan(DeCasteljauQuaternion(t, q1, aN, bN, q2).Gets()))
                { // do linear interpolation if its nan
                    interpPosture.bone_rotation[bone] = postureStart->bone_rotation[bone] * (1 - t) + postureEnd->bone_rotation[bone] * t;
                } // without this extra check, does not work
            }

            pOutputMotion->SetPosture(keyFrameStart + currFrame, interpPosture);

        }

        keyFrameStart = keyFrameEnd;

    } while ((keyFrameStart + N + 1) < totalFrames);

    for (int frame = keyFrameStart + 1; frame < totalFrames; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double rotationFromEuler[9];
    Euler2Rotation(angles, rotationFromEuler);

    q = Quaternion<double>::Matrix2Quaternion(rotationFromEuler);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    double quatToRotation[9];
    q.Quaternion2Matrix(quatToRotation);

    Rotation2Euler(quatToRotation, angles);
 
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
    Quaternion<double> result;

    qStart.Normalize();
    qEnd_.Normalize(); 

    // cos(theta) is the dot product of q1q2
    double q1Dotq2 = (qStart.Gets() * qEnd_.Gets()) + (qStart.Getx() * qEnd_.Getx()) +
        (qStart.Gety() * qEnd_.Gety()) + (qStart.Getz() * qEnd_.Getz());
    
    if (q1Dotq2 < 0) // must choose shortest rotation, always 2 rotations to choose from
    {
        q1Dotq2 *= -1;
        qStart = qStart * -1;
    }

    double theta = acos(q1Dotq2);

    result = (sin((1 - t) * theta) / sin(theta)) * qStart 
        + (sin(t * theta) / sin(theta)) * qEnd_;

    // do not need to normalize result since it already produces a unit quaternion
    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    Quaternion<double> result;
    result = Slerp(2.0, p, q);

    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3) // how do you get t??
{
    vector q0;
    vector r0;
    vector q1;
    vector r1;
    vector q2;
    vector pOfT; // P(t)=Lerp(r0,r1,t)

    // q0 is achieved by lerp/slerp from p0 to p1 by t
    q0 = Lerp(p0, p1, t);
    q1 = Lerp(p1, p2, t);
    r0 = Lerp(q0, q1, t);
    q2 = Lerp(p2, p3, t);
    r1 = Lerp(q1, q2, t);

    pOfT = Lerp(r0, r1, t);

    return pOfT;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    Quaternion<double> q0;
    Quaternion<double> q1;
    Quaternion<double> r0;
    Quaternion<double> q2;
    Quaternion<double> r1;
    Quaternion<double> pOfT; // P(t)=Slerp(r0,r1,t)

     // q0 is achieved by lerp/slerp from p0 to p1 by t, used slides diagram for others
    q0 = Slerp(t, p0, p1);
    q1 = Slerp(t, p1, p2);
    r0 = Slerp(t, q0, q1);
    q2 = Slerp(t, p2, p3);
    r1 = Slerp(t, q1, q2);

    pOfT = Slerp(t, r0, r1);

    return pOfT;
}

vector Interpolator::Lerp(vector v1, vector v2, double t) // Returns a vector lerped by t between v1 and v2
{
    vector result;
    result = (v2 - v1) * t;
    result = result + v1; 
    return result;
}