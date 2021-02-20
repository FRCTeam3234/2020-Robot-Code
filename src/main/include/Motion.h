#pragma once

#include <frc/Timer.h>
class Motion
{
private:

    float m_k;
    float m_dt;
    float m_sp;
    float m_fp;
    float m_sgangl;
    float m_x4;
    float m_y4;
    bool m_done;
    bool m_isleft;
    frc::Timer m_t;
    
public: 
    Motion(bool isleft);
    Motion(bool isleft, float k);

    float P345(float t);
    //float BCurvature(float x1,float x2, float x3, float x4, float y1, float y2, float y3, float y4, float);
    float FB(float tp, float cp);
    float Drive(float cp, float tp, float tm, float tol, float k);
    float Curve(float cp, float ocp, float gangl, float dt, float tol, float k,float x1,float x2, float x3, float x4, float y1, float y2, float y3, float y4);
    float ConvertInchTick(float wd, float tkperrev);
    float ConvertInchTick(float wd, float tkperrev, float grtio);
    void ResetTime();
    float GetTime();
    float GetK();
    bool GetDone();
};